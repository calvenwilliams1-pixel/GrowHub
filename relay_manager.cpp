/*
   relay_manager.cpp
   GrowHub32 - Relay Control & Safety Guardrails Implementation
   Version: 1.4.0
   Revision: Configurable relay mapping (v1.4). Dynamic pin assignment.
             GPIO 12 moved to blacklist, Air Assist default → GPIO 26.
             Mapping persisted via RuntimeCache on SD.
             Hardware teardown/reinit on mapping change under mutex.
             Startup order: load mapping before relayManager_init().
             Pre-latch digitalWrite before pinMode to prevent pulse glitch.
             All Serial output moved outside critical sections.
             Mapping validation on init (pin sanity check).
             Cooldown state preserved across remap (compressor protection).
             force=true bypasses all safeties including rapid-fire lockout.
             Emergency shutdown cuts power before any Serial I/O.

   RELAY LOGIC: Active LOW
   - digitalWrite(pin, LOW)  = Relay ON, circuit CLOSED
   - digitalWrite(pin, HIGH) = Relay OFF, circuit OPEN

   WIRING NOTE:
   - All relay VCC to external 5V supply (NOT ESP32 3.3V or 5V pin).
   - JD-VCC jumper: use separate supply configuration for optocoupler isolation.

   THREADING: All functions designed for main loop task only. Not ISR-safe.
*/

#include "relay_manager.h"
#include "rtc_handler.h"
#include "system_state.h"

// External mutex for g_systemState cross-task protection
extern portMUX_TYPE g_stateMux;

RelayState g_relays[RELAY_COUNT];

// Dynamic pin array — populated from mapping at init
static uint8_t g_relayPins[RELAY_COUNT];

// Friendly names for serial logging
static const char* relayNames[RELAY_COUNT] = {
  "HOH Humidifier",
  "Air Assist Valve",
  "Exhaust Fan",
  "Air Compressor"
};

// Relay capability table — source of truth for per-relay behavior.
// Capabilities are tied to function index, not GPIO pin.
// All relay modules are physically identical.
static const RelayCapability g_relayCaps[RELAY_COUNT] = {
    { false, false, false },  // HOH — silent, no cooldown, continuous
    { true,  false, true  },  // Air Assist — loud, no cooldown, burst cycled
    { false, false, false },  // Exhaust — moderate, no cooldown, continuous
    { true,  false, false }   // Compressor — loud, no cooldown, self-regulated
};

// Current relay mapping (file-scope — access via getter/setter)
static RelayMapping g_relayMapping;

// Default factory mapping
static const RelayMapping g_defaultMapping = {
  .magic = RELAY_MAPPING_MAGIC,
  .pinHOH = DEFAULT_PIN_HOH,
  .pinAirAssist = DEFAULT_PIN_AIR_ASSIST,
  .pinExhaust = DEFAULT_PIN_EXHAUST,
  .pinCompressor = DEFAULT_PIN_COMPRESSOR,
  .reserved = {0, 0, 0}
};

// ============================================
// Pin Validation
// ============================================

static bool isPinBlacklisted(uint8_t pin) {
  switch (pin) {
    case 0: case 1: case 2: case 3:   // Boot + UART (TX/RX)
    case 5:                           // SD CS
    case 12: case 15:                 // Bootstrap (MTDI, MTDO)
    case 18: case 19: case 23:        // SD SPI
    case 21: case 22:                 // I2C
      return true;
    default: return false;
  }
}

bool relayManager_isPinValid(uint8_t pin) {
  if (isPinBlacklisted(pin)) return false;
  if (pin > 39) return false;
  // GPIOs 6-11 are connected to internal flash on most ESP32 modules
  if (pin >= 6 && pin <= 11) return false;
  // GPIOs 34-39 are input-only
  if (pin >= 34) return false;
  return true;
}

// ============================================
// Mapping Accessors
// ============================================

const RelayMapping* relayManager_getMapping() {
  return &g_relayMapping;
}

const RelayMapping* relayManager_getDefaultMapping() {
  return &g_defaultMapping;
}

uint8_t relayManager_getPin(uint8_t relayIndex) {
  if (relayIndex >= RELAY_COUNT) return 255;
  return g_relayPins[relayIndex];
}

bool relayManager_updateMapping(const RelayMapping* newMapping) {
  if (!newMapping) return false;
  if (newMapping->magic != RELAY_MAPPING_MAGIC) return false;

  uint8_t pins[RELAY_FUNCTION_COUNT] = {
    newMapping->pinHOH,
    newMapping->pinAirAssist,
    newMapping->pinExhaust,
    newMapping->pinCompressor
  };

  // Validate all pins
  for (int i = 0; i < RELAY_FUNCTION_COUNT; i++) {
    if (!relayManager_isPinValid(pins[i])) {
      Serial.print(F("[RELAY] Invalid pin in mapping: "));
      Serial.println(pins[i]);
      return false;
    }
    if (pins[i] == 255) {
      Serial.println(F("[RELAY] Pin 255 is reserved — rejected"));
      return false;
    }
  }

  // Check for duplicate pins
  for (int i = 0; i < RELAY_FUNCTION_COUNT; i++) {
    for (int j = i + 1; j < RELAY_FUNCTION_COUNT; j++) {
      if (pins[i] == pins[j]) {
        Serial.print(F("[RELAY] Duplicate pin in mapping: "));
        Serial.println(pins[i]);
        return false;
      }
    }
  }

  // Capture timestamp and cooldown state BEFORE critical section
  unsigned long now = millis();
  bool preserveCooldown[RELAY_COUNT];
  unsigned long preserveCooldownStart[RELAY_COUNT];
  unsigned long preserveCooldownOffEpoch[RELAY_COUNT];
  for (uint8_t i = 0; i < RELAY_COUNT; i++) {
    preserveCooldown[i] = g_relays[i].cooldownLocked;
    preserveCooldownStart[i] = g_relays[i].cooldownStart;
    preserveCooldownOffEpoch[i] = g_relays[i].cooldownOffEpoch;
  }

  // --- Hardware transition under mutex ---
  // CRITICAL: No Serial, no SD, no blocking calls inside this block.
  portENTER_CRITICAL(&g_stateMux);

  // 1. Force all old pins OFF and tristate
  for (uint8_t i = 0; i < RELAY_COUNT; i++) {
    if (g_relays[i].pin != 255) {
      digitalWrite(g_relays[i].pin, HIGH);   // Active LOW = OFF
      pinMode(g_relays[i].pin, INPUT);        // Tristate
    }
  }

  // 2. Update mapping and dynamic array
  memcpy(&g_relayMapping, newMapping, sizeof(RelayMapping));
  g_relayPins[RELAY_HOH] = g_relayMapping.pinHOH;
  g_relayPins[RELAY_AIR_ASSIST] = g_relayMapping.pinAirAssist;
  g_relayPins[RELAY_EXHAUST] = g_relayMapping.pinExhaust;
  g_relayPins[RELAY_COMPRESSOR] = g_relayMapping.pinCompressor;

  // 3. Update RelayState structs and configure new pins
  // Cooldown state is PRESERVED — it belongs to the compressor function,
  // not the GPIO pin. Wiping it would allow short-cycling after remap.
  for (uint8_t i = 0; i < RELAY_COUNT; i++) {
    g_relays[i].pin = g_relayPins[i];
    g_relays[i].isActive = false;
    g_relays[i].lastOnTime = 0;
    g_relays[i].lastOffTime = now;
    g_relays[i].totalOnDuration = 0;
    g_relays[i].cycleCount = 0;
    g_relays[i].cycleWindowStart = now;
    g_relays[i].rapidFireLockoutStart = 0;

    // Restore cooldown state — compressor protection survives remap
    g_relays[i].cooldownLocked = preserveCooldown[i];
    g_relays[i].cooldownStart = preserveCooldownStart[i];
    g_relays[i].cooldownOffEpoch = preserveCooldownOffEpoch[i];

    // Pre-latch HIGH before OUTPUT to prevent nanosecond pulse glitch
    digitalWrite(g_relays[i].pin, HIGH);
    pinMode(g_relays[i].pin, OUTPUT);
  }

  portEXIT_CRITICAL(&g_stateMux);

  // Safe logging outside critical section
  for (uint8_t i = 0; i < RELAY_COUNT; i++) {
    Serial.print(F("[RELAY] Remapped "));
    Serial.print(relayNames[i]);
    Serial.print(F(" -> GPIO "));
    Serial.print(g_relays[i].pin);
    Serial.println(F(" (OFF)"));
  }

  Serial.println(F("[RELAY] Mapping updated and hardware synchronized"));
  return true;
}

void relayManager_resetMapping() {
  relayManager_updateMapping(&g_defaultMapping);
}

// ============================================
// Initialization
// ============================================

bool relayManager_init(const RelayMapping* useMapping) {
  // Apply mapping: use provided mapping if valid, otherwise factory defaults
  if (useMapping && useMapping->magic == RELAY_MAPPING_MAGIC &&
      relayManager_isPinValid(useMapping->pinHOH) &&
      relayManager_isPinValid(useMapping->pinAirAssist) &&
      relayManager_isPinValid(useMapping->pinExhaust) &&
      relayManager_isPinValid(useMapping->pinCompressor) &&
      useMapping->pinHOH != useMapping->pinAirAssist &&
      useMapping->pinHOH != useMapping->pinExhaust &&
      useMapping->pinHOH != useMapping->pinCompressor &&
      useMapping->pinAirAssist != useMapping->pinExhaust &&
      useMapping->pinAirAssist != useMapping->pinCompressor &&
      useMapping->pinExhaust != useMapping->pinCompressor) {
    memcpy(&g_relayMapping, useMapping, sizeof(RelayMapping));
    Serial.println(F("[RELAY] Using cached relay mapping"));
  } else {
    memcpy(&g_relayMapping, &g_defaultMapping, sizeof(RelayMapping));
    Serial.println(F("[RELAY] Using factory default relay mapping"));
  }

  g_relayPins[RELAY_HOH] = g_relayMapping.pinHOH;
  g_relayPins[RELAY_AIR_ASSIST] = g_relayMapping.pinAirAssist;
  g_relayPins[RELAY_EXHAUST] = g_relayMapping.pinExhaust;
  g_relayPins[RELAY_COMPRESSOR] = g_relayMapping.pinCompressor;

  // GH-SYS-003: CRITICAL - Set ALL relays HIGH (OFF) immediately
  unsigned long now = millis();
  for (uint8_t i = 0; i < RELAY_COUNT; i++) {
    g_relays[i].pin = g_relayPins[i];
    g_relays[i].isActive = false;
    g_relays[i].lastOnTime = 0;
    g_relays[i].lastOffTime = now;
    g_relays[i].totalOnDuration = 0;
    g_relays[i].cycleCount = 0;
    g_relays[i].cycleWindowStart = now;
    g_relays[i].cooldownLocked = false;
    g_relays[i].cooldownStart = 0;
    g_relays[i].rapidFireLockoutStart = 0;

    if (g_relays[i].pin != 255) {
      // Pre-latch HIGH before OUTPUT to prevent nanosecond pulse glitch
      digitalWrite(g_relays[i].pin, HIGH);  // Active LOW = OFF
      pinMode(g_relays[i].pin, OUTPUT);
    }

    Serial.print(F("[RELAY] Initialized "));
    Serial.print(relayNames[i]);
    Serial.print(F(" on GPIO "));
    Serial.print(g_relays[i].pin);
    Serial.println(F(" -> OFF (HIGH, safe state)"));
  }

  Serial.println(F("[RELAY] Compressor cooldown state will be loaded from SD cache if available."));

  return true;
}

// ============================================
// Core Relay Control
// ============================================

bool relayManager_setRelay(uint8_t relayIndex, bool turnOn, bool force) {
  if (relayIndex >= RELAY_COUNT) {
    return false;
  }

  RelayState* relay = &g_relays[relayIndex];

  if (relay->isActive == turnOn) {
    return true;
  }

  unsigned long now = millis();

  // --- RAPID-FIRE GUARD (bypassed when force=true) ---
  if (turnOn && !force) {
      if (relay->rapidFireLockoutStart > 0) {
          if (now - relay->rapidFireLockoutStart < RAPID_FIRE_LOCKOUT_MS) {
              Serial.print(F("[SAFETY] Rapid-fire lockout active on "));
              Serial.print(relayNames[relayIndex]);
              Serial.print(F(" ("));
              Serial.print((RAPID_FIRE_LOCKOUT_MS - (now - relay->rapidFireLockoutStart)) / 1000);
              Serial.println(F("s remaining)"));
              return false;
          }
          relay->cycleCount = 0;
          relay->cycleWindowStart = now;
          relay->rapidFireLockoutStart = 0;
      }

      unsigned long windowElapsed = now - relay->cycleWindowStart;
      if (windowElapsed >= RELAY_CYCLE_WINDOW_MS) {
          relay->cycleWindowStart = now;
          relay->cycleCount = 0;
      }

      uint8_t maxAllowed = RELAY_MAX_CYCLES_PER_MIN + RELAY_MANUAL_CYCLE_ALLOWANCE;
      if (relay->cycleCount >= maxAllowed) {
          relay->rapidFireLockoutStart = now;
          Serial.print(F("[SAFETY] Rapid-fire lockout triggered on "));
          Serial.print(relayNames[relayIndex]);
          Serial.print(F(" ("));
          Serial.print(relay->cycleCount);
          Serial.println(F(" ON events/min) — locked for 5s"));
          return false;
      }
  }

  // --- COOLDOWN CHECK (bypassed when force=true) ---
  if (relayManager_requiresCooldown(relayIndex) && turnOn && !force) {
    if (relay->cooldownLocked) {
      unsigned long elapsedSinceOff = now - relay->cooldownStart;
      if (elapsedSinceOff < COMPRESSOR_COOLDOWN_MS) {
        unsigned long remaining = COMPRESSOR_COOLDOWN_MS - elapsedSinceOff;
        Serial.print(F("[SAFETY] Cooldown active - "));
        Serial.print(remaining / 1000);
        Serial.println(F("s remaining. Refusing to start."));
        return false;
      } else {
        relay->cooldownLocked = false;
        Serial.println(F("[SAFETY] Cooldown complete - available for use"));
      }
    }
  }

  // --- RELAY CYCLE LIMIT CHECK (bypassed when force=true) ---
  if (turnOn && !force) {
    if (!relayManager_canToggle(relayIndex)) {
      Serial.print(F("[SAFETY] Relay cycle limit reached for "));
      Serial.print(relayNames[relayIndex]);
      Serial.print(F(" ("));
      Serial.print(relay->cycleCount);
      Serial.println(F("/min)"));
      return false;
    }
  }

  // --- EXECUTE STATE CHANGE ---
  if (turnOn) {
    digitalWrite(relay->pin, LOW);   // Active LOW = ON
    relay->isActive = true;
    relay->lastOnTime = now;
    relay->totalOnDuration = 0;
    relayManager_logCycle(relayIndex);

    Serial.print(F("[RELAY] "));
    Serial.print(relayNames[relayIndex]);
    Serial.println(F(" -> ON"));
  } else {
    digitalWrite(relay->pin, HIGH);  // Active LOW = OFF
    relay->isActive = false;
    relay->lastOffTime = now;
    relay->totalOnDuration = 0;

    if (relayManager_requiresCooldown(relayIndex)) {
      relay->cooldownLocked = true;
      relay->cooldownStart = now;
      relay->cooldownOffEpoch = rtc_getGH2000Seconds();
      Serial.print(F("[SAFETY] Cooldown started on "));
      Serial.print(relayNames[relayIndex]);
      Serial.print(F(" - locked for "));
      Serial.print(COMPRESSOR_COOLDOWN_SEC / 60);
      Serial.println(F(" minutes"));
    }

    Serial.print(F("[RELAY] "));
    Serial.print(relayNames[relayIndex]);
    Serial.println(F(" -> OFF"));
  }

  portENTER_CRITICAL(&g_stateMux);
  switch (relayIndex) {
    case RELAY_HOH:         g_systemState.hoHActive = turnOn; break;
    case RELAY_AIR_ASSIST:  g_systemState.airAssistActive = turnOn; break;
    case RELAY_EXHAUST:     g_systemState.exhaustFanActive = turnOn; break;
    case RELAY_COMPRESSOR:  g_systemState.compressorActive = turnOn; break;
  }
  portEXIT_CRITICAL(&g_stateMux);

  return true;
}

bool relayManager_isRelayOn(uint8_t relayIndex) {
  if (relayIndex >= RELAY_COUNT) return false;
  return g_relays[relayIndex].isActive;
}

bool relayManager_canToggle(uint8_t relayIndex) {
  if (relayIndex >= RELAY_COUNT) return false;

  RelayState* relay = &g_relays[relayIndex];
  unsigned long now = millis();

  if (now - relay->cycleWindowStart >= RELAY_CYCLE_WINDOW_MS) {
    relay->cycleWindowStart = now;
    relay->cycleCount = 0;
  }

  if (relay->cycleCount < RELAY_MAX_CYCLES_PER_MIN) {
    return true;
  }

  return false;
}

unsigned long relayManager_getOnDuration(uint8_t relayIndex) {
  if (relayIndex >= RELAY_COUNT) return 0;

  RelayState* relay = &g_relays[relayIndex];

  if (relay->isActive) {
    unsigned long now = millis();
    unsigned long elapsed = now - relay->lastOnTime;
    relay->totalOnDuration += elapsed;
    relay->lastOnTime = now;
  }

  return relay->totalOnDuration;
}

void relayManager_logCycle(uint8_t relayIndex) {
  if (relayIndex >= RELAY_COUNT) return;

  RelayState* relay = &g_relays[relayIndex];
  unsigned long now = millis();

  if (now - relay->cycleWindowStart >= RELAY_CYCLE_WINDOW_MS) {
    relay->cycleWindowStart = now;
    relay->cycleCount = 0;
  }

  relay->cycleCount++;
}

// ============================================
// Compressor Cooldown (Single Source of Truth)
// ============================================

bool relayManager_isCompressorCooldownActive() {
  RelayState* relay = &g_relays[RELAY_COMPRESSOR];

  if (!relay->cooldownLocked) {
    return false;
  }

  unsigned long now = millis();
  unsigned long elapsed = now - relay->cooldownStart;

  if (elapsed >= COMPRESSOR_COOLDOWN_MS) {
    relay->cooldownLocked = false;
    return false;
  }

  return true;
}

unsigned long relayManager_getCompressorCooldownRemaining() {
  RelayState* relay = &g_relays[RELAY_COMPRESSOR];

  if (!relay->cooldownLocked) {
    return 0;
  }

  unsigned long now = millis();
  unsigned long elapsed = now - relay->cooldownStart;

  if (elapsed >= COMPRESSOR_COOLDOWN_MS) {
    relay->cooldownLocked = false;
    return 0;
  }

  return COMPRESSOR_COOLDOWN_MS - elapsed;
}

// ============================================
// Cooldown Persistence Across Reboots
// ============================================

void relayManager_saveCooldownState() {
  Serial.println(F("[RELAY] Cooldown state flagged for SD persistence"));
}

void relayManager_loadCooldownState(unsigned long lastOffTimestamp, bool wasInCooldown) {
  if (!wasInCooldown) {
    Serial.println(F("[RELAY] No cooldown was active before shutdown"));
    return;
  }

  RelayState* relay = &g_relays[RELAY_COMPRESSOR];
  unsigned long now = millis();

  if (lastOffTimestamp > 0) {
    unsigned long currentTimestamp = rtc_getGH2000Seconds();

    if (currentTimestamp > lastOffTimestamp &&
        (currentTimestamp - lastOffTimestamp) < 86400UL) {
      unsigned long elapsed = currentTimestamp - lastOffTimestamp;

      if ((elapsed * 1000UL) >= COMPRESSOR_COOLDOWN_MS) {
        relay->cooldownLocked = false;
        Serial.print(F("[RELAY] Cooldown expired during downtime ("));
        Serial.print(elapsed);
        Serial.println(F("s) - compressor available immediately"));
        return;
      }

      unsigned long elapsedMs = elapsed * 1000UL;
      unsigned long remainingMs = COMPRESSOR_COOLDOWN_MS - elapsedMs;
      relay->cooldownLocked = true;
      relay->cooldownStart = now - elapsedMs;
      Serial.print(F("[RELAY] Cooldown restored with "));
      Serial.print(remainingMs / 1000);
      Serial.println(F("s remaining"));
      return;
    } else {
      Serial.println(F("[RELAY] RTC invalid or clock skew detected - applying full safety cooldown"));
    }
  }

  relay->cooldownLocked = true;
  relay->cooldownStart = now;
  Serial.print(F("[RELAY] Full cooldown applied (fail-safe) - "));
  Serial.print(COMPRESSOR_COOLDOWN_SEC / 60);
  Serial.println(F(" minutes"));
}

bool relayManager_isRelayLoud(uint8_t relayIndex) {
    if (relayIndex >= RELAY_COUNT) return false;
    return g_relayCaps[relayIndex].isLoud;
}

bool relayManager_requiresCooldown(uint8_t relayIndex) {
    if (relayIndex >= RELAY_COUNT) return false;
    return g_relayCaps[relayIndex].requiresCooldown;
}

bool relayManager_isBurstCycled(uint8_t relayIndex) {
    if (relayIndex >= RELAY_COUNT) return false;
    return g_relayCaps[relayIndex].isBurstCycled;
}

// ============================================
// Emergency Shutdown
// ============================================

void relayManager_forceAllOff() {
  unsigned long now = millis();

  // HARDWARE SAFETY FIRST: Cut power before ANY Serial I/O.
  // In an emergency, milliseconds matter. Serial can block on full UART buffer.
  portENTER_CRITICAL(&g_stateMux);
  for (uint8_t i = 0; i < RELAY_COUNT; i++) {
    if (g_relays[i].pin != 255) {
      digitalWrite(g_relays[i].pin, HIGH);  // Active LOW = OFF
    }
    g_relays[i].isActive = false;
    g_relays[i].lastOffTime = now;
    g_relays[i].totalOnDuration = 0;

    if (relayManager_requiresCooldown(i)) {
      g_relays[i].cooldownLocked = true;
      g_relays[i].cooldownStart = now;
      g_relays[i].cooldownOffEpoch = rtc_getGH2000Seconds();
    }
  }

  g_systemState.hoHActive = false;
  g_systemState.airAssistActive = false;
  g_systemState.exhaustFanActive = false;
  g_systemState.compressorActive = false;
  portEXIT_CRITICAL(&g_stateMux);

  // Safe to log now — hardware is already off
  Serial.println(F("[RELAY] EMERGENCY: ALL relays forced OFF and system state cleared"));
}
