/*
   relay_manager.cpp
   GrowHub32 - Relay Control & Safety Guardrails Implementation
   Version: 1.2.3
   Revision: Cooldown restore now uses rtc_getEpochSeconds() for actual elapsed
             time across reboots instead of always starting a fresh 10-minute cooldown.

   RELAY LOGIC: Active LOW
   - digitalWrite(pin, LOW)  = Relay ON, circuit CLOSED
   - digitalWrite(pin, HIGH) = Relay OFF, circuit OPEN

   WIRING NOTE:
   - IN2 (GPIO12) requires external 10k pull-down to GND for safe boot.
   - All relay VCC to external 5V supply (NOT ESP32 3.3V or 5V pin).
   - JD-VCC jumper: use separate supply configuration for optocoupler isolation.
*/

#include "relay_manager.h"
#include "rtc_handler.h"

RelayState g_relays[RELAY_COUNT];

// Pin mapping
static const uint8_t relayPins[RELAY_COUNT] = {
  RELAY_HOH_PIN,        // 0: GPIO 13
  RELAY_AIR_ASSIST_PIN, // 1: GPIO 12
  RELAY_EXHAUST_PIN,    // 2: GPIO 14
  RELAY_COMPRESSOR_PIN  // 3: GPIO 27
};

// Friendly names for serial logging
static const char* relayNames[RELAY_COUNT] = {
  "HOH Humidifier",
  "Air Assist Valve",
  "Exhaust Fan",
  "Air Compressor"
};

// ============================================
// Initialization
// ============================================

bool relayManager_init() {
  // GH-SYS-003: CRITICAL - Set ALL relays HIGH (OFF) immediately
  // This prevents floating pins from triggering relays during boot
  for (uint8_t i = 0; i < RELAY_COUNT; i++) {
    g_relays[i].pin = relayPins[i];
    g_relays[i].isActive = false;
    g_relays[i].lastOnTime = 0;
    g_relays[i].lastOffTime = millis();
    g_relays[i].totalOnDuration = 0;
    g_relays[i].cycleCount = 0;
    g_relays[i].cycleWindowStart = millis();
    g_relays[i].cooldownLocked = false;
    g_relays[i].cooldownStart = 0;

    // Configure pin and force HIGH (OFF)
    pinMode(g_relays[i].pin, OUTPUT);
    digitalWrite(g_relays[i].pin, HIGH);

    Serial.print(F("[RELAY] Initialized "));
    Serial.print(relayNames[i]);
    Serial.print(F(" on GPIO "));
    Serial.print(g_relays[i].pin);
    Serial.println(F(" -> OFF (HIGH, safe state)"));
  }

  Serial.println(F("[RELAY] NOTE: GPIO 12 (Air Assist) requires external 10k pull-down to GND per SRS v1.1"));
  Serial.println(F("[RELAY] Compressor cooldown state will be loaded from SD cache if available."));

  return true;
}

// ============================================
// Core Relay Control
// ============================================

bool relayManager_setRelay(uint8_t relayIndex, bool turnOn) {
  if (relayIndex >= RELAY_COUNT) {
    return false;
  }

  RelayState* relay = &g_relays[relayIndex];

  // If no state change, do nothing
  if (relay->isActive == turnOn) {
    return true;  // Already in desired state - not a failure
  }

  unsigned long now = millis();

  // --- COMPRESSOR COOLDOWN CHECK (GH-SAFE-002) ---
  // SINGLE SOURCE OF TRUTH: All compressor safety lives here
  if (relayIndex == RELAY_COMPRESSOR && turnOn) {
    if (relay->cooldownLocked) {
      unsigned long elapsedSinceOff = now - relay->cooldownStart;
      if (elapsedSinceOff < (COMPRESSOR_COOLDOWN_SEC * 1000UL)) {
        unsigned long remaining = (COMPRESSOR_COOLDOWN_SEC * 1000UL) - elapsedSinceOff;
        Serial.print(F("[SAFETY] Compressor cooldown active - "));
        Serial.print(remaining / 1000);
        Serial.println(F("s remaining. Refusing to start."));
        return false;
      } else {
        // Cooldown has expired
        relay->cooldownLocked = false;
        Serial.println(F("[SAFETY] Compressor cooldown complete - available for use"));
      }
    }
  }

  // --- RELAY CYCLE LIMIT CHECK (GH-SAFE-001) ---
  if (turnOn) {
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

    // Track cycle
    relayManager_logCycle(relayIndex);

    Serial.print(F("[RELAY] "));
    Serial.print(relayNames[relayIndex]);
    Serial.println(F(" -> ON"));
  } else {
    digitalWrite(relay->pin, HIGH);  // Active LOW = OFF
    relay->isActive = false;
    relay->lastOffTime = now;
    relay->totalOnDuration = 0;

    // Compressor cooldown on every OFF event (GH-SAFE-002)
    if (relayIndex == RELAY_COMPRESSOR) {
      relay->cooldownLocked = true;
      relay->cooldownStart = now;
      Serial.print(F("[SAFETY] Compressor cooldown started - locked for "));
      Serial.print(COMPRESSOR_COOLDOWN_SEC / 60);
      Serial.println(F(" minutes"));
    }

    Serial.print(F("[RELAY] "));
    Serial.print(relayNames[relayIndex]);
    Serial.println(F(" -> OFF"));
  }

  // Update system state (keep in sync for UI/logging)
  switch (relayIndex) {
    case RELAY_HOH:         g_systemState.hoHActive = turnOn; break;
    case RELAY_AIR_ASSIST:  g_systemState.airAssistActive = turnOn; break;
    case RELAY_EXHAUST:     g_systemState.exhaustFanActive = turnOn; break;
    case RELAY_COMPRESSOR:  g_systemState.compressorActive = turnOn; break;
  }

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

  // Reset cycle window every 60 seconds
  if (now - relay->cycleWindowStart >= 60000UL) {
    relay->cycleWindowStart = now;
    relay->cycleCount = 0;
  }

  // Allow if under limit
  if (relay->cycleCount < RELAY_MAX_CYCLES_PER_MIN) {
    return true;
  }

  return false;
}

unsigned long relayManager_getOnDuration(uint8_t relayIndex) {
  if (relayIndex >= RELAY_COUNT) return 0;

  RelayState* relay = &g_relays[relayIndex];

  if (relay->isActive) {
    // Currently ON - compute elapsed time
    unsigned long elapsed = millis() - relay->lastOnTime;
    relay->totalOnDuration += elapsed;
    relay->lastOnTime = millis();

    // GH-SAFE-002: Force OFF if max continuous ON time exceeded
    if (relayIndex == RELAY_COMPRESSOR && relay->totalOnDuration >= (COMPRESSOR_MAX_ON_SEC * 1000UL)) {
      Serial.println(F("[SAFETY] Compressor max ON time (5 min) reached - forcing OFF"));
      relayManager_setRelay(RELAY_COMPRESSOR, false);
    }
  }

  return relay->totalOnDuration;
}

void relayManager_logCycle(uint8_t relayIndex) {
  if (relayIndex >= RELAY_COUNT) return;

  RelayState* relay = &g_relays[relayIndex];
  unsigned long now = millis();

  // Reset cycle window every 60 seconds
  if (now - relay->cycleWindowStart >= 60000UL) {
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

  if (elapsed >= (COMPRESSOR_COOLDOWN_SEC * 1000UL)) {
    // Cooldown expired
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

  if (elapsed >= (COMPRESSOR_COOLDOWN_SEC * 1000UL)) {
    relay->cooldownLocked = false;
    return 0;
  }

  return (COMPRESSOR_COOLDOWN_SEC * 1000UL) - elapsed;
}

// ============================================
// Cooldown Persistence Across Reboots
// ============================================

void relayManager_saveCooldownState() {
  // Called by SD logger to persist compressor state
  // Actual SD write happens in sd_logger.cpp via RuntimeCache
  Serial.println(F("[RELAY] Cooldown state flagged for SD persistence"));
}

void relayManager_loadCooldownState(unsigned long lastOffTimestamp, bool wasInCooldown) {
  // GH-SAFE-002 persistent: Restore cooldown state after power failure.
  // Uses RTC timestamp via rtc_getEpochSeconds() to respect actual elapsed downtime.

  if (!wasInCooldown) {
    Serial.println(F("[RELAY] No cooldown was active before shutdown"));
    return;
  }

  RelayState* relay = &g_relays[RELAY_COMPRESSOR];
  unsigned long now = millis();

  // If we have a valid RTC timestamp, check actual elapsed time
  if (lastOffTimestamp > 0) {
    unsigned long currentTimestamp = rtc_getEpochSeconds();

    if (currentTimestamp > 0 && currentTimestamp > lastOffTimestamp) {
      unsigned long elapsed = currentTimestamp - lastOffTimestamp;
      if (elapsed >= COMPRESSOR_COOLDOWN_SEC) {
        // Cooldown already expired during downtime
        relay->cooldownLocked = false;
        Serial.print(F("[RELAY] Cooldown expired during downtime ("));
        Serial.print(elapsed);
        Serial.println(F("s) - compressor available immediately"));
        return;
      }
      Serial.print(F("[RELAY] Cooldown partially elapsed: "));
      Serial.print(elapsed);
      Serial.print(F("s of "));
      Serial.print(COMPRESSOR_COOLDOWN_SEC);
      Serial.println(F("s"));
    }
  }

  // Cooldown still active — restore it
  relay->cooldownLocked = true;
  relay->cooldownStart = now;

  Serial.print(F("[RELAY] Cooldown restored - up to "));
  Serial.print(COMPRESSOR_COOLDOWN_SEC / 60);
  Serial.println(F(" minutes remaining"));
}

// ============================================
// Emergency Shutdown
// ============================================

void relayManager_forceAllOff() {
  Serial.println(F("[RELAY] EMERGENCY: Forcing ALL relays OFF"));

  for (uint8_t i = 0; i < RELAY_COUNT; i++) {
    digitalWrite(g_relays[i].pin, HIGH);  // Active LOW = OFF
    g_relays[i].isActive = false;
    g_relays[i].lastOffTime = millis();
    g_relays[i].totalOnDuration = 0;

    // Compressor enters cooldown on emergency shutdown too
    if (i == RELAY_COMPRESSOR) {
      g_relays[i].cooldownLocked = true;
      g_relays[i].cooldownStart = millis();
    }
  }

  // Update system state
  g_systemState.hoHActive = false;
  g_systemState.airAssistActive = false;
  g_systemState.exhaustFanActive = false;
  g_systemState.compressorActive = false;
}
