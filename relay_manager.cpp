/*
   relay_manager.cpp
   GrowHub32 - Relay Control & Safety Guardrails Implementation
   Version: 1.2.4
   Revision: Fixed cooldown restoration to back-date cooldownStart by elapsed downtime.
             Added RTC sanity check with 24h cap to prevent absurdly large deltas.
             Fixed double-millis() drift in getOnDuration().
             Removed setRelay() call from getOnDuration() (pure getter now).
             Mutex-protected g_systemState writes in setRelay() and forceAllOff().
             Uses COMPRESSOR_COOLDOWN_MS from config.h for cooldown enforcement.
             COMPRESSOR_MAX_ON_MS is enforced by the caller (automation.cpp loop).
             Uses RELAY_CYCLE_WINDOW_MS from config.h.
             Fixed missing cooldownLocked=true in partial restoration path.

   RELAY LOGIC: Active LOW
   - digitalWrite(pin, LOW)  = Relay ON, circuit CLOSED
   - digitalWrite(pin, HIGH) = Relay OFF, circuit OPEN

   WIRING NOTE:
   - IN2 (GPIO12) requires external 10k pull-down to GND for safe boot.
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

// Relay capability table — source of truth for per-relay behavior.
// When configurable relay mapping is implemented, this table will
// be populated from stored configuration rather than hardcoded.
static const RelayCapability g_relayCaps[RELAY_COUNT] = {
    { false, false, false },  // HOH — silent, no cooldown, continuous
    { true,  false, true  },  // Air Assist — loud, no cooldown, burst cycled
    { false, false, false },  // Exhaust — moderate, no cooldown, continuous
    { true,  true,  false }   // Compressor — loud, cooldown, continuous
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
    g_relays[i].rapidFireLockoutStart = 0;
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

bool relayManager_setRelay(uint8_t relayIndex, bool turnOn, bool force) {
  if (relayIndex >= RELAY_COUNT) {
    return false;
  }

  RelayState* relay = &g_relays[relayIndex];

  // If no state change, do nothing
  if (relay->isActive == turnOn) {
    return true;  // Already in desired state - not a failure
  }

   unsigned long now = millis();

  // --- RAPID-FIRE GUARD (gated on turnOn only — OFF always allowed) ---
  // Prevents relay damage from UI bugs or network floods.
  // Allows RELAY_MAX_CYCLES_PER_MIN + RELAY_MANUAL_CYCLE_ALLOWANCE
  // ON events per window. Exceeding that locks the relay for
  // RAPID_FIRE_LOCKOUT_MS. OFF commands always proceed — relays
  // must be able to de-energize unconditionally.
  if (turnOn) {
      // Check if currently in lockout
      if (relay->rapidFireLockoutStart > 0) {
          if (now - relay->rapidFireLockoutStart < RAPID_FIRE_LOCKOUT_MS) {
              Serial.print(F("[SAFETY] Rapid-fire lockout active on "));
              Serial.print(relayNames[relayIndex]);
              Serial.print(F(" ("));
              Serial.print((RAPID_FIRE_LOCKOUT_MS - (now - relay->rapidFireLockoutStart)) / 1000);
              Serial.println(F("s remaining)"));
              return false;
          }
          // Lockout served — reset for a fresh window
          relay->cycleCount = 0;
          relay->cycleWindowStart = now;
          relay->rapidFireLockoutStart = 0;
      }

      // Count ON events in sliding 60-second window
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

      // Cooldown on every OFF event for relays that require it
    if (relayManager_requiresCooldown(relayIndex)) {
      relay->cooldownLocked = true;
      relay->cooldownStart = now;
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

  // Update system state under mutex for cross-task consistency
  // NOTE: g_relays[] fields are updated BEFORE this critical section.
  // g_systemState is synchronized here for cross-task readers (ESP-NOW, WiFi callbacks).
  // Do NOT read g_relays[] from ISR/callback context — see header threading note.
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

  // Reset cycle window every RELAY_CYCLE_WINDOW_MS
  if (now - relay->cycleWindowStart >= RELAY_CYCLE_WINDOW_MS) {
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
  // Pure getter — returns current continuous ON duration in milliseconds.
  // Updates internal accounting fields (totalOnDuration, lastOnTime) but
  // does NOT change relay state. Caller enforces GH-SAFE-002.
  if (relayIndex >= RELAY_COUNT) return 0;

  RelayState* relay = &g_relays[relayIndex];

  if (relay->isActive) {
    // Capture now once to prevent drift between elapsed calculation and lastOnTime update
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

  // Reset cycle window every RELAY_CYCLE_WINDOW_MS
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
    unsigned long currentTimestamp = rtc_getGH2000Seconds();

    // Sanity check: reject if RTC appears to have gone backwards (dead battery, reset).
    // Also cap maximum believable downtime at 24h to prevent absurdly large deltas
    // from corrupted SD cache or RTC epoch rollover.
    if (currentTimestamp > lastOffTimestamp &&
        (currentTimestamp - lastOffTimestamp) < 86400UL) {
      unsigned long elapsed = currentTimestamp - lastOffTimestamp;

      // Use MS constant for comparison to avoid integer promotion issues
      if ((elapsed * 1000UL) >= COMPRESSOR_COOLDOWN_MS) {
        // Cooldown already expired during downtime
        relay->cooldownLocked = false;
        Serial.print(F("[RELAY] Cooldown expired during downtime ("));
        Serial.print(elapsed);
        Serial.println(F("s) - compressor available immediately"));
        return;
      }

      // Partial cooldown elapsed — back-date cooldownStart so only remaining time is enforced.
      // cooldownStart is a millis() reference point, not an epoch timestamp.
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

  // Fallback: No valid RTC data or sanity check failed.
  // Apply FULL cooldown as fail-safe (GH-SAFE-002: err on the side of compressor protection).
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
  Serial.println(F("[RELAY] EMERGENCY: Forcing ALL relays OFF"));

  for (uint8_t i = 0; i < RELAY_COUNT; i++) {
    digitalWrite(g_relays[i].pin, HIGH);  // Active LOW = OFF
    g_relays[i].isActive = false;
    g_relays[i].lastOffTime = millis();
    g_relays[i].totalOnDuration = 0;

        // Relays requiring cooldown enter cooldown on emergency shutdown
    if (relayManager_requiresCooldown(i)) {
      g_relays[i].cooldownLocked = true;
      g_relays[i].cooldownStart = millis();
    }
  }

  // Update system state under mutex for cross-task consistency
  portENTER_CRITICAL(&g_stateMux);
  g_systemState.hoHActive = false;
  g_systemState.airAssistActive = false;
  g_systemState.exhaustFanActive = false;
  g_systemState.compressorActive = false;
  portEXIT_CRITICAL(&g_stateMux);
}
