/*
   relay_manager.h
   GrowHub32 - Relay Control & Safety Guardrails
   Version: 1.3.0
   Revision: Added rapidFireLockoutStart field for rapid-fire guard.
             Added RelayCapability struct and query functions for v1.4
             configurable relay mapping foundation.
             Removed duplicate relay index defines (now in config.h only).
             Documented single-thread assumption for g_relays[].
             Documented cooldownStart as millis() reference, not epoch.
             Documented getOnDuration() as pure getter (no relay-control side effects).
             Added cooldownOffEpoch field for persistent OFF timestamp (epoch seconds).

   THREADING: All relayManager functions are designed to be called from the
   main Arduino loop task only. They are not ISR-safe and not reentrant.
   g_systemState writes are mutex-protected for cross-task read consistency
   with ESP-NOW/WiFi callbacks, but g_relays[] access is single-threaded.
*/

#ifndef RELAY_MANAGER_H
#define RELAY_MANAGER_H

#include <Arduino.h>
#include "config.h"

// Relay state tracking per channel
struct RelayState {
  uint8_t pin;
  bool isActive;               // true = relay ON (pulled LOW), false = relay OFF (HIGH)
  unsigned long lastOnTime;    // last millis() when turned ON
  unsigned long lastOffTime;   // last millis() when turned OFF
  unsigned long totalOnDuration; // current continuous ON duration (ms)
  uint8_t cycleCount;          // cycles in current minute window
  unsigned long cycleWindowStart; // start of current 60s cycle window
  bool cooldownLocked;         // compressor cooldown lock (GH-SAFE-002)
  unsigned long cooldownStart; // millis() reference point when cooldown began
  // RTC epoch seconds captured at the exact moment the relay was turned OFF.
  // Set by relayManager_setRelay() for relays that require cooldown.
  // Used by sd_logger for accurate cross-reboot cooldown persistence.
  // 0 means RTC was unavailable at the moment of OFF.
  unsigned long cooldownOffEpoch;
  unsigned long rapidFireLockoutStart; // Rapid-fire guard lockout timestamp
};

// ============================================================
// Relay Capabilities (v1.4 foundation for configurable mapping)
// ============================================================
struct RelayCapability {
    bool isLoud;           // Triggers night mode confirmation popup in Web UI
    bool requiresCooldown; // Enforces compressor-style cooldown on re-start
    bool isBurstCycled;    // Uses ON/OFF burst timing (Air Assist behavior)
};

// ============================================================
// Public API
// ============================================================

// Initialize all relays to safe OFF state (GH-SYS-003)
bool relayManager_init();

// Set a relay ON or OFF. Returns true if state change was allowed and executed.
// Set force=true to bypass cooldown and cycle limits (manual override only).
bool relayManager_setRelay(uint8_t relayIndex, bool turnOn, bool force = false);

// Query relay current state
bool relayManager_isRelayOn(uint8_t relayIndex);

// Check if a relay is allowed to toggle (cycle limit enforcement)
bool relayManager_canToggle(uint8_t relayIndex);

// Get current continuous ON duration in milliseconds.
// Pure getter — does NOT change relay state. Caller enforces max ON time.
unsigned long relayManager_getOnDuration(uint8_t relayIndex);

// Track a relay cycle for rate limiting
void relayManager_logCycle(uint8_t relayIndex);

// Compressor cooldown queries (single source of truth - GH-SAFE-002)
bool relayManager_isCompressorCooldownActive();
unsigned long relayManager_getCompressorCooldownRemaining();

// Persist cooldown state across reboots (called by SD logger)
void relayManager_saveCooldownState();
void relayManager_loadCooldownState(unsigned long lastOffTimestamp, bool wasInCooldown);

// Emergency: force all relays OFF immediately (bypasses state tracking)
void relayManager_forceAllOff();

// Query relay capabilities (for Web UI and automation)
bool relayManager_isRelayLoud(uint8_t relayIndex);
bool relayManager_requiresCooldown(uint8_t relayIndex);
bool relayManager_isBurstCycled(uint8_t relayIndex);

// Exposed for safety module and UI (read-only)
extern RelayState g_relays[RELAY_COUNT];

#endif // RELAY_MANAGER_H
