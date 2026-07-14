/*
   relay_manager.h
   GrowHub32 - Relay Control & Safety Guardrails
   Version: 1.2.3
   Revision: Consolidated compressor cooldown as single source of truth.
             Added cooldown persistence using RTC epoch seconds.
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
  unsigned long cooldownStart; // millis() when cooldown began
};

// Public API

// Initialize all relays to safe OFF state (GH-SYS-003)
bool relayManager_init();

// Set a relay ON or OFF. Returns true if state change was allowed and executed.
// Returns false if blocked by safety guardrails.
bool relayManager_setRelay(uint8_t relayIndex, bool turnOn);

// Query relay current state
bool relayManager_isRelayOn(uint8_t relayIndex);

// Check if a relay is allowed to toggle (cycle limit enforcement)
bool relayManager_canToggle(uint8_t relayIndex);

// Get current continuous ON duration in milliseconds.
// For the compressor, this also enforces the 5-minute max ON limit (GH-SAFE-002).
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

// Relay indices
#define RELAY_HOH         0
#define RELAY_AIR_ASSIST  1
#define RELAY_EXHAUST     2
#define RELAY_COMPRESSOR  3
#define RELAY_COUNT       4

// Exposed for safety module and UI (read-only)
extern RelayState g_relays[RELAY_COUNT];

#endif // RELAY_MANAGER_H
