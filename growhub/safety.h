/*
   safety.h
   GrowHub32 - Hardware Protection & System Safety
   Version: 1.2.3
   Revision: Replaced hand-rolled hw_timer watchdog with ESP-IDF Task Watchdog
             Timer (esp_task_wdt) for reliable reboot on main loop hang.
             Added safety_didWatchdogInitFail() for deferred alerting.
*/

#ifndef SAFETY_H
#define SAFETY_H

#include <Arduino.h>
#include <esp_task_wdt.h>
#include "config.h"

struct DryRunState {
  unsigned long hoHStartTime;
  float startHumidity;
  bool alertSent;
};

struct FanStallState {
  unsigned long fanStartTime;
  uint16_t startCO2;
  bool alertSent;
};

void safety_initWatchdog();
void safety_feedWatchdog();
bool safety_didWatchdogInitFail();
void safety_checkDryRun(unsigned long now);
void safety_checkFanStall(unsigned long now);
bool safety_isDryRunAlertActive();
bool safety_isFanStallAlertActive();

#endif
