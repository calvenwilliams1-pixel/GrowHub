// ============================================================
// v1.3: Night Mode Scheduling
// ============================================================

int rtc_minutesUntilNightMode() {
  // Returns minutes until night mode starts, or 0 if already active.
  // Night mode: NIGHT_MODE_START_HOUR:00 to NIGHT_MODE_END_HOUR:00 next day.
  // The schedule spans midnight (e.g., 21:00 → 10:00), so a simple
  // range check like (hour >= start && hour < end) would fail.
  //
  // Algorithm:
  //   Convert current time to minutes-since-midnight.
  //   Convert night start and end to minutes-since-midnight.
  //   If currently in night mode (minutes >= start OR minutes < end), return 0.
  //   If before start (morning/afternoon), return start - current.
  //   If after end but before midnight wrap, this case is caught above.
  //
  // Example: Current time 20:45 → minutes=1245, start=1260 → return 15.
  // Example: Current time 22:00 → minutes=1320, start=1260 → return 0 (in night mode).
  // Example: Current time 09:30 → minutes=570, end=600 → return 0 (in night mode).
  // Example: Current time 11:00 → minutes=660 → return 1260-660=600 (10 hours).

  RTCTime now;
  if (!rtc_readTime(&now)) {
    return -1;  // RTC unavailable — caller should abort
  }

  int currentMinutes = (now.hours * 60) + now.minutes;
  int nightStartMinutes = NIGHT_MODE_START_HOUR * 60;  // e.g., 1260 (21:00)
  int nightEndMinutes = NIGHT_MODE_END_HOUR * 60;      // e.g., 600 (10:00)

  // Check if currently in night mode
  // Night mode is active if: current >= start (evening) OR current < end (early morning)
  if (currentMinutes >= nightStartMinutes || currentMinutes < nightEndMinutes) {
    return 0;  // Night mode is active right now
  }

  // Not in night mode. Must be between end (10:00) and start (21:00).
  // Return minutes until start.
  return nightStartMinutes - currentMinutes;
}
