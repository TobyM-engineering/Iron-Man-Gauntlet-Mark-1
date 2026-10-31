#include "modesw.h"
#include "hal.h"
#include <GauntletBenchIO.h>

static Mode     mode = MODE_SWITCH_FITTED ? Mode::Safe : MODE_DEFAULT, rawMode = mode, candidate = mode;
static bool     forced = !MODE_SWITCH_FITTED;
static bool     seenSafe = !MODE_SWITCH_FITTED || !MODE_POWERUP_NEEDS_SAFE;   // the switch has been in the middle (SAFE) since power-up
static uint32_t candidateSince = 0, lastPollMs = 0, changes = 0, rawChanges = 0, chatterWinMs = 0, chatterN = 0, lastChatterWarn = 0, lastBothLowWarn = 0;
static int      levelA = HIGH, levelB = HIGH;
static void (*onChange)(Mode, Mode) = nullptr;

const char* modeName(Mode m) { return m == Mode::Safe ? "SAFE" : m == Mode::Show ? "SHOW" : "FULL"; }
const char* modeNowName()    { return modeName(mode); }
Mode        modeNow()        { return mode; }
bool        modeAllowsShow() { return mode == Mode::Show || mode == Mode::Flight; }
bool        modeAllowsFull() { return mode == Mode::Flight; }
bool        modeAllowsArming() { return modeAllowsFull(); }
bool        modeWaitingForSafe() { return !seenSafe; }
bool        modeForced()     { return forced; }
uint32_t    modeChanges()    { return changes; }
void        modeOnChange(void (*fn)(Mode, Mode)) { onChange = fn; }

// (Toby, 2026-10-08) the MIDDLE = both high = SAFE, the GPIO 36 end = A low = SHOW, the GPIO 39 end = B low = FULL.
// Open is SAFE: a broken wire to either end, a lost common or a bad contact reads both high. Both low cannot happen on
// an ON-OFF-ON (one common, one pole at a time): a short or a wiring fault - it reads SAFE too.
static Mode decode(int a, int b) {
  if (a == HIGH && b == HIGH) return Mode::Safe;
  if (a == LOW && b == HIGH)  return Mode::Show;
  if (a == HIGH && b == LOW)  return Mode::Flight;
  return Mode::Safe;
}
// what the switch's position means right now: SAFE until the middle (SAFE) has been seen once after power-up
static Mode effective(Mode sw) { if (sw == Mode::Safe) seenSafe = true; return seenSafe ? sw : Mode::Safe; }

static void apply(Mode m, const char* how) {
  if (m == mode) return;
  Mode from = mode; mode = m; changes++;
  Bench.printf("mode: %s -> %s (%s)\n", modeName(from), modeName(m), how);
  if (onChange) onChange(from, m);
}

void modeswBegin() {
  halPinMode(PIN_MODE_A, INPUT);               // 36/39 have no internal pulls: the 10 k are on the board
  halPinMode(PIN_MODE_B, INPUT);
  levelA = halDigitalRead(PIN_MODE_A); levelB = halDigitalRead(PIN_MODE_B);
  rawMode = candidate = decode(levelA, levelB);
  candidateSince = millis();
  if (MODE_SWITCH_FITTED) {
    mode = effective(rawMode);
    Bench.printf("mode: switch on GPIO %u/%u reads A=%d B=%d -> %s (polled every %lu ms, %lu ms debounce, never an interrupt)\n", PIN_MODE_A, PIN_MODE_B, levelA, levelB, modeName(rawMode), (unsigned long)MODE_POLL_MS, (unsigned long)MODE_DEBOUNCE_MS);
    if (!seenSafe) Bench.printf("mode: switched on in %s - holding SAFE until the switch has been in the middle (SAFE) once\n", modeName(rawMode));
    if (levelA == LOW && levelB == LOW) Bench.println("mode: BOTH switch poles read low - impossible on this switch (a short to ground?): SAFE");
  }
  else Bench.printf("mode: no switch fitted (MODE_SWITCH_FITTED false) - %s until key M / /mode?force= says otherwise\n", modeName(mode));
}

void modeswPoll() {
  uint32_t now = millis();
  if (now - lastPollMs < MODE_POLL_MS) return;
  lastPollMs = now;
  int a = halDigitalRead(PIN_MODE_A), b = halDigitalRead(PIN_MODE_B);
  levelA = a; levelB = b;
  Mode m = decode(a, b);
  if (m != candidate) { candidate = m; candidateSince = now; rawChanges++; chatterN++; }
  if (now - chatterWinMs >= 1000) {              // a floating pin: dozens of raw changes a second
    if (chatterN >= 20 && now - lastChatterWarn >= 10000) { lastChatterWarn = now; Bench.printf("mode: the switch pins changed %lu times in a second - not wired, or no pull-ups? (forced %s meanwhile: key M)\n", (unsigned long)chatterN, modeName(mode)); }
    chatterWinMs = now; chatterN = 0;
  }
  if (candidate != rawMode && now - candidateSince >= MODE_DEBOUNCE_MS) {
    rawMode = candidate;
    if (a == LOW && b == LOW && now - lastBothLowWarn >= 10000) { lastBothLowWarn = now; Bench.println("mode: BOTH switch poles read low - impossible on this switch (a short to ground?): SAFE"); }
    if (forced && MODE_SWITCH_FITTED) { forced = false; Bench.println("mode: the switch moved - following it again"); }
    if (!forced) {
      bool waited = !seenSafe;
      Mode e = effective(rawMode);
      if (waited && !seenSafe) Bench.printf("mode: the switch moved to %s - still SAFE: after power-up it must be in the middle (SAFE) once first\n", modeName(rawMode));
      else if (waited) Bench.println("mode: the switch is in SAFE - from now on it is followed");
      apply(e, "switch");
    }
  }
}

void modeForce(Mode m) { forced = true; apply(m, "forced"); if (m == mode) Bench.printf("mode: forced %s (the switch takes over again when it moves)\n", modeName(m)); }
void modeFollowSwitch() { forced = false; if (MODE_SWITCH_FITTED) apply(effective(rawMode), "switch"); }

String modeJson() {
  char b[128];
  snprintf(b, sizeof b, "{\"mode\":\"%s\",\"forced\":%d,\"a\":%d,\"b\":%d,\"switch\":\"%s\",\"waiting\":%d,\"changes\":%lu}", modeName(mode), forced ? 1 : 0, levelA, levelB, modeName(rawMode), seenSafe ? 0 : 1, (unsigned long)changes);
  return String(b);
}
String modeStatusLine() {
  char b[200];
  snprintf(b, sizeof b, "mode: %s%s (switch A=%d B=%d -> %s%s; %lu changes; show %s, lasers / LiDAR / arming %s)", modeName(mode), forced ? " (forced)" : "", levelA, levelB, modeName(rawMode),
           seenSafe ? "" : ", waiting for the middle (SAFE) after power-up", (unsigned long)changes, modeAllowsShow() ? "allowed" : "refused", modeAllowsFull() ? "allowed" : "refused");
  return String(b);
}

#if HW_FAKE
// The self-test: as if the arm had just been switched on with the switch where the fake pins say (the latch again).
void modeFakePowerUp() {
  Mode was = mode;
  seenSafe = !MODE_SWITCH_FITTED || !MODE_POWERUP_NEEDS_SAFE; forced = !MODE_SWITCH_FITTED;
  if (MODE_SWITCH_FITTED) mode = Mode::Safe;
  modeswBegin();
  if (was != mode) { changes++; if (onChange) onChange(was, mode); }
}
#endif
