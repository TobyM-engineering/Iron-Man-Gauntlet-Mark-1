#include "palm.h"
static uint16_t alertMs = 300;   // the drone-battery blink period (palmAlertRate): 700 = getting low, 300 = LOW
#include "config.h"
#include "hal.h"
#include <GauntletBenchIO.h>
#include <GauntletLink.h>

// PALM_WALK_EXTRA pixels more than config says exist: the chain test drives those too, so a
// chain that is longer than PALM_COUNT shows itself. Normally they only ever receive "dark".
static LedChain ring(PALM_COUNT + PALM_WALK_EXTRA, PIN_PALM_LED);
static constexpr uint16_t WALK_STEPS = PALM_COUNT + PALM_WALK_EXTRA;

struct Colour { const char* name; uint8_t r, g, b; };
static const Colour COLOURS[] = {
  { "white",    255, 255, 255 },
  { "repulsor", 150, 205, 255 },   // the cool white-blue of the films
  { "red",      255,   0,   0 },
  { "orange",   255,  70,   0 },
  { "amber",    255, 170,   0 },
  { "green",      0, 255,   0 },
  { "cyan",       0, 210, 255 },
  { "blue",       0,  30, 255 },
  { "magenta",  255,   0, 170 },
};
static constexpr uint8_t N_COLOURS = sizeof(COLOURS) / sizeof(COLOURS[0]);

static uint8_t  colour = 0;                  // index into COLOURS
static uint8_t  level  = 0;                  // 0..PALM_LEVELS, 0 = off; never above PALM_LEVELS
static uint8_t  lastOn = PALM_DEFAULT_LEVEL; // the level it had before the last switch-off: a switch-on comes back at it (Toby 2026-09-29: "turn it back on with whatever brightness it just was")
static void     rememberOff() { if (level) lastOn = level; }   // call before level goes to 0
static bool     held   = false;              // servo attached: dark whatever level says
static bool     glow   = false;              // SAFE's dim palm glow (an idle light): shown only while the setting is off, the setting itself untouched
static float    shown[3]  = { 0, 0, 0 };     // per-channel drive on the ring right now, 0..255
static float    from[3]   = { 0, 0, 0 };     // ramp start
static float    target[3] = { 0, 0, 0 };     // ramp end
static bool     ramping   = false;
static uint32_t rampT0 = 0, lastStepMs = 0;
static String   lastMsg;                     // the console line the last command produced - /ring hands it to the page
static int16_t  walkI  = -1;                 // >= 0: the chain test is running and this is the pixel lit
static uint32_t walkT0 = 0;
static bool     warning    = false;          // battery low: blink white, whatever is set
static bool     alert      = false;          // the drone's battery low: blink red (the white LBO blink wins)
static int8_t   alertShown = -1;
static uint32_t alertT0    = 0;
static int8_t   blinkShown = -1;             // what the blink last wrote (1 on, 0 off, -1 nothing yet)
static uint32_t blinkT0    = 0;
static constexpr uint8_t C_WHITE = 0, C_RED = 2;   // indexes into COLOURS, for the thumb gesture

static void put(const float v[3]) {          // write one colour to every pixel
  uint32_t c = ring.Color((uint8_t)(v[0] + 0.5f), (uint8_t)(v[1] + 0.5f), (uint8_t)(v[2] + 0.5f));
  for (uint16_t i = 0; i < PALM_COUNT; i++) ring.setPixelColor(i, c);
  ring.show();
  for (int i = 0; i < 3; i++) shown[i] = v[i];
}
static void dark() { ring.clear(); ring.show(); for (int i = 0; i < 3; i++) shown[i] = 0; }   // the whole buffer, extras included
static void showWalk() {                     // chain test: only pixel walkI lit, dim white
  ring.clear();
  ring.setPixelColor(walkI, PALM_WALK_LEVEL, PALM_WALK_LEVEL, PALM_WALK_LEVEL);
  ring.show();
  for (int i = 0; i < 3; i++) shown[i] = 0;  // one dim pixel: call it dark for the estimate
}

// The drive the setting asks for, 0..255 per channel. asSet = ignore the servo hold.
static_assert(PALM_GLOW_COLOUR < N_COLOURS && PALM_GLOW_LEVEL <= PALM_LEVELS, "PALM_GLOW_COLOUR / PALM_GLOW_LEVEL out of range");
static void wanted(float out[3], bool asSet = false) {
  if (!asSet && !held && level == 0 && glow) {   // the SAFE glow, under everything the wearer sets
    float g = (PALM_GLOW_LEVEL / (float)PALM_LEVELS) * (PALM_CAP / 255.0f); const Colour& c = COLOURS[PALM_GLOW_COLOUR];
    out[0] = c.r * g; out[1] = c.g * g; out[2] = c.b * g; return;
  }
  float k = (held && !asSet) ? 0.0f : (level / (float)PALM_LEVELS) * (PALM_CAP / 255.0f);
  const Colour& c = COLOURS[colour];
  out[0] = c.r * k; out[1] = c.g * k; out[2] = c.b * k;
}

// WS2812: about PALM_MA_PER_CHANNEL per colour channel per pixel at full drive, plus PALM_MA_IDLE each while dark.
static float milliampsOf(const float v[3]) {
  return PALM_COUNT * (PALM_MA_IDLE + (v[0] + v[1] + v[2]) / 255.0f * PALM_MA_PER_CHANNEL);
}
uint16_t palmMilliamps() { return (uint16_t)(milliampsOf(shown) + 0.5f); }

static unsigned pctOfFull(uint8_t lvl) { return (lvl * PALM_CAP * 100UL + PALM_LEVELS * 255UL / 2) / (PALM_LEVELS * 255UL); }

static void ease() {                         // from what shows now to what is wanted, over PALM_RAMP_MS (palmPoll steps it)
  wanted(target);
  for (int i = 0; i < 3; i++) from[i] = shown[i];
  rampT0 = millis(); ramping = true;
}

static String describe() {                   // the setting: "white 5/10 (15 % of full, ~0.19 A)" or "off"
  if (level == 0) return "off";
  float w[3]; wanted(w, true);
  char b[80];
  snprintf(b, sizeof b, "%s %u/%u (%u %% of full, ~%.2f A)", COLOURS[colour].name, (unsigned)level, (unsigned)PALM_LEVELS, pctOfFull(level), milliampsOf(w) / 1000.0f);
  return b;
}
static const char* heldNote() { return held ? "  (dark until the servo lets go - x)" : ""; }
static void say(const String& m) { lastMsg = m; Bench.println(m); }

void palmBegin() {
  ring.begin();
  dark();                                    // before WiFi, before the sensors: the ring must not sit lit at random
}

void palmPoll() {
  uint32_t now = millis();
  if (warning) {                             // battery low: white, LBO_BLINK_MS on / off, at LBO_BLINK_LEVEL - unless the servo holds the ring dark
    if (held) return;
    int8_t on = (((now - blinkT0) / LBO_BLINK_MS) & 1) ? 0 : 1;
    if (on == blinkShown) return;
    blinkShown = on;
    if (on) { float k = (LBO_BLINK_LEVEL / (float)PALM_LEVELS) * (PALM_CAP / 255.0f); float v[3] = { 255 * k, 255 * k, 255 * k }; put(v); }
    else dark();
    return;
  }
  if (alert && !held) {                      // the drone's battery low: red, 300 ms on / off at 6/10 - unmistakable next to the steady colours
    int8_t on = (((now - alertT0) / alertMs) & 1) ? 0 : 1;
    if (on != alertShown) { alertShown = on; if (on) { float k = (6 / (float)PALM_LEVELS) * (PALM_CAP / 255.0f); float v[3] = { 255 * k, 0, 0 }; put(v); } else dark(); }
    return;
  }
  if (walkI >= 0) {                          // chain test: next pixel every PALM_WALK_MS
    if (now - walkT0 < PALM_WALK_MS) return;
    walkT0 = now;
    if (++walkI >= (int16_t)WALK_STEPS) {
      walkI = -1;
      dark();
      ease();
      say("palm: chain test done - pixels 0-" + String(WALK_STEPS - 1) + " were driven one after another; back to " + describe() +
          ". if some never lit, the chain is broken right after the last one that did (its DOUT, the next pixel's DIN, or that pixel's 5 V / GND)");
      return;
    }
    showWalk();
    Bench.printf("palm test: pixel %d%s\n", (int)walkI, walkI >= (int16_t)PALM_COUNT ? "  (past the configured count - lights only if the chain is longer than config says)" : "");
    return;
  }
  if (!ramping) return;
  if (now - lastStepMs < 15) return;         // ~60 steps a second
  lastStepMs = now;
  float f = (now - rampT0) / (float)PALM_RAMP_MS;
  if (f >= 1.0f) { f = 1.0f; ramping = false; }
  float v[3];
  for (int i = 0; i < 3; i++) v[i] = from[i] + (target[i] - from[i]) * f;
  put(v);
}

// ---- setting changes: one console line each (also what /ring hands back), then the ramp ----
void palmNextColour() {
  if (level == 0) {                          // off: switch on first, in the current colour, at the level it had
    level = lastOn;
    ease();
    say("palm: on - " + describe() + heldNote());
    return;
  }
  colour = (colour + 1) % N_COLOURS;
  ease();
  say("palm: " + describe() + heldNote());
}

static void setColour(uint8_t i) {           // the page's swatches: a colour picked directly; off -> on
  colour = i % N_COLOURS;
  bool wasOff = level == 0;
  if (wasOff) level = lastOn;
  ease();
  say(String(wasOff ? "palm: on - " : "palm: ") + describe() + heldNote());
}

void palmBrighter() {
  if (level >= PALM_LEVELS) {
    say("palm: already at the cap - " + describe() + ". the cap (PALM_CAP in config.h) keeps the ring inside the servo rail");
    return;
  }
  level++;
  ease();
  say("palm: brighter - " + describe() + (level == PALM_LEVELS ? "  <- cap" : "") + heldNote());
}

void palmDimmer() {
  if (level == 0) { say("palm: already off"); return; }
  if (level == 1) rememberOff();
  level--;
  ease();
  say(String("palm: ") + (level ? "dimmer - " : "") + describe() + heldNote());
}

static void setLevel(int n) {                // the page: an absolute level, clamped to the cap
  if (n < 0) n = 0;
  if (n > PALM_LEVELS) n = PALM_LEVELS;
  if (n == level) { say("palm: " + describe() + heldNote()); return; }
  bool up = n > level;
  if (n == 0) rememberOff();
  level = (uint8_t)n;
  ease();
  say(String("palm: ") + (level == 0 ? "" : up ? "brighter - " : "dimmer - ") + describe() + (level == PALM_LEVELS ? "  <- cap" : "") + heldNote());
}

void palmOff() {
  level = 0; ramping = false; walkI = -1;
  dark();
  if (glow && !warning && !alert) ease();          // the SAFE glow comes back under it
}

// SAFE's dim palm glow (Toby, 2026-10-07 night: "idle lights only - accent LEDs, the few dim strip pixels, a dim palm
// glow"): PALM_GLOW_COLOUR at PALM_GLOW_LEVEL while the ring's own setting is off; the setting and the level a
// switch-on comes back at are untouched. The battery blink, the drone alert and the servo hold win over it.
static bool glowHidden = false;   // (2026-10-21, Toby: "in the safe mode the light is on dim white and I can't turn it off") RING OFF hid it: dark until SAFE is left
void palmGlow(bool on) {
  if (!on) glowHidden = false;
  else if (glowHidden) on = false;
  if (on == glow) return;
  glow = on;
  if (!warning && !alert && walkI < 0) ease();
}
bool palmGlowing() { return glow && level == 0 && !held && !warning && !alert; }
void palmGlowHide() {
  if (!glow) return;
  glowHidden = true; glow = false;
  if (!warning && !alert && walkI < 0) ease();
  Bench.println("palm: the SAFE glow off (RING OFF) - it comes back the next time the switch is in SAFE");
}

void palmWarning(bool on) {
  if (on == warning) return;
  warning = on;
  if (warning) {
    ramping = false;
    if (walkI >= 0) { walkI = -1; Bench.println("palm: chain test cancelled - battery low"); }
    blinkT0 = millis(); blinkShown = -1;     // the next poll writes the first "on"
  } else {
    dark();
    ease();                                  // back to whatever was set (nothing, if the level is 0)
  }
}

// Toby's rules (2026-09-26): double-crunch with the other fingers closed = off, whatever is showing;
// with the hand open: off -> on white, white -> red, red -> white; any other colour is left alone.
String palmGesture(bool fist) {
  if (alert) { palmAlert(false); say("palm: drone battery alert acknowledged by the thumb crunches - it comes back if the cell is still low after a reconnect"); return lastMsg; }   // Toby: "if I triple tap the red it should go off"
  if (fist) {
    if (level == 0) say("palm: thumb gesture with a fist - already off");
    else setLevel(0);
  }
  else if (level == 0)        { colour = C_WHITE; level = lastOn; ease(); say("palm: on - " + describe() + heldNote()); }
  else if (colour == C_WHITE) setColour(C_RED);
  else if (colour == C_RED)   setColour(C_WHITE);
  else say(String("palm: thumb gesture - the ring is ") + COLOURS[colour].name + ", only white and red swap (with a fist it switches off)");
  return lastMsg;
}

// A finger dip (2026-09-29): pinky = brighter, index = dimmer, one tenth of the cap each. Only while
// on; down stops at 1 - off is the closed hand + the thumb crunches, and a switch-on comes back here.
String palmDip(bool up) {
  if (level == 0) { say(String("palm: ") + (up ? "pinky" : "index") + " dip while off - nothing to change (the thumb crunches switch it on)"); return lastMsg; }
  if (up) palmBrighter();
  else if (level <= 1) say("palm: index dip - already at 1/" + String(PALM_LEVELS) + ", the lowest a dip goes (a closed hand + the thumb crunches switch it off)");
  else palmDimmer();
  return lastMsg;
}

void palmAlert(bool on) {
  if (on == alert) return;
  alert = on;
  if (alert) { ramping = false; alertT0 = millis(); alertShown = -1; Bench.println("palm: DRONE BATTERY LOW - blinking red"); }
  else { dark(); ease(); Bench.println("palm: drone battery alert over - back to " + describe()); }
}

void palmWalk() {
  if (held) { say("palm: chain test refused - the ring is dark while the servo is attached; try again once it has let go"); return; }
  if (warning) { say("palm: chain test refused - the battery is low and the ring is blinking the warning"); return; }
  walkI = 0; walkT0 = millis(); ramping = false;
  showWalk();
  say("palm: chain test - one dim white pixel at a time, " + String(WALK_STEPS) + " steps of " + String(PALM_WALK_MS) + " ms, in chain order from the pixel wired to GPIO " +
      String(PIN_PALM_LED) + ". watch for the last one that lights - the chain is broken right after it");
  Bench.println("palm test: pixel 0");
}

void palmHold(bool h) {
  if (h == held) return;
  held = h;
  if (held) {
    ramping = false;
    if (walkI >= 0) { walkI = -1; Bench.println("palm: chain test cancelled - the servo attached"); }
    dark();                                  // instantly - before the first servo pulse, not eased
    if (level) Bench.printf("palm: ring dark while the servo is attached (was %s)\n", describe().c_str());
  } else if (warning) {
    blinkShown = -1;                         // the battery blink picks up again on the next poll
  } else if (level || glow) {
    ease();
    Bench.printf("palm: ring back - %s\n", describe().c_str());
  }
}

String palmStatusLine() {
  float white[3] = { PALM_CAP, PALM_CAP, PALM_CAP };
  char cap[64];
  snprintf(cap, sizeof cap, "cap %u %% of full (~%.2f A white)", pctOfFull(PALM_LEVELS), milliampsOf(white) / 1000.0f);
  String s = "palm: GPIO " + String(PIN_PALM_LED) + ", " + String(PALM_COUNT) + " px, " + describe();
  if (held) s += " - dark while the servo is attached";
  if (warning) s += " - BATTERY LOW, blinking white";
  if (walkI >= 0) s += " - chain test running, pixel " + String(walkI);
  s += String(", ") + cap + ", keys l . , (or the sim page)";
  return s;
}
void palmStatus() { Bench.println(palmStatusLine()); }

// ---- JSON for the page ----
static String stateFields() {                // shared by the feed and /ring
  float w[3]; wanted(w, true);
  const Colour& c = COLOURS[colour];
  char b[144];
  snprintf(b, sizeof b, "\"i\":%u,\"lvl\":%u,\"max\":%u,\"held\":%d,\"ma\":%u,\"rgb\":[%u,%u,%u],\"test\":%d,\"low\":%d",
           (unsigned)colour, (unsigned)level, (unsigned)PALM_LEVELS, held ? 1 : 0, (unsigned)(milliampsOf(w) + 0.5f),
           (unsigned)c.r, (unsigned)c.g, (unsigned)c.b, (int)walkI, warning ? 1 : 0);
  return b;
}
String palmJson() { return "{" + stateFields() + "}"; }

static String queryValue(const String& q, const char* name) {   // "c=red&lvl=5" -> the raw value of name, "" if absent
  String key = String(name) + "=";
  int p = 0;
  while (p <= (int)q.length()) {
    int e = q.indexOf('&', p); if (e < 0) e = q.length();
    String part = q.substring(p, e);
    if (part.startsWith(key)) return part.substring(key.length());
    p = e + 1;
  }
  return String();
}

String palmSet(const String& q) {
  lastMsg = "";
  String c = queryValue(q, "c");
  if (c.length()) {
    int i = -1;
    if (isDigit(c[0])) i = c.toInt();
    else for (uint8_t k = 0; k < N_COLOURS; k++) if (c.equalsIgnoreCase(COLOURS[k].name)) i = k;
    if (i >= 0 && i < (int)N_COLOURS) setColour((uint8_t)i);
    else say("palm: no such colour - the page shows the ones there are");
  }
  String lvl = queryValue(q, "lvl");
  if (lvl.length())                 setLevel(lvl.toInt());
  if (queryValue(q, "up") == "1")   palmBrighter();
  if (queryValue(q, "down") == "1") palmDimmer();
  if (queryValue(q, "off") == "1")  setLevel(0);
  if (queryValue(q, "walk") == "1") palmWalk();
  String s = "{" + stateFields() + ",\"cap\":" + String(pctOfFull(PALM_LEVELS)) + ",\"name\":\"" + COLOURS[colour].name + "\",\"colours\":[";
  for (uint8_t k = 0; k < N_COLOURS; k++) {
    const Colour& x = COLOURS[k];
    s += String(k ? ",[\"" : "[\"") + x.name + "\"," + String(x.r) + "," + String(x.g) + "," + String(x.b) + "]";
  }
  s += "],\"msg\":\"" + lastMsg + "\"}";
  return s;
}
void palmAlertRate(uint16_t ms) { alertMs = ms < 100 ? 100 : ms; }

// ---- added for the gauntlet (bench test, the display link) ----
uint8_t palmColourIndex() { return colour; }
uint8_t palmColourCount() { return N_COLOURS; }
uint8_t palmLevel()       { return level; }
float   palmMilliampsNow() { return ring.milliamps(); }
void palmSolidRamp(uint8_t r, uint8_t g, uint8_t b, std::function<bool()> tick) {   // the bench test: a colour at the cap, ramped up over LED_RAMP_MS
  ramping = false; walkI = -1;
  const int steps = 10;
  for (int s = 1; s <= steps; s++) {
    float k = (s / (float)steps) * (PALM_CAP / 255.0f);
    float v[3] = { r * k, g * k, b * k };
    put(v);
    if (!tick()) return;
    halWdtFeed(); delay(LED_RAMP_MS / steps);
  }
}
void palmWalkBlocking(uint16_t msPerPixel, std::function<bool()> tick) {   // the bench test: one dot end to end, blocking, tick false = stop
  ramping = false; walkI = -1;
  for (uint16_t i = 0; i < PALM_COUNT; i++) {
    ring.clear(); ring.setPixelColor(i, PALM_WALK_LEVEL, PALM_WALK_LEVEL, PALM_WALK_LEVEL); ring.show();
    uint32_t t0 = millis();
    while (millis() - t0 < msPerPixel) { if (!tick()) { dark(); return; } halWdtFeed(); delay(5); }
  }
  dark();
}

// ---- the puck link's $G (GauntletLink.h): the ring as it shows, as it is set, and why it differs ----
static_assert(N_COLOURS == glink::PALETTE_N, "palm.cpp COLOURS[] and GauntletLink.h PALETTE_NAMES[] must be the same list");
static uint32_t rgbOf(float r, float g, float b) {
  auto c = [](float v) { return (uint32_t)(v < 0 ? 0 : v > 255 ? 255 : (uint32_t)(v + 0.5f)); };
  return (c(r) << 16) | (c(g) << 8) | c(b);
}
uint32_t palmShownRgb() { return rgbOf(shown[0], shown[1], shown[2]); }
uint32_t palmSetRgb()   { const Colour& c = COLOURS[colour]; return rgbOf(c.r, c.g, c.b); }
uint8_t  palmMode()     { return held ? 1 : warning ? 2 : alert ? 3 : walkI >= 0 ? 4 : 0; }
