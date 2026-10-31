#include "pucks.h"
#include "config.h"
#include <math.h>
#include "hal.h"
#include "servos.h"
#include "power.h"
#include "modesw.h"
#include "fly.h"
#include <GauntletBenchIO.h>
#include <esp_system.h>   // esp_reset_reason: a real power-up starts the heat model cold

// One state per group: the group is the unit the hardware can switch (one D4184 per group, design 04 section 5).
struct Group { bool on = false; uint32_t onSince = 0, offSince = 0; };
static Group    groups[PUCK_GROUPS];
static bool     branch = false;                   // "the bank is live" = at least one group on; no pin of its own
static uint32_t branchUpMs = 0, bursts = 0;
static char     refusal[120] = "";
// duty: on-time per second over the last PUCK_DUTY_WINDOW_MS, one bucket a second, per group
static constexpr uint8_t BUCKETS = PUCK_DUTY_WINDOW_MS / 1000;
static uint16_t dutyMs[PUCK_GROUPS][BUCKETS];
static uint8_t  bucket = 0; static uint32_t bucketMs = 0, lastPollMs = 0;

// The plate model (config PLATE_*): body 0 = the palm plate (groups 1 and 2), body 1 = a finger tab (group 3, one LED each),
// body 2 = the LED-driver bank (2026-10-08: every lit driver's own DRIVER_HEAT_W). A group starts only if its body AND the
// bank are under PLATE_START_MAX_C; a burst ends when either reaches PLATE_CAP_C. After any boot every body starts at the
// cap (PLATE_START_HOT): a reboot is not a cool-down.
// (2026-10-20) one body per LED group since Toby fitted aluminium behind each LED: 0 = the palm 20 mm LEDs' plates (group 1),
// 1 = the palm 10 mm LEDs' plates (group 2), 2 = a finger's stack (group 3, one LED each, the two fingers alike), 3 = the bank.
static constexpr uint8_t BODIES = 4, BANK = 3;
static float    bodyC[BODIES] = { PLATE_AMBIENT_C, PLATE_AMBIENT_C, PLATE_AMBIENT_C, PLATE_AMBIENT_C };
static uint32_t heatCuts = 0;
static uint8_t  bodyOf(uint8_t g) { return g < 3 ? g : 0; }
static const char* bodyName(uint8_t b) { return b == BANK ? "the LED-driver bank" : b == 2 ? "a finger's aluminium" : b == 1 ? "the palm 10 mm LEDs' aluminium" : "the palm 20 mm LEDs' aluminium"; }
static const float BODY_C[BODIES] = { PALM20_J_PER_K, PALM10_J_PER_K, FINGER_J_PER_K, BANK_J_PER_K }, BODY_R[BODIES] = { PALM20_K_PER_W, PALM10_K_PER_W, FINGER_K_PER_W, BANK_K_PER_W };
// Kept across a reset in RTC memory (an OTA push, a brownout, the watchdog, the deep sleep): a reboot is not a cool-down.
static constexpr uint32_t HEAT_RTC_MAGIC = 0x48454154;   // "HEAT"
RTC_NOINIT_ATTR static uint32_t rtcHeatMagic;
RTC_NOINIT_ATTR static float    rtcBodyC[BODIES];
static void thermalStep(float dtS) {
  float p[BODIES] = { 0, 0, 0, 0 };
  for (uint8_t g = 0; g < PUCK_GROUPS; g++) if (groups[g].on) { p[bodyOf(g)] += PUCK_GROUP_LEDS[g] * LED_HEAT_W; p[BANK] += PUCK_GROUP_DRIVERS[g] * DRIVER_HEAT_W; }
  for (uint8_t b = 0; b < BODIES; b++) { bodyC[b] += dtS * (p[b] - (bodyC[b] - PLATE_AMBIENT_C) / BODY_R[b]) / BODY_C[b]; rtcBodyC[b] = bodyC[b]; }
  rtcHeatMagic = HEAT_RTC_MAGIC;
}
// At boot: a real power-up starts cold; any other reset takes the model back from RTC memory (or, if that is not
// there, starts at the cap - unknown is hot).
void pucksThermalBoot(bool powerOn) {
  bool kept = !powerOn && rtcHeatMagic == HEAT_RTC_MAGIC;
  for (uint8_t b = 0; b < BODIES; b++) {
    float v = rtcBodyC[b];
    bool sane = v == v && v >= PLATE_AMBIENT_C - 1 && v <= PLATE_CAP_C + 20;   // NaN fails v == v
    bodyC[b] = powerOn ? PLATE_AMBIENT_C : (kept && sane ? v : PLATE_CAP_C);
    rtcBodyC[b] = bodyC[b];
  }
  rtcHeatMagic = HEAT_RTC_MAGIC;
  Bench.printf("pucks: heat model %s - palm 20 mm %.0f C, palm 10 mm %.0f C, finger %.0f C, driver bank %.0f C (start limit %.0f, cap %.0f)\n",
               powerOn ? "cold (a real power-up)" : kept ? "carried over the reset" : "at the cap (a reset with nothing kept)", bodyC[0], bodyC[1], bodyC[2], bodyC[BANK], PLATE_START_MAX_C, PLATE_CAP_C);
}

float pucksPlateC()  { return bodyC[0]; }
float pucksPalm10C() { return bodyC[1]; }
float pucksTabC()    { return bodyC[2]; }
float pucksBankC()   { return bodyC[BANK]; }
uint32_t pucksHeatCuts() { return heatCuts; }
void  pucksThermalSet(float palmC, float tabC, float bankC) { bodyC[0] = bodyC[1] = palmC; bodyC[2] = tabC; bodyC[BANK] = bankC; }   // the self-test (both palm bodies)
static uint32_t onMax(uint8_t g)   { return PUCK_GROUP_IS_FINGER[g] ? PUCK_ON_MAX_MS_FINGER : PUCK_ON_MAX_MS_PALM; }
static uint32_t offMin(uint8_t g)  { return PUCK_GROUP_IS_FINGER[g] ? PUCK_OFF_MIN_MS_FINGER : PUCK_OFF_MIN_MS_PALM; }
static uint32_t dutySum(uint8_t g) { uint32_t s = 0; for (uint8_t b = 0; b < BUCKETS; b++) s += dutyMs[g][b]; return s; }
static uint8_t  groupsOn()         { uint8_t n = 0; for (uint8_t g = 0; g < PUCK_GROUPS; g++) if (groups[g].on) n++; return n; }
static bool     flightBusy() {
  return flyEnabled();
}

bool     puckGroupIsOn(uint8_t g) { return g < PUCK_GROUPS && groups[g].on; }
bool     puckIsOn(uint8_t i)      { return i < PUCK_COUNT && groups[PUCK_GROUP_OF[i]].on; }
uint8_t  pucksOnCount()           { uint8_t n = 0; for (uint8_t i = 0; i < PUCK_COUNT; i++) if (puckIsOn(i)) n++; return n; }
bool     pucksBranchOn()          { return branch; }
uint32_t pucksBursts()            { return bursts; }
const char* pucksLastRefusal()    { return refusal; }

const char* pucksBlockText() {
  static char b[96];
  if (!servosReady()) return "PCA9685 absent";
  if (!modeAllowsShow()) { snprintf(b, sizeof b, "mode %s", modeNowName()); return b; }
  if (!powerShowOk()) { snprintf(b, sizeof b, "%s (show floor %.1f V)", powerBlockText(), PUCK_SHOW_FLOOR_V); return b; }
  if (servosBusy()) return "a cascade is running (the servo stall + 4.8 A would brown the pack)";
  if (flightBusy()) return "the drone is armed (a puck burst would brown the ELRS module out)";
  return "";
}

static void branchSet(bool on) {
  if (on == branch) return;
  branch = on; branchUpMs = millis();
  Bench.printf("pucks: bank %s\n", on ? "live" : "idle");
}

void pucksBegin() {
  // The three D4184 modules hold their gates low by their own pull-downs, and OE holds every PCA channel low
  // until firmware runs - every group is off from power-on. Nothing to drive here but the state.
  for (uint8_t g = 0; g < PUCK_GROUPS; g++) { groups[g] = Group(); for (uint8_t b = 0; b < BUCKETS; b++) dutyMs[g][b] = 0; }
  bucketMs = millis();
  pucksThermalBoot(esp_reset_reason() == ESP_RST_POWERON);
  Bench.printf("pucks: 7 BuckPucks in 3 groups, a D4184 on each group's Vin- - ch %u %s, ch %u %s, ch %u %s; on/off only; bursts <= %lu s (palm) / %lu s (fingers), rest >= %lu / %lu s, duty <= %u %%, show floor %.1f V\n",
               PCA_CH_PUCK_GROUP[0], PUCK_GROUP_NAME[0], PCA_CH_PUCK_GROUP[1], PUCK_GROUP_NAME[1], PCA_CH_PUCK_GROUP[2], PUCK_GROUP_NAME[2],
               (unsigned long)(PUCK_ON_MAX_MS_PALM / 1000), (unsigned long)(PUCK_ON_MAX_MS_FINGER / 1000),
               (unsigned long)(PUCK_OFF_MIN_MS_PALM / 1000), (unsigned long)(PUCK_OFF_MIN_MS_FINGER / 1000), (unsigned)(PUCK_DUTY_MAX * 100 + 0.5f), PUCK_SHOW_FLOOR_V);
}

static void off(uint8_t g, const char* why) {
  if (!groups[g].on) return;
  groups[g].on = false; groups[g].offSince = millis();
  pcaSetDigital(PCA_CH_PUCK_GROUP[g], false);
  Bench.printf("pucks: group %u (%s) off after %lu ms (%s)\n", g + 1, PUCK_GROUP_NAME[g], (unsigned long)(millis() - groups[g].onSince), why);
  if (!groupsOn()) { branchSet(false); if (!servosBusy() && !pcaDigitalOnCount()) servosPower(false); }   // never under a laser that is still on
}

static bool refusedBattery = false;   // (2026-10-21) the last refusal was the battery (the panel answers RES_BATTERY)
bool pucksRefusedForBattery() { return refusedBattery; }
bool puckGroupOn(uint8_t g, const char* why) {
  refusedBattery = false;
  if (g >= PUCK_GROUPS) return false;
  uint32_t now = millis();
  const char* block = pucksBlockText();
  if (block[0]) { refusedBattery = !powerShowOk(); snprintf(refusal, sizeof refusal, "group %u refused: %s", g + 1, block); Bench.printf("pucks: %s\n", refusal); return false; }
  if (groups[g].on) return true;
  {   // (2026-10-21) the start floor scales with the load that would then be on (the sag grows with it): one bar for "everything"
    uint8_t leds = PUCK_GROUP_DRIVERS[g]; for (uint8_t k = 0; k < PUCK_GROUPS; k++) if (groups[k].on) leds += PUCK_GROUP_DRIVERS[k];
    float floorV = PUCK_SHOW_FLOOR_V + (LIGHT_FULL_FLOOR_V - PUCK_SHOW_FLOOR_V) * leds / (float)PUCK_COUNT;
    if (powerVolts() < floorV) {
      refusedBattery = true;
      snprintf(refusal, sizeof refusal, "group %u refused: the battery reads %.2f V - %u LEDs on need %.2f V (all seven %.1f V)", g + 1, powerVolts(), leds, floorV, LIGHT_FULL_FLOOR_V);
      Bench.printf("pucks: %s\n", refusal); return false;
    }
  }
  if (groups[g].offSince && now - groups[g].offSince < offMin(g)) {
    snprintf(refusal, sizeof refusal, "group %u refused: cooling, %lu s to go", g + 1, (unsigned long)((offMin(g) - (now - groups[g].offSince)) / 1000));
    Bench.printf("pucks: %s\n", refusal); return false;
  }
  for (uint8_t b : { bodyOf(g), BANK }) if (bodyC[b] >= PLATE_START_MAX_C) {
    snprintf(refusal, sizeof refusal, "group %u refused: %s is about %.0f C (no burst above %.0f, the cap is %.0f)", g + 1, bodyName(b), bodyC[b], PLATE_START_MAX_C, PLATE_CAP_C);
    Bench.printf("pucks: %s\n", refusal); return false;
  }
  if (dutySum(g) > PUCK_DUTY_MAX * PUCK_DUTY_WINDOW_MS + PUCK_DUTY_SLACK_MS) {   // the time already on (2026-10-20: a whole backstop burst no longer fits the window - the heat model ends a burst)
    snprintf(refusal, sizeof refusal, "group %u refused: %lu s on in the last %lu s is the duty limit", g + 1, (unsigned long)(dutySum(g) / 1000), (unsigned long)(PUCK_DUTY_WINDOW_MS / 1000));
    Bench.printf("pucks: %s\n", refusal); return false;
  }
  if (!branch) branchSet(true);
  groups[g].on = true; groups[g].onSince = millis();
  pcaSetDigital(PCA_CH_PUCK_GROUP[g], true);
  bursts++; refusal[0] = 0;
  Bench.printf("pucks: group %u (%s) ON (%s) - at most %lu s\n", g + 1, PUCK_GROUP_NAME[g], why, (unsigned long)(onMax(g) / 1000));
  return true;
}

void puckGroupOff(uint8_t g) { if (g < PUCK_GROUPS) off(g, "asked"); }
void pucksAllOff(const char* why) { for (uint8_t g = 0; g < PUCK_GROUPS; g++) off(g, why); branchSet(false); }

bool pucksShow(const char* name) {
  bool any = false, all = strcmp(name, "all") == 0;
  for (uint8_t g = 0; g < PUCK_GROUPS; g++) {
    bool want = all || (strcmp(name, "repulsor") == 0 && !PUCK_GROUP_IS_FINGER[g]) || (strcmp(name, "fingers") == 0 && PUCK_GROUP_IS_FINGER[g]);
    if (want && puckGroupOn(g, name)) any = true;
  }
  return any;
}

void pucksPoll() {
  uint32_t now = millis();
  if (now - bucketMs >= 1000) {                        // a new one-second duty bucket
    bucketMs = now; bucket = (bucket + 1) % BUCKETS;
    for (uint8_t g = 0; g < PUCK_GROUPS; g++) dutyMs[g][bucket] = 0;
  }
  uint32_t dt = lastPollMs ? now - lastPollMs : 0; lastPollMs = now;
  thermalStep(dt / 1000.0f);
  for (uint8_t g = 0; g < PUCK_GROUPS; g++) {
    if (!groups[g].on) continue;
    dutyMs[g][bucket] = (uint16_t)min<uint32_t>(dutyMs[g][bucket] + dt, 1000);
    if (now - groups[g].onSince >= onMax(g)) off(g, "burst limit");
    else if (bodyC[bodyOf(g)] >= PLATE_CAP_C || bodyC[BANK] >= PLATE_CAP_C) {
      heatCuts++; char w[80]; snprintf(w, sizeof w, "%s reached the %.0f C cap (model)", bodyName(bodyC[bodyOf(g)] >= PLATE_CAP_C ? bodyOf(g) : BANK), PLATE_CAP_C); off(g, w);
    }
  }
  if (groupsOn() && pucksBlockText()[0]) pucksAllOff(pucksBlockText());    // the world changed under a burst: cascade, arm, mode, battery
  if (branch && !groupsOn() && now - branchUpMs > 1000) branchSet(false);  // belt and braces: never "live" with nothing on
}

String pucksJson() {
  uint32_t now = millis();
  String s = "{\"branch\":" + String(branch ? 1 : 0) + ",\"on\":[";
  for (uint8_t g = 0; g < PUCK_GROUPS; g++) { if (g) s += ","; s += groups[g].on ? "1" : "0"; }
  s += "],\"ch\":[";
  for (uint8_t g = 0; g < PUCK_GROUPS; g++) { if (g) s += ","; s += String(PCA_CH_PUCK_GROUP[g]); }
  s += "],\"names\":[";
  for (uint8_t g = 0; g < PUCK_GROUPS; g++) { if (g) s += ","; s += "\"" + String(PUCK_GROUP_NAME[g]) + "\""; }
  s += "],\"lit\":[";
  for (uint8_t i = 0; i < PUCK_COUNT; i++) { if (i) s += ","; s += puckIsOn(i) ? "1" : "0"; }
  s += "],\"cool\":[";
  for (uint8_t g = 0; g < PUCK_GROUPS; g++) { if (g) s += ","; uint32_t c = (groups[g].offSince && now - groups[g].offSince < offMin(g)) ? offMin(g) - (now - groups[g].offSince) : 0; s += String(c); }
  s += "],\"duty\":[";
  for (uint8_t g = 0; g < PUCK_GROUPS; g++) { if (g) s += ","; s += String(dutySum(g) / 1000); }
  s += "],\"plate\":" + String(bodyC[0], 1) + ",\"palm10\":" + String(bodyC[1], 1) + ",\"tab\":" + String(bodyC[2], 1) + ",\"bank\":" + String(bodyC[BANK], 1) + ",\"heatcuts\":" + String(heatCuts);
  s += ",\"bursts\":" + String(bursts) + ",\"block\":\"" + pucksBlockText() + "\",\"last\":\"" + refusal + "\"}";
  return s;
}
String pucksStatusLine() {
  String s = "pucks: bank " + String(branch ? "LIVE" : "idle") + ", groups on:";
  bool any = false; for (uint8_t g = 0; g < PUCK_GROUPS; g++) if (groups[g].on) { s += " " + String(g + 1) + " (" + PUCK_GROUP_NAME[g] + ")"; any = true; }
  if (!any) s += " none";
  s += ", palm plate ~" + String(bodyC[0], 0) + " C, finger tab ~" + String(bodyC[1], 0) + " C, driver bank ~" + String(bodyC[BANK], 0) + " C (model, cap " + String(PLATE_CAP_C, 0) + ")";
  s += ", " + String(bursts) + " bursts; " + (pucksBlockText()[0] ? String("blocked: ") + pucksBlockText() : String("allowed")) + (refusal[0] ? String("; last refusal: ") + refusal : String(""));
  return s;
}
