// Host test of lib/GauntletLink/src/GauntletLink.h - the arm <-> puck protocol - on a PC, no board needed:
//     powershell -ExecutionPolicy Bypass -File lib/GauntletLink/test/run.ps1
// Encodes with the header's own encoders (the ones Ironman/s3link.cpp sends with) and decodes with the same
// header's parser (the one the puck copies), then attacks it: every single-bit error, wrong protocol versions,
// frames cut at every length, frames too long, out-of-range fields, a million bytes of garbage, and good frames
// buried in noise. Exit code 0 = everything passed.
#include "../src/GauntletLink.h"
#include <stdio.h>
#include <string.h>
#include <string>
#include <vector>

using namespace glink;
static int passed = 0, failed = 0;
static void check(bool ok, const char* what) { if (ok) passed++; else { failed++; printf("FAIL  %s\n", what); } }

static uint32_t rng = 12345;
static uint32_t rnd() { rng ^= rng << 13; rng ^= rng >> 17; rng ^= rng << 5; return rng; }

static std::string frameOf(const char* body) { char line[MAX_LINE + 8]; size_t n = frame(line, sizeof line, body); return std::string(line, n); }
static std::string liveLine(const Live& g) { char b[MAX_LINE]; size_t n = liveBody(b, sizeof b, g); return n ? frameOf(b) : std::string(); }
static std::string statusLine(const Status& s) { char b[MAX_LINE]; size_t n = statusBody(b, sizeof b, s); return n ? frameOf(b) : std::string(); }

// feeds a byte string; returns how many frames came out whole, and the last one's fields
struct Got { int frames = 0; std::vector<std::string> bodies; };
static Got feedAll(Parser& p, const std::string& s) { Got g; for (char c : s) if (p.feed(c)) { g.frames++; g.bodies.push_back(p.body()); } return g; }
static Result parseLive(Parser& p, Live& g) { const char* f[MAX_FIELDS]; uint8_t n = p.fields(f, MAX_FIELDS); return liveParse(f, n, g); }

static Live sampleLive() {
  Live g; g.seq = 4242; g.t = 123456789; g.fq[0] = 9239; g.fq[1] = 0; g.fq[2] = 3827; g.fq[3] = 0; g.hq[0] = 7071; g.hq[1] = -7071; g.hq[2] = 0; g.hq[3] = 0; g.imu = 7;
  for (int i = 0; i < 5; i++) { g.curl[i] = (int16_t)(i * 200 - 1 + (i == 0)); g.fmv[i] = (uint16_t)(3300 - i * 300); }
  for (int i = 0; i < 6; i++) { g.servo[i] = (int16_t)(i == 5 ? -1 : i * 200); }
  g.drv = 0x15; g.plates = 5;
  g.ring = 0x4D4D4D; g.ringset = 0x96CDFF; g.ringlvl = 7; g.ringmode = 1; g.strip = 0xFF4600; g.striplvl = 77; g.striplit = 100; g.strippos = 0;
  g.groups = 5; g.accent = 1; g.lasers = 3; g.lidar = 1234; g.mode = 1; g.gev = GEV_PLATE; g.gn = 65535; g.hand = 2; g.mv = 7861; g.bstate = 2; g.ma = 1720; g.outs = 0x25;
  return g;
}
static bool sameLive(const Live& a, const Live& b) {
  if (a.seq != b.seq || a.t != b.t || a.imu != b.imu || a.drv != b.drv || a.plates != b.plates || a.ring != b.ring || a.ringset != b.ringset || a.ringlvl != b.ringlvl || a.ringmode != b.ringmode) return false;
  if (a.strip != b.strip || a.striplvl != b.striplvl || a.striplit != b.striplit || a.strippos != b.strippos || a.groups != b.groups || a.accent != b.accent || a.lasers != b.lasers || a.lidar != b.lidar) return false;
  if (a.mode != b.mode || a.gev != b.gev || a.gn != b.gn || a.hand != b.hand || a.mv != b.mv || a.bstate != b.bstate || a.ma != b.ma || a.outs != b.outs) return false;
  for (int i = 0; i < 4; i++) if (a.fq[i] != b.fq[i] || a.hq[i] != b.hq[i]) return false;
  for (int i = 0; i < 5; i++) if (a.curl[i] != b.curl[i] || a.fmv[i] != b.fmv[i]) return false;
  for (int i = 0; i < 6; i++) if (a.servo[i] != b.servo[i]) return false;
  return true;
}
static Live widestLive() {   // every field at its widest text
  Live g; g.seq = 65535; g.t = 4294967295u; for (int i = 0; i < 4; i++) { g.fq[i] = -10000; g.hq[i] = -10000; } g.imu = 7;
  for (int i = 0; i < 5; i++) { g.curl[i] = 1000; g.fmv[i] = 3300; } for (int i = 0; i < 6; i++) g.servo[i] = 1000; g.drv = 63; g.plates = 5;
  g.ring = g.ringset = g.strip = 0xFFFFFF; g.ringlvl = 10; g.ringmode = 4; g.striplvl = 255; g.striplit = 100; g.strippos = 99;
  g.groups = 7; g.accent = 1; g.lasers = 3; g.lidar = 40000; g.mode = 2; g.gev = GEV_COUNT - 1; g.gn = 65535; g.hand = 2; g.mv = 9999; g.bstate = 6; g.ma = 9999; g.outs = 127;
  return g;
}
static Status widestStatus() { Status s; s.seq = 9999; s.mode = 2; s.mv = 65535; s.bstate = 6; s.plates = 5; s.ringi = 255; s.ringlvl = 10; s.armed = 1; s.lq = 100; s.cv100 = 65535; strcpy(s.fcmode, "ANGL*ABCDEF"); s.lidar = 65535; s.laser = 3; s.pucks = 255; s.flags = 255; s.page = 4; return s; }

int main() {
  // ---- the CRC itself: CRC-8/DVB-S2's published check value
  check(crcOf("123456789", 9) == 0x29B1, "CRC-16/CCITT-FALSE of \"123456789\" is 0x29B1");

  // ---- $G round trip
  { Live g = sampleLive(); std::string L = liveLine(g); Parser p; Got got = feedAll(p, L); Live r; bool ok = got.frames == 1 && parseLive(p, r) == Result::Ok && sameLive(g, r);
    check(ok, "$G round trip: every field comes back as sent"); check(L.size() <= G_MAX_LEN, "$G sample fits G_MAX_LEN"); }
  { Live g = widestLive(); std::string L = liveLine(g); Parser p; Got got = feedAll(p, L); Live r; bool ok = got.frames == 1 && parseLive(p, r) == Result::Ok && sameLive(g, r);
    check(ok, "$G widest round trip"); check(L.size() <= G_MAX_LEN, "$G widest line fits G_MAX_LEN"); check(L.size() < MAX_LINE, "$G widest line fits MAX_LINE");
    printf("      $G widest = %zu bytes (G_MAX_LEN %zu, MAX_LINE %zu)\n", L.size(), G_MAX_LEN, MAX_LINE); }
  { Live g; std::string L = liveLine(g); Parser p; Got got = feedAll(p, L); Live r; check(got.frames == 1 && parseLive(p, r) == Result::Ok && sameLive(g, r), "$G defaults round trip (no sensors: -1 curls, -1 servos, -1 mA)"); }

  // ---- $S round trip and size
  { Status s = widestStatus(); std::string L = statusLine(s); Parser p; Got got = feedAll(p, L); const char* f[MAX_FIELDS]; uint8_t n = got.frames ? p.fields(f, MAX_FIELDS) : 0; Status r;
    bool ok = got.frames == 1 && statusParse(f, n, r) == Result::Ok && r.seq == s.seq && r.mv == s.mv && r.plates == 5 && strcmp(r.fcmode, s.fcmode) == 0 && r.page == 4;
    check(ok, "$S widest round trip (the FC mode's '*' inside a field)"); check(L.size() <= S_MAX_LEN, "$S widest fits S_MAX_LEN");
    printf("      $S widest = %zu bytes (S_MAX_LEN %zu)\n", L.size(), S_MAX_LEN); }

  // ---- $H round trip, and the puck's version is reported
  { Heartbeat h; h.seq = 7; h.uptimeS = 3600; h.page = 1; h.mlx = 1; h.oled = 0; h.fps = 30; h.heap = 180000; char b[MAX_LINE]; heartbeatBody(b, sizeof b, h); std::string L = frameOf(b);
    Parser p; Got got = feedAll(p, L); const char* f[MAX_FIELDS]; uint8_t n = got.frames ? p.fields(f, MAX_FIELDS) : 0; Heartbeat r; long theirs = 0;
    check(got.frames == 1 && heartbeatParse(f, n, r, &theirs) == Result::Ok && theirs == VERSION && r.heap == 180000 && r.fps == 30, "$H round trip with the version"); }

  // ---- $E (2026-10-07 night): round trip, an older puck's parsers ignore it, an unknown action is still a line, bad values refused
  { Event e; e.seq = 65535; e.act = ACT_THERMAL_ON; e.res = RES_BLOCKED; e.mode = 0; e.need = 1; char b[MAX_LINE]; size_t bn = eventBody(b, sizeof b, e); std::string L = frameOf(b);
    Parser p; Got got = feedAll(p, L); const char* f[MAX_FIELDS]; uint8_t n = got.frames ? p.fields(f, MAX_FIELDS) : 0; Event r;
    bool rt = bn && got.frames == 1 && eventParse(f, n, r) == Result::Ok && r.seq == 65535 && r.act == ACT_THERMAL_ON && r.res == RES_BLOCKED && r.mode == 0 && r.need == 1;
    check(rt, "$E round trip (thermal on, BLOCKED in SAFE, needs SHOW)");
    Status s; Live g; Heartbeat h; long th = 0;
    check(statusParse(f, n, s) == Result::NotThisType && liveParse(f, n, g) == Result::NotThisType && heartbeatParse(f, n, h, &th) == Result::NotThisType,
          "$E is NotThisType to the $S / $G / $H parsers (a puck built before it ignores the line)");
    check(L.size() <= 32, "$E fits 32 bytes"); }
  { std::string L = frameOf("E,1,200,0,2,2,2"); Parser p; feedAll(p, L); const char* f[MAX_FIELDS]; uint8_t n = p.fields(f, MAX_FIELDS); Event r;
    check(eventParse(f, n, r) == Result::Ok && r.act == 200 && std::string(actName(r.act)) == "gesture", "$E with an action number the puck does not know is still a line"); }
  { const char* bad[] = { "E,1,3,4,1,1,2", "E,1,3,0,3,1,2", "E,1,3,0,1,1", "E,1,3,0,1,1,9", "E,70000,3,0,1,1,2" }; int ok = 0;
    for (const char* bb : bad) { std::string L = frameOf(bb); Parser p; feedAll(p, L); const char* f[MAX_FIELDS]; uint8_t n = p.fields(f, MAX_FIELDS); Event r; if (eventParse(f, n, r) != Result::Ok) ok++; }
    check(ok == 5, "$E: a result or mode out of range, a missing field, another version and a seq past 65535 are refused"); }
  // ---- $C (2026-10-20, the puck's control panel): round trip, the other parsers ignore it, bad values refused
  { Command c; c.seq = 65535; c.act = ACT_PLATE; c.a = 4; c.b = 2; char b[MAX_LINE]; size_t bn = commandBody(b, sizeof b, c); std::string L = frameOf(b);
    Parser p; Got got = feedAll(p, L); const char* f[MAX_FIELDS]; uint8_t n = got.frames ? p.fields(f, MAX_FIELDS) : 0; Command r;
    check(bn && got.frames == 1 && commandParse(f, n, r) == Result::Ok && r.seq == 65535 && r.act == ACT_PLATE && r.a == 4 && r.b == 2, "$C round trip (the hand plate, toggle)");
    Status s; Live g; Heartbeat h; Event e; long th = 0;
    check(statusParse(f, n, s) == Result::NotThisType && liveParse(f, n, g) == Result::NotThisType && heartbeatParse(f, n, h, &th) == Result::NotThisType && eventParse(f, n, e) == Result::NotThisType,
          "$C is NotThisType to the $S / $G / $H / $E parsers");
    check(L.size() <= 32, "$C fits 32 bytes");
    check(std::string(actName(ACT_ALL_OFF)) == "all off" && ACT_ALL_OFF == 17 && ACT_LIDAR == 18 && std::string(actName(ACT_LIDAR)) == "lidar", "ACT_ALL_OFF 17 and ACT_LIDAR 18 on the end of the list: the older numbers unchanged"); }
  { const char* bad[] = { "C,1,6,4", "C,1,6,4,2,9", "C,70000,6,4,2,2", "C,1,300,0,0,2", "C,1,6,70000,0,2", "C,1,6,-1,0,2" }; int ok = 0;
    for (const char* bb : bad) { std::string L = frameOf(bb); Parser p; feedAll(p, L); const char* f[MAX_FIELDS]; uint8_t n = p.fields(f, MAX_FIELDS); Command r; if (commandParse(f, n, r) != Result::Ok) ok++; }
    check(ok == 6, "$C: a missing field, another version, a seq / act / argument out of range are refused"); }
  { Status s; s.flags = (1 << FLAG_SHUTDOWN) | (1 << FLAG_THERMAL); std::string L = statusLine(s); Parser p; Got got = feedAll(p, L); const char* f[MAX_FIELDS]; uint8_t n = got.frames ? p.fields(f, MAX_FIELDS) : 0; Status r;
    check(got.frames == 1 && statusParse(f, n, r) == Result::Ok && (r.flags & (1 << FLAG_THERMAL)) && (r.flags & (1 << FLAG_SHUTDOWN)), "$S flags THERMAL (bit 6) and SHUTDOWN (bit 5) round trip"); }

  // ---- bandwidth: the worst case of both arm -> puck frames at their rates, against 115200 baud 8N1
  { double perSec = (double)G_MAX_LEN * (1000.0 / G_PERIOD_MS) + (double)S_MAX_LEN * (1000.0 / S_PERIOD_MS), line = BAUD / 10.0;
    printf("      worst case arm -> puck: %.0f bytes/s of %.0f = %.0f %% of the line\n", perSec, line, 100.0 * perSec / line);
    check(perSec <= 0.55 * line, "worst-case $G + $S under 55 % of the line"); }

  // ---- every single-bit error in a $G line is caught (never accepted)
  { std::string L = liveLine(sampleLive()); int caught = 0, total = 0;
    for (size_t i = 1; i + 1 < L.size(); i++) for (int b = 0; b < 8; b++) {
      std::string m = L; m[i] = (char)(m[i] ^ (1 << b)); if (m[i] == '\n' || m[i] == '\r' || m[i] == '$') continue;   // those re-frame the line: the cut case below
      Parser p; Got got = feedAll(p, m); Live r; total++; if (got.frames == 0 || parseLive(p, r) != Result::Ok) caught++; }
    char t[96]; snprintf(t, sizeof t, "every single-bit error in a $G line rejected (%d of %d)", caught, total); check(caught == total, t); }
  // two-bit errors in the body: CRC8 catches all bursts up to 8 bits and almost everything else; none may parse as different values
  { Live g = sampleLive(); std::string L = liveLine(g); int wrong = 0, accepted = 0;
    for (int k = 0; k < 20000; k++) { std::string m = L; size_t i = 1 + rnd() % (L.size() - 5), j = 1 + rnd() % (L.size() - 5); m[i] ^= (char)(1 << (rnd() % 7)); m[j] ^= (char)(1 << (rnd() % 7));
      if (m == L) { continue; }
      Parser p; Got got = feedAll(p, m); Live r; if (got.frames && parseLive(p, r) == Result::Ok) { accepted++; if (!sameLive(r, g)) wrong++; } }
    char t[120]; snprintf(t, sizeof t, "20000 random two-bit errors: %d accepted, %d with different values (CRC-16)", accepted, wrong); check(accepted == 0, t); printf("      %s\n", t); }

  // three-bit errors: CRC-16/CCITT catches every one at this length
  { Live g = sampleLive(); std::string L = liveLine(g); int accepted = 0;
    for (int k = 0; k < 20000; k++) { std::string m = L; for (int e = 0; e < 3; e++) { size_t i = 1 + rnd() % (L.size() - 7); m[i] ^= (char)(1 << (rnd() % 7)); }
      if (m == L) { continue; }
      Parser p; Got got = feedAll(p, m); Live r; if (got.frames && parseLive(p, r) == Result::Ok && !sameLive(r, g)) accepted++; }
    char t[100]; snprintf(t, sizeof t, "20000 random three-bit errors: %d accepted", accepted); check(accepted == 0, t); }
  // a real v1 line (two-digit CRC8) is recognised as v1, counted, never accepted
  { const char* body = "S,1,1,7800,0,2,0,5,0,0,0,-,0,0,0,0,0"; uint8_t c8 = 0; for (const char* q = body; *q; q++) c8 = crc8(c8, (uint8_t)*q); char line[128]; snprintf(line, sizeof line, "$%s*%02X\n", body, c8);
    Parser p; Got got = feedAll(p, line); check(got.frames == 0 && p.v1 == 1, "a v1 line (CRC8, two digits) counted as v1 and not accepted"); }
  // lower-case CRC digits are not the format: rejected
  { std::string L = liveLine(sampleLive()); size_t star = L.rfind('*'); for (size_t i = star + 1; i < L.size() - 1; i++) L[i] = (char)tolower((unsigned char)L[i]);
    Parser p; Got got = feedAll(p, L); bool hasLetter = false; for (size_t i = star + 1; i < L.size() - 1; i++) if (isalpha((unsigned char)L[i])) hasLetter = true;
    check(!hasLetter || got.frames == 0, "lower-case CRC digits rejected"); }
  // ---- the version: a frame from another protocol version is rejected as such, never read
  { char b[MAX_LINE]; Live g = sampleLive(); liveBody(b, sizeof b, g); std::string body(b); for (int ver : { 1, 3, 9 }) { std::string bb = body; bb.replace(2, 1, std::to_string(ver));
      std::string L = frameOf(bb.c_str()); Parser p; Got got = feedAll(p, L); Live r; char t[80]; snprintf(t, sizeof t, "$G version %d rejected as BadVersion", ver); check(got.frames == 1 && parseLive(p, r) == Result::BadVersion, t); } }
  { std::string L = frameOf("S,1,1,7800,0,2,0,5,0,0,0,-,0,0,0,0,0"); Parser p; Got got = feedAll(p, L); const char* f[MAX_FIELDS]; uint8_t n = p.fields(f, MAX_FIELDS); Status r;   // a v1 status frame
    check(got.frames == 1 && statusParse(f, n, r) == Result::BadVersion, "a v1 $S (no version field) rejected as BadVersion"); }
  { std::string L = frameOf("H,1,12,1,1,1,30,180000"); Parser p; Got got = feedAll(p, L); const char* f[MAX_FIELDS]; uint8_t n = p.fields(f, MAX_FIELDS); Heartbeat r; long theirs = 0;
    check(got.frames == 1 && heartbeatParse(f, n, r, &theirs) == Result::BadVersion && theirs == 1, "a v1 $H rejected, its version reported as 1"); }
  { std::string L = frameOf("G,2,1,2"); Parser p; feedAll(p, L); Live r; check(parseLive(p, r) == Result::BadFields, "$G with the right version but too few fields: BadFields"); }

  // ---- fields out of range or not numbers: rejected
  { char b[MAX_LINE]; Live g = sampleLive(); liveBody(b, sizeof b, g); std::string body(b);
    auto withField = [&](int idx, const char* val) { std::vector<std::string> parts; size_t s = 0, c; while ((c = body.find(',', s)) != std::string::npos) { parts.push_back(body.substr(s, c - s)); s = c + 1; } parts.push_back(body.substr(s));
      parts[idx] = val; std::string out; for (size_t i = 0; i < parts.size(); i++) { if (i) out += ","; out += parts[i]; } return out; };
    struct Bad { int idx; const char* val; const char* what; } bads[] = { { 33, "11", "ring level 11" }, { 13, "1001", "curl 1001" }, { 23, "-2", "servo -2" }, { 4, "10001", "quaternion 10001" }, { 31, "GGGGGG", "ring colour not hex" },
                                                                         { 42, "x", "lidar not a number" }, { 43, "3", "mode 3" }, { 3, "-1", "negative clock" }, { 49, "", "empty field" } };
    for (auto& bd : bads) { std::string L = frameOf(withField(bd.idx, bd.val).c_str()); Parser p; Got got = feedAll(p, L); Live r; char t[80]; snprintf(t, sizeof t, "out of range rejected: %s", bd.what); check(got.frames == 1 && parseLive(p, r) == Result::BadFields, t); } }

  // ---- truncated: a $G cut at every length, then a good one - the cut one never comes out, the good one always does
  { std::string L = liveLine(sampleLive()); int ok = 0;
    for (size_t k = 1; k < L.size(); k++) { Parser p; Got a = feedAll(p, L.substr(0, k)); Got b = feedAll(p, L); Live r; if (a.frames == 0 && b.frames == 1 && parseLive(p, r) == Result::Ok && p.truncated == 1) ok++; }
    char t[96]; snprintf(t, sizeof t, "a $G cut at each of %zu lengths: dropped (counted as truncated), the next frame read whole", L.size() - 1); check(ok == (int)L.size() - 1, t); }
  // a $G cut by a newline (a line ending early): rejected
  { std::string L = liveLine(sampleLive()); int ok = 0; for (size_t k = 2; k + 1 < L.size(); k++) { std::string m = L.substr(0, k) + "\n"; Parser p; Got g = feedAll(p, m); if (g.frames == 0) ok++; }
    check(ok == (int)L.size() - 3, "a $G ended early by a newline at every length: rejected"); }

  // ---- too long: a line past MAX_LINE is dropped and the parser recovers
  { std::string junk = "$G," + std::string(400, '1') + "\n"; Parser p; Got a = feedAll(p, junk); Got b = feedAll(p, liveLine(sampleLive())); check(a.frames == 0 && b.frames == 1 && p.tooLong == 1, "a 400-byte line dropped, the next frame read"); }

  // ---- a million bytes of garbage: nothing parses as a $G, $S or $H
  { Parser p; int parsed = 0, frames = 0; const char alpha[] = "$G,S,H*0123456789ABCDEF-\n\r,,,$*";
    for (int k = 0; k < 1000000; k++) { char c = (k & 1) ? (char)(rnd() & 0xFF) : alpha[rnd() % (sizeof alpha - 1)];
      if (p.feed(c)) { frames++; const char* f[MAX_FIELDS]; uint8_t n = p.fields(f, MAX_FIELDS); Live g; Status s; Heartbeat h; if (liveParse(f, n, g) == Result::Ok || statusParse(f, n, s) == Result::Ok || heartbeatParse(f, n, h) == Result::Ok) parsed++; } }
    char t[120]; snprintf(t, sizeof t, "1 000 000 bytes of garbage: %d accidental CRC passes, %d parsed as a frame", frames, parsed); check(parsed == 0, t); printf("      %s\n", t); }

  // ---- 2000 good frames buried in noise (noise between them, some of it starting fake frames): every good one read
  { Parser p; int got = 0, bad = 0; std::string all;
    for (int k = 0; k < 2000; k++) { Live g = sampleLive(); g.seq = (uint16_t)k; std::string L = liveLine(g); int nz = rnd() % 40; std::string noise; for (int j = 0; j < nz; j++) { char c = (char)(rnd() & 0x7F); if (c == '\n') c = 'x'; noise += c; } all += noise + L; }
    for (char c : all) if (p.feed(c)) { const char* f[MAX_FIELDS]; uint8_t n = p.fields(f, MAX_FIELDS); Live r; if (liveParse(f, n, r) == Result::Ok) { if (r.seq == got) got++; else bad++; } }
    char t[100]; snprintf(t, sizeof t, "2000 $G frames in noise: %d read in order, %d out of order", got, bad); check(got == 2000 && bad == 0, t); }

  // ---- the frame helper refuses what would not fit
  { char out[16]; check(frame(out, sizeof out, "G,2,1,2,3,4,5,6,7,8,9") == 0, "frame() refuses a buffer too small"); std::string big(300, '1'); char out2[400]; check(frame(out2, sizeof out2, big.c_str()) == 0, "frame() refuses a body longer than MAX_LINE"); }

  printf("\nglink host test: %d passed, %d failed (protocol v%u)\n", passed, failed, VERSION);
  return failed ? 1 : 0;
}
