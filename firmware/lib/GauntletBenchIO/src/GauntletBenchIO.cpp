#include "GauntletBenchIO.h"
#include <WiFi.h>
#include <esp_wifi.h>   // esp_wifi_scan_stop (wifiHold)
#include <WebServer.h>
#include <Update.h>
#include <ArduinoOTA.h>
#include <lwip/sockets.h>   // select(): does a feed socket have room before we write to it
#include <vector>
#include <Preferences.h>
#include "secrets.h"   // WIFI_SSID / WIFI_PASS (+ optional WIFI_SSID2 / WIFI_PASS2) - gitignored, see secrets.h.example

BenchStream Bench;

static constexpr uint32_t REJOIN_MS    = 30000;   // background re-kick
static constexpr uint32_t NET_SWITCH_MS = 12000;  // 0.5.2: with two networks, try the other one after this without a join

// 0.5.2: the networks to try in turn; the one that last joined goes first (NVS "benchnet")
struct BenchNet { const char* ssid; const char* pass; };
static const BenchNet NETS[] = {
  { WIFI_SSID, WIFI_PASS },
#ifdef WIFI_SSID2
  { WIFI_SSID2, WIFI_PASS2 },
#endif
#ifdef WIFI_SSID3
  { WIFI_SSID3, WIFI_PASS3 },
#endif
};
static constexpr uint8_t NET_N = sizeof(NETS) / sizeof(NETS[0]);
static uint8_t netIdx = 0;
static volatile uint16_t staReason = 0;            // 0.5.2: why the join failed (set in the WiFi event task, printed by poll(), once per try)
static bool              staReasonShown = false;
static constexpr size_t   LOG_SIZE     = 4096;    // ring of recent output
static constexpr size_t   KEY_QUEUE    = 64;      // pending keystrokes

static WebServer http(80);

static const char*           hostName   = "gauntlet";
static bool                  servicesUp = false;
static uint32_t              lastRejoin = 0;
static uint32_t              restartAt  = 0;       // 0 = none pending
static bool                  uploadOk   = false;   // set only when Update.end() succeeds
static std::function<void()> updateStartFn;

// ---------------- extra GET routes (Bench.onGet) --------------------------
// A sketch's own endpoints, e.g. HandBench's /imu JSON for the arm simulator.
// Sent with CORS * and no caching so a page served from anywhere on the LAN
// (the board itself, a laptop, a phone) can poll them.
struct ExtraRoute { const char* path; const char* type; std::function<String()> fn; };
static std::vector<ExtraRoute> extraRoutes;
static void registerExtra(const ExtraRoute& r) {
  http.on(r.path, HTTP_GET, [r] {
    http.sendHeader("Access-Control-Allow-Origin", "*");
    http.sendHeader("Cache-Control", "no-store");
    http.send(200, r.type, r.fn());
  });
}

// ---------------- live feed (Bench.liveFeed) -----------------------------
// HTTP/1.1 keep-alive, several clients, non-blocking. See the header.
static inline void keyPush(uint8_t c);            // console input queue, defined below
static String logSince(const String& fromArg, uint32_t& head);   // the output ring, defined below
static constexpr uint8_t  LIVE_SLOTS   = 4;
static constexpr uint8_t  LIVE_PAGES   = 4;       // HTML pages on the feed port (livePage(path, html) / livePageGz), served straight from flash
static constexpr uint32_t LIVE_IDLE_MS = 20000;   // drop a connection silent this long
static constexpr size_t   LIVE_MAX_REQ = 2048;    // longer than any GET we expect
static constexpr size_t   LIVE_CHUNK   = 1024;    // one write when the socket says it has room: under lwIP's low-water mark (~2.9 KB), so it never blocks
static constexpr uint8_t  LIVE_CHUNKS_PER_POLL = 6;   // and at most this many per poll per client, so a fast client cannot hog the loop either
static constexpr uint32_t LIVE_SEND_MAX_MS = 3000;    // a client that has stopped taking data for this long is dropped
// A response is queued here and drained a chunk at a time, only when the socket reports room. Before
// 0.4.3 the response was written in one go and NetworkClient::write() waits (up to a second, ten
// times) for a client that is slow to acknowledge - on a weak WiFi link one stalled page froze the
// whole loop, and with the telemetry page and the simulator open together the arm stuttered.
struct LiveSlot { NetworkClient client; String req; uint32_t lastMs = 0;
                  String out; const char* flashBody = nullptr; size_t flashLen = 0; size_t sent = 0; bool sending = false; uint32_t sendStart = 0; };
static NetworkServer*          liveServer = nullptr;
static uint16_t                livePort   = 0;
static std::function<String()> liveFn;
static const char*             liveType   = "application/json";
static LiveSlot                liveSlots[LIVE_SLOTS];
static const char*             livePageHtml = nullptr;   // served for "/" and "/arm"
struct LivePageEntry { const char* path; const char* body; size_t len; bool gz; };   // gz: the body is gzip bytes, sent with Content-Encoding: gzip (0.4.4)
static LivePageEntry           livePages[LIVE_PAGES];     // and any others, by path (HandBench: /hud)
static uint8_t                 livePageN = 0;
static uint32_t                liveRequests = 0, liveAccepts = 0;
struct LiveRoute { const char* path; const char* type; std::function<String(const String&)> fn; };
static std::vector<LiveRoute>  liveRoutes;

static String urlDecode(const String& s) {
  String o; o.reserve(s.length());
  for (size_t i = 0; i < s.length(); i++) {
    char c = s[i];
    if (c == '+') o += ' ';
    else if (c == '%' && i + 2 < s.length()) { o += (char)strtol(s.substring(i + 1, i + 3).c_str(), nullptr, 16); i += 2; }
    else o += c;
  }
  return o;
}
static String queryArg(const String& query, const char* name) {   // "a=1&c=w" -> the value of name, decoded; "" if absent
  String key = String(name) + "=";
  int p = 0;
  while (p <= (int)query.length()) {
    int e = query.indexOf('&', p); if (e < 0) e = query.length();
    String part = query.substring(p, e);
    if (part.startsWith(key)) return urlDecode(part.substring(key.length()));
    p = e + 1;
  }
  return String();
}

static void liveStart() {
  if (liveServer || !livePort) return;
  liveServer = new NetworkServer(livePort);
  liveServer->setNoDelay(true);
  liveServer->begin();
}

static void liveTarget(const String& req, String& path, String& query) {   // "GET /servo?pin=33 HTTP/1.1..." -> "/servo", "pin=33"
  path = "/"; query = "";
  int a = req.indexOf(' ');
  if (a < 0) return;
  int b = req.indexOf(' ', a + 1);
  if (b < 0) b = req.indexOf('\r', a + 1);
  if (b < 0) b = req.length();
  String t = req.substring(a + 1, b);
  int q = t.indexOf('?');
  if (q >= 0) { path = t.substring(0, q); query = t.substring(q + 1); } else path = t;
}

static void liveRespond(LiveSlot& s, const String& req) {
  bool isGet = req.startsWith("GET ");
  String path, query; liveTarget(req, path, query);
  const char* type = liveType; String body; const char* pageBody = nullptr; size_t pageLen = 0; bool ok = true; String extraHead;
  const LivePageEntry* page = nullptr;
  bool root = path == "/" || path == "/arm" || path == "/arm.html" || path == "/index.html";
  for (uint8_t i = 0; i < livePageN && !page; i++) if (path == livePages[i].path || (root && !strcmp(livePages[i].path, "/"))) page = &livePages[i];
  static LivePageEntry rootPlain;                                   // the one-argument livePage(html), still served for older sketches
  if (!page && livePageHtml && root) { rootPlain = { "/", livePageHtml, strlen(livePageHtml), false }; page = &rootPlain; }
  if (!isGet) { ok = false; type = "text/plain"; body = "GET only\n"; }
  else if (path == "/key") {                                       // console keys, same origin as the page
    String c = queryArg(query, "c");
    for (size_t i = 0; i < c.length(); i++) keyPush((uint8_t)c[i]);
    type = "text/plain"; body = "ok\n";
  }
  else if (path == "/log") {                                       // the console ring, same protocol as port 80: ?from=N, X-Log-Head = the new N
    uint32_t head = 0;
    body = logSince(queryArg(query, "from"), head);
    type = "text/plain; charset=utf-8";
    extraHead = "X-Log-Head: " + String(head) + "\r\nAccess-Control-Expose-Headers: X-Log-Head\r\n";   // a page on another origin may read the header too
  }
  else if (page) { type = "text/html; charset=utf-8"; pageBody = page->body; pageLen = page->len; if (page->gz) extraHead += "Content-Encoding: gzip\r\n"; }
  else {
    const LiveRoute* r = nullptr;
    for (auto& x : liveRoutes) if (path == x.path) { r = &x; break; }
    if (r) { type = r->type; body = r->fn(query); } else body = liveFn();
  }
  size_t len = pageBody ? pageLen : body.length();
  String head; head.reserve(300 + (pageBody ? 0 : len));
  head += ok ? "HTTP/1.1 200 OK" : "HTTP/1.1 405 Method Not Allowed";
  head += "\r\nContent-Type: "; head += type;
  head += "\r\nContent-Length: "; head += len;
  head += "\r\nAccess-Control-Allow-Origin: *\r\nCache-Control: no-store\r\nConnection: keep-alive\r\n";
  head += extraHead; head += "\r\n";
  if (!pageBody) head += body;                     // one buffer for a RAM response; a flash page follows the head from where it is
  s.out = std::move(head); s.flashBody = pageBody; s.flashLen = pageBody ? len : 0; s.sent = 0; s.sending = true; s.sendStart = millis();
  liveRequests++;
}

static bool liveWritable(NetworkClient& c) {      // does the socket have room right now? (lwIP: at least the low-water mark free)
  int fd = c.fd(); if (fd < 0) return false;
  fd_set set; FD_ZERO(&set); FD_SET(fd, &set);
  struct timeval tv = { 0, 0 };
  return select(fd + 1, NULL, &set, NULL, &tv) > 0 && FD_ISSET(fd, &set);
}
static void liveDrain(LiveSlot& s, uint32_t now) {
  uint8_t guard = LIVE_CHUNKS_PER_POLL;
  while (s.sending && guard--) {
    size_t total = s.out.length() + s.flashLen;
    if (s.sent >= total) { s.sending = false; s.out = ""; s.flashBody = nullptr; s.flashLen = 0; s.sent = 0; s.lastMs = now; return; }
    if (!s.client.connected()) { s.sending = false; s.out = ""; s.flashBody = nullptr; return; }
    if (!liveWritable(s.client)) {
      if (now - s.sendStart > LIVE_SEND_MAX_MS) { s.client.stop(); s.sending = false; s.out = ""; s.flashBody = nullptr; }   // not taking data: let the slot go
      return;
    }
    const uint8_t* p; size_t n;
    if (s.sent < s.out.length()) { p = (const uint8_t*)s.out.c_str() + s.sent; n = s.out.length() - s.sent; }
    else { size_t o = s.sent - s.out.length(); p = (const uint8_t*)s.flashBody + o; n = s.flashLen - o; }
    if (n > LIVE_CHUNK) n = LIVE_CHUNK;
    size_t w = s.client.write(p, n);             // room confirmed and under one segment: returns at once
    if (!w) { s.client.stop(); s.sending = false; s.out = ""; s.flashBody = nullptr; return; }
    s.sent += w; s.sendStart = now;
  }
}

static void livePoll() {
  if (!liveServer) return;
  uint32_t now = millis();
  NetworkClient nc = liveServer->accept();
  if (nc) {
    nc.setNoDelay(true);
    liveAccepts++;
    LiveSlot* slot = nullptr;
    for (auto& s : liveSlots) if (!s.client.connected()) { slot = &s; break; }
    if (!slot) {                                    // all busy: evict the one silent longest, preferring one with nothing in flight
      slot = &liveSlots[0];
      for (auto& s : liveSlots) if ((s.sending ? 1 : 0) < (slot->sending ? 1 : 0) || (s.sending == slot->sending && s.lastMs < slot->lastMs)) slot = &s;
      slot->client.stop();
    }
    slot->client = nc; slot->req = ""; slot->lastMs = now; slot->sending = false; slot->out = ""; slot->flashBody = nullptr; slot->flashLen = 0; slot->sent = 0;
  }
  for (auto& s : liveSlots) {
    if (!s.client.connected()) { if (s.req.length()) s.req = ""; if (s.sending) { s.sending = false; s.out = ""; s.flashBody = nullptr; } continue; }
    if (s.sending) { liveDrain(s, now); if (s.sending) continue; }   // finish the response in flight before reading the next request
    int n = s.client.available();
    while (n-- > 0) {
      s.req += (char)s.client.read();
      if (s.req.length() > LIVE_MAX_REQ) { s.client.stop(); s.req = ""; break; }
      if (s.req.endsWith("\r\n\r\n")) {           // end of the request head; GETs carry no body
        liveRespond(s, s.req);
        s.req = ""; s.lastMs = now;
        liveDrain(s, now);                         // usually the whole response goes out right here
        break;                                     // one request per poll per client; a pipelined second one waits its turn
      }
    }
    if (s.client.connected() && !s.sending && now - s.lastMs > LIVE_IDLE_MS) { s.client.stop(); s.req = ""; }
  }
}

// ---------------- console buffers ----------------------------------------
// Output ring. logHead counts every byte ever printed; GET /log?from=N
// returns bytes N..head so a poller never misses or repeats anything.
static uint8_t  logBuf[LOG_SIZE];
static uint32_t logHead = 0;
static inline void logPut(uint8_t c) { logBuf[logHead % LOG_SIZE] = c; logHead++; }
// Bytes N..head of the ring ("" = everything still held). A poller that fell behind, or
// asked with a head from before a reboot, gets the whole ring - never an error.
static String logSince(const String& fromArg, uint32_t& head) {
  head = logHead;
  uint32_t oldest = head > LOG_SIZE ? head - LOG_SIZE : 0;
  uint32_t from   = fromArg.length() ? strtoul(fromArg.c_str(), nullptr, 10) : oldest;
  if (from > head || from < oldest) from = oldest;
  String body; body.reserve(head - from);
  for (uint32_t i = from; i < head; i++) body += (char)logBuf[i % LOG_SIZE];
  return body;
}
// The same as JSON, for a page that folds the console into another request: {"head":N,"txt":"..."}.
String BenchStream::logJson(const String& fromArg) {
  uint32_t head = 0;
  String raw = logSince(fromArg, head);
  String s; s.reserve(raw.length() + raw.length() / 8 + 40);
  s += "{\"head\":"; s += head; s += ",\"txt\":\"";
  for (size_t i = 0; i < raw.length(); i++) {
    char c = raw[i];
    if (c == '"' || c == '\\') { s += '\\'; s += c; }
    else if (c == '\n') s += "\\n";
    else if (c == '\t') s += "\\t";
    else if ((uint8_t)c >= 0x20) s += c;           // other control bytes (\r among them) are dropped
  }
  s += "\"}";
  return s;
}

// Input queue, filled by GET /key?c=..., drained by Bench.read().
static uint8_t keyBuf[KEY_QUEUE];
static uint8_t keyHead = 0, keyTail = 0;
static inline int  keyCount() { return (keyHead - keyTail + KEY_QUEUE) % KEY_QUEUE; }
static inline void keyPush(uint8_t c) {
  uint8_t n = (keyHead + 1) % KEY_QUEUE;
  if (n != keyTail) { keyBuf[keyHead] = c; keyHead = n; }
}
static inline int keyPeek() { return keyCount() ? keyBuf[keyTail] : -1; }
static inline int keyPop()  {
  if (!keyCount()) return -1;
  uint8_t c = keyBuf[keyTail];
  keyTail = (keyTail + 1) % KEY_QUEUE;
  return c;
}

static void fireUpdateStart() {
  if (updateStartFn) updateStartFn();
}

// ---------------- battery monitor ----------------------------------------
static int8_t   battPin     = -1;
static float    battRatio   = 1.0f;
static float    battScale   = 1.0f, battOffset = 0.0f;   // two-point calibration
static float    battSamples[10];                  // 100 ms each -> 1 s window
static uint8_t  battFill = 0, battPos = 0;
static float    battVolts   = 0;
static uint32_t battLastSampleMs = 0, battCandidateSince = 0, battLastNagMs = 0;
static BatteryState battState = BatteryState::Unknown, battCandidate = BatteryState::Unknown;
static std::function<void()> battCriticalFn;
// "USB" is only believable if we have never seen a pack, or the reading
// stepped down into the USB band in one jump (a switch-off with USB still
// attached). A pack that *declines* into that band is a collapsing pack.
static bool     battEverSawPack = false;
static float    battLastRaw     = 0;
static uint32_t battStepDownMs  = 0;
static constexpr float    BATT_PACK_SEEN = 6.0f, BATT_STEP_V = 1.0f;
static constexpr uint32_t BATT_STEP_WINDOW_MS = 5000;

static constexpr float BATT_FAULT_LO = 4.6f, BATT_FAULT_LO_CLEAR = 4.8f;
static constexpr float BATT_FAULT_HI = 9.0f, BATT_FAULT_HI_CLEAR = 8.8f;
static constexpr float BATT_NOPACK_HI = 5.8f, BATT_NOPACK_HI_CLEAR = 6.0f;   // USB back-feed reads ~5.0-5.3 V, steady
static constexpr float BATT_NOISE_FAULT = 0.6f;   // spread within 1 s; a floating pin wanders, a pack or USB does not
static constexpr float BATT_LOW = 7.0f, BATT_LOW_CLEAR = 7.2f;
static constexpr float BATT_CRIT = 6.6f, BATT_CRIT_CLEAR = 6.9f;
static constexpr uint32_t BATT_PERSIST_MS = 2000, BATT_NAG_MS = 15000;

static const char* battName(BatteryState s) {
  switch (s) {
    case BatteryState::Ok:       return "ok";
    case BatteryState::Low:      return "LOW";
    case BatteryState::Critical: return "CRITICAL";
    case BatteryState::Fault:    return "MONITOR FAULT";
    case BatteryState::NoPack:   return "no pack (USB)";
    default:                     return "unknown";
  }
}

// What the current voltage argues for, given where we are now (hysteresis).
static BatteryState battClassify(float v, float spread, BatteryState cur, bool usbBelievable) {
  bool inFault = (cur == BatteryState::Fault);
  if (spread > BATT_NOISE_FAULT) return BatteryState::Fault;
  if (v < (inFault ? BATT_FAULT_LO_CLEAR : BATT_FAULT_LO) ||
      v > (inFault ? BATT_FAULT_HI_CLEAR : BATT_FAULT_HI)) return BatteryState::Fault;
  if (v < BATT_NOPACK_HI || (cur == BatteryState::NoPack && v < BATT_NOPACK_HI_CLEAR))
    return usbBelievable ? BatteryState::NoPack : BatteryState::Critical;   // a pack that got here is dying
  if (v <= BATT_CRIT || (cur == BatteryState::Critical && v < BATT_CRIT_CLEAR)) return BatteryState::Critical;
  if (v <= BATT_LOW  || (cur == BatteryState::Low      && v < BATT_LOW_CLEAR))  return BatteryState::Low;
  return BatteryState::Ok;
}

static void battAnnounce(BatteryState s, float v) {
  switch (s) {
    case BatteryState::Fault:
      Bench.printf("[batt] MONITOR FAULT: reading %.2f V is not a 2S pack. divider leg snapped / pin floating? (battery state unknown)\n", v);
      break;
    case BatteryState::NoPack:
      Bench.printf("[batt] %.2f V steady = USB/bench power back-feeding the junction, no pack. monitoring paused.\n", v);
      break;
    case BatteryState::Critical:
      if (v < BATT_NOPACK_HI) Bench.printf("[batt] CRITICAL %.2f V - below any sane pack and it got here from a pack reading: collapsing. safe state forced.\n", v);
      else                    Bench.printf("[batt] CRITICAL %.2f V - safe state forced. stop and charge.\n", v);
      break;
    case BatteryState::Low:
      Bench.printf("[batt] LOW %.2f V - wrap up soon\n", v);
      break;
    case BatteryState::Ok:
      Bench.printf("[batt] %.2f V ok\n", v);
      break;
    default: break;
  }
}

static void battPoll() {
  if (battPin < 0) return;
  uint32_t now = millis();
  if (now - battLastSampleMs < 100) return;
  battLastSampleMs = now;

  uint32_t mv = 0;
  for (int i = 0; i < 4; i++) mv += analogReadMilliVolts(battPin);   // eFuse-calibrated
  float v = ((mv / 4.0f) / 1000.0f * battRatio) * battScale + battOffset;
  if (v >= BATT_PACK_SEEN) battEverSawPack = true;
  if (battLastRaw - v > BATT_STEP_V) battStepDownMs = now;             // one-sample drop: a switch, not a sag
  battLastRaw = v;
  battSamples[battPos] = v;
  battPos = (battPos + 1) % 10;
  if (battFill < 10) battFill++;
  float sum = 0, lo = 1e9f, hi = -1e9f;
  for (uint8_t i = 0; i < battFill; i++) { sum += battSamples[i]; if (battSamples[i] < lo) lo = battSamples[i]; if (battSamples[i] > hi) hi = battSamples[i]; }
  battVolts = sum / battFill;
  if (battFill < 5) return;                        // let the average settle

  bool usbBelievable = !battEverSawPack || battState == BatteryState::NoPack ||
                       (battStepDownMs && now - battStepDownMs < BATT_STEP_WINDOW_MS);
  BatteryState want = battClassify(battVolts, hi - lo, battState, usbBelievable);
  if (want != battCandidate) { battCandidate = want; battCandidateSince = now; }
  bool persisted = (now - battCandidateSince) >= BATT_PERSIST_MS || battState == BatteryState::Unknown;
  if (want != battState && persisted) {
    battState = want;
    battAnnounce(battState, battVolts);
    battLastNagMs = now;
    if (battState == BatteryState::Critical && battCriticalFn) battCriticalFn();
  } else if (battState != BatteryState::Ok && battState != BatteryState::Unknown && battState != BatteryState::NoPack
             && now - battLastNagMs >= BATT_NAG_MS) {
    battAnnounce(battState, battVolts);
    battLastNagMs = now;
  }
}

void BenchStream::batteryBegin(uint8_t adcPin, float dividerRatio, float scale, float offset) {
  battPin = adcPin; battRatio = dividerRatio; battScale = scale; battOffset = offset;
  analogReadResolution(12);
  analogSetPinAttenuation(adcPin, ADC_11db);
  battFill = battPos = 0; battState = battCandidate = BatteryState::Unknown;
}
float        BenchStream::batteryVolts() const     { return battVolts; }
BatteryState BenchStream::batteryState() const     { return battState; }
const char*  BenchStream::batteryStateName() const { return battName(battState); }
void         BenchStream::onBatteryCritical(std::function<void()> fn) { battCriticalFn = std::move(fn); }

// Restart from poll(), never from inside an HTTP handler, so the response
// gets out first and the restart doesn't depend on HTTP stack state.
static void scheduleRestart(uint32_t inMs) { restartAt = millis() + inMs; }

// ---------------- HTTP ---------------------------------------------------
static void httpStatus() {
  String s;
  s += String(hostName) + "\n";
  s += "ip      " + WiFi.localIP().toString() + "\n";
  s += "rssi    " + String(WiFi.RSSI()) + " dBm\n";
  s += "uptime  " + String(millis() / 1000) + " s\n";
  s += "heap    " + String(ESP.getFreeHeap()) + " B free\n";
  s += "sketch  " + ESP.getSketchMD5() + "\n";
  s += "log     head=" + String(logHead) + "\n";
  if (battPin >= 0) s += "battery " + String(battVolts, 2) + " V  " + battName(battState) + "\n";
  s += "\nGET  /log?from=N   console output since byte N (X-Log-Head = new N)\n";
  s += "GET  /key?c=X      send keystrokes\n";
  s += "POST /update       multipart field 'firmware' = .bin\n";
  s += "POST /update2?offset=N&size=S[&final=1]   16 KB chunks, field 'chunk' (resumable)\n";
  s += "GET  /reboot\n";
  for (auto& r : extraRoutes) s += "GET  " + String(r.path) + "\n";
  if (livePort) {
    s += "live feed  http://" + WiFi.localIP().toString() + ":" + String(livePort) + "/  keep-alive; " + String(liveAccepts) + " connections, " + String(liveRequests) + " requests; /key and /log work there too" + (livePageHtml ? "; / is the page" : "");
    for (uint8_t i = 0; i < livePageN; i++) s += String("; ") + livePages[i].path + (livePages[i].gz ? " (page, gzip)" : " (page)");
    for (auto& r : liveRoutes) s += String("; ") + r.path;
    s += "; anything else is the feed\n";
  }
  http.send(200, "text/plain", s);
}

static void httpLog() {
  uint32_t head = 0;
  String body = logSince(http.hasArg("from") ? http.arg("from") : String(), head);
  http.sendHeader("X-Log-Head", String(head));
  http.sendHeader("Cache-Control", "no-store");
  http.send(200, "text/plain", body);
}

static void httpKey() {
  String c = http.arg("c");
  for (size_t i = 0; i < c.length(); i++) keyPush((uint8_t)c[i]);
  http.sendHeader("Access-Control-Allow-Origin", "*");     // a page served from another port (the simulator) may press keys too
  http.send(200, "text/plain", "ok\n");
}

static void httpUpdateDone() {
  http.sendHeader("Connection", "close");
  if (uploadOk) {
    http.send(200, "text/plain", "OK - rebooting\n");
    scheduleRestart(500);
  } else {
    String why = Update.hasError() ? Update.errorString() : "no firmware received";
    http.send(500, "text/plain", "FAIL: " + why + "\n");
    Bench.printf("[ota] rejected: %s\n", why.c_str());
  }
}

static void httpUpdateUpload() {
  HTTPUpload& up = http.upload();
  switch (up.status) {
    case UPLOAD_FILE_START:
      uploadOk = false;
      fireUpdateStart();
      Bench.printf("[ota] http upload: %s\n", up.filename.c_str());
      if (!Update.begin(UPDATE_SIZE_UNKNOWN))
        Bench.printf("[ota] begin failed: %s\n", Update.errorString());
      break;
    case UPLOAD_FILE_WRITE:
      if (Update.write(up.buf, up.currentSize) != up.currentSize)
        Bench.printf("[ota] write failed: %s\n", Update.errorString());
      break;
    case UPLOAD_FILE_END:
      if (Update.end(true)) {
        uploadOk = true;
        Bench.printf("[ota] %u bytes written - ok\n", up.totalSize);
      } else {
        Bench.printf("[ota] end failed: %s\n", Update.errorString());
      }
      break;
    case UPLOAD_FILE_ABORTED:
      Update.abort();
      Bench.println("[ota] upload aborted");
      break;
  }
}

// ---------------- chunked, resumable firmware upload ---------------------
// POST /update2?offset=N&size=S[&final=1], multipart field 'chunk'. Each
// chunk is one short request, so a WiFi stall (the ELRS module's RF jams
// the ESP32 at bench distance) costs a retry of 16 KB, not the whole image.
// flash.sh drives this and falls back to /update on boards without it.
static uint32_t chunkExpected = 0;
static bool     chunkOpen     = false;
static bool     chunkFailed   = false;

static void httpChunkUpload() {
  HTTPUpload& up = http.upload();
  uint32_t offset = strtoul(http.arg("offset").c_str(), nullptr, 10);
  uint32_t size   = strtoul(http.arg("size").c_str(),   nullptr, 10);
  switch (up.status) {
    case UPLOAD_FILE_START:
      chunkFailed = false;
      if (offset == 0) {
        if (chunkOpen) Update.abort();
        fireUpdateStart();
        // UNKNOWN size = erase lazily, one sector per write. A known size
        // erases the whole image up front (seconds) inside this request,
        // starving WiFi long enough to lose the connection.
        chunkOpen = Update.begin(UPDATE_SIZE_UNKNOWN);
        chunkExpected = 0;
        if (!chunkOpen) { Bench.printf("[ota] begin failed: %s\n", Update.errorString()); chunkFailed = true; }
        else Bench.printf("[ota] chunked upload, %lu bytes\n", (unsigned long)size);
      } else if (!chunkOpen || offset != chunkExpected) {
        chunkFailed = true;                       // out of sequence; handler answers 409
      }
      break;
    case UPLOAD_FILE_WRITE:
      if (chunkFailed) break;
      if (Update.write(up.buf, up.currentSize) != up.currentSize) {
        Bench.printf("[ota] write failed: %s\n", Update.errorString());
        Update.abort(); chunkOpen = false; chunkFailed = true;
      } else {
        chunkExpected += up.currentSize;
      }
      break;
    case UPLOAD_FILE_END:
      break;
    case UPLOAD_FILE_ABORTED:
      chunkFailed = true;                         // client retries this chunk
      break;
  }
}

static void httpChunkDone() {
  uint32_t offset = strtoul(http.arg("offset").c_str(), nullptr, 10);
  if (chunkFailed) {
    if (chunkOpen && offset != chunkExpected)
      http.send(409, "text/plain", "expected " + String(chunkExpected) + "\n");
    else
      http.send(500, "text/plain", String("chunk failed: ") + Update.errorString() + "\n");
    return;
  }
  if (http.hasArg("final")) {
    chunkOpen = false;
    if (Update.end(true)) {
      Bench.printf("[ota] %lu bytes written - ok\n", (unsigned long)chunkExpected);
      http.sendHeader("Connection", "close");
      http.send(200, "text/plain", "OK - rebooting\n");
      scheduleRestart(500);
    } else {
      http.send(500, "text/plain", String("end failed: ") + Update.errorString() + "\n");
    }
    return;
  }
  http.send(200, "text/plain", "next " + String(chunkExpected) + "\n");
}

// ---------------- bring-up -----------------------------------------------
static void startServices() {
  http.on("/",       HTTP_GET,  httpStatus);
  http.on("/log",    HTTP_GET,  httpLog);
  http.on("/key",    HTTP_GET,  httpKey);
  http.on("/update", HTTP_POST, httpUpdateDone, httpUpdateUpload);
  http.on("/update2", HTTP_POST, httpChunkDone, httpChunkUpload);
  http.on("/reboot", HTTP_GET,  [] {
    http.send(200, "text/plain", "rebooting\n");
    scheduleRestart(300);
  });
  for (auto& r : extraRoutes) registerExtra(r);
  http.onNotFound([] { http.send(404, "text/plain", "try GET /\n"); });
  http.begin();

  ArduinoOTA.setHostname(hostName);        // also registers <hostname>.local via mDNS
  ArduinoOTA.onStart([] { fireUpdateStart(); Bench.println("[ota] espota start"); });
  ArduinoOTA.onEnd([]   { Bench.println("\n[ota] espota done - rebooting"); });
  ArduinoOTA.onError([](ota_error_t e) { Bench.printf("[ota] espota error %u\n", (unsigned)e); });
  ArduinoOTA.begin();
  liveStart();

  servicesUp = true;
  Bench.printf("[net] up on %s  ip=%s  http://%s.local/\n", NETS[netIdx].ssid, WiFi.localIP().toString().c_str(), hostName);
  if (NET_N > 1) {                         // 0.5.2: this network goes first at the next boot (written only when it changes)
    Preferences p;
    if (p.begin("benchnet", false)) { if (p.getUChar("net", 0) != netIdx) p.putUChar("net", netIdx); p.end(); }
  }
}

static bool wifiOn = true;                 // false = begin(name, 0, false): no radio at all until wifiStart()

static void joinStart() {
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);                    // OTA is unreliable with modem sleep
  WiFi.setAutoReconnect(true);
  WiFi.setHostname(hostName);
  static bool hooked = false;
  if (!hooked) {
    hooked = true;
    WiFi.onEvent([](arduino_event_id_t, arduino_event_info_t info) {   // our own begin() / disconnect() report "leaving": not the reason
      uint16_t r = info.wifi_sta_disconnected.reason;
      if (r != WIFI_REASON_ASSOC_LEAVE && r != WIFI_REASON_STA_LEAVING) staReason = r;
    }, ARDUINO_EVENT_WIFI_STA_DISCONNECTED);
  }
  staReason = 0; staReasonShown = false;
  WiFi.begin(NETS[netIdx].ssid, NETS[netIdx].pass);
  lastRejoin = millis();
}
static void reasonPoll() {                 // 0.5.2: one line per try saying why it did not join (the try's last reason)
  uint16_t r = staReason;
  if (!r || staReasonShown) return;
  staReasonShown = true;
  const char* hint = (r == WIFI_REASON_NO_AP_FOUND) ? " - not in range on 2.4 GHz (the ESP32 cannot see 5 GHz networks)"
                   : (r == WIFI_REASON_NO_AP_FOUND_W_COMPATIBLE_SECURITY || r == WIFI_REASON_NO_AP_FOUND_IN_AUTHMODE_THRESHOLD) ? " - seen, but its security is not one the ESP32 can use"
                   : (r == WIFI_REASON_AUTH_FAIL || r == WIFI_REASON_4WAY_HANDSHAKE_TIMEOUT || r == WIFI_REASON_HANDSHAKE_TIMEOUT) ? " - wrong password?"
                   : "";
  Bench.printf("[net] %s: %s (%u)%s\n", NETS[netIdx].ssid, WiFi.disconnectReasonName((wifi_err_reason_t)r), (unsigned)r, hint);
}

void BenchStream::begin(const char* hostname, uint32_t joinWaitMs, bool wifi) {
  hostName = hostname;
  if (!wifi) {                             // 0.5.0: the console stays on USB Serial, nothing touches the radio
    wifiOn = false;
    WiFi.mode(WIFI_OFF);
    Bench.println("[net] WiFi OFF - the console is the USB serial port (wifiStart() turns WiFi on until a reboot)");
    return;
  }
  wifiOn = true;
  if (NET_N > 1) {                         // 0.5.2: start with the network that last joined
    Preferences p;
    if (p.begin("benchnet", true)) { uint8_t n = p.getUChar("net", 0); p.end(); if (n < NET_N) netIdx = n; }
  }
  joinStart();

  if (joinWaitMs == 0) {                   // no wait: poll() sees the join and starts the services
    Bench.printf("[net] joining %s in the background%s\n", NETS[netIdx].ssid, NET_N > 1 ? " (then the other network in turn)" : "");
    return;
  }
  Bench.printf("[net] joining %s ", NETS[netIdx].ssid);
  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < joinWaitMs) {
    delay(250);
    Bench.print('.');
  }
  Bench.println();
  lastRejoin = millis();

  if (WiFi.status() == WL_CONNECTED) startServices();
  else Bench.println("[net] not joined yet - retrying in the background");
}

void BenchStream::wifiStart() {
  if (wifiOn) return;
  wifiOn = true;
  joinStart();
  Bench.printf("[net] WiFi ON at the console's request - joining %s in the background (off again after a reboot)\n", NETS[netIdx].ssid);
}
void BenchStream::wifiStop() {             // 0.5.1: the radio off until the next reboot (a deep sleep turns it off as well;
  if (!wifiOn) return;                     //        this makes it explicit and stops poll() re-joining before the sleep)
  wifiOn = false;
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  Bench.println("[net] WiFi OFF");
}
bool BenchStream::wifiEnabled() const { return wifiOn; }
// 0.5.3: hold = no background joins or scans while not joined. A scan sweeps the radio over every channel, and the
// orientation glove's ESP-NOW link to its forearm lives on ONE channel - so with no network in range the glove holds
// after a minute and its link stays put. A joined connection is left alone; one that drops is not re-joined until
// wifiHold(false), which tries again at once.
static bool netHeld = false;
static bool netScanning = false;           // an async scan started by poll() is under way
void BenchStream::wifiHold(bool on) {
  if (on == netHeld) return;
  netHeld = on;
  if (!wifiOn) return;
  if (on) {
    WiFi.setAutoReconnect(false);
    if (netScanning) { esp_wifi_scan_stop(); WiFi.scanDelete(); netScanning = false; }
    if (WiFi.status() != WL_CONNECTED) WiFi.disconnect();   // a join under way scans too
    Bench.println("[net] holding: no joins or scans until released");
  } else {
    WiFi.setAutoReconnect(true);
    lastRejoin = millis() - REJOIN_MS - NET_SWITCH_MS;      // the next poll() starts a scan
    Bench.println("[net] released: looking for a network again");
  }
}
bool BenchStream::wifiHeld() const { return netHeld; }

void BenchStream::poll() {
  battPoll();                                // independent of WiFi

  if (restartAt && (int32_t)(millis() - restartAt) >= 0) {
    Bench.println("[net] restarting");
    delay(300);
    ESP.restart();
  }
  if (!wifiOn) return;

  if (WiFi.status() != WL_CONNECTED) {
    if (netHeld) return;                     // 0.5.3
    // 0.5.2: no join yet -> scan (in the background), then join the strongest network of ours the radio can see, on
    // its channel and BSSID. A blind begin() gave up for good after one missed scan ("no AP found" is not retried by the
    // core), so a network in plain view was never joined. The list is logged when it changes (2.4 GHz only).
    static String   lastSeen;
    if (netScanning) {
      int n = WiFi.scanComplete();
      if (n == WIFI_SCAN_RUNNING) return;
      netScanning = false;
      lastRejoin = millis();
      if (n < 0) { Bench.println("[net] scan failed"); WiFi.begin(NETS[netIdx].ssid, NETS[netIdx].pass); return; }
      int best = -1; uint8_t bestNet = 0; String seen, list;
      for (int i = 0; i < n; i++) {
        String id = WiFi.SSID(i);
        seen += id + "|";
        if (i < 15) list += (i ? ", " : " ") + (id.length() ? id : String("(hidden)")) + " ch" + String(WiFi.channel(i)) + " " + String(WiFi.RSSI(i));
        for (uint8_t k = 0; k < NET_N; k++)
          if (id == NETS[k].ssid && (best < 0 || WiFi.RSSI(i) > WiFi.RSSI(best))) { best = i; bestNet = k; }
      }
      if (seen != lastSeen) { lastSeen = seen; Bench.printf("[net] the radio sees %d network(s) on 2.4 GHz:%s\n", n, list.c_str()); }
      if (best < 0) { WiFi.scanDelete(); return; }            // none of ours in range: scan again after the wait
      uint8_t bssid[6]; memcpy(bssid, WiFi.BSSID(best), 6);
      int32_t ch = WiFi.channel(best), rssi = WiFi.RSSI(best);
      WiFi.scanDelete();
      netIdx = bestNet;
      Bench.printf("[net] joining %s (ch %d, %d dBm)\n", NETS[netIdx].ssid, (int)ch, (int)rssi);
      staReason = 0; staReasonShown = false;
      WiFi.begin(NETS[netIdx].ssid, NETS[netIdx].pass, ch, bssid);
      return;
    }
    if (millis() - lastRejoin > (NET_N > 1 ? NET_SWITCH_MS : REJOIN_MS)) {
      lastRejoin = millis();
      reasonPoll();                        // why the try that is ending failed
      WiFi.disconnect();                   // a scan cannot run while a join is under way
      if (WiFi.scanNetworks(true) == WIFI_SCAN_RUNNING) { netScanning = true; return; }
      staReason = 0; staReasonShown = false;   // the scan would not start: a plain re-kick
      WiFi.begin(NETS[netIdx].ssid, NETS[netIdx].pass);
    }
    return;
  }
  if (!servicesUp) startServices();

  ArduinoOTA.handle();
  http.handleClient();
  livePoll();
}

bool   BenchStream::wifiUp() const { return WiFi.status() == WL_CONNECTED; }
String BenchStream::ip() const     { return WiFi.localIP().toString(); }
void   BenchStream::onUpdateStart(std::function<void()> fn) { updateStartFn = std::move(fn); }
void   BenchStream::onGet(const char* path, std::function<String()> fn, const char* contentType) {
  extraRoutes.push_back({ path, contentType, std::move(fn) });
  if (servicesUp) registerExtra(extraRoutes.back());   // otherwise startServices() registers it
}
void   BenchStream::liveFeed(uint16_t port, std::function<String()> fn, const char* contentType) {
  livePort = port; liveFn = std::move(fn); liveType = contentType;
  if (servicesUp) liveStart();
}
void   BenchStream::livePage(const char* html) { livePageHtml = html; }
static void livePageAdd(const char* path, const char* body, size_t len, bool gz) {
  for (uint8_t i = 0; i < livePageN; i++) if (!strcmp(livePages[i].path, path)) { livePages[i] = { path, body, len, gz }; return; }
  if (livePageN < LIVE_PAGES) livePages[livePageN++] = { path, body, len, gz };
}
void   BenchStream::livePage(const char* path, const char* html) { livePageAdd(path, html, strlen(html), false); }
void   BenchStream::livePageGz(const char* path, const uint8_t* gz, size_t len) { livePageAdd(path, (const char*)gz, len, true); }
void   BenchStream::onLive(const char* path, std::function<String(const String&)> fn, const char* contentType) {
  liveRoutes.push_back({ path, contentType, std::move(fn) });
}

// ---------------- Stream ---------------------------------------------------
int  BenchStream::available() { return Serial.available() + keyCount(); }
int  BenchStream::read()      { return Serial.available() ? Serial.read() : keyPop(); }
int  BenchStream::peek()      { return Serial.available() ? Serial.peek() : keyPeek(); }
void BenchStream::flush()     { Serial.flush(); }

size_t BenchStream::write(uint8_t c) {
  logPut(c);
  return Serial.write(c);
}

size_t BenchStream::write(const uint8_t* b, size_t n) {
  for (size_t i = 0; i < n; i++) logPut(b[i]);
  return Serial.write(b, n);
}
