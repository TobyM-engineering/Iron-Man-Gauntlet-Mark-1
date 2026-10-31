// PuckUpdater - the puck's update mode (Toby, 2026-10-08): a small program of its own in the second program slot
// (ota_1 at 0x310000 of the 16 MB layout "app3M_fat9M_16MB"), so the normal firmware (ota_0) never carries WiFi: WiFi
// needs ~80 KB of internal RAM, which the 3D view's triangle buffer uses (measured 2026-10-08: with WiFi in the
// normal firmware the puck could not even allocate its buffers).
//
//   enter : hold a finger on the screen while powering the puck up, ~3 s (or "update" on the USB console): the
//           normal firmware points the boot at this slot and restarts
//   here  : WiFi only, no 3D view; the screen says the address. Push the new normal firmware with
//               curl -F "image=@Puck.ino.bin" http://<address>/update      (firmware/puck-ota.ps1)
//           it is written into ota_0, the boot points back there and the puck restarts into it
//   leave : a tap on the screen, "normal" on the USB console, or 10 minutes with no update -> back to the normal
//           firmware unchanged
//
// Flashed once over USB (firmware/puck-updater-usb.ps1); an over-the-air update replaces
// only the normal firmware. WiFi credentials in secrets.h (gitignored; copy secrets.example.h).
#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <Update.h>
#include <Wire.h>
#include <esp_ota_ops.h>
#include <esp_wifi.h>
#define LGFX_USE_V1
#include <LovyanGFX.hpp>
#include "secrets.h"

// the display as the normal firmware drives it (Puck/app.cpp, config.h: 80 MHz, inverted, BGR)
class LGFX : public lgfx::LGFX_Device {
  lgfx::Panel_GC9A01 _panel;
  lgfx::Bus_SPI _bus;
public:
  LGFX() {
    { auto cfg = _bus.config(); cfg.spi_host = SPI2_HOST; cfg.spi_mode = 0; cfg.freq_write = 40000000; cfg.freq_read = 16000000;
      cfg.spi_3wire = false; cfg.use_lock = true; cfg.dma_channel = SPI_DMA_CH_AUTO; cfg.pin_sclk = 10; cfg.pin_mosi = 11; cfg.pin_miso = -1;
      cfg.pin_dc = 8; _bus.config(cfg); _panel.setBus(&_bus); }
    { auto cfg = _panel.config(); cfg.pin_cs = 9; cfg.pin_rst = 14; cfg.pin_busy = -1; cfg.panel_width = 240; cfg.panel_height = 240;
      cfg.offset_x = 0; cfg.offset_y = 0; cfg.offset_rotation = 0; cfg.readable = false; cfg.invert = true; cfg.rgb_order = false;
      cfg.dlen_16bit = false; cfg.bus_shared = false; _panel.config(cfg); }
    setPanel(&_panel);
  }
};
static LGFX tft;
static WebServer server(80);

constexpr uint32_t IDLE_LEAVE_MS = 10UL * 60 * 1000;   // no update for this long -> back to the normal firmware
constexpr uint8_t TOUCH_ADDR = 0x15;
constexpr int TOUCH_SDA = 6, TOUCH_SCL = 7, TOUCH_RST = 13;

static uint32_t lastActivity = 0, received = 0, total = 0;
static bool uploading = false;
static String ip = "-", lastErr;

static void screen(const char* title, const char* l1, const char* l2 = "", uint16_t col = TFT_CYAN) {
  tft.fillScreen(TFT_BLACK);
  tft.drawCircle(120, 120, 117, col);
  tft.setTextDatum(middle_center);
  tft.setTextColor(col, TFT_BLACK); tft.setTextSize(2); tft.drawString(title, 120, 82);
  tft.setTextColor(TFT_WHITE, TFT_BLACK); tft.setTextSize(1); tft.drawString(l1, 120, 118); tft.drawString(l2, 120, 136);
  tft.setTextColor(TFT_DARKGREY, TFT_BLACK); tft.drawString("tap = back to normal", 120, 182);
}
static void progress() {
  char b[40]; snprintf(b, sizeof b, "%u KB received", (unsigned)(received / 1024));
  tft.fillRect(30, 150, 180, 16, TFT_BLACK);
  tft.setTextDatum(middle_center); tft.setTextColor(TFT_YELLOW, TFT_BLACK); tft.setTextSize(1); tft.drawString(b, 120, 158);
}

// back to the normal firmware (ota_0), unchanged
static void boot_normal(const char* why) {
  Serial.printf("updater: back to the normal firmware (%s)\n", why);
  screen("RESTARTING", why, "", TFT_DARKGREY);
  const esp_partition_t* p = esp_partition_find_first(ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_APP_OTA_0, nullptr);
  if (p) esp_ota_set_boot_partition(p);
  delay(400);
  ESP.restart();
}

static int touch_fingers() {   // 0 / 1, -1 when the chip does not answer
  Wire.beginTransmission(TOUCH_ADDR); Wire.write(0x02);
  if (Wire.endTransmission(false) != 0 || Wire.requestFrom((int)TOUCH_ADDR, 1) != 1) return -1;
  return (Wire.read() & 0x0F) ? 1 : 0;
}

static void handle_upload() {
  HTTPUpload& up = server.upload();
  if (up.status == UPLOAD_FILE_START) {
    received = 0; total = 0; uploading = true; lastErr = "";
    Serial.printf("updater: receiving %s\n", up.filename.c_str());
    screen("UPDATING", up.filename.c_str(), "", TFT_YELLOW);
    if (!Update.begin(UPDATE_SIZE_UNKNOWN, U_FLASH)) lastErr = Update.errorString();   // the next slot from this one: ota_0
  } else if (up.status == UPLOAD_FILE_WRITE) {
    if (lastErr.isEmpty() && Update.write(up.buf, up.currentSize) != up.currentSize) lastErr = Update.errorString();
    received += up.currentSize; lastActivity = millis();
    static uint32_t shown = 0; if (received - shown >= 64 * 1024) { shown = received; progress(); }
  } else if (up.status == UPLOAD_FILE_END) {
    total = up.totalSize; uploading = false;
    if (lastErr.isEmpty() && !Update.end(true)) lastErr = Update.errorString();   // end(true): verified, boot points at it
    Serial.printf("updater: %u bytes, %s\n", (unsigned)total, lastErr.isEmpty() ? "verified" : lastErr.c_str());
  } else if (up.status == UPLOAD_FILE_ABORTED) {
    Update.abort(); uploading = false; lastErr = "upload aborted";
  }
}

// Join the network through its STRONGEST access point (the house has several; left to itself the driver took the
// first one it heard and the join failed with reason 2, "authentication expired"). Every join was refused with
// reason 2 until 2026-10-11 although the scan heard the network fine: the router never answered - the puck's own
// transmit power, the known trouble of small ESP32 boards' antennas. Each attempt takes the next transmit power and
// alternates the join below with a plain WiFi.begin; the log says which one got in.
static uint8_t bestBssid[6]; static int bestCh = 0;
static int attempt = 0;
static const wifi_power_t kTx[] = {WIFI_POWER_11dBm, WIFI_POWER_8_5dBm, WIFI_POWER_15dBm, WIFI_POWER_13dBm, WIFI_POWER_19_5dBm};
static const char* const kTxName[] = {"11", "8.5", "15", "13", "19.5"};   // 11 got in first on 2026-10-11 (19.5, the default, never)
// The networks it knows (secrets.h): WIFI_SSID / WIFI_PASS, and optionally WIFI_SSID2 / WIFI_PASS2 and WIFI_SSID3 /
// WIFI_PASS3 (2026-10-11: the puck goes into the arm, where there is no USB to change them - home, a phone hotspot,
// the venue). It joins the one it hears strongest.
struct Net { const char* ssid; const char* pass; };
static const Net kNets[] = {{WIFI_SSID, WIFI_PASS},
#ifdef WIFI_SSID2
                            {WIFI_SSID2, WIFI_PASS2},
#endif
#ifdef WIFI_SSID3
                            {WIFI_SSID3, WIFI_PASS3},
#endif
};
constexpr int NNETS = sizeof kNets / sizeof kNets[0];
static int netIdx = 0;   // the network of the last join attempt (shown on the screen)
static int onlyNet = -1; // "net N" on the USB console: join network N alone (to prove a weaker one, e.g. the hotspot)
static void join() {
  const int k = attempt % 5; const bool plain = (attempt / 5) % 2 == 1;
  attempt++;
  WiFi.setTxPower(kTx[k]);
  Serial.printf("updater: join attempt %d - transmit %s dBm, %s\n", attempt, kTxName[k], plain ? "plain WiFi.begin" : "strongest access point");
  int n = WiFi.scanNetworks(), best = -1, bestNet = -1;
  for (int i = 0; i < n; i++) {   // everything heard, a name's bytes in hex when it is not plain ASCII (an iPhone's
    // hotspot has a curly apostrophe in it: the name in secrets.h must match byte for byte)
    String s = WiFi.SSID(i); bool ascii = true;
    for (unsigned c = 0; c < s.length(); c++) if ((uint8_t)s[c] >= 0x80) ascii = false;
    Serial.printf("updater: heard \"%s\" %d dBm", s.c_str(), (int)WiFi.RSSI(i));
    if (!ascii) { Serial.print(" bytes"); for (unsigned c = 0; c < s.length(); c++) Serial.printf(" %02X", (uint8_t)s[c]); }
    Serial.println();
  }
  for (int i = 0; i < n; i++)
    for (int j = 0; j < NNETS; j++)
      if ((onlyNet < 0 || onlyNet == j) && WiFi.SSID(i) == kNets[j].ssid) {
        Serial.printf("updater: hears \"%s\" - RSSI %d dBm, channel %d, security %d\n", kNets[j].ssid, (int)WiFi.RSSI(i), (int)WiFi.channel(i), (int)WiFi.encryptionType(i));
        if (best < 0 || WiFi.RSSI(i) > WiFi.RSSI(best)) { best = i; bestNet = j; }
      }
  if (best < 0) {   // none heard: try them in turn
    netIdx = onlyNet >= 0 ? onlyNet : (attempt - 1) % NNETS;
    Serial.printf("updater: no known network in the scan (%d networks heard) - trying \"%s\" anyway\n", n, kNets[netIdx].ssid);
    WiFi.begin(kNets[netIdx].ssid, kNets[netIdx].pass); return;
  }
  netIdx = bestNet;
  if (plain) { WiFi.begin(kNets[netIdx].ssid, kNets[netIdx].pass); return; }
  memcpy(bestBssid, WiFi.BSSID(best), 6); bestCh = WiFi.channel(best);
  Serial.printf("updater: joining \"%s\" at its strongest access point (%d dBm, channel %d)\n", kNets[netIdx].ssid, (int)WiFi.RSSI(best), bestCh);
  // the network is WPA2/WPA3 mixed: offer WPA3 with both handshake methods (hunt-and-peck and hash-to-element - the
  // default, one method only, is a known mismatch: "authentication expired") and accept WPA2 too
  wifi_config_t cfg = {};
  strncpy((char*)cfg.sta.ssid, kNets[netIdx].ssid, sizeof cfg.sta.ssid - 1);
  strncpy((char*)cfg.sta.password, kNets[netIdx].pass, sizeof cfg.sta.password - 1);
  cfg.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
  cfg.sta.pmf_cfg.capable = true; cfg.sta.pmf_cfg.required = false;
  cfg.sta.sae_pwe_h2e = WPA3_SAE_PWE_BOTH;
  cfg.sta.bssid_set = 1; memcpy(cfg.sta.bssid, bestBssid, 6); cfg.sta.channel = (uint8_t)bestCh;
  esp_wifi_disconnect();
  esp_wifi_set_config(WIFI_IF_STA, &cfg);
  esp_err_t e = esp_wifi_connect();
  if (e != ESP_OK) Serial.printf("updater: esp_wifi_connect -> %d\n", (int)e);
}

void setup() {  Serial.begin(115200);
  delay(200);
  const esp_partition_t* run = esp_ota_get_running_partition();
  Serial.printf("\npuck updater (running from %s at 0x%06x)\n", run ? run->label : "?", run ? (unsigned)run->address : 0);
  pinMode(2, OUTPUT); digitalWrite(2, HIGH);   // backlight
  tft.init(); tft.setRotation(0);
  pinMode(TOUCH_RST, OUTPUT); digitalWrite(TOUCH_RST, LOW); delay(10); digitalWrite(TOUCH_RST, HIGH); delay(60);
  Wire.begin(TOUCH_SDA, TOUCH_SCL, 400000);
  screen("UPDATE MODE", "joining WiFi ...", NNETS > 1 ? "(the strongest known network)" : WIFI_SSID);
  WiFi.mode(WIFI_STA);
  WiFi.setHostname("puck");
  WiFi.onEvent([](arduino_event_id_t e, arduino_event_info_t info) {   // why a join failed, in the WiFi driver's words
    Serial.printf("updater: WiFi disconnected, reason %u\n", (unsigned)info.wifi_sta_disconnected.reason);
  }, ARDUINO_EVENT_WIFI_STA_DISCONNECTED);
  WiFi.setSleep(false);   // as the arm's bench library: joins and OTA are unreliable with modem sleep
  WiFi.setAutoReconnect(false);   // one join attempt at a time (below, every 15 s): a router refusing it is not hammered
  join();
  lastActivity = millis();
  server.on("/", HTTP_GET, [] {
    server.send(200, "text/plain", "puck updater - POST the normal firmware's .bin to /update as the form field \"image\"\n");
  });
  server.on("/update", HTTP_POST, [] {
    bool ok = lastErr.isEmpty() && total > 0;
    server.send(ok ? 200 : 500, "text/plain", ok ? String("OK - ") + total + " bytes, restarting into it\n" : "FAILED: " + lastErr + "\n");
    if (ok) { screen("UPDATED", "restarting ...", "", TFT_GREEN); delay(600); ESP.restart(); }
    screen("UPDATE FAILED", lastErr.c_str(), ip.c_str(), TFT_RED);
  }, handle_upload);
}

void loop() {
  static bool up = false;
  static uint32_t tryMs = millis();
  if (!up && WiFi.status() == WL_CONNECTED) {
    up = true; ip = WiFi.localIP().toString();
    MDNS.begin("puck");
    server.begin();
    Serial.printf("updater: WiFi up on attempt %d, http://%s/update (puck.local), RSSI %d\n", attempt, ip.c_str(), (int)WiFi.RSSI());
    String l1 = "http://" + ip + "/update";
    String l2 = String("on ") + kNets[netIdx].ssid;
    screen("UPDATE MODE", l1.c_str(), l2.c_str());
  }
  if (!up && millis() - tryMs > 15000) {   // still not joined: say so, try the next way
    tryMs = millis();
    char b[40]; snprintf(b, sizeof b, "WiFi status %d, still trying", (int)WiFi.status());
    Serial.printf("updater: %s\n", b);
    screen("UPDATE MODE", "no WiFi yet", b, TFT_ORANGE);
    WiFi.disconnect(); delay(200); join();
  }
  if (up) server.handleClient();
  // a tap leaves - only once the finger that entered update mode has been lifted for a second, and only a real tap:
  // the finger seen on 3 reads in a row (~15 ms) - one stray read had sent it back on its own (2026-10-11)
  static uint32_t upSince = 0; static bool armed = false; static int downReads = 0;
  int f = touch_fingers();
  if (f == 0) { if (!upSince) upSince = millis(); if (!armed && millis() - upSince > 1000) armed = true; }
  if (f == 1) upSince = 0;
  if (armed && !uploading) {
    if (f == 1) downReads++;
    else if (f == 0) { if (downReads >= 3) boot_normal("tapped"); downReads = 0; }
  }
  if (!uploading && millis() - lastActivity > IDLE_LEAVE_MS) boot_normal("10 minutes, no update");
  static String line;   // the USB console: "normal" = back to the normal firmware (the bench's tap)
  while (Serial.available()) { char ch = (char)Serial.read(); if (ch == '\n' || ch == '\r') { line.trim(); if (line == "normal") boot_normal("console");
      if (line.startsWith("net ")) { int k = line.substring(4).toInt() - 1; onlyNet = (k >= 0 && k < NNETS) ? k : -1;
        Serial.printf("updater: %s%s\n", onlyNet < 0 ? "any known network" : "only network ", onlyNet < 0 ? "" : kNets[onlyNet].ssid);
        if (!up) { WiFi.disconnect(); delay(200); join(); } }
      line = ""; } else if (line.length() < 32) line += ch; }
  delay(5);
}
