#pragma once
#include <Arduino.h>
#include <functional>

// `Bench` - the stream every sketch prints to and reads from. Always on.
//
// At boot it joins WiFi and serves HTTP on port 80:
//   GET  /              status
//   GET  /log?from=N    everything printed since byte N (4 KB ring), header
//                       X-Log-Head carries the new N. This is the console.
//   GET  /key?c=X       feed keystrokes to the sketch's menu
//   POST /update        firmware .bin (what flash.sh uses)
//   GET  /reboot        remote reset
//   GET  <extra>        routes the sketch adds with onGet() (HandBench: /imu, /arm)
// ArduinoOTA (espota) is also up. Everything still echoes to USB Serial and
// USB Serial input still works, so a cable is a fallback, never a need.
//
// The console is polled request/response on purpose: on this network,
// TCP data the board originates spontaneously (a telnet mirror) did not
// reliably reach the PC, while replies to PC requests always did.
//
// WiFi joins are retried forever in poll(). A board that boots with the
// router down still comes up once the router is back.
//
// ADC2 is unusable while WiFi is up. Every analog input in the pin map
// (32, 33, 34, 35, 36, 39) is ADC1, so this never bites.

// Battery monitor state. FAULT means the *monitor* is broken (reading is
// not a plausible 2S voltage - divider leg snapped, pin floating), and is
// deliberately not treated as a low battery.
// NoPack: a steady 4.6-5.8 V is USB (or a 5 V bench feed) back-feeding the
// battery junction through the logic regulator - not a pack at all. No
// warnings, no safe state, arming allowed.
enum class BatteryState : uint8_t { Unknown, Ok, Low, Critical, Fault, NoPack };   // CamelCase: LOW/HIGH are Arduino macros

class BenchStream : public Stream {
 public:
  // Call at the END of setup(), after all hardware is initialised. Blocks
  // up to joinWaitMs (15 s) for the first WiFi join, then returns either way.
  // 0 = do not wait at all: poll() finishes the join and starts the services
  // once it is up, so loop() runs from the first moment (PalmRing: the ring
  // must light and the switch must count the instant power comes on).
  void begin(const char* hostname, uint32_t joinWaitMs = 15000, bool wifi = true);
  // 0.5.0: wifi = false keeps the radio OFF (the console is the USB serial port, the extra routes and pages wait);
  // wifiStart() turns it on later until the next reboot. 0.5.1: wifiStop() turns the radio off for good (the
  // gauntlet's CRITICAL shutdown calls it before its deep sleep). The gauntlet itself runs with WiFi on at boot.
  void wifiStart();
  void wifiStop();
  bool wifiEnabled() const;
  // 0.5.3: wifiHold(true) stops the background joins and scans while not joined (a scan sweeps every channel, which
  // breaks an ESP-NOW link on the current one); a joined connection is left alone, a lost one waits for wifiHold(false).
  void wifiHold(bool on);
  bool wifiHeld() const;

  // Call every loop() iteration.
  void poll();

  bool   wifiUp() const;
  String ip() const;

  // Fired when a firmware upload begins (HTTP or espota). Park servos,
  // kill LEDs, disarm - nothing should be left mid-state while flash is
  // being written and the chip reboots.
  void onUpdateStart(std::function<void()> fn);

  // Extra GET routes for the sketch, e.g. a JSON feed for a browser page. fn
  // builds the whole body; it is sent with "Access-Control-Allow-Origin: *"
  // and no caching, so a page served from anywhere on the LAN can poll it.
  // Register before or after begin(). Listed by GET /.
  void onGet(const char* path, std::function<String()> fn, const char* contentType = "text/plain");

  // A keep-alive feed on its own port for a page polling 30-50 times a second
  // (the arm simulator). The main server above closes every connection,
  // serves one client at a time, and waits up to 5 s for a connected client
  // that has not spoken yet - browsers open such spare connections routinely,
  // which froze the simulator for seconds at a time. This one keeps up to
  // four connections open, answers whichever has a complete request, and
  // never waits - not even to send: a response is drained in 1 KB pieces
  // only while the socket reports room (0.4.3), so a client on a weak link
  // that is slow to acknowledge cannot stall the loop, and with it every
  // other client. Any GET path returns fn(). CORS *. Starts with begin().
  void liveFeed(uint16_t port, std::function<String()> fn, const char* contentType = "application/json");
  // The console ring since byte N as JSON, {"head":N,"txt":"..."} - for a
  // page that folds the console into its own poll instead of a second request.
  String logJson(const String& fromArg);
  // Optional page on the same port: GET / (or /arm) returns this HTML, so a
  // page and the feed it polls share one origin - no cross-port fetch for a
  // browser to block. The string must stay valid (a static). The two-argument
  // form adds more pages by path (up to four; HandBench: "/hud"), served
  // straight from flash, no copy.
  void livePage(const char* html);
  void livePage(const char* path, const char* html);
  // 0.4.4: a page pre-compressed - the bytes of `gzip -9 page.html` (HandBench/prebuild.sh makes
  // the arrays) - sent with Content-Encoding: gzip, about a quarter of the flash the plain text
  // took. path "/" is the root page (it also answers /arm, /arm.html and /index.html).
  void livePageGz(const char* path, const uint8_t* gz, size_t len);
  // Extra paths on the feed port: fn(query) builds the body (query is the raw
  // "a=1&b=2" part). GET /key?c=X and GET /log?from=N (the console ring, same
  // protocol as port 80, X-Log-Head exposed to CORS) are built in there too,
  // so a page served from the feed port can press keys and read the console
  // without crossing ports.
  void onLive(const char* path, std::function<String(const String& query)> fn, const char* contentType = "text/plain");

  // ---- battery monitor (2S pack through a resistor divider on an ADC1 pin)
  // Sampled continuously from poll(): calibrated ADC, 1 s moving average,
  // 2 s persistence and hysteresis so servo sag doesn't flap it.
  //   FAULT     reading < 4.6 V, > 9 V, or wandering - the monitor itself is broken
  //   NOPACK    steady 4.6-5.8 V - USB/bench power, no pack; monitoring paused
  //   LOW       <= 7.0 V (clears at 7.2)  - warn
  //   CRITICAL  <= 6.6 V (clears at 6.9)  - onBatteryCritical fires once
  // Warnings print to the console on every state change and repeat every
  // 15 s while not OK; GET / shows the current reading and state.
  // volts = (pin volts x dividerRatio) x scale + offset; scale/offset from a
  // two-point multimeter calibration (1, 0 = uncalibrated).
  void         batteryBegin(uint8_t adcPin, float dividerRatio, float scale = 1.0f, float offset = 0.0f);
  float        batteryVolts() const;          // filtered, 0 until the first samples
  BatteryState batteryState() const;
  const char*  batteryStateName() const;
  bool         batteryCritical() const { return batteryState() == BatteryState::Critical; }
  void         onBatteryCritical(std::function<void()> fn);   // go to a safe state

  // Stream - output goes to Serial and the /log ring; input comes from
  // Serial and /key.
  int    available() override;
  int    read() override;
  int    peek() override;
  void   flush() override;
  size_t write(uint8_t c) override;
  size_t write(const uint8_t* buf, size_t len) override;
  using Print::write;
};

extern BenchStream Bench;
