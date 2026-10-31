<div align="center">

# Iron Man Gauntlet — Mark 1

## A wearable Iron Man forearm and glove that flies a drone with nothing but your hand.

<img src="media/palm-light.jpg" alt="The Mark 1 gauntlet lying palm up, the palm repulsor lit white with three red accent LEDs around it" width="380">

<img src="https://img.shields.io/badge/Platform-ESP32-blue?style=flat-square">
<img src="https://img.shields.io/badge/Firmware-C++%20(Arduino)-brightgreen?style=flat-square">
<img src="https://img.shields.io/badge/Radio-ExpressLRS%20%2F%20CRSF-orange?style=flat-square">
<img src="https://img.shields.io/badge/Sensors-2×%20BNO085%20%2B%205%20flex-yellow?style=flat-square">
<img src="https://img.shields.io/badge/License-MIT-lightgrey?style=flat-square">

</div>

A 3D-printed forearm gauntlet and sensor glove, run by one ESP32, that **flies a BetaFPV Air65 tiny whoop by hand gesture**: finger curl is the throttle, the wrist is the sticks, and there is no transmitter, because the arm *is* the radio. It also **opens four servo-driven armour plates in sequence**, lights a **palm repulsor**, fires a **green and a red laser**, measures distance with a **LiDAR**, shows a **thermal camera** and a **live 3D copy of the arm** on a round touch screen, and **buzzes in your hand** to confirm every gesture.

Every function is a hand gesture. It doesn't need a phone, a computer or a button, but it also serves its own **3D simulator and telemetry page** to any browser on the WiFi.

---

# 🎥 Videos

> Videos are coming soon. These are the ones planned:

- **Opening the armour** — the hand plate lifts, plates 2 and 1 follow, plate 3 last
- **Closing the armour** — the same cascade in reverse
- **Green laser** — the plate opens and the laser fires out of it
- **Red laser and LiDAR** — firing the red laser with the distance reading on the screen
- **What the simulator sees** — the screen and the browser copying the real arm live
- **Drone out** — the plates open, the drone flies out, the plates close, then flying it by hand
- **Drone in** — the plates open, the drone lands, the plates close with it inside
- **All the lights** — repulsor, palm and fingertip LEDs, the forearm strip

---

# 📸 Photos

<p align="center">
  <img src="media/plates-open-side.jpg" width="560">
</p>
<p align="center"><b>Plates open.</b> The round screen at the elbow end is showing the arm's 3D model in the same pose.</p>

<p align="center">
  <img src="media/plates-open-front.jpg" width="300">
  <img src="media/plates-open-top.jpg" width="300">
</p>
<p align="center"><b>Plates open, from the front and from above.</b> The glow under the hand is the palm light.</p>

<p align="center">
  <img src="media/plates-closed.jpg" width="300">
  <img src="media/plates-closed-top.jpg" width="300">
</p>
<p align="center"><b>Plates closed.</b> On the right, the screen showing the arm in its exact orientation, with the battery level.</p>

<p align="center">
  <img src="media/plates-closed-side.jpg" width="560">
</p>
<p align="center"><b>Plates closed, side view.</b></p>

---

# 🌍 Project Background

I wanted to see if a drone could be flown with no controller at all. Not a glove that drives a transmitter, but a glove that *is* the transmitter. An ELRS radio module is wired straight to the arm's ESP32 and bound to the drone, so every stick movement comes from the hand.

I built the hand-flying first on a bench glove (two orientation sensors, five flex sensors and the radio module on their own ESP32) and flew the drone with it. Mark 1 runs the same flight code with better sensors and the radio built into the arm, and adds the armour, the lights, the lasers and the screen, in one wearable with one firmware.

---

# 🧩 How It Works

<img src="hardware/system-overview.svg" alt="System overview: the hand's sensors and outputs cross the wrist to the ESP32 in the forearm, which drives the plates, lights and lasers through a PCA9685, talks to the round screen, and flies the drone through an ELRS module; a browser connects over WiFi; a 2S LiHV pack feeds a 5 V servo rail, a 5 V logic rail and the radio" width="900">

One ESP32 does everything. The forearm holds the brain, the power and the drivers, and the hand holds the sensors. **37 soldered wires** cross the wrist. A PCA9685 drives the six servos and every on/off output (both lasers, three high-power LED groups), so the ESP32's own pins stay free for the LED data lines, the radio, the screen link and the analog inputs.

## Flying a drone with your hand

The ESP32 sends the drone **stick frames 50 times a second** through a BetaFPV ELRS Nano TX over one wire (half duplex: the pin sends a frame, then lets go so the drone's telemetry comes back on the same wire). The drone flies in angle mode on its stock Betaflight tune.

| Stick | Comes from |
|---|---|
| **Throttle** | How far the four fingers are curled. A fist is a gentle descent, the middle is the drone's hover throttle, lifting the fingers climbs. |
| **Roll** | The hand's tilt against gravity |
| **Pitch** | The wrist bending up or down (the hand *relative to the forearm*) |
| **Yaw** | The hand turning left or right relative to the forearm |

**Why two orientation sensors.** One BNO085 is on the back of the hand and one is on the forearm. The firmware takes the hand's rotation relative to the forearm's, as quaternions, so "wrist straight" means level whether the arm is out in front, pointing at the floor, or the whole body is turning.

**Making it feel right** was most of the work:

- **The throttle follows the fingers but never jumps.** It can climb 20 % a second and fall 35 %, so a twitch is a few percent and a real move lands in half a second.
- **The hover point is learned.** The middle of every flight is averaged into a hover number, and the middle of the finger range gives exactly that throttle.
- **Neutral is wherever your hand is at lift-off.** The trims are taken from the flying pose, and the heading between the two sensors (they drift apart with no compass) is solved again at arming and at lift-off.
- **Every wrist is different.** A six-second capture measures how far your wrist really goes each way, and full stick is mapped to that.

**Arming:** thumb out, a fist, one quick pulse of the fingers out and back. It only arms with the wrist in line, the mode switch on FULL, the pack above 7.5 V and the drone reporting level and ready. If it refuses, the screen and the browser say why, including Betaflight's own arming flags read back over the radio.

**Disarming:** point (index out, the other three in) and click the thumb twice. It also disarms by itself if the link drops, a sensor goes quiet for 0.3 s, the mode switch leaves FULL, or the drone tilts past 75°.

**Flip-over:** if the drone lands upside down, the same arm gesture runs Betaflight's turtle mode instead, with full-stick pushes chosen from the drone's reported attitude until it's upright.

## Gestures

Every gesture needs the wrist in line with the forearm, so normal hand movement can't set one off. **One buzz in the hand = done, two = refused** (wrong mode, low battery, or busy).

| Gesture | Does | Mode |
|---|---|---|
| Three thumb clicks, hand open | Palm ring on, again for white ↔ red | SHOW |
| Three thumb clicks, hand closed | Palm ring off | SHOW |
| Pinky dips down and back / index dips | Ring brighter / dimmer | SHOW |
| Fist with the thumb in, held, then the whole hand open fast | Open every plate | SHOW |
| The same fist, a quick open and straight back | Close every plate | SHOW |
| Thumb out, fingers down: two quick pulses | Thermal camera on / off | SHOW |
| Index + middle out, ring + pinky in, three thumb clicks | Green laser on (plate 1 opens first), two clicks off | FULL |
| Thumb in, fingers closed fast, a small wrist jolt | Red laser + LiDAR on, let the fingers go to stop | FULL |
| Thumb out, a fist, one pulse out and back | Arm the drone | FULL |
| Point + two thumb clicks | Disarm | any |

A **thumb click** is a quick drop in the thumb sensor's reading: at least 12 % of its travel inside 80 ms, so a slow curl or a resting hand never counts. Three clicks have to come in one rhythm and be about the same size.

The **mode switch** on the forearm decides what gestures can do: **SAFE** (the middle position, which is also where a broken wire lands) only reports them, **SHOW** adds the plates, lights and camera, **FULL** adds the lasers, the LiDAR and the drone.

## The round screen

<p align="center">
  <img src="media/puck-3d-view.png" width="760">
</p>
<p align="center">
  <img src="media/puck-control-panel.png" width="900">
</p>
<p align="center"><i>Rendered from the screen's own firmware code: the 3D view (hand view and plates open), and the control panel in SAFE, SHOW and FULL.</i></p>

A Waveshare ESP32-S3 round touch display sits at the elbow end of the forearm. It draws the gauntlet in **3D, in software**: about 6,000 triangles from the real CAD model, at around 16 frames a second, moving with the real arm. The main board streams both sensors' orientation, all five fingers and every servo position to it 20 times a second over a serial link with CRC-checked frames.

- **Main page:** the 3D arm, roll / pitch, finger bars, battery and the mode.
- **Control panel:** swipe sideways for a button for every action (plates, lasers, LiDAR, lights, ring colour, camera, disarm). It follows the same rules as the gestures and can never arm the drone.
- **Thermal page:** the MLX90640 camera, with dead pixels filled in.
- **Calibration:** six steps (fingers flat, fist, thumb, arm out, down, up), each saved by the arm.
- It also runs the small OLED. WiFi updates go through a separate "update mode" program, so the 3D view keeps all its memory.

## Safety, built in

- **Nothing moves at power-up.** The PCA9685's output-enable is pulled up, so servos, lasers and LED groups stay off until the firmware is running, and the arm starts in SAFE.
- **No laser in flight.** Both lasers are blocked while the drone is armed and for 1.5 s after, and each laser waits for its plate to open.
- **Battery rules.** No arming under 7.5 V, a vibration warning to land at 7.3 V, a chirp every 5 s from 7.0 V, and at 6.6 V everything turns off and the arm goes to sleep.
- **Stalled servos let go.** An INA219 measures the servo current, and a plate that jams is released before the servo burns out.
- **The lights know their own heat.** Each high-power LED group has a heat model of its heat-sink and stops itself before 55 °C.
- **A watchdog** resets the arm if the main loop ever hangs.
- **A self-test with no hardware.** Every pin goes through one hardware layer, so a test build runs the whole firmware with every sensor faked: **48 tests** (gestures, plate order, battery limits, radio frames, fail-safes) in about a second on any ESP32.

## The arm's own web pages

<p align="center">
  <img src="media/simulator.png" width="700">
</p>
<p align="center"><i>The 3D simulator page, shown here in its demo mode.</i></p>

The arm serves these itself over WiFi:

- **Simulator** — a 3D model of the arm and hand driven by both orientation sensors and all five fingers, with the drone, the laser beam and the LiDAR distance drawn to scale.
- **Telemetry** — every sensor on a live graph, the gesture rules checked as you move, the drone's link and battery, the firmware's console.
- **Setup** — each servo's open and closed end set on its real plate, and the battery reading set against a meter, without recompiling.
- **Over-the-air updates** in small chunks that survive the radio module jamming the ESP32's WiFi.

---

# 🛠 Hardware

| Part | Job |
|---|---|
| ESP32 WROOM-32 devkit | The main controller |
| BETAFPV ELRS Nano TX V2 | The radio, bound straight to the drone |
| BetaFPV Air65 | The drone (Betaflight, angle mode) |
| 2× BNO085 orientation sensors | Hand and forearm |
| 5× flex sensors + ADS1115 | The fingers |
| PCA9685 + 6× MG90S servos | Four armour plates |
| INA219 | Servo current, for stall detection |
| Waveshare ESP32-S3 1.28″ round touch display | The screen, the thermal camera and the OLED |
| MLX90640 thermal camera | Under the hand plate |
| Garmin LIDAR-Lite v3 | Distance, under the hand plate |
| Green 520 nm laser · red laser | Out of plate 1 · under the hand plate |
| WS2812 ring + disc (19 LEDs) · WS2812 strip | Palm repulsor · forearm |
| 7 high-power LEDs on BuckPuck drivers, in 3 MOSFET groups | Palm and fingertips |
| DRV2605L + vibration motor | The buzz in the hand |
| 2S LiHV 1100 mAh, 10 A fuse, IRF4905 switch, two Pololu 5 V regulators | Power |

**[→ hardware/hardware.md](hardware/hardware.md)** has the full parts list, the pin map and the power tree.

## Wiring

<picture>
  <source media="(prefers-color-scheme: dark)" srcset="hardware/wiring-diagram-dark.png">
  <img src="hardware/wiring-diagram.png" alt="The wiring diagram: every part of the arm laid out from the elbow to the fingers with every wire in its real colour" width="900">
</picture>

Every part, pin and wire in the arm, in its real colour and gauge, from the elbow (left) to the fingertips (right). I wrote a small app to be the one place I look at the wiring: it's generated from the design files (nothing typed by hand), checked by a script before every build, and you can click any part to see each pin and where its wire goes. 70 parts, 166 wires, 44 resistors, capacitors and jumpers.

---

# 🧠 Notable Engineering Problems

### The drone "fell out of the sky" and it was my design
Holding a tight fist for over a second cut the throttle, which was meant for landing. It was the same squeeze as "come down", and once arming accepted a looser fist, a firm descent could reach it. The counters showed exactly one cut in the session the drone dropped. Now a fist is only ever a slow descent.

### Fifty thumb clicks to disarm
The palm ring's three-click gesture was still running in flight. Every time the disarm clicks fell into a rhythm it fired, wiped the click history and blocked the thumb for 600 ms. It no longer runs while the drone is armed.

### A plate that opened itself after closing
Releasing a servo turned the PCA9685's outputs off in the middle of a pulse, which sent a short broken pulse that threw plate 1 fully open. The outputs now wait one full pulse period after a release.

### A laser that turned itself off the moment it came on
Its five-minute safety timer compared a time read at the top of the loop with one read a few lines later. When the clock ticked in between, the subtraction wrapped around to about four billion milliseconds and the timer fired instantly, but only sometimes.

### The LiDAR was a different model than its driver
The driver was written for a LIDAR-Lite v4. The part is a v3, which answers the v4's read with a believable number for any distance under 2.56 m, so readings that looked right were never proof. It now uses the v3's own registers and actually goes to sleep.

### A pull-up that couldn't pull up
The PCA9685 board already pulls its output-enable down through 10 kΩ, so the 10 kΩ pull-up in the plan would only have reached 1.65 V, under the chip's "high". It's 1 kΩ.

---

# 🚀 Building It

The firmware is three Arduino sketches and two small libraries, built with `arduino-cli` and the ESP32 core 3.3.x.

```bash
git clone https://github.com/TobyM-engineering/Iron-Man-Gauntlet-Mark-1.git
cd Iron-Man-Gauntlet-Mark-1/firmware
cp lib/GauntletBenchIO/src/secrets.h.example lib/GauntletBenchIO/src/secrets.h   # your WiFi, never commit it

./flash.sh Ironman --fake --usb COM5    # the self-test: no hardware needed, 48 tests at boot on any ESP32
./flash.sh Ironman --usb COM5           # the first real flash (sets the partition table), battery OFF
./flash.sh Ironman                      # every later build, over WiFi to ironman.local
./flash.sh Puck --usb COM3              # the round screen
./monitor.sh                            # the arm's console over WiFi
```

**Libraries:** Adafruit NeoPixel, PWM Servo Driver, INA219, SSD1306, GFX, BusIO and MLX90640, SparkFun BNO080 Cortex 1.1.12, and LovyanGFX for the screen.

**Two things worth knowing:** `Ironman/config.h` holds every pin, address and limit, and it won't compile if a pin is used twice or a boot pin is used. And never plug in USB with the battery on: the devkit has no USB diode.

---

# 📌 Repo Status

✅ Mark 1 built and running: plates, lights, lasers, LiDAR, thermal camera and the screen, all from gestures

✅ Drone flown by hand on the bench glove; Mark 1 runs the same flight code with the radio built in

✅ Self-test 48 / 48 on hardware

This is my Mark 1. More versions to come.

---

# 📂 Repository Structure

```
├── firmware/
│   ├── Ironman/        the arm (ESP32): gestures, flight, servos, lights, power, self-test
│   ├── Puck/           the round screen (ESP32-S3): 3D view, control panel, thermal camera
│   ├── PuckUpdater/    the screen's WiFi update mode
│   ├── lib/            the arm-to-screen protocol, and WiFi / over-the-air updates
│   ├── web/            the simulator and telemetry pages the arm serves
│   └── flash.sh        build and flash
├── hardware/           parts, pin map, power, the system overview and the wiring diagram
└── media/              photos and screen captures
```

Some comments in the firmware refer to design notes and earlier test rigs that aren't in this repository.

---

# 📄 Licence

MIT, see [LICENSE](LICENSE).
