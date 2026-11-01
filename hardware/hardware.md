# Hardware

Everything here is also in [`firmware/Ironman/config.h`](../firmware/Ironman/config.h), which is the single source of truth: the firmware refuses to compile if a pin is booked twice, a strapping or flash pin is used, or two devices share an I²C address on one bus.

Every wire, pin, resistor and capacitor is in the **[wiring app](https://tobym-engineering.github.io/Iron-Man-Gauntlet-Mark-1/hardware/wiring-app.html)** (`wiring-app.html` in this folder: open it in a browser, it works offline too).

## Parts

| Part | Job | Notes |
|---|---|---|
| ESP32 WROOM-32 devkit (30-pin) | Main controller, in the forearm | Arduino framework, `min_spiffs` partitions for over-the-air updates |
| BETAFPV ELRS Nano TX V2 | The radio | Fed straight from the pack (it takes 7–13 V, not 5 V). Never power it without its antenna |
| BetaFPV Air65 | The drone | Betaflight, angle mode, stock tune |
| 2× BNO085 (GY-BNO08X boards) | Hand and forearm orientation | Game rotation vector at 50 Hz, one on each I²C bus |
| 5× Adafruit #182 flex sensors | The fingers | 47 kΩ dividers; thumb on an ESP32 ADC pin, four fingers on the ADS1115 |
| ADS1115 | Four finger sensors | 860 samples/s, round robin, in the hand |
| Adafruit PCA9685 (#815) | Six servos + every on/off output | Output-enable pulled up through 1 kΩ: everything off until the firmware runs |
| 6× MG90S servos | Plates 1 and 2 (one each), plate 3 and the hand plate (two each, mirrored) | Each servo's open / closed end set on its real plate |
| INA219 | Current into the servo branch | Releases a jammed plate |
| Waveshare ESP32-S3-Touch-LCD-1.28 | The round touch screen | Talks to the main board over UART; owns its own I²C bus for the camera and the OLED |
| MLX90640 (Adafruit #4407) | Thermal camera | Under the hand plate, on the screen's bus |
| SSD1306 0.96″ OLED | Status | On the screen's bus |
| Garmin LIDAR-Lite v3 (Adafruit #4058) | Distance | Under the hand plate; asleep whenever the red laser is off |
| Quarton VLM-520-74 LPA | Green laser, out of plate 1 | Switched low-side by an RFP30N06LE MOSFET |
| Adafruit #1056 | Red laser, under the hand plate | TTL input straight from a PCA9685 channel |
| WS2812 12-LED ring + 7-LED disc | Palm repulsor | Brightness capped at 30 % |
| WS2812 strip | Forearm | Capped at 30 % |
| 3× XM-L2 (20 mm) + 4× Luxeon 10 mm | High-power palm and fingertip LEDs | One LEDdynamics BuckPuck (1 A) each, switched in three groups by D4184 MOSFET modules |
| DRV2605L (Adafruit #2305) + vibration motor | Haptic feedback | On the hand's I²C bus |
| Passive buzzer module | Alerts | |
| 3-position slide switch | Mode: SAFE / SHOW / FULL | Middle = SAFE, so a broken wire is safe |
| Ovonic 2S 1100 mAh 60C LiHV | Power | 8.7 V full |
| 10 A fast-blow fuse, IRF4905 P-MOSFET + slide switch | Protection and master switch | |
| Pololu D24V50F5 (5 V, 5 A) | Servo rail | Servos, LED strip, palm ring, green laser |
| Pololu D24V22F5 (5 V, 2.5 A) | Logic rail | ESP32, round screen, LiDAR |

All junctions are wires twisted, soldered and heat-shrunk — no perfboard, no screw terminals. 37 wires cross the wrist (20 AWG for power, 22 AWG for everything else).

## Pin map (main ESP32)

| GPIO | Function |
|---|---|
| 21 / 22 | I²C bus 1 (forearm): PCA9685 `0x40`, forearm BNO085 `0x4A`, INA219 `0x41` |
| 16 / 17 | I²C bus 2 (hand, across the wrist): hand BNO085 `0x4A`, ADS1115 `0x48`, LiDAR `0x62`, DRV2605L `0x5A` |
| 23 | PCA9685 output-enable (1 kΩ pull-up) |
| 25 | CRSF to the ELRS module — one wire, half duplex, 400 000 baud |
| 13 | Forearm LED strip data (330 Ω) |
| 27 | Palm ring data (330 Ω) |
| 18 / 19 | UART2 to / from the round screen |
| 34 | Thumb flex sensor |
| 35 | Battery voltage (100 kΩ / 22 kΩ divider) |
| 36 / 39 | Mode switch, polled |
| 32 / 33 | BNO085 interrupt lines (forearm / hand) |
| 26 | Buzzer |
| 4 | Three red accent LEDs in the palm |

Never used: GPIO 12 (a strapping pin — high at boot stops it starting), 0 / 2 / 15 (straps), 6–11 (flash), 1 / 3 (console). All analog inputs are on ADC1, because ADC2 stops working while WiFi is on.

## PCA9685 channels (as designed)

| Channel | Output |
|---|---|
| 0, 1 | Plate 1, plate 2 |
| 2, 3 | Plate 3, servos A and B |
| 4, 5 | Hand plate, servos A and B |
| 6 | Green laser MOSFET gate |
| 7 | Red laser TTL input |
| 9, 10, 11 | High-power LED groups: palm 20 mm, palm square, fingertips |

Channels can be remapped on a built arm without recompiling (`/pcamap?set=6:13` on the arm's setup page, kept in flash).

## Power

```
2S LiHV 1100 mAh ── 10 A fuse ── IRF4905 master switch ──┬── D24V50F5 → 5 V servo rail (via the INA219)
                                                          ├── D24V22F5 → 5 V logic rail → ESP32 → 3.3 V for the sensors
                                                          ├── ELRS Nano TX (battery voltage)
                                                          ├── BuckPuck LED drivers (battery voltage)
                                                          └── 100 kΩ / 22 kΩ divider → GPIO 35
```

**Grounding:** the "muscle" (regulators, the PCA9685's servo terminal, the radio, the LED drivers, the LED strips) returns to one star point; the "brains" (the sensors, the flex dividers, the PCA9685's logic header) return to the ESP32's ground.

| Battery | What happens |
|---|---|
| under 7.5 V | The drone will not arm (the radio module needs 7 V) |
| 7.3 V, armed | Vibration: land now |
| 7.0 V | A short chirp every 5 s; no high-power light show |
| 6.6 V | Five-second grace, then everything off and deep sleep |

**Never connect USB and the battery at the same time** — the devkit has no USB diode, so two supplies would meet on its 5 V pin.
