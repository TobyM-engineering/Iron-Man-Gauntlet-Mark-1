// config.h - the knobs. Everything you might need to change is here.
#pragma once

// ---------------------------------------------------------------- UART link from the main ESP32
// Puck pin that receives the main board's TX (wire: main GPIO18 -> this pin, GND -> GND).
// GPIO16 = pin 8 of the board's 12-pin SH1.0 header P2 (Waveshare ESP32-S3-Touch-LCD-1.28 schematic:
// 1 GND, 2 VSYS, 3 RUN, 4 BOOT/GPIO0, 5 GND, 6 3V3, 7 GPIO15, 8 GPIO16, 9 GPIO17, 10 GPIO18, 11 GPIO21, 12 GPIO33).
// Nothing on the board uses it (wiki pin table: LCD 2/8/9/10/11/14, touch + IMU 3/4/5/6/7/13, battery 1, USB 43/44),
// and it is the pin ironman-gauntlet's design (wire E14) and its Display/ sketch already chose. USB serial stays free:
// typed $S lines and test commands go in there.
#define PUCK_RX_PIN        16          // header pin 8  <- arm GPIO 18 (its UART2 TX)
#define PUCK_TX_PIN        15          // header pin 7  -> arm GPIO 19 (its UART2 RX): the puck's $H heartbeat
#define LINK_BAUD          115200      // GauntletLink: the main board's UART2 at 115200
#define LINK_TIMEOUT_MS    1500        // the older 40-byte packet (browser preview only)

// ---------------------------------------------------------------- the show bus (header pins 5 GND, 6 3V3, 9 SDA, 10 SCL)
// MLX90640 thermal camera (0x33) + SSD1306 OLED (0x3C) on the S3's second I2C controller (Wire1), read and drawn by
// a task on core 0 so the renderer on core 1 never waits for them. 4.7 k pull-ups to pin 6's 3V3 (design 04 §6).
#define SHOW_SDA           17          // header pin 9
#define SHOW_SCL           18          // header pin 10
#define SHOW_BUS_HZ        400000      // the MLX90640 wants 400 kHz (1 MHz at most); the SSD1306 is fine with it
#define MLX_ADDR           0x33
#define OLED_ADDR          0x3C
#define OLED_PERIOD_MS     250         // the OLED redraws 4x a second
#define SHOW_PROBE_MS      2000        // an absent device is looked for again this often

// ---------------------------------------------------------------- display (Waveshare ESP32-S3-Touch-LCD-1.28)
#define SPI_MHZ            80          // noise/sparkles on screen -> 40
#define PANEL_INVERT       true        // colours inverted (black<->white) -> flip
#define PANEL_RGB_ORDER    false       // red and blue swapped -> flip
#define STRIP_H            48          // rows per DMA strip (divides 240)
#ifndef PUCK_STYLE
#define PUCK_STYLE         1           // 0 round 2, 1 A red HUD (Toby's pick, 2026-10-07), 2 B arc-reactor HUD, 3 C candy metal (view.h)
#endif
#ifndef CREASE_LEVEL
#define CREASE_LEVEL       1           // inner creases everywhere but the fingers (view.h crease_level; Toby 2026-10-11)
#endif
#define LINE_CAP           1600        // max glowing edge lines per frame, half a core (styles A / B; 12 bytes each - 1200 x 16 until 2026-10-10)
#ifndef PUCK_BODY_TONE
#define PUCK_BODY_TONE     0           // the body colour, red edges kept: 0 the style's gunmetal, 1 the link-lost grey, 2 a lighter grey (Toby to pick, 2026-10-10)
#endif
#define LOD_ZOOM           0.5f       // past this zoom (0 hand view .. 1 whole gauntlet) draw the light detail level
#define SHADING_SMOOTH     1
          // 1 = smooth (Gouraud, lit per vertex), 0 = flat (one colour per triangle)
#define TRI_CAP            4000        // max visible triangles per frame (RAM: 30 bytes each); the 6100-triangle bake shows
                                       // up to ~3100 at once. Boot takes less if RAM is short. 8000-triangle bake: 5800
#define RAM_RESERVE        24000       // bytes of heap always left free after the triangle buffer

// ---------------------------------------------------------------- touch (CST816S) and IMU (QMI8658), shared I2C
#define I2C_SDA            6
#define I2C_SCL            7
#define TOUCH_RST          13
#define TOUCH_SWAP_XY      0           // drag moves the view the wrong way: try these
#define TOUCH_FLIP_X       0
#define TOUCH_FLIP_Y       0
#define ORBIT_RAD_PER_PX   0.012f      // drag sensitivity
#define DOUBLE_TAP_MS      350
#define LONG_PRESS_MS      600         // hold = zoom out to the whole arm (no touch ever zeroes the IMUs)

// demo tilt: sensor axes -> screen axes (x right, y up, z out of the glass). Measured on Toby's puck
// (2026-10-06): face up reads z = -1 g, top edge down reads sensor x = +1 g, right edge down reads sensor
// y = +1 g, so screen x = -sensor y, screen y = -sensor x, screen z = -sensor z (the chip sits turned 90 deg,
// face down). Applied as g = (SIGN_X * (SWAP ? ay : ax), SIGN_Y * (SWAP ? ax : ay), SIGN_Z * az).
#define IMU_SIGN_X         -1
#define IMU_SIGN_Y         -1
#define IMU_SIGN_Z         -1
#define IMU_SWAP_XY        1
#define DEMO_LEVEL_SAMPLES 15          // "level" = the puck's own pose over its first ~0.5 s (long press re-levels)
#define DEMO_TILT_MAX_DEG  55.0f       // the hand never tilts further than this from straight

// ---------------------------------------------------------------- live hand orientation
// Sensor-axis remap, same meaning as the simulator's IMU presets (web/src/config.js): columns are where
// the sensor X, Y, Z axes point in the ARM frame (a = along the arm to the fingers, d = dorsal, l = lateral).
// Default = the sim's default "identity" preset.
#define IMU_REMAP { {1, 0, 0}, {0, 1, 0}, {0, 0, 1} }

// ---------------------------------------------------------------- camera
#define CAM_FOCAL_PX       260.0f      // 240 px screen: ~50 deg field of view
#define HAND_FILL_PX       92.0f       // hand view: the hand's outline reaches this far from the centre - the band out to the glass is the HUD's
#define FULL_FIT           0.92f       // whole-arm view (hold): everything the arm reaches from the hand fills this much of the glass

#define ZOOM_TIME_S        0.45f       // zoom in/out smoothing time constant
#define PLATES_OPEN_FRAC   0.25f       // forearm plate past this fraction of its travel = "plates open"
#define PLATES_CLOSED_FRAC 0.12f

// ---------------------------------------------------------------- demo
#define DEMO_PLATE_PERIOD_S 10.0f      // plates open for this long, then closed for this long

// ---------------------------------------------------------------- colours: HandBench's materials, lightened for a 240 px screen
// LINEAR albedo (sRGB hex converted); metalness / roughness / clear coat are in view.h, lights in render.h.
// (2026-10-06 used HandBench's near-black 0x121317 armour; on the puck the shapes vanished.)
#define MAT_ARMOR_RGB      {0.0802f, 0.0908f, 0.1144f}   // 0x50555f light gunmetal armour: forearm shells, plate 3, hand plate
#define MAT_SHELL_RGB      {0.0595f, 0.0648f, 0.0844f}   // 0x454850 gunmetal hand and finger shells
#define MAT_GUN_RGB        {0.0242f, 0.0262f, 0.0331f}   // 0x2b2d33 dark gunmetal mechanism (linkage arms, servos)
#define MAT_UNDER_RGB      {0.0110f, 0.0129f, 0.0176f}   // 0x1c1e24 under-suit (glove)
#define MAT_RED_RGB        {0.4621f, 0.0070f, 0.0123f}   // 0xb5141e HandBench's metallic red: plates 1-2, fingertips
