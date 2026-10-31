// Copy to secrets.h (gitignored) and fill in the networks the puck's update mode may join - it takes the one it hears
// strongest. Only the first is needed; the puck lives in the arm with no USB, so put every network it may meet here
// (home, a phone hotspot, the venue) BEFORE it goes in: changing them needs a USB flash of the updater.
#pragma once
#define WIFI_SSID "your-network"
#define WIFI_PASS "your-password"
// #define WIFI_SSID2 "your-phone-hotspot"
// #define WIFI_PASS2 "its-password"
// #define WIFI_SSID3 "the-venue"
// #define WIFI_PASS3 "its-password"
