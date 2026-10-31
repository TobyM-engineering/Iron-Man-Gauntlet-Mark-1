// Puck - round-screen display for the R3 gauntlet (Waveshare ESP32-S3-Touch-LCD-1.28).
// The code lives in app.cpp; knobs in config.h; the arm model in model_data.h (tools/bake_puck.py).
// Display only: this board never commands a servo.

void app_setup();
void app_loop();

void setup() { app_setup(); }
void loop() { app_loop(); }
