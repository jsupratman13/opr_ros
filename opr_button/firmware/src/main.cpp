#include <Arduino.h>

uint8_t g_DATA[4] = { 0x7E, 0x00, 0x00, 0x00 };  // NOLINT

void setup()
{
  pinMode(0, INPUT_PULLUP);  // stop button
  pinMode(1, INPUT_PULLUP);  // start button
  Serial.begin(9600);
}

void loop()
{
  uint8_t stop_button_pressed = !digitalRead(0);
  uint8_t start_button_pressed = !digitalRead(1);

  g_DATA[1] = stop_button_pressed;
  g_DATA[2] = start_button_pressed;
  g_DATA[3] = g_DATA[0] ^ g_DATA[1] ^ g_DATA[2];  // checksum

  Serial.write(g_DATA, 9);

  delay(100);
}
