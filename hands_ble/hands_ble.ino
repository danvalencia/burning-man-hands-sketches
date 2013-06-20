/*
  This example will turn ON/OFF LED on pin 13 when receiving
  command 0x01/0x00.
*/

#include <SPI.h>
#include "Adafruit_WS2801.h"
#include <ble.h>

/* PIN for LED */
#define PIN    13

/* Command: ON/OFF */
#define ON     0x01
#define OFF    0x00

#define CHANGE_COLOR 0x01

int dataPin  = 2;    // white/green wire
int clockPin = 3;    // red wire

Adafruit_WS2801 strip = Adafruit_WS2801(20, dataPin, clockPin);


void setup()
{
  pinMode(PIN, OUTPUT);
  
  SPI.setDataMode(SPI_MODE0);
  SPI.setBitOrder(LSBFIRST);
  SPI.setClockDivider(SPI_CLOCK_DIV16);
  SPI.begin();
  
  Serial.begin(115200);
   while (!Serial) {
      ; // wait for serial port to connect. Needed for Leonardo only
  }
  
  strip.begin();

  // Update LED contents, to start they are all 'off'
  strip.show();


  ble_begin();
}

void loop()
{
  if (ble_available())
  {
    Serial.println("BLE is Available!");
    byte command = ble_read();
    byte red = ble_read();
    byte green = ble_read();
    byte blue = ble_read();
    
    Serial.println(command, HEX);

    switch(command)
    {
       case(CHANGE_COLOR):
         colorWipe(Color(red, green, blue), 50);
         Serial.println(red, HEX);
         Serial.println(green, HEX);
         Serial.println(blue, HEX);
         break;
       default:
         break;
    }
  }
  
  ble_do_events();
}

// fill the dots one after the other with said color
// good for testing purposes
void colorWipe(uint32_t c, uint8_t wait) {
  int i;
  
  for (i=0; i < strip.numPixels(); i++) {
      strip.setPixelColor(i, c);
      strip.show();
      delay(wait);
  }
}

/* Helper functions */

// Create a 24 bit color value from R,G,B
uint32_t Color(byte r, byte g, byte b)
{
  uint32_t c;
  c = r;
  c <<= 8;
  c |= g;
  c <<= 8;
  c |= b;
  return c;
}

