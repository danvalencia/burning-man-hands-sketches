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
#define RAINBOW      0x02

int dataPin  = 2;    // white/green wire
int clockPin = 3;    // red wire

byte latestCommand = 0x01;

byte red = 0x00;
byte green = 0x00;
byte blue = 0x00;

byte xpos = 0x00;
byte ypos = 0x00;


byte command = 0x00;
byte delayInMillis = 0x00;



Adafruit_WS2801 strip;

Adafruit_WS2801 strip_1 = Adafruit_WS2801(9, 30, 31);
Adafruit_WS2801 strip_2 = Adafruit_WS2801(21, 32, 33);
Adafruit_WS2801 strip_3 = Adafruit_WS2801(16, 34, 35);
Adafruit_WS2801 strip_4 = Adafruit_WS2801(23, 36, 37);
Adafruit_WS2801 strip_5 = Adafruit_WS2801(16, 38, 39);
Adafruit_WS2801 strip_6 = Adafruit_WS2801(21, 40, 41);
Adafruit_WS2801 strip_7 = Adafruit_WS2801(9, 42, 43);
Adafruit_WS2801 strip_8 = Adafruit_WS2801(9, 44, 45);

Adafruit_WS2801 strip_array[] = {strip_1, strip_2, strip_3, strip_4, strip_5, strip_6, strip_7, strip_8};


void setup()
{
  pinMode(PIN, OUTPUT);
  
  
  Serial.begin(115200);
   while (!Serial) {
      ; // wait for serial port to connect. Needed for Leonardo only
  }
  
  for(int x; x<8; x++)
  {
    strip_array[x].begin();
    strip_array[x].show();
  }
  //strip.begin();

  // Update LED contents, to start they are all 'off'
  //strip.show();
  
  setupBluetooth();
}

//void loop()
//{
//  if (ble_available())
//  {
//    Serial.println("BLE is Available!");
//    command = ble_read();
//    
//    Serial.println(command, HEX);
//
//    switch(command)
//    {
//       case(CHANGE_COLOR):
//         red = ble_read();
//         green = ble_read();
//         blue = ble_read();
//         xpos = ble_read();
//         ypos = ble_read();
//         latestCommand = CHANGE_COLOR;
//         Serial.println(red, HEX);
//         Serial.println(green, HEX);
//         Serial.println(blue, HEX);
//         Serial.println(xpos, HEX);
//         Serial.println(ypos, HEX);
//         break;
//       case(RAINBOW):
//         delayInMillis = ble_read();
//         latestCommand = RAINBOW;
//         break;
//       default:
//         break;
//    }
//  }
//  
//  strip = strip_array[xpos + 1];
//  doCommand();
//  
//  ble_do_events();
//}

void loop()
{
  for(int x; x<8; x++)
  {
     strip = strip_array[x];
     colorWipe(Color(0xFF, 0x00, 0x00), 100);
  }
}

void setupBluetooth()
{
  SPI.setDataMode(SPI_MODE0);
  SPI.setBitOrder(LSBFIRST);
  SPI.setClockDivider(SPI_CLOCK_DIV16);
  SPI.begin();

  ble_begin();

}

void doCommand()
{

   switch(command)
   {
      case(CHANGE_COLOR):
         //colorWipe(Color(red, green, blue), 0);
         updateColor(Color(red, green, blue));
         break;
      case(RAINBOW):
         rainbow(delayInMillis);
         break;
      default:
         break;   
   }
}

void updateColor(uint32_t color) {
   strip.setPixelColor(ypos, color);
   strip.show();
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

void rainbow(uint8_t wait) {
  int i, j;
   
  for (j=0; j < 256; j++) {     // 3 cycles of all 256 colors in the wheel
    for (i=0; i < strip.numPixels(); i++) {
      strip.setPixelColor(i, Wheel( (i + j) % 255));
    }  
    strip.show();   // write all the pixels out
    delay(wait);
  }
}

//Input a value 0 to 255 to get a color value.
//The colours are a transition r - g -b - back to r
uint32_t Wheel(byte WheelPos)
{
  if (WheelPos < 85) {
   return Color(WheelPos * 3, 255 - WheelPos * 3, 0);
  } else if (WheelPos < 170) {
   WheelPos -= 85;
   return Color(255 - WheelPos * 3, 0, WheelPos * 3);
  } else {
   WheelPos -= 170; 
   return Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
}

