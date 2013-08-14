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
#define UPDATE_PIXEL 0x03

#define NUM_STRIPS 8  

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

typedef struct {
   byte x;
   byte y;
} Pixel;


Pixel thePix;

Pixel grid[][11] = {
  {{-1, -1}, {-1, -1}, {1, 0},   {2, 0},   {3, 0},  {4, 0},  {5, 1}, {6, 0},   {-1, -1}, {-1, -1}, {-1, -1}},
  {{-1, -1}, {-1, -1}, {1, 1},   {2, 1},   {3, 1},  {4, 1},  {5, 2}, {6, 1},   {-1, -1}, {-1, -1}, {-1, -1}},
  {{-1, -1}, {-1, -1}, {1, 2},   {2, 2},   {3, 2},  {4, 2},  {5, 3}, {6, 2},   {-1, -1}, {-1, -1}, {-1, -1}},
  {{-1, -1}, {-1, -1}, {1, 3},   {2, 3},   {3, 3},  {4, 3},  {5, 4}, {6, 3},   {-1, -1}, {-1, -1}, {-1, -1}},
  {{-1, -1}, {-1, -1}, {1, 4},   {2, 4},   {3, 4},  {4, 4},  {5, 5}, {-1, -1}, {-1, -1}, {-1, -1}, {-1, -1}},
  {{-1, -1}, {-1, -1}, {1, 5},   {2, 5},   {3, 5},  {4, 5},  {5, 6}, {-1, -1}, {-1, -1}, {-1, -1}, {-1, -1}},
  {{-1, -1}, {-1, -1}, {1, 6},   {2, 6},   {3, 6},  {4, 6},  {5, 7}, {-1, -1}, {-1, -1}, {-1, -1}, {-1, -1}},
  {{-1, -1}, {-1, -1}, {1, 7},   {2, 7},   {3, 7},  {4, 7},  {5, 8}, {-1, -1}, {-1, -1}, {-1, -1}, {-1, -1}},
  {{-1, -1}, {-1, -1}, {1, 8},   {2, 8},   {3, 8},  {4, 8},  {5, 9}, {-1, -1}, {-1, -1}, {-1, -1}, {-1, -1}},
  {{-1, -1}, {-1, -1}, {1, 9},   {2, 9},   {3, 9},  {4, 9},  {5, 10},{-1, -1}, {-1, -1}, {-1, -1}, {-1, -1}},
  {{-1, -1}, {-1, -1}, {1, 10},  {2, 10},  {3, 10}, {4, 10}, {5, 11},{6, 4},   {-1, -1}, {-1, -1}, {-1, -1}},
  {{-1, -1}, {0, 0},   {1, 11},  {2, 11},  {3, 11}, {4, 11}, {5, 12},{6, 5},   {7, 0},   {-1, -1}, {-1, -1}},
  {{-1, -1}, {0, 1},   {1, 12},  {2, 12},  {3, 12}, {4, 12}, {5, 13},{6, 6},   {7, 1},   {7, 3},   {-1, -1}},
  {{0, 4},   {0, 2},   {1, 13},  {2, 13},  {3, 13}, {4, 13}, {5, 14},{6, 7},   {7, 2},   {7, 4},   {-1, -1}},
  {{0, 5},   {0, 3},   {1, 14},  {2, 14},  {3, 14}, {4, 14}, {5, 15},{6, 8},   {-1, -1}, {7, 5},   {7, 6}},
  {{0, 6},   {-1, -1}, {1, 15},  {2, 15},  {3, 15}, {4, 15}, {5, 16},{-1, -1}, {-1, -1}, {-1, -1}, {7, 7}},
  {{0, 7},   {-1, -1}, {1, 16},  {-1, -1}, {3, 16}, {-1, -1},{5, 17},{-1, -1}, {-1, -1}, {-1, -1}, {7, 8}},
  {{0, 8},   {-1, -1}, {1, 17},  {-1, -1}, {3, 17}, {-1, -1},{5, 18},{-1, -1}, {-1, -1}, {-1, -1}, {-1, -1}},
  {{-1, -1}, {-1, -1}, {1, 18},  {-1, -1}, {3, 18}, {-1, -1},{5, 19},{-1, -1}, {-1, -1}, {-1, -1}, {-1, -1}},
  {{-1, -1}, {-1, -1}, {1, 19},  {-1, -1}, {3, 19}, {-1, -1},{5, 20},{-1, -1}, {-1, -1}, {-1, -1}, {-1, -1}},
  {{-1, -1}, {-1, -1}, {1, 20},  {-1, -1}, {3, 20}, {-1, -1},{5, 21},{-1, -1}, {-1, -1}, {-1, -1}, {-1, -1}},
  {{-1, -1}, {-1, -1}, {-1, -1}, {-1, -1}, {3, 21}, {-1, -1},{-1, -1},{-1, -1},{-1, -1}, {-1, -1}, {-1, -1}},
  {{-1, -1}, {-1, -1}, {-1, -1}, {-1, -1}, {3, 22}, {-1, -1},{-1, -1},{-1, -1},{-1, -1}, {-1, -1}, {-1, -1}},
};

Adafruit_WS2801 strip_array[] = {strip_1, strip_2, strip_3, strip_4, strip_5, strip_6, strip_7, strip_8};


void setup()
{
  //pinMode(PIN, OUTPUT);
    
  Serial.begin(115200);
  while (!Serial) {
      ; // wait for serial port to connect. Needed for Leonardo only
  }
  
  for(int i = 0; i < NUM_STRIPS; i++)
  {
    strip = strip_array[i];
    strip.begin();
    strip.show();
  }
  

  for(byte i = 0; i < 23; i++)
  {
    for(byte j = 0; j < 11; j++)
    {
      thePix = (Pixel)grid[i][j];
      if(thePix.x != 255)
      {
        Serial.println(thePix.x);
        Serial.println(thePix.y);

        strip = strip_array[thePix.x];
        strip.setPixelColor(thePix.y, Color(0,255,0));
        strip.show();
        delay(50);
        
        
      }
      
    } 
  }
   
  initialTest();
  
  setupBluetooth();
}

void loop()
{
  if (ble_available())
  {
    Serial.println("BLE is Available!");
    command = ble_read();
    
    Serial.println(command, HEX);

    switch(command)
    {
       case(CHANGE_COLOR):
         red = ble_read();
         green = ble_read();
         blue = ble_read();
         latestCommand = CHANGE_COLOR;
         Serial.println(red, HEX);
         Serial.println(green, HEX);
         Serial.println(blue, HEX);
         break;
       case(RAINBOW):
         delayInMillis = ble_read();
         latestCommand = RAINBOW;
         break;
       case(UPDATE_PIXEL):
         red = ble_read();
         green = ble_read();
         blue = ble_read();
         xpos = ble_read();
         ypos = ble_read();
         latestCommand = UPDATE_PIXEL;
         strip = strip_array[xpos - 1];
         Serial.println(red, HEX);
         Serial.println(green, HEX);
         Serial.println(blue, HEX);
         Serial.println(xpos, HEX);
         Serial.println(ypos, HEX);
         break;

       default:
         break;
    }
  }
  
  doCommand();
  
  ble_do_events();
}

void initialTest()
{
  for(int i; i < NUM_STRIPS; i++)
  {
     strip = strip_array[i];
     colorWipe(Color(255, 0, 0), 50);
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
         colorWipe(Color(red, green, blue), 0);
         //updateColor(Color(red, green, blue));
         break;
      case(RAINBOW):
         rainbow(delayInMillis);
         break;
      case(UPDATE_PIXEL):
         updateColor(Color(red, green, blue));
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

