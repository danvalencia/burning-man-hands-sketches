/*
  This example will turn ON/OFF LED on pin 13 when receiving
  command 0x01/0x00.
*/

#include <SPI.h>
#include "Adafruit_WS2801.h"
#include <ble.h>
#include <SimpleTimer.h>


/* PIN for LED */
#define PIN    13

/* Command: ON/OFF */
#define ON     0x01
#define OFF    0x00

#define VERTICAL_LOOP   0x01
#define HORIZONTAL_LOOP 0x02
#define UPDATE_PIXEL    0x03
#define RAINBOW         0x04
#define LASER           0x05
#define COLOR_FADE      0x06
#define RANDOMNESS      0x07
#define SHUFFLE         0x08

#define NUM_STRIPS 8 

#define NUM_COLS 11
#define NUM_ROWS 23

#define DIVISOR (256/NUM_STRIPS);

#define YES 1
#define NO 0

#define RIGHT 0
#define LEFT 1
#define UP 2
#define DOWN 3

byte latestCommand = 0x00;

byte red = 0x00;
byte green = 0x00;
byte blue = 0x00;

byte xpos = 0x00;
byte ypos = 0x00;

byte command = COLOR_FADE;
byte delayInMillis = 0x00;

byte byteIndex = 0;

int timerId;

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

typedef struct {
   byte x;
   byte y;
} Position;


Pixel thePix;

Pixel grid[][11] = {
  {{-1, -1}, {-1, -1}, {-1, -1},   {2, 0},   {3, 0},  {4, 0},  {5, 0}, {6, 0},   {-1, -1}, {-1, -1}, {-1, -1}},
  {{-1, -1}, {-1, -1}, {1, 0},   {2, 1},   {3, 1},  {4, 1},  {5, 1}, {6, 1},   {-1, -1}, {-1, -1}, {-1, -1}},
  {{-1, -1}, {-1, -1}, {1, 1},   {2, 2},   {3, 2},  {4, 2},  {5, 2}, {6, 2},   {-1, -1}, {-1, -1}, {-1, -1}},
  {{-1, -1}, {-1, -1}, {1, 2},   {2, 3},   {3, 3},  {4, 3},  {5, 3}, {6, 3},   {-1, -1}, {-1, -1}, {-1, -1}},
  {{-1, -1}, {-1, -1}, {1, 3},   {2, 4},   {3, 4},  {4, 4},  {5, 4}, {-1, -1}, {-1, -1}, {-1, -1}, {-1, -1}},
  {{-1, -1}, {-1, -1}, {1, 4},   {2, 5},   {3, 5},  {4, 5},  {5, 5}, {-1, -1}, {-1, -1}, {-1, -1}, {-1, -1}},
  {{-1, -1}, {-1, -1}, {1, 5},   {2, 6},   {3, 6},  {4, 6},  {5, 6}, {-1, -1}, {-1, -1}, {-1, -1}, {-1, -1}},
  {{-1, -1}, {-1, -1}, {1, 6},   {2, 7},   {3, 7},  {4, 7},  {5, 7}, {-1, -1}, {-1, -1}, {-1, -1}, {-1, -1}},
  {{-1, -1}, {-1, -1}, {1, 7},   {2, 8},   {3, 8},  {4, 8},  {5, 8}, {-1, -1}, {-1, -1}, {-1, -1}, {-1, -1}},
  {{-1, -1}, {-1, -1}, {1, 8},   {2, 9},   {3, 9},  {4, 9},  {5, 9},{-1, -1}, {-1, -1}, {-1, -1}, {-1, -1}},
  {{-1, -1}, {-1, -1}, {1, 9},  {2, 10},  {3, 10}, {4, 10}, {5, 10},{6, 4},   {-1, -1}, {-1, -1}, {-1, -1}},
  {{-1, -1}, {0, 0},   {1, 10},  {2, 11},  {3, 11}, {4, 11}, {5, 11},{6, 5},   {7, 0},   {-1, -1}, {-1, -1}},
  {{-1, -1}, {0, 1},   {1, 11},  {2, 12},  {3, 12}, {4, 12}, {5, 12},{6, 6},   {7, 1},   {7, 3},   {-1, -1}},
  {{0, 4},   {0, 2},   {1, 12},  {2, 13},  {3, 13}, {4, 13}, {5, 13},{6, 7},   {7, 2},   {7, 4},   {-1, -1}},
  {{0, 5},   {0, 3},   {1, 13},  {2, 14},  {3, 14}, {4, 14}, {5, 14},{6, 8},   {-1, -1}, {7, 5},   {7, 6}},
  {{0, 6},   {-1, -1}, {1, 14},  {2, 15},  {3, 15}, {4, 15}, {5, 15},{-1, -1}, {-1, -1}, {-1, -1}, {7, 7}},
  {{0, 7},   {-1, -1}, {1, 15},  {-1, -1}, {3, 16}, {-1, -1},{5, 16},{-1, -1}, {-1, -1}, {-1, -1}, {7, 8}},
  {{0, 8},   {-1, -1}, {1, 16},  {-1, -1}, {3, 17}, {-1, -1},{5, 17},{-1, -1}, {-1, -1}, {-1, -1}, {-1, -1}},
  {{-1, -1}, {-1, -1}, {1, 17},  {-1, -1}, {3, 18}, {-1, -1},{5, 18},{-1, -1}, {-1, -1}, {-1, -1}, {-1, -1}},
  {{-1, -1}, {-1, -1}, {1, 18},  {-1, -1}, {3, 19}, {-1, -1},{5, 19},{-1, -1}, {-1, -1}, {-1, -1}, {-1, -1}},
  {{-1, -1}, {-1, -1}, {1, 19},  {-1, -1}, {3, 20}, {-1, -1},{5, 20},{-1, -1}, {-1, -1}, {-1, -1}, {-1, -1}},
  {{-1, -1}, {-1, -1}, {-1, -1}, {-1, -1}, {3, 21}, {-1, -1},{-1, -1},{-1, -1},{-1, -1}, {-1, -1}, {-1, -1}},
  {{-1, -1}, {-1, -1}, {-1, -1}, {-1, -1}, {3, 22}, {-1, -1},{-1, -1},{-1, -1},{-1, -1}, {-1, -1}, {-1, -1}},
};

Adafruit_WS2801 strip_array[] = {strip_1, strip_2, strip_3, strip_4, strip_5, strip_6, strip_7, strip_8};

int pin = 13;
volatile byte state = LOW;

byte prevState = LOW;
byte turn = 0;

Position laserPosition = {4 , 3};
byte laserDirection = UP;

SimpleTimer timer;

boolean firstTime = true;
byte shuffleInterval;

byte randomDirection = UP;

void setup()
{
  pinMode(pin, OUTPUT);
  //attachInterrupt(0, switchMode, CHANGE);
    
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
    
  setupBluetooth();
  
  randomSeed(analogRead(0));
  horizontalLoop(Wheel(random(255)), YES);
  clearGrid();
  
  timerId = timer.setInterval(60000, nextTurn);
}

void nextTurn() {
    turn = turn + 1;
    Serial.print("Next Turn: "); 
    Serial.println(turn);
}

void loop()
{
    timer.run();  
    byteIndex++;
    if(state)
    {
       switch(turn) {
           case 0:
             randomNesss();
             delay(50);
             if(byteIndex >= 255) {
               nextMode(1);
             }
             break;
           case 1:
             verticalLoop(Wheel(random(255)), YES);
             verticalLoop(Wheel(random(255)),YES);
             verticalLoop(Wheel(random(255)), YES);
             verticalLoop(Wheel(random(255)),YES);
             clearGrid();
             nextMode(2);
             break;
           case 2:
             horizontalLoop(Wheel(random(255)), YES);
             horizontalLoop(Wheel(random(255)), YES);
             horizontalLoop(Wheel(random(255)), YES);
             horizontalLoop(Wheel(random(255)), YES);
             nextMode(3);
             clearGrid();
             break;
           case 3:
             colorWipe(Wheel(byteIndex++),0);
             if(byteIndex >= 255) {
               nextMode(4);
               clearGrid();
             }
             break;
           default:
             laser();
             if(byteIndex >= 255) {
               nextMode(0);
             }
             break;
       }       
    }
    else
    {
        if (ble_available())
        {
          Serial.println("BLE!");
          command = ble_read();
          Serial.println(command, HEX);
      
          switch(command)
          {
             case(VERTICAL_LOOP):
               red = ble_read();
               green = ble_read();
               blue = ble_read();
               latestCommand = VERTICAL_LOOP;
               Serial.println(red, HEX);
               Serial.println(green, HEX);
               Serial.println(blue, HEX);
               break;
             case(HORIZONTAL_LOOP):
               red = ble_read();
               green = ble_read();
               blue = ble_read();
               latestCommand = HORIZONTAL_LOOP;
               Serial.println(red, HEX);
               Serial.println(green, HEX);
               Serial.println(blue, HEX);
               break;
             case(LASER):
               latestCommand = LASER;
               break;
             case(COLOR_FADE):
               latestCommand = COLOR_FADE;
               break;
             case(RANDOMNESS):
               latestCommand = RANDOMNESS;
               break;
             case(SHUFFLE):
               shuffleInterval = ble_read();
               Serial.print("Interval:");
               Serial.println(shuffleInterval);
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
               strip = strip_array[xpos];
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
}

void switchMode()
{
  state = !state;
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
      case(VERTICAL_LOOP):
         verticalLoop(Wheel(byteIndex), YES);
         clearGrid();

         //colorWipe(Color(red, green, blue), 0);
         //updateColor(Color(red, green, blue));
         break;
      case(HORIZONTAL_LOOP):
         horizontalLoop(Wheel(byteIndex), YES);
         clearGrid();

         //colorWipe(Color(red, green, blue), 0);
         //updateColor(Color(red, green, blue));
         break;
//      case(LASER):
//         laser();
//         break;
      case(COLOR_FADE):
         colorWipe(Wheel(byteIndex),0);
         break;
      case(RANDOMNESS):
         randomNesss();
         break;
      case(RAINBOW):
         rainbow(delayInMillis);
         break;
      case(UPDATE_PIXEL):
         updateColor(Color(red, green, blue));
         break;
      case(SHUFFLE):
         shuffle();
         break;
      default:
         break;   
   }
}

void shuffle() {

   switch(turn) {
       case 0:
         colorWipe(Wheel(byteIndex++),0);
         break;
       case 1:
         randomNesss();
         break;
       case 2:
         horizontalLoop(Wheel(random(255)), YES);
         break;
       case 3:
         verticalLoop(Wheel(random(255)), YES);
         break;
//       case 4:
//         //laser();
//         break;
       default:
         turn = 0; //start over
         break;
   }
   latestCommand = SHUFFLE;
}

void updateColor(uint32_t color) {
   strip.setPixelColor(ypos, color);
   strip.show();
}
// fill the dots one after the other with said color
// good for testing purposes
void colorWipe(uint32_t c, uint8_t wait) {
    int i;
    
    for (byte k=0; k<NUM_STRIPS; k++)
    {
      strip = strip_array[k];
      for (i=0; i < strip.numPixels(); i++) {
          strip.setPixelColor(i, c);
          strip.show();
          delay(wait);
      }
    }
}

void nextMode(byte mode) {
    delay(300);
    turn = mode;  
}

void randomNesss() {
    long randomX = random(22);
    long randomY = random(10);
    long color = Wheel(random(255));
    setPixelColor(randomX, randomY, color, YES);
    delay(random(100));
    setPixelColor(randomX, randomY, Color(0,0,0), YES);     
}

void clearGrid() {
    for(byte i = 0; i<NUM_ROWS; i++)
    {
        for(byte j=0; j<NUM_COLS; j++)
        {
            setPixelColor(i, j, 0, YES);  
        }
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
  for (byte j=0; j < 256; j++) {     // 3 cycles of all 256 colors in the wheel
    verticalLoop(Wheel(j), NO);
    showStrips();
    delay(wait);
  }
}

void horizontalLoop(uint32_t color, byte shouldDisplay)
{
  for(byte i = 0; i < 23; i++)
  {
    for(byte j = 0; j < 11; j++)
    {
      setPixelColor(i, j, color, shouldDisplay);
    } 
    delay(200);
  }
}

void verticalLoop(uint32_t color, byte shouldDisplay)
{
  for(byte j = 0; j < 11; j++)
  {
    for(byte i = 0; i < 23; i++)
    {
      setPixelColor(i,j,color,shouldDisplay);
    } 
    delay(200);
  }
}

void setPixelColor(byte x, byte y, uint32_t color, byte shouldDisplay)
{
   Pixel thePixel = (Pixel)grid[x][y]; 

   if(thePixel.x != 255)
   {
        strip = strip_array[thePixel.x];
        strip.setPixelColor(thePixel.y, color);
        if(shouldDisplay) {
          strip.show();        
        }
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

void laser() {
  drawLaser();
  determineNextPosition();
}

void determineNextPosition() {
  byte finalPos;
  if(laserDirection == UP) {
     finalPos = laserPosition.y + 1;
  }else{
     finalPos = laserPosition.y - 1;
  } 

  Pixel nextPixel = (Pixel)grid[finalPos][laserPosition.x];
  if(nextPixel.x == 255) {
    if(laserDirection == UP)
    {
      laserDirection = DOWN;    
    }else{
      laserDirection = UP;    
    }
  }else{
    if(laserDirection == UP)
    {
      laserPosition.y += 1;
    }else{
      laserPosition.y -= 1;
    }
  }
}

void drawLaser() {
   if(laserDirection == UP){
     setPixelColor( laserPosition.y,laserPosition.x, Color(0,255,0), YES);        
     setPixelColor(laserPosition.y-1,laserPosition.x, Color(0,255,0), YES);        
     setPixelColor( laserPosition.y-2,laserPosition.x, Color(0,255,0), YES);        
     setPixelColor( laserPosition.y-3,laserPosition.x, Color(0,255,0), YES);        
     setPixelColor(laserPosition.y-4, laserPosition.x, Color(0,0,0), YES); 
   }else{
     setPixelColor( laserPosition.y,laserPosition.x, Color(0,255,0), YES);        
     setPixelColor(laserPosition.y+1,laserPosition.x,  Color(0,255,0), YES);        
     setPixelColor( laserPosition.y+2,laserPosition.x, Color(0,255,0), YES);        
     setPixelColor( laserPosition.y+3,laserPosition.x, Color(0,255,0), YES);        
     setPixelColor(laserPosition.y+4, laserPosition.x, Color(0,0,0), YES); 
   }
   
   showStrips();      
}  

void showStrips() {
   for(byte i = 0; i < NUM_STRIPS; i++) {
       strip_array[i].show();
   }  
}
