#include <Adafruit_NeoPixel.h>  // for the NeoPixel array
#include "SPI.h"                // the following three headers are all required
#include "Adafruit_GFX.h"       // to drive the TFT
#include "Adafruit_ILI9341.h"

// Bling control 
//
// This program uses code from the Arduino "Serial Event" example as well as
// code from the Adafruit "strandtest" and "graphicstest"examples. It implements 
// a serial protocol for sending commands on the serial line from a roboRIO to 
// this program to control the LEDs on an Adafruit neopixel strip. The protocol is 
// quite simple and is required because the NeoPixel "show" method locks interrupts
// for a long enough period (and there are some lengthy delay loops in other 
// places) that incomming characters can be dropped. To get around this the
// code:
//  - runs a function to display a selected pattern on the LED array,
//  - polls the serial line for an "I" character (a command "Interrupt"
//    from the roboRIO). 
//  - repeat ad infinitum
// When it sees the comand character all pixel processing stops,
// a "ready" character ("R") is sent back to the roboRIO, and the serial line
// is then monitored for a command string of one or more commands in the form:
//   <cmd char><cmd_val>
// the last of which is terminated with a "Z". For example:
//   F2C255D25Z - set the color to 255 and the delay to 25 for function 2
//   E1Z        - start executing function 1
// Once received, the string is parsed, the commands executed, and the code
// goes back into the simple display/serial poll loop described above. The
// currently supported command characters are documented in processCommand
// (below). The protocl looks like this:
//
//    Arduino                   roboRIO
//       |                        |
//       |<-----------------------I
//       |                        |
//       R----------------------->|
//       |                        |
//       |<---------------------<cmd>
//       .                        .
//       .                        .
//       R----------------------->|  (roboRIO waits for command execution to
//       .                        .   complete)
//                  or
//  
//       R----------------------->|
//       |                        |
//       |<---------------------<cmd>
//       |                        |
//       |<-----------------------I  (roboRIO interrupts current command to
//       |                        |   begin executing a new one)
//       R----------------------->| 
//       .                        .

// To Do: add blink and rainbow blink
//      : more detailed comments on debug usage
//      : ifdefs to account for presence/absense of LCD
// For the Adafruit shield, these are the defaults.
#define TFT_DC  9
#define TFT_CS 10

// Adafruit TFT touchscreen object construction
// Use hardware SPI (on Uno, #13, #12, #11) and the above for CS/DC
Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC);

#define STRIP_PIN    6 // The Arduino digital IO pin used to send data to the LED array
#define STATUS_PIN   8 // The status LED pin
#define STRIP_LEN  240 // The number of pixels in the LED strip

// NeoPixel object construction
// Parameter 1 = number of pixels in strip
// Parameter 2 = Arduino pin number (most are valid)
// Parameter 3 = pixel type flags, add together as needed:
//   NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
//   NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
//   NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
//   NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)
Adafruit_NeoPixel strip = Adafruit_NeoPixel(240, STRIP_PIN, NEO_GRB + NEO_KHZ800);

// ToDo: These no longer need to be globals. They should be declared in serialEvent
char     commandChar = ' ';       // the last command from the roboRIO
uint32_t commandVal  = 0;

// A structure to contain the configuration data for a specific bling function
typedef struct {
  uint32_t color;
  uint32_t delay;
  byte     brightness;
  uint16_t pixelStart;
  uint16_t pixelEnd;
  uint32_t repeat;
} blingParms_t;

// The bling functions that can be configured and run via the serial command interface
typedef enum {
  CLEAR,                 // 0
  COLOR_WIPE,            // 1
  COLOR_WIPE_WITH_BLANK, // 2
  THEATRE_CHASE,         // 3
  RAINBOW,               // 4
  THEATRE_CHASE_RAINBOW, // 5
  COLOR_BAR,             // 6
  NUM_FUNCTIONS     
} function_t;

// Global variables
boolean      commandFlag         = false; // a flag indicating that there is a command req from the roboRIO
boolean      serialDebugEnabled  = false; // set this to true to see a bunch of debug stuff on the serial output
boolean      lcdDebugEnabled     = false; // set this to true to see a bunch of debug stuff on the serial output
boolean      doneSent            = false; // a flag to indicate that we have sent the done signal to the roboRIO
function_t   configFunction;              // the bling func we are currently configuring from received commands
function_t   runningFunction;             // the bling function we are currently using on the LED array
uint32_t     repeatCount;                 // the number of remaining times we will execute the current bling func
int          ledState            = LOW;   // ledState used to set the status LED
uint32_t     prevLedMillis       = 0;     // will store last time status LED was updated
uint32_t     prevShowMillis      = 0;     // will store the last time the debug status was output

// the following variables are longs because the time, measured in miliseconds,
// will quickly become a bigger number than can be stored in an int.
long ledInterval  =  500;                 // interval at which to blink (milliseconds)

#define BLU 0x0000FF
#define GRN 0x00FF00
#define RED 0xFF0000
#define YEL 0xFFFF00
#define PUR 0xFF00FF
#define WHT 0xFFFFFF
#define TUR 0x00FFFF

// This table holds configuration data for each bling function we support. It can be modified by sending
// the "F" command to select the function to configure followed by other commands to modify specific
// values for the selected function.

#define DNT 0 // Do not change these
#define UN  0 // Unused - changing these won't make any difference

blingParms_t blingParmsTable [NUM_FUNCTIONS] = {
//   C    D    B    P    Q    R
  {DNT,  UN,  UN,   0, 240,   1}, // 0 - clear
  {BLU,  25,  64,   0, 240,   1}, // 1 - colorWipe
  {GRN,  25,  64,   0, 240,   5}, // 2 - colorWipeWithBlank
  {TUR,  25,  64,   0, 240,   5}, // 3 - theatreChase
  { UN,  25,  64,   0, 240,   5}, // 4 - rainbow
  {YEL,  25,  64,   0, 240,   5}, // 5 - theatreChaseRainbow
  {RED,  25,  64,   0, 240,   5}  // 6 - colorBar
};

// ----------------------------------------------------------------------------- //
// setup
// This runs once at reset time
// ----------------------------------------------------------------------------- //

void setup() {
  // initialize serial:
  Serial.begin(9600);
  
  // Initialize the I/O pin used for the status LED
  pinMode(STATUS_PIN, OUTPUT);
  
  // Initialize the neopixel array
  strip.begin();
  strip.show(); // Initialize all pixels to 'off'

  // Global variable initialization
  runningFunction = CLEAR;
  configFunction  = CLEAR;
  repeatCount     = blingParmsTable[CLEAR].repeat;
  if (lcdDebugEnabled) {
    tft.begin();
    tft.fillScreen(ILI9341_BLACK);
    tft.setTextColor(ILI9341_YELLOW); 
    tft.setTextSize(2);
    tft.setCursor(0, 0);
    tft.setRotation(1);
  }
}

// ----------------------------------------------------------------------------- //
// loop
// This, and serialEvent below, are run one after another in an endless loop
// ----------------------------------------------------------------------------- //

void loop() {
  // Give the outside world some idea of what we are up to ...
  doBlink();
    
  if ((!commandFlag) && (repeatCount > 0)) {
  // No command reception/processing is occurring so we are OK to go ahead
  // and run a bling function
      doBling();
      repeatCount--;
  } else {
      if (!doneSent) {
        if (serialDebugEnabled) {
          Serial.println("Ready");
        } else {
          Serial.print("R");
        }
        doneSent = true; 
      }
  } // end !commandFlag ...
} 

// ----------------------------------------------------------------------------- //
// serialEvent
// A serial event occurs whenever new data comes in the
// hardware serial RX.  This routine is run between each
// time loop() runs, so using delay inside loop can delay
// response.  Multiple bytes of data may be available.
// ----------------------------------------------------------------------------- //

void serialEvent() { 
  while (Serial.available()) {
     commandChar = Serial.read();   
  
     if ('I' == commandChar) {
       commandFlag = true;
       if (lcdDebugEnabled) {
         tft.fillScreen(ILI9341_BLACK);
         tft.setCursor(0, 0);
         tft.println("Send a Command");
       }
       Serial.print("R");
       continue;
     }   
     
     // ToDo: shorten the timeout that parseInt uses so that
     // we don't wait so long for the end of a number string if
     // a human tester forgets to put in a termination char
     commandVal  = Serial.parseInt();
     processCommand(commandChar, commandVal);
     
     // Peek at the next character. If it is a "Z" we are
     // done.
     // ToDo: we can make this protocol more robust by assuming that
     // any noncommand char is a termination char
     char foo = Serial.peek();

     if ('Z' == foo)
      {
        // We need to read the char to get it out of the buffer *and*
        // continue the loop so that Serial.available runs and returns
        // 0 (otherwise the "Z" character will still be the first 
        // char in the buffer then next time we get into this routine -
        // despite what the docs say)
        Serial.read();
        commandFlag = false;
      } // end if "end" char
  } // end while Serial.available 
}

// ----------------------------------------------------------------------------- //
// doBling
// This function simply looks at the currentRunning function (as set by the most 
// recent "E" command and executes the approriate function
// ----------------------------------------------------------------------------- //

void doBling() {
  
  serialStatusShow('R');
  LCDStatusShow('R');
  // Set the staus LED on to indicate that we are running a bling function
  
  if (lcdDebugEnabled) {
    tft.fillRect (312,232, 8,8,0xFFFF00);
  }
  digitalWrite(STATUS_PIN, HIGH);
  
  // Set the brightness for the function about to be called. We don't apply
  // brightness changes for CLEAR becaue we may not be clearing all of the
  // pixels so we don't want to effect the brightness of the ones left uncleared
  if (CLEAR != runningFunction) {
    strip.setBrightness(blingParmsTable[runningFunction].brightness);
  }
  
  switch (runningFunction)
  {
    case COLOR_WIPE:
       colorWipe(blingParmsTable[runningFunction].color, 
                 blingParmsTable[runningFunction].delay,
                 blingParmsTable[runningFunction].pixelStart,
                 blingParmsTable[runningFunction].pixelEnd);
       break;
    case COLOR_WIPE_WITH_BLANK:
       colorWipeWithBlank(blingParmsTable[runningFunction].color, 
                          blingParmsTable[runningFunction].delay,
                          blingParmsTable[runningFunction].pixelStart,
                          blingParmsTable[runningFunction].pixelEnd);
       break;
    case THEATRE_CHASE:
       theatreChase(blingParmsTable[runningFunction].color,
                    blingParmsTable[runningFunction].delay,
                    blingParmsTable[runningFunction].pixelStart,
                    blingParmsTable[runningFunction].pixelEnd);
       break;
    case RAINBOW:
       rainbow(blingParmsTable[runningFunction].delay,
               blingParmsTable[runningFunction].pixelStart,
               blingParmsTable[runningFunction].pixelEnd);
       break;       
    case THEATRE_CHASE_RAINBOW:
       theatreChaseRainbow(blingParmsTable[runningFunction].delay,
                           blingParmsTable[runningFunction].pixelStart,
                           blingParmsTable[runningFunction].pixelEnd);  
       break;
    case CLEAR:
    case COLOR_BAR:
       colorBar(blingParmsTable[runningFunction].color,
                blingParmsTable[runningFunction].pixelStart,
                blingParmsTable[runningFunction].pixelEnd);
       break; 
    default:
       break;
  } // end switch
}

// ----------------------------------------------------------------------------- //
// processCommand
// Handle a single command/value pair received on the serial line
// ----------------------------------------------------------------------------- //

void processCommand(char cmdChar, uint32_t cmdVal) {
  // Debug
  // Serial.print(cmdChar);
  // Serial.print(" ");
  // Serial.println(cmdVal);
  
  switch (cmdChar) {
    case 'B':
      // Set the brightness for the function being configured
      blingParmsTable[configFunction].brightness = cmdVal;
      break;
    case 'C':
      // Set the color for the function being configured
      // (not all functions actually use this parameter)
      blingParmsTable[configFunction].color = cmdVal;
      break;
    case 'D':
      // Set the delay for the function being configured
      // (not all functions actually use this parameter)
      blingParmsTable[configFunction].delay = cmdVal;
      break;
    case 'E':
      // Select the bling function to execute
      runningFunction = (function_t)cmdVal;
      repeatCount = blingParmsTable[runningFunction].repeat;
      break;
    case 'F':
      // Select the bling function to configure with any following
      // configurations commands (B, C, D, etc.)
      configFunction = (function_t)cmdVal;
      break;
    case 'P':
      // Set the pixel count for the function being configured
      // (not all functions actually use this parameter)
      if (cmdVal > STRIP_LEN) {
        cmdVal = STRIP_LEN;
      }
      blingParmsTable[configFunction].pixelStart = cmdVal;
      break;
    case 'Q':
      // Set the pixel count for the function being configured
      // (not all functions actually use this parameter)
      if (cmdVal > STRIP_LEN) {
        cmdVal = STRIP_LEN;
      }
      blingParmsTable[configFunction].pixelEnd = cmdVal;
      break;
    case 'R':
      // Set the repeat count for the function being configured
      blingParmsTable[configFunction].repeat = cmdVal;
      break;
    default: 
      if (serialDebugEnabled) {
        Serial.print("Unrecognized cmdChar: <");
        Serial.print(cmdChar);
        Serial.println(">");
      }
  } // end switch(cmdChar)
  
    serialStatusShow('C');
    LCDStatusShow('C');
    doneSent = false;
}

void doBlink() {
  // check to see if it's time to blink the LED; that is, if the 
  // difference between the current time and last time you blinked 
  // the LED is bigger than the interval at which you want to 
  // blink the LED.
  unsigned long currentMillis = millis();
 
  if (currentMillis - prevLedMillis > ledInterval) {
    // save the last time you blinked the LED 
    prevLedMillis = currentMillis;   

    // if the LED is off turn it on and vice-versa:
    if (ledState == LOW) {
      ledState = HIGH;
      if (lcdDebugEnabled) {
        tft.fillRect (312,232, 8,8,0xFFFF00);
      }
    } else {
      if (lcdDebugEnabled) {
        tft.fillRect (312, 232,8,8,0);
      }
      ledState = LOW;
    }
    // set the LED with the ledState of the variable:
    digitalWrite(STATUS_PIN, ledState);
  }
}

// ----------------------------------------------------------------------------- //
// serialStatusShow
// This is an all-purpose status output routine to let the outside world know 
// what we are doing. With debugEnabled == true, it signals the outside world 
// via an LED on pin 8 as follows:
//  - runing bling function: LED is ON
//  - awaiting a command string on the serial line: flashing once per second
// With debugEnabled == true, the settings of all of the configurable parameters 
// are output to the serial console once per second
// ----------------------------------------------------------------------------- //

void serialStatusShow (const char prefix)
{      
    if (serialDebugEnabled) {
      // Show the current running function and its configuration
      Serial.print(prefix);
      Serial.print(": E=");
      Serial.print(runningFunction);
      Serial.print(" B=");
      Serial.print(blingParmsTable[runningFunction].brightness);
      Serial.print(" C=");
      Serial.print(blingParmsTable[runningFunction].color);
      Serial.print(" D=");
      Serial.print(blingParmsTable[runningFunction].delay);
      Serial.print(" P=");
      Serial.print(blingParmsTable[runningFunction].pixelStart);
      Serial.print(" Q=");
      Serial.print(blingParmsTable[runningFunction].pixelEnd);
      Serial.print(" R=");
      Serial.print(blingParmsTable[runningFunction].repeat);
      Serial.print(" ");
      Serial.println(repeatCount);
      
      // Next line ...
      Serial.print("   F=");
      Serial.print(configFunction);
      Serial.print(" B=");
      Serial.print(blingParmsTable[configFunction].brightness);
      Serial.print(" C=");
      Serial.print(blingParmsTable[configFunction].color);
      Serial.print(" D=");
      Serial.print(blingParmsTable[configFunction].delay);
      Serial.print(" P=");
      Serial.print(blingParmsTable[configFunction].pixelStart);
      Serial.print(" Q=");
      Serial.print(blingParmsTable[configFunction].pixelEnd);
      Serial.print(" R=");
      Serial.println(blingParmsTable[configFunction].repeat);
    }
  }

// ----------------------------------------------------------------------------- //
// LCDStatusShow
// This is an all-purpose status output routine to let the outside world know 
// what we are doing. It is nearly identical to serialStatusShow - it has slight 
// format differences and outputs to an attached TFT LCD touchscreen rather than
// to the serial port.
// With debugEnabled == true, it signals the outside world 
// via an LED on pin 8 as follows:
//  - runing bling function: LED is ON
//  - awaiting a command string on the serial line: flashing once per second
// With debugEnabled == true, the settings of all of the configurable parameters 
// are output to the serial console once per second
// ----------------------------------------------------------------------------- //

void LCDStatusShow (const char prefix)
{      
    if (lcdDebugEnabled) {
      // Since we aren't sophisticated enough (or, perhaps, lacking in update
      // speed) we can't yet scroll the LCD dislay so we wipe it clean and start
      // from the top each time we output a block of status info.
      tft.fillScreen(ILI9341_BLACK);
      tft.setCursor(0, 0);
      
      // Show the current running function and its configuration
      tft.print(prefix);
      tft.print(": E");
      tft.print(runningFunction);
      tft.print(" B");
      tft.print(blingParmsTable[runningFunction].brightness);
      tft.print(" P");
      tft.print(blingParmsTable[runningFunction].pixelStart);
      tft.print(" Q");
      tft.println(blingParmsTable[runningFunction].pixelEnd);
      tft.print("   C");
      tft.print(blingParmsTable[runningFunction].color);
      tft.print(" D");
      tft.println(blingParmsTable[runningFunction].delay);
      tft.print("   R");
      tft.print(blingParmsTable[runningFunction].repeat);
      tft.print(" ");
      tft.println(repeatCount);
      tft.println(" ");
      
      // Next line ...
      tft.print("   F");
      tft.print(configFunction);
      tft.print(" B");
      tft.print(blingParmsTable[configFunction].brightness);
      tft.print(" P");
      tft.print(blingParmsTable[configFunction].pixelStart);
      tft.print(" Q");
      tft.println(blingParmsTable[configFunction].pixelEnd);
      tft.print("   C");
      tft.print(blingParmsTable[configFunction].color);
      tft.print(" D");
      tft.println(blingParmsTable[configFunction].delay);
      tft.print("   R");
      tft.println(blingParmsTable[configFunction].repeat);
    }
  }

// ----------------------------------------------------------------------------- //
// colorWipe
// Fill the dots one after the other with a color but use a breakable
// delay to enable faster response to command requests from the roboRIO
// ----------------------------------------------------------------------------- //

void colorWipe(uint32_t c, uint8_t wait, uint16_t pixelStart, uint16_t pixelEnd) {
  for(uint16_t i=pixelStart; i<pixelEnd; i++) {
      strip.setPixelColor(i, c);
      strip.show();
      if (delayWithBreak (wait)) {
        return;
      }
  } // for
  return;
}

// ----------------------------------------------------------------------------- //
// colorWipeWithBlank
// Clear the strip then...
// Fill the dots one after the other with a color but use a breakable
// delay to enable faster response to command requests from the roboRIO
// ----------------------------------------------------------------------------- //

void colorWipeWithBlank(uint32_t c, uint8_t wait, uint16_t pixelStart, uint16_t pixelEnd) {
  for(uint16_t i=pixelStart; i<pixelEnd; i++) {
      strip.setPixelColor(i, 0);
  } // end for i
  strip.show();

  
  for(uint16_t i=pixelStart; i<pixelEnd; i++) {
      strip.setPixelColor(i, c);
      strip.show();
      if (delayWithBreak (wait)) {
        return;
      }
  } // end for i
  return;
}

// ----------------------------------------------------------------------------- //
// theatreChase
// Theatre-style crawling lights
// delay to enable faster response to command requests from the roboRIO 
// ----------------------------------------------------------------------------- //

void theatreChase(uint32_t c, uint8_t wait, uint16_t pixelStart, uint16_t pixelEnd) {
  for (int j=0; j<10; j++) {  //do 10 cycles of chasing
    for (int q=0; q < 3; q++) {
      for (int i=pixelStart; i < pixelEnd; i=i+3) {
        strip.setPixelColor(i+q, c);    //turn every third pixel on
      }
      strip.show();
      if (delayWithBreak (wait)) {
        return;
      }
     
      for (int i=pixelStart; i < pixelEnd; i=i+3) {
        strip.setPixelColor(i+q, 0);        //turn every third pixel off
      }
    } // end for q
  } // end for j
}

// ----------------------------------------------------------------------------- //
// rainbow
// *Insert discription of rainbow*
// ----------------------------------------------------------------------------- //

void rainbow(uint8_t wait, uint16_t pixelStart, uint16_t pixelEnd) {
  uint16_t i, j;

  for(j=0; j<256; j++) {
    for(i=pixelStart; i<pixelEnd; i++) {
      strip.setPixelColor(i, Wheel((i+j) & 255));
    }
    strip.show();
    if (delayWithBreak (wait)) {
      return;
    }
  } // end for j
}

// ----------------------------------------------------------------------------- //
// Wheel
// ----------------------------------------------------------------------------- //

uint32_t Wheel(byte WheelPos) {
  WheelPos = 255 - WheelPos;
  if(WheelPos < 85) {
   return strip.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  } else if(WheelPos < 170) {
    WheelPos -= 85;
   return strip.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  } else {
   WheelPos -= 170;
   return strip.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
  }
}

// ----------------------------------------------------------------------------- //
// theatreChaseRainbow
// Theatre-style crawling lights with rainbow effect
// delay to enable faster response to command requests from the roboRIO 
// ----------------------------------------------------------------------------- //

void theatreChaseRainbow(uint8_t wait, uint16_t pixelStart, uint16_t pixelEnd) {
  for (int j=0; j < 256; j++) {     // cycle all 256 colors in the wheel
    for (int q=0; q < 3; q++) {
        for (int i=pixelStart; i < pixelEnd; i=i+3) {
          strip.setPixelColor(i+q, Wheel( (i+j) % 255));    //turn every third pixel on
        }
        strip.show();
        if (delayWithBreak (wait)) {
          return;
        }
       
        for (int i=pixelStart; i < pixelEnd; i=i+3) {
          strip.setPixelColor(i+q, 0);        //turn every third pixel off
        }
    }
  }
}

// ----------------------------------------------------------------------------- //
// colorBar
// ----------------------------------------------------------------------------- //

void colorBar(uint32_t c, uint16_t pixelStart, uint16_t pixelEnd) {
  for(uint16_t i=pixelStart; i<pixelEnd; i++) {
      strip.setPixelColor(i, c);
  } 
  strip.show();
  return;
}

// ----------------------------------------------------------------------------- //
// delayWithBreak
// A delay function which checks the serial device every millisecond to
// see if there is a command request character ("X"). If one is seen
// we exit the delay and signal the caller that we have ended early
// ----------------------------------------------------------------------------- //

boolean delayWithBreak (uint32_t count)
{
  uint32_t localCount = 0;
  
  for (localCount = 0; localCount < count; localCount++) {
    delay (1);
    
    // Note that we peek here (not read) so that the char is still
    // in the serial buffer and can be seen and read as usual by
    // the serialEvent routine
    char tmp = Serial.peek();

     if ('X' == tmp)
      {
        return true;
      }
  } // end for
  return false;
}
