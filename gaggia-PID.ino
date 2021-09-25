// -----
// Gaggia Classic Pro Elegant Hack firmware
//
// GNU GPL V3 license
//
// -----

#include <Arduino.h>
#include <RotaryEncoder.h>
#include <U8g2lib.h>  // U8g2 Library for Oled https://github.com/olikraus/u8g2
#include "QuickPID.h"
#include <EEPROM.h>
#include <math.h>


// To be done


//*****************************************
//****    User modifiable parameters   ****
//*****************************************
#define PULL_MIN_THRESHHOLD 18000 // in milliseconds.  Must be this long before it is considered a shot pulled or shot counter increment
//*****************************************

// Conditional compile flag to add more features
#define DISPLAY_TEMP

// Global Define
#define LifeCycle_EEPROM_Addr 0
#define LINE1 65  // Power Minute Line
#define LINE1_1 30  // PID legend line
#define LINE1_2 50  // PID value line
#define LINE2 100
#define LINE3 126

// PIN definitions
#define PULLING_PIN 7 // Pulling shot detection +5
#define PULLING_GROUND   2  // used for shot detection +5V to switch to PULLING_PIN
#define ROTARY_ENCODER_PIN1  3  // Used for ISR for rotary knob
#define ROTARY_ENCODER_PIN2  4  // used for ISR for rotary knob
#define ROTARY_GROUND 5 // used for rotary knob ground
#define BUTTON_PIN 20 // used to detect rotary encoder button press
#define BUTTON_GND 21 // used for ground
#define STEAM_PIN 6 // Steam detction.  Ground connected to ground pin #2
#define BUTTON_DEBOUNCE 300  // button debounce in millisecond


// Glogal Variable
long int lifeCycle = 0;
int targetTemp = 200;
long int myTemp1, myTemp2;
String myString;   // temp string
unsigned long pullOn = LOW;  // Pulling Shot Button ON
unsigned long lastPull = 0; // last pull time
unsigned long steamOn = 0; // steam turn on time
unsigned long buttonTime = 0;  // Time stamp of when last button press 
int buttonConfig = false; // used to indicate button configuration is ON
int PID_P = 4; // PID variable
int PID_I = 2;
int PID_D = 5;


// U8g2 Library Full Frame Buffer mode used
U8G2_SSD1327_MIDAS_128X128_F_4W_HW_SPI u8g2(U8G2_R0, /* cs=*/ 10, /* dc=*/ 9, /* reset=*/ 8);

// A pointer to the dynamic created rotary encoder instance.
// This will be done in setup()
RotaryEncoder *encoder = nullptr;

// Interupt handler for rotary encoder detection
void checkPosition()
{
      encoder->tick(); // just call tick() to check the state.
}

// -------------------------------------------------------------
void setup(void) {
    u8g2.begin();  //  Start U8g2 library
    u8g2.setContrast(200);  //  Brightness setting from 0 to 255

    // EEPROM.put(LifeCycle_EEPROM_Addr, (long int) 56);
    
    // Read EEPROM LifeCycle
    EEPROM.get(LifeCycle_EEPROM_Addr, lifeCycle);

    // if getting a lifeCycle out of range, means new board and new initilize to 1
    if (lifeCycle < 0 or lifeCycle > 10,000) EEPROM.put(LifeCycle_EEPROM_Addr, 1);

    // Setup digital Pulling Switch IN with pullup.  0 means pulling shot.  
    pinMode(PULLING_PIN, INPUT_PULLUP);

    // Use extra ground PIN
    pinMode(PULLING_GROUND, OUTPUT);
    digitalWrite(PULLING_GROUND, LOW);

    // Use extra ground pin for rotary encoder
    pinMode(ROTARY_GROUND, OUTPUT);
    digitalWrite(ROTARY_GROUND, LOW);   

    // Setup digital Steam Switch IN with pullup.  0 means Steam is ON
    pinMode(STEAM_PIN, INPUT);

    // setup the rotary encoder functionality

    // use FOUR3 mode when PIN_IN1, PIN_IN2 signals are always HIGH in latch position.
    // encoder = new RotaryEncoder(PIN_IN1, PIN_IN2, RotaryEncoder::LatchMode::FOUR3);
  
    // use FOUR0 mode when PIN_IN1, PIN_IN2 signals are always LOW in latch position.
    // encoder = new RotaryEncoder(PIN_IN1, PIN_IN2, RotaryEncoder::LatchMode::FOUR0);
  
    // use TWO03 mode when PIN_IN1, PIN_IN2 signals are both LOW or HIGH in latch position.
    encoder = new RotaryEncoder(ROTARY_ENCODER_PIN1, ROTARY_ENCODER_PIN2, RotaryEncoder::LatchMode::FOUR3);
  
    // register interrupt routine
    attachInterrupt(digitalPinToInterrupt(ROTARY_ENCODER_PIN1), checkPosition, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ROTARY_ENCODER_PIN2), checkPosition, CHANGE);

    // configure button pin - setup digital Pulling Switch IN with pullup & configure GND PIN
    pinMode(BUTTON_PIN, INPUT_PULLUP);
    pinMode(BUTTON_GND, OUTPUT);
    digitalWrite(BUTTON_GND, LOW);   

    
    
} // setup()

// Display Text on Oled display
void refreshScreen(void) {

    int space;
      
    // Clear Display Buffer
    u8g2.clearBuffer();  // Clear Oled display

    // Check if in pulling shot display mode
    if (pullOn > 0) {
        // Display shot timer when it is pulling shot
        // Display PullOn Symbol
        u8g2.setFont(u8g2_font_open_iconic_embedded_4x_t);  // Choose font to use
        u8g2.drawGlyph(0, 60, 67);  //Power

        // Display Pullon Second unit
        u8g2.setFont(u8g2_font_timB14_tf);  // Choose font to use
        u8g2.drawStr(97, LINE1, "sec");

        // Display actual elapsed sec
        myString = ((millis() - pullOn) / 1000) % 100;  // figure out number of sec after pulling shot. Max 2 digit
    
        // calculate text space if it is 2 digit or 3 digit
        if (myString.length() == 1) {
          space = 50; // single digit
        } else {
          space = 35; // double digit
        }

        u8g2.setFont(u8g2_font_logisoso46_tn);  // Choose font to use
        u8g2.drawStr(space, LINE1, myString.c_str());  // set position and text to display
        
    }

    // check if in steam display mode
    if (steamOn > 0) {
      // Display steam timer when steaming
        // Display steam Symbol
        u8g2.setFont(u8g2_font_open_iconic_embedded_4x_t);  // Choose font to use
        u8g2.drawGlyph(0, 60, 80);  // bluetooth symbol

        // Display steamOn Second unit
        u8g2.setFont(u8g2_font_timB14_tf);  // Choose font to use
        u8g2.drawStr(97, LINE1, "sec");

        // Display actual elapsed sec
        myString = ((millis() - steamOn) / 1000) % 100;  // figure out number of sec after pulling shot. Max 2 digit
    
        // calculate text space if it is 2 digit or 3 digit
        if (myString.length() == 1) {
          space = 50; // single digit
        } else {
          space = 35; // double digit
        }

        u8g2.setFont(u8g2_font_logisoso46_tn);  // Choose font to use
        u8g2.drawStr(space, LINE1, myString.c_str());  // set position and text to display
    }

    // check if in button config mode
    //     buttonConfig == false   NOT in config mode
    //     buttonConfig == 1 in P config mode
    //     buttonConfig == 2 in I config mode
    //     buttonConfig == 3 in D config mode
    if (steamOn == 0 && pullOn == 0 && buttonConfig > 0) {
      // Not in steam or pulling shot and button configuration is ON

      // display PID legend
      u8g2.setFont(u8g2_font_timB14_tf);  // 
      u8g2.drawStr(0, LINE1_1, "    P      I      D");
      
      // Display actual P value and flash it if it is being configured
      if (buttonConfig == 1 && ((millis() % 1000) < 500) || buttonConfig != 1) { 
        myString = PID_P;  
        u8g2.drawStr(20, LINE1_2, myString.c_str());  // set position and text to display
      }

      // Display actual I value and flash it if it is being configured
      if (buttonConfig == 2 && ((millis() % 1000) < 500) || buttonConfig !=2) { 
        myString = PID_I;  
        u8g2.drawStr(60, LINE1_2, myString.c_str());  // set position and text to display
      }

      // Display actual P value and flash it if it is being configured
      if (buttonConfig == 3 && ((millis() % 1000) < 500) || buttonConfig !=3) { 
        myString = PID_D;  
        u8g2.drawStr(100, LINE1_2, myString.c_str());  // set position and text to display
      }
      
    } else buttonConfig = false;

    // check if just in normal mode not pulling shots or steaming or button config
    if (pullOn == 0 && steamOn == 0 && buttonConfig == 0)  {
        // Normal display cycle - Indiate current powerup minute

        // display solid power symbol if power up time > 15 min
        // flash power symbol once every 0.5 seconds if warming        
        if (millis() > 900000 || (millis() % 1000 < 500)) {
          // Display Power Symbole
          u8g2.setFont(u8g2_font_open_iconic_embedded_4x_t);  // Choose font to use
          u8g2.drawGlyph(0, 60, 78);  //Power 
        }
    
        // Display Power-up time Unit
        u8g2.setFont(u8g2_font_timB14_tf);  // Choose font to use
        u8g2.drawStr(97, LINE1, "min");
    
        // Display actual elapsed min
        myString = ((millis() / 60000) % 100) ;  // figure out number of min since power up.  /1000/60. Modulo 100 to only display 2 digit
    
        // calculate text space if it is 2 digit or 3 digit
        if (myString.length() == 1) {
          space = 50; // single digit
        } else {
          space = 35; // double digit
        }
        
        u8g2.setFont(u8g2_font_logisoso46_tn);  // Choose font to use
        u8g2.drawStr(space, LINE1, myString.c_str());  // set position and text to display
    }

#if defined DISPLAY_TEMP   // Only compiled in this section if version support temperature
    // Display Curent temperature icon
    u8g2.setFont(u8g2_font_open_iconic_thing_2x_t);  // Choose font to use
    u8g2.drawGlyph(0, (LINE2-3), 72);  // Display Boiler Symbol

    // Display Actual temperature
    myString = String(targetTemp) + "\xb0"; // add degree symbol
    u8g2.setFont(u8g2_font_timB24_tf);  // Choose font to use
    u8g2.drawStr(25, LINE2, myString.c_str());

    // Display Target Temp
    u8g2.setFont(u8g2_font_timB14_tf);  // Choose font to use
    myString = String("200") + "\xb0";
    u8g2.drawStr(95, LINE2, myString.c_str());
#endif

    // Display Pull
    u8g2.setFont(u8g2_font_timB14_tf);  // Choose font to use
    myString = String("Pull");
    u8g2.drawStr(0, LINE3, myString.c_str());

    // Display LifeCycle, calculate offset base on number of digit
    // myTemp1 = random(0, 200000); // test
    // myTemp2 = log10 (myTemp1); // test
    // myString = String(myTemp1); // test
    myTemp2 = log10 (lifeCycle);
    myString = String(lifeCycle);
    u8g2.drawStr((120-myTemp2*10), LINE3, myString.c_str());


    // Display lastPull time
    if (lastPull != 0) {
       // Display last pull time
        myString = (lastPull / 1000) % 100;  // figure out number of sec after pulling shot. Max 2 digit
    
        // calculate text space if it is 2 digit or 3 digit
        if (myString.length() == 1) {
          space = 110; // single digit
        } else {
          space = 100; // double digit
        }

        myString = myString + "\"";

        u8g2.setFont(u8g2_font_timB14_tf);  // Choose font to use
        u8g2.drawStr(space, 22, myString.c_str());  // set position and text to display
    }
      
    // Display buffer
    u8g2.sendBuffer();   // Update Oled display
}  // refreshScreen

// Startup display show life time pull
void startupScreen(void) {
  
    // Clear Display Buffer
    u8g2.clearBuffer();  // Clear Oled display

    // Display startup text
    u8g2.setFont(u8g2_font_timB24_tf);  // Choose font to use
    u8g2.drawStr(0, 65, "Life Pull");
 
    // Display actual value
    myString = lifeCycle;
    // lifeCycle = millis() & 200000;  // test
    // myString = lifeCycle;  // test
    myTemp2 = log10 (lifeCycle);
    u8g2.drawStr((55-myTemp2*16/2), 110, myString.c_str());

    // Display buffer
    u8g2.sendBuffer();   // Update Oled display
} // startupScreen

void loop(void) {

  startupScreen();
  delay(5000);
  int pos = 0, newPos=0 ;  // used for rotary encoder

 while (1) {
      // Read if Pulling Switching is ON - LOW is ON because pin is PULLUP
      if (digitalRead(PULLING_PIN) == LOW) {
        if (pullOn == 0) pullOn = millis();   // Switch is ON - save off current millis value.  Only if the first time entering     
      } else {
        // Switch is OFF
        //Check if we need to increment poll counter if it is > 18 seconds (PULL_MIN_THRESHHOLD)
        if (pullOn != 0 && (millis()-pullOn) > PULL_MIN_THRESHHOLD) {
          // Increment Poll counter
          lifeCycle = lifeCycle + 1;
          // write to EEProm
          EEPROM.put(LifeCycle_EEPROM_Addr, lifeCycle);          
        }
        
        // save off lastPull
        if (pullOn !=0 ) {
          // if shot is < 18 (PULL_MIN_THRESHHOLD)  seconds, most like flushing group head or steam wond.  Don't display the short pull
          if ((millis() - pullOn) > PULL_MIN_THRESHHOLD)
            lastPull = millis() - pullOn;
        }
        
        // reset pullOn
        pullOn = 0;      
      }

      // Detect Steaming on
      if (digitalRead(STEAM_PIN) == HIGH) {
        if (steamOn == 0) steamOn = millis();  // Steam is on - save off current millis value.  Only first time entering
      } else {
        // switch is off.  Reset switchOn
        steamOn = 0;
      }

      // rotary encoder detection
      int newPos = encoder->getPosition();
      if (pos != newPos) {
          // rotary encoder has been turned.  Direction are either +1 or -1

          // check what config stage we are in
          switch (buttonConfig) {
            case 0:
              // if buttonConfig == 0 change temperature by 1 degree
              targetTemp = targetTemp + (int) (encoder->getDirection());
            break;

            case 1:
              // if buttonConfig == 1 change PID_P by 1
              PID_P = PID_P + (int) (encoder->getDirection());
            break;

            case 2:
              // if buttonConfig == 2 change PID_I by 1
              PID_I = PID_I + (int) (encoder->getDirection());
            break;

            case 3:
              // if buttonConfig == 3 change PID_D by 1
              PID_D = PID_D + (int) (encoder->getDirection());
            break;
          }
          
          pos = newPos;
      } // if

      // detect button press
      if (digitalRead(BUTTON_PIN) == LOW && ((millis() - buttonTime) > BUTTON_DEBOUNCE )) {
        
        // save off button press value
        buttonTime = millis();

        // button config mode on & increment to next stage
        buttonConfig = (buttonConfig + 1) % 4; // cycle through 0-3 ONLY while always increase by 1
      }
      
      refreshScreen();
      delay(50);  // sleep a little
  } // while (1)
} // loop
