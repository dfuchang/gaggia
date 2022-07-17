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
#include <Adafruit_MAX31865.h>  // temperature sensor

// To be done
// allow live edit of pulling min boiler on time
// Error checking after button enter value, otherwise, EEPROM value check might re-init the whole EEPROM


//*****************************************
//****    User modifiable parameters   ****
//*****************************************
// #define DEBUG  // Used to compile for serial output debug info
#define PULL_MIN_THRESHHOLD 15000 // in milliseconds.  Must be this long before it is considered a shot pulled or shot counter increment
#define BREW_BAND 3 // +- temperature range to turn on PID 
//*****************************************

// Conditional compile flag to add more features
#define DISPLAY_TEMP

// Global Define
#define DEFAULT_KP 57   // default PID Kp value to initialize new board
#define DEFAULT_KI 38    // default PID Ki value to initialize new board
#define DEFAULT_KD 2    // default PID Kd value to initialize new board
#define DEFAULT_BREWING_MIN_ON 225 // Define in ms when pulling shots the min boiler on time > 100 and < 2000 (WINDOW_SIZE)
#define DEFAULT_STEAM_TEMP_TARGET 292  // target temperature for steam temp

// EEPROM Address
#define EE_LifeCycle_EEPROM_Addr 0   // EEPROM address for storage
#define EE_BREW_TEMP_TARGET_ADDR 4   // EEPROM address for storage
#define EE_PID_P 6   // EEPROM address for PID's P
#define EE_PID_I 8   // EEPROM address for PID's I
#define EE_PID_D 10  // EEPROM address for PID's D
#define EE_BREWING_MIN_ON 12 // EEPROM address for DEFAULT_BREWING_MIN_ON
#define EE_STEAM_TEMP 14 // EEPROM address for steam temp

#define LINE1 65  // Power Minute Line
#define LINE1_1 30  // PID legend line
#define LINE1_2 50  // PID value line
#define LINE2 100
#define LINE3 126
#define BUTTON_DEBOUNCE 300  // button debounce in millisecond
#define RREF      430.0  // MAX31865 value of Rref resistor 430.0 for PT100
#define RNOMINAL  100.2  // MAX31865 nominal 0-degree-C resistance of sensor. 100.0 for PT100
#define WINDOW_SIZE 1000  // Used to control relay slow PWM
#define WINDOW_MIN 50 // minimum time to turn on Boiler is 0.05s
// #define STEAM_TEMP_TARGET 292  // target temperature for steam temp
#define BREW_TEMP_TARGET_DEFAULT 220  // target tmperature for brewing temp

// PIN definitions
#define PULLING_GROUND   2  // used for shot detection +5V to switch to PULLING_PIN
#define STEAM_PIN 6 // Steam detction.  Ground connected to ground pin #2
#define ROTARY_ENCODER_PIN1  3  // Used for ISR for rotary knob
#define ROTARY_ENCODER_PIN2  4  // used for ISR for rotary knob
#define ROTARY_GROUND 5 // used for rotary knob ground
#define PULLING_PIN 7 // Pulling shot detection +5
#define BUTTON_PIN 20 // used to detect rotary encoder button press
#define BUTTON_GND 21 // used for ground
#define TEMP_SENSOR_CS_PIN 19 // Used for MAX31865
#define RELAY_PIN_GND 18 // used to control heating element ground
#define RELAY_PIN_5V 17 // used to control heating element
#define STEAM_RELAY_PIN_GND 16 // used to control steam relay GND
#define STEAM_RELAY_PIN_5V 15 // used to control steam relay +5V
#define OLED_CS 10   // OLED Chip Select pin
#define OLED_DC 9    // OLED DC pin
#define OLED_RESET 8  // OLED Reset pin

// Glogal Variable
long int lifeCycle = 0;  
// int targetTemp = 200; 
float boilerTemp=0;
long int myTemp1, myTemp2;
String myString;   // temp string
unsigned long pullOn = LOW;  // Pulling Shot Button ON
unsigned long lastPull = 0; // last pull time
unsigned long steamOn = 0; // steam turn on time
unsigned long buttonTime = 0;  // Time stamp of when last button press 
int buttonConfig = false; // used to indicate button configuration is ON
int brewTempTarget;  // used to track last brew temperature before going to steam
int boilerStat=0;  // status of relay to turn on boiler for display purposes
int BrewingMinOn; // Define in ms when pulling shots the min boiler on time > 100 and < 2000 (WINDOW_SIZE)
int boilerOffset, testMode; // use for display only
int steamTemp;  // steam temperature setting

// QuickPID definition
unsigned long PID_WinStartTime; // used for PID Relay PWM window calculation
float Setpoint=0, Output=0; // USED for Quickpid
//Specify the links and initial tuning parameters
float Kp, Ki, Kd;
float POn = 0.5;   // proportional on Error to Measurement ratio (0.0-1.0), default = 1.0
float DOn = 0.5;   // derivative on Error to Measurement ratio (0.0-1.0), default = 0.0
QuickPID myQuickPID(&boilerTemp, &Output, &Setpoint, Kp, Ki, Kd, POn, DOn, QuickPID::DIRECT);

// U8g2 Library Full Frame Buffer mode used (clock, data, cs and dc)
// U8G2_SSD1327_MIDAS_128X128_F_4W_HW_SPI u8g2(U8G2_R0, /* cs=*/ 10, /* dc=*/ 9, /* reset=*/ 8);
U8G2_SSD1327_MIDAS_128X128_F_4W_HW_SPI u8g2(U8G2_R0, /* cs=*/ OLED_CS, /* dc=*/ OLED_DC, /* reset=*/ OLED_RESET);

// Temperature sensor definition.  (We are using hardware SPI)
// Use software SPI: CS, DI, DO, CLK
// Adafruit_MAX31865 thermo = Adafruit_MAX31865(10, 11, 12, 13);
// use hardware SPI, just pass in the CS pin
Adafruit_MAX31865 thermo = Adafruit_MAX31865(TEMP_SENSOR_CS_PIN);

// A pointer to the dynamic created rotary encoder instance.
// This will be done in setup()
RotaryEncoder *encoder = nullptr;

// Interupt handler for rotary encoder detection
void checkPosition()
{
      encoder->tick(); // just call tick() to check the state.
}

// read eeprom value
int readEEPROM() 
{
    int temp;
    
    // Read EEPROM LifeCycle
    EEPROM.get(EE_LifeCycle_EEPROM_Addr, lifeCycle);
    // check for EEPROM lifeCycle inrange
    if (lifeCycle < 0 || lifeCycle > 10000) return(1);

    // Read EEPROM Target Brew Temperature
    EEPROM.get(EE_BREW_TEMP_TARGET_ADDR, brewTempTarget);
    // check for EEPROM EEPROM Target Brew Temperature inrange
    if (brewTempTarget < 0 || brewTempTarget > 350) return(1);

    // Read EEPROM PID_P
    EEPROM.get(EE_PID_P, temp);  Kp = temp;  // Casting here is important
    // check for EEPROM PID_P inrange
    if (Kp < 0 || Kp > 350) return(1);

    // Read EEPROM PID_I
    EEPROM.get(EE_PID_I, temp);  Ki = temp;  // Casting here is important
    // check for EEPROM PID_P inrange
    if (Ki < 0 || Ki > 350) return(1);

    // Read EEPROM PID_D
    EEPROM.get(EE_PID_D, temp); Kd = temp; // Casting here is important
    // check for EEPROM PID_P inrange
    if (Kd < 0 || Kd > 350) return(1);
    
    // Read EEPROM BrewingMinOn
    EEPROM.get(EE_BREWING_MIN_ON, temp); BrewingMinOn= temp; // Casting here is important
    // check for EEPROM BrewingMinOn inrange
    if (BrewingMinOn < 0 || BrewingMinOn > 2000) return(1);
    //if (BrewingMinOn < 0 || BrewingMinOn > 2000) EEPROM.put(EE_BREWING_MIN_ON, (int) DEFAULT_BREWING_MIN_ON);

    // Read EEPROM Steam Temp
    EEPROM.get(EE_STEAM_TEMP, temp); steamTemp= temp; // Casting here is important
    // check for EEPROM steam temp in range
    if (steamTemp < 0 || steamTemp > 330) return(1);
    // if (steamTemp < 0 || steamTemp > 330) EEPROM.put(EE_STEAM_TEMP, (int) DEFAULT_STEAM_TEMP_TARGET);

    // Serial.println("EEPROM Reading successful");
    return(0);
}

// -------------------------------------------------------------
void setup(void) {
    #ifdef DEBUG
    Serial.begin(115200); // open the serial port at 115200 bps:
    #endif
    
    u8g2.begin();  //  Start U8g2 library
    u8g2.setContrast(200);  //  Brightness setting from 0 to 255

    // EEPROM.put(EE_LifeCycle_EEPROM_Addr, (long int) 127);
    
    // Read EEPROM LifeCycle
    if (readEEPROM() == 1) {
      // Serial.println("EEPROM Fault Re-initialize");
      // invalid value in eeprom.  Reinitialize EEPROM
      // New board and new counter initilize to 1
      EEPROM.put(EE_LifeCycle_EEPROM_Addr, (long int) 1);

      // New board and new initialize brew target temp to 
      EEPROM.put(EE_BREW_TEMP_TARGET_ADDR, (int) BREW_TEMP_TARGET_DEFAULT);
      
      // New board and new initialize PID_P
      EEPROM.put(EE_PID_P, (int) DEFAULT_KP);
            
      // New board and new initialize PID_I
      EEPROM.put(EE_PID_I, (int) DEFAULT_KI);
            
      // New board and new initialize PID_D
      EEPROM.put(EE_PID_D, (int) DEFAULT_KD);

      // New board and new initialize BrewingMinOn
      EEPROM.put(EE_BREWING_MIN_ON, (int) DEFAULT_BREWING_MIN_ON);

      // New board and new initialize Steam Temp
      EEPROM.put(EE_STEAM_TEMP, (int) DEFAULT_STEAM_TEMP_TARGET);


      // Read it again
      if (readEEPROM() == 1) while (1);
    }

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
  
    // register interrupt routine for rotary encoder
    attachInterrupt(digitalPinToInterrupt(ROTARY_ENCODER_PIN1), checkPosition, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ROTARY_ENCODER_PIN2), checkPosition, CHANGE);

    // configure button pin - setup digital Pulling Switch IN with pullup & configure GND PIN
    pinMode(BUTTON_PIN, INPUT_PULLUP);
    pinMode(BUTTON_GND, OUTPUT);
    digitalWrite(BUTTON_GND, LOW);   

    // MAX 31865 initialization
    thermo.begin(MAX31865_3WIRE);  // set to 2WIRE or 4WIRE as necessary

    // PID setup
    pinMode(RELAY_PIN_5V, OUTPUT);  // Relay pin config
    digitalWrite(RELAY_PIN_5V, LOW); // reset relay to 0
    pinMode(RELAY_PIN_GND, OUTPUT);  // Relay pin config
    digitalWrite(RELAY_PIN_GND, LOW); // reset relay to 0
    pinMode(STEAM_RELAY_PIN_5V, OUTPUT); // Steam relay 5v config
    digitalWrite(STEAM_RELAY_PIN_5V, HIGH); // Set Steam relay 5v to HIGH
    pinMode(STEAM_RELAY_PIN_GND, OUTPUT); // Steam relay GND config
    digitalWrite(STEAM_RELAY_PIN_GND, LOW); // Set Steam relay 5v to low
    PID_WinStartTime = millis();  // initialize Relay PWM window timer
    Setpoint = brewTempTarget;  // set default temperature target
    
    myQuickPID.SetOutputLimits(0, WINDOW_SIZE);  //tell the PID to range between 0 and the full window size
    myQuickPID.SetTunings(Kp, Ki, Kd, POn, DOn);
    myQuickPID.SetMode(QuickPID::AUTOMATIC);     //turn the PID on
    
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
    //     buttonConfig == 4 in BrewingMinON config mode
    //     buttonConfig == 5 in Steam Temp config mode
    if (steamOn == 0 && pullOn == 0 && buttonConfig > 0) {
      // Not in steam or pulling shot and button configuration is ON
      
      // First page
      if (buttonConfig == 1 || buttonConfig == 2 || buttonConfig ==3) {
        // display PID legend
        u8g2.setFont(u8g2_font_timB14_tf);  // 
        u8g2.drawStr(0, LINE1_1, "    P      I      D");
        
        // Display actual P value and flash it if it is being configured
        if (buttonConfig == 1 && ((millis() % 1000) < 500) || buttonConfig != 1) { 
          myString = (int) Kp;  
          u8g2.drawStr(20, LINE1_2, myString.c_str());  // set position and text to display
        }
  
        // Display actual I value and flash it if it is being configured
        if (buttonConfig == 2 && ((millis() % 1000) < 500) || buttonConfig !=2) { 
          myString = (int) Ki;  
          u8g2.drawStr(60, LINE1_2, myString.c_str());  // set position and text to display
        }
  
        // Display actual d value and flash it if it is being configured
        if (buttonConfig == 3 && ((millis() % 1000) < 500) || buttonConfig !=3) { 
          myString = (int) Kd;  
          u8g2.drawStr(100, LINE1_2, myString.c_str());  // set position and text to display
        }
      }

      // Second Page
      if (buttonConfig == 4 || buttonConfig == 5) {
        // display BrewingMinOn
        u8g2.setFont(u8g2_font_timB14_tf);  // 
        u8g2.drawStr(0, LINE1_1, "BMO  Steam");
        
        // Display actual BrewingMinOn value and flash it if it is being configured
        if (buttonConfig == 4 && ((millis() % 1000) < 500) || buttonConfig != 4) { 
          myString = (int) BrewingMinOn;  
          u8g2.drawStr(10, LINE1_2, myString.c_str());  // set position and text to display
        }

        // Display actual Steam Temp value and flash it if it is being configured
        if (buttonConfig == 5 && ((millis() % 1000) < 500) || buttonConfig != 5) { 
          myString = (int) steamTemp;  
          u8g2.drawStr(60, LINE1_2, myString.c_str());  // set position and text to display
        }
        
      } // if (buttonConfig == 4 ...
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
    // Display Curent temperature icon if boiler is on
    if (boilerStat == true) {
      u8g2.setFont(u8g2_font_open_iconic_thing_2x_t);  // Choose font to use
      u8g2.drawGlyph(0, (LINE2-3), 72);  // Display Boiler Symbol
    }

    // Display Actual temperature
    myString = String((int)boilerTemp) + "\xb0"; // add degree symbol
    u8g2.setFont(u8g2_font_timB24_tf);  // Choose font to use
    u8g2.drawStr(25, LINE2, myString.c_str());

    // Display Target Temp
    u8g2.setFont(u8g2_font_timB14_tf);  // Choose font to use
    myString = String((int) Setpoint) + "\xb0";
    u8g2.drawStr(95, LINE2, myString.c_str());
#endif

    // Display Boiler On %
    u8g2.setFont(u8g2_font_timB14_tf);  // Choose font to use
    myString = String("Boiler");
    u8g2.drawStr(0, LINE3, myString.c_str());

    // Display LifeCycle, calculate offset base on number of digit
    // myTemp2 = log10 (lifeCycle);
    // myString = String(lifeCycle);
    if (Output+boilerOffset <= 1) myTemp2=2;  // if log10 is 0, need 2 space.
      else myTemp2 = log10 (Output+boilerOffset)+1; // add 1 for % sign
    myString = String((int)(Output+boilerOffset)*100/WINDOW_SIZE) + "%";
    u8g2.drawStr((120-myTemp2*9), LINE3, myString.c_str());

    myString = String((int)testMode);  // debug
    u8g2.drawStr(60, LINE3, myString.c_str()); //debug

    // Display lastPull time if we are not pulling shot or steaming and in button config mode.  
    if (!(steamOn == 0 && pullOn == 0 && buttonConfig > 0)) {
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
  int pos = 0, newPos=0, temp=0;  // used for rotary encoder
  float myTemp;

  while (1) {
      // Read if Pulling Switching is ON - LOW is ON because pin is PULLUP
      if (digitalRead(PULLING_PIN) == LOW) {
        if (pullOn == 0) {
          pullOn = millis();   // Switch is ON - save off current millis value.  Only if the first time entering 
        }
      } else {
        // Switch is OFF
        //Check if we need to increment poll counter if it is > 18 seconds (PULL_MIN_THRESHHOLD)
        if (pullOn != 0 && (millis()-pullOn) > PULL_MIN_THRESHHOLD) {
          // Increment Poll counter
          lifeCycle = lifeCycle + 1;
          // write to EEProm
          EEPROM.put(EE_LifeCycle_EEPROM_Addr, (long int) lifeCycle); 
        }
        
        // save off lastPull & reset PID
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
        if (steamOn == 0) {
          // First time detecting steam on
          steamOn = millis();  // Steam is on - save off current millis value.  Only first time entering
          
          // set PID setpoint target to new steam temperature
          Setpoint = steamTemp;
  
          // Set Steam PID
          myQuickPID.SetTunings(Kp, Ki, Kd, POn, DOn);
        }
        
      } else {
        // steam is now off
        if (steamOn > 0) { 
          // transitioning from steam On to steam OFF
          // set PID setpoint target to new steam temperature
          Setpoint = brewTempTarget;
          
          // switch is off.  Reset switchOn
          steamOn = 0;
  
          // Turn steam relay on.  
          digitalWrite(STEAM_RELAY_PIN_5V, HIGH);
  
          // Set Brew PID
          myQuickPID.SetTunings(Kp, Ki, Kd, POn, DOn);
        }
      }

      // rotary encoder detection
      int newPos = encoder->getPosition();
      if (pos != newPos) {
          // rotary encoder has been turned.  Direction are either +1 or -1

          // check what config stage we are in
          switch (buttonConfig) {
            case 0:
              // if buttonConfig == 0 change temperature by 1 degree but only change it if it is NOT steaming
              if (steamOn == 0){
                brewTempTarget = Setpoint + (float) (encoder->getDirection());
                Setpoint = brewTempTarget;
                
                // save off in EEPROM
                EEPROM.put(EE_BREW_TEMP_TARGET_ADDR, (int)brewTempTarget);  
              }
            break;

            case 1:
              // if buttonConfig == 1 change PID_P by 1
              Kp = Kp + (float) (encoder->getDirection()); 
            break;

            case 2:
              // if buttonConfig == 2 change PID_I by 1
              Ki = Ki + (float) (encoder->getDirection()); 
            break;

            case 3:
              // if buttonConfig == 3 change PID_D by 1
              Kd = Kd + (float) (encoder->getDirection());
            break;

            case 4:
              // if buttonConfig == 4 change BrewingMinOn by 50
              if (encoder->getDirection() == RotaryEncoder::Direction::CLOCKWISE) BrewingMinOn = BrewingMinOn + 5;
              else BrewingMinOn = BrewingMinOn - 5;
            break;

            
            case 5:
              // if buttonConfig == 5 change temperature by 1 degree 
              if (steamOn == 0){
                steamTemp = steamTemp + (float) (encoder->getDirection()); 
              }
            break;
          }

          // save PID changes to controller
          myQuickPID.SetTunings(Kp, Ki, Kd, POn, DOn);
          
          pos = newPos;
      } // if

      // detect button press
      if (digitalRead(BUTTON_PIN) == LOW && ((millis() - buttonTime) > BUTTON_DEBOUNCE )) {
        
        // save off button press value
        buttonTime = millis();

        // button config mode on & increment to next stage
        buttonConfig = (buttonConfig + 1) % 6; // cycle through 0-4 ONLY while always increase by 1

        // safe all EEPROM.  If it has not change, put will not do the actual write.
        EEPROM.put(EE_PID_P, (int)Kp); 
        EEPROM.put(EE_PID_I, (int)Ki); 
        EEPROM.put(EE_PID_D, (int)Kd); 
        EEPROM.put(EE_BREWING_MIN_ON, (int)BrewingMinOn); 
        EEPROM.put(EE_STEAM_TEMP, (int)steamTemp); 
      }

      // Update temperature
      boilerTemp = thermo.temperature(RNOMINAL, RREF)*1.8+32;
      // boilerTemp = round(boilerTemp * 10.0)/10.0;  // round to 1 decimal places for display
      
      //************************************************
      // PID Processing
      if (millis() - PID_WinStartTime >= WINDOW_SIZE) {
          //time to shift the Relay Window
                  
          #ifdef DEBUG  
          Serial.println("shift PID window");
          #endif
  
          PID_WinStartTime += WINDOW_SIZE;

          // if current emp is within +- BREW_BAND degrees turn on PID. (Use only during steam and when pulling shots
          //   This is needed when getting hot water to turn heater on at 100% if temp dip below 5 degree
          /*  
            if ((abs(Setpoint-boilerTemp) < BREW_BAND) || (boilerTemp < 250)) 
            #ifdef DEBUG  
            Serial.println("A ");
            #endif
            
            // if PID is manual, turn it on first
            // Output=0;
            if (myQuickPID.GetMode() == QuickPID::MANUAL) {
              Output=0;
              myQuickPID.SetMode(QuickPID::AUTOMATIC);
            }
            
            // compute PID
            myQuickPID.Compute();*/
          if ((abs(Setpoint-boilerTemp) > BREW_BAND) && ((boilerTemp > 250) || (pullOn >0))) {
            #ifdef DEBUG  
            Serial.println("M ");
            #endif
            
            myQuickPID.SetMode(QuickPID::MANUAL);
            
            // outside of temperature range, just turn boiler on or off
            if (Setpoint > boilerTemp) Output = WINDOW_SIZE;
             else Output = 0;
          } else {

                        
            #ifdef DEBUG  
            Serial.println("A ");
            #endif
            
            // if PID is manual, turn it on first
            // Output=0;
            if (myQuickPID.GetMode() == QuickPID::MANUAL) {
              Output=0;
              myQuickPID.SetMode(QuickPID::AUTOMATIC);
            }
            
            // compute PID
            myQuickPID.Compute();
          }

        }

      // ***********************************************
      // *** NOTE the next section ORDER is important ***
      // set relay to turn on boiler and pass in offset.  The offset is used to subtract away from relay pwm ms unit less than window size
      // if not pulling shot, use offset of 0
      
      // calculate pull start time
      myTemp = millis() - pullOn;
      
      if (pullOn == 0) {
        boilerControl(0);
        
        testMode = 0;
      }

      // if Pulling shot and temp less than current target +1 (safety), add BrewingMinOn as offset to Output
      if ((pullOn > 0) && (myTemp <= 5000)){ 
        if (boilerTemp <= (Setpoint+1)) {
          boilerControl(BrewingMinOn);
        } else boilerControl(0);
        
        testMode = 1;
      } 
      
      // if pulling shot,  after 5 seconds add offset  to just 50% of BrewingMinOn since reached 9 bars pressure
      if ((pullOn >0) && (myTemp > 5000) && (myTemp <= 12000)) {
        // Check if we are in the > 1 about Setpoint, if yes, turn off offset.  
        if (boilerTemp < (Setpoint+1)) boilerControl(0);
        else
          boilerControl(0);
          
        testMode = 2;
      }

      // if pulling shot,  after 9 seconds add offset  to just 125% of BrewingMinOn since reached 9 bars pressure
      if ((pullOn >0) && (myTemp > 9000)) {
        // Check if we are in the > 1 about Setpoint, if yes, turn off offset.  
        if (boilerTemp < (Setpoint+1)) boilerControl(BrewingMinOn*0.6);
        else
          boilerControl(0);
          
        testMode = 3;
      }
      // *** Order important ends ***
      // ***********************************************
      
      refreshScreen();


      #ifdef DEBUG1
      // Only print approximately every second
      if ((millis() % 1000) > 800) {
        Serial.print("Lifecycle:");
        Serial.print(lifeCycle);
        Serial.print(" ");
    
        Serial.print("tempTarget:");
        Serial.print(brewTempTarget);
        Serial.print(" ");

        Serial.print("Setpoint:");
        Serial.print(Setpoint);
        Serial.print(" ");
        
        Serial.print("boilerTemp:");
        Serial.print(boilerTemp);
        Serial.print(" ");
        
        Serial.print("Output:");
        Serial.print(Output);
        Serial.print(" ");
    
        Serial.print("boilerStat:");
        Serial.print(boilerStat);
        Serial.print(" ");
    
        Serial.print(Kp);
        Serial.print(" ");
        Serial.print(Ki);
        Serial.print(" ");
        Serial.print(Kd);
        Serial.print(" ");
        Serial.println(); 
      }
      #endif
    
      // delay(50);  // sleep a little
  } // while (1)
} // loop

// ***********************************************************************
// This routine is used to controller boiler relay based on PID controller
void boilerControl(int offset) {
      // make sure with offset it doesn't go pass the maximum window size.  
      if ((Output + offset) > WINDOW_SIZE) offset = WINDOW_SIZE-Output;
      boilerOffset = offset;
      // Two safety checks.  
      // safety check #1 - do not turn on boiler if temperature is > 305 degree.   The gaggia also has a temperature
      // fuse that will cutoff power to broiler at 330 degree or so.  
      // if (boilerTemp > 305) Output = 0;
          
      // if PID output value > minimal turn on time && PID output > 
      if (((unsigned int)(Output+offset) < WINDOW_MIN) || ((unsigned int)(Output+offset) < (millis() - PID_WinStartTime))) {

        // if steam is ON control the stream relay instead of brew relay
        if (steamOn > 0) digitalWrite(STEAM_RELAY_PIN_5V, LOW);
        else {
          // use brewing heater relay instead.
          digitalWrite(RELAY_PIN_5V, LOW);
        }
        boilerStat = false;
      } 
      else {
        // if steam is ON control the stream relay instead of brew relay
        if (steamOn > 0) digitalWrite(STEAM_RELAY_PIN_5V, HIGH);
        else {
          digitalWrite(RELAY_PIN_5V, HIGH);  //Safety check #2 make sure boiler is not on if boilerTemp is more than 10 degreen higher than set
        }
        boilerStat = true;
      }
}
