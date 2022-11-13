

/* WARNING, 5K PULLUP RESISTORS *REQUIRED* CONNECTED TO THE I2C PINS (SCL/PB2/ICPIN 7 & SDA/PB0/ICPIN 5)


DERIVED FROM:
    https://github.com/datacute/Tiny4kOLED
    Project name: ATTiny85 - OLED (I2C)
    Project URI: https://www.studiopieters.nl/attiny85-oled-i2c
    Description: ATTiny85 - OLED (I2C)
    Version: 2.0.9
    License: MIT
USES
    https://github.com/datacute/TinyOLED-Fonts
    TinyOLED-Fonts/src/font8x16atari.h  by Neven Boyanov
    TinyOLED-Fonts/src/font16x32digits.h  by  Neven Boyanov
    
AND
    Arduino Rotary Encoder Tutorial
    by Dejan Nedelkovski, www.HowToMechatronics.com


    Warning: 
      AtTiny85 Clock: Internal 8 or 16 MHz for the encoder to work smoothly
      Remembaer using the 'Tools>burn bootloader' option in Arduino IDE after
      changing clock speed.

        Tools>Clock: Internal 8 MHz
        Tools>Burn boouloader

  Switches on an output voltage for the duration set with a rotary encoder.
  Set and remaining times are displayed on a 128x32 px OLED display in 
  MM:SS format. Timer is started through the rotary encoder's  pushbutton. 
  If the encoder's knob is turned again while the timer is still running the 
  signal is turned off and the timer paused. 
  
  Continuous mode (no timed switch off) is selected turning the encoder's
  knob as if to set a negative time. 

  The sketch uses the same pin for the pushbutton input and the driver signal
  to allow using an AtTiny MCU. 

    
*/
#include <TinyWireM.h>
#include <Tiny4kOLED.h>

#include <font16x32digits.h>
#include <font8x16atari.h>

// ROTARY ENCODER PINS
#define ENCODER_A       3  // PB3 ICPIN 2
#define ENCODER_B       4  // PB4 ICPIN 3

// INPUT/OUTPUT SWITCH PIN
// When the start button is detected in pin 1
// the same pin is turned to output and used to
// drive the MOSFET until the selected time expires
#define IOPIN          1  // PB1 ICPIN 6

void blinkit(unsigned char times) {
  while (times) {
    digitalWrite(IOPIN, HIGH); 
    delay(200); 
    digitalWrite(IOPIN, LOW); 
    times--;  
    delay(500);
  }
}

/*------------------------------------------------------------------- 
 * Globals & protoypes
 -------------------------------------------------------------------*/
int counter = 0;               // current set seconds
int lastcounter = 99;          // previous value of counter (must be different)
int lasttimersetting = 0;      //
int aState = 0;                // state of encoder A, B signals
int bState = 0;
int aLastState = 0;            // last state of ENCODER_A signal
bool on_state = false;         // Switch is turned on
bool continuous = false;       // Continuous mode 
unsigned long startmillis = 0; // time the switch was turned on;
unsigned long last_encoder_change = 0; // time the encoder knob was last turned
unsigned long last_disp_refresh = 0;   // time of last display refresh 

#define DISPLAY_REFRESH 100   // miliseconds between display refreshes


void update_display();                 // update 128x32 OLED display 
inline void zitoa(int val, char *buf); // add leading zero
void read_encoder();                   // read encoder, update t counter


/******************************************************************** 
 * SET UP
 ********************************************************************/
void setup() {

  // Initialize IO PIN (MUST BE PULLED DOWN)
  pinMode(IOPIN, OUTPUT);
  digitalWrite(IOPIN, LOW);    //Set switch off
  pinMode(IOPIN, INPUT);
  delay(300);

  // Initialize encoder
  pinMode (ENCODER_A,INPUT);
  pinMode (ENCODER_B,INPUT);
  // Read the initial state of the ENCODER_A
  aLastState = digitalRead(ENCODER_A); 


  // Initialize I2C & OLED
  TinyWireM.begin();
  oled.begin();   // Send the initialization sequence to the oled. This leaves the display turned off

  // Adjust rotation as needed
  // Default: Connectors at the left when reading text (comment out SetRotation)
  oled.setRotation((uint8_t) 2);

  // Clear the memory before turning on the display
  oled.clear();
  // Turn on the display and diaplay an empty screen
  oled.on();
  oled.clear();
  oled.setFont(FONT8X16ATARI);
  oled.setCursor(10, 2);     // (X in px, Y 8-px rows both starting at 0
  oled.print("HELLO");
  delay(1000);
  oled.clear();

  last_encoder_change = millis();
  update_display();
  
  //>>>>>>>>>>>>>>>>> oled.switchRenderFrame();   // Switch frames

}


void turn_off() {
  pinMode(IOPIN, OUTPUT);
  digitalWrite(IOPIN, LOW);  // turn switch off
  pinMode(IOPIN, INPUT);
  on_state = false;
  counter = lasttimersetting;
}

/******************************************************************** 
 * MAIN LOOP
 ********************************************************************/
void loop() {

 /*---------------------------------------------------------------------
  *  If switch is on decrement timer, check for end and turn off timer
  *---------------------------------------------------------------------
  *
  */
  if (on_state) {
    if (!continuous) {
      if ((millis() - startmillis) > 1014) {  // EMPIRICAL millis per second as clocked
        counter--;
        startmillis = millis();
      }
      if (counter < 1) {
        on_state = false;
        counter = 0;
        turn_off();
      }
    }
  }
  
 /*---------------------------------------------------------------------
  *  Button pressed? Turn to ON driver signal (same pin as button input)
  *  except long press (> 0.75s) with reset timer settings
  *---------------------------------------------------------------------
  *
  */
  if (!on_state) {
    byte button;
    unsigned long time_pressed = millis();
    byte pressed = 0;
    byte long_press = 0;
    while (button = digitalRead(IOPIN)) {
        pressed = 1;
        if ( (millis() - time_pressed) > 750) {  // long press reset to 0;
          long_press = 1;
        }
     }
     if (long_press) {
      counter = 0;
      lasttimersetting = 0;
      continuous = false;
     } else if (pressed) {
        pinMode(IOPIN, OUTPUT);
        digitalWrite(IOPIN, HIGH);
        lasttimersetting = counter;
        on_state = true; // Switch is turned on
        startmillis = millis(); // time the switch was turned on;   
    }
  }
  
 /*---------------------------------------------------------------------
  *  Update time counter
  *---------------------------------------------------------------------
  *
  */
  read_encoder();
  
  update_display();


} ///////////////////////////////// END MAIN LOOP ///////////////////////

/******************************************************************** 
 * AUXILIARIES
 ********************************************************************/

 /*---------------------------------------------------------------------
  *  Format 2 digit number with leading zero 
  *---------------------------------------------------------------------
  *
   */
  
inline void zitoa(int val, char *buf) {
  if (val < 10) {
    *buf = '0';
    itoa(val, ++buf, 10);
  } else {
    itoa(val, buf, 10);
  }
}

 /*---------------------------------------------------------------------
  *  Update Display
  *---------------------------------------------------------------------
  *  Left half 8x16font (5x8=40px)
  *    TURN>
  *    SET+-    At startup
  *    
  *    PUSH>
  *    START    When time>0
  *    
  *    *ON*     Pause    When Running (inverted)
  *  
  *  Right side 16x32 font (5x16=80px)
  *   MM:SS or  CONT
  *
  * I2C OLED updates are fairly slow, so read_encoder is called 
  * between operations.
  *
  */
void update_display() {
  if ( (millis() - last_disp_refresh) < DISPLAY_REFRESH) {
    return;
  }
  oled.setFont(FONT8X16ATARI);
  oled.setCursor(0, 0);     // (X in px, Y 8-px rows both starting at 0
  read_encoder();
  if (on_state) {
    oled.print(F("*RUN*"));
    read_encoder();
    oled.setCursor(0, 2);     // (X in px, Y 8-px rows both starting at 0
    read_encoder();
    oled.print(F("o-STP"));
  } else if (counter>0) {
    oled.print(F("PUSH>"));
    read_encoder();
    oled.setCursor(0, 2);     // (X in px, Y 8-px rows both starting at 0
    read_encoder();
    oled.print(F("START"));
  } else {
    oled.print(F("TURN>"));
    read_encoder();
    oled.setCursor(0, 2);     // (X in px, Y 8-px rows both starting at 0
    read_encoder();
    oled.print(F(" SET "));
  }
  read_encoder();
  

  // MM:SS or CONT
  oled.setFont(FONT16X32DIGITS);
  if (continuous) {
    oled.setCursor(48, 0);    // left side + 8px space
    oled.print(F(". - ."));    
  } else {
    unsigned int minutes  = counter / 60;
    unsigned int seconds  = counter % 60;
    char buf[8]; // 
    zitoa(minutes, buf);
    oled.setCursor(48, 0);    // left side + 8px space
    read_encoder();
    oled.print(buf);
    read_encoder();
    oled.setCursor(80, 0);    // 48 + 2chars x 16px
    read_encoder();
    if (counter%2) oled.print(F(".")); else oled.print(".");
    read_encoder();
    oled.setCursor(96, 0);    // add 16px colon
    read_encoder();
    zitoa(seconds, buf);
    oled.print(buf);
  }
  read_encoder();
  last_disp_refresh = millis();

  // Swap which half of RAM is being written to, and which half is being displayed
  //>>>>>>>>>>>>>>>>> oled.switchFrame();
  // Keep current counter
}


/*---------------------------------------------------------------------
  *  Read rotary encoder and increment/decrement seconds counter
  *---------------------------------------------------------------------
  *
  * If the previous and the current state of the ENCODER_A input are 
  * different, that means a Pulse has occured
  * 
  * If the ENCODER_B state is different to the ENCODER_A state, 
  * that means the encoder is rotating clockwise
  * 
  * This function should be called as frequently as possible to avoid
  * missing pulses.
  * 
  */

unsigned int increment = 1; 
  
void read_encoder() {
  int oldcounter = counter;
  // Read the current state of ENCODER_A
  aState = digitalRead(ENCODER_A);
  // If last and current state of ENCODER_A are different, then pulse occurred
  //>    // React to only 1 state change to avoid double count
  //>    // while (aState != aLastState  && aState == 1) {
  while (aState != aLastState) {
    // If the ENCODER_B state is different than the ENCODER_A state then
    // the encoder is rotating CCW so decrement
    if (digitalRead(ENCODER_B) != aState) {
      counter ++;
    } else {
      // Encoder is rotating CW so increment
      counter --;
    }
    // Remember last ENCODER_A state
    aLastState = aState;
    // Put in a slight delay to help debounce the reading
    delay(1);
    // Read again
    aState = digitalRead(ENCODER_A);
  }
  if (counter != oldcounter) {
    if (counter < 0) {
      counter = 0;
      continuous = true;
    } else  {
      if (counter > 5999) { // cap top 59m 59s
        counter = 5999;
      }
      continuous = false;   // positive times disable continuous mode
    }
    if (on_state) {  // Knob moved while on, switch off
      turn_off();
    }    
    // if turning fast adjust increment
    if ( (millis() < last_encoder_change) < 500) {
      increment *= 2;
    } else {
      increment = 1;
    }
    last_encoder_change = millis();
  }
}


  
