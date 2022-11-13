

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


    AtTiny85 Clock: Internal 8 MHz

    
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
int counter = 0;       // current set seconds
int lastcounter = 99;  // previous value of counter (must be different)
int aState, bState;    // state of encoder A, B signals
int aLastState;        // last state of ENCODER_A signal
bool on_state = false; // Switch is turned on
unsigned long startmillis; // time the switch was turned on;
unsigned long last_encoder_change; 


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
  last_encoder_change = millis();


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
  oled.setCursor(0, 0);     // (X in px, Y 8-px rows both starting at 0
  oled.print("HELLO");
  delay(1000);

  //>>>>>>>>>>>>>>>>> oled.switchRenderFrame();   // Switch frames

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
    if ((millis() - startmillis) > 1014) {  // EMPIRICAL millis per second as clocked
      counter--;
      startmillis = millis();
      update_display();
    }
    if (counter < 1) {
      on_state = false;
      counter = 0;
      digitalWrite(IOPIN, LOW);  // turn switch off
      update_display();
    }
  }
  
 /*---------------------------------------------------------------------
  *  Button pressed? Turn to ON driver signal (same as btton signal)
  *---------------------------------------------------------------------
  *
  */
  if (!on_state) {
    byte button = digitalRead(IOPIN); 
    if (button) {
        pinMode(IOPIN, OUTPUT);
        digitalWrite(IOPIN, HIGH);
        on_state = true; // Switch is turned on
        startmillis = millis(); // time the switch was turned on;   
        update_display(); 
    }
  }
  
 /*---------------------------------------------------------------------
  *  Update time counter
  *---------------------------------------------------------------------
  *
  */

  read_encoder();
  if (counter != lastcounter) {
    update_display();
    lastcounter = counter;
  }
  
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
  *   MM:SS         
  *
  * I2C OLED updates are fairly slow, so read_encoder is called 
  * between operations.
  *
  */
void update_display() {
  oled.setFont(FONT8X16ATARI);
  oled.setCursor(0, 0);     // (X in px, Y 8-px rows both starting at 0
  read_encoder();
  if (on_state) {
    oled.print(F("*RUN*"));
    read_encoder();
    oled.setCursor(0, 2);     // (X in px, Y 8-px rows both starting at 0
    read_encoder();
    oled.print(F("-Adj+"));
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
    oled.print(F("-SET+"));
  }
  read_encoder();
  
  unsigned int minutes  = counter / 60;
  unsigned int seconds  = counter % 60;
  char buf[8]; // 

  // MM:SS
  oled.setFont(FONT16X32DIGITS);
  zitoa(minutes, buf);
  oled.setCursor(48, 0);    // left side + 8px space
  read_encoder();
  oled.print(buf);
  read_encoder();
  oled.setCursor(80, 0);    // 48 + 2chars x 16px
  read_encoder();
  oled.print(F(":"));
  read_encoder();
  oled.setCursor(96, 0);    // add 16px semicolon
  read_encoder();
  zitoa(seconds, buf);
  oled.print(buf);
  read_encoder();

  // Swap which half of RAM is being written to, and which half is being displayed
  //>>>>>>>>>>>>>>>>> oled.switchFrame();
  // Keep current counter
}


/*---------------------------------------------------------------------
  *  Read rotary encoder asnd increment/decrement seconds counter
  *---------------------------------------------------------------------
  *
  * If the previous and the current state of the ENCODER_A input are 
  * different, that means a Pulse has occured
  * 
  * If the ENCODER_B state is different to the ENCODER_A state, 
  * that means the encoder is rotating clockwise
  * 
  * This function should be called as frequently as possible to avoid
  * miussing pulses.
  * 
  */

  
void read_encoder() {
  aState = digitalRead(ENCODER_A);  // Reads the "current" state of the ENCODER_A
  bState = digitalRead(ENCODER_B); 

  unsigned int increment = 2; 
  while (aState != aLastState){       // Allow For multiple changes
    if (millis() - last_encoder_change < 333) {  // encoder turning fast?
      increment *= 2;
    }
    last_encoder_change  = millis();
    //  chack direction
    if (bState != aState) { 
      counter = (counter <= (5999-increment))? counter+increment: 5999;  // cap to 99m 99s = 5999s
    } else {
      counter = (counter >= increment)? counter-increment : 0;  // avoid negatives
    }
    aLastState = aState; // Updates the previous state of the ENCODER_A with the current state
    aState = digitalRead(ENCODER_A); 
    bState = digitalRead(ENCODER_B); 
  }
}
