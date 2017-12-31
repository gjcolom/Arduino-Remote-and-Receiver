/* Multipurpose RC controller. This implementation will be specifically for a 
 *  quadcopter with an arduino receiver and MultiWii FLight Controller.
 *  Guillermo Colom
   1 - GND
   2 - VCC 3.3V !!! NOT 5V
   3 - CE to Arduino pin 9
   4 - CSN to Arduino pin 10
   5 - SCK to Arduino pin 13
   6 - MOSI to Arduino pin 11
   7 - MISO to Arduino pin 12
   8 - UNUSED
   - 
   Analog Joystick or two 10K potentiometers:
   GND to Arduino GND
   VCC to Arduino +5V
   X1 Pot to Arduino A0
   Y1 Pot to Arduino A1
   M1 Pot to Arduino A6
   X2 Pot to Arduino A2
   Y2 Pot to Arduino A3
   M2 Pot to Arduino A7
   
 - V1.00 11/26/13
   Based on examples at http://www.bajdi.com/
   Questions: terry@yourduino.com */

/*-----( Import needed libraries )-----*/
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Wire.h>  // for LCD. Comes with Arduino IDE
#include <LiquidCrystal_I2C.h>  //for LCD.
/*-----( Declare Constants and Pin Numbers )-----*/
#define CE_PIN       9
#define CSN_PIN     10
#define JOYSTICK_X1 A3
#define JOYSTICK_Y1 A2
#define JOYSTICK_X2 A1
#define JOYSTICK_Y2 A0
#define AUX1        A7
#define AUX2        A6
#define AUX3        3
#define AUX4        4
#define DEADZONE    110         // distance from center for defining "deadzone"
#define DEADPULSE   1500      // default "center" pulse width
#define MINPULSE    1000      // minimum pulse width
#define MAXPULSE    2000      // maximum pulse width
#define CHANNELS    8         // number of channels
#define XBYTES      18        // number of bytes to be transmited

// NOTE: the "LL" at the end of the constant is "LongLong" type
const uint64_t pipe = 0xE8E8F0F0E1LL; // Define the transmit pipe


/*-----( Declare objects )-----*/
RF24 radio(CE_PIN, CSN_PIN);  // Create a Radio
/*-----( Declare Variables )-----*/

// set the LCD address to 0x27 for a 16 chars 2 line display
// A FEW use address 0x3F
// Set the pins on the I2C chip used for LCD connections:
//                    addr, en,rw,rs,d4,d5,d6,d7,bl,blpol
LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);  // Set the LCD I2C address


//Values to transmit
byte transmitBytes[XBYTES];        // 2 bytes per value
int  pulseValues[CHANNELS];          // actual pulse values {Throttle Y2, Roll X1, Pitch Y1, Yaw X2}
int  transmitDelay = 10000;   // microseconds between transmissions
unsigned long startTime;      // used to find elapsed time
unsigned long stopTime;       // used to find elapsed time

//Variables for calculating pulse widths
bool   reverseX1  = true;    // reverse axis direction roll
bool   reverseY1  = true;     // reverse axis direction pitch
bool   reverseX2  = false;    // reverse axis direction yaw
bool   reverseY2  = false;     // reverse axis direction throttle


void setup()   /****** SETUP: RUNS ONCE ******/
{
  Serial.begin(9600);
  
  // Initialize Display
  initializeDisplay ();
  displayOnLine("Hello", 0, 2);
  displayOnLine("Quadcopter", 1, 1);
  displayOnLine("Controller", 2, 1);
  delay(2000);
  while(digitalRead(3)+digitalRead(4))
  {
    displayOnLine("Warning!", 0, 2);
    displayOnLine("Set switches", 1, 1);
    displayOnLine("To Low      ", 2, 1);   
  }
  
  radio.begin();
  radio.openWritingPipe(pipe);
  startTime = micros();
}//--(end setup )---


void loop()   /****** LOOP: RUNS CONSTANTLY ******/
{
  stopTime = micros();
  if ((stopTime - startTime) >= transmitDelay || (stopTime < startTime))
  {
    startTime = micros();
    
    pulseValues[0] = pulseCalculate2(JOYSTICK_X1, 1.25, 512, reverseX1); //Roll
    pulseValues[1] = pulseCalculate2(JOYSTICK_Y1, 1.25, 512, reverseY1); //Pitch
    pulseValues[2] = pulseCalculate2(JOYSTICK_Y2, 1.25, 512, reverseY2); //Throttle
    pulseValues[3] = pulseCalculate2(JOYSTICK_X2, 1.25, 512, reverseX2); //Yaw
    pulseValues[4] = pulseCalculate(AUX1, false); //Aux1
    pulseValues[5] = pulseCalculate(AUX2, false); //Aux2
    pulseValues[6] = pinPulse(AUX3); //Aux3
    pulseValues[7] = pinPulse(AUX4); //Aux4
    
    pulseToByteArray(pulseValues, transmitBytes);
    transmitBytes[XBYTES-2] = 1;
    transmitBytes[XBYTES-1] = 1;
    
    radio.write( transmitBytes, sizeof(transmitBytes) );
    displayValues();
  }
  
}//--(end main loop )---

/*-----( Declare User-written Functions )-----*/

// Calculate Pulse Values
int pulseCalculate(int aPin, bool reverse)
{
  float input;
  int pulseOut;
  int readVal = reverseAxis(analogRead(aPin), reverse);
  input = (float)readVal;
  pulseOut = (int)(input*1000.0/1023.0);
  pulseOut += MINPULSE;
  return pulseOut;
}// end pulseCalculate()

int pinPulse(int pin)
{
  if (digitalRead(pin))
    return 2000;
  else
    return 1000;
}

//This function is meant to help create a "Dead Zone" when the Joysticks are 
//approximately centered, to output 1500 as values
int pulseCalculate2(int aPin, float sensitivity, int center, bool reverse)
{
  int pulseOut;
  int readVal = reverseAxis(analogRead(aPin), reverse);
  if (readVal >= center - DEADZONE && readVal <= center + DEADZONE)
    pulseOut = DEADPULSE;
  if (readVal < center - DEADZONE)
    pulseOut =(int)(sensitivity * (readVal - (center - DEADZONE)) + DEADPULSE);
  if (readVal > center + DEADZONE)
    pulseOut =(int)(sensitivity * (readVal - (center + DEADZONE)) + DEADPULSE);
  if (pulseOut < MINPULSE)
    pulseOut = MINPULSE;
  if (pulseOut > MAXPULSE)
    pulseOut = MAXPULSE;
  return pulseOut;
}// end pulseCalculate()
// Pulse to byteArray, high byte first
void pulseToByteArray(int pulses[], byte bytes[])
{
  for (int i = 0; i < CHANNELS; i++)
  {
    bytes[2*i] = pulses[i]/256;
    bytes[2*i+1] = pulses[i]%256;
  }
}// end pulseToByteArray()

int reverseAxis(int axis, bool reverse)
{
  if (reverse)
    axis = 1023 - axis;
  return axis;
} // ens reverseAxis()

//Initialize LCD Display
void initializeDisplay ()
{
lcd.begin(16,4);   // initialize the lcd for 16 chars 4 lines, turn on backlight

// ------- Quick 3 blinks of backlight  -------------
  for(int i = 0; i< 3; i++)
  {
    lcd.backlight();
    delay(100);
    lcd.noBacklight();
    delay(100);
  }
  lcd.backlight(); // finish with backlight on
}// end initializeDisplay()
void displayOnLine(char script[], int row, int col)
{
  lcd.setCursor(col, row);
  lcd.print(script);
}// en displayOnLine()



void displayValues()
{
  displayOnLine("Roll:", 0, 0);
  lcd.print(pulseValues[0]);
  displayOnLine("Ptch:", 1, 0);
  lcd.print(pulseValues[1]);
  lcd.print("    ");
  displayOnLine("Thr: ", 2, 0);
  lcd.print(pulseValues[2]);
  displayOnLine("Yaw: ", 3, 0);
  lcd.print(pulseValues[3]);
  displayOnLine(" 1:", 0, 9);
  lcd.print((float)pulseValues[4]/1000);
  displayOnLine(" 2:", 1, 9);
  lcd.print((float)pulseValues[5]/1000);
  displayOnLine(" 3:", 2, 9);
    if (pulseValues[6]>1000)
      lcd.print("high");
    else
      lcd.print("low ");
  displayOnLine(" 4:", 3, 9);
    if (pulseValues[7]>1000)
      lcd.print("high");
    else
      lcd.print("low ");
}// end displayValues()

//*********( THE END )***********

