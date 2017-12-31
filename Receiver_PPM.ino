 /*PPM Receiver. Receiver code Written by Guillermo J Colom
 *Combines:
 */
 /* YourDuinoStarter Example: nRF24L01 Receive Joystick values

 - WHAT IT DOES: Receives data from another transceiver with
   2 Analog values from a Joystick or 2 Potentiometers
   Displays received values on Serial Monitor
 - SEE the comments after "//" on each line below
 - CONNECTIONS: nRF24L01 Modules See:
 http://arduino-info.wikispaces.com/Nrf24L01-2.4GHz-HowTo
   1 - GND
   2 - VCC 3.3V !!! NOT 5V
   3 - CE to Arduino pin 9
   4 - CSN to Arduino pin 10
   5 - SCK to Arduino pin 13
   6 - MOSI to Arduino pin 11
   7 - MISO to Arduino pin 12
   8 - UNUSED
   
 - V1.00 11/26/13
   Based on examples at http://www.bajdi.com/
   Questions: terry@yourduino.com */
 
 /*PPM generator originally written by David Hasko
 * on https://code.google.com/p/generate-ppm-signal/ 
 */

//////////////////////PPM_CONFIGURATION///////////////////////////////
#define CHANNEL_NUMBER 8  //set the number of chanels
#define CHANNEL_DEFAULT_VALUE 1500  //set the default servo value
#define FRAME_LENGTH 22500  //set the PPM frame length in microseconds (1ms = 1000µs)
#define PULSE_LENGTH 300  //set the pulse length
#define onState 1  //set polarity of the pulses: 1 is positive, 0 is negative
#define sigPin  7  //set PPM signal output pin on the arduino
#define CE_PIN   9
#define CSN_PIN 10

/*-----( Import needed libraries )-----*/
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

/*-----( Declare objects )-----*/
RF24 radio(CE_PIN, CSN_PIN); // Create a Radio

/*-----( Declare Variables )-----*/
unsigned long radioFailCount = 0;
unsigned long midFailCount = 250;
unsigned long maxFailCount = 20000;
byte receivedBytes[18];  // 2 element array holding Joystick readings, and other values
bool failExceeded = false;
int  currentPulses[CHANNEL_NUMBER] = {1500, 1500, 1000, 1500, 1500, 1500, 1000, 1000};
int  failPulses[CHANNEL_NUMBER] = {1500, 1500, 1500, 1500, 1500, 1500, 2000, 1000};
int  failExceededPulses[CHANNEL_NUMBER] = {1500, 1500, 1300, 1500, 1500, 1500, 2000, 1000};
bool pendingUpdate = false;

// NOTE: the "LL" at the end of the constant is "LongLong" type
const uint64_t pipe = 0xE8E8F0F0E1LL; // Define the transmit pipe


/*this array holds the servo values for the ppm signal
 change theese values in your code (usually servo values move between 1000 and 2000)*/
int ppm[CHANNEL_NUMBER];

void transposeArray(int oldValues[CHANNEL_NUMBER], int newValues[CHANNEL_NUMBER]);

void setup()
{  
  Serial.begin(9600);             //Monitor Progress
  //initiallize default ppm values
  for(int i=0; i<CHANNEL_NUMBER; i++){
      ppm[i]= currentPulses[i];
  }
  pinMode(sigPin, OUTPUT);
  digitalWrite(sigPin, !onState);  //set the PPM signal pin to the default state (off)
  cli();
  TCCR1A = 0; // set entire TCCR1 register to 0
  TCCR1B = 0;
  
  OCR1A = 100;  // compare match register, change this
  TCCR1B |= (1 << WGM12);  // turn on CTC mode
  TCCR1B |= (1 << CS11);  // 8 prescaler: 0,5 microseconds at 16mhz
  TIMSK1 |= (1 << OCIE1A); // enable timer compare interrupt
  sei();

  //Radio Stuff
  //delay(1000);
  Serial.println("Nrf24L01 Receiver Starting");
  radio.begin();
  radio.openReadingPipe(1,pipe);
  radio.startListening();
}

void loop()
{
  {
    if ( radio.available() )
    {
      // Read the data payload until we've received everything
      Serial.println("radio available");
      receivedBytes[16] = 0;
      while (!receivedBytes[16])
      {
        // Fetch the data payload
        radio.read( receivedBytes, sizeof(receivedBytes) );
        
        if (receivedBytes[16])
        {
            radioFailCount = 0;
            bytesToIntsArray(receivedBytes, currentPulses, CHANNEL_NUMBER);
            pendingUpdate = true;
        }  
      }
      
    }// end if available
    else
    {    
        radioFailCount++;
        Serial.println("No radio available");
        if (radioFailCount >= midFailCount && radioFailCount < maxFailCount)
        {
          transposeArray(ppm, failPulses);
        }
        
        if (radioFailCount >= maxFailCount)
        {
          transposeArray(ppm, failExceededPulses);
        }        
    }// end if not available // end radio read
  }
  while(receivedBytes[16]&&pendingUpdate)
  {
    transposeArray(ppm, currentPulses);
    pendingUpdate = false;

    //Uncomment for testing
    /*
    for(int i = 0; i < CHANNEL_NUMBER; i++)
    {
      Serial.print(ppm[i]);
      Serial.print("  ");
    }
    Serial.println();
    */
  }
  /*
  }
  for (int i = 0; i < 4; i++)
  {
    pulses[i].writeMicroseconds(currentPulses[i]);
  }
  */
  delay(10);
  
}//end main loop()


void bytesToIntsArray(byte bytes[], int integers[], int intSize)
{
  for (int i = 0; i < intSize; i++)
  {
    integers[i] = bytes[2*i]*256 + bytes[2*i + 1];
  }
}// end bytesToIntsArray()

//copy new values into old array
void transposeArray(int oldValues[CHANNEL_NUMBER], int newValues[CHANNEL_NUMBER])
{
   for (int i = 0; i < CHANNEL_NUMBER; i++)
   {
    oldValues[i] = newValues[i];
   }
}// transposeArray()


//Interrupt Service Routine for PPM
ISR(TIMER1_COMPA_vect){  //leave this alone
  static boolean state = true;
  
  TCNT1 = 0;
  
  if (state) {  //start pulse
    digitalWrite(sigPin, onState);
    OCR1A = PULSE_LENGTH * 2;
    state = false;
  } else{  //end pulse and calculate when to start the next pulse
    static byte cur_chan_numb;
    static unsigned int calc_rest;
  
    digitalWrite(sigPin, !onState);
    state = true;

    if(cur_chan_numb >= CHANNEL_NUMBER){
      cur_chan_numb = 0;
      calc_rest = calc_rest + PULSE_LENGTH;// 
      OCR1A = (FRAME_LENGTH - calc_rest) * 2;
      calc_rest = 0;
    }
    else{
      OCR1A = (ppm[cur_chan_numb] - PULSE_LENGTH) * 2;
      calc_rest = calc_rest + ppm[cur_chan_numb];
      cur_chan_numb++;
    }     
  }

}

