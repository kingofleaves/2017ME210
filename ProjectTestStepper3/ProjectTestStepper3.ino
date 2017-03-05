 /*************************************************************
  File:      Lab2Part3.ino
  Contents:  

  Notes:     Target: Arduino Uno
             Arduino IDE version: 1.6.7

  History:
  when       who  what/why
  ----       ---  ---------------------------------------------



 ************************************************************/

/*---------------Includes-----------------------------------*/

#include <Timers.h>
#include <Pulse.h>

/*---------------Module Defines-----------------------------*/

// Pins
#define PIN_STEP 5
#define PIN_DIR 3 
#define PIN_POT A3

// ONE_QUARTER*TIME_PERIOD SHOULD BE 6120


#define ONE_EIGHTH 10
#define ONE_QUARTER 33    // 53
#define TIME_PERIOD 1
#define SPEED 200


// Interrupts

// Timers


/*---------------Module Function Prototypes-----------------*/

void SetupPins(void);                                       // Allocates pin #s with their purpose

/*---------------Module Variables---------------------------*/

int dir = 0;                                                // Initial direction is LOW

int potReading =  SPEED;                                    // PROPORTIONAL TO SPEED 
unsigned int stepPeriod = 30 + potReading*0.94819;          // period to be sent 

/*---------------Lab 1 Main Functions-----------------------*/

void setup() {
  Serial.begin(9600);
  SetupPins();
  digitalWrite(PIN_DIR, HIGH);                               // Set initial dir pin to LOW 
  InitPulse(PIN_STEP, stepPeriod);                          // Prepare to generate pulse stream 


}

void loop() {
  
  Pulse(ONE_QUARTER);
  delay(TIME_PERIOD); 

}

/*----------------Module Functions--------------------------*/

void SetupPins() {
  pinMode(PIN_STEP, OUTPUT); 
  pinMode(PIN_DIR, OUTPUT); 
  pinMode(PIN_POT, INPUT); 
}


