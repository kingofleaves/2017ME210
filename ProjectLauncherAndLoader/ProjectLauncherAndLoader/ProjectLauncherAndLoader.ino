/*************************************************************
  File:      Lab2.ino
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
#define PIN_LAUNCHER_LEFT 8
#define PIN_LAUNCHER_RIGHT 9
#define PIN_STEP 5
#define PIN_DIR 3 

// Interrupts

// Other
#define ONE_QUARTER 33    // 53
#define TIME_PERIOD 1
#define SPEED 200

// Timers
#define TIMER_PWM 1


/*---------------Module Function Prototypes-----------------*/


void SetupPins(void);                                       // Allocates pin #s with their purpose
void PWM(void);                                             // PWM function for the DC motor 


/*---------------Module Variables---------------------------*/
int isDCOn = 0; 

int dir = 0;                                                // Initial direction is LOW

int potReading =  SPEED;                                    // PROPORTIONAL TO SPEED 
unsigned int stepPeriod = 30 + potReading*0.94819;          // period to be sent 


/*---------------Lab 1 Main Functions-----------------------*/

void setup() {
  Serial.begin(9600);
  SetupPins();
  TMRArd_InitTimer(TIMER_PWM, 1000);
  digitalWrite(PIN_DIR, LOW);                               // Set initial dir pin to LOW 
  InitPulse(PIN_STEP, stepPeriod);                          // Prepare to generate pulse stream 

}

void loop() {
  PWM(); 
  Pulse(ONE_QUARTER);
  delay(TIME_PERIOD);
}

/*----------------Module Functions--------------------------*/

void SetupPins() {
  pinMode(PIN_LAUNCHER_LEFT, OUTPUT); 
  pinMode(PIN_LAUNCHER_RIGHT, OUTPUT); 
  pinMode(PIN_STEP, OUTPUT); 
  pinMode(PIN_DIR, OUTPUT); 

}

void PWM(){
  if (TMRArd_IsTimerExpired(TIMER_PWM)){                    // if timer is expired..
    int launcherSpeed = 1020;                                     // read new pot value
        if (isDCOn){                                          // then, if DC is on..
        digitalWrite(PIN_LAUNCHER_RIGHT, LOW);                            // turn it off
        digitalWrite(PIN_LAUNCHER_LEFT, LOW);
        isDCOn = 0;                                          
        TMRArd_InitTimer(TIMER_PWM,(1023-launcherSpeed)/100);     // and reset the timer 
        }
        
        else {                                                // if DC is off.. 
        digitalWrite(PIN_LAUNCHER_RIGHT, HIGH);                           // turn it on
        digitalWrite(PIN_LAUNCHER_LEFT, HIGH);
        isDCOn = 1;
        TMRArd_InitTimer(TIMER_PWM,launcherSpeed/100);            // and reset the timer 
        }
  }   
}
