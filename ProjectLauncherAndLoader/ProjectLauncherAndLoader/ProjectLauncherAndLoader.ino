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
#define PIN_POT A3

// Interrupts

// Timers
#define TIMER_PWM 1


/*---------------Module Function Prototypes-----------------*/

void SetupPins(void);                                       // Allocates pin #s with their purpose
void PWM(void);                                             // PWM function for the DC motor 
void Step(void);                                            // Function for the stepper motor 
void Direc(void);                                           // Function for the motor direction


/*---------------Module Variables---------------------------*/
int isDCOn = 0; 

int dir = 0;                                                // Initial direction is LOW

/*---------------Lab 1 Main Functions-----------------------*/

void setup() {
  Serial.begin(9600);
  SetupPins();
  TMRArd_InitTimer(TIMER_PWM, 1000);
  digitalWrite(PIN_DIR, LOW);                               // Set initial dir pin to LOW 

}

void loop() {
  PWM(); 
  Step();  
  Direc();
}

/*----------------Module Functions--------------------------*/

void SetupPins() {
  pinMode(PIN_LAUNCHER_LEFT, OUTPUT); 
  pinMode(PIN_LAUNCHER_RIGHT, OUTPUT); 
  pinMode(PIN_STEP, OUTPUT); 
  pinMode(PIN_DIR, OUTPUT); 
  pinMode(PIN_POT, INPUT); 

}

// This function handles the PWM by checking if the timer has expired yet. If it hasn't,
// exit this function. If it has, check the potentiometer, change the motor's state, and then
// set a timer proportional to the potentiometer's 
void PWM(){
  if (Serial.available() == 0){                               // if there's no keypress..
    if (TMRArd_IsTimerExpired(TIMER_PWM)){                    // if timer is expired..
    int potReading = 1020;                                     // read new pot value
        if (isDCOn){                                          // then, if DC is on..
        digitalWrite(PIN_LAUNCHER_RIGHT, LOW);                            // turn it off
        digitalWrite(PIN_LAUNCHER_LEFT, LOW);
        isDCOn = 0;                                          
        TMRArd_InitTimer(TIMER_PWM,(1023-potReading)/100);     // and reset the timer 
        }
        
        else {                                                // if DC is off.. 
        digitalWrite(PIN_LAUNCHER_RIGHT, HIGH);                           // turn it on
        digitalWrite(PIN_LAUNCHER_LEFT, HIGH);
        isDCOn = 1;
        TMRArd_InitTimer(TIMER_PWM,potReading/100);            // and reset the timer 
        }
    }
    // 2nd if loop - if timer is not expired, do nothing 
    }   
  else{                                                       // if there's a keypress..
    digitalWrite(PIN_LAUNCHER_RIGHT, LOW);                                // turn DC off 
    digitalWrite(PIN_LAUNCHER_LEFT, LOW);
  }

}


// This function handles the step speed by checking if the last pulse has completed. If it hasn't,
// exit this function. If it has, check the potentiometer, set a pulse with a period
// proportional to the potentiometer value, and then begin the pulse.
void Step(){

    if(IsPulseFinished()){                                    // perform if previous pulse is finished 
    int potReading = 100;                                       // read new pot value 
    unsigned int stepPeriod = 30 + potReading*0.94819;        // period to be sent 
    // Serial.println(stepPeriod);
    InitPulse(PIN_STEP, stepPeriod);                          // Prepare to generate pulse stream 
    Pulse(5); 
    }

    else{
      // If pulse is not finished, do nothing 
    }
}

// This function changes the direction upon a keypress. It does this by checking if there's a keypress;
// if there is, it changes the direction and clears the serial monitor of the previous keypress 
void Direc(){
 if (Serial.available() != 0){                                // if there's a keypress..
    Serial.read();
    if (dir == 0) {                                             // and if direction is 'low'..
      digitalWrite(PIN_DIR, HIGH);                              // make direction high 
      dir = 1;
    }
    else {                                                      // and if direction is 'high'..
      digitalWrite(PIN_DIR, LOW);                               // make direction low 
      dir = 0;
    }
  } 
} 







