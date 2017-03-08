/*************************************************************
  File:      RaptorBasics.ino
  Contents:  This program is a warmup for ME210 Lab 0, and
             serves as an introduction to event-driven programming
  Notes:     Target: Arduino Leonardo
             Arduino IDE version: 1.6.7

  History:
  when       who  what/why
  ----       ---  ---------------------------------------------
  2016-01-09 MTP  program created
  2016-01-10 KN   Updated Raptorlib and RaptorProof to match
  2016-01-11 KLG  minor tweaks to make signed/unsigned consistent
 ************************************************************/

/*---------------Includes-----------------------------------*/

#include <Timers.h>
#include <Pulse.h>

/*---------------Module Defines-----------------------------*/

#define MOTOR_SPEED    150      // between 0 and 255
#define MOTOR_PULSE_SPEED  215   // between 0 and 255 // AW: changed from 200 -> 215
#define FORWARD_SPEED 253
#define TURN_SPEED 253
#define FORWARD_PULSE 40
#define TURN_PULSE 20


#define TURN_INCREMENT_RATIO 10

#define SENSOR_THRESHOLD_OFFSET 2*200 // 5/1024 * 2 * 200 ~= 2 V

#define RIGHT_TURN true
#define LEFT_TURN false

#define FORWARD_OUTPUT LOW
#define REVERSE_OUTPUT HIGH
#define FORWARD false
#define REVERSE true

/*---------------PIN Defines--------------------------------*/

#define PIN_SENSOR_LEFT A3
#define PIN_SENSOR_CENTER A2
#define PIN_SENSOR_RIGHT A5

#define PIN_LAUNCHER_LEFT 8
#define PIN_LAUNCHER_RIGHT 9
#define PIN_MOTOR_LEFT 10
#define PIN_MOTOR_RIGHT 11
#define PIN_MOTOR_LEFT_DIR 12
#define PIN_MOTOR_RIGHT_DIR 13

// loader digital pins
#define PIN_STEP 5
#define PIN_DIR 3 

/*---------------Other Defines------------------------------*/


// Other
#define ONE_QUARTER 33    // 53
#define TIME_PERIOD 1
#define SPEED 200


/*---------------Timer Defines------------------------------*/

#define TIMER_LAUNCH 0
#define TIME_INTERVAL_LAUNCH 10000

#define TIMER_PULSE 1

#define TIMER_ATJUNCTION 2
#define TIME_INTERVAL_ATJUNCTION 10000

#define TIMER_PWM 1




/*---------------State Definitions--------------------------*/
typedef enum {
  STATE_RETURN, STATE_LAUNCH, STATE_RELOAD, STATE_EXIT_SAFESPACE, STATE_MOVE_FACTCHECK, STATE_MOVE_LAUNCH, STATE_OFF
} States_t;

typedef enum {
  STATE_LINE_FOLLOWING, STATE_JUNCTION_TURN, STATE_SEARCH_IR
} motionModeStates_t;

typedef enum {
  STATE_MOVE_FORWARD, STATE_MOVE_FORWARD_FROM_LEFT, STATE_MOVE_FORWARD_FROM_RIGHT, STATE_TURN_LEFT, STATE_TURN_RIGHT, STATE_JUNCTION_TURN_LEFT, STATE_JUNCTION_TURN_RIGHT, STATE_JUNCTION_STOP, STATE_UTURN
} LocomotionStates_t;

typedef enum {
  STATE_FORWARD, STATE_REVERSE, STATE_LEFT, STATE_RIGHT, STATE_TURN_L, STATE_TURN_R
} MotionStates_t;


/*---------------Module Variables---------------------------*/
States_t state;
LocomotionStates_t locoState;
unsigned int tapeThreshold = 0;
unsigned int countRight = 0;
bool countRightEnabled = true;
unsigned int countLeft = 0;
bool countLeftEnabled = true;
bool turnDirection;
unsigned int waitCount = 0;
bool stateComplete = false;
bool atJunction = false;
bool atT = false;

//launcher loader
int isDCOn = 0; 
int dir = 0;                                                // Initial direction is LOW
int potReading =  SPEED;                                    // PROPORTIONAL TO SPEED 
unsigned int stepPeriod = 30 + potReading*0.94819;          // period to be sent 


/*---------------Module Function Prototypes-----------------*/
void SetupPins(void);

void checkGlobalEvents(void);
void checkJunction(void);
void handleJunctionTurn(MotionStates_t turnDirection);
void handleMotors(MotionStates_t motionType, unsigned int motorSpeed);
bool sensorCenterDark(void);
bool sensorRightDark(void);
bool sensorLeftDark(void);
void checkLeftRightSensors(void);
void activateLauncherAndLoader(void);
//launcher loader
void PWM(void);                                             // PWM function for the DC motor
//initial alignment
void rotateUntilIR(void);

/*---------------Raptor Main Functions----------------*/

void setup() {
  Serial.begin(9600);
  setupPins(); //setup Pins
  // Initialize States:
  handleMotors(STATE_FORWARD, 0, FORWARD_PULSE);
  stopFlywheel();
  state = STATE_MOVE_LAUNCH;
  locoState = STATE_MOVE_FORWARD_FROM_RIGHT;
  TMRArd_InitTimer(TIMER_LAUNCH, TIME_INTERVAL_LAUNCH);
  TMRArd_StopTimer(TIMER_LAUNCH);
  TMRArd_InitTimer(TIMER_PULSE, FORWARD_PULSE);
  TMRArd_InitTimer(TIMER_ATJUNCTION, TIME_INTERVAL_ATJUNCTION);
  tapeThreshold = SENSOR_THRESHOLD_OFFSET;

  //launcher loader
  TMRArd_InitTimer(TIMER_PWM, 1000);
  digitalWrite(PIN_DIR, LOW);                               // Set initial dir pin to LOW 
}

void loop() {
  if (state != STATE_OFF) {
    //checkGlobalEvents();
    activateLauncherAndLoader(); 
    // Debugging Code Below
    //handleLineFollowing();
    checkLeftRightSensors();      // check left and right sensors to keep tabs on position relative to junctions
    checkJunction(STATE_TURN_L);
    //Serial.println("looping");
  
    // End of Debugging Code
  }
}

/*----------------Module Functions--------------------------*/
/** WORK IN PROGRESS **/
void checkGlobalEvents(void) {
  checkLeftRightSensors();      // check left and right sensors to keep tabs on position relative to junctions
  checkJunction(STATE_TURN_R);
  handleLineFollowing();
//  switch (state) {
//    case STATE_MOVE_LAUNCH:
//      handleLocomotion(STATE_MOVE_LAUNCH);
//      if (stateComplete) {
//          state = STATE_LAUNCH;
//          stateComplete = false;
//      }
//      break;
//    case STATE_LAUNCH:
//      handleLocomotion(STATE_LAUNCH);
//      if (stateComplete) {
//          state = STATE_RELOAD;
//          stateComplete = false;
//      }
//      break;
//    case STATE_MOVE_FACTCHECK:
//      handleLocomotion(STATE_MOVE_FACTCHECK);
//      if (stateComplete) {
//          state = STATE_MOVE_LAUNCH;
//          stateComplete = false;
//      }
//      break;
//    case STATE_RELOAD:
//      handleLocomotion(STATE_RELOAD);
//      if (stateComplete) {
//          state = STATE_MOVE_FACTCHECK;
//          stateComplete = false;      
//      }
//      break;
//  }
}


/** TESTED AND WORKING **/
void handleJunctionTurn(MotionStates_t turnDirection) {
  // called when both left and right sensors (and center) are dark/on tape. Blocks response until robot turns to adjacent tape.
  Serial.println("Entered Turn");
  Serial.println(analogRead(PIN_SENSOR_RIGHT));
  Serial.println(analogRead(PIN_SENSOR_LEFT));
  if ( (turnDirection == STATE_TURN_R) || atT) {
    while (!sensorCenterDark()) {
      handleMotors(turnDirection, TURN_SPEED, TURN_PULSE);
      // Turn right till center sensor captures tape to the right.
    }
  }
  while (sensorCenterDark()) {
    handleMotors(turnDirection, TURN_SPEED, TURN_PULSE);
    // Turn right till center sensor goes off current tape.
  }
  while (!sensorCenterDark()) {
    handleMotors(turnDirection, TURN_SPEED, TURN_PULSE);
    // Turn right till center sensor captures tape to the right.
  }
  if (turnDirection == STATE_TURN_L) {
    while (sensorCenterDark()) {
    handleMotors(turnDirection, TURN_SPEED, TURN_PULSE);
    // Turn right till center sensor goes off current tape.
    }
  }
  countLeft = 0;
  countRight = 0;
  atT = true;
  //Proceed to line-following again.
}

/** STILL TESTING **/
void rotateUntilIR(void) {
  
}

/** TESTED AND WORKING **/
void checkJunction(MotionStates_t turnDirection) {
  //checks for junction - if sensor towards the direction of turn goes over black tape, trigger turn.
  if (!atJunction) {
    Serial.println("Checking Junction");    
    if (sensorRightDark() && sensorLeftDark()) {
      atJunction = true;
      handleJunctionTurn(turnDirection);
      TMRArd_InitTimer(TIMER_ATJUNCTION, TIME_INTERVAL_ATJUNCTION);
    } else if (sensorRightDark()) {
      handleMotors(STATE_TURN_R, TURN_SPEED, TURN_PULSE);
      Serial.println("adjust R");
    } else if (sensorLeftDark()) {
      handleMotors(STATE_TURN_L, TURN_SPEED, TURN_PULSE);
      Serial.println("adjust L");
    }
  } else if (countLeft >= 1 && countRight >= 1 && countLeftEnabled && countRightEnabled) {
    atJunction = false;
  }
  if (TMRArd_IsTimerExpired(TIMER_ATJUNCTION)) {
    atJunction = false;
  }
}


/** TESTED AND WORKING **/
void handleLineFollowing(void) {
// Code to follow when doing line following
  if(sensorCenterDark()) {
      handleMotors(STATE_LEFT, FORWARD_SPEED, FORWARD_PULSE);
  } else {      
      handleMotors(STATE_RIGHT, FORWARD_SPEED, FORWARD_PULSE);
  }
}


/** Tested and working **/
void setupPins() {
  pinMode(PIN_MOTOR_LEFT, OUTPUT);
  pinMode(PIN_MOTOR_RIGHT, OUTPUT);
  pinMode(PIN_MOTOR_LEFT_DIR, OUTPUT);
  pinMode(PIN_MOTOR_RIGHT_DIR, OUTPUT);
  pinMode(PIN_LAUNCHER_LEFT, OUTPUT); 
  pinMode(PIN_LAUNCHER_RIGHT, OUTPUT); 
  pinMode(PIN_STEP, OUTPUT); 
  pinMode(PIN_DIR, OUTPUT); 


}

/** Tested and working **/
bool sensorCenterDark(void) {
  //True when center sensor is on black tape
  return (analogRead(PIN_SENSOR_CENTER) > tapeThreshold);
}

/** Tested and working **/
bool sensorRightDark(void) {
  //True when right sensor is on black tape
  return (analogRead(PIN_SENSOR_RIGHT) > tapeThreshold);
}

/** Tested and working **/
bool sensorLeftDark(void) {
  //True when left sensor is on black tape
  return (analogRead(PIN_SENSOR_LEFT) > tapeThreshold);
}

/** Tested and working **/
void checkLeftRightSensors(void) {
  //checks left and right sensors and counts how many times they cross black tape.
  //resets both counts every time junction turn is finished.
  
  // Check right sensor first
  if (sensorRightDark() && countRightEnabled) {
    countRight ++;
    countRightEnabled = false;
    //Serial.print("R False. val = ");
    //Serial.println(countRight);
  }
  if (!sensorRightDark() && !countRightEnabled) {
    countRightEnabled = true;
    //Serial.print("R True. val = ");
    //Serial.println(countRight);
  }

  //Then check left sensor
  if (sensorLeftDark() && countLeftEnabled) {
    countLeft ++;
    countLeftEnabled = false;
    //Serial.print("L False. val = ");
    //Serial.println(countLeft);
  }
  if (!sensorLeftDark() && !countLeftEnabled) {
    countLeftEnabled = true;
    //Serial.print("L True. val = ");
    //Serial.println(countLeft);
  }
}


/** Tested and working **/
unsigned char timerLaunchExpired(void) {
  return (TMRArd_IsTimerExpired(TIMER_LAUNCH) == TMRArd_EXPIRED);
}

/** Tested and working **/
void stopFlywheel(void) {
  digitalWrite(PIN_LAUNCHER_RIGHT, LOW);
  digitalWrite(PIN_LAUNCHER_LEFT, LOW);
}

/***********************************
 * Function: handleMotors
 * arguments: motion (FORWARD, REVERSE, LEFT, RIGHT, TURN_L, TURN_R), motorSpeed (0 - 255)
 * 
 * 
 * 
 ***********************************/
/** Tested and working **/
void handleMotors(MotionStates_t currState, unsigned int motorSpeed, unsigned int pulseDur) {
  if(TMRArd_IsTimerExpired(TIMER_PULSE)) {
    if (waitCount == 255) waitCount = 0;
    if (motorSpeed > 255) motorSpeed = 255;  //max motorSpeed is 255.
    waitCount++;
    //Serial.println(waitCount);
    
    if (waitCount < 256 - motorSpeed) {
      analogWrite(PIN_MOTOR_LEFT, 0);
      analogWrite(PIN_MOTOR_RIGHT, 0);
    } else {
      if (currState == STATE_FORWARD || currState == STATE_RIGHT || currState == STATE_TURN_R) {
        digitalWrite(PIN_MOTOR_LEFT_DIR, FORWARD_OUTPUT);
        //Serial.println("LEFT FORWARD");
      } else {
        digitalWrite(PIN_MOTOR_LEFT_DIR, REVERSE_OUTPUT);
        //Serial.println("LEFT REVERSE");
      }
      if (currState == STATE_FORWARD || currState == STATE_LEFT || currState == STATE_TURN_L) {
        digitalWrite(PIN_MOTOR_RIGHT_DIR, FORWARD_OUTPUT);
        //Serial.println("RIGHT FORWARD");
      } else {
        digitalWrite(PIN_MOTOR_RIGHT_DIR, REVERSE_OUTPUT);
        //Serial.println("RIGHT REVERSE");
      }
      if (currState == STATE_LEFT) {
        analogWrite(PIN_MOTOR_LEFT, 0);
        //Serial.println("LEFT 0");
      } else {
        analogWrite(PIN_MOTOR_LEFT, MOTOR_PULSE_SPEED + 30);
        //Serial.println("LEFT ++");
      }
      if (currState == STATE_RIGHT) {
        analogWrite(PIN_MOTOR_RIGHT, 0);
        //Serial.println("RIGHT 0");
      } else {
        analogWrite(PIN_MOTOR_RIGHT, MOTOR_PULSE_SPEED);
        //Serial.println("RIGHT ++");
      }
      waitCount = 0;
    }
    TMRArd_InitTimer(TIMER_PULSE, pulseDur);
  }
}

/** TESTED AND WORKING **/
void activateLauncherAndLoader() {
  InitPulse(PIN_STEP, stepPeriod);                          // Prepare to generate pulse stream 
  while (true) {
    PWM(); 
    Pulse(ONE_QUARTER);
    delay(TIME_PERIOD); 
  }
}

// This function handles the loader PWM
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

/**** old functions that are replaced by others ****/


//void handleMoveForward(void) {
//  digitalWrite(PIN_MOTOR_LEFT_DIR, FORWARD);
//  digitalWrite(PIN_MOTOR_RIGHT_DIR, FORWARD);
//  analogWrite(PIN_MOTOR_LEFT, MOTOR_SPEED);
//  analogWrite(PIN_MOTOR_RIGHT, MOTOR_SPEED);
//}
//
//void handleMoveForwardFromLeft(void) {
//  digitalWrite(PIN_MOTOR_LEFT_DIR, FORWARD);
//  digitalWrite(PIN_MOTOR_RIGHT_DIR, FORWARD);
//  analogWrite(PIN_MOTOR_LEFT, MOTOR_SPEED);
//  analogWrite(PIN_MOTOR_RIGHT, 0);
//}
//
//void handleMoveForwardFromRight(void) {
//  digitalWrite(PIN_MOTOR_LEFT_DIR, FORWARD);
//  digitalWrite(PIN_MOTOR_RIGHT_DIR, FORWARD);
//  analogWrite(PIN_MOTOR_LEFT, 0);
//  analogWrite(PIN_MOTOR_RIGHT, MOTOR_SPEED);
//}
//
//void handleStop(void) {
//  analogWrite(PIN_MOTOR_LEFT, 0);
//  analogWrite(PIN_MOTOR_RIGHT, 0);
//}
//
//void handleTurnLeft(void) {
//  digitalWrite(PIN_MOTOR_LEFT_DIR, REVERSE);
//  digitalWrite(PIN_MOTOR_RIGHT_DIR, FORWARD);
//  analogWrite(PIN_MOTOR_LEFT, MOTOR_SPEED);
//  analogWrite(PIN_MOTOR_RIGHT, MOTOR_SPEED);
//}
//
//void handleTurnRight(void) {
//  digitalWrite(PIN_MOTOR_LEFT_DIR, FORWARD);
//  digitalWrite(PIN_MOTOR_RIGHT_DIR, REVERSE);
//  analogWrite(PIN_MOTOR_LEFT, MOTOR_SPEED);
//  analogWrite(PIN_MOTOR_RIGHT, MOTOR_SPEED);
//}
//
//void handleTurnRightAim(void) {
//  // Turns right in pulses
//  if (TMRArd_IsTimerExpired(TIMER_PULSE)) {
//    if (waitCount < TURN_INCREMENT_RATIO) {
//      analogWrite(PIN_MOTOR_LEFT, 0);
//      analogWrite(PIN_MOTOR_RIGHT, 0);
//      TMRArd_InitTimer(TIMER_PULSE, TIME_INTERVAL_PULSE);
//      waitCount++;
//    } else {
//      digitalWrite(PIN_MOTOR_LEFT_DIR, FORWARD);
//      digitalWrite(PIN_MOTOR_RIGHT_DIR, REVERSE);
//      analogWrite(PIN_MOTOR_LEFT, MOTOR_PULSE_SPEED);
//      analogWrite(PIN_MOTOR_RIGHT, MOTOR_PULSE_SPEED);
//      TMRArd_InitTimer(TIMER_PULSE, TIME_INTERVAL_PULSE);
//      waitCount = 0;
//    }
//  }
//}
//
//void handleForwardAim(void) {
//  // Moves Forward in pulses
//  if (TMRArd_IsTimerExpired(TIMER_PULSE)) {
//    handleMotors(STATE_FORWARD, TEST_MOTOR_SPEED);
//  }
//}
//
//void handleLocomotion(States_t targetState) {
//  checkLeftRightSensors();      // check left and right sensors to keep tabs on position relative to junctions
//  if (!atJunction) checkJunction(RIGHT_TURN);    // check to see we are at a junction
//  switch (locoState) {          // line following algorithm
//    case STATE_MOVE_FORWARD_FROM_RIGHT:
//      handleMoveForwardFromRight();
//      // should be turning left (right wheel moving forward)
//      if (!sensorCenterDark()) {
//        locoState = STATE_TURN_RIGHT;
//        //Serial.println("TR");
//        delay(1000);
//      }
//      break;
//      
//    case STATE_MOVE_FORWARD_FROM_LEFT:
//      handleMoveForwardFromLeft();
//      // should be turning right (left wheel moving forward)
//      if (!sensorCenterDark()) {
//        locoState = STATE_TURN_LEFT;
//        //Serial.println("TL");
//        delay(1000);
//      }
//      break;
//      
//    case STATE_TURN_RIGHT:
//      //handleTurnRight();
//      handleMoveForwardFromLeft();
//      // Turning right on the spot
//      if (sensorCenterDark()) {
//        locoState = STATE_MOVE_FORWARD_FROM_LEFT;
//        //Serial.println("FFL");
//        delay(1000);
//      }
//      break;
//      
//    case STATE_TURN_LEFT:
//      //handleTurnLeft();
//      handleMoveForwardFromRight();
//      // Turning left on the spot
//      if (sensorCenterDark()) {
//        locoState = STATE_MOVE_FORWARD_FROM_RIGHT;
//        //Serial.println("FFR");
//        delay(1000);
//      }
//      break;
//
//    case STATE_JUNCTION_TURN_RIGHT:
//      handleTurnRight();
//      // Turning right on the spot
//      if (countLeft >= 2 && countLeftEnabled && sensorCenterDark()) {
//        countLeft = 0;
//        countRight = 0;
//        locoState = STATE_MOVE_FORWARD_FROM_LEFT;
//        atJunction = false;
//        //Serial.println("FFL, L:0 R:0");
//        delay(1000);
//      }
//      break;
//      
//    case STATE_JUNCTION_TURN_LEFT:
//      handleTurnLeft();
//      // Turning left on the spot
//      if (countRight >= 2 && countRightEnabled && sensorCenterDark()) {
//        countRight = 0;
//        countLeft = 0;
//        locoState = STATE_MOVE_FORWARD_FROM_RIGHT;
//        atJunction = false;
//        //Serial.println("FFR, L:0 R:0");
//        delay(1000);
//      }
//      break;
//      
//    case STATE_MOVE_FORWARD:
//      handleMoveForward();
//      // Both wheels forward
//      if (sensorCenterDark()) {
//        locoState = STATE_MOVE_FORWARD_FROM_RIGHT;
//        //Serial.println("FFR");
//        delay(1000);
//      }
//      break;
//    case STATE_JUNCTION_STOP:
//      handleStop();
//      // Both wheels stop
//      if (timerLaunchExpired()) {
//        locoState = STATE_UTURN;
//        //Serial.println("U-TURN");
//        delay(1000);
//      }
//      break;
//  }
//}

