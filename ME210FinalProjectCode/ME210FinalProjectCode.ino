/*************************************************************
  File:      RaptorBasics.ino
  Contents:  This program is a warmup for ME210 Lab 0, and
             serves as an introduction to event-driven programming
  Notes:     Target: Arduino Leonardo
stage             Arduino IDE version: 1.6.7

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
#define MOTOR_PULSE_SPEED  200  // between 0 and 255 // AW: changed from 200 -> 215
#define FORWARD_INTERVAL 3
#define TURN_INTERVAL 3
#define FORWARD_PULSE 30
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
#define TIMER_STAGE_4 4

#define TIMER_ATJUNCTION 2
#define TIME_INTERVAL_ATJUNCTION 10000

#define TIMER_PWM 1

#define STOP_INTERVAL 256




/*---------------State Definitions--------------------------*/
typedef enum {
  STATE_RETURN, STATE_LAUNCH, STATE_RELOAD, STATE_EXIT_SAFESPACE, STATE_MOVE_FACTCHECK, STATE_MOVE_LAUNCH, STATE_OFF, STATE_TO_FIRST_JUNCTION, STATE_UTURN
} States_t;

typedef enum {
  STATE_FORWARD, STATE_REVERSE, STATE_LEFT, STATE_RIGHT, STATE_PIVOT_L, STATE_PIVOT_R, STATE_STOP
} MotionStates_t;


/*---------------Module Variables---------------------------*/
States_t state;
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
int leftDifferential = 50;

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
void handleMotors(MotionStates_t motionType, unsigned int pulseInterval, unsigned int pulseDur);
bool sensorCenterDark(void);
bool sensorRightDark(void);
bool sensorLeftDark(void);
void checkLeftRightSensors(void);
void activateLauncherAndLoader(void);
//launcher loader
void PWM(void);                                             // PWM function for the DC motor
//initial alignment
void rotateUntilIR(void);
//stage 4
void stage2(void);
void stage3(void);
void stage4(void);
void stage5(void);

/*---------------Raptor Main Functions----------------*/

void setup() {
  Serial.begin(9600);
  setupPins(); //setup Pins
  // Initialize States:
  handleMotors(STATE_FORWARD, 256, FORWARD_PULSE);
  stopFlywheel();
  state = STATE_OFF;
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
  switch (state) {
    case STATE_EXIT_SAFESPACE:
      stage1();
      break;
    case STATE_TO_FIRST_JUNCTION:
      stage2();
      break;
    case STATE_UTURN:
      stage3();
      break;
    case STATE_MOVE_FACTCHECK:
      stage4();
      break;
    case STATE_MOVE_LAUNCH:
      stage5();
      break;
    case STATE_LAUNCH:
      stage6();
      break;
    case STATE_RETURN:
      stage7();
      break;
    default:
      handleMotors(STATE_FORWARD, FORWARD_INTERVAL, FORWARD_PULSE);
      break;
  }
}

/*----------------Module Functions--------------------------*/
/** WORK IN PROGRESS **/
//void checkGlobalEvents(void) {
//  checkLeftRightSensors();      // check left and right sensors to keep tabs on position relative to junctions
//  checkJunction(STATE_PIVOT_R);
//  handleLineFollowing();
//}


/** TESTED AND WORKING **/
void handleJunctionTurn(MotionStates_t turnDirection) {
  // called when both left and right sensors (and center) are dark/on tape. Blocks response until robot turns to adjacent tape.
  Serial.println("Entered Turn");
  Serial.println(analogRead(PIN_SENSOR_RIGHT));
  Serial.println(analogRead(PIN_SENSOR_LEFT));
  if ((turnDirection == STATE_PIVOT_R) || atT) {
    while (!sensorCenterDark()) {
      handleMotors(turnDirection, TURN_INTERVAL, TURN_PULSE);
      // Turn right till center sensor captures tape to the right.
    }
  }
  while (sensorCenterDark()) {
    handleMotors(turnDirection, TURN_INTERVAL, TURN_PULSE);
    // Turn right till center sensor goes off current tape.
  }
  while (!sensorCenterDark()) {
    handleMotors(turnDirection, TURN_INTERVAL, TURN_PULSE);
    // Turn right till center sensor captures tape to the right.
  }
  if (turnDirection == STATE_PIVOT_L) {
    while (sensorCenterDark()) {
    handleMotors(turnDirection, TURN_INTERVAL, TURN_PULSE);
    // Turn right till center sensor goes off current tape.
    }
  }
  countLeft = 0;
  countRight = 0;
  if (turnDirection == STATE_STOP) {
    handleMotors(turnDirection, 0, 0);
  }  
  //Proceed to line-following again.
}


/** TESTED AND WORKING **/
void checkJunction(MotionStates_t turnDirection) {
  //checks for junction - if sensor towards the direction of turn goes over black tape, trigger turn.
  if (!atJunction) {
    Serial.println("Checking Junction");    
    if (sensorRightDark() && sensorLeftDark()) {
      atJunction = true;
      if (state == STATE_MOVE_LAUNCH) {
        handleMotors(STATE_PIVOT_R, STOP_INTERVAL, TURN_PULSE);
      } else {
        handleJunctionTurn(turnDirection);
      }
      TMRArd_InitTimer(TIMER_ATJUNCTION, TIME_INTERVAL_ATJUNCTION);
    } else if (sensorRightDark()) {
      handleMotors(STATE_PIVOT_R, TURN_INTERVAL, TURN_PULSE);
      Serial.println("adjust R");
    } else if (sensorLeftDark()) {
      handleMotors(STATE_PIVOT_L, TURN_INTERVAL, TURN_PULSE);
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
      handleMotors(STATE_LEFT, FORWARD_INTERVAL, FORWARD_PULSE);
  } else {      
      handleMotors(STATE_RIGHT, FORWARD_INTERVAL, FORWARD_PULSE);
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


/** TESTED AND WORKING **/
void activateLauncherAndLoader() {
  InitPulse(PIN_STEP, stepPeriod);                          // Prepare to generate pulse stream 
  TMRArd_InitTimer(TIMER_LAUNCH, TIME_INTERVAL_LAUNCH);
  while (!TMRArd_IsTimerExpired(TIMER_LAUNCH)) {
    PWM(); 
    Pulse(ONE_QUARTER);
    delay(TIME_PERIOD); 
  }
}

void stage1() {
  state = STATE_TO_FIRST_JUNCTION;
}

void stage2() {
  while (!atJunction) {
    handleLineFollowing();
    checkLeftRightSensors();
    checkJunction(STATE_PIVOT_R);
  }
  state = STATE_UTURN;
}

void stage3() {
  atT = true;
  while (!atJunction) {
    handleLineFollowing();
    checkLeftRightSensors();
    checkJunction(STATE_PIVOT_R);
  }
  atT = false;
  state = STATE_MOVE_FACTCHECK;
}

void stage4() {
  TMRArd_InitTimer(TIMER_STAGE_4, 4000);
  while (!TMRArd_IsTimerExpired(TIMER_STAGE_4)) {
    handleLineFollowing();
  }
  TMRArd_InitTimer(TIMER_STAGE_4, 5000);
  while (!TMRArd_IsTimerExpired(TIMER_STAGE_4)) {
    handleMotors(STATE_REVERSE, FORWARD_INTERVAL, FORWARD_PULSE*7);
  }
  //activateLauncherandLoader();
  TMRArd_InitTimer(TIMER_STAGE_4, 1000);
  while (!TMRArd_IsTimerExpired(TIMER_STAGE_4)) {
    handleMotors(STATE_FORWARD, FORWARD_INTERVAL, FORWARD_PULSE*4);
  }
  state=STATE_MOVE_LAUNCH;
}

void stage5() {
  handleLineFollowing();
  checkLeftRightSensors();      // check left and right sensors to keep tabs on position relative to junctions
  checkJunction(STATE_FORWARD);
  state = STATE_LAUNCH;
}

void stage6() {
  activateLauncherAndLoader();
  state = STATE_RETURN;
  leftDifferential = 30;
  handleJunctionTurn(STATE_PIVOT_L);
  atT = true;
}

/*** IN PROGRESS ***/
void stage7() {
//  handleLineFollowing();
//  checkLeftRightSensors();
//  checkJunction(STATE_PIVOT_LEFT);
  stage3();
  stage4();
  state = STATE_EXIT_SAFESPACE;
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

/***********************************
 * Function: handleMotors
 * arguments: currState (FORWARD, REVERSE, LEFT, RIGHT, PIVOT_L, PIVOT_R), pulseInterval (0-255 or 256 to stop), pulseDur (0 or longer)
 * Description: this function takes 3 arguments: 
 * currState, a MotionState which defines the motion of the robot, i.e. which way it moves (outlined below in the table), 
 * pulseInterval, the number of inactive pulses (no motor motion) between consecutive active pulses, and
 * pulseDur, the duration (in ms) of each pulse sent to the motor.
 * ----------------------------------------------------------
 * MotionState | Right Motor | Left Motor | Described Motion 
 * ----------------------------------------------------------
 * FORWARD     | FORWARD     | FORWARD    | Straight Forward 
 * REVERSE     | REVERSE     | REVERSE    | Straight Backward
 * LEFT        | FORWARD     | STOPPED    | Left Forward
 * RIGHT       | STOPPED     | FORWARD    | Right Forward
 * PIVOT_L     | FORWARD     | REVERSE    | Pivot left on the spot
 * PIVOT_R     | REVERSE     | FORWARD    | Pivot right on the spot
 * ---------------------------------------
 * 
 ***********************************/
/** Tested and working **/
void handleMotors(MotionStates_t currState, unsigned int pulseInterval, unsigned int pulseDur) {
  if(TMRArd_IsTimerExpired(TIMER_PULSE)) {
    if (waitCount == 255) waitCount = 0;
    //Serial.println(waitCount);
    
    if (waitCount < pulseInterval) {
      analogWrite(PIN_MOTOR_LEFT, 0);
      analogWrite(PIN_MOTOR_RIGHT, 0);
      waitCount++;
    } else {
      if (currState == STATE_FORWARD || currState == STATE_RIGHT || currState == STATE_PIVOT_R) {
        digitalWrite(PIN_MOTOR_LEFT_DIR, FORWARD_OUTPUT);
        //Serial.println("LEFT FORWARD");
      } else {
        digitalWrite(PIN_MOTOR_LEFT_DIR, REVERSE_OUTPUT);
        //Serial.println("LEFT REVERSE");
      }
      if (currState == STATE_FORWARD || currState == STATE_LEFT || currState == STATE_PIVOT_L) {
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
        analogWrite(PIN_MOTOR_LEFT, MOTOR_PULSE_SPEED + leftDifferential);
        //Serial.println("LEFT ++");
      }
      if (currState == STATE_RIGHT) {
        analogWrite(PIN_MOTOR_RIGHT, 0);
        //Serial.println("RIGHT 0");
      } else {
        analogWrite(PIN_MOTOR_RIGHT, MOTOR_PULSE_SPEED);
        //Serial.println("RIGHT ++");
      }
      if (currState == STATE_STOP) {
        analogWrite(PIN_MOTOR_RIGHT, 0);
        analogWrite(PIN_MOTOR_LEFT, 0);
      }
      waitCount = 0;
    }
    TMRArd_InitTimer(TIMER_PULSE, pulseDur);
  }
}
