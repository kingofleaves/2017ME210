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

/*---------------Module Defines-----------------------------*/

#define MOTOR_SPEED    150      // between 0 and 255
#define MOTOR_TURN_SPEED  250   // between 0 and 255

#define SENSOR_THRESHOLD_OFFSET 3*200 // 5/1024 * 3 * 200 ~= 2.9 V

#define RIGHT_TURN true
#define LEFT_TURN false

#define FORWARD LOW
#define REVERSE HIGH

/*---------------PIN Defines--------------------------------*/

#define PIN_SENSOR_LEFT 3
#define PIN_SENSOR_CENTER 4
#define PIN_SENSOR_RIGHT 5

#define PIN_MOTOR_LEFT 10
#define PIN_MOTOR_RIGHT 11
#define PIN_MOTOR_LEFT_DIR 12
#define PIN_MOTOR_RIGHT_DIR 13

/*---------------Timer Defines------------------------------*/

#define TIMER_LAUNCH 0
#define TIME_INTERVAL_LAUNCH 10000

#define TIMER_TURN 1
#define TIME_INTERVAL_TURN 20

#define TURN_INCREMENT_RATIO 10



/*---------------State Definitions--------------------------*/
typedef enum {
  STATE_RETURN, STATE_LAUNCH, STATE_RELOAD, STATE_EXIT_SAFESPACE, STATE_MOVE_FACTCHECK, STATE_MOVE_LAUNCH, STATE_OFF
} States_t;

typedef enum {
  STATE_MOVE_FORWARD, STATE_MOVE_FORWARD_FROM_LEFT, STATE_MOVE_FORWARD_FROM_RIGHT, STATE_TURN_LEFT, STATE_TURN_RIGHT, STATE_JUNCTION_TURN_LEFT, STATE_JUNCTION_TURN_RIGHT, STATE_JUNCTION_STOP, STATE_UTURN
} LocomotionStates_t;


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


/*---------------Module Function Prototypes-----------------*/
void SetupPins(void);

void checkGlobalEvents(void);
void checkJunction(void);

void handleLocomotion(States_t targetState);


void handleMoveForward(void);
void handleMoveForwardFromRight(void);
void handleMoveForwardFromLeft(void);
void handleStop(void);
void handleTurnLeft(void);
void handleTurnRight(void);
void handleTurnRightAim(void);
bool sensorCenterDark(void);
bool sensorRightDark(void);
bool sensorLeftDark(void);
void checkLeftRightSensors(void);


/*---------------Raptor Main Functions----------------*/

void setup() {
  Serial.begin(9600);
  setupPins();
  state = STATE_MOVE_LAUNCH;
  locoState = STATE_MOVE_FORWARD;
  TMRArd_InitTimer(TIMER_LAUNCH, TIME_INTERVAL_LAUNCH);
  TMRArd_StopTimer(TIMER_LAUNCH);

  handleMoveForward();
  //tapeThreshold = analogRead(PIN_SENSOR_CENTER) + SENSOR_THRESHOLD_OFFSET;
  tapeThreshold = SENSOR_THRESHOLD_OFFSET;
}

void loop() {
  // put your main code here, to run repeatedly:
  if (state != STATE_OFF) {
    checkGlobalEvents();
    
    // Debugging Code Below


    // End of Debugging Code
  }
}

/*----------------Module Functions--------------------------*/
void checkGlobalEvents(void) {
  switch (state) {
    case STATE_MOVE_LAUNCH:
      handleLocomotion(STATE_MOVE_LAUNCH);
      if (stateComplete) {
          state = STATE_LAUNCH;
          stateComplete = false;
      }
      break;
    case STATE_LAUNCH:
      handleLocomotion(STATE_LAUNCH);
      if (stateComplete) {
          state = STATE_RELOAD;
          stateComplete = false;
      }
      break;
    case STATE_MOVE_FACTCHECK:
      handleLocomotion(STATE_MOVE_FACTCHECK);
      if (stateComplete) {
          state = STATE_MOVE_LAUNCH;
          stateComplete = false;
      }
      break;
    case STATE_RELOAD:
      handleLocomotion(STATE_RELOAD);
      if (stateComplete) {
          state = STATE_MOVE_FACTCHECK;
          stateComplete = false;      
      }
      break;
  }
}

void handleLocomotion(States_t targetState) {
  checkLeftRightSensors();      // check left and right sensors to keep tabs on position relative to junctions
  //checkJunction(RIGHT_TURN);    // check to see we are at a junction
  switch (locoState) {          // line following algorithm
    case STATE_MOVE_FORWARD_FROM_RIGHT:
      handleMoveForwardFromRight();
      // should be turning left (right wheel moving forward)
      if (!sensorCenterDark()) {
        locoState = STATE_TURN_RIGHT;
        Serial.println("TR");
      }
      break;
      
    case STATE_MOVE_FORWARD_FROM_LEFT:
      handleMoveForwardFromLeft();
      // should be turning right (left wheel moving forward)
      if (!sensorCenterDark()) {
        locoState = STATE_TURN_LEFT;
        Serial.println("TL");
      }
      break;
      
    case STATE_TURN_RIGHT:
      //handleTurnRight();
      handleMoveForwardFromLeft();
      // Turning right on the spot
      if (sensorCenterDark()) {
        locoState = STATE_MOVE_FORWARD_FROM_LEFT;
        Serial.println("FFL");
      }
      break;
      
    case STATE_TURN_LEFT:
      //handleTurnLeft();
      handleMoveForwardFromRight();
      // Turning left on the spot
      if (sensorCenterDark()) {
        locoState = STATE_MOVE_FORWARD_FROM_RIGHT;
        Serial.println("FFR");
      }
      break;

    case STATE_JUNCTION_TURN_RIGHT:
      handleTurnRight();
      // Turning right on the spot
      if (countLeft >= 2 && countLeftEnabled && sensorCenterDark()) {
        countLeft = 0;
        countRight = 0;
        locoState = STATE_MOVE_FORWARD_FROM_LEFT;
        Serial.println("FFL, L:0 R:0");
      }
      break;
      
    case STATE_JUNCTION_TURN_LEFT:
      handleTurnLeft();
      // Turning left on the spot
      if (countRight >= 2 && countRightEnabled && sensorCenterDark()) {
        countRight = 0;
        countLeft = 0;
        locoState = STATE_MOVE_FORWARD_FROM_RIGHT;
        Serial.println("FFR, L:0 R:0");
      }
      break;
      
    case STATE_MOVE_FORWARD:
      handleMoveForward();
      // Both wheels forward
      if (sensorCenterDark()) {
        locoState = STATE_MOVE_FORWARD_FROM_RIGHT;
        Serial.println("FFR");
      }
      break;
    case STATE_JUNCTION_STOP:
      handleStop();
      // Both wheels stop
      if (timerLaunchExpired()) {
        locoState = STATE_UTURN;
        Serial.println("U-TURN");
      }
      break;
  }
}


void setupPins() {
  pinMode(PIN_MOTOR_LEFT, OUTPUT);
  pinMode(PIN_MOTOR_RIGHT, OUTPUT);
  pinMode(PIN_MOTOR_LEFT_DIR, OUTPUT);
  pinMode(PIN_MOTOR_RIGHT_DIR, OUTPUT);
  pinMode(PIN_SENSOR_LEFT, INPUT);
  pinMode(PIN_SENSOR_RIGHT, INPUT);
  pinMode(PIN_SENSOR_CENTER, INPUT);
}

void handleMoveForward(void) {
  digitalWrite(PIN_MOTOR_LEFT_DIR, FORWARD);
  digitalWrite(PIN_MOTOR_RIGHT_DIR, FORWARD);
  analogWrite(PIN_MOTOR_LEFT, MOTOR_SPEED);
  analogWrite(PIN_MOTOR_RIGHT, MOTOR_SPEED);
}

void handleMoveForwardFromLeft(void) {
  digitalWrite(PIN_MOTOR_LEFT_DIR, FORWARD);
  digitalWrite(PIN_MOTOR_RIGHT_DIR, FORWARD);
  analogWrite(PIN_MOTOR_LEFT, MOTOR_SPEED);
  analogWrite(PIN_MOTOR_RIGHT, 0);
}

void handleMoveForwardFromRight(void) {
  digitalWrite(PIN_MOTOR_LEFT_DIR, FORWARD);
  digitalWrite(PIN_MOTOR_RIGHT_DIR, FORWARD);
  analogWrite(PIN_MOTOR_LEFT, 0);
  analogWrite(PIN_MOTOR_RIGHT, MOTOR_SPEED);
}

void handleStop(void) {
  analogWrite(PIN_MOTOR_LEFT, 0);
  analogWrite(PIN_MOTOR_RIGHT, 0);
}

void handleTurnLeft(void) {
  digitalWrite(PIN_MOTOR_LEFT_DIR, REVERSE);
  digitalWrite(PIN_MOTOR_RIGHT_DIR, FORWARD);
  analogWrite(PIN_MOTOR_LEFT, MOTOR_SPEED);
  analogWrite(PIN_MOTOR_RIGHT, MOTOR_SPEED);
}

void handleTurnRight(void) {
  digitalWrite(PIN_MOTOR_LEFT_DIR, FORWARD);
  digitalWrite(PIN_MOTOR_RIGHT_DIR, REVERSE);
  analogWrite(PIN_MOTOR_LEFT, MOTOR_SPEED);
  analogWrite(PIN_MOTOR_RIGHT, MOTOR_SPEED);
}

void handleTurnRightAim(void) {
  // Turns right in pulses
  if (TMRArd_IsTimerExpired(TIMER_TURN)) {
    if (waitCount < TURN_INCREMENT_RATIO) {
      analogWrite(PIN_MOTOR_LEFT, 0);
      analogWrite(PIN_MOTOR_RIGHT, 0);
      TMRArd_InitTimer(TIMER_TURN, TIME_INTERVAL_TURN);
      waitCount++;
    } else {
      digitalWrite(PIN_MOTOR_LEFT_DIR, FORWARD);
      digitalWrite(PIN_MOTOR_RIGHT_DIR, REVERSE);
      analogWrite(PIN_MOTOR_LEFT, MOTOR_TURN_SPEED);
      analogWrite(PIN_MOTOR_RIGHT, MOTOR_TURN_SPEED);
      TMRArd_InitTimer(TIMER_TURN, TIME_INTERVAL_TURN);
      waitCount = 0;
    }
  }
}

bool sensorCenterDark(void) {
  //True when center sensor is on black tape
  return (analogRead(PIN_SENSOR_CENTER) > tapeThreshold);
}

bool sensorRightDark(void) {
  //True when right sensor is on black tape
  return (analogRead(PIN_SENSOR_RIGHT) > tapeThreshold);
}

bool sensorLeftDark(void) {
  //True when left sensor is on black tape
  return (analogRead(PIN_SENSOR_LEFT) > tapeThreshold);
}

void checkLeftRightSensors(void) {
  //checks left and right sensors and counts how many times they cross black tape.
  //resets both counts every time junction turn is finished.
  
  // Check right sensor first
  if (sensorRightDark() && countRightEnabled) {
    countRight ++;
    countRightEnabled = false;
    Serial.println("R False. val = " + countRight);
  }
  if (!sensorRightDark() && !countRightEnabled) {
    countRightEnabled = true;
    Serial.println("R True. val = " + countRight);
  }

  //Then check left sensor
  if (sensorLeftDark() && countLeftEnabled) {
    countLeft ++;
    countLeftEnabled = false;
    Serial.println("L False. val = " + countLeft);
  }
  if (!sensorLeftDark() && !countLeftEnabled) {
    countLeftEnabled = true;
    Serial.println("L True. val = " + countLeft);
  }
}

void checkJunction(bool turnDirection) {
  //checks for junction - if sensor towards the direction of turn goes over black tape, trigger turn.
  if ((turnDirection == RIGHT_TURN) && sensorRightDark()) {
    //Turn Right
    locoState = STATE_JUNCTION_TURN_RIGHT;
    Serial.println("JTR");
    handleTurnRight();
  }
  if ((turnDirection == LEFT_TURN) && sensorLeftDark()) {
    //Turn Left
    locoState = STATE_JUNCTION_TURN_LEFT;
    Serial.println("JTL");
    handleTurnLeft();
  }
}

unsigned char timerLaunchExpired(void) {
  return (TMRArd_IsTimerExpired(TIMER_LAUNCH) == TMRArd_EXPIRED);
}


