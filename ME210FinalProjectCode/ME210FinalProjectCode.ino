/*************************************************************
  File:      ME210FinalProjectCode.ino
  Contents:  This program is meant to be run with Team 10's 
             robot "Grandma", so that it can accomplish the 
             tasks required to beat the block.
  Notes:     Target: Arduino Uno
stage             Arduino IDE version: 1.6.7
 ************************************************************/

/*---------------Includes-----------------------------------*/
#include <Timers.h>
#include <Pulse.h>

/*---------------Module Defines-----------------------------*/
#define SENSOR_THRESHOLD_OFFSET 1.5*200 
// offsets the threshold which determines whether a tape sensor's reading is high or low
// 5/1024 * 1.5 * 200 ~= 1.5 V

#define IRRearThreshold 650 
// IR Sensor Threshold (similar to above)

/* Definitions for HandleMotors */
#define BASE_MOTOR_SPEED  255  
//Base motor speed for the wheels that is passed to analogWrite for the pwm (between 0 and 255) 

//turn directions
#define RIGHT_TURN true
#define LEFT_TURN false

//Forward and reverse outputs for wheel motors
#define FORWARD_OUTPUT LOW
#define REVERSE_OUTPUT HIGH
#define FORWARD false
#define REVERSE true
/* End Definitions for HandleMotors */

/*---------------PIN Defines--------------------------------*/
#define PIN_SENSOR_LEFT A3      // Sensor on the left, by the wheel
#define PIN_SENSOR_CENTER A2    // Sensor in the center, at the front
#define PIN_SENSOR_RIGHT A5     // Sensor on the right, by the wheel

// Launcher Motor Pins
#define PIN_LAUNCHER_LEFT 8     // Right Launcher Motor 
#define PIN_LAUNCHER_RIGHT 9    // Left Launcher Motor

// Wheel Motor Pins
#define PIN_MOTOR_LEFT 10       // Left Wheel Motor Enable
#define PIN_MOTOR_RIGHT 11      // Right Wheel Motor Enable
#define PIN_MOTOR_LEFT_DIR 12   // Left Wheel Motor Direction
#define PIN_MOTOR_RIGHT_DIR 13  // Right Wheel Motor Direction

// Loader (Stepper Motor) Pins
#define PIN_STEP 5
#define PIN_DIR 3 

// IR Sensor Pin
#define IR_REAR A1          

/*---------------Other Defines------------------------------*/
#define ONE_QUARTER 33    
#define TIME_PERIOD 1
#define SPEED 200

/*---------------Timer Defines------------------------------*/
// Timer and duration (in ms) for each launch sequence
#define TIMER_LAUNCH 0
#define TIME_INTERVAL_LAUNCH 2000 

// Timer and duration (in ms) for pulses in handleMotors
#define TIMER_PULSE 1           
#define FORWARD_INTERVAL 3      // Interval between successive pulses in handleMotors (# of pulses skipped)
#define TURN_INTERVAL 5         // Interval between successive pulses in handleMotors (# of pulses skipped)
#define FORWARD_PULSE 30        // Duration of each pulse in handleMotors (not the pwm in analogWrite)
#define TURN_PULSE 30           // Duration of each pulse in handleMotors (not the pwm in analogWrite)

// Other miscellaneous Timer and duration definitions
#define TIMER_STAGE_4 4
#define TIMER_STAGE_2 6
#define TIMER_BETWEEN_JUNCTIONS 3

#define TIMER_ATJUNCTION 2
#define TIME_INTERVAL_ATJUNCTION 5000

#define TIMER_PWM 5




 
/*---------------State Definitions--------------------------*/
typedef enum {
  STATE_LAUNCH, STATE_EXIT_SAFESPACE, STATE_MOVE_FACTCHECK, STATE_MOVE_LAUNCH, STATE_OFF
} States_t;
// States of the robot (refer to state diagram for details)

typedef enum {
  STATE_FORWARD, STATE_REVERSE, STATE_LEFT, STATE_RIGHT, STATE_PIVOT_L, STATE_PIVOT_R, STATE_STOP
} MotionStates_t;
// States of locomotion that the robot can take. (details in comments for handleMotors function)

/*---------------Module Variables---------------------------*/
States_t state;
unsigned int tapeThreshold = 0; // Threshold for the tape sensors to determine whether it's on black or white. Will be offset by SENSOR_THRESHOLD_OFFSET.

// counters
unsigned int countRight = 0;    // counts how many tapes the right sensor has passed
bool countRightEnabled = true;  // goes to false when on tape, then true again when off tape
unsigned int countLeft = 0;     // counts how many tapes the left sensor has passed
bool countLeftEnabled = true;   // goes to false when on tape, then true again when off tape

unsigned int waitCount = 0;     // number of pulses skipped in handleMotors after last active pulse.

bool atJunction = false;        // true if currently at a junction and executing special functions
bool atT = false;               // true if the upcoming junction is a T-junction
int junctionCount = 0;          // number of junctions we have encountered

// Differentials for left and right motor speeds (to tune the motions)
int leftDifferential = 0;       
int rightDifferential = -30;


//launcher loader
int isDCOn = 0; 
int dir = 0;                                                // Initial direction is LOW
int potReading =  SPEED;                                    // PROPORTIONAL TO SPEED 
unsigned int stepPeriod = 30 + potReading*0.94819;          // period to be sent 


/*---------------Module Function Prototypes-----------------*/
void SetupPins(void);
void checkJunction(void);
void handleJunctionTurn(MotionStates_t turnDirection);
void handleMotors(MotionStates_t motionType, unsigned int pulseInterval, unsigned int pulseDur);
bool sensorCenterDark(void);
bool sensorRightDark(void);
bool sensorLeftDark(void);
void checkLeftRightSensors(void);
void activateLauncherAndLoader(void);
void PWM(void);                                             // PWM function for the DC motor
void rotateUntilIR(void);
void stage2(void);
void stage3(void);
void stage4(void);
void stage5(void);

/*---------------Raptor Main Functions----------------*/

void setup() {
  Serial.begin(9600);
  setupPins(); //setup Pins
  // Initialize States:
  handleMotors(STATE_STOP, 0, 0);
  stopFlywheel();
  state = STATE_EXIT_SAFESPACE;
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
    case STATE_MOVE_LAUNCH:
      stage2();
      break;
    case STATE_LAUNCH:
      stage3();
      break;
      case STATE_MOVE_FACTCHECK:
      stage4();
      break;
    case STATE_OFF:
      handleMotors(STATE_STOP, 0, 0);
      // stops wheels.
    default:
      handleMotors(STATE_FORWARD, FORWARD_INTERVAL, FORWARD_PULSE);
      break;
  }
}

/*----------------Module Functions--------------------------*/
/***********************************
 * Function: handleJunctionTurn
 * arguments: turnDirection
 * Description: handles the locomotion
 * for turning at a junction.to face 
 * the correct line.
 * Currently handles left and right 
 * pivot turns, and STATE_STOP which 
 * is specifically for launching at 
 * a junction.
 ***********************************/
void handleJunctionTurn(MotionStates_t turnDirection) {
  // called when both left and right sensors (and center) are dark/on tape. Blocks response until robot turns to adjacent tape.

  if ((turnDirection != STATE_PIVOT_L) || atT) {
    while (!sensorCenterDark()) {
      handleMotors(turnDirection, TURN_INTERVAL, TURN_PULSE);
      // Turn right till center sensor captures tape to the right.
    }
  }
  if (turnDirection == STATE_STOP) {
    handleMotors(turnDirection, 0, 0);
  }  else {
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
  }
  countLeft = 0;
  countRight = 0;
  atJunction = true;
  //Proceed to line-following again.
}


/** TESTED AND WORKING **/
/***********************************
 * Function: checkJunction
 * arguments: turnDirection 
 * Description: This function handles 
 * the robot's ability to detect a 
 * junction and aligns itself to it. 
 * Once aligned, (i.e. left and right
 * sensors both dark), the robot will
 * execute a function that depends on 
 * the current state it is in.
 ***********************************/
void checkJunction(MotionStates_t turnDirection) {
  if (!atJunction) {
    if (sensorRightDark() && sensorLeftDark()) {
      // both left and right sensors on tape => aligned
      handleJunctionTurn(turnDirection);
      TMRArd_InitTimer(TIMER_ATJUNCTION, TIME_INTERVAL_ATJUNCTION);
    } else if (sensorRightDark()) {
      handleMotors(STATE_PIVOT_R, TURN_INTERVAL, TURN_PULSE);
    } else if (sensorLeftDark()) {
      handleMotors(STATE_PIVOT_L, TURN_INTERVAL, TURN_PULSE);
    }
  } else if (countLeft >= 1 && countRight >= 1 && countLeftEnabled && countRightEnabled) {
    atJunction = false;
  }
  if (TMRArd_IsTimerExpired(TIMER_ATJUNCTION)) {
    atJunction = false;
  }
}


/***********************************
 * Function: handleLineFollowing
 * arguments: none
 * Description: This function handles 
 * the line following algorithm. The
 * robot turns left while on tape, 
 * and turns right when off-tape. 
 * Whether the robot is on the tape 
 * is determined by the center sensor.
 ***********************************/
 void handleLineFollowing(void) {
// Code to follow when doing line following
  if(sensorCenterDark()) {
      handleMotors(STATE_LEFT, FORWARD_INTERVAL, FORWARD_PULSE);
  } else {      
      handleMotors(STATE_RIGHT, FORWARD_INTERVAL, FORWARD_PULSE);
  }
}


/***********************************
 * Function: setupPins
 * arguments: none
 * Description: This function sets up
 * the pins on the Arduino board.
 ***********************************/
void setupPins() {
  pinMode(PIN_MOTOR_LEFT, OUTPUT);
  pinMode(PIN_MOTOR_RIGHT, OUTPUT);
  pinMode(PIN_MOTOR_LEFT_DIR, OUTPUT);
  pinMode(PIN_MOTOR_RIGHT_DIR, OUTPUT);
  pinMode(PIN_LAUNCHER_LEFT, OUTPUT); 
  pinMode(PIN_LAUNCHER_RIGHT, OUTPUT); 
  pinMode(PIN_STEP, OUTPUT); 
  pinMode(PIN_DIR, OUTPUT); 
  pinMode(IR_REAR, INPUT);

}

/***********************************
 * Function: sensorCenterDark
 * arguments: none
 * Description: returns true if center
 * sensor is on black, false if white
 ***********************************/
bool sensorCenterDark(void) {
  //True when center sensor is on black tape
  return (analogRead(PIN_SENSOR_CENTER) > tapeThreshold);
}

/***********************************
 * Function: sensorCenterDark
 * arguments: none
 * Description: returns true if right
 * sensor is on black, false if white
 ***********************************/
 bool sensorRightDark(void) {
  //True when right sensor is on black tape
  return (analogRead(PIN_SENSOR_RIGHT) > tapeThreshold);
}

/***********************************
 * Function: sensorCenterDark
 * arguments: none
 * Description: returns true if left
 * sensor is on black, false if white
 ***********************************/
bool sensorLeftDark(void) {
  //True when left sensor is on black tape
  return (analogRead(PIN_SENSOR_LEFT) > tapeThreshold);
}

/***********************************
 * Function: checkLeftRightSensors
 * arguments: none
 * Description: keeps track of how 
 * many tapes each of the left and 
 * right sensors have passed.
 ***********************************/
void checkLeftRightSensors(void) {
  //checks left and right sensors and counts how many times they cross black tape.
  //resets both counts every time junction turn is finished.
  
  // Check right sensor first
  if (sensorRightDark() && countRightEnabled) {
    countRight ++;
    countRightEnabled = false;
  }
  if (!sensorRightDark() && !countRightEnabled) {
    countRightEnabled = true;
  }

  //Then check left sensor
  if (sensorLeftDark() && countLeftEnabled) {
    countLeft ++;
    countLeftEnabled = false;
  }
  if (!sensorLeftDark() && !countLeftEnabled) {
    countLeftEnabled = true;
  }
}

/***********************************
 * Function: stopFlyWheel
 * arguments: none
 * Description: stops motors that 
 * control the flywheels.
 ***********************************/
void stopFlywheel(void) {
  digitalWrite(PIN_LAUNCHER_RIGHT, LOW);
  digitalWrite(PIN_LAUNCHER_LEFT, LOW);
}


/***********************************
 * Function: activateLauncherAndLoader
 * arguments: none
 * Description: Activates loader and 
 * launcher motors to start shooting.
 ***********************************/
void activateLauncherAndLoader() {
  InitPulse(PIN_STEP, stepPeriod);                          // Prepare to generate pulse stream 
  TMRArd_InitTimer(TIMER_LAUNCH, TIME_INTERVAL_LAUNCH);
  bool isLeft = true;
  while (!TMRArd_IsTimerExpired(TIMER_LAUNCH)) {
    PWM(); 
    Pulse(ONE_QUARTER);
    delay(TIME_PERIOD); 
  }
}

/***********************************
 * Function: stage1
 * arguments: none
 * Description: This function handles 
 * the locomotion of pivotting until
 * the robot is aligned with back 
 * facing the IR beacon. Then, it 
 * proceeds to leave the safe space.
 * Summary: 
 * - pivot till IR beacon detected
 * - Move forward till tape encountered
 * - align to tape
 * - move forward and start line following
 * Next State: Move_Launch
 ***********************************/
void stage1(void){
  int IRRearReading = 0;
  IRRearReading = analogRead(IR_REAR);
  handleMotors(STATE_PIVOT_R, 0, 40);
  
   // Pivots and blocks response until robot hits the sensor.
  while(IRRearReading < IRRearThreshold){                             
      handleMotors(STATE_PIVOT_R, 3, 40);
      IRRearReading = analogRead( IR_REAR);
  }
  
  // When it reaches here, it's aligned with the beacon. Moves forward and blocks response until robot hits the line.
  while(!sensorRightDark() && !sensorLeftDark()) {
    handleMotors(STATE_FORWARD, 3, 25);
    } 

  // When it reaches here, it has hit the line. Straighten out and blocks response until both sensors are on the line.
  while(!sensorRightDark() || !sensorLeftDark()) {
        
    if (sensorRightDark()) {
      handleMotors(STATE_PIVOT_R, 3, 40);
    } else if (sensorLeftDark()) {
      handleMotors(STATE_PIVOT_L, 3, 40);
    } else {                                                            // this is because above pivot movements may cause both sensors to go off 
      handleMotors(STATE_FORWARD, 3, 40);
    }
  } 

  state = STATE_MOVE_LAUNCH;
  TMRArd_InitTimer(TIMER_STAGE_4, 5000);
  while (!TMRArd_IsTimerExpired(TIMER_STAGE_4)) {
    handleLineFollowing();
  }
}

/***********************************
 * Function: stage2 
 * arguments: none
 * Description: This function handles 
 * the locomotion of moving to the 
 * next junction and aligning to it 
 * to shoot at the corresponding tower.
 * Summary: 
 * - follow line till junction
 * - turn left
 * - make some adjustments to align 
 *   to the junction again
 * - proceed to Launch state.
 * Next State: Launch
 ***********************************/
void stage2() {
  rightDifferential += 0;
  while (!atJunction) { 
    handleLineFollowing();
    checkLeftRightSensors();
    checkJunction(STATE_PIVOT_L);
  } // turn left at first junction
  rightDifferential -= 0;
  leftDifferential -= 25;
  TMRArd_InitTimer(TIMER_STAGE_2, 4000);
  while (!TMRArd_IsTimerExpired(TIMER_STAGE_2)) {
    handleLineFollowing();
  } // move slightly forward to align to the line.
  TMRArd_InitTimer(TIMER_STAGE_2, 300);
  while (!TMRArd_IsTimerExpired(TIMER_STAGE_2)) {
    handleMotors(STATE_REVERSE, 0, FORWARD_PULSE);
  } // reverse past junction so it can be used for alignment to the towers
  leftDifferential +=25;
  TMRArd_InitTimer(TIMER_STAGE_2, 200);
  while (!TMRArd_IsTimerExpired(TIMER_STAGE_2)) {
    handleMotors(STATE_REVERSE, 256, FORWARD_PULSE);
  } // stop wheels
  atJunction = false;
  state = STATE_LAUNCH;
  while (!atJunction) {
    handleLineFollowing();
    checkJunction(STATE_STOP); // STATE_STOP triggers special sequence to align on junction and stop.
  } // aligns robot to junction for shooting
  handleMotors(STATE_STOP, 0, 0); // Stops Wheels to get ready for launch
}

/***********************************
 * Function: stage3
 * arguments: none
 * Description: This function handles 
 * the launching of balls into the 
 * current tower
 * Summary: 
 * - turn on loader and launcher
 * - shoot 4 balls (half the load)
 * - stops loader, then launcher.
 * - proceed to next tower or Fact Checker
 * Next State: Move_Launch or Fact_Check
 ***********************************/
void stage3() {
  // Shoot!
  activateLauncherAndLoader();
  delay(2000);  // delay before stopping flywheels so the last ball has time to pass through and be launched.
  stopFlywheel();
  // Stops Shooting
  
  leftDifferential = 0;
  rightDifferential = 0;

  //check if we are at 2nd tower
  if (junctionCount!=1) {
    // if not, move to 2nd tower -> turn right and stages 2 onwards
    handleJunctionTurn(STATE_PIVOT_R);
    state = STATE_MOVE_LAUNCH;
    TMRArd_InitTimer(TIMER_BETWEEN_JUNCTIONS, 10000); 
    // sets a timer so the robot passes through first junction without turning twice
    
    while (!TMRArd_IsTimerExpired(TIMER_BETWEEN_JUNCTIONS)) {
      handleLineFollowing();
    }
    junctionCount++;
  } else {
    //both towers cleared, time for fact check.
    state = STATE_MOVE_FACTCHECK;
  }
  atJunction = false;
}

/***********************************
 * Function: stage4
 * arguments: none
 * Description: This function handles 
 * the locomotion of reversing onto 
 * the Fact Checker.
 * Summary: 
 * - follow line to straighten out
 * - reverse into Fact Checker
 * - move forward to get off
 * - enter Off state to stop the robot
 * Next State: Off
 ***********************************/
void stage4() {
  TMRArd_InitTimer(TIMER_STAGE_4, 4000);
  while (!TMRArd_IsTimerExpired(TIMER_STAGE_4)) {
    handleLineFollowing();
  } // follow line for a while to align robot
  leftDifferential -= 10;
  TMRArd_InitTimer(TIMER_STAGE_4, 3000);
  while (!TMRArd_IsTimerExpired(TIMER_STAGE_4)) {
    handleMotors(STATE_REVERSE, 0, FORWARD_PULSE);
  } // reverse onto Fact Checker
  leftDifferential -= 35;
  TMRArd_InitTimer(TIMER_STAGE_4, 500);
  while (!TMRArd_IsTimerExpired(TIMER_STAGE_4)) {
    handleMotors(STATE_FORWARD, 0, FORWARD_PULSE);
  } // move forward to get off the Fact Checker
  
  leftDifferential += 45;
  state=STATE_OFF; // COMPLETE!!!
}

/***********************************
 * Function: PWM
 * arguments: none
 * Description: This function handles 
 * the loader PWM.
 ***********************************/
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
 * arguments: currState (STATE_FORWARD, STATE_REVERSE, STATE_LEFT, STATE_RIGHT, STATE_PIVOT_L, STATE_PIVOT_R, STATE_STOP), 
 *            pulseInterval (0-255 or 256 to stop), 
 *            pulseDur (0 or longer)
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
 * STOP        | STOPPED     | STOPPED    | Stop moving
 * ---------------------------------------
 * 
 ***********************************/
/** Tested and working **/
void handleMotors(MotionStates_t currState, unsigned int pulseInterval, unsigned int pulseDur) {
  if(TMRArd_IsTimerExpired(TIMER_PULSE)) {
    if (waitCount == 255) waitCount = 0;
        
    if ((waitCount < pulseInterval) || (currState == STATE_STOP)) {
      analogWrite(PIN_MOTOR_LEFT, 0);
      analogWrite(PIN_MOTOR_RIGHT, 0);
      waitCount++;
    } else {
      if (currState == STATE_FORWARD || currState == STATE_RIGHT || currState == STATE_PIVOT_R) {
        digitalWrite(PIN_MOTOR_LEFT_DIR, FORWARD_OUTPUT);
      } else {
        digitalWrite(PIN_MOTOR_LEFT_DIR, REVERSE_OUTPUT);
      }
      if (currState == STATE_FORWARD || currState == STATE_LEFT || currState == STATE_PIVOT_L) {
        digitalWrite(PIN_MOTOR_RIGHT_DIR, FORWARD_OUTPUT);
      } else {
        digitalWrite(PIN_MOTOR_RIGHT_DIR, REVERSE_OUTPUT);
      }
      if (currState == STATE_LEFT) {
        analogWrite(PIN_MOTOR_LEFT, 0);
      } else {
        analogWrite(PIN_MOTOR_LEFT, BASE_MOTOR_SPEED + leftDifferential);
      }
      if (currState == STATE_RIGHT) {
        analogWrite(PIN_MOTOR_RIGHT, 0);
      } else {
        analogWrite(PIN_MOTOR_RIGHT, BASE_MOTOR_SPEED + rightDifferential);
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
