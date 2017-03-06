/*************************************************************

// This main function is scanIRRear, which returns 0 when no IR beacon is detected, and 
// 1 when the IR beacon is detected. IR threshold can be modified by (i) the #define line 
// below, or (ii) changing the amplification ratio of the op-amp 

 ************************************************************/

/*---------------Module Defines-----------------------------*/
#define IR_REAR A2                                                // CHANGE AS NECESSARY
#define IRRearThreshold 600                                       // CHANGE THIS THRESHOLD AS NECESSARY (max possible is around 760ish, if railing is 3.6V)

/*---------------Module Function Prototypes-----------------*/
void SetupPins(void);                                                    
int ScanIRRear(void);

/*---------------Module Variables---------------------------*/
int IRRearReading;

/*----------------Main Functions----------------------------*/

void setup() {
  Serial.begin(9600);
  SetupPins();
}

void loop() {
  int output = ScanIRRear(); 
  // Serial.println(IRRearReading);
  Serial.println(output); 
}

/*----------------Module Functions--------------------------*/

void SetupPins() {
  pinMode(IR_REAR, INPUT);
}

int ScanIRRear() {                                    // This outputs 1 when IR reading > threshold, and 0 otherwise
  IRRearReading = analogRead(IR_REAR);
  int rearReached;
  if (IRRearReading < IRRearThreshold)
    rearReached = 0;
  else
    rearReached = 1;
    return rearReached;
}

