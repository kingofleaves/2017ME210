/*************************************************************
  File:      ME210Lab1Proof.ino
  Contents:  This program contains the TA Solution for ME210 Lab 1
             for Signal Conditioning.  Code is provided for
             serves as an introduction to event-driven programming
  Notes:     Target: Arduino Uno
             Arduino IDE version: 1.6.7

  History:
  when       who  what/why
  ----       ---  ---------------------------------------------
  2016-11-29 AHL  program created
  2017-01-21 AHL  program updated / debugged
  2017-01-24 AHL  modified for consistency with lab handout

  Arduino Interrupt References:
  http://www.atmel.com/Images/Atmel-42735-8-bit-AVR-Microcontroller-ATmega328-328P_datasheet.pdf
  http://www.instructables.com/id/Arduino-Timer-Interrupts/step2/Structuring-Timer-Interrupts/

 ************************************************************/

/*---------------Includes-----------------------------------*/

#include <Timers.h> 

/*---------------Module Defines-----------------------------*/

// Pinout
#define PIN_SIGNAL_IN 2
#define PIN_POT 3
#define PIN_SIGNAL_OUT 9

// Interrupts
#define INT_SIGNAL_IN 0

// Timers
#define TIMER_0            0
#define TIME_INTERVAL      2000

/*---------------Module Function Prototypes-----------------*/

void SetupPins(void);
void SetupTimerInterrupt(void);
void PrintIRFrequency(void);
void CountFallingEdges(void);

/*---------------Module Variables---------------------------*/

static uint8_t signalToggle;
unsigned long counter = 0, final_value; 
unsigned long previous_time=0, current_time, count1, count2; 

/*---------------Lab 1 Main Functions-----------------------*/

void setup() {
  Serial.begin(9600);
  signalToggle = 0;
  SetupPins();
  SetupTimerInterrupt();
  attachInterrupt(digitalPinToInterrupt(PIN_SIGNAL_IN), CountFallingEdges, FALLING);
  previous_time = TMRArd_GetTime(); 
  count2 = counter; 
  
}

void loop() {
 PrintIRFrequency(); 

}

/*----------------Module Functions--------------------------*/



void SetupPins() {
  pinMode(PIN_SIGNAL_OUT, OUTPUT);
}


void PrintIRFrequency() {
current_time = TMRArd_GetTime(); 
double dtime = current_time - previous_time; 
count1 = counter; 
double freq = ((double) count1-count2)/dtime; 
count2 = count1; 
previous_time = current_time; 
Serial.println(freq*1000, DEC); 

}

void CountFallingEdges() {

counter++; 

}

/******************************************************************************
  Function:    SetupTimerInterrupt
  Contents:    This function sets up the necessary registers to use Timer2 as
               a 1.25 kHz interrupt timer.  The specific details are a beyond
               the scope of this lab, but useful knowledge and can be found
               in the reference listed in the header.
  Parameters:  None
  Returns:     None
  Notes:       None
******************************************************************************/

void SetupTimerInterrupt() {
  cli();                               // Stop interrupts

  //Set Timer2 interrupt at 8kHz
  TCCR2A = 0;                          // Set entire TCCR2A register to 0
  TCCR2B = 0;                          // Same for TCCR2B
  TCNT2  = 0;                          // Initialize counter value to 0

  // Set compare match register to some initial value
  // 99 = (16*10^6) / (2500*64) - 1 (must be <256)
  OCR2A = 180;
  TCCR2A |= (1 << WGM21);              // Turn on CTC mode
  TCCR2B |= (1 << CS21);               // Set CS22 bit for 64 prescaler
  TIMSK2 |= (1 << OCIE2A);             // Enable timer compare interrupt

  sei();                               //Allow interrupts
}

/******************************************************************************
  Function:    UpdateCompareMatchRegister
  Contents:    This is a wrapper function to change the compare match register
               for Timer2 in order to change the time frequency.  This can be
               calculated using OCR2A = (16*10^6) / (freq*64) - 1 (must be <256)
  Parameters:  uint8_t newVal - A value between 0 and 255 that defines the new
               value of the compare register.
  Returns:     None
  Notes:       Input value must be a uint8_t between 0 and 255.
******************************************************************************/


/******************************************************************************
  Function:    ISR(TIMER2_COMPA_vect)
  Contents:    This defines the interrupt service routine that is called when
               the compare match register is hit by Timer2, which we set above.
               This generates a pulse wave at a frequency half that of the timer,
               since this his function takes takes two cycles for full wave-
               toggle high then toggle low.
  Parameters:  None
  Returns:     None
  Notes:
******************************************************************************/

ISR (TIMER2_COMPA_vect) {
  if (signalToggle) {
    digitalWrite(PIN_SIGNAL_OUT, HIGH);
    signalToggle = 0;
  }
  else {
    digitalWrite(PIN_SIGNAL_OUT, LOW);
    signalToggle = 1;
  }
}
