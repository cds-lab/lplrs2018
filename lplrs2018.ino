/*
   lplrs2018

   This sketch is for a fluid reward system.  It opens and closes a solenoid valve in response to a manual
   trigger and in response to a digital trigger.  A drain function opens and then closes the valve after a
   fixed period of time.

   Load cells integrated into the platform measure the weight of the fluid reservoir.  The load cells are
   read with an HX711 on a Sparkfun breakout board.  Intended use is to tare the scale and measure the
   change in weight resulting from fluid release.  The scale factor for the HX711 output is stored as a
   float at EEPROM address 0.

   This sketch offers two types of calibration:
   1.  Find HX711 scale by measuring a calibration weight.
   2.  Find flow rate by releasing fluid and measuring change in weight.

   Some technical details:
   Written for Arduino Mega 2560.

   External interrupt pins include 2, 3, 18, 19, 20, 21, of which I use 2, 3, and 18.
   D18 is INT3
   D2 is INT4
   D3 is INT5

   15 PWM pins with 6 timers:
   timer0--4, 13; 8 bit and used for delay(), millis(), micros() etc.
   timer1--11, 12; 16 bit
   timer2--9, 10; 8 bit
   timer3--2, 3, 5; 16 bit and allows PWM output that we use on pin 5.
   timer4--6, 7, 8; 16 bit
   timer5--44, 45, 46; 16 bit and used for servo library

   Available timers then are
   timer1--1 s delay
   timer2--8 ms delay for debounce
   timer4--25 ms delay for hold voltage
   timer5--1 ms delay for 1 kHz clock

   Lee Lovejoy
   March 27, 2018
   ll2833@columbia.edu
*/

#include <stdio.h>

/*
   Use bogde's excellent library for the HX711 on github: https://github.com/bogde/HX711
*/
#include <HX711.h>

/*
   EEPROM (electrically erasable programmable read-only memory) library
*/
#include <EEPROM.h>

/*
   Define pins

   Input
   D2--Manual trigger
   D3--Digital trigger
   D18--Panel button

   Output
   D5--gate power to solenoid, a PWM output
   D9--LED
   D15--Trigger copy

   HX711 control
   D4--clock
   D7--digital output
*/

#define __manualTrigger 2
#define __digitalTrigger 3
#define __panelButton 18

#define __valvePower 5
#define __LED 9
#define __triggerCopy 15

#define __hx711CLK 4
#define __hx711DOUT 7

// Create HX711 object
HX711 hx711(__hx711DOUT, __hx711CLK);

/*
   Booleans to track state
*/

volatile bool manualTrigger = false;
volatile bool digitalTrigger = false;
volatile bool draining = false;
volatile bool valveOpen = false;

/*
   Variables for calibration
*/
volatile bool inCalibration = false;
volatile bool sequenceComplete = false;
volatile unsigned int tickCounter = 0;
volatile unsigned int maxTicks = 0;
volatile unsigned int releaseCount = 0;
volatile unsigned int maxReleases = 0;

/*
   Variables for valve control
*/
const int hitVoltage = 255;
const int holdVoltage = 85;

/*
   Drain timer counter
*/

volatile unsigned int drainCycles = 0;
const unsigned int maxDrainCycles = 200;  // 200 seconds when using timer1 at 1 Hz

char menu[8][100] = {
  "Available commands from serial monitor:",
  "T#--tare scale using # samples (default 100 if # not specified)",
  "R#--read scale without units using # samples (default 100)",
  "U#--read scale with units using # samples (default 100)",
  "W#--weight calibration using # samples (default 100)",
  "F#--flow calibration using # samples (default 100)",
  "M--this menu",
  ""
};
const int nMenuItems = 8;

/*
   Interrupt Service Routine

   INT3 (pin D18, panel button)
   Initialize and start timer for debounce.
*/
ISR (INT3_vect)
{
  TIFR2 = (1 << OCF2B);     // Clear pending interrupts
  TCNT2 = 0;                // reset TCNT2
  TIMSK2 |= (1 << OCIE2B);  // enable interrupt TIMER2_COMPB_vect
}

/*
   Interrupt Service Routine

   INT4 (pin D2, manual trigger)
   Initialize and start timer for debounce.
*/

ISR (INT4_vect)
{
  TIFR2 = (1 << OCF2A);     // Clear pending interrupts
  TCNT2 = 0;                // reset TCNT2
  TIMSK2 |= (1 << OCIE2A);  // enable interrupt TIMER2_COMPA_vect
}

/*
   Interrupt Service Routine

   INT5 (pin D3, digital trigger)
   Check digital input and control valve
*/

ISR (INT5_vect)
{
  if (digitalRead(__digitalTrigger)) // Remember __digitalTrigger uses a pull-down resistor
  {
    if (!digitalTrigger)
    {
      /*
         Digital trigger state is off and digital trigger has been engaged.

         Set digital trigger state on.
         Open valve if it is not already open.
      */
      digitalTrigger = true;
      if (!valveOpen)
      {
        valveOpen = true;
        analogWrite(__valvePower, hitVoltage);
        digitalWrite(__LED, HIGH);
        digitalWrite(__triggerCopy, HIGH);
        TIFR4 = (1 << OCF4A);     // Clear pending timer4 interrupts
        TCNT4 = 0;                // reset TCNT4
        TIMSK4 |= (1 << OCIE4A);  // enable interrupt TIMER4_COMPA_vect
      }
    }
  }
  else
  {
    if (digitalTrigger)
    {
      /*
         Digital trigger state is on but digital trigger has been released.

         Set digital trigger state off.
         Close valve if it is not being held open by auto drain or manual trigger
      */
      digitalTrigger = false;
      if (!draining && !manualTrigger)
      {
        valveOpen = false;
        analogWrite(__valvePower, 0);
        digitalWrite(__LED, LOW);
        digitalWrite(__triggerCopy, LOW);
      }
    }
  }
}

/*
   Interrupt Service Routine:  deactivate drain
*/
ISR (TIMER1_COMPA_vect)
{
  if (draining && ++drainCycles > maxDrainCycles)
  {
    TIMSK1 &= (0 << OCIE1A);
    draining = false;
    if (!manualTrigger && !digitalTrigger)
    {
      valveOpen = false;
      analogWrite(__valvePower, 0);
      digitalWrite(__LED, LOW);
      digitalWrite(__triggerCopy, LOW);
    }
  }
}

/*
   Interrupt Service Routine:  check release count for calibration mode.

   Note that setting the digital trigger pin HIGH will trigger the interrupt.
*/
ISR (TIMER1_COMPB_vect)
{
  if (++releaseCount < maxReleases)
  {
    tickCounter = 0;
    pinMode(__digitalTrigger, OUTPUT);
    digitalWrite(__digitalTrigger, HIGH);
    TIFR5 = (1 << OCF5A);     // Clear pending timer5 interrupts
    TCNT5 = 0;                // reset TCNT5
    TIMSK5 |= (1 << OCIE5A);  // enable interrupt TIMER5_COMPA_vect
  }
  else
  {
    TIMSK1 &= (0 << OCIE1B);
    sequenceComplete = true;
    pinMode(__digitalTrigger, INPUT);
  }
}

/*
   Interrupt Service Routine:  delay has elapsed, so disable timer and debounce manual trigger
*/
ISR (TIMER2_COMPA_vect)
{
  TIMSK2 &= (0 << OCIE2A);            // Disable interrupt TIMER2_COMPA_vect
  if (!digitalRead(__manualTrigger))  // Remember manual trigger is on a pull-up
  {
    if (!manualTrigger)
    {
      /*
         Manaul trigger state is off and manual trigger has been engaged.

         Set manual trigger state on.
         Send hit voltage to valve if it is not already open.
         Start timer for hold voltage
      */
      manualTrigger = true;
      if (!valveOpen)
      {
        valveOpen = true;
        analogWrite(__valvePower, hitVoltage);
        digitalWrite(__LED, HIGH);
        digitalWrite(__triggerCopy, HIGH);
        TIFR4 = (1 << OCF4A);     // Clear pending timer4 interrupts
        TCNT4 = 0;                // reset TCNT4
        TIMSK4 |= (1 << OCIE4A);  // enable interrupt TIMER4_COMPA_vect
      }
    }
  }
  else
  {
    if (manualTrigger)
    {
      /*
         Manaul trigger state is on but manual trigger has been released.

         Set manual trigger state off.
         Close valve if it is not being held open by drain or digital trigger.
      */
      manualTrigger = false;
      if (!draining && !digitalTrigger)
      {
        valveOpen = false;
        analogWrite(__valvePower, 0);
        digitalWrite(__LED, LOW);
        digitalWrite(__triggerCopy, LOW);
      }
    }
  }
}

/*
   Interrupt Service Routine:  delay has elapsed, so disable timer and debounce panel button
*/
ISR (TIMER2_COMPB_vect)
{
  TIMSK2 &= (0 << OCIE2B);          // Disable interrupt TIMER2_COMPB_vect
  if (!digitalRead(__panelButton))  // Remember panel button is on a pull-up
  {
    if (!draining)
    {
      /*
         Drain state is off and panel button has been engaged

         Set drain state on and send hit voltage to valve if it is not already open.
         Start timer for hold voltage.
         Enable hit/hold timer.
         Enable drain timer.
      */
      if (!valveOpen)
      {
        draining = true;
        drainCycles = 0;
        valveOpen = true;
        analogWrite(__valvePower, hitVoltage);
        digitalWrite(__LED, HIGH);
        digitalWrite(__triggerCopy, HIGH);
        TIFR1 = (1 << OCF1A);     // Clear pending timer1 interrupts
        TCNT1 = 0;                // reset TCNT1
        TIMSK1 |= (1 << OCIE1A);  // enable itnerrupt TIMER1_COMPA_vect
        TIFR4 = (1 << OCF4A);     // Clear pending timer4 interrupts
        TCNT4 = 0;                // reset TCNT4
        TIMSK4 |= (1 << OCIE4A);  // enable interrupt TIMER4_COMPA_vect
      }
    }
    else
    {
      /*
         Drain state is on and switch has been engaged.

         Disable drain timer.
         Set drain state off.
         Close valve if it is not being held open by manual or digital trigger.
      */
      TIMSK1 &= (0 << OCIE1A);
      draining = false;
      if (!manualTrigger && !digitalTrigger)
      {
        valveOpen = false;
        analogWrite(__valvePower, 0);
        digitalWrite(__LED, LOW);
        digitalWrite(__triggerCopy, LOW);
      }
    }
  }
}

/*
   Transition to hold voltage; delay has elapsed, so disable timer and decrease power to valve
*/
ISR (TIMER4_COMPA_vect)
{
  TIMSK4 &= (0 << OCIE4A);
  if (valveOpen) analogWrite(__valvePower, holdVoltage);
}

/*
   Interrupt Service Routine:  deactivate test release after maxTicks
*/
ISR (TIMER5_COMPA_vect)
{
  if (tickCounter++ > maxTicks)
  {
    TIMSK5 &= (0 << OCIE5A);
    digitalWrite(__digitalTrigger, LOW);
  }
}

void setup() {
  float hx711Scale;
  char floatBuffer[10];

  // Set pins
  pinMode(__manualTrigger, INPUT_PULLUP);
  pinMode(__digitalTrigger, INPUT);
  pinMode(__panelButton, INPUT_PULLUP);

  pinMode(__LED, OUTPUT);
  digitalWrite(__LED, LOW);

  pinMode(__triggerCopy, OUTPUT);
  digitalWrite(__triggerCopy, LOW);

  pinMode(__valvePower, OUTPUT);
  analogWrite(__valvePower, 0);

  /*
     Configure external interrupts

     INT3 will generate an interrupt request on falling edge (ISC31 to 1 and ISC40 to 0)
     INT4 will generate an interrupt request on any logic change (ISC41 to 0 and ISC40 to 1)
     INT5 will generate an interrupt request on any logic change (ISC51 to 0 and ISC50 to 1)

     EICRA--external itnerrupt control register A
     EICRB--external interrupt control register B
     EIMSK--external interrupt mask register
  */

  EICRA |= (1 << ISC31);  // set INT3 to trigger on falling edge
  EIMSK |= (1 << INT3);   // enable INT3

  EICRB |= (1 << ISC40);  // set INT4 to trigger on any logic change
  EIMSK |= (1 << INT4);   // enable INT4

  EICRB |= (1 << ISC50);  // set INT5 to trigger on any logic change
  EIMSK |= (1 << INT5);   // enable INT5

  /*
     Configure timer / counter interrupts

     Timer1 (16 bit) set to CTC mode with CMR 15624 and prescale 1024--1 s delay
     Timer2 (8 bit) set to CTC mode with CMR 124 and prescale 1024--8 ms delay
     Timer4 (16 bit) set to CTC mode with CMR 6249 and prescale 64--25 ms delay
     Timer5 (16 bit) set to CTC mode with CMR 249 and prescale 64--1 ms delay
  */

  TCCR1A = 0;                           // clear TCCR1A
  TCCR1B = 0;                           // clear TCCR1B
  TCNT1 = 0;                            // reset counter
  OCR1A = 15624;                        // set compare match register A to 15624
  OCR1B = 15624;                        // set compare match register B to 15624
  TCCR1B |= (1 << WGM12);               // set TCCR1B bit WGM12 to enable CTC mode
  TCCR1B |= (1 << CS12) | (1 << CS10);  // set TCCR1B bits CS12 and CS10 for 1024 prescale

  TCCR2A = 0;                           // clear TCCR2A
  TCCR2B = 0;                           // clear TCCR2B
  TCNT2 = 0;                            // reset counter
  OCR2A = 124;                          // set compare match register A to 124
  OCR2B = 124;                          // set compare match register B to 124
  TCCR2A |= (1 << WGM21);               // set TCCR2A bit WGM21 to enable CTC mode
  TCCR2B |= (1 << CS22) | (1 << CS21) | (1 << CS20);  // set TCCR2B bits CS22, CS21, and CS20 for 1024 prescale

  TCCR4A = 0;                           // clear TCCR4A
  TCCR4B = 0;                           // clear TCCR4B
  TCNT4 = 0;                            // reset counter
  OCR4A = 6249;                         // set compare match register A to 6249
  TCCR4B |= (1 << WGM42);               // set TCCR4B bit WGM42 to enable CTC mode
  TCCR4B |= (1 << CS41) | (1 << CS40);  // set TCCR4B bits CS41 and CS40 for 64 prescale

  TCCR5A = 0;                           // clear TCCR5A
  TCCR5B = 0;                           // clear TCCR5B
  TCNT5 = 0;                            // reset counter
  OCR5A = 249;                          // set compare match register to 249
  TCCR5B |= (1 << WGM52);               // set TCCR5B bit WGM52 to enable CTC mode
  TCCR5B |= (1 << CS51) | (1 << CS50);  // set TCCR5B bits CS51 and CS50 for 64 prescale

  // Open serial port
  Serial.begin(115200);
  Serial.println("lplrs2018 reward system version 1.0 | Lee Lovejoy, ll2833@columbia.edu, March 2018");

  // Load the scale for HX711 from EEPROM
  EEPROM.get(0, hx711Scale);
  hx711.set_scale(hx711Scale);
  Serial.print("HX711 scale:   ");
  Serial.println(hx711.get_scale());

  // Display serial monitor menu
  for (int i = 0 ; i < nMenuItems ; i++) Serial.println(menu[i]);
}

void loop() {
  float hx711Scale = 0;
  float reading;
  float dispensedVolume = 0;
  float totalDispensedVolume = 0;
  float calibrationWeight[] = {5, 10, 20, 50};
  int openTime[] = {40, 60, 80, 100, 120, 140, 160, 180, 200, 220, 240};
  int numOpens[] = {64, 43, 32,  26,  21,  18,  16,  14,  13,  12,  11};
  int nOpenTimes = 11;
  int nSequences = 8;
  int nCalibrationWeights = 4;
  char floatBuffer[3][20];
  char textBuffer[80];
  byte precision = 4;

  int command;
  int modifier;
  int numSamples = 100;

  //  Wait for command to be available in serial buffer
  while (!Serial.available());

  //  Capture first character.  This is the command.
  command = Serial.read();

  //  Capture remainder.  This is the number of samples.
  modifier = Serial.parseInt();
  if (modifier) numSamples = modifier;

  switch (command) {

    /*
       M--print menu to serial
    */
    case 'M':
      for (int i = 0 ; i < nMenuItems ; i++) Serial.println(menu[i]);
      break;

    /*
       T--tare scale
    */
    case 'T':
      digitalWrite(__LED, HIGH);
      hx711.tare(numSamples);
      Serial.println(0);
      digitalWrite(__LED, LOW);
      break;

    /*
       R--read scale without units
    */
    case 'R':
      digitalWrite(__LED, HIGH);
      Serial.println(hx711.get_value(numSamples), 3);
      digitalWrite(__LED, LOW);
      break;

    /*
       U--read scale with units
    */
    case 'U':
      digitalWrite(__LED, HIGH);
      Serial.println(hx711.get_units(numSamples), 3);
      digitalWrite(__LED, LOW);
      break;

    /*
       W--weight calibration
    */
    case 'W':
      Serial.println("WEIGHT CALIBRATION ROUTINE");
      Serial.println("");
      Serial.println("The HX711 is tared between each reading--we are determining the scale.");
      Serial.println("");
      Serial.println("Use keyboard to proceed as instructed.");
      Serial.println("");

      for (int i = 0 ; i < nCalibrationWeights ; i++)
      {
        Serial.print("Add the ");
        Serial.print(calibrationWeight[i], 1);
        Serial.println(" g weight and send a character to tare the scale.");

        // wait for first character then clear buffer
        while (!Serial.available());
        while (Serial.available() > 0) Serial.read();

        Serial.println("Taring scale now...");
        digitalWrite(__LED, HIGH);
        hx711.tare(numSamples);
        digitalWrite(__LED, LOW);
        Serial.println("Remove the weight and send a character to read weight.");

        // wait for first character then clear buffer
        while (!Serial.available());
        while (Serial.available()) Serial.read();

        Serial.println("Measuring weight now...");
        digitalWrite(__LED, HIGH);
        reading = hx711.get_value(numSamples);
        digitalWrite(__LED, LOW);
        hx711Scale += -reading / calibrationWeight[i];
        Serial.print("Reading:  ");
        Serial.println(reading);
        Serial.println("");
      }
      hx711Scale /= nCalibrationWeights;
      Serial.print("HX711 scale is  ");
      Serial.println(hx711Scale);
      Serial.println("");
      EEPROM.put(0, hx711Scale);
      Serial.println("Wrote scale to EEPROM");

      hx711.set_scale(hx711Scale);
      Serial.println("Set scale in HX711 object");

      Serial.println("Calibration routine complete");
      break;

    /*
       F--flow calibration
    */
    case 'F':
      Serial.println("FLOW CALIBRATION ROUTINE");
      Serial.println("");
      Serial.println("Please fill the reservoir and drain to starting level.");
      Serial.println("Remember to disconnect the external digital trigger!");
      Serial.println("");
      Serial.println("Once you begin, manual trigger and auto drain disabled until calibration complete.");
      Serial.println("");
      Serial.println("Send any character to continue.");

      // wait for first character then clear buffer
      while (!Serial.available());
      while (Serial.available()) Serial.read();
      Serial.println("Starting flow calibration routine.");

      inCalibration = true;

      /*
         Calibration loop
      */
      for (int i = 0 ; i < nSequences ; i++)
      {
        for (int j = 0 ; j < nOpenTimes ; j++)
        {
          digitalWrite(__LED, HIGH);
          hx711.tare(numSamples);
          digitalWrite(__LED, LOW);

          maxTicks = openTime[j];
          releaseCount = 0;
          maxReleases = numOpens[j];
          sequenceComplete = false;

          // Don't clear the interrupt flag; that way this executes immediately
          TCNT1 = 0;                // reset TCNT1
          TIMSK1 |= (1 << OCIE1B);  // enable itnerrupt TIMER1_COMPB_vect

          while (!sequenceComplete);
          digitalWrite(__LED, HIGH);
          dispensedVolume = -hx711.get_units(numSamples);
          digitalWrite(__LED, LOW);

          /*
             Send output as text string to serial:
             duration of valve opening (ms)
             iteration number
             total dispensed volume (mL)
             dispensed volume per valve opening (mL)
             flow rate (mL per ms)
          */

          dtostrf(totalDispensedVolume, precision + 6, precision, floatBuffer[0]);
          dtostrf(dispensedVolume / numOpens[j], precision + 6, precision, floatBuffer[1]);
          dtostrf(dispensedVolume / (numOpens[j]*openTime[j]), precision + 6, precision, floatBuffer[2]);
          sprintf(textBuffer, "%4d %d %s %s %s", openTime[j], i + 1, floatBuffer[0], floatBuffer[1], floatBuffer[2]);
          Serial.println(textBuffer);

          totalDispensedVolume += dispensedVolume;
        }
      }

      Serial.println("Completed flow calibration routine.");

      inCalibration = false;
      break;
  }
}
