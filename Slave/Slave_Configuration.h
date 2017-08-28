#ifndef _SLAVE_CONFIGURATIONH
#define _SLAVE_CONFIGURATIONH
/*
   Slave processor code for RepRapPro mult-extruder machines
   
   Adrian Bowyer 7 August 2012
   RepRapPro Ltd
   http://reprappro.com
   
   Licence: GPL
*/

#define REPRAPPRO_MULTIMATERIALS

#define VERSION_DATE "1.1, 2013-05-15"

#include "slaveCommands.h"

// Various...

#define MASTER Serial1  // comms to master
#define BAUD 250000        // comms speed

// So we can talk and listen to the user directly while the Master is in control
#define DEBUG_IO Serial
#define DEBUG_BAUD 115200

#define BUFLEN 64          // input string
#define RING_B 32          // DDA parameter ring buffer
#define HOT_ENDS 2         // number of heaters controlled
#define DRIVES 4           // number of steppers controlled

// === Pin definitions ===
// Sanguinololu V 1.2 or higher

#define ENABLE 0
#define DISABLE 1

#define FORWARDS 0
#define BACKWARDS 1

#define STEPS   { 15, 22, 3, 1 }
#define DIRS    { 21, 23, 2, 0 }
#define ENABLES { 14, 14, 26, 14 } 
 

#define THERMS { 7, 6 } // Analogue
#define HEATERS { 13, 12 }

#define LED_PIN 27
#define LED_BLINK 4

// Incoming master-clock interrupt on D16 (PCINT16)
// Change state on this to step a drive
  
#define INTERRUPT_PIN 16 

//===========================================================================
//=============================Thermal Settings  ============================
//===========================================================================

// Set this if you want to define the constants in the thermistor circuit
// and work out temperatures algebraically - added by AB.

// See http://en.wikipedia.org/wiki/Thermistor#B_or_.CE.B2_parameter_equation

// BETA is the B value
// RS is the value of the series resistor in ohms
// R_INF is R0.exp(-BETA/T0), where R0 is the thermistor resistance at T0 (T0 is in kelvin)
// Normally T0 is 298.15K (25 C).  If you write that expression in brackets in the #define the compiler 
// should compute it for you (i.e. it won't need to be calculated at run time).

// If the A->D converter has a range of 0..16383 and the measured voltage is V (between 0 and 16383)
// then the thermistor resistance, R = V.RS/(16383 - V)
// and the temperature, T = BETA/ln(R/R_INF)
// To get degrees celsius (instead of kelvin) add -273.15 to T

// This DOES assume that all extruders use the same thermistor type.

// To set these values experimentally, see: https://github.com/reprappro/HotEndModel

#define ABS_ZERO -273.15
//#define AD_RANGE 16383.0
#define AD_RANGE 1023.0


// RS 198-961
#define TH_BETA 3960.0
#define TH_RS 4700.0
#define TH_R_INF ( 100000.0*exp(-TH_BETA/298.15) )

#define HEATER_MAXTEMP 275.0
#define HEATER_MINTEMP -1.0

// PID constants

//#define TEMP_INTERVAL 500ul  // check temperature this many milliseconds
#define TEMP_INTERVAL 122ul  // check temperature this many milliseconds
#define PID_MAX 255 // limits current to nozzle
#define FULL_PID_BAND 150.0 // Bang bang when error greater than this
#define D_MIX 0.95
//#define KP { 0.01,  0.01 } //8
//#define KI { 0.0032, 0.0035 }
//#define KD { 0.04, 0.04 }
//#define PID_MAX_MIN { 70, 95 }
#define KPV 12
#define KIV (2.2*0.001*(float)TEMP_INTERVAL)
#define KDV (80/(0.001*(float)TEMP_INTERVAL))
#define KP { KPV,  KPV }
#define KI { KIV, KIV }
#define KD { KDV, KDV }
#define PIDM (125.0/KIV)
#define PID_I_MAX { PIDM, PIDM }
#define PID_I_MIN { 0.0, 0.0 }

#define TEST_POWER 0.25
#define TEST_INTERVAL 1
#define TEST_DURATION 240

#endif


