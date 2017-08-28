#ifndef _SLAVE_COMMANDSH
#define _SLAVE_COMMANDSH
/*
   Functions to drive, and to return values from, a slave processor

   Adrian Bowyer 29 July 2012
*/

#ifdef REPRAPPRO_MULTIMATERIALS

// NOTE: All extruder numbers throughout the code on the master processor are
// one greater than the corresponding number on the slave.  This is because the
// master has the original extruder (which it numbers 0) that the slave does not
// have.  Thus the slave's extruder 0 is the master's number 1, etc.

// Commands for communication between the master and slave

#define BEGIN_C ':'
#define END_C '\n'

#define GET_T 't'      // Get temperature
#define GET_TT 's'     // Get target temperature
#define SET_T 'T'      // Set temperature
#define SET_B 'E'      // Set thermistor beta
#define SET_R 'R'      // Set thermistor series R
#define SET_I 'I'      // Set thermistor infinite R
#define GET_B 'e'      // Get thermistor beta
#define GET_R 'r'      // Get thermistor series R
#define GET_I 'i'      // Get thermistor infinite R
#define DEBUG 'W'      // Debugging on/off
#define STOP 'S'       // Shut down everything
#define MOTOR 'M'      // Turn on motor
#define NO_MOTOR 'm'   // Turn off motor
#define NO_OP 'N'      // Do nothing
#define H_TEST 'A'     // Heater test
#define DRIVE 'D'      // Set current drive
#define DIR_F 'F'      // Set direction forward
#define DIR_B 'B'      // Set direction backwards
#define GET_KP 'j'     // Get PID Kp
#define GET_KI 'k'     // Get PID Ki
#define GET_KD 'l'     // Get PID Kd
#define GET_KW 'v'     // Get PID W
#define SET_KP 'J'     // Set PID Kp
#define SET_KI 'K'     // Set PID Ki
#define SET_KD 'L'     // Set PID Kd
#define SET_KW 'V'     // Set PID W

#define NO_DRIVE 100   // Index of a guaranteed non-existent drive

#endif

#endif

