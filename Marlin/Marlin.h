// Tonokip RepRap firmware rewrite based off of Hydra-mmm firmware.
// Licence: GPL

#ifndef MARLIN_H
#define MARLIN_H

#include "Configuration.h"

#ifndef REPRAPPRO_MULTIMATERIALS
#define  HardwareSerial_h // trick to disable the standard HWserial
#endif

#define  FORCE_INLINE __attribute__((always_inline)) inline

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>

#ifndef REPRAPPRO_DUE
#include <util/delay.h>
#include <avr/pgmspace.h>
#include <avr/eeprom.h>
#include  <avr/wdt.h>
#include  <avr/interrupt.h>
#endif

#include "fastio.h"

#include "pins.h"

#if ARDUINO >= 100 
   #include "Arduino.h"
#else
   #include "WProgram.h"
#endif

#include "MarlinSerial.h"

#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

#include "WString.h"

#ifdef REPRAPPRO_MULTIMATERIALS
  #define MYSERIAL Serial
  #define MYSERIAL1 Serial1
#else

#if MOTHERBOARD == 8 // Teensylu
  #define MYSERIAL Serial
  #define MYSERIAL1 Serial1
#else
  #define MYSERIAL MSerial
#endif

#endif

//this is a unfinsihed attemp to removes a lot of warning messages, see:
// http://www.avrfreaks.net/index.php?name=PNphpBB2&file=printview&t=57011
//typedef char prog_char PROGMEM; 
// //#define PSTR    (s )        ((const PROGMEM char *)(s))
// //# define MYPGM(s) (__extension__({static prog_char __c[] = (s); &__c[0];})) 
// //#define MYPGM(s) ((const prog_char *g PROGMEM=s))
#define MYPGM(s) PSTR(s)
//#define MYPGM(s)  (__extension__({static char __c[] __attribute__((__progmem__)) = (s); &__c[0];}))  //This is the normal behaviour
//#define MYPGM(s)  (__extension__({static prog_char __c[]  = (s); &__c[0];})) //this does not work but hides the warnings


#define SERIAL_PROTOCOL(x) MYSERIAL.print(x);
#define SERIAL_PROTOCOL_F(x,y) MYSERIAL.print(x,y);
#define SERIAL_PROTOCOLPGM(x) serialprintPGM(MYPGM(x));
#define SERIAL_PROTOCOLLN(x) {MYSERIAL.print(x);MYSERIAL.write('\n');}
#define SERIAL_PROTOCOLLNPGM(x) {serialprintPGM(MYPGM(x));MYSERIAL.write('\n');}


const char errormagic[] PROGMEM ="Error:";
const char echomagic[] PROGMEM ="echo:";
#define SERIAL_ERROR_START serialprintPGM(errormagic);
#define SERIAL_ERROR(x) SERIAL_PROTOCOL(x)
#define SERIAL_ERRORPGM(x) SERIAL_PROTOCOLPGM(x)
#define SERIAL_ERRORLN(x) SERIAL_PROTOCOLLN(x)
#define SERIAL_ERRORLNPGM(x) SERIAL_PROTOCOLLNPGM(x)

#define SERIAL_ECHO_START serialprintPGM(echomagic);
#define SERIAL_ECHO(x) SERIAL_PROTOCOL(x)
#define SERIAL_ECHOPGM(x) SERIAL_PROTOCOLPGM(x)
#define SERIAL_ECHOLN(x) SERIAL_PROTOCOLLN(x)
#define SERIAL_ECHOLNPGM(x) SERIAL_PROTOCOLLNPGM(x)

#define SERIAL_ECHOPAIR(name,value) {SERIAL_ECHOPGM(name);SERIAL_ECHO(value);}


//things to write to serial from Programmemory. saves 400 to 2k of RAM.
#define SerialprintPGM(x) serialprintPGM(MYPGM(x))
FORCE_INLINE void serialprintPGM(const char *str)
{
  char ch=pgm_read_byte(str);
  while(ch)
  {
    MYSERIAL.write(ch);
    ch=pgm_read_byte(++str);
  }
}

// printing floats to 3DP
FORCE_INLINE void serialPrintFloat( float f){
  SERIAL_ECHO((int)f);
  SERIAL_ECHOPGM(".");
  int mantissa = (f - (int)f) * 1000;
  SERIAL_ECHO( abs(mantissa) );
}

void get_command();
void process_commands();

void manage_inactivity(byte debug);


#if X_ENABLE_PIN > -1
  #define  enable_x() WRITE(X_ENABLE_PIN, X_ENABLE_ON)
  #define disable_x() WRITE(X_ENABLE_PIN,!X_ENABLE_ON)
#else
  #define enable_x() ;
  #define disable_x() ;
#endif

#if Y_ENABLE_PIN > -1
  #define  enable_y() WRITE(Y_ENABLE_PIN, Y_ENABLE_ON)
  #define disable_y() WRITE(Y_ENABLE_PIN,!Y_ENABLE_ON)
#else
  #define enable_y() ;
  #define disable_y() ;
#endif

#if Z_ENABLE_PIN > -1
  #define  enable_z() WRITE(Z_ENABLE_PIN, Z_ENABLE_ON)
  #define disable_z() WRITE(Z_ENABLE_PIN,!Z_ENABLE_ON)
#else
  #define enable_z() ;
  #define disable_z() ;
#endif

       
#ifdef REPRAPPRO_MULTIMATERIALS
 #define enable_e0() WRITE(E0_ENABLE_PIN, E_ENABLE_ON)
 #define disable_e0() WRITE(E0_ENABLE_PIN,!E_ENABLE_ON)
 #define disable_e1() slaveDriveOff(1)
 #define disable_e2() slaveDriveOff(2)
 #define enable_e1() slaveDriveOn(1)
 #define enable_e2() slaveDriveOn(2)
#else

 #if defined(E0_ENABLE_PIN) && (E0_ENABLE_PIN > -1)
  #define enable_e0() WRITE(E0_ENABLE_PIN, E_ENABLE_ON)
  #define disable_e0() WRITE(E0_ENABLE_PIN,!E_ENABLE_ON)
 #else
  #define enable_e0()  /* nothing */
  #define disable_e0() /* nothing */
 #endif

 #if (EXTRUDERS > 1) && defined(E1_ENABLE_PIN) && (E1_ENABLE_PIN > -1)
  #define enable_e1() WRITE(E1_ENABLE_PIN, E_ENABLE_ON)
  #define disable_e1() WRITE(E1_ENABLE_PIN,!E_ENABLE_ON)
 #else
  #define enable_e1()  /* nothing */
  #define disable_e1() /* nothing */
 #endif

 #if (EXTRUDERS > 2) && defined(E2_ENABLE_PIN) && (E2_ENABLE_PIN > -1)
  #define enable_e2() WRITE(E2_ENABLE_PIN, E_ENABLE_ON)
  #define disable_e2() WRITE(E2_ENABLE_PIN,!E_ENABLE_ON)
 #else
  #define enable_e2()  /* nothing */
  #define disable_e2() /* nothing */
 #endif

#endif




enum AxisEnum {X_AXIS=0, Y_AXIS=1, Z_AXIS=2, E_AXIS=3};


void FlushSerialRequestResend();
void ClearToSend();

void get_coordinates();
void prepare_move();
void kill();
void Stop();
void shutDown();

boolean check_all_temps();

bool IsStopped();

void enquecommand(const char *cmd); //put an ascii command at the end of the current buffer.
void prepare_arc_move(char isclockwise);

#ifndef CRITICAL_SECTION_START
  #define CRITICAL_SECTION_START  unsigned char _sreg = SREG; cli();
  #define CRITICAL_SECTION_END    SREG = _sreg;
#endif //CRITICAL_SECTION_START

extern float homing_feedrate[];
extern float fast_home_feedrate[];
extern bool axis_relative_modes[];
extern volatile int feedmultiply;
extern int saved_feedmultiply;
extern float current_position[NUM_AXIS] ;
extern float add_homeing[3];
extern unsigned char FanSpeed;
#ifdef FAN_SOFT_PWM
extern unsigned char FanSpeedSoftPwm;
#endif

extern float destination[NUM_AXIS];
extern float modified_destination[NUM_AXIS];
extern float offset[3];
extern float feedrate, next_feedrate, saved_feedrate;


// Handling multiple extruders pins
extern uint8_t active_extruder;

#endif
