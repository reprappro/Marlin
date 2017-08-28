/*
  temperature.h - temperature controller
  Part of Marlin

  Copyright (c) 2011 Erik van der Zalm

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef temperature_h
#define temperature_h 

#include "Marlin.h"
#include "planner.h"
#include "slave_comms.h"

// If we are using a slave board we have multiple extruders, but we only have to worry
// about the temperature of the first one of them.

#ifdef REPRAPPRO_MULTIMATERIALS
#define EXTRUDERS_T 1
#else
#define EXTRUDERS_T EXTRUDERS
#endif

// public functions
void tp_init();  //initialise the heating
void manage_heater(); //it is critical that this is called periodically.

void max_temp_error(uint8_t extruder);
void min_temp_error(uint8_t extruder);
void bed_temp_error();

void setExtruderThermistor(int8_t e, const float& b, const float& r, const float& i);
void setBedThermistor(const float& b, const float& r, const float& i);
float getExtruderBeta(int8_t e);
float getExtruderRs(int8_t e);
float getExtruderRInf(int8_t e);
float getBedBeta();
float getBedRs();
float getBedRInf();
void getThermistor(int eb, float &beta, float &resistor, float &thermistor, float &inf);
void setThermistor(int eb, const float &beta, const float &resistor, const float &thermistor, float &inf);

#ifdef PIDTEMP
void getPIDValues(int e, float &Kpi, float &Kii, float &Kdi, float &Kmi);
void setPIDValues(int e, const float &Kpi, const float &Kii, const float &Kdi, const float &Kmi);
#endif

//low leven conversion routines
// do not use this routines and variables outsie of temperature.cpp
int temp2analog(int celsius, uint8_t e);
int temp2analogBed(int celsius);
float analog2temp(int raw, uint8_t e);
float analog2tempBed(int raw);
extern int target_raw[EXTRUDERS_T];  
extern int heatingtarget_raw[EXTRUDERS_T];  
extern int current_raw[EXTRUDERS_T];
  static int minttemp[EXTRUDERS_T] = { 50 };
  static int maxttemp[EXTRUDERS_T] = { 16383 }; // the first value used for all
extern int target_raw_bed;
extern int current_raw_bed;

extern char dudMinCount;
extern char dudMaxCount;
extern char dudBedCount;

extern float Kp,Ki,Kd,Kc;
extern int Ki_Max;

#ifdef PIDTEMP
  extern float pid_setpoint[EXTRUDERS_T];
#endif
  
//high level conversion routines, for use outside of temperature.cpp
//inline so that there is no performance decrease.
//deg=degreeCelsius

#ifdef REPRAPPRO_MULTIMATERIALS
FORCE_INLINE float degHotend(uint8_t extruder) 
{
  if(extruder == 0)  
  	return analog2temp(current_raw[extruder], extruder);
  else
	return slaveDegHotend(extruder);
};

FORCE_INLINE void setTargetHotend(const float &celsius, uint8_t extruder) 
{
  if(extruder == 0)
  {  
    if(dudMinCount < 0 || dudMaxCount < 0)
    {
        if(dudMaxCount < 0)
           max_temp_error(extruder);
        else
          min_temp_error(extruder);
  	target_raw[extruder] = temp2analog(0.0, extruder);
	#ifdef PIDTEMP
  		pid_setpoint[extruder] = 0.0;
	#endif //PIDTEMP
    } else
    {
  	target_raw[extruder] = temp2analog(celsius, extruder);
	#ifdef PIDTEMP
  		pid_setpoint[extruder] = celsius;
	#endif //PIDTEMP      
    }
  } else
	slaveSetTargetHotend(celsius, extruder);
};

FORCE_INLINE float degTargetHotend(uint8_t extruder) 
{
  if(extruder == 0)  
  	return analog2temp(target_raw[extruder], extruder);
  else
	return slaveDegTargetHotend(extruder);
};

FORCE_INLINE bool isHeatingHotend(uint8_t extruder)
{
  if(extruder == 0)   
  	return target_raw[extruder] > current_raw[extruder];
  else
	return slaveIsHeatingHotend(extruder);
};

FORCE_INLINE bool isCoolingHotend(uint8_t extruder) 
{
  if(extruder == 0)  
  	return target_raw[extruder] < current_raw[extruder];
  else
	return slaveIsCoolingHotend(extruder);
};

#else

FORCE_INLINE float degHotend(uint8_t extruder) {  
  return analog2temp(current_raw[extruder], extruder);
};

FORCE_INLINE void setTargetHotend(const float &celsius, uint8_t extruder) 
{  
      if(dudMinCount < 0 || dudMaxCount < 0)
    {
        if(dudMaxCount < 0)
           max_temp_error(extruder);
        else
          min_temp_error(extruder);
  	target_raw[extruder] = temp2analog(0.0, extruder);
	#ifdef PIDTEMP
  		pid_setpoint[extruder] = 0.0;
	#endif //PIDTEMP
    } else
    {
  	target_raw[extruder] = temp2analog(celsius, extruder);
	#ifdef PIDTEMP
  		pid_setpoint[extruder] = celsius;
	#endif //PIDTEMP      
    }
  
  //target_raw[extruder] = temp2analog(celsius, extruder);
#ifdef PIDTEMP
  pid_setpoint[extruder] = celsius;
#endif //PIDTEMP
};

FORCE_INLINE float degTargetHotend(uint8_t extruder) {  
  return analog2temp(target_raw[extruder], extruder);
};

FORCE_INLINE bool isHeatingHotend(uint8_t extruder){  
  return target_raw[extruder] > current_raw[extruder];
};

FORCE_INLINE bool isCoolingHotend(uint8_t extruder) {  
  return target_raw[extruder] < current_raw[extruder];
};
#endif // REPRAPPRO_MULTIMATERIALS



FORCE_INLINE float degBed() {
  return analog2tempBed(current_raw_bed);
};

FORCE_INLINE float degTargetBed() {   
  return analog2tempBed(target_raw_bed);
};

FORCE_INLINE void setTargetBed(const float &celsius) {  
  if(dudBedCount < 0)
  {
    bed_temp_error();
    target_raw_bed = temp2analogBed(0.0);
  } else
    target_raw_bed = temp2analogBed(celsius);
};

FORCE_INLINE bool isHeatingBed() {
  return target_raw_bed > current_raw_bed;
};

FORCE_INLINE bool isCoolingBed() {
  return target_raw_bed < current_raw_bed;
};




int getHeaterPower(int heater);
void disable_heater();
void updatePID();
void max_temp_error(uint8_t e);
void min_temp_error(uint8_t e);

FORCE_INLINE void autotempShutdown(){
}

void PID_autotune(float temp);

#endif

