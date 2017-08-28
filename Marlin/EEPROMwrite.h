#ifndef EEPROM_H
#define EEPROM_H

#include "Marlin.h"
#include "planner.h"
#include "temperature.h"
#include "FPUTransform.h"
//#include <EEPROM.h>

extern float max_length[];
extern float extruder_x_off[];
extern float extruder_y_off[];
extern float extruder_z_off[];
extern float extruder_standby[];
extern float extruder_temperature[]; 

template <class T> int EEPROM_writeAnything(int &ee, const T& value)
{
  const byte* p = (const byte*)(const void*)&value;
  int i;
  for (i = 0; i < (int)sizeof(value); i++)
    eeprom_write_byte((unsigned char *)ee++, *p++);
  return i;
}

template <class T> int EEPROM_readAnything(int &ee, T& value)
{
  byte* p = (byte*)(void*)&value;
  int i;
  for (i = 0; i < (int)sizeof(value); i++)
    *p++ = eeprom_read_byte((unsigned char *)ee++);
  return i;
}
//======================================================================================




#define EEPROM_OFFSET 100


// IMPORTANT:  Whenever there are changes made to the variables stored in EEPROM
// in the functions below, also increment the version number. This makes sure that
// the default values are used whenever there is a change to the data, to prevent
// wrong data being written to the variables.
// ALSO:  always make sure the variables in the Store and retrieve sections are in the same order.
#define EEPROM_VERSION "V08"  

inline void EEPROM_StoreSettings() 
{
#ifdef EEPROM_SETTINGS
  char ver[4]= "000";
  int i=EEPROM_OFFSET;
  EEPROM_writeAnything(i,ver); // invalidate data first 
  EEPROM_writeAnything(i,axis_steps_per_unit);  
  EEPROM_writeAnything(i,max_feedrate);  
  EEPROM_writeAnything(i,max_acceleration_units_per_sq_second);
  EEPROM_writeAnything(i,acceleration);
  EEPROM_writeAnything(i,retract_acceleration);
  EEPROM_writeAnything(i,minimumfeedrate);
  EEPROM_writeAnything(i,mintravelfeedrate);
  EEPROM_writeAnything(i,minsegmenttime);
  EEPROM_writeAnything(i,max_xy_jerk);
  EEPROM_writeAnything(i,max_z_jerk);
  EEPROM_writeAnything(i,max_e_jerk);
  EEPROM_writeAnything(i,add_homeing);
  
  float Kpi, Kii, Kdi, Kmi;
  for(int e=1; e <= EXTRUDERS; e++) // 0 is the Bed, currently not implemented
  {
   #ifdef PIDTEMP
    getPIDValues(e, Kpi, Kii, Kdi, Kmi);
    #ifdef REPRAPPRO_MULTIMATERIALS
    if(e > 1 && slaveError) // If the slave is not online, store default values for that.
    {
       SERIAL_PROTOCOLLN("Slave not working! Storing default values.");
       Kpi = DEFAULT_Kp;   
       Kii = DEFAULT_Ki;
       Kdi = DEFAULT_Kd;
       Kmi = PID_INTEGRAL_DRIVE_MAX;
    }
    #endif
    EEPROM_writeAnything(i,Kpi);
    EEPROM_writeAnything(i,Kii);
    EEPROM_writeAnything(i,Kdi);
    EEPROM_writeAnything(i,Kmi);
   #else
    EEPROM_writeAnything(i,3000);
    EEPROM_writeAnything(i,0);
    EEPROM_writeAnything(i,0);
    EEPROM_writeAnything(i,0);
   #endif
  }
  
  #if defined(UMFPUSUPPORT) && (UMFPUSUPPORT > -1) 
   EEPROM_writeAnything(i,FPUEnabled);
  #else
   EEPROM_writeAnything(i,0);
  #endif
  
  for(int e=0; e < 3; e++)
    EEPROM_writeAnything(i, max_length[e]);
    
  #ifdef ADVANCE
   EEPROM_writeAnything(i,advance_k);
  #else
   EEPROM_writeAnything(i,0);
  #endif
  
  float beta, resistor, thermistor, inf;
  for(int e=0; e <= EXTRUDERS; e++) // 0 is the bed, the rest are the extruders
  {
    getThermistor(e, beta, resistor, thermistor, inf);
    EEPROM_writeAnything(i,beta);
    EEPROM_writeAnything(i,resistor);
    EEPROM_writeAnything(i,thermistor);
    EEPROM_writeAnything(i,inf);
  }
  
  for(int e=0; e < EXTRUDERS; e++) // G10 values
  {  
    EEPROM_writeAnything(i, extruder_x_off[e]);
    EEPROM_writeAnything(i, extruder_y_off[e]);
    EEPROM_writeAnything(i, extruder_z_off[e]);
    EEPROM_writeAnything(i, extruder_standby[e]);
    EEPROM_writeAnything(i, extruder_temperature[e]);  
  }
  
  char ver2[4]=EEPROM_VERSION;
  i=EEPROM_OFFSET;
  EEPROM_writeAnything(i,ver2); // validate data
  SERIAL_ECHO_START;
  SERIAL_ECHOLNPGM("Settings Stored");
#endif //EEPROM_SETTINGS
}


inline void EEPROM_printSettings()
{  // if def=true, the default values will be used
  #ifdef EEPROM_SETTINGS  
      SERIAL_ECHO_START;
      SERIAL_ECHOLNPGM("Steps per unit:");
      SERIAL_ECHO_START;
      SERIAL_ECHOPAIR("  M92 X",axis_steps_per_unit[0]);
      SERIAL_ECHOPAIR(" Y",axis_steps_per_unit[1]);
      SERIAL_ECHOPAIR(" Z",axis_steps_per_unit[2]);
      SERIAL_ECHOPAIR(" E",axis_steps_per_unit[3]);
      SERIAL_ECHOLN("");
      
    SERIAL_ECHO_START;
      SERIAL_ECHOLNPGM("Maximum feedrates (mm/s):");
      SERIAL_ECHO_START;
      SERIAL_ECHOPAIR("  M203 X",max_feedrate[0]);
      SERIAL_ECHOPAIR(" Y",max_feedrate[1] ); 
      SERIAL_ECHOPAIR(" Z", max_feedrate[2] ); 
      SERIAL_ECHOPAIR(" E", max_feedrate[3]);
      SERIAL_ECHOLN("");
    SERIAL_ECHO_START;
      SERIAL_ECHOLNPGM("Maximum Acceleration (mm/s2):");
      SERIAL_ECHO_START;
      SERIAL_ECHOPAIR("  M201 X" ,max_acceleration_units_per_sq_second[0] ); 
      SERIAL_ECHOPAIR(" Y" , max_acceleration_units_per_sq_second[1] ); 
      SERIAL_ECHOPAIR(" Z" ,max_acceleration_units_per_sq_second[2] );
      SERIAL_ECHOPAIR(" E" ,max_acceleration_units_per_sq_second[3]);
      SERIAL_ECHOLN("");
    SERIAL_ECHO_START;
      SERIAL_ECHOLNPGM("Acceleration: S=acceleration, T=retract acceleration");
      SERIAL_ECHO_START;
      SERIAL_ECHOPAIR("  M204 S",acceleration ); 
      SERIAL_ECHOPAIR(" T" ,retract_acceleration);
      SERIAL_ECHOLN("");
    SERIAL_ECHO_START;
      SERIAL_ECHOLNPGM("Advanced variables: S=Min feedrate (mm/s), T=Min travel feedrate (mm/s), B=minimum segment time (ms), X=maximum xY jerk (mm/s),  Z=maximum Z jerk (mm/s)");
      SERIAL_ECHO_START;
      SERIAL_ECHOPAIR("  M205 S",minimumfeedrate ); 
      SERIAL_ECHOPAIR(" T" ,mintravelfeedrate ); 
      SERIAL_ECHOPAIR(" B" ,minsegmenttime ); 
      SERIAL_ECHOPAIR(" X" ,max_xy_jerk ); 
      SERIAL_ECHOPAIR(" Z" ,max_z_jerk);
      SERIAL_ECHOPAIR(" E" ,max_e_jerk);
      SERIAL_ECHOLN(""); 
    SERIAL_ECHO_START;
      SERIAL_ECHOPAIR("  M206 X",add_homeing[0]);
      SERIAL_ECHOPAIR(" Y",add_homeing[1] ); 
      SERIAL_ECHOPAIR(" Z", add_homeing[2] ); 
      SERIAL_ECHOLN("");
    #ifdef PIDTEMP
      SERIAL_ECHO_START;
      SERIAL_ECHOLNPGM("PID settings:");
      SERIAL_ECHO_START;
      float Kpi, Kii, Kdi, Kmi;
      for(int e=1; e <= EXTRUDERS; e++) // 0 is the Bed, currently not implemented
      {
       getPIDValues(e, Kpi, Kii, Kdi, Kmi);
       #ifdef REPRAPPRO_MULTIMATERIALS
       if(e > 1 && slaveError) // If the slave is not online, show default values for that.
       {
         SERIAL_PROTOCOLLN("Slave not working! Default values are:");
         Kpi = DEFAULT_Kp;   
         Kii = DEFAULT_Ki;
         Kdi = DEFAULT_Kd;
         Kmi = PID_INTEGRAL_DRIVE_MAX;
       }
       #endif
       SERIAL_ECHOPAIR("   M301 H", e);
       SERIAL_ECHOPAIR(" P", Kpi); 
       SERIAL_ECHOPAIR(" I", Kii); 
       SERIAL_ECHOPAIR(" D", Kdi);
       SERIAL_ECHOPAIR(" W", Kmi);
       SERIAL_ECHOLN(""); 
      }
    #endif
    #if defined(UMFPUSUPPORT) && (UMFPUSUPPORT > -1) 
      SERIAL_ECHOPAIR(" FPU Enabled" , FPUEnabled?" yes":" no");
      SERIAL_ECHOLN(""); 
    #endif
    
    for(int e=0; e < 3; e++)
    {
        SERIAL_ECHOPAIR(" Axis ", e);
        SERIAL_ECHOPAIR(" max: ", max_length[e]);
    }
    SERIAL_ECHOLN("");
    
    #ifdef ADVANCE
     SERIAL_ECHOPAIR(" advance_k: ", advance_k);
     SERIAL_ECHOLN("");
    #endif
    
    float beta, resistor, thermistor, inf;
    for(int e=0; e <= EXTRUDERS; e++) // 0 is the bed, the rest are the extruders
    {
      getThermistor(e, beta, resistor, thermistor, inf);
      SERIAL_ECHOPAIR(" Bed/extruder ", e);
      SERIAL_ECHOPAIR(" beta: ", beta);
      SERIAL_ECHOPAIR(" resistor: ", resistor);
      SERIAL_ECHOPAIR(" thermistor: ", thermistor);
      SERIAL_ECHOPAIR(" inf: ", inf);
      SERIAL_ECHOLN("");
    }
   
    for(int e=0; e < EXTRUDERS; e++) // G10 values
    {  
      SERIAL_ECHOPAIR(" Extruder: ", e);
      SERIAL_ECHOPAIR(" dX: ", extruder_x_off[e]);
      SERIAL_ECHOPAIR(" dY: ", extruder_y_off[e]);
      SERIAL_ECHOPAIR(" dZ: ", extruder_z_off[e]);
      SERIAL_ECHOPAIR(" standby temp: ", extruder_standby[e]);
      SERIAL_ECHOPAIR(" operating temp: ", extruder_temperature[e]);
      SERIAL_ECHOLN("");  
    } 
    
  #endif
} 


inline void EEPROM_RetrieveSettings(bool def=false)
{  // if def=true, the default values will be used
  #ifdef EEPROM_SETTINGS
    int i=EEPROM_OFFSET;
    char stored_ver[4];
    char ver[4]=EEPROM_VERSION;
    EEPROM_readAnything(i,stored_ver); //read stored version
    SERIAL_PROTOCOL(" Firmware version: ");
    SERIAL_PROTOCOL(ver);
    SERIAL_PROTOCOL(", EEPROM version: ");
    SERIAL_PROTOCOLLN(stored_ver);
    if ((!def)&&(strncmp(ver,stored_ver,3)==0)) 
    {   // version number match
      EEPROM_readAnything(i,axis_steps_per_unit);  
      EEPROM_readAnything(i,max_feedrate);  
      EEPROM_readAnything(i,max_acceleration_units_per_sq_second);
      EEPROM_readAnything(i,acceleration);
      EEPROM_readAnything(i,retract_acceleration);
      EEPROM_readAnything(i,minimumfeedrate);
      EEPROM_readAnything(i,mintravelfeedrate);
      EEPROM_readAnything(i,minsegmenttime);
      EEPROM_readAnything(i,max_xy_jerk);
      EEPROM_readAnything(i,max_z_jerk);
      EEPROM_readAnything(i,max_e_jerk);
      EEPROM_readAnything(i,add_homeing);
      
      float Kpi, Kii, Kdi, Kmi;
      for(int e=1; e <= EXTRUDERS; e++) // 0 is the Bed, currently not implemented
      {
        EEPROM_readAnything(i,Kpi);
        EEPROM_readAnything(i,Kii);
        EEPROM_readAnything(i,Kdi);
        EEPROM_readAnything(i,Kmi);   
       #ifdef PIDTEMP   
          setPIDValues(e, Kpi, Kii, Kdi, Kmi);
       #endif
      }
      
      #ifndef UMFPUSUPPORT
        int FPUEnabled;
      #endif
      EEPROM_readAnything(i,FPUEnabled);
      
      for(int e=0; e < 3; e++)
        EEPROM_readAnything(i,max_length[e]);
        
      #ifndef ADVANCE
        int advance_k;
      #endif
      EEPROM_readAnything(i,advance_k);   
        
      float beta, resistor, thermistor, inf;
      for(int e=0; e <= EXTRUDERS; e++) // 0 is the bed, the rest are the extruders
      {
        EEPROM_readAnything(i,beta);
        EEPROM_readAnything(i,resistor);
        EEPROM_readAnything(i,thermistor);
        EEPROM_readAnything(i,inf);
        setThermistor(e, beta, resistor, thermistor, inf);
      }
      
      for(int e=0; e < EXTRUDERS; e++) // G10 values
      {  
        EEPROM_readAnything(i, extruder_x_off[e]);
        EEPROM_readAnything(i, extruder_y_off[e]);
        EEPROM_readAnything(i, extruder_z_off[e]);
        EEPROM_readAnything(i, extruder_standby[e]);
        EEPROM_readAnything(i, extruder_temperature[e]); 
      }
      

      SERIAL_ECHO_START;
      SERIAL_ECHOLNPGM("Stored settings retreived:");
    }
    else 
  #endif
    {
      float tmp1[]=DEFAULT_AXIS_STEPS_PER_UNIT;
      float tmp2[]=DEFAULT_MAX_FEEDRATE;
      long tmp3[]=DEFAULT_MAX_ACCELERATION;
      float tmp4[]=AXES_MAX_LENGTHS;
      for (short i=0;i<4;i++) 
      {
        axis_steps_per_unit[i]=tmp1[i];  
        max_feedrate[i]=tmp2[i];  
        max_acceleration_units_per_sq_second[i]=tmp3[i];
      }
      for (int i = 0; i < 3; i++) {
        add_homeing[i] = 0;
        max_length[i] = tmp4[i];
      }
      acceleration=DEFAULT_ACCELERATION;
      retract_acceleration=DEFAULT_RETRACT_ACCELERATION;
      minimumfeedrate=DEFAULT_MINIMUMFEEDRATE;
      minsegmenttime=DEFAULT_MINSEGMENTTIME;       
      mintravelfeedrate=DEFAULT_MINTRAVELFEEDRATE;
      max_xy_jerk=DEFAULT_XYJERK;
      max_z_jerk=DEFAULT_ZJERK;
      max_e_jerk=DEFAULT_EJERK;
      for (int i = 0; i < EXTRUDERS; i++) {
        extruder_x_off[i] = X_EXTRUDER_OFFSET;
        extruder_y_off[i] = Y_EXTRUDER_OFFSET;
        extruder_z_off[i] = Z_EXTRUDER_OFFSET;
        extruder_standby[i] = STANDBY_TEMP;
        extruder_temperature[i] = DEFAULT_TEMP;
        setExtruderThermistor(i, E_BETA, E_RS, E_R_INF);
        setPIDValues(i + 1, DEFAULT_Kp, DEFAULT_Ki, DEFAULT_Kd, PID_INTEGRAL_DRIVE_MAX);
      }
      setBedThermistor(BED_BETA, BED_RS, BED_R_INF);
      #ifdef ADVANCE
      advance_k=EXTRUDER_ADVANCE_K;
      #endif
      #ifdef UMFPUSUPPORT
      FPUEnabled = false;
      #endif
      SERIAL_ECHO_START;
      SERIAL_ECHOLN("Using Default settings:");
    }
  #ifdef EEPROM_CHITCHAT
    EEPROM_printSettings();
  #endif
}  

#endif


