#ifndef _SLAVE_COMMSH
#define _SLAVE_COMMSH
/*
   Functions to drive, and to return values from, a slave processor

   Adrian Bowyer 29 July 2012
*/

#ifdef REPRAPPRO_MULTIMATERIALS

#include "slaveCommands.h"

#define TIMEOUT 6 // ms - empirically set with reliability test (2 ms works, 1 doesn't...)
#define SLAVE_BUF 64
#define SLAVE_BAUD 250000

// Slave error codes.  ORed together.

#define SLAVE_HEAT1 1
#define SLAVE_HEAT2 2
#define SLAVE_TIMEOUT 4
#define SLAVE_OVERFLOW 8

        
extern char slaveXmitBuffer[];
extern char slaveRcvBuffer[];
extern boolean setDir[];
extern boolean driveOn[];
extern boolean firstTalk;
extern boolean inSlaveMessage;
extern unsigned long timeout;
extern unsigned long slaveDelay;
extern long precision[];
extern char slaveError;


float slaveDegHotend(uint8_t heater);
void slaveSetTargetHotend(const float &celsius, uint8_t heater);
float slaveDegTargetHotend(uint8_t heater);
bool slaveIsHeatingHotend(uint8_t heater);
bool slaveIsCoolingHotend(uint8_t heater);
void slaveStep(uint8_t drive, uint8_t v);
void slaveDir(uint8_t drive, boolean forward);
void clearSlaveChannel();
void talkToSlave(char s[]);
char* listenToSlave();
void setup_slave();
char* ftoa(char *a, const float& f, int prec);

FORCE_INLINE float getFloatFromSlave(uint8_t device, char command)
{
	slaveXmitBuffer[0] = command;
	slaveXmitBuffer[1] = '0' + device - 1; // Our extruder 0 is the Master's extruder; slave's  device 0 is our e1
	slaveXmitBuffer[2] = 0;
	talkToSlave(slaveXmitBuffer);
	listenToSlave();
        //Serial.print(slaveRcvBuffer);Serial.print(" **");
	float r = atof(slaveRcvBuffer);
        slaveRcvBuffer[0] = 0; // Clear to avoid confusion
        if(r)
        return r;
        return -273.0;
}


FORCE_INLINE void slaveSetTargetHotend(const float &celsius, uint8_t heater) 
{
	slaveXmitBuffer[0] = SET_T;
	slaveXmitBuffer[1] = '0' + heater - 1; // Our extruder 0 is the Master's extruder; slave's e0 is our e1
	ftoa(&slaveXmitBuffer[2], celsius, 1); 
	talkToSlave(slaveXmitBuffer);	
}

FORCE_INLINE float slaveDegHotend(uint8_t heater) 
{
	return getFloatFromSlave(heater, GET_T);
}

FORCE_INLINE float slaveDegTargetHotend(uint8_t heater) 
{ 
 	return getFloatFromSlave(heater, GET_TT);
}

FORCE_INLINE void getSlavePIDValues(int e, float &Kpi, float &Kii, float &Kdi, float &Kmi)
{
  Kpi = getFloatFromSlave(e, GET_KP);
  delay(1);
  Kii = getFloatFromSlave(e, GET_KI);
  delay(1);
  Kdi = getFloatFromSlave(e, GET_KD);
  delay(1);
  Kmi = getFloatFromSlave(e, GET_KW);
}

FORCE_INLINE void setSlavePIDValues(int e, const float &Kpi, const float &Kii, const float &Kdi, const float &Kmi)
{
  slaveXmitBuffer[0] = SET_KP;
  slaveXmitBuffer[1] = '0' + e - 1; // Our extruder 0 is the Master's extruder; slave's e0 is our e1
  ftoa(&slaveXmitBuffer[2], Kpi, 4); 
  talkToSlave(slaveXmitBuffer);
  delay(1);
  slaveXmitBuffer[0] = SET_KI;
  ftoa(&slaveXmitBuffer[2], Kii, 4); 
  talkToSlave(slaveXmitBuffer);
  delay(1);
  slaveXmitBuffer[0] = SET_KD;
  ftoa(&slaveXmitBuffer[2], Kdi, 4); 
  talkToSlave(slaveXmitBuffer);
  delay(1);
  slaveXmitBuffer[0] = SET_KW;
  ftoa(&slaveXmitBuffer[2], Kmi, 4); 
  talkToSlave(slaveXmitBuffer);  
}



FORCE_INLINE void setSlaveExtruderThermistor(int8_t heater, const float& b, const float& r, const float& i)
{
	slaveXmitBuffer[0] = SET_B;
        slaveXmitBuffer[1] = '0' + heater - 1; // Our extruder 0 is the Master's extruder; slave's e0 is our e1
        ftoa(&slaveXmitBuffer[2], b, 1);
	talkToSlave(slaveXmitBuffer);
        delay(1);
        slaveXmitBuffer[0] = SET_R;
        ftoa(&slaveXmitBuffer[2], r, 1);
	talkToSlave(slaveXmitBuffer);
        delay(1);
        slaveXmitBuffer[0] = SET_I;
        ftoa(&slaveXmitBuffer[2], i, 4);
	talkToSlave(slaveXmitBuffer);
}

FORCE_INLINE float getSlaveExtruderBeta(int8_t e)
{
  return getFloatFromSlave(e, GET_B);
}

FORCE_INLINE float getSlaveExtruderRs(int8_t e)
{
  return getFloatFromSlave(e, GET_R);
}

FORCE_INLINE float getSlaveExtruderRInf(int8_t e)
{
  return getFloatFromSlave(e, GET_I);
}

// Dunno what these are for...
/*FORCE_INLINE bool slaveIsHeatingHotend(uint8_t heater) 
{ 
	return slaveDegHotend(heater) < slaveDegTargetHotend(heater); 
}

FORCE_INLINE bool  slaveIsCoolingHotend(uint8_t heater) 
{ 
	return !slaveIsHeatingHotend(heater) ; 
}*/

FORCE_INLINE void slaveHeatTest(uint8_t heater)
{
	slaveXmitBuffer[0] = H_TEST;
	slaveXmitBuffer[1] = '0' + heater - 1; // Our extruder 0 is the Master's extruder; slave's e0 is our e1
	slaveXmitBuffer[2] = 0;
	talkToSlave(slaveXmitBuffer);
}



// NB this assumes the drive has already been selected
FORCE_INLINE void slaveStep(uint8_t drive, uint8_t v)
{
	if(v) return; // Slave clocks on every change
	digitalWrite(SLAVE_CLOCK, !digitalRead(SLAVE_CLOCK));
}

FORCE_INLINE void slaveDir(uint8_t drive, boolean forward)
{
	if(forward)
        {
            if(setDir[drive]) return;  // For reasons that defy rational explanash, Marlin sets direction every step...
	    slaveXmitBuffer[0] = DIR_F;
	} else
        {
            if(!setDir[drive]) return;
	    slaveXmitBuffer[0] = DIR_B;
        }
	slaveXmitBuffer[1] = '0' + drive - 1; // Our extruder 0 is the Master's extruder; slave's e0 is our e1
	slaveXmitBuffer[2] = 0;
	talkToSlave(slaveXmitBuffer);
        setDir[drive] = forward;
        delay(1); // Give it a moment
}

FORCE_INLINE void slaveDrive(uint8_t drive)
{
	slaveXmitBuffer[0] = DRIVE;
	slaveXmitBuffer[1] = '0' + drive - 1; // Our extruder 0 is the Master's extruder; slave's e0 is our e1
	slaveXmitBuffer[2] = 0;
	talkToSlave(slaveXmitBuffer);
}

FORCE_INLINE void slaveDriveOff(uint8_t drive)
{
        if(!driveOn[drive]) // Marlin keeps turning things off when they're already off...
          return;
	slaveXmitBuffer[0] = NO_MOTOR;
	slaveXmitBuffer[1] = '0' + drive - 1; // Our extruder 0 is the Master's extruder; slave's e0 is our e1
	slaveXmitBuffer[2] = 0;
	talkToSlave(slaveXmitBuffer);
        driveOn[drive] = false;
}

FORCE_INLINE void slaveDriveOn(uint8_t drive)
{
        if(driveOn[drive]) // Marlin keeps turning things on when they're already on...
          return;
	slaveXmitBuffer[0] = MOTOR;
	slaveXmitBuffer[1] = '0' + drive - 1; // Our extruder 0 is the Master's extruder; slave's e0 is our e1
	slaveXmitBuffer[2] = 0;
	talkToSlave(slaveXmitBuffer);
        driveOn[drive] = true;
}


FORCE_INLINE void stopSlave()
{
	slaveXmitBuffer[0] = STOP;
	slaveXmitBuffer[1] = 0;
	talkToSlave(slaveXmitBuffer);
}

FORCE_INLINE void slaveDebug(boolean b)
{
	slaveXmitBuffer[0] = DEBUG;
        if(b)
	  slaveXmitBuffer[1] = '1';
        else
          slaveXmitBuffer[1] = '0';
        slaveXmitBuffer[2] = 0;
	talkToSlave(slaveXmitBuffer);
}


// Clear the comms channel

FORCE_INLINE void clearSlaveChannel()
{
  firstTalk = false;
  for(uint8_t i = 0; i < 10; i++)
  {
    delay(1);
    MYSERIAL1.print(END_C);
  }
  listenToSlave(); // Clear junk from input
}

FORCE_INLINE void talkToSlave(char s[]) 
{
  if(firstTalk)
    clearSlaveChannel();
  MYSERIAL1.print(BEGIN_C);
  MYSERIAL1.print(s);
  MYSERIAL1.print(END_C); 
}


FORCE_INLINE char* listenToSlave() 
{
	char c = 0;
	int8_t bp = 0;
        slaveDelay = 0;
        inSlaveMessage = false;
        slaveError &= ~SLAVE_TIMEOUT;
        slaveError &= ~SLAVE_OVERFLOW;
        timeout = millis();
	while(c != END_C && (slaveDelay < TIMEOUT))
	{
          if(MYSERIAL1.available())
          {
            c = MYSERIAL1.read();
            switch(c)
            {
            case BEGIN_C:
               bp = 0;
               slaveRcvBuffer[0] = 0;
               inSlaveMessage = true;
               break;
       
            case END_C:
               if(inSlaveMessage)
               {
                 slaveRcvBuffer[bp] = 0;
                 bp = 0;
                 inSlaveMessage = false;
                 return slaveRcvBuffer;
               }
               break;
        
            default:
               if(inSlaveMessage)
               {
                 slaveRcvBuffer[bp] = c;
                 bp++;
               }
            }
            
            if(bp >= SLAVE_BUF)
            {
              bp = SLAVE_BUF-1;
              slaveRcvBuffer[bp] = 0;
              SERIAL_PROTOCOLPGM("slave receive buffer overflow: ");
              slaveError |= SLAVE_OVERFLOW;
              SERIAL_PROTOCOLLN(slaveRcvBuffer);
              slaveRcvBuffer[0] = 0;
              bp = 0;
            }
          }
          slaveDelay = millis() - timeout;
	}
        if(slaveDelay >= TIMEOUT)
          slaveError |= SLAVE_TIMEOUT;
	slaveRcvBuffer[bp] = 0;
	return slaveRcvBuffer;
}

#endif

#endif

