/*
   Slave processor code for RepRapPro mult-extruder machines
   
   Adrian Bowyer 7 August 2012
   RepRapPro Ltd
   http://reprappro.com
   
   Licence: GPL
*/

// Function prototypes

char* strplus(char* a, char* b);
void error(boolean stp, char* s);
void stopSlave();
void zeroAll();
void setTemperature(int8_t heater, const float& t);
int getRawTemperature(int8_t heater);
float getTemperature(int8_t heater);
int getRawTargetTemperature(float t);
void heatControl();
void tempCheck();
void step(int8_t drive);
void test();
void command();
void incomming();
void configureInterrupt();
void setDirection(int8_t drive, bool forward);
void enable(int8_t drive);
void disable(int8_t drive);
void clearMasterChannel();

#include "Slave_Configuration.h"

unsigned long time;
char buf[BUFLEN];
char scratch[2*BUFLEN];
int bp;
boolean debug;
boolean inMessage;
boolean errorStopped = true;

// Pin arrays

int8_t steps[DRIVES] = STEPS;
int8_t dirs[DRIVES] = DIRS;
int8_t enables[DRIVES] = ENABLES;
int8_t therms[HOT_ENDS] = THERMS;
int8_t heaters[HOT_ENDS] = HEATERS;
volatile int8_t currentDrive = NO_DRIVE;
long precision[] = {0,10,100,1000,10000,100000,1000000,10000000,100000000};

// Heater arrays

float setTemps[HOT_ENDS];
//int intSetTemps[HOT_ENDS];
float currentTemps[HOT_ENDS];

// PID variables

float Kp[HOT_ENDS] = KP;
float Ki[HOT_ENDS] = KI;
float Kd[HOT_ENDS] = KD;
float PidMax[HOT_ENDS] = PID_I_MAX;
float PidMin[HOT_ENDS] = PID_I_MIN;
float temp_iState[HOT_ENDS];
float temp_dState[HOT_ENDS];
float lastTemp[HOT_ENDS];
float dt = 0.001*(float)TEMP_INTERVAL;

// Thermistor

float eBeta[HOT_ENDS]; 
float eRs[HOT_ENDS];
float eRInf[HOT_ENDS];
char dudTempCount[HOT_ENDS];
char ledBlinkCount;

/* *******************************************************************

  The master clock interrupt
*/

ISR ( PCINT2_vect ) 
{
  stepExtruder(currentDrive);
}



/* *******************************************************************

   General administration and utilities
*/

void setup() 
{
  int8_t i;
    
  errorStopped = false;
  inMessage = false;
  ledBlinkCount = 0;
  
  DEBUG_IO.begin(DEBUG_BAUD);
  DEBUG_IO.print("\nRepRapPro slave controller Version ");
  DEBUG_IO.print(VERSION_DATE);
  DEBUG_IO.print(" restarted.\n Temps:");
  debug = false;
  MASTER.begin(BAUD); 
  bp = 0;

  for(i = 0; i < DRIVES; i++)
  {
    pinMode(steps[i], OUTPUT);
    pinMode(dirs[i], OUTPUT);
    pinMode(enables[i], OUTPUT);
    disable(i);
    setDirection(i, FORWARDS);
  }
  currentDrive = NO_DRIVE;
  
  for(i = 0; i < HOT_ENDS; i++)
  {
    pinMode(therms[i], INPUT);
    pinMode(heaters[i], OUTPUT);
    analogWrite(heaters[i], 0);
    temp_iState[i] = 0.0;
    temp_dState[i] = 0.0;
    lastTemp[i] = ABS_ZERO;
    setTemperature(i, ABS_ZERO);
    currentTemps[i] = ABS_ZERO;
    eBeta[i] = TH_BETA;
    eRs[i] = TH_RS;
    eRInf[i] = TH_R_INF;
    dudTempCount[i] = 0;
    DEBUG_IO.print(getTemperature(i));
    DEBUG_IO.print(' ');
  }
  DEBUG_IO.println();
  
  if(LED_PIN >= 0)
    pinMode(LED_PIN, OUTPUT);

// See http://www.me.ucsb.edu/~me170c/Code/How_to_Enable_Interrupts_on_ANY_pin.pdf
  
  PCICR |= (1<<PCIE2);
  PCMSK2 |= (1<<PCINT16);
  MCUCR = (1<<ISC01) | (1<<ISC00); // Rising and falling edge trigger
  
  pinMode(INTERRUPT_PIN, INPUT);
  digitalWrite(INTERRUPT_PIN, HIGH); // Set pullup
  interrupts();
  time = millis() + TEMP_INTERVAL;
  clearMasterChannel();
}

inline char* strplus(char* a, char* b)
{
  strcpy(scratch, a);
  return strcat(scratch, b);
}

void zeroAll()
{
  int8_t i;
  for(i = 0; i < DRIVES; i++)
    disable(i);
  for(i = 0; i < HOT_ENDS; i++)
  {
    analogWrite(heaters[i], 0);
    setTemperature(i, ABS_ZERO);
  } 
}

void stopSlave()
{
  zeroAll();
  if(errorStopped)
    return;
  debugMessage("Stopped");
}


inline void error(boolean fatal, char* s)
{
  DEBUG_IO.println(strplus("ERROR: ", s));
  if(!fatal)
    return;
  stopSlave();
  errorStopped = true;
}

char* ftoa(char *a, const float& f, int prec)
{
  char *ret = a;
  long whole = (long)f;
  itoa(whole, a, 10);
  while (*a != '\0') a++;
  *a++ = '.';
  long decimal = abs((long)((f - (float)whole) * precision[prec]));
  itoa(decimal, a, 10);
  return ret;
}

inline void debugMessage(char* s1)
{
  if(!debug)
   return;
  DEBUG_IO.println(s1); 
}

inline void debugMessage(char* s1, char* s2)
{
  if(!debug)
   return;
  DEBUG_IO.print(s1);
  DEBUG_IO.println(s2); 
}

inline void debugMessage(char* s, int i)
{
  if(!debug)
   return;
  DEBUG_IO.print(s);
  DEBUG_IO.println(i); 
}

inline void debugMessage(char* s, float f)
{
  if(!debug)
   return;
  DEBUG_IO.print(s);
  DEBUG_IO.println(f); 
}

inline void debugMessage(char* s1, int i1, char* s2, int i2)
{
  if(!debug)
   return;
  DEBUG_IO.print(s1);
  DEBUG_IO.print(i1);
  DEBUG_IO.print(s2);
  DEBUG_IO.println(i2);  
}

inline void debugMessage(char* s1, float f1, char* s2, int i2)
{
  if(!debug)
   return;
  DEBUG_IO.print(s1);
  DEBUG_IO.print(f1);
  DEBUG_IO.print(s2);
  DEBUG_IO.println(i2);  
}

inline void talkToMaster(int i)
{
  MASTER.print(BEGIN_C);
  MASTER.print(i);
  MASTER.print(END_C);
}

inline void talkToMaster(float f)
{
  MASTER.print(BEGIN_C);
  MASTER.print(f);
  MASTER.print(END_C);
}

inline void incomming()
{ 
    if(!MASTER.available())
      return;
    
    char c = (char)MASTER.read();
    switch(c)
    {
    case BEGIN_C:
       bp = 0;
       buf[0] = 0;
       inMessage = true;
       break;
       
    case END_C:
       if(inMessage)
       {
         buf[bp] = 0;
         bp = 0;
         inMessage = false;
         command();
         return;
       }
       break;
        
    default:
       if(inMessage)
       {
         buf[bp] = c;
         bp++;
       }
    }
       
    if(bp >= BUFLEN)
    {
      bp = BUFLEN-1;
      buf[bp] = 0;
      error(false, strplus("command overflow: ", buf));
    } 
}

void clearMasterChannel()
{
  for(uint8_t i = 0; i < 10; i++)
    MASTER.print(END_C);
}

/* **********************************************************************

   Heaters and temperature
*/


inline void setTemperature(int8_t heater, const float& t)
{
  if(heater < 0 || heater >= HOT_ENDS)
    return;
  if(dudTempCount[heater] >= 0)
    setTemps[heater]=t;
  else
    setTemps[heater]=0.0;
}


inline int getRawTemperature(int8_t heater)
{
  if(heater < 0 || heater >= HOT_ENDS)
    return 0;
  return analogRead(therms[heater]);
}


inline float getTemperature(int8_t heater)
{
  if(heater < 0 || heater >= HOT_ENDS)
    return ABS_ZERO;
  float r = (float)getRawTemperature(heater);
  r = ABS_ZERO + eBeta[heater]/log( (r*eRs[heater]/(AD_RANGE - r)) /eRInf[heater] );
  currentTemps[heater] = r;
  
  if(dudTempCount[heater] >= 0)
  {
    if(r > HEATER_MAXTEMP || r < HEATER_MINTEMP)
      dudTempCount[heater]++;
    else
      dudTempCount[heater] = 0;
   
    if(dudTempCount[heater] > 3 && !errorStopped)
    {
      dudTempCount[heater] = -1;
      error(false, "temp bounds exceeded"); 
    }
  }
  
  return r;
}

inline float getTargetTemperature(int8_t heater)
{
  if(heater < 0 || heater >= HOT_ENDS)
    return ABS_ZERO;
  return setTemps[heater];
}


inline float pid(int8_t heater) 
{   
   if(heater < 0 || heater >= HOT_ENDS)
     return 0;
     
   float error, target, current;
   
   target = getTargetTemperature(heater);
   current = getTemperature(heater);   
   error = target - current;
   if(error < -FULL_PID_BAND)
   {
     temp_iState[heater] = 0;
     return 0;
   }
   if(error > FULL_PID_BAND)
   {
     temp_iState[heater] = 0;
     return 1;
   }  
   
   temp_iState[heater] += error;
   if (temp_iState[heater] < PidMin[heater]) temp_iState[heater] = PidMin[heater];
   if (temp_iState[heater] > PidMax[heater]) temp_iState[heater] = PidMax[heater];
   
   temp_dState[heater] =  Kd[heater]*(current - lastTemp[heater])*(1.0 - D_MIX) + D_MIX*temp_dState[heater]; 

   float result = Kp[heater]*error + Ki[heater]*temp_iState[heater] - temp_dState[heater];

   lastTemp[heater] = current;

   if (result < 0) result = 0;
   if (result > 255.0) result = 255.0;
   
   return result/255.0;
}


void heatControl()
{
  unsigned int power;
  for(int8_t heater = 0; heater < HOT_ENDS; heater++)
  {
     power = (unsigned int)(pid(heater)*PID_MAX);
     if(dudTempCount[heater] >= 0)
       analogWrite(heaters[heater], power);
     else
       analogWrite(heaters[heater], 0);
  }    
}

inline void tempCheck()
{    
  if((long)(time - millis()) > 0)
    return;
  time += TEMP_INTERVAL;
  
  heatControl();
    
  if(LED_PIN >= 0)
  {
    if(errorStopped)
    {
      digitalWrite(LED_PIN, 1);
      return;
    }
    ledBlinkCount++;
    if(ledBlinkCount > LED_BLINK)
    {
      digitalWrite(LED_PIN, !digitalRead(LED_PIN));
      ledBlinkCount = 0;
    }
  } 
}

void heaterTest(int8_t h)
{
  if(errorStopped)
    return;
    
  analogWrite(heaters[h], (int)(TEST_POWER*PID_MAX));
  float t = 0;
  while(t < TEST_DURATION)
  {
    DEBUG_IO.println(getTemperature(h));
    delay((int)(1000*TEST_INTERVAL));
    t += TEST_INTERVAL;
  }
  analogWrite(heaters[h], 0);
}

/* *********************************************************************

   Stepper motors and DDA
*/

inline void stepExtruder(int8_t drive)
{
  if(drive < 0 || drive >= DRIVES || errorStopped)
    return;
  digitalWrite(steps[drive],1);
  digitalWrite(steps[drive],0);
}

inline void setDirection(int8_t drive, bool dir)
{
  if(drive < 0 || drive >= DRIVES || errorStopped)
    return;
  digitalWrite(dirs[drive], dir);
}

inline void enable(int8_t drive)
{
  if(drive < 0 || drive >= DRIVES || errorStopped)
    return;
  digitalWrite(enables[drive], ENABLE);
}

inline void disable(int8_t drive)
{
 if(drive < 0 || drive >= DRIVES)
   return;
 digitalWrite(enables[drive], DISABLE);
}

/* *********************************************************************

   The main loop and command interpreter
*/


void command()
{
  if(!buf[0])
    return;
  
  debugMessage("Received: ", buf);
  
  uint8_t dh = buf[1]-'0';
  switch(buf[0])
  {
    case '\n':
      break;
      
    case GET_T: // Get temperature of an extruder
      talkToMaster(currentTemps[dh]);
      debugMessage("Sent temp: ", currentTemps[dh], " for extruder ", dh);
      break;
      
    case GET_TT: // Get target temperature of an extruder
      talkToMaster(setTemps[dh]);
      debugMessage("Sent target temp: ", setTemps[dh], " for extruder ", dh);
      break;      
    
    case SET_T: // Set temperature of an extruder
      setTemperature(dh, atof(&buf[2]));
      debugMessage("Set target temp to: ", setTemps[dh], " for extruder ", dh);
      break;
      
    case SET_B:
      eBeta[dh] = atof(&buf[2]);
      debugMessage("Set eBeta to: ", eBeta[dh]);
      break;
    
    case SET_R:
      eRs[dh] = atof(&buf[2]);
      debugMessage("Set eRs to: ", eRs[dh]);
      break;
    
    case SET_I:
      eRInf[dh] = atof(&buf[2]);
      debugMessage("Set eRInf to: ", eRInf[dh]);
      break;
      
    case GET_B:
      talkToMaster(eBeta[dh]);
      debugMessage("Sent eBeta: ", eBeta[dh]);
      break;
    
    case GET_R:
      talkToMaster(eRs[dh]);
      debugMessage("Sent eRs: ", eRs[dh]);
      break;
    
    case GET_I:
      talkToMaster(eRInf[dh]);
      debugMessage("Sent eRInf: ", eRInf[dh]);
      break;
      
    case DRIVE:  // Set the current drive
      if(dh == currentDrive)
        return;
      currentDrive = dh;
      enable(currentDrive);
      debugMessage("Drive set to: ", currentDrive);
      break;
      
    case DIR_F:  // Set an extruder's direction forwards
      setDirection(dh, FORWARDS);
      debugMessage("Forwards set for: ", dh);
      break;
    
    case DIR_B:   // Set an extruder's direction backwards
      setDirection(dh, BACKWARDS);
      debugMessage("Backwards set for: ", dh);
      break;
        
    case DEBUG:
      debug = dh;
      debugMessage("Debug set to: ", debug);
      break;
    
    case STOP: // Shut everything down; carry on listening for commands
      stopSlave();
      break;
      
    case MOTOR:
      enable(dh);
      debugMessage("Drive turned on: ", dh);
      break;    
      
    case NO_MOTOR:
      disable(dh);
      debugMessage("Drive turned off: ", dh);
      break;
      
    case SET_KP:
      Kp[dh] = atof(&buf[2]);
      debugMessage("Set Kp to: ", Kp[dh]);
      break;
      
    case GET_KP:
      talkToMaster(Kp[dh]);
      debugMessage("Sent Kp: ", Kp[dh]);
      break;
      
    case SET_KI:
      PidMax[dh] = PidMax[dh]*Ki[dh];
      Ki[dh] = atof(&buf[2]);
      debugMessage("Set Ki to: ", Ki[dh]);
      Ki[dh] = Ki[dh]*0.001*(float)TEMP_INTERVAL;
      PidMax[dh] = constrain(PidMax[dh]/Ki[dh], 0.0, 255.0);
      break;
      
    case GET_KI:
      talkToMaster((float)(Ki[dh]/(0.001*(float)TEMP_INTERVAL)));
      debugMessage("Sent Ki: ", Ki[dh]);
      break;
      
    case SET_KD:
      Kd[dh] = atof(&buf[2]);
      debugMessage("Set Kd to: ", Kd[dh]);
      Kd[dh] = Kd[dh]/(0.001*(float)TEMP_INTERVAL);
      break;
      
    case GET_KD:
      talkToMaster((float)(Kd[dh]*0.001*(float)TEMP_INTERVAL));
      debugMessage("Sent Kd: ", Kd[dh]);
      break;  
      
    case SET_KW:
      PidMax[dh] = atof(&buf[2]);
      debugMessage("Set w to: ", PidMax[dh]);
      PidMax[dh] = constrain(PidMax[dh], 0.0, 255.0)/Ki[dh];
      break;   
      
    case GET_KW:
      talkToMaster(PidMax[dh]*Ki[dh]);
      debugMessage("Sent W: ", PidMax[dh]*Ki[dh]);
      break;
      
    case H_TEST:
      heaterTest(dh);
      break;
      
    case NO_OP:
    case 0:
      break;
      
    default:
      error(false, strplus("dud command: ", buf));
      break;
  }
}




void loop() 
{
  incomming();
  if(errorStopped)
    zeroAll();
  tempCheck();   
} 



