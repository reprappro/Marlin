/*
  temperature.c - temperature control
  Part of Marlin
  
 Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm
 
 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.
 
 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.
 
 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 This firmware is a mashup between Sprinter and grbl.
  (https://github.com/kliment/Sprinter)
  (https://github.com/simen/grbl/tree)
 
 It has preliminary support for Matthew Roberts advance algorithm 
    http://reprap.org/pipermail/reprap-dev/2011-May/003323.html

 */


#include "Marlin.h"
#include "ultralcd.h"
#include "temperature.h"

//===========================================================================
//=============================public variables============================
//===========================================================================
int target_raw[EXTRUDERS_T] = { 0 };
int target_raw_bed = 0;

int current_raw[EXTRUDERS_T] = { 0 };
int current_raw_bed = 0;

#ifdef HEATER_0_USES_THERMISTOR
float eBeta, eRs, eRInf;
#endif

#ifdef BED_USES_THERMISTOR
float bBeta, bRs, bRInf;
#endif

#ifdef PIDTEMP
  // used external
  float pid_setpoint[EXTRUDERS_T] = { 0.0 };
  
  float Kp=DEFAULT_Kp;
  float Ki=DEFAULT_Ki;
  int Ki_Max=PID_INTEGRAL_DRIVE_MAX;
  float Kd=DEFAULT_Kd;
  
#endif //PIDTEMP

void bed_temp_error();

char dudMaxCount, dudMinCount, dudBedCount;
#define DUD_TEMP_COUNT 3
  
  
//===========================================================================
//=============================private variables============================
//===========================================================================
static volatile bool temp_meas_ready = false;

static unsigned long  previous_millis_bed_heater;
//static unsigned long previous_millis_heater;

#ifdef PIDTEMP
  //static cannot be external:
  static float temp_iState[EXTRUDERS_T] = { 0 };
  static float temp_dState[EXTRUDERS_T] = { 0 };
  static float pTerm[EXTRUDERS_T];
  static float iTerm[EXTRUDERS_T];
  static float dTerm[EXTRUDERS_T];
  //int output;
  static float pid_error[EXTRUDERS_T];
  static float temp_iState_min[EXTRUDERS_T];
  static float temp_iState_max[EXTRUDERS_T];
  // static float pid_input[EXTRUDERS_T];
  // static float pid_output[EXTRUDERS_T];
  static bool pid_reset[EXTRUDERS_T];
#endif //PIDTEMP
  static unsigned char soft_pwm[EXTRUDERS_T];


// Init min and max temp with extreme values to prevent false errors during startup
//  static int minttemp[EXTRUDERS_T] = { 0 };
//  static int maxttemp[EXTRUDERS_T] = { 16383 }; // the first value used for all
  static int bed_minttemp = 0;
  static int bed_maxttemp = 16383;
  

//===========================================================================
//=============================   functions      ============================
//===========================================================================


void setExtruderThermistor(int8_t e, const float& b, const float& r, const float& i)
{
  #ifdef REPRAPPRO_MULTIMATERIALS
  if(e > 0)
  {
     setSlaveExtruderThermistor(e, b, r, i);
     return;
  }
  #endif
  eBeta = b;
  eRs = r;
  eRInf = i;
}

void setBedThermistor(const float& b, const float& r, const float& i)
{
  bBeta = b;
  bRs = r;
  bRInf = i;
}

float getExtruderBeta(int8_t e)
{
  if(e == 0)
    return eBeta;
  #ifdef REPRAPPRO_MULTIMATERIALS
  return getSlaveExtruderBeta(e);
  #endif
}

float getExtruderRs(int8_t e)
{
  if(e == 0)
    return eRs;
  #ifdef REPRAPPRO_MULTIMATERIALS
  return getSlaveExtruderRs(e);
  #endif
}

float getExtruderRInf(int8_t e)
{
  if(e == 0)
    return eRInf;
  #ifdef REPRAPPRO_MULTIMATERIALS
  return getSlaveExtruderRInf(e);
  #endif
}

float getBedBeta() { return bBeta; }
float getBedRs() { return bRs; }
float getBedRInf() { return bRInf; }

void getThermistor(int eb, float &beta, float &resistor, float &thermistor, float &inf)
{
  int h = eb;
  if(!h)
  {
   //get BED thermistor
   beta = getBedBeta();
   resistor = getBedRs();
   inf = getBedRInf();
  } else
  {
    // Extruder thermistor
    h--;
    beta = getExtruderBeta(h);
    resistor = getExtruderRs(h);
    inf = getExtruderRInf(h);         
  }
  thermistor = inf/(exp(-beta/298.15));
}


void setThermistor(int eb, const float &beta, const float &resistor, const float &thermistor, float &inf)
{
  inf = thermistor*exp(-beta/298.15);
  int h = eb;
  if(!h)
    setBedThermistor(beta, resistor, inf);
  else
  {
    h--;
    setExtruderThermistor(h, beta, resistor, inf);
  }
}


void PID_autotune(float temp)
{
  float input;
  int cycles=0;
  bool heating = true;

  unsigned long temp_millis = millis();
  unsigned long t1=temp_millis;
  unsigned long t2=temp_millis;
  long t_high;
  long t_low;

  long bias=PID_MAX/2;
  long d = PID_MAX/2;
  float Ku, Tu;
  float Kp, Ki, Kd;
  float max, min;
  
  SERIAL_ECHOLN("PID Autotune start");
  
  disable_heater(); // switch off all heaters.
  
  soft_pwm[0] = PID_MAX/2;
    
  for(;;) {

    if(temp_meas_ready == true) { // temp sample ready
      CRITICAL_SECTION_START;
      temp_meas_ready = false;
      CRITICAL_SECTION_END;
      input = analog2temp(current_raw[0], 0);
      
      max=max(max,input);
      min=min(min,input);
      if(heating == true && input > temp) {
        if(millis() - t2 > 5000) { 
          heating=false;
          soft_pwm[0] = (bias - d) >> 1;
          t1=millis();
          t_high=t1 - t2;
          max=temp;
        }
      }
      if(heating == false && input < temp) {
        if(millis() - t1 > 5000) {
          heating=true;
          t2=millis();
          t_low=t2 - t1;
          if(cycles > 0) {
            bias += (d*(t_high - t_low))/(t_low + t_high);
            bias = constrain(bias, 20 ,PID_MAX-FULL_PID_BAND);
            if(bias > PID_MAX/2) d = PID_MAX - 1 - bias;
            else d = bias;

            SERIAL_PROTOCOLPGM(" bias: "); SERIAL_PROTOCOL(bias);
            SERIAL_PROTOCOLPGM(" d: "); SERIAL_PROTOCOL(d);
            SERIAL_PROTOCOLPGM(" min: "); SERIAL_PROTOCOL(min);
            SERIAL_PROTOCOLPGM(" max: "); SERIAL_PROTOCOLLN(max);
            if(cycles > 2) {
              Ku = (4.0*d)/(3.14159*(max-min)/2.0);
              Tu = ((float)(t_low + t_high)/1000.0);
              SERIAL_PROTOCOLPGM(" Ku: "); SERIAL_PROTOCOL(Ku);
              SERIAL_PROTOCOLPGM(" Tu: "); SERIAL_PROTOCOLLN(Tu);
              Kp = 0.6*Ku;
              Ki = 2*Kp/Tu;
              Kd = Kp*Tu/8;
              SERIAL_PROTOCOLLNPGM(" Clasic PID ")
              SERIAL_PROTOCOLPGM(" Kp: "); SERIAL_PROTOCOLLN(Kp);
              SERIAL_PROTOCOLPGM(" Ki: "); SERIAL_PROTOCOLLN(Ki);
              SERIAL_PROTOCOLPGM(" Kd: "); SERIAL_PROTOCOLLN(Kd);
            }
          }
          soft_pwm[0] = (bias + d) >> 1;
          cycles++;
          min=temp;
        }
      } 
    }
    if(input > (temp + 20)) {
      SERIAL_PROTOCOLLNPGM("PID Autotune failed! Temperature to high");
      return;
    }
    if(millis() - temp_millis > 2000) {
      temp_millis = millis();
      SERIAL_PROTOCOLPGM("ok T:");
      SERIAL_PROTOCOL(degHotend(0));   
      SERIAL_PROTOCOLPGM(" @:");
      SERIAL_PROTOCOLLN(getHeaterPower(0));       
    }
    if(((millis() - t1) + (millis() - t2)) > (10L*60L*1000L*2L)) {
      SERIAL_PROTOCOLLNPGM("PID Autotune failed! timeout");
      return;
    }
    if(cycles > 5) {
      SERIAL_PROTOCOLLNPGM("PID Autotune finished ! Place the Kp, Ki and Kd constants in the configuration.h");
      return;
    }
    lcd_status();
  }
}

void updatePID()
{
#ifdef PIDTEMP
  for(int e = 0; e < EXTRUDERS_T; e++) { 
     temp_iState_max[e] = Ki_Max / Ki;  
  }
#endif
}

#ifdef PIDTEMP
void getPIDValues(int eb, float &Kpi, float &Kii, float &Kdi, float &Kmi)
{
  //TODO - also allow Bed PID updating
  eb--;
#ifdef REPRAPPRO_MULTIMATERIALS
  if(eb)
  {
    getSlavePIDValues(eb, Kpi, Kii, Kdi, Kmi);
    return;
  }
#endif
  Kpi = Kp;
  Kii = Ki/PID_dT;
  Kdi = Kd*PID_dT;
  Kmi = Ki_Max;
}

void setPIDValues(int eb, const float &Kpi, const float &Kii, const float &Kdi, const float &Kmi)
{
  //TODO - also allow Bed PID updating
  eb--;
#ifdef REPRAPPRO_MULTIMATERIALS
  if(eb)
  {
    setSlavePIDValues(eb, Kpi, Kii, Kdi, Kmi);
    return;
  } else
#endif
  {
    Kp = Kpi;
    Ki = Kii;
    Kd = Kdi;
    Ki_Max = constrain(Kmi, 0, 255);  
    
    SERIAL_PROTOCOL("ok");
    SERIAL_PROTOCOL(" p:");
    SERIAL_PROTOCOL(Kp);
    SERIAL_PROTOCOL(" i:");
    SERIAL_PROTOCOL(Ki);
    Ki = Ki*PID_dT;
    SERIAL_PROTOCOL(" d:");
    SERIAL_PROTOCOL(Kd);
    Kd = Kdi/PID_dT;
    SERIAL_PROTOCOL(" w:");
    SERIAL_PROTOCOL(Ki_Max);
    SERIAL_PROTOCOLLN("");
    updatePID();
  }
}

#endif
  
int getHeaterPower(int heater) {
  return soft_pwm[heater];
}


void manage_heater()
{

  float pid_input;
  float pid_output;

  if(temp_meas_ready != true)   //better readability
    return;

  CRITICAL_SECTION_START;
  temp_meas_ready = false;
  CRITICAL_SECTION_END;

  for(int e = 0; e < EXTRUDERS_T; e++) 
  {

  #ifdef PIDTEMP
    pid_input = analog2temp(current_raw[e], e);


        pid_error[e] = pid_setpoint[e] - pid_input;
        if(pid_error[e] > FULL_PID_BAND) {
          pid_output = PID_MAX;
          pid_reset[e] = true;
        }
        else if(pid_error[e] < -FULL_PID_BAND) {
          pid_output = 0;
          pid_reset[e] = true;
        }
        else {
          if(pid_reset[e] == true) {
            temp_iState[e] = 0.0;
            pid_reset[e] = false;
          }
          pTerm[e] = Kp * pid_error[e];
          temp_iState[e] += pid_error[e];
          temp_iState[e] = constrain(temp_iState[e], temp_iState_min[e], temp_iState_max[e]);
          iTerm[e] = Ki * temp_iState[e];
          //K1 defined in Configuration.h in the PID settings
          #define K2 (1.0-K1)
          dTerm[e] = (Kd * (pid_input - temp_dState[e]))*K2 + (K1 * dTerm[e]);
          temp_dState[e] = pid_input;
          pid_output = constrain(pTerm[e] + iTerm[e] - dTerm[e], 0, PID_MAX);
        }

    #ifdef PID_DEBUG
    SERIAL_ECHOLN(" PIDDEBUG "<<e<<": Input "<<pid_input<<" Output "<<pid_output" pTerm "<<pTerm[e]<<" iTerm "<<iTerm[e]<<" dTerm "<<dTerm[e]);  
    #endif //PID_DEBUG
  #else /* PID off */
    pid_output = 0;
    if(current_raw[e] < target_raw[e]) {
      pid_output = PID_MAX;
    }
  #endif

    // Check if temperature is within the correct range
    if((current_raw[e] > minttemp[e]) && (current_raw[e] < maxttemp[e])) 
    {
      soft_pwm[e] = (int)pid_output >> 1;
    }
    else {
      soft_pwm[e] = 0;
    }
  } // End extruder for loop
  
  
  if(millis() - previous_millis_bed_heater < BED_CHECK_INTERVAL)
    return;
  previous_millis_bed_heater = millis();

  //check master temps for errors here because only done every 5secs
  // Slave does its own checking
#ifdef REPRAPPRO_MULTIMATERIALS

    if(dudMaxCount >= 0)
    {
      if(degHotend(0) >= HEATER_MAXTEMP)
      {
        dudMaxCount++;
        if(dudMaxCount >= DUD_TEMP_COUNT)
        {
          dudMaxCount = -1;
          setTargetHotend(0,0);
          max_temp_error(0);
          //#ifndef BOGUS_TEMPERATURE_FAILSAFE_OVERRIDE
          //Stop();
          //#endif
        }
      } else
        dudMaxCount = 0;
    } else
      setTargetHotend(0,0);

    if(dudMinCount >= 0)
    {  
      if(degHotend(0) <= HEATER_MINTEMP)
      {
        dudMinCount++;
        if(dudMinCount >= DUD_TEMP_COUNT)
        {
          dudMinCount = -1;
          setTargetHotend(0,0);
          min_temp_error(0);
          //#ifndef BOGUS_TEMPERATURE_FAILSAFE_OVERRIDE
          //{
          //  Stop();
          //}
          //#endif
        }
      } else
        dudMinCount = 0;
    } else
      setTargetHotend(0,0);

#endif
  
  #if TEMP_BED_PIN > -1

      // Check if temperature is within the correct range
      if(dudBedCount >= 0)
      {
        if((current_raw_bed > bed_minttemp) && (current_raw_bed < bed_maxttemp)) 
        {
          dudBedCount = 0;
          if(current_raw_bed >= target_raw_bed)
          {
            WRITE(HEATER_BED_PIN,LOW);
          }
          else 
          {
            WRITE(HEATER_BED_PIN,HIGH);
          }
        } else
        {
          dudBedCount++;
          if(dudBedCount >= DUD_TEMP_COUNT)
          {
            dudBedCount = -1;
            bed_temp_error();
            WRITE(HEATER_BED_PIN,LOW);
          }
        }
      } else 
      {
        WRITE(HEATER_BED_PIN,LOW);
      }
  #endif
}

// Use algebra to work out temperatures, not tables
// NB - this assumes all extruders use the same thermistor type.
int temp2analogi(int celsius, const float& beta, const float& rs, const float& r_inf)
{
   float r = r_inf*exp(beta/(celsius - ABS_ZERO));
   return AD_RANGE - (int)(0.5 + AD_RANGE*r/(r + rs));
}

float analog2tempi(int raw, const float& beta, const float& rs, const float& r_inf)
{
   float rawf = (float)(AD_RANGE - raw);
   return ABS_ZERO + beta/log( (rawf*rs/(AD_RANGE - rawf))/r_inf );
}


#ifdef REPRAPPRO_MULTIMATERIALS


float analog2temp_remote(uint8_t e)
{
	return slaveDegHotend(e);
}

int temp2analog_remote(int celsius, uint8_t e)
{
	// What do we do about this, then?
	return temp2analogi(celsius, eBeta, eRs, eRInf);
}
#endif


int temp2analog(int celsius, uint8_t e) 
{
#ifdef REPRAPPRO_MULTIMATERIALS
	if(e > 0) return temp2analog_remote(celsius, e);
#endif
	return temp2analogi(celsius, eBeta, eRs, eRInf); 
}
float analog2temp(int raw, uint8_t e) 
{
#ifdef REPRAPPRO_MULTIMATERIALS
	if(e > 0) return analog2temp_remote(e);
#endif
	return analog2tempi(raw, eBeta, eRs, eRInf); 
}

int temp2analogBed(int celsius) 
{
	return temp2analogi(celsius, bBeta, bRs, bRInf); 
}
float analog2tempBed(int raw) 
{ 
	return analog2tempi(raw, bBeta, bRs, bRInf); 
}



void tp_init()
{
  dudMaxCount = 0;
  dudMinCount = 0;
  dudBedCount = 0;
  
  // Finish init of mult extruder arrays 
  for(int e = 0; e < EXTRUDERS_T; e++) {
    // populate with the first value 
    maxttemp[e] = maxttemp[0];
#ifdef PIDTEMP
    temp_iState_min[e] = 0.0;
    temp_iState_max[e] = Ki_Max / Ki;
#endif //PIDTEMP
  }

  #if (HEATER_0_PIN > -1) 
    SET_OUTPUT(HEATER_0_PIN);
  #endif  
  #if (HEATER_1_PIN > -1) 
    SET_OUTPUT(HEATER_1_PIN);
  #endif  
  #if (HEATER_2_PIN > -1) 
    SET_OUTPUT(HEATER_2_PIN);
  #endif  
  #if (HEATER_BED_PIN > -1) 
    SET_OUTPUT(HEATER_BED_PIN);
  #endif  
  #if (FAN_PIN > -1) 
    SET_OUTPUT(FAN_PIN);
  #endif  


  // Set analog inputs
  ADCSRA = 1<<ADEN | 1<<ADSC | 1<<ADIF | 0x07;
  DIDR0 = 0;
  #ifdef DIDR2
    DIDR2 = 0;
  #endif
  #if (TEMP_0_PIN > -1)
    #if TEMP_0_PIN < 8
       DIDR0 |= 1 << TEMP_0_PIN; 
    #else
       DIDR2 |= 1<<(TEMP_0_PIN - 8); 
    #endif
  #endif
  #if (TEMP_1_PIN > -1)
    #if TEMP_1_PIN < 8
       DIDR0 |= 1<<TEMP_1_PIN; 
    #else
       DIDR2 |= 1<<(TEMP_1_PIN - 8); 
    #endif
  #endif
  #if (TEMP_2_PIN > -1)
    #if TEMP_2_PIN < 8
       DIDR0 |= 1 << TEMP_2_PIN; 
    #else
       DIDR2 = 1<<(TEMP_2_PIN - 8); 
    #endif
  #endif
  #if (TEMP_BED_PIN > -1)
    #if TEMP_BED_PIN < 8
       DIDR0 |= 1<<TEMP_BED_PIN; 
    #else
       DIDR2 |= 1<<(TEMP_BED_PIN - 8); 
    #endif
  #endif
  
  // Use timer0 for temperature measurement
  // Interleave temperature interrupt with millies interrupt
  OCR0B = 128;
  TIMSK0 |= (1<<OCIE0B);  
  
  // Wait for temperature measurement to settle
  delay(250);

#ifdef HEATER_0_MINTEMP
  minttemp[0] = temp2analog(HEATER_0_MINTEMP, 0);
#endif //MINTEMP
#ifdef HEATER_0_MAXTEMP
  maxttemp[0] = temp2analog(HEATER_0_MAXTEMP, 0);
#endif //MAXTEMP

#ifdef REPRAPPRO_MULTIMATERIALS
  // Nothing to do here - remote handles it
#else
#if (EXTRUDERS_T > 1) && defined(HEATER_1_MINTEMP)
  minttemp[1] = temp2analog(HEATER_1_MINTEMP, 1);
#endif // MINTEMP 1
#if (EXTRUDERS_T > 1) && defined(HEATER_1_MAXTEMP)
  maxttemp[1] = temp2analog(HEATER_1_MAXTEMP, 1);
#endif //MAXTEMP 1

#if (EXTRUDERS_T > 2) && defined(HEATER_2_MINTEMP)
  minttemp[2] = temp2analog(HEATER_2_MINTEMP, 2);
#endif //MINTEMP 2
#if (EXTRUDERS_T > 2) && defined(HEATER_2_MAXTEMP)
  maxttemp[2] = temp2analog(HEATER_2_MAXTEMP, 2);
#endif //MAXTEMP 2
#endif

#ifdef BED_MINTEMP
  bed_minttemp = temp2analogBed(BED_MINTEMP);
#endif //BED_MINTEMP
#ifdef BED_MAXTEMP
  bed_maxttemp = temp2analogBed(BED_MAXTEMP);
#endif //BED_MAXTEMP
}



void disable_heater()
{
  for(int i=0;i<EXTRUDERS_T;i++)
    setTargetHotend(0,i);
  setTargetBed(0);
  #if TEMP_0_PIN > -1
  target_raw[0]=0;
  soft_pwm[0]=0;
   #if HEATER_0_PIN > -1  
     WRITE(HEATER_0_PIN,LOW);
   #endif
  #endif
     
  #if TEMP_1_PIN > -1
    target_raw[1]=0;
    soft_pwm[1]=0;
    #if HEATER_1_PIN > -1 
      WRITE(HEATER_1_PIN,LOW);
    #endif
  #endif
      
  #if TEMP_2_PIN > -1
    target_raw[2]=0;
    soft_pwm[2]=0;
    #if HEATER_2_PIN > -1  
      WRITE(HEATER_2_PIN,LOW);
    #endif
  #endif 

  #if TEMP_BED_PIN > -1
    target_raw_bed=0;
    #if HEATER_BED_PIN > -1  
      WRITE(HEATER_BED_PIN,LOW);
    #endif
  #endif 
}

void max_temp_error(uint8_t e) {
  //disable_heater();
  if(IsStopped() == false) {
    SERIAL_ERROR_START;
    SERIAL_ERRORLN((int)e);
    SERIAL_ERRORLNPGM(": Extruder switched off. MAXTEMP triggered !");
  }
}

void bed_temp_error() {
  //disable_heater();
  if(IsStopped() == false) {
    SERIAL_ERROR_START;
    //SERIAL_ERRORLN((int)e);
    SERIAL_ERRORLNPGM(": Bed switched off. Temp error triggered !");
  }
}

void min_temp_error(uint8_t e) {
  //disable_heater();
  if(IsStopped() == false) {
    SERIAL_ERROR_START;
    SERIAL_ERRORLN((int)e);
    SERIAL_ERRORLNPGM(": Extruder switched off. MINTEMP triggered !");
  }
}

void bed_max_temp_error(void) {
#if HEATER_BED_PIN > -1
  WRITE(HEATER_BED_PIN, 0);
#endif
  if(IsStopped() == false) {
    SERIAL_ERROR_START;
    SERIAL_ERRORLNPGM("Temperature heated bed switched off. MAXTEMP triggered !!");
  }
}


// Timer 0 is shared with millies
ISR(TIMER0_COMPB_vect)
{
  //these variables are only accesible from the ISR, but static, so they don't loose their value
  static unsigned char temp_count = 0;
  static unsigned long raw_temp_0_value = 0;
  static unsigned long raw_temp_1_value = 0;
  static unsigned long raw_temp_2_value = 0;
  static unsigned long raw_temp_bed_value = 0;
  static unsigned char temp_state = 0;
  static unsigned char pwm_count = 1;
  static unsigned char soft_pwm_0;
  static unsigned char soft_pwm_1;
  static unsigned char soft_pwm_2;
  
  boolean heatOn = ( (dudMinCount >= 0) && (dudMaxCount >= 0) );
  
  if(pwm_count == 0){
    soft_pwm_0 = soft_pwm[0];
    if(soft_pwm_0 > 0) WRITE(HEATER_0_PIN,heatOn);
    #ifdef REPRAPPRO_MULTIMATERIALS
        // Nothing to do here - remote handles it
    #else
    #if EXTRUDERS_T > 1
    soft_pwm_1 = soft_pwm[1];
    if(soft_pwm_1 > 0) WRITE(HEATER_1_PIN,heatOn);
    #endif
    #if EXTRUDERS_T > 2
    soft_pwm_2 = soft_pwm[2];
    if(soft_pwm_2 > 0) WRITE(HEATER_2_PIN,heatOn);
    #endif
    #endif
  }
  if(soft_pwm_0 <= pwm_count) WRITE(HEATER_0_PIN,0);
  #ifdef REPRAPPRO_MULTIMATERIALS
    // Nothing to do here - remote handles it
  #else  
  #if EXTRUDERS_T > 1
  if(soft_pwm_1 <= pwm_count) WRITE(HEATER_1_PIN,0);
  #endif
  #if EXTRUDERS_T > 2
  if(soft_pwm_2 <= pwm_count) WRITE(HEATER_2_PIN,0);
  #endif
  #endif
  pwm_count++;
  pwm_count &= 0x7f;
  
  switch(temp_state) {
    case 0: // Prepare TEMP_0
      #if (TEMP_0_PIN > -1)
        #if TEMP_0_PIN > 7
          ADCSRB = 1<<MUX5;
        #else
          ADCSRB = 0;
        #endif
        ADMUX = ((1 << REFS0) | (TEMP_0_PIN & 0x07));
        ADCSRA |= 1<<ADSC; // Start conversion
      #endif
      #ifdef ULTIPANEL
        buttons_check();
      #endif
      temp_state = 1;
      break;
    case 1: // Measure TEMP_0
      #if (TEMP_0_PIN > -1)
        raw_temp_0_value += ADC;
      #endif
      
      temp_state = 2;
      break;
    case 2: // Prepare TEMP_BED
      #if (TEMP_BED_PIN > -1)
        #if TEMP_BED_PIN > 7
          ADCSRB = 1<<MUX5;
        #endif
        ADMUX = ((1 << REFS0) | (TEMP_BED_PIN & 0x07));
        ADCSRA |= 1<<ADSC; // Start conversion
      #endif
      #ifdef ULTIPANEL
        buttons_check();
      #endif
      temp_state = 3;
      break;
    case 3: // Measure TEMP_BED
      #if (TEMP_BED_PIN > -1)
        raw_temp_bed_value += ADC;
      #endif
      temp_state = 4;
      break;
    case 4: // Prepare TEMP_1
      #if (TEMP_1_PIN > -1)
        #if TEMP_1_PIN > 7
          ADCSRB = 1<<MUX5;
        #else
          ADCSRB = 0;
        #endif
        ADMUX = ((1 << REFS0) | (TEMP_1_PIN & 0x07));
        ADCSRA |= 1<<ADSC; // Start conversion
      #endif
      #ifdef ULTIPANEL
        buttons_check();
      #endif
      temp_state = 5;
      break;
    case 5: // Measure TEMP_1
      #if (TEMP_1_PIN > -1)
        raw_temp_1_value += ADC;
      #endif
      temp_state = 6;
      break;
    case 6: // Prepare TEMP_2
      #if (TEMP_2_PIN > -1)
        #if TEMP_2_PIN > 7
          ADCSRB = 1<<MUX5;
        #else
          ADCSRB = 0;
        #endif
        ADMUX = ((1 << REFS0) | (TEMP_2_PIN & 0x07));
        ADCSRA |= 1<<ADSC; // Start conversion
      #endif
      #ifdef ULTIPANEL
        buttons_check();
      #endif
      temp_state = 7;
      break;
    case 7: // Measure TEMP_2
      #if (TEMP_2_PIN > -1)
        raw_temp_2_value += ADC;
      #endif
      temp_state = 0;
      temp_count++;
      break;
    default:
//      SERIAL_ERROR_START;
//      SERIAL_ERRORLNPGM("Temp measurement error!");
     break;
  }
    
  if(temp_count >= 16) // 8 ms * 16 = 128ms.
  {
    #if defined(HEATER_0_USES_AD595) || defined(HEATER_0_USES_MAX6675)
      current_raw[0] = raw_temp_0_value;
    #else
      current_raw[0] = 16383 - raw_temp_0_value;
    #endif
#ifdef REPRAPPRO_MULTIMATERIALS
  // Nothing to do here - remote handles it
#else
#if EXTRUDERS_T > 1
    #ifdef HEATER_1_USES_AD595
      current_raw[1] = raw_temp_1_value;
    #else
      current_raw[1] = 16383 - raw_temp_1_value;
    #endif
#endif
    
#if EXTRUDERS_T > 2
    #ifdef HEATER_2_USES_AD595
      current_raw[2] = raw_temp_2_value;
    #else
      current_raw[2] = 16383 - raw_temp_2_value;
    #endif
#endif
#endif

    current_raw_bed = 16383 - raw_temp_bed_value;

    
    temp_meas_ready = true;
    temp_count = 0;
    raw_temp_0_value = 0;
    raw_temp_1_value = 0;
    raw_temp_2_value = 0;
    raw_temp_bed_value = 0;
    
    //check_all_temps(); // This checks for above max and below min

    for(unsigned char e = 0; e < EXTRUDERS_T; e++)
    {
#ifdef REPRAPPRO_MULTIMATERIALS
        if(e > 0)
        {
            //Do nothing
        }else{
#endif
  if(dudMaxCount >= 0)
  {
       if(current_raw[e] >= maxttemp[e]) 
       {
         dudMaxCount++;
         if(dudMaxCount > DUD_TEMP_COUNT)
         {
          dudMaxCount = -1;
          target_raw[e] = 0;
          max_temp_error(e);
          //#ifndef BOGUS_TEMPERATURE_FAILSAFE_OVERRIDE
          //{
          //  Stop();
          //}
          //#endif
         }
       } else
         dudMaxCount = 0;
  } else
    target_raw[e] = 0;
       
  if(dudMinCount >= 0)
  {
       if(current_raw[e] <= minttemp[e]) 
       {
         dudMinCount++;
         if(dudMinCount > DUD_TEMP_COUNT)
         {
          target_raw[e] = 0;
          min_temp_error(e);
          dudMinCount = -1;
         // #ifndef BOGUS_TEMPERATURE_FAILSAFE_OVERRIDE
         // {
         //  Stop();
         //}
         //#endif
         }
       } else
         dudMinCount = 0;
  } else
    target_raw[e] = 0;
    
#ifdef REPRAPPRO_MULTIMATERIALS
    }
#endif
    }


#if defined(BED_MAXTEMP) && (HEATER_BED_PIN > -1)
  if(dudBedCount >= 0)
  {
    if(current_raw_bed >= bed_maxttemp) 
    {
      dudBedCount++;
      if(dudBedCount >= DUD_TEMP_COUNT)
      {
       dudBedCount = -1;
       target_raw_bed = 0;
       bed_temp_error();
       //Stop();
      }
    } else
      dudBedCount = 0;
  } else
    target_raw_bed = 0;
#endif
  }
}

