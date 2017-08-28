
#include "Marlin.h"
#include "slave_comms.h"

#ifdef REPRAPPRO_MULTIMATERIALS

char slaveXmitBuffer[SLAVE_BUF];
char slaveRcvBuffer[SLAVE_BUF];
boolean setDir[EXTRUDERS];
boolean firstTalk;
boolean inSlaveMessage;
char slaveError;
boolean driveOn[EXTRUDERS];
unsigned long timeout;
unsigned long slaveDelay;
long precision[] = {0,10,100,1000,10000,100000,1000000,10000000,100000000};

void slaveErrorReport()
{
  if(!slaveError)
    return;
    
  SERIAL_PROTOCOL("Slave error(s); comms errors will also cause heater errors: ");
  if(slaveError & SLAVE_HEAT1)
    SERIAL_PROTOCOL("Dud heater 1, ");
  if(slaveError & SLAVE_HEAT2)
    SERIAL_PROTOCOL("Dud heater 2, ");
  if(slaveError & SLAVE_TIMEOUT)
    SERIAL_PROTOCOL("Comms timeout, ");
  if(slaveError & SLAVE_OVERFLOW)  
    SERIAL_PROTOCOL("Comms buffer overflow, ");
  SERIAL_PROTOCOLLN("");
}

void setup_slave()
{
	MYSERIAL1.begin(SLAVE_BAUD);
	SET_OUTPUT(SLAVE_CLOCK);
        slaveError = 0;
        firstTalk = true;
        inSlaveMessage = false;
        for(int i = 0; i < EXTRUDERS; i++)
        {
          setDir[i] = true;
          driveOn[i] = false;
        }
  //get slave extruder temps until readings are sensible
    SERIAL_PROTOCOLLNPGM("Slave init. Please wait ... ");
    for(uint8_t i=0;i<20;i++)
    {
        slaveError &= ~SLAVE_HEAT1;
        slaveError &= ~SLAVE_HEAT2;
        SERIAL_PROTOCOLPGM(".");
/*        if(slaveDegHotend(1) > -1)
        {
            if(slaveDegHotend(2) > -1)
            {
                SERIAL_PROTOCOLLNPGM("Slave ready");
                return;
            }
            slaveError |= SLAVE_HEAT2;
        }
        slaveError |= SLAVE_HEAT1;*/
        

        if(slaveDegHotend(1) <= -1)
            slaveError |= SLAVE_HEAT1;
            
        if(slaveDegHotend(2) <= -1)
          slaveError |= SLAVE_HEAT2;
        
        if(!((slaveError & SLAVE_HEAT1) || (slaveError & SLAVE_HEAT2)))
        {
          SERIAL_PROTOCOLLNPGM("Slave ready");
          return;
        }
        delay(1000);
    }
    slaveErrorReport();
    return;
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

#endif
