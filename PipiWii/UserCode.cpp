#include <avr/io.h>
#include "Arduino.h"
#include "config.h";
#include "def.h"
#include "types.h"
#include "UserCode.h";
#include "Serial.h";
#include "GPS.h";
#include "Multiwii.h";
#include "BHTuning.h";
#include "FrSkyTelemtry.h";


/**************************************************************************************/
/***************       Calculate first and last used servos        ********************/
/**************************************************************************************/
#if defined(SERVO)
  #if defined(PRI_SERVO_FROM) && defined(SEC_SERVO_FROM)
    #if PRI_SERVO_FROM < SEC_SERVO_FROM
      #define SERVO_START PRI_SERVO_FROM
    #else
      #define SERVO_START SEC_SERVO_FROM
    #endif
  #else
    #if defined(PRI_SERVO_FROM)
      #define SERVO_START PRI_SERVO_FROM
    #endif
    #if defined(SEC_SERVO_FROM)
      #define SERVO_START SEC_SERVO_FROM
    #endif
  #endif
  #if defined(PRI_SERVO_TO) && defined(SEC_SERVO_TO)
    #if PRI_SERVO_TO > SEC_SERVO_TO
      #define SERVO_END PRI_SERVO_TO
    #else
      #define SERVO_END SEC_SERVO_TO
    #endif
  #else
    #if defined(PRI_SERVO_TO)
      #define SERVO_END PRI_SERVO_TO
    #endif
    #if defined(SEC_SERVO_TO)
      #define SERVO_END SEC_SERVO_TO
    #endif
  #endif


#endif


// USER CODE HOOKS (BH)

void userSetup() {
  #if defined(TELEMETRY_FRSKY) // FRSKY
     init_telemetry();
  #endif
  #ifdef BH_TUNE
     bhTune_setup();
  #endif
  #ifdef SERIALOG
    char buf[32];
    unsigned char i;
    SerialOpen(SERIALOG_PORT,SERIALOG_BAUD);

    #ifdef SERIALOG_D_ASPD      // log airspeed
      SerialWriteString(SERIALOG_PORT,"ASPD;");
    #endif
    #ifdef SERIALOG_D_GSPD      // log gps (ground) speed
      SerialWriteString(SERIALOG_PORT,"GSPD;");
    #endif
    #ifdef SERIALOG_D_ALT
      SerialWriteString(SERIALOG_PORT,"ALT;");
    #endif
    #ifdef SERIALOG_D_ATT
      SerialWriteString(SERIALOG_PORT,"ATT_P;ATT_R;ATT_H;");
    #endif
    #ifdef SERIALOG_D_TKP_FI
      SerialWriteString(SERIALOG_PORT,"TKP_FI;");
    #endif
    #ifdef SERIALOG_D_TKP_FMOT
      SerialWriteString(SERIALOG_PORT,"FMOT;");
    #endif
    #ifdef SERIALOG_D_RC
      SerialWriteString(SERIALOG_PORT,"RC_P;RC_R;RC_T;RC_Y;");
    #endif
    #ifdef SERIALOG_D_MOT
      for (i = 0; i < NUMBER_MOTOR; i++) {
          sprintf(buf,"MOT_%d;",i);
          SerialWriteString(SERIALOG_PORT,buf);
      }
    #endif
    #ifdef SERIALOG_D_SER
        for(i=SERVO_START-1; i<SERVO_END; i++) {
          sprintf(buf,"SER_%d;",i);
          SerialWriteString(SERIALOG_PORT,buf);
        }
    #endif    
    SerialWriteString(SERIALOG_PORT,"\r\n");
  #endif
}

void userLoopFast() {

   #if defined(AIRSPEED)
      ReadAirSpeed();
      debug[0]= pressure_diff;
      debug[1]= air_pressure;    
   #endif

  
}

void userLoopSlow() {
  #if defined(TELEMETRY_FRSKY) //FRSKY
     telemetry_frsky(); //FRSKY
  #endif //FRSKY

  #ifdef BH_TUNE
      bhTune_loopSlow();
   #endif

  #ifdef SERIALOG
    static char buff[256];
    char* buf = buff;
    unsigned char i;

      *buf=0;
      #ifdef SERIALOG_D_ASPD      // log airspeed
        //itoa(airspeed,buf,10);
	sprintf(buf,"%d;",airspeed);
        while(*buf!=0) ++buf; 
        //*buf++ = ';';
	//SerialWriteString(SERIALOG_PORT,buf);
      #endif
      #ifdef SERIALOG_D_GSPD      // log gps (ground) speed
//        itoa(GPS_speed,buf,10);
	sprintf(buf,"%d;",GPS_speed);
        while(*buf!=0) ++buf; 
        //*buf++ = ';';

//	SerialWriteString(SERIALOG_PORT,buf);
      #endif
      #ifdef SERIALOG_D_ALT
//        itoa(alt.EstAlt,buf,10);
	sprintf(buf,"%d;",alt.EstAlt);
        while(*buf!=0) ++buf; 
//	SerialWriteString(SERIALOG_PORT,buf);
      #endif
      #ifdef SERIALOG_D_ATT
	sprintf(buf,"%d;%d;%d;",att.angle[PITCH],att.angle[ROLL],att.heading);
        while(*buf!=0) ++buf; 
//	SerialWriteString(SERIALOG_PORT,buf);
      #endif
      #ifdef SERIALOG_D_TKP_FI
	uint16_t tmpp = tkp_fiangle*1000;
//        itoa(tmpp,buf,10);
        sprintf(buf,"%d;",tmpp);
        while(*buf!=0) ++buf; 

//	SerialWriteString(SERIALOG_PORT,buf);
      #endif
      #ifdef SERIALOG_D_TKP_FMOT
//        itoa(tkp_fmotors,buf,10);
	sprintf(buf,"%d;",tkp_fmotors);
        while(*buf!=0) ++buf; 
//	SerialWriteString(SERIALOG_PORT,buf);
      #endif
      #ifdef SERIALOG_D_RC
	sprintf(buf,"%d;%d;%d;%d;",rcData[PITCH],rcData[ROLL],rcData[THROTTLE],rcData[YAW]);
        while(*buf!=0) ++buf; 
        //SerialWriteString(SERIALOG_PORT,buf);
      #endif
      #ifdef SERIALOG_D_MOT
        for (i = 0; i < NUMBER_MOTOR; i++) {
            sprintf(buf,"%d;",motor[i]);
            while(*buf!=0) ++buf; 
          //  SerialWriteString(SERIALOG_PORT,buf);
        }
      #endif
      #ifdef SERIALOG_D_SER
          for(i=SERVO_START-1; i<SERVO_END; i++) {
  	    sprintf(buf,"%d;",servo[i]);
            while(*buf!=0) ++buf; 
//            SerialWriteString(SERIALOG_PORT,buf);
          }
      #endif    
      sprintf(buf,"\r\n");
      SerialWriteString(SERIALOG_PORT,buff);
  #endif
}

void userLoop1Hz() {
   #ifdef BH_TUNE
      bhTune_save();
   #endif
}
