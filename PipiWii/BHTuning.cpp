/**
 * PID POTENTIOMETER TUNING (BH)
 * tune PID values using TX potentiometer
 *
   Maximum PID values (for reference when setting your range)
  -------------------------------------
  Param       P       I        D
   -------------------------------------
   ROLL        20.0    0.250    100
   PITCH       20.0    0.250    100
   YAW         20.0    0.250    100
   PIDALT      20.0    0.250    100
   PIDPOS       2.5    2.5
   PIDPOSR     25.0    2.50       0.250
   PIDNAVR     25.0    2.50       0.250
   PIDLEVEL    20.0    0.250    100
   PIDMAG      20.0
 */

#include "Arduino.h"
#include "config.h"
#include "def.h"
#include "types.h"
#include "MultiWii.h"
#include "EEPROM.h"


#ifdef BH_TUNE
   enum bhTune_params {P, I, D};

   // ****************************************************************
   // ***************** START OF CONFIGURABLE PARAMS *****************

   // How many channels to tune at once; 
   //  a channel is defined by PID loop to tune, PID parameter to tune, parameter value range, RC ch. of TX potentiometer input
   // If you change this you MUST change number of array elements for bhTune_iKey, bhTune_iParam, bhTune_iValRange, bhTune_iInputCh
   const uint8_t bhTune_iNumCh = 2;

   // PID loop to tune - ROLL, PITCH, YAW, PIDALT, PIDPOS, PIDPOSR, PIDNAVR, IDLEVEL or PIDMAG
   const uint8_t bhTune_iKey[bhTune_iNumCh] = {ROLL,ROLL};
  
   // PID parameter to tune - P, I or D
//   const uint8_t bhTune_iParam[bhTune_iNumCh] = {P,I,D};
   const uint8_t bhTune_iParam[bhTune_iNumCh] = {P,I};
   
   // parameter value range - see maximums above
   const float bhTune_iValRange[bhTune_iNumCh][2] = {{1.0, 15.0},{0.005, 0.05}};
//   const float bhTune_iValRange[bhTune_iNumCh][2] = {{0.5, 10},{0.005, 0.1}};
   
   // RC ch. of TX potentiometer input (e.g. AUX1)
//   const uint8_t bhTune_iInputCh[bhTune_iNumCh] = {AUX2,AUX3,AUX4};
   const uint8_t bhTune_iInputCh[bhTune_iNumCh] = {AUX2,AUX3};
   
   // lock pitch & roll values
   const uint8_t bhTune_bLockPitchRoll = 1;

   
   // ****************** END OF CONFIGURABLE PARAMS ******************
   // ****************************************************************
   
   // check if setting exists
   uint8_t bhTune_isSettingInvalid(uint8_t nChannel) {
    return (bhTune_iKey[nChannel] > PIDMAG) ||
            (bhTune_iParam[nChannel] > D) ||
            (bhTune_iKey[nChannel] == PIDPOS && bhTune_iParam[nChannel] == D) ||
            (bhTune_iKey[nChannel] == PIDMAG && bhTune_iParam[nChannel] != P)
            ? 1 : 0;
   }
   
   uint8_t bhTune_bSaved = 1;
   uint8_t bhTune_iParamRangeDelta[bhTune_iNumCh];
   uint8_t bhTune_iParamRangeMin[bhTune_iNumCh];

   // APPLY MULTIPLIER, LIMIT
   uint8_t bhTune_clampUserVal(float fParamVal, uint8_t param) {
      
      // max vals (multipliers applied)
      uint8_t aPMax[] = {200, 200, 200, 200, 250, 250, 250, 200, 200};
      uint8_t aIMax[] = {250, 250, 250, 250, 250, 250, 250, 250};
      uint8_t aDMax[] = {100, 100, 100, 100, 0, 250, 250, 100};
      
      // multipliers
      uint8_t aPMult[] = {10, 10, 10, 10, 100, 10, 10, 10, 10};
      uint16_t aIMult[] = {1000, 1000, 1000, 1000, 100, 100, 100, 1000};
      uint16_t aDMult[] = {1, 1, 1, 1, 0, 1000, 1000, 1};
      
      // multiply and limit
      switch (bhTune_iParam[param]) {
      case P:
         return min(max(0, fParamVal) * aPMult[bhTune_iKey[param]], aPMax[bhTune_iKey[param]]);
         break;
      case I:
         return min(max(0, fParamVal) * aIMult[bhTune_iKey[param]], aIMax[bhTune_iKey[param]]);
         break;
      case D:
         return min(max(0, fParamVal) * aDMult[bhTune_iKey[param]], aDMax[bhTune_iKey[param]]);
         break;
      }
   }
    
   // SETUP
   void bhTune_setup() {
     for(int i=0;i<bhTune_iNumCh;++i) {
        if (!bhTune_isSettingInvalid(i)) {            
            bhTune_iParamRangeMin[i] = bhTune_clampUserVal(bhTune_iValRange[i][0],i);
            bhTune_iParamRangeDelta[i] = bhTune_clampUserVal(bhTune_iValRange[i][1],i) - bhTune_iParamRangeMin[i];
        }
     }
   }

   // UPDATE PARAM
   void bhTune_setConfVal(uint8_t iParamVal, uint8_t iTuneKey,uint8_t iParam) {

       switch (iParam) {
        case P:
             conf.pid[iTuneKey].P8 = iParamVal;
           break;
        case I:
             conf.pid[iTuneKey].I8 = iParamVal;
           break;
        case D:
             conf.pid[iTuneKey].D8 = iParamVal;
           break;
        }
   }
   
   // LOOP
   void bhTune_loopSlow() {
         
     static uint8_t iParamVal[bhTune_iNumCh];
     static uint8_t iParamValOld[bhTune_iNumCh];
     
     // get param val (apply TX input to range)
     for(int i = 0; i< bhTune_iNumCh; ++i) {
      if (!bhTune_isSettingInvalid(i)) {
        
           iParamVal[i] = bhTune_iParamRangeDelta[i] * ((min(max(rcData[bhTune_iInputCh[i]], 1000), 2000) - 1000) * 0.001) + bhTune_iParamRangeMin[i];
           
           // check if it's changed
           if (iParamVal[i] != iParamValOld[i]) {
              
              // update state vars
              bhTune_bSaved = 0;
              iParamValOld[i] = iParamVal[i];
              
              // pitch & roll locked
              if ((bhTune_iKey[i] == PITCH || bhTune_iKey[i] == ROLL) && bhTune_bLockPitchRoll) {
                 bhTune_setConfVal(iParamVal[i], PITCH, bhTune_iParam[i]);
                 bhTune_setConfVal(iParamVal[i], ROLL, bhTune_iParam[i]);
              } else {
                 bhTune_setConfVal(iParamVal[i], bhTune_iKey[i], bhTune_iParam[i]);
              }
           }
        }
      }
   }
   
  
   // SAVE
   void bhTune_save() {
      if (!f.ARMED && !bhTune_bSaved) {
         bhTune_bSaved = 1;
         writeParams(0);
      }
   }
   
#endif

/*
MULTIPLIERS
              P       I       D
ROLL         10    1000       1
PITCH        10    1000       1
YAW          10    1000       1
PIDALT       10    1000       1
PIDPOS      100       100       0
PIDPOSR      10       100    1000
PIDNAVR      10       100    1000
PIDLEVEL     10    1000       1
PIDMAG       10
*/
