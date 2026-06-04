/* ==============================================================================
System Name:  현대자동차 수소 지게차 80V

File Name:		PARAMETER.H

Description:	현대
          	    Orientation Control for a Three Phase AC Induction Motor. 

Originator:		Digital control systems Group - Texas Instruments

Note: In this software, the default inverter is supposed to be DMC1500 board.
=====================================================================================
 History:
-------------------------------------------------------------------------------------
 04-15-2005	Version 3.20
=================================================================================  */

//#include "F2806x_Cla_typedefs.h"// F2806x CLA Type definitions
//#include "F2806x_Device.h"      // F2806x Headerfile Include File
//#include "F2806x_Examples.h"    // F2806x Examples Include File
//#include "DSP28x_Project.h"

#ifndef PROTECTRELAY_H
#define PROTECTRELAY_H

#include "F2806x_Cla_typedefs.h"// F2806x CLA Type definitions
#include "F2806x_Device.h"      // F2806x Headerfile Include File
#include "F2806x_Examples.h"    // F2806x Examples Include File
#include "DSP28x_Project.h"

/*
 *
 */
#define ProRelayOnTime          50  // 50msec
#define PRelayOnTime            50  // 50msec
#define ProRelayOFFTime         250 // 250msec
#define WakeUpOnTimeOut         350*4 // 250msec
/*
 *
 */
#define PRelayOFFTime           50  //
#define WakeUpOFFTimeOutCount   50*2
typedef enum
{
  STATE_IDLE,
  STATE_WakeUpReady,
  STATE_WakeUpON,
  STATE_WakeUpOFF,
  STATE_CLEAR
}ProtectRelayState;

struct ProtectRelayState_BIT
{       
    // bits   description
    // TOOOS : [오류] 260.06.04 수정  STATE09 추석 처리, 유니온 선언 에러
    unsigned int     WakeUpEN                    :1; // 00
    unsigned int     PRelayDI                    :1; // 01
    unsigned int     NRelayDI                    :1; // 02
    unsigned int     PreRelayDI                  :1; // 03
    unsigned int     PRelayDO                    :1; // 04
    unsigned int     NRelayDO                    :1; // 05
    unsigned int     PreRelayDO                  :1; // 06
    unsigned int     LatchRelayOn                :1; // 07
    unsigned int     LatchRelayOFF               :1; // 08
    unsigned int     ProtectRelayCyle            :1; // 09 // bit8 보호차단 1회성 가드(0:장전/미실행, 1:차단완료). INIT 및 PrtctReset 시 0으로 재장전
   // TODOS : [검증] 260.06.04 수정  STATE09 추석 처리 
   // unsigned int     STATE09                   :1; // 10
    unsigned int     STATE10                     :1; // 10
    unsigned int     STATE11                     :1; // 11
    unsigned int     STATE12                     :1; // 12
    unsigned int     WakeUpState                 :1; // 13
    unsigned int     WakeUpSEQERR                :1; // 14
    unsigned int     RlyFaulttSate               :1; // 15
};
union ProtectRelaySate_REG
{
   unsigned int     all;
   struct ProtectRelayState_BIT bit;
};

typedef struct PrtectRelay_Date
{
    /*
     *
     */
    ProtectRelayState StateMachine;
    Uint16 ProRleayOnTimerCount;
    Uint16 ProRleayOffTimerCount;
    Uint16 PRleayOnTimerCount;
    Uint16 WakeUpOnTimeOutCount;
    Uint16 WakeUpOffTimeOutCount;
    union  ProtectRelaySate_REG    State;


}PrtectRelayReg;
extern void ProtectRelaySateCheck(PrtectRelayReg *p);
extern void ProtectRelayVarINIT(PrtectRelayReg *p);
extern void ProtectOffHandle(PrtectRelayReg *p);
extern void ProtectRelayWakeUpHandle(PrtectRelayReg *p);
extern void ProtecLatchRelayHandle(PrtectRelayReg *p);
#endif  // end of PARAMETER.H definition


//===========================================================================
// No more.
//===========================================================================
