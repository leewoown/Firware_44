/* ==============================================================================
System Name:  �����ڵ��� ���� ������ 80V

File Name:		PARAMETER.H

Description:	����
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

#ifndef BAT_Algorithm_H
#define BAT_Algorithm_H

#include "F2806x_Cla_typedefs.h"// F2806x CLA Type definitions
#include "F2806x_Device.h"      // F2806x Headerfile Include File
#include "F2806x_Examples.h"    // F2806x Examples Include File
#include "DSP28x_Project.h"

#define Kokam100Ah            0
#define Kokam60Ah             0
#define FarasisP52Ah          1
#define Frey60Ah              1

#define C_SocInitCTVaule      1
#define C_SocCumulativeTime   0.00027778 //1/3600
#define C_CTSampleTime        0.05
#define C_SocSamPleCount      50

/*
 *  Frey60Ah
 */
#define LFP_VOLT_A_BOT   3.050
#define LFP_VOLT_A_TOP   3.211
#define LFP_VOLT_B_BOT   3.211
#define LFP_VOLT_B_TOP   3.294
#define LFP_VOLT_C_BOT   3.294
#define LFP_VOLT_C_TOP   3.317
#define LFP_VOLT_D_BOT   3.317
#define LFP_VOLT_D_TOP   3.337

typedef enum
{
  SOC_STATE_IDLE,
  SOC_STATE_RUNNING,
  SOC_STATE_Save,
  SOC_STATE_CLEAR
} SoCState;
struct SoCSate_BIT
{       // bits   description
   unsigned int     CalMeth         :1; // 0
   unsigned int     State01         :1; // 1
   unsigned int     State02         :1; // 2
   unsigned int     State03         :1; // 3
   unsigned int     State04         :1; // 4
   unsigned int     State05         :1; // 5
   unsigned int     State06         :1; // 6
   unsigned int     State07         :1; // 7
   unsigned int     State08         :1; // 8
   unsigned int     State09         :1; // 9
   unsigned int     State10         :1; // 10
   unsigned int     State11         :1; // 11
   unsigned int     State12         :1; // 12
   unsigned int     State13         :1; // 13
   unsigned int     State14         :1; // 14
   unsigned int     State15         :1; // 15
};
union SoCState_REG
{
   unsigned int     all;
   struct SoCSate_BIT bit;
};
typedef struct
{
  /*
   *
   */
  unsigned int CTCount;
  unsigned int SysTime;
  SoCState state;
  float32 SysSOCdtF;
  float32 SysSoCCTF;
  float32 SysSoCCTAbsF;
  float32 SysAhNewF;
  float32 SysAhOldF;
  float32 SysAhF;
  float32 SysSOCBufF1;
  float32 SysSOCBufF2;
  float32 SysSOCF;


  /*
   *
   */
  float32  AVGXF;
  float32  SOCX4InF;
  float32  SOCX3InF;
  float32  SOCX2InF;
  float32  SOCX1InF;

  float32  SOCX4OutF;
  float32  SOCX3OutF;
  float32  SOCX2OutF;
  float32  SOCX1OutF;
/*
 *
 */
  float32  SOCX4InFAZore;
  float32  SOCX3InFAZore;
  float32  SOCX2InFAZore;
  float32  SOCX1InFAZore;
  float32  SOCX4OutFAZore;
  float32  SOCX3OutFAZore;
  float32  SOCX2OutFAZore;
  float32  SOCX1OutFAZore;
  Uint16   AZoreCalCout;



  float32  SOCX4InFBZore;
  float32  SOCX3InFBZore;
  float32  SOCX2InFBZore;
  float32  SOCX1InFBZore;
  float32  SOCX4OutFBZore;
  float32  SOCX3OutFBZore;
  float32  SOCX2OutFBZore;
  float32  SOCX1OutFBZore;
  Uint16   BZoreCalCout;


  float32  SOCX4InFCZore;
  float32  SOCX3InFCZore;
  float32  SOCX2InFCZore;
  float32  SOCX1InFCZore;
  float32  SOCX4OutFCZore;
  float32  SOCX3OutFCZore;
  float32  SOCX2OutFCZore;
  float32  SOCX1OutFCZore;
  Uint16   CZoreCalCout;

  float32  SOCX4InFDZore;
  float32  SOCX3InFDZore;
  float32  SOCX2InFDZore;
  float32  SOCX1InFDZore;
  float32  SOCX4OutFDZore;
  float32  SOCX3OutFDZore;
  float32  SOCX2OutFDZore;
  float32  SOCX1OutFDZore;
  Uint16   DZoreCalCout;


  float32  SOCbufF;
  float32  SysSocInitF;
  float32  CellAgvVoltageF;




  union SoCState_REG SoCStateRegs;
} SocReg;





#endif  // end of PARAMETER.H definition


//===========================================================================
// No more.
//===========================================================================
