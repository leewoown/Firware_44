

/**
 * main.c
 */

#include "DSP28x_Project.h"
#include "parameter.h"
#include "SysVariable.h"
#include "ProtectRelay.h"
#include "BAT_LTC6802.h"
#include "BATAlgorithm.h"
#include <stdio.h>
#include <math.h>
#include <string.h>

/*
 *
 */
void InitGpio(void);

void MemCopy(Uint16 *SourceAddr, Uint16* SourceEndAddr, Uint16* DestAddr);

/*
 *
 */
void InitECanaGpio(void);
void InitECana(void);
void CANATX(unsigned int ID, unsigned char Length, unsigned int Data0, unsigned int Data1,unsigned int Data2,unsigned int Data3);

/*
 *
 */
void SysTimerINIT(SystemReg *s);
void SysVarINIT(SystemReg *s);
void CANRegVarINIT(CANAReg *P);
void DigitalInput(SystemReg *sys);
void DigitalOutput(SystemReg *sys);
/*
 *
 */
void ProtectRelaySateCheck(PrtectRelayReg *p);
void ProtectRelayVarINIT(PrtectRelayReg *p);
void ProtectOffHandle(PrtectRelayReg *p);
void ProtectRelayWakeUpHandle(PrtectRelayReg *p);
void ProtectRelayHandle(PrtectRelayReg *p);
/*
 *
 */
void Cal80VSysVoltageHandle(SystemReg *s);
void Cal80VSysTemperatureHandle(SystemReg *s);
void Cal80VSysCurrentHandle(SystemReg *s);
void CalFarasis52AhRegsInit(SocReg *P);
void CalFarasis52AhSocInit(SocReg *P);
void CalFarasis52AhSocHandle(SocReg *P);
void Cal80VSysFaultCheck(SystemReg *s);
void Cal80VSysAlarmtCheck(SystemReg *s);
/*
 *
 */
void Cal12VSysVoltageHandle(SystemReg *s);
void Cal12VSysTemperatureHandle(SystemReg *s);
void Cal12VSysCurrentHandle(SystemReg *s);
void CalFrey60AhRegsInit(SocReg *P);
void CalFrey60AhSocInit(SocReg *P);
void CalFrey60AhSocHandle(SocReg *P);
void Cal12VSysAlarmtCheck(SystemReg *s);
void Cal12VSysFaultCheck(SystemReg *s);


/*
 *
 */
int float32ToInt(float32 Vaule, Uint32 Num);
/*
 *
 */

/*
 *
 */
void SPI_Write(unsigned int WRData);
unsigned int SPI_Read(void);
void BAT_InitSPI(void);
void SPI_BATWrite(unsigned int WRData);
/*
 *
 */
int LTC6804_read_cmd(char address, short command, char data[], int len);
int LTC6804_write_cmd(char address, short command, char data[], int len);
int LTC6804_DieTemperatureRead(int pack_id, float *temperature);
void init_PEC15_Table(void);
unsigned short pec15(char *data, int len);
int SlaveBMSIint(SlaveReg *s);
int SlaveBmsBalance(SlaveReg *s);
void SlaveVoltagHandler(SlaveReg *s);
void SlaveVoltagBalaHandler(SlaveReg *s);
int  SlaveBMSDigiteldoutOHandler(SlaveReg *P);
void SalveTempsVoltHandler(SlaveReg *s);
void SalveTempsVoltHandler_B(SlaveReg *s);
void TempTemps(SystemReg *s);
/*
 *  인터럽트 함수 선언
 */
interrupt void cpu_timer0_isr(void);
interrupt void ISR_CANRXINTA(void);
//interrupt void cpu_timer2_isr(void);

SystemReg       SysRegs;
float32 randomCT=0;
float32 randomA=0;
float32 randomC=0;
PrtectRelayReg  PrtectRelayRegs;
SlaveReg        Slave1Regs;
SlaveReg        Slave2Regs;
SlaveReg        Slave3Regs;
CANAReg         CANARegs;
SocReg          Farasis52AhSocRegs;
SocReg          Frey60AhSocRegs;
float32         NCMsocTestVoltAGV =3.210;
float32         NUMsocTestVCT =0.0;
float32         LFPsocTestVoltAGV =3.050;
float32         LFPsocTestVCT =0.0;
float32         Slave1Temps=0;
float32         Slave2Temps=0;
float32         Slave3Temps=0;
unsigned int    ProtectRelayCyle=0;
unsigned int LFPINITFLAG=1;
//extern unsigned int    CellVoltUnBalaneFaulCount=0;
void main(void)
{
//    struct ECAN_REGS ECanaShadow;
    InitSysCtrl();
    /*
     * To check the clock status of the C2000 in operation
     */
  //  GpioCtrlRegs.GPAMUX2.bit.GPIO18 = 3; //enable XCLOCKOUT through GPIO mux
  //  SysCtrlRegs.XCLK.bit.XCLKOUTDIV = 0; //XCLOCKOUT = 1/2* SYSCLK

// Step 2. Initalize GPIO:
// This example function is found in the DSP2803x_Gpio.c file and
// illustrates how to set the GPIO to it's default state.
// For this example use the following configuration:
// Step 3. Clear all interrupts and initialize PIE vector table:
    DINT;
// Initialize PIE control registers to their default state.
// The default state is all PIE interrupts disabled and flags
// are cleared.
// This function is found in the DSP2803x_PieCtrl.c file.
    InitPieCtrl();
// Disable CPU interrupts and clear all CPU interrupt flags:
    IER = 0x0000;
    IFR = 0x0000;
    InitPieVectTable();
    EALLOW;  // This is needed to write to EALLOW protected registers

    /*
     *  인터럽트 함수 선언
     */
    PieVectTable.TINT0 = &cpu_timer0_isr;
    PieVectTable.ECAN0INTA  = &ISR_CANRXINTA;
//    PieVectTable.TINT2 = &cpu_timer2_isr;
    EDIS;    // This is needed to disable write to EALLOW protected registers
    InitGpio();
    GpioCtrlRegs.GPAMUX2.bit.GPIO18 = 3; //enable XCLOCKOUT through GPIO mux
    SysCtrlRegs.XCLK.bit.XCLKOUTDIV = 2; //XCLOCKOUT = SYSCLK
    InitSpiGpio();
    InitSpi();
    InitECanGpio();
    InitECan();

    MemCopy(&RamfuncsLoadStart, &RamfuncsLoadEnd, &RamfuncsRunStart);
    InitFlash();

    ConfigCpuTimer(&CpuTimer0, 80, 1000);
    //CpuTimer0Regs.PRD.all = 80000;// 90000 is 1msec
    CpuTimer0Regs.PRD.all = 80400;// 90000 is 1msec
    //   ConfigCpuTimer(&CpuTimer1, 80, 1000000);
    //   ConfigCpuTimer(&CpuTimer2, 80, 1000000);
    CpuTimer0Regs.TCR.all = 0x4001; // Use write-only instruction to set TSS bit = 0
    //  CpuTimer1Regs.TCR.all = 0x4001; // Use write-only instruction to set TSS bit = 0
    //  CpuTimer2Regs.TCR.all = 0x4001; // Use write-only instruction to set TSS bit = 0
//    InitAdc();
//    AdcOffsetSelfCal();
    EALLOW;
    EDIS;    // This is needed to disable write to EALLOW protected registers
    IER |= M_INT1;
    IER |= M_INT13;
    IER |= M_INT14;
    IER |= M_INT9;//test
    PieCtrlRegs.PIEIER1.bit.INTx7 = 1;      // Enable TINT0 in the PIE: Group 1 interrupt 7
    PieCtrlRegs.PIEIER9.bit.INTx5 = 1;      // Enable ECAN-A interrupt of PIE group 9
//  PieCtrlRegs.PIEIER9.bit.INTx1 = 1;      // SCIA RX interrupt of PIE group
    EINT;   // Enable Global interrupt INTM
    ERTM;   // Enable Global realtime interrupt DBGM
    SysRegs.SysMachine =System_STATE_INIT;
    Slave1Regs.StateMachine =STATE_BATIDLE;
    Slave2Regs.StateMachine =STATE_BATIDLE;
    Slave3Regs.StateMachine =STATE_BATIDLE;
    while(1)
    {

        SysRegs.Maincount++;

        switch(SysRegs.SysMachine)
        {
            case System_STATE_INIT:
                 SysTimerINIT(&SysRegs);
                 SysVarINIT(&SysRegs);
                 CANRegVarINIT(&CANARegs);
                 ProtectRelayVarINIT(&PrtectRelayRegs);
                 CalFarasis52AhRegsInit(&Farasis52AhSocRegs);
                 CalFrey60AhRegsInit(&Frey60AhSocRegs);
                 SysRegs.BAT80VDigitalOutPutReg.bit.LEDAlarmOUT=0;
                 SysRegs.BAT80VDigitalOutPutReg.bit.LEDFaultOUT=0;
                 SysRegs.BAT80VDigitalOutPutReg.bit.LEDProtectOUT=0;
                 SysRegs.BAT80VStateReg.bit.CANCOMEnable=0;
                 SysRegs.BAT80VStateReg.bit.BalanceMode=0;

                 /*
                  *
                  */
                 SysRegs.SysRegTimer5msecCount=0;
                 SysRegs.SysRegTimer10msecCount=0;
                 SysRegs.SysRegTimer50msecCount=0;
                 SysRegs.SysRegTimer100msecCount=0;
                 SysRegs.SysRegTimer300msecCount=0;
                 SysRegs.SysRegTimer500msecCount=0;
                 SysRegs.BalanceModeCount=0;
                 SysRegs.BalanceTimeCount=0;
                 SysRegs.Bat80VFaultStatecount=0;
           //      CANARegs.CellVoltagteTotalNum=0;
          //      CANARegs.CellVoltagteStartNum=0;
                 /*
                  *
                  */
                 if(Slave1Regs.StateMachine==STATE_BATIDLE)
                 {
                     Slave1Regs.ID=BMS_ID_1;
                     Slave1Regs.ErrorCount=0;
                     SlaveBMSIint(&Slave1Regs);
                     Slave1Regs.Balance.all=0x0000;
                     memset(&Slave1Regs.CellVoltage[0],3200,12);
                     memset(&Slave1Regs.CellTemperature[0],250,8);
                     Slave1Regs.StateMachine = STATE_BATSTANDBY;
                 }
                 if(Slave2Regs.StateMachine==STATE_BATIDLE)
                  {
                      Slave2Regs.ID=BMS_ID_2;
                      Slave2Regs.ErrorCount=0;
                      SlaveBMSIint(&Slave2Regs);
                      Slave2Regs.Balance.all=0x0000;
                      memset(&Slave2Regs.CellVoltage[0],3200,12);
                      memset(&Slave2Regs.CellTemperature[0],250,8);
                      Slave2Regs.StateMachine = STATE_BATSTANDBY;
                  }
                 if(Slave3Regs.StateMachine==STATE_BATIDLE)
                  {
                      Slave3Regs.ID=BMS_ID_3;
                      Slave3Regs.ErrorCount=0;
                      SlaveBMSIint(&Slave3Regs);
                      Slave3Regs.Balance.all=0x0000;
                      memset(&Slave3Regs.CellVoltage[0],3200,12);
                      memset(&Slave3Regs.CellTemperature[0],250,8);
                      Slave3Regs.StateMachine = STATE_BATSTANDBY;
                  }
                 SysRegs.BAT80VStateReg.bit.CANCOMEnable=0;
                 SysRegs.BAT80VStateReg.bit.BalanceMode=0;
                 SysRegs.BalanceModeCount=0;
                 SysRegs.BalanceTimeCount=0;

                  /*
                   *
                  */
                  delay_ms(500);
                  /*
                   *
                   */
                  /*
                   *
                   */
                  Farasis52AhSocRegs.state=SOC_STATE_IDLE;
                  Frey60AhSocRegs.state=SOC_STATE_IDLE;
                  SysRegs.BAT80VFaulBuftReg.all=0;
                  SysRegs.BAT12VFaulBuftReg.all=0;
                  SysRegs.SysMachine=System_STATE_STANDBY;

            break;
            case System_STATE_STANDBY:
                  SysRegs.BAT80VStateReg.bit.SysSTATE = 0;
                  SysRegs.BAT12VStateReg.bit.SysSTATE = 0;
                  SysRegs.BAT80VStateReg.bit.CANCOMEnable=0;
                  SysRegs.BAT80VDigitalOutPutReg.bit.LEDAlarmOUT=0;
                  SysRegs.BAT80VDigitalOutPutReg.bit.LEDFaultOUT=0;
                  /*
                   * CELL VOLTAGE measurement
                   */
                  Slave1Regs.StateMachine = STATE_BATREAD;
                  Slave2Regs.StateMachine = STATE_BATREAD;
                  Slave3Regs.StateMachine = STATE_BATREAD;
                  Slave1Regs.ID=BMS_ID_1;
                  SlaveVoltagHandler(&Slave1Regs);
                  Slave2Regs.ID=BMS_ID_2;
                  SlaveVoltagHandler(&Slave2Regs);
                  Slave3Regs.ID=BMS_ID_3;
                  SlaveVoltagHandler(&Slave3Regs);
                  /*
                   * BAT80V CELL VOLTATGE, SOC
                   * Initialize SOC calculation
                   */
                  memcpy(&SysRegs.Bat80VCellVoltageF[0],     &Slave1Regs.CellVoltageF[0],sizeof(float32)*12);
                  memcpy(&SysRegs.Bat80VCellVoltageF[12],    &Slave2Regs.CellVoltageF[0],sizeof(float32)*12);
                  Cal80VSysVoltageHandle(&SysRegs);
                  Farasis52AhSocRegs.CellAgvVoltageF = SysRegs.Bat80VCellAgvVoltageF;
                  CalFarasis52AhSocInit(&Farasis52AhSocRegs);
                  SysRegs.Bat80VSOCF=Farasis52AhSocRegs.SysSocInitF;
                  SysRegs.Bat80VSOCF=51; //test 삭제
                  Farasis52AhSocRegs.state= SOC_STATE_RUNNING;
                  /*
                   * BAT12V CELL VOLTATGE, SOC
                   * Initialize SOC calculation
                   */
                  memcpy(&SysRegs.Bat12VCellVoltageF[0], &Slave3Regs.CellVoltageF[0],sizeof(float32)*4);
                  Cal12VSysVoltageHandle(&SysRegs);
                  Frey60AhSocRegs.CellAgvVoltageF = SysRegs.Bat12VCellAgvVoltageF;
                  CalFrey60AhSocInit(&Frey60AhSocRegs);
                  SysRegs.Bat12VSOCF=Frey60AhSocRegs.SysSocInitF;
                  Frey60AhSocRegs.state= SOC_STATE_RUNNING;
                  /*
                   * SChange the state machine to System_STATE_READY
                   */
                  SysRegs.BAT80VFaulBuftReg.all=0;
                  SysRegs.BAT12VFaulBuftReg.all=0;
                  SysRegs.Bat80VFaultStatecount=0;
                  SysRegs.SysMachine=System_STATE_READY;
                  CANATX(0x600,8,CANARegs.SwVerProducttype.all,Product_Voltage,Product_Capacity,CANARegs.BAT80VConfing);
                  SysRegs.BAT80VStateReg.bit.CANCOMEnable=1;

            break;
            case System_STATE_READY:
                 SysRegs.BAT80VStateReg.bit.CANCOMEnable=1;
                 SysRegs.BAT80VStateReg.bit.SysSTATE = 1;
                 SysRegs.BAT12VStateReg.bit.SysSTATE = 1;
                 SysRegs.BAT80VDigitalOutPutReg.bit.LEDAlarmOUT=0;
                 if(SysRegs.BAT80VStateReg.bit.SysAalarm==1)
                 {
                     SysRegs.BAT80VStateReg.bit.SysSTATE = 3;
                     SysRegs.BAT80VDigitalOutPutReg.bit.LEDAlarmOUT=1;
                 }
                 if(SysRegs.BAT12VStateReg.bit.SysAalarm==1)
                 {
                     SysRegs.BAT12VStateReg.bit.SysSTATE = 3;
                     SysRegs.BAT80VDigitalOutPutReg.bit.LEDAlarmOUT=1;
                 }
                 if(SysRegs.BAT80VStateReg.bit.SysFault==1)
                 {
                     SysRegs.SysMachine=System_STATE_PROTECTER;
                 }
                 if(CANARegs.PMSCMDRegs.bit.RUNStatus==1)
                 {
                     PrtectRelayRegs.State.bit.WakeUpEN=1;
                     if(SysRegs.BAT80VStateReg.bit.SysFault==1)
                     {
                         SysRegs.SysMachine=System_STATE_PROTECTER;
                         PrtectRelayRegs.State.bit.WakeUpEN=0;
                     }
                     ProtectRelayWakeUpHandle(&PrtectRelayRegs);
                     SysRegs.SysMachine=System_STATE_RUNING;
                 }
                 //ProtecLatchRelayHandle(&PrtectRelayRegs);
            break;
            case System_STATE_RUNING:
                 SysRegs.BAT80VStateReg.bit.SysSTATE = 2;
                 SysRegs.BAT12VStateReg.bit.SysSTATE = 2;
                 SysRegs.BAT80VStateReg.bit.CANCOMEnable=1;
                 SysRegs.BAT80VDigitalOutPutReg.bit.LEDAlarmOUT=0;
                 if(SysRegs.BAT80VStateReg.bit.SysAalarm==1)
                 {
                     SysRegs.BAT80VStateReg.bit.SysSTATE = 3;
                     SysRegs.BAT80VDigitalOutPutReg.bit.LEDAlarmOUT=1;
                 }
                 if(SysRegs.BAT12VStateReg.bit.SysAalarm==1)
                 {
                     SysRegs.BAT12VStateReg.bit.SysSTATE = 3;
                     SysRegs.BAT80VDigitalOutPutReg.bit.LEDAlarmOUT=1;
                 }
                 /*
                  *
                  */
                 if(CANARegs.PMSCMDRegs.bit.RUNStatus==0)
                 {
                     PrtectRelayRegs.State.bit.WakeUpEN=0;
                     ProtectRelayWakeUpHandle(&PrtectRelayRegs);
                     SysRegs.SysMachine=System_STATE_READY;
                 }
                 if(SysRegs.BAT80VStateReg.bit.SysFault==1)
                 {
                     SysRegs.SysMachine=System_STATE_PROTECTER;
                 }
              //   ProtecLatchRelayHandle(&PrtectRelayRegs);
            break;
            case System_STATE_PROTECTER:
                 SysRegs.BAT80VStateReg.bit.SysSTATE =4;
                 SysRegs.BAT12VStateReg.bit.SysSTATE =4;
                 if(SysRegs.BAT80VStateReg.bit.SysFault==1)
                 {
                     ProtectRelayHandle(&PrtectRelayRegs);
                   // SysRegs.SysMachine=System_STATE_STANDBY;
                 }
                 if(CANARegs.PMSCMDRegs.bit.PrtctReset==1)
                 {
                     CANARegs.PMSCMDRegs.bit.PrtctReset=0;
                     SysRegs.BAT80VFaultReg.all=0;
                     SysRegs.BAT80VFaulBuftReg.all=0;
                     SysRegs.BAT80VFaultReg.all=0;
                     SysRegs.BAT80VStateReg.bit.SysFault=0;
                     delay_ms(200);
                     SysRegs.SysMachine=System_STATE_READY;
                     //SysRegs.BAT80VStateReg.bit.SysSTATE=2;
                 }
                 SysRegs.BAT80VStateReg.bit.CANCOMEnable=1;
                 SysRegs.BAT80VDigitalOutPutReg.bit.LEDAlarmOUT=0;
                 SysRegs.BAT80VDigitalOutPutReg.bit.LEDFaultOUT=1;
            break;
            case System_STATE_DATALOG:
                 SysRegs.BAT80VStateReg.bit.CANCOMEnable=1;
            break;
            case System_STATE_ProtectHistory:

            break;
            case System_STATE_MANUALMode:

            break;
            case System_STATE_CLEAR:

            break;
            default :
            break;
        }
        if(SysRegs.CellVoltsampling>=CellVoltsampling100msec)
        {
            /*
             *
             */
            if(SysRegs.Bat80VCurrentAsbF<=6.0)
            {
                SysRegs.BalanceModeCount++;
                if(SysRegs.BalanceModeCount>=100)
                {
                    SysRegs.BalanceModeCount=101;
                    SysRegs.BAT80VStateReg.bit.BalanceMode=1;
                }
            }
            if(SysRegs.Bat80VCurrentAsbF>6.0)
            {
                SysRegs.BAT80VStateReg.bit.BalanceMode=0;
                SysRegs.BAT80VStateReg.bit.BalanceStatStop=0;
                SysRegs.BalanceModeCount=0;
                SysRegs.BalanceTimeCount=0;
            }
            if(SysRegs.Bat80VCellMinVoltageF<3.0)
            {
                SysRegs.BAT80VStateReg.bit.BalanceMode=0;
                SysRegs.BAT80VStateReg.bit.BalanceStatStop=0;
                SysRegs.BalanceModeCount=0;
                SysRegs.BalanceTimeCount=0;
            }
            //SysRegs.BAT80VStateReg.bit.BalanceMode=0;
            if(SysRegs.BAT80VStateReg.bit.BalanceMode==1)
            {
                SysRegs.BalanceTimeCount++;
                if(SysRegs.BalanceTimeCount>10)
                {
                   SysRegs.BAT80VStateReg.bit.BalanceStatStop = !  SysRegs.BAT80VStateReg.bit.BalanceStatStop;
                   SysRegs.BalanceTimeCount=0;
                }
            }
            if(SysRegs.BAT80VStateReg.bit.BalanceStatStop==0)
            {
                /*
                 *
                 */
                Slave1Regs.Balance.all = 0x0000;
                Slave2Regs.Balance.all = 0x0000;
                Slave3Regs.Balance.all = 0x0000;
                SlaveBmsBalance(&Slave1Regs);
                SlaveBmsBalance(&Slave2Regs);
                SlaveBmsBalance(&Slave3Regs);
                /*
                 *
                 */

                Slave1Regs.ID=BMS_ID_1;
                SlaveVoltagHandler(&Slave1Regs);
                /*
                 *
                 */
                Slave2Regs.ID=BMS_ID_2;
                SlaveVoltagHandler(&Slave2Regs);
                /*
                 *
                 */
                Slave3Regs.ID=BMS_ID_3;
                SlaveVoltagHandler(&Slave3Regs);

                //SalveTempsVoltHandler(&Slave1Regs);
                //Slave1Regs.BATICDO.bit.GPIO1= ! Slave1Regs.BATICDO.bit.GPIO1;
                //SlaveBMSDigiteldoutOHandler(&Slave1Regs);
                //delay_ms(1);
                Slave1Regs.BatICTempsF = Slave1Temps;
                SalveTempsVoltHandler_B(&Slave1Regs);

                //SalveTempsVoltHandler(&Slave2Regs);
                //Slave2Regs.BATICDO.bit.GPIO1= ! Slave2Regs.BATICDO.bit.GPIO1;
                //SlaveBMSDigiteldoutOHandler(&Slave2Regs);
                //delay_ms(1);
                Slave2Regs.BatICTempsF = Slave2Temps;
                SalveTempsVoltHandler_B(&Slave2Regs);

                //Slave3Regs.BATICDO.bit.GPIO1= ! Slave3Regs.BATICDO.bit.GPIO1;
                //SlaveBMSDigiteldoutOHandler(&Slave3Regs);
                //SalveTempsVoltHandler(&Slave3Regs);
                //delay_ms(1);
                Slave3Regs.BatICTempsF = Slave3Temps;
                SalveTempsVoltHandler_B(&Slave3Regs);
            }
            if(SysRegs.BAT80VStateReg.bit.BalanceStatStop==1)
            {
                Slave1Regs.ID=BMS_ID_1;
                Slave1Regs.SysCellMinVoltage = SysRegs.Bat80VCellMinVoltageF;
                SlaveVoltagBalaHandler(&Slave1Regs);
                SlaveBmsBalance(&Slave1Regs);

                Slave2Regs.ID=BMS_ID_2;
                Slave2Regs.SysCellMinVoltage = SysRegs.Bat80VCellMinVoltageF;
                SlaveVoltagBalaHandler(&Slave2Regs);
                SlaveBmsBalance(&Slave2Regs);

            }
            /*
             Slave1Regs.BATICDO.bit.GPIO1= ! Slave1Regs.BATICDO.bit.GPIO1;
             SlaveBMSDigiteldoutOHandler(&Slave1Regs);
             delay_ms(1);
             SalveTempsVoltHandler(&Slave1Regs);

             Slave2Regs.BATICDO.bit.GPIO1= ! Slave2Regs.BATICDO.bit.GPIO1;
             SlaveBMSDigiteldoutOHandler(&Slave2Regs);
             delay_ms(1);

             SalveTempsVoltHandler(&Slave2Regs);
             */
          //  Slave3Regs.CellVoltageF[0]=Slave3Regs.CellVoltageF[0]+0.094;
          //  Slave3Regs.CellVoltageF[1]=Slave3Regs.CellVoltageF[1]+0.085;
          //  Slave3Regs.CellVoltageF[2]=Slave3Regs.CellVoltageF[2]+0.082;
          //  Slave3Regs.CellVoltageF[3]=Slave3Regs.CellVoltageF[3]+0.086;
            SysRegs.CellVoltsampling=0;

        }
        if(SysRegs.CellTempssampling>50)
        {

             SysRegs.CellTempssampling=0;

        }
        if(SysRegs.Maincount>3000){SysRegs.Maincount=0;}
        LTC6804_DieTemperatureRead(BMS_ID_1, &Slave1Temps);
        LTC6804_DieTemperatureRead(BMS_ID_2, &Slave2Temps);
        LTC6804_DieTemperatureRead(BMS_ID_3, &Slave3Temps);
      //  randValuem =rand();// (float32) rand();
    }
  /*  if(SysRegs.SysMachine==System_STATE_READY)//||(SysRegs.SysMachine==System_STATE_RUNING)||(SysRegs.SysMachine==System_STATE_PROTECTER))
    {
        SysRegs.SysMachine=System_STATE_STANDBY;
    }*/

}

interrupt void cpu_timer0_isr(void)
{
   SysRegs.MainIsr1++;
   SysRegs.SysRegTimer5msecCount++;
   SysRegs.SysRegTimer10msecCount++;
   SysRegs.SysRegTimer50msecCount++;
   SysRegs.SysRegTimer100msecCount++;
   SysRegs.SysRegTimer300msecCount++;
   SysRegs.SysRegTimer500msecCount++;
   SysRegs.SysRegTimer1000msecCount++;
   SysRegs.CellVoltsampling++;
   SysRegs.CellTempssampling++;
   if(SysRegs.SysRegTimer5msecCount   >SysRegTimer5msec)    {SysRegs.SysRegTimer5msecCount=0;}
   if(SysRegs.SysRegTimer10msecCount  >SysRegTimer10msec)   {SysRegs.SysRegTimer10msecCount=0;}
   if(SysRegs.SysRegTimer50msecCount  >SysRegTimer50msec)   {SysRegs.SysRegTimer50msecCount=0;}
   if(SysRegs.SysRegTimer100msecCount >SysRegTimer100msec)  {SysRegs.SysRegTimer100msecCount=0;}
   if(SysRegs.SysRegTimer300msecCount >SysRegTimer300msec)   {SysRegs.SysRegTimer300msecCount=0;}
   if(SysRegs.SysRegTimer1000msecCount>SysRegTimer1000msec)  {SysRegs.SysRegTimer1000msecCount=0;}

   /*
    * DigitalInput detection
    */
   DigitalInput(&SysRegs);
  /*
   * current sensing detection
  */
  // DigitalInput(&SysRegs);
   Cal80VSysCurrentHandle(&SysRegs);
   Cal12VSysCurrentHandle(&SysRegs);
   /*
    *  Farasis52AhSocRegs.CellAgvVoltageF = NCMsocTestVoltAGV;
    *  Farasis52AhSocRegs.SysSoCCTF       = NUMsocTestVCT;
    *  Farasis52AhSocRegs.SysSoCCTAbsF    = NUMsocTestVCT;
    */
   //Farasis52ASocRegs
   Farasis52AhSocRegs.CellAgvVoltageF = SysRegs.Bat80VCellAgvVoltageF;
   Farasis52AhSocRegs.SysSoCCTF       = SysRegs.Bat80VCurrentF;
   Farasis52AhSocRegs.SysSoCCTAbsF    = SysRegs.Bat80VCurrentAsbF;
   CalFarasis52AhSocHandle(&Farasis52AhSocRegs);
   if(Farasis52AhSocRegs.SoCStateRegs.bit.CalMeth==0)
   {
       SysRegs.Bat80VSOCF=Farasis52AhSocRegs.SysSocInitF;
   }
   if(Farasis52AhSocRegs.SoCStateRegs.bit.CalMeth==1)
   {
       SysRegs.Bat80VSOCF=Farasis52AhSocRegs.SysSOCF;
   }
   /*
    * Frey60AhSocRegs.CellAgvVoltageF = LFPsocTestVoltAGV;
    * Frey60AhSocRegs.SysSoCCTF       = LFPsocTestVCT;
    * Frey60AhSocRegs.SysSoCCTAbsF    = LFPsocTestVCT;
    */
   //Frey60AhSocRegs
   Frey60AhSocRegs.CellAgvVoltageF = SysRegs.Bat12VCellAgvVoltageF;
   Frey60AhSocRegs.SysSoCCTF       = -1.0* SysRegs.Bat12VCurrentF;
   Frey60AhSocRegs.SysSoCCTAbsF    =  SysRegs.Bat12VCurrentAsbF;
   CalFrey60AhSocHandle(&Frey60AhSocRegs);
   if(Frey60AhSocRegs.SoCStateRegs.bit.CalMeth==0)
   {
       SysRegs.Bat12VSOCF=Frey60AhSocRegs.SysSocInitF;
   }
   if(Frey60AhSocRegs.SoCStateRegs.bit.CalMeth==1)
   {
       SysRegs.Bat12VSOCF=Frey60AhSocRegs.SysSOCF;
   }

   /*
    * 80V Battery Alarm & Fault Check
    */
   Cal80VSysAlarmtCheck(&SysRegs);
   if(SysRegs.BAT80VAlarmReg.all != 0)
   {
       SysRegs.BAT80VStateReg.bit.SysAalarm=1;
   }
   else
   {
       SysRegs.BAT80VStateReg.bit.SysAalarm=0;
   }
   // Cal80VSysFaultCheck(&SysRegs);
   if(SysRegs.BAT80VFaultReg.all != 0)
   {

       SysRegs.BAT80VStateReg.bit.SysFault=1;
   }
   else
   {
       SysRegs.BAT80VStateReg.bit.SysFault=0;
   }
   /*
    * 12V Battery Alarm & Fault Check
    */
   Cal12VSysAlarmtCheck(&SysRegs);
   if(SysRegs.BAT12VAlarmReg.all != 0)
   {
       SysRegs.BAT12VStateReg.bit.SysAalarm=1;
   }
   else
   {
       SysRegs.BAT12VStateReg.bit.SysAalarm=0;
   }
   //Cal12VSysFaultCheck(&SysRegs);
   if(SysRegs.BAT80VFaultReg.all != 0)
   {
       SysRegs.BAT12VStateReg.bit.SysFault=1;
   }
   else
   {
       SysRegs.BAT12VStateReg.bit.SysFault=0;
   }
   /*
    *
    */

   switch(SysRegs.SysRegTimer5msecCount)
   {
       case 1:

       break;
       default :
       break;

   }
   switch(SysRegs.SysRegTimer10msecCount)
   {
       case 1:
               SysRegs.SysCanRxCount++;
               if(SysRegs.SysCanRxCount>=3000)
               {
                   CANARegs.PMSCMDRegs.bit.RUNStatus=0;
                   SysRegs.BAT80VFaultReg.bit.PackISO_ERR=1;
                   SysRegs.SysCanRxCount=11000;
               }
       break;
       case 2:
               if(SysRegs.BAT80VStateReg.bit.CANCOMEnable==1)
               {
                   CANARegs.BAT80VPT = (unsigned int)(SysRegs.Bat80VVoltageF*10);
                   CANARegs.BAT80VCT = (int)(SysRegs.Bat80VCurrentF*10);
                   CANARegs.BAT80VSOC =(unsigned int)(SysRegs.Bat80VSOCF*10);
                   CANARegs.BAT80VSOH =(unsigned int)(SysRegs.Bat80VSOHF*10);
                   //At 80MHZ, operation time is 0.151msec
                   CANATX(0x601,8,CANARegs.BAT80VPT,CANARegs.BAT80VCT,CANARegs.BAT80VSOC,1000);
               }
       break;
       case 3:
               if(SysRegs.BAT80VStateReg.bit.CANCOMEnable==1)
               {
                 CANARegs.BAT12VPT = (unsigned int)(SysRegs.Bat12VVoltageF*10);
                 CANARegs.BAT12VCT = (int)(SysRegs.Bat12VCurrentF*-10.0);
                 CANARegs.BAT12VSOC =(unsigned int)(SysRegs.Bat12VSOCF*10);
                 SysRegs.Bat12VSOHF=100;
                 CANARegs.BAT12VSOH =(unsigned int)(SysRegs.Bat12VSOHF*10);
                //At 80MHZ, operation time is 0.151msec
                 CANATX(0x607,8,CANARegs.BAT12VPT,CANARegs.BAT12VCT,CANARegs.BAT12VSOC,CANARegs.BAT12VSOH );
               }
       break;

       default :
       break;
   }
   switch(SysRegs.SysRegTimer50msecCount)
   {
       case 1:
              //LEDSysState_H;
              //At 80MHZ, operation time is 1.29usec
              //2호,3 차량
              memcpy(&SysRegs.Bat80VCellVoltageF[0],     &Slave1Regs.CellVoltageF[0],sizeof(float32)*12);
              memcpy(&SysRegs.Bat80VCellVoltageF[12],    &Slave2Regs.CellVoltageF[0],sizeof(float32)*12);
             // memcpy(&SysRegs.Bat80VCellVoltageF[0],     &Slave1Regs.CellVoltageF[0],sizeof(float32)*12);
             // memcpy(&SysRegs.Bat80VCellVoltageF[12],    &Slave2Regs.CellVoltageF[0],sizeof(float32)*12);
              //LEDSysState_L;
       break;
       case 5:
              //LEDSysState_H;
               SysRegs.Bat12VCellVoltageF[0] = Slave3Regs.CellVoltageF[1];
               SysRegs.Bat12VCellVoltageF[1] = Slave3Regs.CellVoltageF[2];
               SysRegs.Bat12VCellVoltageF[2] = Slave3Regs.CellVoltageF[2];
               SysRegs.Bat12VCellVoltageF[3] = Slave3Regs.CellVoltageF[0];
              //memcpy(&SysRegs.Bat12VCellVoltageF[0],  &Slave3Regs.CellVoltageF[0],sizeof(float32)*4);
              Cal12VSysVoltageHandle(&SysRegs);
              //LEDSysState_L;
       break;
       case 10:
               //LEDSysState_H;
               //At 80MHZ, operation time is 33usec
               Cal80VSysVoltageHandle(&SysRegs);
               //LEDSysState_L;
       break;
       case 20:
               //LEDSysState_H;
               //At 80MHZ, operation time is 33usec

               //LEDSysState_L;
       break;
       case 30 :
               memcpy(&CANARegs.BAT80VoltageCell[0],     &Slave1Regs.CellVoltage[0],sizeof(Uint16)*12);
               memcpy(&CANARegs.BAT80VoltageCell[12],    &Slave2Regs.CellVoltage[0],sizeof(Uint16)*12);

       break;
       case 40 :

       break;
       default :
       break;
   }

   switch(SysRegs.SysRegTimer100msecCount)
   {
       case 5:
                //At 80MHZ, operation time is 0.151msec
               if(SysRegs.BAT80VStateReg.bit.CANCOMEnable==1)
               {
                   CANARegs.BAT80VStatus.bit.BATStatus =SysRegs.BAT80VStateReg.bit.SysSTATE;
                   CANARegs.BAT80VDigitalOutPutReg.bit.NRlyOUT= PrtectRelayRegs.State.bit.NRelayDO;
                   CANARegs.BAT80VDigitalOutPutReg.bit.CHARlyOUT= PrtectRelayRegs.State.bit.PreRelayDO;
                   CANARegs.BAT80VDigitalOutPutReg.bit.PRlyOUT= PrtectRelayRegs.State.bit.PRelayDO;
                   CANARegs.BAT80VStatus.bit.BalanceEN =SysRegs.BAT80VStateReg.bit.BalanceMode;
                   CANARegs.BAT80VAh = (int)(SysRegs.Bat80VAhF*10);
                   CANATX(0x602,8,CANARegs.BAT80VStatus.all,CANARegs.BAT80VDigitalOutPutReg.all,CANARegs.BAT80VAh,0X000);
               }
       break;
       case 8:
                //At 80MHZ, operation time is 0.151msec
               if(SysRegs.BAT80VStateReg.bit.CANCOMEnable==1)
               {
                   CANATX(0x603,8,SysRegs.BAT80VAlarmReg.all,SysRegs.BAT80VFaultReg.all,0X000,0X000);
               }
       break;
       case 11:
                //At 80MHZ, operation time is 0.151msec
               if(SysRegs.BAT80VStateReg.bit.CANCOMEnable==1)
               {
                 SysRegs.Bat80VCHAPWRContintyF    =  12.2;
                 SysRegs.Bat80VDisCHAPWRContintyF =  8.6;
                 SysRegs.Bat80VCHAPWRPeakF = 20.2;
                 SysRegs.Bat80VDisCHAPWRPeakF = 12.2;
                 CANARegs.BAT80VCHAPWRContinty    = (unsigned int)(SysRegs.Bat80VCHAPWRContintyF*10);
                 CANARegs.BAT80VDisCHAPWRContinty = (unsigned int)(SysRegs.Bat80VDisCHAPWRContintyF*10);
                 CANARegs.BAT80VCHAPWRPeak        = (unsigned int)(SysRegs.Bat80VCHAPWRPeakF*10);
                 CANARegs.BAT80VDisCHAPWRPeak     = (unsigned int)(SysRegs.Bat80VDisCHAPWRPeakF*10);
                 CANATX(0x604,8,CANARegs.BAT80VCHAPWRContinty,CANARegs.BAT80VDisCHAPWRContinty,CANARegs.BAT80VCHAPWRPeak,CANARegs.BAT80VDisCHAPWRPeak);
               }
       break;
       case 14:
           //At 80MHZ, operation time is 0.151msec
                if(SysRegs.BAT80VStateReg.bit.CANCOMEnable==1)
                {
                   CANARegs.BAT80VoltageMax = (unsigned int)(SysRegs.Bat80VCellMaxVoltageF*1000);
                   CANARegs.BAT80VoltageMin = (unsigned int)(SysRegs.Bat80VCellMinVoltageF*1000);
                   CANARegs.BAT80VoltageAgv = (unsigned int)(SysRegs.Bat80VCellAgvVoltageF*1000);
                   CANARegs.BAT80VoltageDiv = (unsigned int)(SysRegs.Bat80VCellDivVoltageF*1000);
                   CANATX(0x605,8,CANARegs.BAT80VoltageMax,CANARegs.BAT80VoltageMin,CANARegs.BAT80VoltageAgv,CANARegs.BAT80VoltageDiv);
                }
       break;
       case 17:
                if(SysRegs.BAT80VStateReg.bit.CANCOMEnable==1)
                {

                    //SysRegs.Bat80VCellMaxTemperatureF =S//18.6+SysRegs.NumB;
                    //SysRegs.Bat80VCellMinTemperatureF =//16.3+SysRegs.NumB;
                    //SysRegs.Bat80VCellAgvTemperatureF =//(SysRegs.Bat80VCellMaxTemperatureF+SysRegs.Bat80VCellMinTemperatureF)/2;
                    //SysRegs.Bat80VCellDivTemperatureF = //SysRegs.Bat80VCellMaxTemperatureF-SysRegs.Bat80VCellMinTemperatureF;
                    CANARegs.BAT80VTemperaturelMAX    = (int)(SysRegs.Bat80VCellMaxTemperatureF*10);
                    CANARegs.BAT80VTemperaturelMIN    = (int)(SysRegs.Bat80VCellMinTemperatureF*10);
                    CANARegs.BAT80VTemperatureAVG     = (int)(SysRegs.Bat80VCellAgvTemperatureF*10);
                    CANARegs.BAT80VTemperatureDiv     = (int)(SysRegs.Bat80VCellDivTemperatureF*10);
                    CANATX(0x606,8,CANARegs.BAT80VTemperaturelMAX,CANARegs.BAT80VTemperaturelMIN,CANARegs.BAT80VTemperatureAVG,CANARegs.BAT80VTemperatureDiv);
                }
       break;
       case 20:
                if(SysRegs.BAT80VStateReg.bit.CANCOMEnable==1)
                {
                   //CANARegs.BAT12VStatus_A.bit.BalanceEN
                   CANARegs.BAT12VStatus_A.bit.PRelayEN=1;
                   CANARegs.BAT12VAh = (int)(SysRegs.Bat12VAhF*10);
                   CANATX(0x608,8,CANARegs.BAT12VStatus_A.all,0x0000,CANARegs.BAT12VAh,0X000);
                }
       break;
       case 23:
                if(SysRegs.BAT80VStateReg.bit.CANCOMEnable==1)
                {
                  CANATX(0x609,8,SysRegs.BAT12VAlarmReg.all,SysRegs.BAT12VFaultReg.all,0X000,0X000);
                }
       break;
       case 26:

       break;
       case 30:
               if(SysRegs.BAT80VStateReg.bit.CANCOMEnable==1)
               {
                   CANARegs.BAT12VoltageMax         = (unsigned int)(SysRegs.Bat12VCellMaxVoltageF*1000);
                   CANARegs.BAT12VoltageMin         = (unsigned int)(SysRegs.Bat12VCellMinVoltageF*1000);
                   CANARegs.BAT12VoltageAgv         = (unsigned int)(SysRegs.Bat12VCellAgvVoltageF*1000);
                   CANARegs.BAT12VoltageDiv         = (unsigned int)(SysRegs.Bat12VCellDivVoltageF*10);

                   CANATX(0x60A,8,CANARegs.BAT12VoltageMax,CANARegs.BAT12VoltageMin,CANARegs.BAT12VoltageAgv,CANARegs.BAT12VoltageDiv);
               }
       break;
       case 35:
           SysRegs.NumA=SysRegs.MainIsr1/300;
           SysRegs.NumB=(float32)(SysRegs.NumA*0.1);
               if(SysRegs.BAT80VStateReg.bit.CANCOMEnable==1)
               {
                //   SysRegs.Bat12VCellMaxTemperatureF = 19.6+SysRegs.NumB;
                 //  SysRegs.Bat12VCellMinTemperatureF = 17.6+SysRegs.NumB;
             //      SysRegs.Bat12VCellAgvTemperatureF = (SysRegs.Bat12VCellMaxTemperatureF+SysRegs.Bat12VCellMinTemperatureF)/2;
              //     SysRegs.Bat12VCellDivTemperatureF =  SysRegs.Bat12VCellMaxTemperatureF-SysRegs.Bat12VCellMinTemperatureF;
              //     SysRegs.Bat12VCellMaxTemperatureF     = SysRegs.Bat12VCellMaxTemperatureF-3.0;
              //     SysRegs.Bat12VCellMinTemperatureF     = SysRegs.Bat12VCellMinTemperatureF-2.0;
              //     SysRegs.Bat12VCellAgvTemperatureF     =(SysRegs.Bat12VCellMaxTemperatureF+SysRegs.Bat12VCellMinTemperatureF)/2.0;
              //     SysRegs.Bat12VCellDivTemperatureF     =SysRegs.Bat12VCellMaxTemperatureF-SysRegs.Bat12VCellMinTemperatureF;
                   CANARegs.BAT12VTemperatureMAX     = (int)(SysRegs.Bat12VCellMaxTemperatureF*10);
                   CANARegs.BAT12VTemperaturelMIN    = (int)(SysRegs.Bat12VCellMinTemperatureF*10);
                   CANARegs.BAT12VTemperatureAVG     = (int)(SysRegs.Bat12VCellAgvTemperatureF*10);
                   CANARegs.BAT12VTemperatureDiv     = (int)(SysRegs.Bat12VCellDivTemperatureF*10);
                   CANATX(0x60B,8,CANARegs.BAT12VTemperatureMAX,CANARegs.BAT12VTemperaturelMIN,CANARegs.BAT12VTemperatureAVG,CANARegs.BAT12VTemperatureDiv);
               }
       break;
       case 38:
               if(SysRegs.BAT80VStateReg.bit.CANCOMEnable==1)
               {
                  // SysRegs.Bat12VCellMaxTemperatureF = 22.3;
                  // SysRegs.Bat12VCellMinTemperatureF = 20.1;
                  // SysRegs.Bat12VCellMaxTemperatureF = 21.7;
                  // SysRegs.Bat12VCellDivTemperatureF = SysRegs.Bat12VCellMaxTemperatureF-SysRegs.Bat12VCellMinTemperatureF;
                  // CANARegs.BAT12VTemperatureMAX     = (unsigned int)(SysRegs.Bat12VCellMaxTemperatureF*10);
                  // CANARegs.BAT12VTemperaturelMIN    = (unsigned int)(SysRegs.Bat12VCellMinTemperatureF*10);
                  // CANARegs.BAT12VTemperatureAVG     = (unsigned int)(SysRegs.Bat12VCellAgvVoltageF*10);
                   //CANARegs.BAT12VTemperatureDiv     = (unsigned int)(SysRegs.Bat12VCellDivTemperatureF*10);
                   CANATX(0x60C,8,CANARegs.MailBox0RxCount,CANARegs.MailBox1RxCount,CANARegs.MailBox2RxCount,SysRegs.SysCanRxCount);
               }
       break;
       case 45:
               if(SysRegs.BAT80VStateReg.bit.CANCOMEnable==1)
               {
                  //CANARegs.BAT12VCHAPWRContinty    = (unsigned int)(SysRegs.Bat12VCHAPWRContintyF*0.001);
                  //CANARegs.BAT12VDisCHAPWRContinty = (unsigned int)(SysRegs.Bat12VDisCHAPWRContintyF*0.001);
                  //CANARegs.BAT12VCHAPWRPeak        = (unsigned int)(SysRegs.Bat12VCHAPWRPeakF*0.001);
                  //CANARegs.BAT12VDisCHAPWRPeak     = (unsigned int)(SysRegs.Bat12VDisCHAPWRPeakF*0.001);
                  CANATX(0x060D,8,Slave3Regs.CellVoltage[0],Slave3Regs.CellVoltage[1],Slave3Regs.CellVoltage[2],Slave3Regs.CellVoltage[3]);
               }

       break;
       default:
       break;
   }

   switch(SysRegs.SysRegTimer300msecCount)
   {

       case 50:
              SysRegs.Bat12VCellTemperatureF[0] =Slave1Regs.CellTemperatureF[0]+2.0;
              SysRegs.Bat12VCellTemperatureF[1] =Slave1Regs.CellTemperatureF[0]+1.0;
              SysRegs.Bat12VCellTemperatureF[2] =Slave1Regs.CellTemperatureF[0]+3.0;
              SysRegs.Bat12VCellTemperatureF[3] =Slave1Regs.CellTemperatureF[0]-1.0;
               //memcpy(&SysRegs.Bat12VCellTemperatureF[0],    &Slave1Regs.CellTemperatureF[0],sizeof(float32)*2);
               //memcpy(&SysRegs.Bat12VCellTemperatureF[2],    &Slave1Regs.CellTemperatureF[0],sizeof(float32)*2);
       break;

       case 100:
               memcpy(&SysRegs.Bat80VCellTemperatureF[0],    &Slave1Regs.CellTemperatureF[0],sizeof(float32)*6);
               memcpy(&SysRegs.Bat80VCellTemperatureF[6],    &Slave2Regs.CellTemperatureF[0],sizeof(float32)*6);
       break;

       case 120:
               memcpy(&CANARegs.BAT12TempCell[0],           &Slave3Regs.CellTemperature[0],sizeof(int)*4);
       break;

       case 150:
               memcpy(&CANARegs.BAT80VTemperatureCell[0],    &Slave1Regs.CellTemperature[0],sizeof(int)*6);
               memcpy(&CANARegs.BAT80VTemperatureCell[6],    &Slave2Regs.CellTemperature[0],sizeof(int)*6);
       break;

       case 200:
               Cal80VSysTemperatureHandle(&SysRegs);
       break;

       case 250:
               Cal12VSysTemperatureHandle(&SysRegs);
       break;
       default :
       break;
   }
   if(SysRegs.SysRegTimer500msecCount >SysRegTimer500msec)
   {
       //CANATX(0x60D,8,CANARegs.SwVerProducttype.all,Product_Voltage,Product_Capacity,CANARegs.BAT80VConfing);
       SysRegs.SysRegTimer500msecCount=0;
   }
   switch(SysRegs.SysRegTimer1000msecCount)
   {
       case 1:

               if(SysRegs.BAT80VStateReg.bit.CANCOMEnable==1)
               {
                   CANARegs.SwVerProducttype.byte.BYTEL = Product_Type;
                   CANARegs.SwVerProducttype.byte.BYTEH = Product_Version;
                   CANARegs.BAT80VConfing = 0x0118;
                   //CANARegs.BatConfParallelSerial.byte.BYTEL= Product_SysCellVauleS;
                  // CANARegs.BatConfParallelSerial.byte.BYTEH= Product_SysCellVauleP;
                   CANATX(0x600,8,CANARegs.SwVerProducttype.all,Product_Voltage,Product_Capacity,CANARegs.BAT80VConfing);
               }

       break;
       default :
       break;
   }

   DigitalOutput(&SysRegs);

   //
   InitECan();
   // Acknowledge this interrupt to receive more interrupts from group 1

   if(SysRegs.MainIsr1>3000) {SysRegs.MainIsr1=0;}
   PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}
interrupt void ISR_CANRXINTA(void)
{
    struct ECAN_REGS ECanaShadow;
    if(ECanaRegs.CANGIF0.bit.GMIF0 == 1)
    {
        CANARegs.MailBoxRxCount++;
        if(CANARegs.MailBoxRxCount>3000){CANARegs.MailBoxRxCount=0;}
        if(ECanaRegs.CANRMP.bit.RMP0==1)
        {
            if(ECanaMboxes.MBOX0.MSGID.bit.STDMSGID==0x3C2)
            {
                CANARegs.MailBox0RxCount++;
                if(CANARegs.MailBox0RxCount>3000){CANARegs.MailBox0RxCount=0;}
                SysRegs.Bat80VCurrentData.byte.CurrentH   = (ECanaMboxes.MBOX0.MDL.byte.BYTE0<<8)|(ECanaMboxes.MBOX0.MDL.byte.BYTE1);
                SysRegs.Bat80VCurrentData.byte.CurrentL   = (ECanaMboxes.MBOX0.MDL.byte.BYTE2<<8)|(ECanaMboxes.MBOX0.MDL.byte.BYTE3);
            }
        }
        if(ECanaRegs.CANRMP.bit.RMP1==1)
        {
            if(ECanaMboxes.MBOX1.MSGID.bit.STDMSGID==0x3C3)
            {
               CANARegs.MailBox1RxCount++;
               if(CANARegs.MailBox1RxCount>3000){CANARegs.MailBox1RxCount=0;}
               SysRegs.Bat12VCurrentData.byte.CurrentH   = (ECanaMboxes.MBOX1.MDL.byte.BYTE0<<8)|(ECanaMboxes.MBOX1.MDL.byte.BYTE1);
               SysRegs.Bat12VCurrentData.byte.CurrentL   = (ECanaMboxes.MBOX1.MDL.byte.BYTE2<<8)|(ECanaMboxes.MBOX1.MDL.byte.BYTE3);
      //         if(CANRXRegs.MailBox0RxCount>3000)

            }
        }
        if(ECanaRegs.CANRMP.bit.RMP2==1)
        {
            if(ECanaMboxes.MBOX2.MSGID.bit.STDMSGID==0x700)
            {
                CANARegs.MailBox2RxCount++;
                if(CANARegs.MailBox2RxCount>3000){CANARegs.MailBox2RxCount=0;}
                //CANRXRegs.VCUCMDCount=0;
                 CANARegs.PMSCMDRegs.all      =  (ECanaMboxes.MBOX2.MDL.byte.BYTE1<<8)|(ECanaMboxes.MBOX2.MDL.byte.BYTE0);
                //CANRXRegs.WORD700_1          =  (ECanaMboxes.MBOX2.MDL.byte.BYTE2<<8)|(ECanaMboxes.MBOX2.MDL.byte.BYTE3);
                //CANRXRegs.WORD700_2          =  (ECanaMboxes.MBOX2.MDH.byte.BYTE5<<8)|(ECanaMboxes.MBOX2.MDH.byte.BYTE4);
                //CANRXRegs.WORD700_3          =  (ECanaMboxes.MBOX2.MDH.byte.BYTE7<<8)|(ECanaMboxes.MBOX2.MDH.byte.BYTE6);
                SysRegs.SysCanRxCount=0;
            }

        }
        if(ECanaRegs.CANRMP.bit.RMP3==1)
        {
            if(ECanaMboxes.MBOX3.MSGID.bit.STDMSGID==0x400)
            {
                CANARegs.MailBox3RxCount++;
                if(CANARegs.MailBox3RxCount>3000){CANARegs.MailBox3RxCount=0;}
                //CANRXRegs.VCUCMDCount=0;
                //CANRXRegs.PCCMDRegs.all      =  (ECanaMboxes.MBOX3.MDL.byte.BYTE1<<8)|(ECanaMboxes.MBOX3.MDL.byte.BYTE0);
                //CANRXRegs.WORD700_1          =  (ECanaMboxes.MBOX3.MDL.byte.BYTE2<<8)|(ECanaMboxes.MBOX3.MDL.byte.BYTE3);
                //CANRXRegs.WORD700_2          =  (ECanaMboxes.MBOX3.MDH.byte.BYTE5<<8)|(ECanaMboxes.MBOX3.MDH.byte.BYTE4);
                //CANRXRegs.WORD700_3          =  (ECanaMboxes.MBOX3.MDH.byte.BYTE7<<8)|(ECanaMboxes.MBOX3.MDH.byte.BYTE6);
            }
        }
    }
    ECanaShadow.CANRMP.all = 0;
    ECanaShadow.CANRMP.bit.RMP0 = 1;  //interrupt pending clear by writing 1
    ECanaShadow.CANRMP.bit.RMP1 = 1;
    ECanaShadow.CANRMP.bit.RMP2 = 1;
    ECanaShadow.CANRMP.bit.RMP3 = 1;
    ECanaRegs.CANRMP.all = ECanaShadow.CANRMP.all ;
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP9;

   // IER |= 0x0100;                  // Enable INT9
   // EINT;

}//EOF
/*
interrupt void cpu_timer2_isr(void)
{  EALLOW;
   CpuTimer2.InterruptCount++;
   // The CPU acknowledges the interrupt.
  // A_OVCHACurrent;
   EDIS;
}
*/
