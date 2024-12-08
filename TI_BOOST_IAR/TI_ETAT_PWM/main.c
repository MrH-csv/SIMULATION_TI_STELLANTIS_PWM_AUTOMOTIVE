
/*******************************************************************************
**                                 Author(s)                                  **
********************************************************************************
** Initials     Name     AHNM                   19/11/2024                    **
** ---------------------------------------------------------------------------**
** DM           Angel Habid Navarro                                           **
*******************************************************************************/

/*******************************************************************************
**                            Dependencies and Includes                       **
*******************************************************************************/
#include "tle_device.h"
#include "RTE_Components.h"

/*******************************************************************************
**                             Macros                                         **
*******************************************************************************/
#define ETAT_port_P0_1 0x02 
#define GPIO_P0_1_AS_MEDIUM_DRIVER 0X44
#define GPIO_P0_OPEN_DRAIN_disable 0X00
#define GPIO_PULL_UP_PIN_P0_1_SELCTED 0x02
#define P0_1_PULL_UP_ENABLE 0x02
#define ETAT_HIGH  PORT->P0_DATA.reg=0x02
#define ETAT_LOW PORT->P0_DATA.reg=0x00
#define ETAT_Freq 100 /** Frequency = 10hz = 100ms because the systick is set to 1 ms */
#define MCU_Sys_Freq 40000000
#define Sys_CLK_Prescal 64
#define Low_State 0
#define High_State 1
#define CDE_port_P1_2_direction 0x00/**P1.2 as input */
#define GPIO_P1_2_AS_MEDIUM_DRIVER 0X44
/*******************************************************************************
**                             Function prototypes.                           **
*******************************************************************************/
extern void Systick_IRQ_subroutine(void);
extern void Timer2_IRQ_subroutine(void);
void GPIO_INT(void);
void ETAT_PWM(void);
void Tog_ETAT(void);
void CDE_PWM_Measurement (void);
int DiagMatrix(void);

/*******************************************************************************
**                             Global Variables   .                           **
*******************************************************************************/
volatile unsigned int ETAT_PWM_Status;
volatile unsigned int Global_counter_V;
unsigned int PWM_control_var;
volatile unsigned int Period_ticks;
volatile uint16_t timer_low_count;// homologar tipos de datos genericos.
volatile uint16_t timer_high_count;
static uint16_t Period_captured; 
static uint16_t DutyCycle_captured;
static uint16_t Tog_Edge_Detect;
volatile float CDE_Meas_DutyC;// tratar de utilizar int siempre .
static float temporal_1;// variable temporal global no es temporal.
volatile int DutyC_percen;
volatile float CDE_Meas_Freq;
volatile int Freq_hz;
volatile int Output_PWM;
volatile int ETAT_Duty_C;
volatile unsigned int Failure_Report_Timer;
int DC_ETAT_Out;
volatile unsigned int Fail_Confirmation_Timer_1;
int failure_flag_1;// cambiar por bool = typedef unsigned char => boolean
volatile unsigned int Fail_Confirmation_Timer_2;
int failure_flag_2;
volatile unsigned int Fail_Confirmation_Timer_3;
int failure_flag_3;
volatile unsigned int Fail_Confirmation_Timer_4;
int failure_flag_4;
volatile unsigned int Fail_Confirmation_Timer_5;
int failure_flag_5;
/*******************************************************************************
**                             main          CMSIS_Irq_Dis();                 **
*******************************************************************************/

int main(void)
{
  /** Variables initialization */

  timer_low_count = 0;
  timer_high_count = 0;
  Period_ticks = 0;
  PWM_control_var = 0;
  ETAT_PWM_Status = 0;
  Global_counter_V = 0;
  Tog_Edge_Detect = 0;
  CDE_Meas_DutyC = 0;
  CDE_Meas_Freq = 0;
  ETAT_Duty_C = 11u;
  Failure_Report_Timer = 0;
  DC_ETAT_Out = 11;
  Fail_Confirmation_Timer_1 = 0; 
  failure_flag_1 = 0;
  Fail_Confirmation_Timer_2 = 0; 
  failure_flag_2 = 0;
  Fail_Confirmation_Timer_3 = 0; 
  failure_flag_3 = 0;
  Fail_Confirmation_Timer_4 = 0; 
  failure_flag_4 = 0;
  Fail_Confirmation_Timer_5 = 0; 
  failure_flag_5 = 0;


  /*Start the MCU modules */ 
  TLE_Init();
  /* Start Timer2 */
  TIMER2_Clk_Prescaler_En();
  TIMER2_Clk_Prescaler_Sel(TIMER2x_Clk_Div_64);/**< Timer2x Input Clock Select: fsys/64 */
  TIMER2_ExternalCtrl_En();
  TIMER2_Mode_Capture_Set();
  TIMER2_ExtStart();
  TIMER2_ExtStart_FallingEdge_Set();
  /* Start GPIOS */
  GPIO_INT();/**Initialization and configuration of GPIOS */
  for (;;)
  {  
    (void)WDT1_Service();/** Main watchdog1 (WDT1) service */        
    CDE_PWM_Measurement();
    ETAT_PWM();
  }
}

/*******************************************************************************
**                             Function deployment.                           **
*******************************************************************************/
extern void Systick_IRQ_subroutine(void)
{  

  Global_counter_V++;

  if (Failure_Report_Timer > 0)
  {
    Failure_Report_Timer--;
  }

  if (Fail_Confirmation_Timer_1 > 0)
  {
    Fail_Confirmation_Timer_1--;
  }

  if (Fail_Confirmation_Timer_2 > 0)
  {
    Fail_Confirmation_Timer_2--;
  }

  if (Fail_Confirmation_Timer_3 > 0)
  {
    Fail_Confirmation_Timer_3--;
  }

  if (Fail_Confirmation_Timer_4 > 0)
  {
    Fail_Confirmation_Timer_4--;
  }

  if (Fail_Confirmation_Timer_5 > 0)
  {
    Fail_Confirmation_Timer_5--;
  }
}

extern void Timer2_IRQ_subroutine(void)
{  
  if (TIMER2->T2CON.bit.TR2 == 1u)
  {
    if (Tog_Edge_Detect == 0u)
    {
      /* Get the duty cycle */
      DutyCycle_captured = TIMER2_Get_Capture();
      TIMER2->T2MOD.bit.EDGESEL = 0u;/** Next capture on falling edge */
      Tog_Edge_Detect = 1u;
    }
    else
    {
      /* Get the period */
      Period_captured = TIMER2_Get_Capture();
      TIMER2->T2MOD.bit.EDGESEL = 1u;/** Next capture on rising edge */
      Tog_Edge_Detect = 0u;
      TIMER2_Stop();/** Stop Timer2 */
      TIMER2_Clear_Count();/** Reset Timer2 */
    }
  }
  }

void GPIO_INT(void)
{
  /** ETAT_MJP PORT CONFIGURATION */
  PORT->P0_DIR.reg = ETAT_port_P0_1;
  SCU->P0_POCON0.reg = GPIO_P0_1_AS_MEDIUM_DRIVER;
  PORT->P0_OD.reg = GPIO_P0_OPEN_DRAIN_disable;
  PORT->P0_PUDSEL.reg = GPIO_PULL_UP_PIN_P0_1_SELCTED;
  PORT->P0_PUDEN.reg = P0_1_PULL_UP_ENABLE;
   /** CDE_MJP PORT 0.2 CONFIGURATION */
  SCU->P1_POCON0.reg = GPIO_P1_2_AS_MEDIUM_DRIVER;
  SCU->P1_POCON1.reg = GPIO_P1_2_AS_MEDIUM_DRIVER;
  SCU->P1_POCON2.reg = 0x04;
  PORT->P1_DIR.reg &= CDE_port_P1_2_direction;
  /* Set input pin for Timer2 & Timer21 */
  TIMER2_Select_T2EX(TIMER2_T2EX_P12);/**Port 1.2 as external input */
}

void ETAT_PWM(void)
{
  ETAT_Duty_C = Output_PWM ;
  int ETAT_high_time;
  int ETAT_low_time;  

  ETAT_high_time = ETAT_Duty_C;//(ETAT_Freq - (ETAT_Freq - ETAT_Duty_C));
  ETAT_low_time = (ETAT_Freq - ETAT_Duty_C );

  if (PWM_control_var == 0)
  {
    if (Global_counter_V >= ETAT_high_time )
    {
      Tog_ETAT();
      Global_counter_V = 0;/**Clear the counter */
      PWM_control_var = 1; 
    } 
  }
  if (PWM_control_var == 1)
  {
    if (Global_counter_V >= ETAT_low_time )
    {
      Tog_ETAT();
      Global_counter_V = 0;/**Clear the counter flag */
      PWM_control_var = 0;
    }
  }
}

void Tog_ETAT(void)
{
  if(ETAT_PWM_Status == 1)
  {
   ETAT_HIGH;
  }
  if(ETAT_PWM_Status == 0)
  {
    ETAT_LOW;
  } 
  ETAT_PWM_Status = (ETAT_PWM_Status == 1)? 0 : 1;
}

void CDE_PWM_Measurement (void)
{
  float sys_frec;
  int temp;
  int temp1;  
  
  temp = 0;
  temp1 = 0;
  sys_frec = 0;
  
  sys_frec = MCU_Sys_Freq / Sys_CLK_Prescal;
  /*Duty Cycle capture:*/
  CDE_Meas_DutyC = (float)DutyCycle_captured/(float)Period_captured;
  temporal_1 = CDE_Meas_DutyC * 100;
  temporal_1 =  100 - temporal_1;
  temp = (int)temporal_1;
  if((temp >= (DutyC_percen + 1)) || (temp <= (DutyC_percen - 1)))
  {
    DutyC_percen = temp;
  }
  
  /*Frequency capture: */
  CDE_Meas_Freq = sys_frec /(float)Period_captured; 
  temp1 = (int)CDE_Meas_Freq;
  if((temp1 >= (Freq_hz + 1) )|| (temp1 <= (Freq_hz - 1)))
  {
    Freq_hz = temp1;
  }
  
  Output_PWM = DiagMatrix();
}

int DiagMatrix(void)
{
  if (Failure_Report_Timer <= 1)
  {
    if ((Freq_hz == 100 || Freq_hz == 99 || Freq_hz == 101 || Freq_hz == 250 || Freq_hz == 249 || Freq_hz == 251 ) && (DutyC_percen > 2 && DutyC_percen < 98 ))
    {
        DC_ETAT_Out = 11; 
    }
    else
    {
       /*Duty Cycle*/
      if (DutyC_percen < 3 )
      {
        if (failure_flag_4 == 0)
        {
          Fail_Confirmation_Timer_4 = 400;
          failure_flag_4 = 1;
        }
        
        if (Fail_Confirmation_Timer_4 <= 1 && failure_flag_4 == 1)
        {
          DC_ETAT_Out = 65; /** */
          failure_flag_4 = 0;
          Failure_Report_Timer = 5000;
        }
      }
    
      if (DutyC_percen > 97)
      {
        if (failure_flag_5 == 0)
        {
          Fail_Confirmation_Timer_5 = 400;
          failure_flag_5 = 1;
        }
        
        if (Fail_Confirmation_Timer_5 <= 1 && failure_flag_5 == 1)
        {
          DC_ETAT_Out = 70; /** */
          failure_flag_5 = 0;
          Failure_Report_Timer = 5000;
        }
      }
      /*Frequency*/
      if (Freq_hz <= 98)
      {
        if (failure_flag_1 == 0)
        {
          Fail_Confirmation_Timer_1 = 400;
          failure_flag_1 = 1;
        }
        
        if (Fail_Confirmation_Timer_1 <= 1 && failure_flag_1 == 1)
        {
          DC_ETAT_Out = 75; /** Frequency Under the threshold */
          failure_flag_1 = 0;
          Failure_Report_Timer = 5000;
        }
      }
      
      if (Freq_hz >= 102 && Freq_hz <= 248)
      {
        if (failure_flag_2 == 0)
        {
          Fail_Confirmation_Timer_2 = 400;
          failure_flag_2 = 1;
        }
        
        if (Fail_Confirmation_Timer_2 <= 1 && failure_flag_2 == 1)
        {
          DC_ETAT_Out = 80; /** Frequency out threshold between the two operative frequencies */ 
          failure_flag_2 = 0;
          Failure_Report_Timer = 5000;
        }
      }

      if (Freq_hz >= 252)
      {
        if (failure_flag_3 == 0)
        {
          Fail_Confirmation_Timer_3 = 400;
          failure_flag_3 = 1;
        }
        
        if (Fail_Confirmation_Timer_3 <= 1 && failure_flag_3 == 1)
        {
          DC_ETAT_Out = 85; /** Frequency Above threshold */
          failure_flag_3 = 0;
          Failure_Report_Timer = 5000;
        }
      }
    }
  }
  
  return DC_ETAT_Out;
}
