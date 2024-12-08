/*
 ***********************************************************************************************************************
 *
 * Copyright (c) Infineon Technologies AG
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,are permitted provided that the
 * following conditions are met:
 *
 *   Redistributions of source code must retain the above copyright notice, this list of conditions and the  following
 *   disclaimer.
 *
 *   Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
 *   following disclaimer in the documentation and/or other materials provided with the distribution.
 *
 *   Neither the name of the copyright holders nor the names of its contributors may be used to endorse or promote
 *   products derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE  FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY,OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT  OF THE
 * USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 **********************************************************************************************************************/
/**
 * \file     timer2x.h
 *
 * \brief    Timer2/Timer21 low level access library
 *
 * \version  V0.2.3
 * \date     18. Oct 2022
 */

/*******************************************************************************
**                             Author(s) Identity                             **
********************************************************************************
** Initials     Name                                                          **
** ---------------------------------------------------------------------------**
** DM           Daniel Mysliwitz                                              **
** DCM          Dragos C. Molocea                                             **
** BG           Blandine Guillot                                              **
** JO           Julia Ott                                                     **
** TA           Thomas Albersinger                                            **
** VO           Vanessa Ongaro                                                **
*******************************************************************************/

/*******************************************************************************
**                          Revision Control History                          **
********************************************************************************
** V0.1.0: 2014-05-13, TA:   Initial version                                  **
** V0.1.1: 2014-11-02, DM:   Timer values init added                          **
** V0.1.2: 2015-02-10, DM:   Individual header file added                     **
** V0.1.3: 2017-02-16, DM:   Timer2/Timer21 prefix changed to capital letters **
**                           Macros for Timer2x bit accesses added            **
**                           Set_Reload function added                        **
** V0.1.4: 2017-05-26, DM:   API extended                                     **
** V0.1.5: 2017-10-19, DM:   MISRA 2012 compliance, the following PC-Lint     **
**                           rules are globally deactivated:                  **
**                           - Info 793: ANSI/ISO limit of 6 'significant     **
**                             characters in an external identifier           **
**                           - Info 835: A zero has been given as right       **
**                             argument to operator                           **
**                           - Info 845: The left argument to operator '&'    **
**                             is certain to be 0                             **
**                           Replaced macros by INLINE functions              **
**                           Replaced register accesses within functions by   **
**                           function calls                                   **
**                           Replaced __STATIC_INLINE by INLINE               **
** V0.1.6: 2018-06-27, DCM:  Added handling in case of timer_interval_us value**
**                           in invalid range                                 **
** V0.1.7: 2018-12-03, JO:   Doxygen update                                   **
**                           Move revision history from timer2x.c to timer2x.h**
**                           Replaced __STATIC_INLINE by INLINE               **
** V0.1.8: 2020-04-15, BG:   Updated revision history format                  **
** V0.1.9: 2020-07-13, JO:   EP-431: remove ARMCC v6 Compiler warnings        **
**                           - Added typecasts to remove 'implicit typecast'  **
**                             warning                                        **
** V0.2.0: 2020-07-21, BG:   EP-439: Formatted .h/.c files                    **
** V0.2.1: 2021-04-23, BG:   EP-726: Corrected MISRA 2012 violations          **
** V0.2.2: 2022-02-28, JO:   EP-936: Updated copyright and branding           **
** V0.2.3: 2022-10-18, VO:   EP-1252: Removed ARMCC V6.18 warnings            **
*******************************************************************************/

#ifndef TIMER2X_H
#define TIMER2X_H

/*******************************************************************************
**                                  Includes                                  **
*******************************************************************************/
#include "tle987x.h"
#include "types.h"
#include "sfr_access.h"

/*******************************************************************************
**                           Global Type Definitions                          **
*******************************************************************************/
/** \enum TTIMER2_T2IN_Pxx
 *  \brief This enum lists the T2IN Inputs.
 */
typedef enum TIMER2_T2IN_Pxx
{
  TIMER2_T2IN_P00 = 0u, /**< The input pin for T2_0 */
  TIMER2_T2IN_P25 = 1u  /**< The input pin for T2_1 */
} TTIMER2_T2IN_Pxx;

/** \enum TTIMER2_T2EX_Pxx
 *  \brief This enum lists the T2EX Inputs.
 */
typedef enum TIMER2_T2EX_Pxx
{
  TIMER2_T2EX_LIN = 0u,      /**< The input pin for T2EX_0 */
  TIMER2_T2EX_P12 = 1u,      /**< The input pin for T2EX_1 */
  TIMER2_T2EX_CCU6_CC60 = 2u /**< The input pin for t2_ext_trigger */
} TTIMER2_T2EX_Pxx;

/** \enum TTIMER21_T2IN_Pxx
 *  \brief This enum lists the T21IN Inputs.
 */
typedef enum TIMER21_T2IN_Pxx
{
  TIMER21_T2IN_P01 = 0u, /**< The input pin for T21_0 */
  TIMER21_T2IN_P11 = 1u, /**< The input pin for T21_1 */
  TIMER21_T2IN_P04 = 2u  /**< The input pin for T21_2 */
} TTIMER21_T2IN_Pxx;

/** \enum TTIMER21_T2EX_Pxx
 *  \brief This enum lists the T21EX Inputs.
 */
typedef enum TIMER21_T2EX_Pxx
{
  TIMER21_T2EX_P02 = 0u,    /**< The input pin for T21EX_0 */
  TIMER21_T2EX_P14 = 1u,        /**< The input pin for T21EX_1 */
  TIMER21_T2EX_P23 = 2u,        /**< The input pin for T21EX_2 */
  TIMER21_T2EX_P12 = 3u,        /**< The input pin for T21EX_3 */
  TIMER21_T2EX_CCU6_COUT60 = 4u /**< The input pin for t21_ext_trigger */
} TTIMER21_T2EX_Pxx;

/** \enum TIMER2x_Clock_Prescaler
 *  \brief This enum lists TIMER2x Clock Prescaler Configuration.
 */
typedef enum IMER2x_Clock_Prescaler
{
  TIMER2x_Clk_Div_1   = 0u, /**< Timer2x Input Clock Select: fsys */
  TIMER2x_Clk_Div_2   = 1u, /**< Timer2x Input Clock Select: fsys/2 */
  TIMER2x_Clk_Div_4   = 2u, /**< Timer2x Input Clock Select: fsys/4 */
  TIMER2x_Clk_Div_8   = 3u, /**< Timer2x Input Clock Select: fsys/8 */
  TIMER2x_Clk_Div_16  = 4u, /**< Timer2x Input Clock Select: fsys/16 */
  TIMER2x_Clk_Div_32  = 5u, /**< Timer2x Input Clock Select: fsys/32 */
  TIMER2x_Clk_Div_64  = 6u, /**< Timer2x Input Clock Select: fsys/64 */
  TIMER2x_Clk_Div_128 = 7u  /**< Timer2x Input Clock Select: fsys/128 */
} TIMER2x_Clock_Prescaler;

/*******************************************************************************
**                          Global Macro Definitions                          **
*******************************************************************************/
#define MIN_TIMER_INTERVAL_US         1u
#define T2_REG_MAX_UPCOUNT_TICKS      0u
#define T2_REG_MIN_UPCOUNT_TICKS      0xFFFFu


/*******************************************************************************
**                     Global Inline Function Definitions                     **
*******************************************************************************/
/* TIMER2 */
/** \brief enables TIMER2 clock prescaler.
 *
 * \brief <b>Example</b><br>
 * \brief This example enables TIMER2 clock prescaler.
 * ~~~~~~~~~~~~~~~{.c}
 * void Example_Function(void)
 * {
 *     TIMER2_Clk_Prescaler_En();
 * }
 * ~~~~~~~~~~~~~~~
 * \ingroup TIMER2x_api
 */
INLINE void TIMER2_Clk_Prescaler_En(void)
{
  Field_Mod8(&TIMER2->T2MOD.reg, (uint8)TIMER2_T2MOD_PREN_Pos, (uint8)TIMER2_T2MOD_PREN_Msk, 1u);
}

/** \brief disables TIMER2 clock prescaler.
 *
 * \brief <b>Example</b><br>
 * \brief This example disables TIMER2 clock prescaler.
 * ~~~~~~~~~~~~~~~{.c}
 * void Example_Function(void)
 * {
 *     TIMER2_Clk_Prescaler_Dis();
 * }
 * ~~~~~~~~~~~~~~~
 * \ingroup TIMER2x_api
 */
INLINE void TIMER2_Clk_Prescaler_Dis(void)
{
  Field_Mod8(&TIMER2->T2MOD.reg, (uint8)TIMER2_T2MOD_PREN_Pos, (uint8)TIMER2_T2MOD_PREN_Msk, 0u);
}

/** \brief sets TIMER2 clock prescaler.
 *
 * \brief <b>Example</b><br>
 * \brief This example sets TIMER2 clock prescaler to DIV2.
 * ~~~~~~~~~~~~~~~{.c}
 * void Example_Function(void)
 * {
 *     TIMER2_Clk_Prescaler_En();
 *     TIMER2_Clk_Prescaler_Sel(1u);
 * }
 * ~~~~~~~~~~~~~~~
 * \ingroup TIMER2x_api
 */
INLINE void TIMER2_Clk_Prescaler_Sel(uint8 t2pre)
{
  Field_Mod8(&TIMER2->T2MOD.reg, (uint8)TIMER2_T2MOD_T2PRE_Pos, (uint8)TIMER2_T2MOD_T2PRE_Msk, t2pre);
}

/** \brief reads TIMER2 clock prescaler.
 *
 * \return TIMER2 clock prescaler
 *
 * \brief <b>Example</b><br>
 * \brief This example reads TIMER2 clock prescaler.
 * ~~~~~~~~~~~~~~~{.c}
 * void Example_Function(void)
 * {
 *     uint8 pre;
 *
 *     TIMER2_Clk_Prescaler_En();
 *     pre = TIMER2_Clk_Prescaler_Get();
 * }
 * ~~~~~~~~~~~~~~~
 * \ingroup TIMER2x_api
 */
INLINE uint8 TIMER2_Clk_Prescaler_Get(void)
{
  return ( u8_Field_Rd8(&TIMER2->T2MOD.reg, (uint8)TIMER2_T2MOD_T2PRE_Pos, (uint8)TIMER2_T2MOD_T2PRE_Msk) );
}

/** \brief reads TIMER2 Overflow Status.
 *
 * \return TIMER2 Overflow Status
 *
 * \brief <b>Example</b><br>
 * \brief This example checks the TIMER2 Overflow Status.
 * ~~~~~~~~~~~~~~~{.c}
 * void Example_Function(void)
 * {
 *     TIMER2_Overflow_Int_En();
 *     if (TIMER2_Overflow_Sts() == (uint8)1)
 *     {
 *       TIMER2_TF2_CALLBACK();
 *     }
 *     TIMER2_Overflow_Int_Clr();
 *     TIMER2_Overflow_Int_Dis();
 * }
 * ~~~~~~~~~~~~~~~
 * \ingroup TIMER2x_api
 */
INLINE uint8 TIMER2_Overflow_Sts(void)
{
  return ( u1_Field_Rd8(&TIMER2->T2CON.reg, (uint8)TIMER2_T2CON_TF2_Pos, (uint8)TIMER2_T2CON_TF2_Msk) );
}

/** \brief reads TIMER2 External Status.
 *
 * \return TIMER2 External Status
 *
 * \brief <b>Example</b><br>
 * \brief This example checks the TIMER2 External Status.
 * ~~~~~~~~~~~~~~~{.c}
 * void Example_Function(void)
 * {
 *     TIMER2_ExternalCtrl_En();
 *     if (TIMER2_External_Sts() == (uint8)1)
 *     {
 *       TIMER2_EXF2_CALLBACK();
 *     }
 *     TIMER2_ExternalCtrl_Dis();
 * }
 * ~~~~~~~~~~~~~~~
 * \ingroup TIMER2x_api
 */
INLINE uint8 TIMER2_External_Sts(void)
{
  return ( u1_Field_Rd8(&TIMER2->T2CON.reg, (uint8)TIMER2_T2CON_EXF2_Pos, (uint8)TIMER2_T2CON_EXF2_Msk) );
}

/** \brief enables TIMER2 External Control.
 *
 * \brief <b>Example</b><br>
 * \brief This example checks the TIMER2 External Status.
 * ~~~~~~~~~~~~~~~{.c}
 * void Example_Function(void)
 * {
 *     TIMER2_External_Int_En();
 *     TIMER2_ExternalCtrl_En();
 *     if (TIMER2_External_Sts() == (uint8)1)
 *     {
 *       TIMER2_EXF2_CALLBACK();
 *     }
 *     TIMER2_External_Int_Clr();
 *     TIMER2_ExternalCtrl_Dis();
 *     TIMER2_External_Int_Dis();
 * }
 * ~~~~~~~~~~~~~~~
 * \ingroup TIMER2x_api
 */
INLINE void TIMER2_ExternalCtrl_En(void)
{
  Field_Mod8(&TIMER2->T2CON.reg, (uint8)TIMER2_T2CON_EXEN2_Pos, (uint8)TIMER2_T2CON_EXEN2_Msk, 1u);
}

/** \brief disables TIMER2 External Control.
 *
 * \brief <b>Example</b><br>
 * \brief This example reads TIMER2 External Status while External control is disabled.
 * ~~~~~~~~~~~~~~~{.c}
 * void Example_Function(void)
 * {
 *     uint8 sts;
 *
 *     TIMER2_ExternalCtrl_En();
 *     sts = TIMER2_External_Sts();
 *     TIMER2_ExternalCtrl_Dis();
 * }
 * ~~~~~~~~~~~~~~~
 * \ingroup TIMER2x_api
 */
INLINE void TIMER2_ExternalCtrl_Dis(void)
{
  Field_Mod8(&TIMER2->T2CON.reg, (uint8)TIMER2_T2CON_EXEN2_Pos, (uint8)TIMER2_T2CON_EXEN2_Msk, 0u);
}

/** \brief sets Rising Edge to Capture/Reload Mode for TIMER2.
 *
 * \brief <b>Example</b><br>
 * \brief This example sets Rising Edge to Capture/Reload Mode.
 * ~~~~~~~~~~~~~~~{.c}
 * void Example_Function(void)
 * {
 *     TIMER2_Reload_Capture_RisingEdge_Set();
 * }
 * ~~~~~~~~~~~~~~~
 * \ingroup TIMER2x_api
 */
INLINE void TIMER2_Reload_Capture_RisingEdge_Set(void)
{
  Field_Mod8(&TIMER2->T2MOD.reg, (uint8)TIMER2_T2MOD_EDGESEL_Pos, (uint8)TIMER2_T2MOD_EDGESEL_Msk, 1u);
}

/** \brief sets Falling Edge to Capture/Reload Mode for TIMER2.
 *
 * \brief <b>Example</b><br>
 * \brief This example sets Falling Edge to Capture/Reload Mode.
 * ~~~~~~~~~~~~~~~{.c}
 * void Example_Function(void)
 * {
 *     TIMER2_Reload_Capture_FallingEdge_Set();
 * }
 * ~~~~~~~~~~~~~~~
 * \ingroup TIMER2x_api
 */
INLINE void TIMER2_Reload_Capture_FallingEdge_Set(void)
{
  Field_Mod8(&TIMER2->T2MOD.reg, (uint8)TIMER2_T2MOD_EDGESEL_Pos, (uint8)TIMER2_T2MOD_EDGESEL_Msk, 0u);
}

/** \brief sets Rising Edge to External Start for TIMER2.
 *
 * \brief <b>Example</b><br>
 * \brief This example sets Rising Edge to External Start.
 * ~~~~~~~~~~~~~~~{.c}
 * void Example_Function(void)
 * {
 *     TIMER2_ExtStart();
 *     TIMER2_ExtStart_RisingEdge_Set();
 *     TIMER2_ExtStop();
 * }
 * ~~~~~~~~~~~~~~~
 * \ingroup TIMER2x_api
 */
INLINE void TIMER2_ExtStart_RisingEdge_Set(void)
{
  Field_Mod8(&TIMER2->T2MOD.reg, (uint8)TIMER2_T2MOD_T2REGS_Pos, (uint8)TIMER2_T2MOD_T2REGS_Msk, 1u);
}

/** \brief sets Falling Edge to External Start for TIMER2.
 *
 * \brief <b>Example</b><br>
 * \brief This example sets Falling Edge to External Start.
 * ~~~~~~~~~~~~~~~{.c}
 * void Example_Function(void)
 * {
 *     TIMER2_ExtStart();
 *     TIMER2_ExtStart_FallingEdge_Set();
 *     TIMER2_ExtStop();
 * }
 * ~~~~~~~~~~~~~~~
 * \ingroup TIMER2x_api
 */
INLINE void TIMER2_ExtStart_FallingEdge_Set(void)
{
  Field_Mod8(&TIMER2->T2MOD.reg, (uint8)TIMER2_T2MOD_T2REGS_Pos, (uint8)TIMER2_T2MOD_T2REGS_Msk, 0u);
}

/** \brief enables Up/Down Counter for TIMER2.
 *
 * \brief <b>Example</b><br>
 * \brief This example enables Up/Down Counter for TIMER2.
 * ~~~~~~~~~~~~~~~{.c}
 * void Example_Function(void)
 * {
 *     TIMER2_UpDownCount_En();
 * }
 * ~~~~~~~~~~~~~~~
 * \ingroup TIMER2x_api
 */
INLINE void TIMER2_UpDownCount_En(void)
{
  Field_Mod8(&TIMER2->T2MOD.reg, (uint8)TIMER2_T2MOD_DCEN_Pos, (uint8)TIMER2_T2MOD_DCEN_Msk, 1u);
}

/** \brief Disables Up/Down Counter for TIMER2.
 *
 * \brief <b>Example</b><br>
 * \brief This example starts Up Counter once TR2 is set and External Control was disabled.
 * ~~~~~~~~~~~~~~~{.c}
 * void Example_Function(void)
 * {
 *     TIMER2_ExternalCtrl_Dis();
 *     TIMER2_UpDownCount_Dis();
 *     TIMER2_Start();
 * }
 * ~~~~~~~~~~~~~~~
 * \ingroup TIMER2x_api
 */
INLINE void TIMER2_UpDownCount_Dis(void)
{
  Field_Mod8(&TIMER2->T2MOD.reg, (uint8)TIMER2_T2MOD_DCEN_Pos, (uint8)TIMER2_T2MOD_DCEN_Msk, 0u);
}

/** \brief Selects Capture Mode for TIMER2.
 *
 * \brief <b>Example</b><br>
 * \brief This example enters 16-bit Capture Mode.
 * ~~~~~~~~~~~~~~~{.c}
 * void Example_Function(void)
 * {
 *     TIMER2_ExternalCtrl_En();
 *     TIMER2_Mode_Capture_Set();
 * }
 * ~~~~~~~~~~~~~~~
 * \ingroup TIMER2x_api
 */
INLINE void TIMER2_Mode_Capture_Set(void)
{
  Field_Mod8(&TIMER2->T2CON.reg, (uint8)TIMER2_T2CON_CP_RL2_Pos, (uint8)TIMER2_T2CON_CP_RL2_Msk, 1u);
}

/** \brief Selects Reload Mode for TIMER2.
 *
 * \brief <b>Example</b><br>
 * \brief This example enters Auto-Reload Mode.
 * ~~~~~~~~~~~~~~~{.c}
 * void Example_Function(void)
 * {
 *     TIMER2_Mode_Reload_Set();
 * }
 * ~~~~~~~~~~~~~~~
 * \ingroup TIMER2x_api
 */
INLINE void TIMER2_Mode_Reload_Set(void)
{
  Field_Mod8(&TIMER2->T2CON.reg, (uint8)TIMER2_T2CON_CP_RL2_Pos, (uint8)TIMER2_T2CON_CP_RL2_Msk, 0u);
}

/** \brief Selects Mode Counter for TIMER2.
 *
 * \brief <b>Example</b><br>
 * \brief This example sets the Mode Counter.
 * ~~~~~~~~~~~~~~~{.c}
 * void Example_Function(void)
 * {
 *     TIMER2_Mode_Counter_Set();
 * }
 * ~~~~~~~~~~~~~~~
 * \ingroup TIMER2x_api
 */
INLINE void TIMER2_Mode_Counter_Set(void)
{
  Field_Mod8(&TIMER2->T2CON.reg, (uint8)TIMER2_T2CON_C_T2_Pos, (uint8)TIMER2_T2CON_C_T2_Msk, 1u);
}

/** \brief Selects Mode Timer for TIMER2.
 *
 * \brief <b>Example</b><br>
 * \brief This example sets the Mode Timer.
 * ~~~~~~~~~~~~~~~{.c}
 * void Example_Function(void)
 * {
 *     TIMER2_Mode_Timer_Set();
 * }
 * ~~~~~~~~~~~~~~~
 * \ingroup TIMER2x_api
 */
INLINE void TIMER2_Mode_Timer_Set(void)
{
  Field_Mod8(&TIMER2->T2CON.reg, (uint8)TIMER2_T2CON_C_T2_Pos, (uint8)TIMER2_T2CON_C_T2_Msk, 0u);
}

/** \brief Clears TIMER2 Overflow Interrupt flag.
 *
 * \brief <b>Example</b><br>
 * \brief This example treats the TIMER2 Overflow Interrupt.
 * ~~~~~~~~~~~~~~~{.c}
 * void Example_Function(void)
 * {
 *     TIMER2_Overflow_Int_En();
 *     if (TIMER2_Overflow_Sts() == (uint8)1)
 *     {
 *       TIMER2_TF2_CALLBACK();
 *       TIMER2_Overflow_Int_Clr();
 *     }
 *     TIMER2_Overflow_Int_Dis();
 * }
 * ~~~~~~~~~~~~~~~
 * \ingroup TIMER2x_api
 */
INLINE void TIMER2_Overflow_Int_Clr(void)
{
  Field_Wrt8(&TIMER2->T2ICLR.reg, (uint8)TIMER2_T2ICLR_TF2CLR_Pos, (uint8)TIMER2_T2ICLR_TF2CLR_Msk, 1u);
}

/** \brief Clears TIMER2 External Interrupt flag.
 *
 * \brief <b>Example</b><br>
 * \brief This example treats the TIMER2 External Interrupt.
 * ~~~~~~~~~~~~~~~{.c}
 * void Example_Function(void)
 * {
 *     TIMER2_External_Int_En();
 *     if (TIMER2_External_Sts() == 1)
 *     {
 *       TIMER2_EXF2_CALLBACK();
 *       TIMER2_External_Int_Clr();
 *     }
 *     TIMER2_External_Int_Dis();
 * }
 * ~~~~~~~~~~~~~~~
 * \ingroup TIMER2x_api
 */
INLINE void TIMER2_External_Int_Clr(void)
{
  Field_Wrt8(&TIMER2->T2ICLR.reg, (uint8)TIMER2_T2ICLR_EXF2CLR_Pos, (uint8)TIMER2_T2ICLR_EXF2CLR_Msk, 1u);
}

/** \brief Enables TIMER2 Overflow Interrupt.
 *
 * \brief <b>Example</b><br>
 * \brief This example treats the TIMER2 Overflow Interrupt.
 * ~~~~~~~~~~~~~~~{.c}
 * void Example_Function(void)
 * {
 *     TIMER2_Overflow_Int_En();
 *     if (TIMER2_Overflow_Sts() == (uint8)1)
 *     {
 *       TIMER2_TF2_CALLBACK();
 *       TIMER2_Overflow_Int_Clr();
 *     }
 *     TIMER2_Overflow_Int_Dis();
 * }
 * ~~~~~~~~~~~~~~~
 * \ingroup TIMER2x_api
 */
INLINE void TIMER2_Overflow_Int_En(void)
{
  Field_Mod8(&TIMER2->T2CON1.reg, (uint8)TIMER2_T2CON1_TF2EN_Pos, (uint8)TIMER2_T2CON1_TF2EN_Msk, 1u);
}

/** \brief Disables TIMER2 Overflow Interrupt.
 *
 * \brief <b>Example</b><br>
 * \brief This example treats the TIMER2 Overflow Interrupt.
 * ~~~~~~~~~~~~~~~{.c}
 * void Example_Function(void)
 * {
 *     TIMER2_Overflow_Int_En();
 *     if (TIMER2_Overflow_Sts() == (uint8)1)
 *     {
 *       TIMER2_TF2_CALLBACK();
 *       TIMER2_Overflow_Int_Clr();
 *     }
 *     TIMER2_Overflow_Int_Dis();
 * }
 * ~~~~~~~~~~~~~~~
 * \ingroup TIMER2x_api
 */
INLINE void TIMER2_Overflow_Int_Dis(void)
{
  Field_Mod8(&TIMER2->T2CON1.reg, (uint8)TIMER2_T2CON1_TF2EN_Pos, (uint8)TIMER2_T2CON1_TF2EN_Msk, 0u);
}

/** \brief Enables TIMER2 External Interrupt.
 *
 * \brief <b>Example</b><br>
 * \brief This example treats the TIMER2 External Interrupt.
 * ~~~~~~~~~~~~~~~{.c}
 * void Example_Function(void)
 * {
 *     TIMER2_External_Int_En();
 *     if (TIMER2_External_Sts() == 1)
 *     {
 *       TIMER2_EXF2_CALLBACK();
 *       TIMER2_External_Int_Clr();
 *     }
 *     TIMER2_External_Int_Dis();
 * }
 * ~~~~~~~~~~~~~~~
 * \ingroup TIMER2x_api
 */
INLINE void TIMER2_External_Int_En(void)
{
  Field_Mod8(&TIMER2->T2CON1.reg, (uint8)TIMER2_T2CON1_EXF2EN_Pos, (uint8)TIMER2_T2CON1_EXF2EN_Msk, 1u);
}

/** \brief Disables TIMER2 External Interrupt.
 *
 * \brief <b>Example</b><br>
 * \brief This example treats the TIMER2 External Interrupt.
 * ~~~~~~~~~~~~~~~{.c}
 * void Example_Function(void)
 * {
 *     TIMER2_External_Int_En();
 *     if (TIMER2_External_Sts() == 1)
 *     {
 *       TIMER2_EXF2_CALLBACK();
 *       TIMER2_External_Int_Clr();
 *     }
 *     TIMER2_External_Int_Dis();
 * }
 * ~~~~~~~~~~~~~~~
 * \ingroup TIMER2x_api
 */
INLINE void TIMER2_External_Int_Dis(void)
{
  Field_Mod8(&TIMER2->T2CON1.reg, (uint8)TIMER2_T2CON1_EXF2EN_Pos, (uint8)TIMER2_T2CON1_EXF2EN_Msk, 0u);
}

/** \brief enables TIMER21 clock prescaler.
 *
 * \brief <b>Example</b><br>
 * \brief This example enables TIMER21 clock prescaler.
 * ~~~~~~~~~~~~~~~{.c}
 * void Example_Function(void)
 * {
 *     TIMER21_Clk_Prescaler_En();
 * }
 * ~~~~~~~~~~~~~~~
 * \ingroup TIMER2x_api
 */
INLINE void TIMER21_Clk_Prescaler_En(void)
{
  Field_Mod8(&TIMER21->T2MOD.reg, (uint8)TIMER21_T2MOD_PREN_Pos, (uint8)TIMER21_T2MOD_PREN_Msk, 1u);
}

/** \brief disables TIMER21 clock prescaler.
 *
 * \brief <b>Example</b><br>
 * \brief This example disables TIMER21 clock prescaler.
 * ~~~~~~~~~~~~~~~{.c}
 * void Example_Function(void)
 * {
 *     TIMER21_Clk_Prescaler_Dis();
 * }
 * ~~~~~~~~~~~~~~~
 * \ingroup TIMER2x_api
 */
INLINE void TIMER21_Clk_Prescaler_Dis(void)
{
  Field_Mod8(&TIMER21->T2MOD.reg, (uint8)TIMER21_T2MOD_PREN_Pos, (uint8)TIMER21_T2MOD_PREN_Msk, 0u);
}

/** \brief sets TIMER21 clock prescaler.
 *
 * \param t2pre TIMER21 clock prescaler
 *
 * \brief <b>Example</b><br>
 * \brief This example sets TIMER21 clock prescaler to DIV2.
 * ~~~~~~~~~~~~~~~{.c}
 * void Example_Function(void)
 * {
 *     TIMER21_Clk_Prescaler_En();
 *     TIMER21_Clk_Prescaler_Sel(1u);
 * }
 * ~~~~~~~~~~~~~~~
 * \ingroup TIMER2x_api
 */
INLINE void TIMER21_Clk_Prescaler_Sel(uint8 t2pre)
{
  Field_Mod8(&TIMER21->T2MOD.reg, (uint8)TIMER21_T2MOD_T2PRE_Pos, (uint8)TIMER21_T2MOD_T2PRE_Msk, t2pre);
}

/** \brief reads TIMER21 clock prescaler.
 *
 * \return TIMER21 clock prescaler
 *
 * \brief <b>Example</b><br>
 * \brief This example reads TIMER21 clock prescaler.
 * ~~~~~~~~~~~~~~~{.c}
 * void Example_Function(void)
 * {
 *     uint8 pre;
 *
 *     TIMER21_Clk_Prescaler_En();
 *     pre = TIMER21_Clk_Prescaler_Get();
 * }
 * ~~~~~~~~~~~~~~~
 * \ingroup TIMER2x_api
 */
INLINE uint8 TIMER21_Clk_Prescaler_Get(void)
{
  return ( u8_Field_Rd8(&TIMER21->T2MOD.reg, (uint8)TIMER21_T2MOD_T2PRE_Pos, (uint8)TIMER21_T2MOD_T2PRE_Msk) );
}

/** \brief reads TIMER21 Overflow Status.
 *
 * \return TIMER21 Overflow Status
 *
 * \brief <b>Example</b><br>
 * \brief This example checks the TIMER21 Overflow Status.
 * ~~~~~~~~~~~~~~~{.c}
 * void Example_Function(void)
 * {
 *     TIMER21_Overflow_Int_En();
 *     if (TIMER21_Overflow_Sts() == (uint8)1)
 *     {
 *       TIMER21_TF2_CALLBACK();
 *     }
 *     TIMER21_Overflow_Int_Clr();
 *     TIMER21_Overflow_Int_Dis();
 * }
 * ~~~~~~~~~~~~~~~
 * \ingroup TIMER2x_api
 */
INLINE uint8 TIMER21_Overflow_Sts(void)
{
  return ( u1_Field_Rd8(&TIMER21->T2CON.reg, (uint8)TIMER21_T2CON_TF2_Pos, (uint8)TIMER21_T2CON_TF2_Msk) );
}

/** \brief reads TIMER21 External Status.
 *
 * \return TIMER21 External Status
 *
 * \brief <b>Example</b><br>
 * \brief This example checks the TIMER21 External Status.
 * ~~~~~~~~~~~~~~~{.c}
 * void Example_Function(void)
 * {
 *     TIMER21_ExternalCtrl_En();
 *     if (TIMER21_External_Sts() == (uint8)1)
 *     {
 *       TIMER21_EXF2_CALLBACK();
 *     }
 *     TIMER21_ExternalCtrl_Dis();
 * }
 * ~~~~~~~~~~~~~~~
 * \ingroup TIMER2x_api
 */
INLINE uint8 TIMER21_External_Sts(void)
{
  return ( u1_Field_Rd8(&TIMER21->T2CON.reg, (uint8)TIMER21_T2CON_EXF2_Pos, (uint8)TIMER21_T2CON_EXF2_Msk) );
}

/** \brief enables TIMER21 External Control.
 *
 * \brief <b>Example</b><br>
 * \brief This example checks the TIMER21 External Status.
 * ~~~~~~~~~~~~~~~{.c}
 * void Example_Function(void)
 * {
 *     TIMER21_External_Int_En();
 *     TIMER21_ExternalCtrl_En();
 *     if (TIMER21_External_Sts() == (uint8)1)
 *     {
 *       TIMER21_EXF2_CALLBACK();
 *     }
 *     TIMER21_External_Int_Clr();
 *     TIMER21_ExternalCtrl_Dis();
 *     TIMER21_External_Int_Dis();
 * }
 * ~~~~~~~~~~~~~~~
 * \ingroup TIMER2x_api
 */
INLINE void TIMER21_ExternalCtrl_En(void)
{
  Field_Mod8(&TIMER21->T2CON.reg, (uint8)TIMER21_T2CON_EXEN2_Pos, (uint8)TIMER21_T2CON_EXEN2_Msk, 1u);
}

/** \brief disables TIMER21 External Control.
 *
 * \brief <b>Example</b><br>
 * \brief This example reads TIMER21 External Status while External control is disabled.
 * ~~~~~~~~~~~~~~~{.c}
 * void Example_Function(void)
 * {
 *     uint8 sts;
 *
 *     TIMER21_ExternalCtrl_En();
 *     sts = TIMER21_External_Sts();
 *     TIMER21_ExternalCtrl_Dis();
 * }
 * ~~~~~~~~~~~~~~~
 * \ingroup TIMER2x_api
 */
INLINE void TIMER21_ExternalCtrl_Dis(void)
{
  Field_Mod8(&TIMER21->T2CON.reg, (uint8)TIMER21_T2CON_EXEN2_Pos, (uint8)TIMER21_T2CON_EXEN2_Msk, 0u);
}

/** \brief sets Rising Edge to Capture/Reload Mode for TIMER21.
 *
 * \brief <b>Example</b><br>
 * \brief This example sets Rising Edge to Capture/Reload Mode.
 * ~~~~~~~~~~~~~~~{.c}
 * void Example_Function(void)
 * {
 *     TIMER21_Reload_Capture_RisingEdge_Set();
 * }
 * ~~~~~~~~~~~~~~~
 * \ingroup TIMER2x_api
 */
INLINE void TIMER21_Reload_Capture_RisingEdge_Set(void)
{
  Field_Mod8(&TIMER21->T2MOD.reg, (uint8)TIMER21_T2MOD_EDGESEL_Pos, (uint8)TIMER21_T2MOD_EDGESEL_Msk, 1u);
}

/** \brief sets Falling Edge to Capture/Reload Mode for TIMER21.
 *
 * \brief <b>Example</b><br>
 * \brief This example sets Falling Edge to Capture/Reload Mode.
 * ~~~~~~~~~~~~~~~{.c}
 * void Example_Function(void)
 * {
 *     TIMER21_Reload_Capture_FallingEdge_Set();
 * }
 * ~~~~~~~~~~~~~~~
 * \ingroup TIMER2x_api
 */
INLINE void TIMER21_Reload_Capture_FallingEdge_Set(void)
{
  Field_Mod8(&TIMER21->T2MOD.reg, (uint8)TIMER21_T2MOD_EDGESEL_Pos, (uint8)TIMER21_T2MOD_EDGESEL_Msk, 0u);
}

/** \brief sets Rising Edge to External Start for TIMER21.
 *
 * \brief <b>Example</b><br>
 * \brief This example sets Rising Edge to External Start.
 * ~~~~~~~~~~~~~~~{.c}
 * void Example_Function(void)
 * {
 *     TIMER21_ExtStart_RisingEdge_Set();
 * }
 * ~~~~~~~~~~~~~~~
 * \ingroup TIMER2x_api
 */
INLINE void TIMER21_ExtStart_RisingEdge_Set(void)
{
  Field_Mod8(&TIMER21->T2MOD.reg, (uint8)TIMER21_T2MOD_T2REGS_Pos, (uint8)TIMER21_T2MOD_T2REGS_Msk, 1u);
}

/** \brief sets Falling Edge to External Start for TIMER21.
 *
 * \brief <b>Example</b><br>
 * \brief This example sets Falling Edge to External Start.
 * ~~~~~~~~~~~~~~~{.c}
 * void Example_Function(void)
 * {
 *     TIMER21_ExtStart_FallingEdge_Set();
 * }
 * ~~~~~~~~~~~~~~~
 * \ingroup TIMER2x_api
 */
INLINE void TIMER21_ExtStart_FallingEdge_Set(void)
{
  Field_Mod8(&TIMER21->T2MOD.reg, (uint8)TIMER21_T2MOD_T2REGS_Pos, (uint8)TIMER21_T2MOD_T2REGS_Msk, 0u);
}

/** \brief enables Up/Down Counter for TIMER21.
 *
 * \brief <b>Example</b><br>
 * \brief This example enables Up/Down Counter for TIMER21.
 * ~~~~~~~~~~~~~~~{.c}
 * void Example_Function(void)
 * {
 *     TIMER21_UpDownCount_En();
 * }
 * ~~~~~~~~~~~~~~~
 * \ingroup TIMER2x_api
 */
INLINE void TIMER21_UpDownCount_En(void)
{
  Field_Mod8(&TIMER21->T2MOD.reg, (uint8)TIMER21_T2MOD_DCEN_Pos, (uint8)TIMER21_T2MOD_DCEN_Msk, 1u);
}

/** \brief Disables Up/Down Counter for TIMER21.
 *
 * \brief <b>Example</b><br>
 * \brief This example starts Up Counter once TR2 is set and External Control was disabled.
 * ~~~~~~~~~~~~~~~{.c}
 * void Example_Function(void)
 * {
 *     TIMER21_ExternalCtrl_Dis();
 *     TIMER21_UpDownCount_Dis();
 *     TIMER21_Start();
 * }
 * ~~~~~~~~~~~~~~~
 * \ingroup TIMER2x_api
 */
INLINE void TIMER21_UpDownCount_Dis(void)
{
  Field_Mod8(&TIMER21->T2MOD.reg, (uint8)TIMER21_T2MOD_DCEN_Pos, (uint8)TIMER21_T2MOD_DCEN_Msk, 0u);
}

/** \brief Selects Capture Mode for TIMER21.
 *
 * \brief <b>Example</b><br>
 * \brief This example enters 16-bit Capture Mode.
 * ~~~~~~~~~~~~~~~{.c}
 * void Example_Function(void)
 * {
 *     TIMER21_ExternalCtrl_En();
 *     TIMER21_Mode_Capture_Set();
 * }
 * ~~~~~~~~~~~~~~~
 * \ingroup TIMER2x_api
 */
INLINE void TIMER21_Mode_Capture_Set(void)
{
  Field_Mod8(&TIMER21->T2CON.reg, (uint8)TIMER21_T2CON_CP_RL2_Pos, (uint8)TIMER21_T2CON_CP_RL2_Msk, 1u);
}

/** \brief Selects Reload Mode for TIMER21.
 *
 * \brief <b>Example</b><br>
 * \brief This example enters Auto-Reload Mode.
 * ~~~~~~~~~~~~~~~{.c}
 * void Example_Function(void)
 * {
 *     TIMER21_Mode_Reload_Set();
 * }
 * ~~~~~~~~~~~~~~~
 * \ingroup TIMER2x_api
 */
INLINE void TIMER21_Mode_Reload_Set(void)
{
  Field_Mod8(&TIMER21->T2CON.reg, (uint8)TIMER21_T2CON_CP_RL2_Pos, (uint8)TIMER21_T2CON_CP_RL2_Msk, 0u);
}

/** \brief Selects Mode Counter for TIMER21.
 *
 * \brief <b>Example</b><br>
 * \brief This example sets the Mode Counter.
 * ~~~~~~~~~~~~~~~{.c}
 * void Example_Function(void)
 * {
 *     TIMER21_Mode_Counter_Set();
 * }
 * ~~~~~~~~~~~~~~~
 * \ingroup TIMER2x_api
 */
INLINE void TIMER21_Mode_Counter_Set(void)
{
  Field_Mod8(&TIMER21->T2CON.reg, (uint8)TIMER21_T2CON_C_T2_Pos, (uint8)TIMER21_T2CON_C_T2_Msk, 1u);
}

/** \brief Selects Mode Timer for TIMER21.
 *
 * \brief <b>Example</b><br>
 * \brief This example sets the Mode Timer.
 * ~~~~~~~~~~~~~~~{.c}
 * void Example_Function(void)
 * {
 *     TIMER21_Mode_Timer_Set();
 * }
 * ~~~~~~~~~~~~~~~
 * \ingroup TIMER2x_api
 */
INLINE void TIMER21_Mode_Timer_Set(void)
{
  Field_Mod8(&TIMER21->T2CON.reg, (uint8)TIMER21_T2CON_C_T2_Pos, (uint8)TIMER21_T2CON_C_T2_Msk, 0u);
}

/** \brief Clears TIMER21 Overflow Interrupt status.
 *
 * \brief <b>Example</b><br>
 * \brief This example treats the TIMER21 Overflow Interrupt.
 * ~~~~~~~~~~~~~~~{.c}
 * void Example_Function(void)
 * {
 *     TIMER21_Overflow_Int_En();
 *     if (TIMER21_Overflow_Sts() == 1)
 *     {
 *       TIMER21_TF2_CALLBACK();
 *       TIMER21_Overflow_Int_Clr();
 *     }
 *     TIMER21_Overflow_Int_Dis();
 * }
 * ~~~~~~~~~~~~~~~
 * \ingroup TIMER2x_api
 */
INLINE void TIMER21_Overflow_Int_Clr(void)
{
  Field_Wrt8(&TIMER21->T2ICLR.reg, (uint8)TIMER21_T2ICLR_TF2CLR_Pos, (uint8)TIMER21_T2ICLR_TF2CLR_Msk, 1u);
}

/** \brief Clears TIMER21 External Interrupt status.
 *
 * \brief <b>Example</b><br>
 * \brief This example treats the TIMER21 External Interrupt.
 * ~~~~~~~~~~~~~~~{.c}
 * void Example_Function(void)
 * {
 *     TIMER21_External_Int_En();
 *     if (TIMER21_External_Sts() == 1)
 *     {
 *       TIMER21_EXF2_CALLBACK();
 *       TIMER21_External_Int_Clr();
 *     }
 *     TIMER21_External_Int_Dis();
 * }
 * ~~~~~~~~~~~~~~~
 * \ingroup TIMER2x_api
 */
INLINE void TIMER21_External_Int_Clr(void)
{
  Field_Wrt8(&TIMER21->T2ICLR.reg, (uint8)TIMER21_T2ICLR_EXF2CLR_Pos, (uint8)TIMER21_T2ICLR_EXF2CLR_Msk, 1u);
}

/** \brief Enables TIMER21 Overflow Interrupt.
 *
 * \brief <b>Example</b><br>
 * \brief This example treats the TIMER21 Overflow Interrupt.
 * ~~~~~~~~~~~~~~~{.c}
 * void Example_Function(void)
 * {
 *     TIMER21_Overflow_Int_En();
 *     if (TIMER21_Overflow_Sts() == 1)
 *     {
 *       TIMER21_TF2_CALLBACK();
 *       TIMER21_Overflow_Int_Clr();
 *     }
 *     TIMER21_Overflow_Int_Dis();
 * }
 * ~~~~~~~~~~~~~~~
 * \ingroup TIMER2x_api
 */
INLINE void TIMER21_Overflow_Int_En(void)
{
  Field_Mod8(&TIMER21->T2CON1.reg, (uint8)TIMER21_T2CON1_TF2EN_Pos, (uint8)TIMER21_T2CON1_TF2EN_Msk, 1u);
}

/** \brief Disables TIMER21 Overflow Interrupt.
 *
 * \brief <b>Example</b><br>
 * \brief This example treats the TIMER21 Overflow Interrupt.
 * ~~~~~~~~~~~~~~~{.c}
 * void Example_Function(void)
 * {
 *     TIMER21_Overflow_Int_En();
 *     if (TIMER21_Overflow_Sts() == 1)
 *     {
 *       TIMER21_TF2_CALLBACK();
 *       TIMER21_Overflow_Int_Clr();
 *     }
 *     TIMER21_Overflow_Int_Dis();
 * }
 * ~~~~~~~~~~~~~~~
 * \ingroup TIMER2x_api
 */
INLINE void TIMER21_Overflow_Int_Dis(void)
{
  Field_Mod8(&TIMER21->T2CON1.reg, (uint8)TIMER21_T2CON1_TF2EN_Pos, (uint8)TIMER21_T2CON1_TF2EN_Msk, 0u);
}

/** \brief Enables TIMER21 External Interrupt.
 *
 * \brief <b>Example</b><br>
 * \brief This example treats the TIMER21 External Interrupt.
 * ~~~~~~~~~~~~~~~{.c}
 * void Example_Function(void)
 * {
 *     TIMER21_External_Int_En();
 *     if (TIMER21_External_Sts() == 1)
 *     {
 *       TIMER21_EXF2_CALLBACK();
 *       TIMER21_External_Int_Clr();
 *     }
 *     TIMER21_External_Int_Dis();
 * }
 * ~~~~~~~~~~~~~~~
 * \ingroup TIMER2x_api
 */
INLINE void TIMER21_External_Int_En(void)
{
  Field_Mod8(&TIMER21->T2CON1.reg, (uint8)TIMER21_T2CON1_EXF2EN_Pos, (uint8)TIMER21_T2CON1_EXF2EN_Msk, 1u);
}


/** \brief Disables TIMER21 External Interrupt.
 *
 * \brief <b>Example</b><br>
 * \brief This example treats the TIMER21 External Interrupt.
 * ~~~~~~~~~~~~~~~{.c}
 * void Example_Function(void)
 * {
 *     TIMER21_External_Int_En();
 *     if (TIMER21_External_Sts() == 1)
 *     {
 *       TIMER21_EXF2_CALLBACK();
 *       TIMER21_External_Int_Clr();
 *     }
 *     TIMER21_External_Int_Dis();
 * }
 * ~~~~~~~~~~~~~~~
 * \ingroup TIMER2x_api
 */
INLINE void TIMER21_External_Int_Dis(void)
{
  Field_Mod8(&TIMER21->T2CON1.reg, (uint8)TIMER21_T2CON1_EXF2EN_Pos, (uint8)TIMER21_T2CON1_EXF2EN_Msk, 0u);
}

/*******************************************************************************
**                        Global Function Declarations                        **
*******************************************************************************/
/** \brief Initializes the Timer2 module based on the Config Wizard for MOTIX MCU configuration
 *
 * \ingroup timer2x_api
 */
void TIMER2_Init(void);

/** \brief Initializes the Timer21 module based on the Config Wizard for MOTIX MCU configuration
 *
 * \ingroup timer2x_api
 */
void TIMER21_Init(void);

/** \brief Initializes the TIMER2 in Reload Mode
 *
 * \param timer_interval_us timer interval in microseconds
 * \retval TRUE setup done
 * \retval FALSE setup failed
 *
 * \brief <b>Example</b><br>
 * \brief This example Initializes the TIMER2 in Reload Mode with timer equals to 5000 us.
 * ~~~~~~~~~~~~~~~{.c}
 * void Example_Function(void)
 * {
 *     TIMER2_Interval_Timer_Setup(0x1388);
 * }
 * ~~~~~~~~~~~~~~~
 * \ingroup timer2x_api
 */
bool TIMER2_Interval_Timer_Setup(uint32 timer_interval_us);

/** \brief Initializes the TIMER21 in Reload Mode
 *
 * \param timer_interval_us timer interval in microseconds
 * \retval TRUE setup done
 * \retval FALSE setup failed
 *
 * \brief <b>Example</b><br>
 * \brief This example Initializes the TIMER21 in Reload Mode with timer equals to 5000 us.
 * ~~~~~~~~~~~~~~~{.c}
 * void Example_Function(void)
 * {
 *     TIMER21_Interval_Timer_Setup(0x1388);
 * }
 * ~~~~~~~~~~~~~~~
 * \ingroup timer2x_api
 */
bool TIMER21_Interval_Timer_Setup(uint32 timer_interval_us);

/*******************************************************************************
**                     Global Inline Function Declarations                    **
*******************************************************************************/
INLINE void TIMER2_Start(void);
INLINE void TIMER2_Stop(void);
INLINE void TIMER2_ExtStart(void);
INLINE void TIMER2_ExtStop(void);
INLINE uint16 TIMER2_Value_Get(void);
INLINE uint16 TIMER2_Get_Count(void);
INLINE uint16 TIMER2_Capture_Value_Get(void);
INLINE uint16 TIMER2_Get_Capture(void);
INLINE void TIMER2_Clear_Count(void);
INLINE void TIMER2_Set_Reload(uint16 value);
INLINE void TIMER2_Select_T2IN(TTIMER2_T2IN_Pxx pinsel);
INLINE void TIMER2_Select_T2EX(TTIMER2_T2EX_Pxx pinsel);
INLINE void TIMER21_Start(void);
INLINE void TIMER21_Stop(void);
INLINE void TIMER21_ExtStart(void);
INLINE void TIMER21_ExtStop(void);
INLINE uint16 TIMER21_Value_Get(void);
INLINE uint16 TIMER21_Get_Count(void);
INLINE uint16 TIMER21_Capture_Value_Get(void);
INLINE uint16 TIMER21_Get_Capture(void);
INLINE void TIMER21_Clear_Count(void);
INLINE void TIMER21_Set_Reload(uint16 value);
INLINE void TIMER21_Select_T2IN(TTIMER21_T2IN_Pxx pinsel);
INLINE void TIMER21_Select_T2EX(TTIMER21_T2EX_Pxx pinsel);

/*******************************************************************************
**                     Global Inline Function Definitions                     **
*******************************************************************************/
/** \brief starts the TIMER2 by software
 *
 * \brief <b>Example</b><br>
 * \brief This example starts the TIMER2 by software.
 * ~~~~~~~~~~~~~~~{.c}
 * void Example_Function(void)
 * {
 *     TIMER2_Start();
 * }
 * ~~~~~~~~~~~~~~~
 * \ingroup TIMER2x_api
 */
INLINE void TIMER2_Start(void)
{
  Field_Mod8(&TIMER2->T2CON.reg, (uint8)TIMER2_T2CON_TR2_Pos, (uint8)TIMER2_T2CON_TR2_Msk, 1);
}

/** \brief stops the TIMER2 by software
 *
 * \brief <b>Example</b><br>
 * \brief This example stops and reset TIMER2 by software.
 * ~~~~~~~~~~~~~~~{.c}
 * void Example_Function(void)
 * {
 *     TIMER2_Stop();
 *     TIMER2_Clear_Count();
 * }
 * ~~~~~~~~~~~~~~~
 * \ingroup TIMER2x_api
 */
INLINE void TIMER2_Stop(void)
{
  Field_Mod8(&TIMER2->T2CON.reg, (uint8)TIMER2_T2CON_TR2_Pos, (uint8)TIMER2_T2CON_TR2_Msk, 0);
}

/** \brief prepares TIMER2 to be started externally
 *
 * \brief <b>Example</b><br>
 * \brief This example sets Rising Edge to External Start.
 * ~~~~~~~~~~~~~~~{.c}
 * void Example_Function(void)
 * {
 *     TIMER2_ExtStart();
 *     TIMER2_ExtStart_RisingEdge_Set();
 *     TIMER2_ExtStop();
 * }
 * ~~~~~~~~~~~~~~~
 * \ingroup TIMER2x_api
 */
INLINE void TIMER2_ExtStart(void)
{
  Field_Mod8(&TIMER2->T2MOD.reg, (uint8)TIMER2_T2MOD_T2RHEN_Pos, (uint8)TIMER2_T2MOD_T2RHEN_Msk, 1);
}

/** \brief disable TIMER2 to be started externally
 *
 * \brief <b>Example</b><br>
 * \brief This example sets Rising Edge to External Start.
 * ~~~~~~~~~~~~~~~{.c}
 * void Example_Function(void)
 * {
 *     TIMER2_ExtStart();
 *     TIMER2_ExtStart_RisingEdge_Set();
 *     TIMER2_ExtStop();
 * }
 * ~~~~~~~~~~~~~~~
 * \ingroup TIMER2x_api
 */
INLINE void TIMER2_ExtStop(void)
{
  Field_Mod8(&TIMER2->T2MOD.reg, (uint8)TIMER2_T2MOD_T2RHEN_Pos, (uint8)TIMER2_T2MOD_T2RHEN_Msk, 0);
}

/** \brief reads the TIMER2 count value
 *
 * \return TIMER2 count value
 *
 * \brief <b>Example</b><br>
 * \brief This example reads the TIMER2 count value.
 * ~~~~~~~~~~~~~~~{.c}
 * void Example_Function(void)
 * {
 *     uint16 timer_ticks;
 *     timer_ticks = TIMER2_Get_Count();
 * }
 * ~~~~~~~~~~~~~~~
 * \ingroup TIMER2x_api
 */
INLINE uint16 TIMER2_Get_Count(void)
{
  uint16 res;
  res = (uint16)(((uint16)u8_Field_Rd8(&TIMER2->T2H.reg, (uint8)TIMER2_T2H_T2H_Pos, (uint8)TIMER2_T2H_T2H_Msk)) << (uint8)8);
  res |= (uint16) u8_Field_Rd8(&TIMER2->T2L.reg, (uint8)TIMER2_T2L_T2L_Pos, (uint8)TIMER2_T2L_T2L_Msk);
  return (res);
}

/** \brief reads the TIMER2 count value
 *
 * \return TIMER2 count value
 *
 * \brief <b>Example</b><br>
 * \brief This example reads the TIMER2 count value.
 * ~~~~~~~~~~~~~~~{.c}
 * void Example_Function(void)
 * {
 *     uint16 timer_ticks;
 *
 *     timer_ticks = TIMER2_Value_Get();
 * }
 * ~~~~~~~~~~~~~~~
 * \ingroup TIMER2x_api
 */
INLINE uint16 TIMER2_Value_Get(void)
{
  uint16 res;
  res = (uint16)(((uint16)u8_Field_Rd8(&TIMER2->T2H.reg, (uint8)TIMER2_T2H_T2H_Pos, (uint8)TIMER2_T2H_T2H_Msk)) << (uint8)8);
  res |= (uint16) u8_Field_Rd8(&TIMER2->T2L.reg, (uint8)TIMER2_T2L_T2L_Pos, (uint8)TIMER2_T2L_T2L_Msk);
  return (res);
}

/** \brief reads the TIMER2 capture value
 *
 * \return TIMER2 capture value
 *
 * \brief <b>Example</b><br>
 * \brief This example reads the TIMER2 capture value.
 * ~~~~~~~~~~~~~~~{.c}
 * void Example_Function(void)
 * {
 *     uint16 timer_ticks;
 *
 *     timer_ticks = TIMER2_Get_Capture();
 * }
 * ~~~~~~~~~~~~~~~
 * \ingroup TIMER2x_api
 */
INLINE uint16 TIMER2_Get_Capture(void)
{
  uint16 res;
  res = (uint16)(((uint16)u8_Field_Rd8(&TIMER2->RC2H.reg, (uint8)TIMER2_RC2H_RC2_Pos, (uint8)TIMER2_RC2H_RC2_Msk)) << (uint8)8);
  res |= (uint16) u8_Field_Rd8(&TIMER2->RC2L.reg, (uint8)TIMER2_T2L_T2L_Pos, (uint8)TIMER2_T2L_T2L_Msk);
  return (res);
}

/** \brief reads the TIMER2 capture value
 *
 * \return TIMER2 capture value
 *
 * \brief <b>Example</b><br>
 * \brief This example reads the TIMER2 capture value.
 * ~~~~~~~~~~~~~~~{.c}
 * void Example_Function(void)
 * {
 *     uint16 timer_ticks;
 *
 *     timer_ticks = TIMER2_Capture_Value_Get();
 * }
 * ~~~~~~~~~~~~~~~
 * \ingroup TIMER2x_api
 */
INLINE uint16 TIMER2_Capture_Value_Get(void)
{
  uint16 res;
  res = (uint16)(((uint16)u8_Field_Rd8(&TIMER2->RC2H.reg, (uint8)TIMER2_RC2H_RC2_Pos, (uint8)TIMER2_RC2H_RC2_Msk)) << (uint8)8);
  res |= (uint16) u8_Field_Rd8(&TIMER2->RC2L.reg, (uint8)TIMER2_T2L_T2L_Pos, (uint8)TIMER2_T2L_T2L_Msk);
  return (res);
}

/** \brief clears the TIMER2 count value
 *
 * \brief <b>Example</b><br>
 * \brief This example stops and reset TIMER2 by software.
 * ~~~~~~~~~~~~~~~{.c}
 * void Example_Function(void)
 * {
 *     TIMER2_Stop();
 *     TIMER2_Clear_Count();
 * }
 * ~~~~~~~~~~~~~~~
 * \ingroup TIMER2x_api
 */
INLINE void TIMER2_Clear_Count(void)
{
  Field_Wrt8(&TIMER2->T2H.reg, (uint8)TIMER2_T2H_T2H_Pos, (uint8)TIMER2_T2H_T2H_Msk, 0);
  Field_Wrt8(&TIMER2->T2L.reg, (uint8)TIMER2_T2L_T2L_Pos, (uint8)TIMER2_T2L_T2L_Msk, 0);
}

/** \brief sets TIMER2 reload value
 *
 * \param value reload value in ticks
 *
 * \brief <b>Example</b><br>
 * \brief This example sets TIMER2 Reload Value to 5000 ticks.
 * ~~~~~~~~~~~~~~~{.c}
 * void Example_Function(void)
 * {
 *     TIMER2_Mode_Reload_Set();
 *     TIMER2_Set_Reload(0x1388);
 * }
 * ~~~~~~~~~~~~~~~
 * \ingroup TIMER2x_api
 */
INLINE void TIMER2_Set_Reload(uint16 value)
{
  Field_Wrt8(&TIMER2->RC2H.reg, (uint8)TIMER2_RC2H_RC2_Pos, (uint8)TIMER2_RC2H_RC2_Msk, (uint8)(value >> 8u));
  Field_Wrt8(&TIMER2->RC2L.reg, (uint8)TIMER2_RC2L_RC2_Pos, (uint8)TIMER2_RC2L_RC2_Msk, (uint8)(value & (uint16)0xFFu));
}

/** \brief sets TIMER2 T2IN Pin
 *
 * \param pinsel Pin select, (uint8)TIMER2_T2IN_Pxy
 *
 * \brief <b>Example</b><br>
 * \brief This example sets TIMER2 Input to T2_1(P2.5).
 * ~~~~~~~~~~~~~~~{.c}
 * void Example_Function(void)
 * {
 *     TIMER2_Select_T2IN(TIMER2_T2IN_P25);
 * }
 * ~~~~~~~~~~~~~~~
 * \ingroup TIMER2x_api
 */
INLINE void TIMER2_Select_T2IN(TTIMER2_T2IN_Pxx pinsel)
{
  Field_Mod8(&SCU->MODPISEL2.reg, (uint8)SCU_MODPISEL2_T2IS_Pos, (uint8)SCU_MODPISEL2_T2IS_Msk, (uint8)pinsel);
}

/** \brief sets TIMER2 T2EX Pin
 *
 * \param pinsel Pin select, (uint8)TIMER2_T2EX_Pxy
 *
 * \brief <b>Example</b><br>
 * \brief This example sets TIMER2 External Input to T2EX_1(P1.2).
 * ~~~~~~~~~~~~~~~{.c}
 * void Example_Function(void)
 * {
 *     TIMER2_Select_T2EX(TIMER2_T2EX_P12);
 * }
 * ~~~~~~~~~~~~~~~
 * \ingroup TIMER2x_api
 */
INLINE void TIMER2_Select_T2EX(TTIMER2_T2EX_Pxx pinsel)
{
  if (pinsel < TIMER2_T2EX_CCU6_CC60)
  {
    Field_Mod8(&SCU->MODPISEL1.reg, (uint8)SCU_MODPISEL1_T2EXCON_Pos, (uint8)SCU_MODPISEL1_T2EXCON_Msk, 0);
    Field_Mod8(&SCU->MODPISEL2.reg, (uint8)SCU_MODPISEL2_T2EXIS_Pos, (uint8)SCU_MODPISEL2_T2EXIS_Msk, (uint8)pinsel);
  }
  else
  {
    Field_Mod8(&SCU->MODPISEL1.reg, (uint8)SCU_MODPISEL1_T2EXCON_Pos, (uint8)SCU_MODPISEL1_T2EXCON_Msk, 1);
  }
}

/** \brief starts the TIMER21 by software
 *
 * \brief <b>Example</b><br>
 * \brief This example starts the TIMER21 by software.
 * ~~~~~~~~~~~~~~~{.c}
 * void Example_Function(void)
 * {
 *     TIMER21_Start();
 * }
 * ~~~~~~~~~~~~~~~
 * \ingroup TIMER2x_api
 */
INLINE void TIMER21_Start(void)
{
  Field_Mod8(&TIMER21->T2CON.reg, (uint8)TIMER21_T2CON_TR2_Pos, (uint8)TIMER21_T2CON_TR2_Msk, 1);
}

/** \brief stops the TIMER21 by software
 *
 * \brief <b>Example</b><br>
 * \brief This example stops and reset TIMER21 by software.
 * ~~~~~~~~~~~~~~~{.c}
 * void Example_Function(void)
 * {
 *     TIMER21_Stop();
 *     TIMER21_Clear_Count();
 * }
 * ~~~~~~~~~~~~~~~
 * \ingroup TIMER2x_api
 */
INLINE void TIMER21_Stop(void)
{
  Field_Mod8(&TIMER21->T2CON.reg, (uint8)TIMER21_T2CON_TR2_Pos, (uint8)TIMER21_T2CON_TR2_Msk, 0);
}

/** \brief prepares TIMER21 to be started externally
 *
 * \brief <b>Example</b><br>
 * \brief This example sets Rising Edge to External Start.
 * ~~~~~~~~~~~~~~~{.c}
 * void Example_Function(void)
 * {
 *     TIMER21_ExtStart();
 *     TIMER21_ExtStart_RisingEdge_Set();
 *     TIMER21_ExtStop();
 * }
 * ~~~~~~~~~~~~~~~
 * \ingroup TIMER2x_api
 */
INLINE void TIMER21_ExtStart(void)
{
  Field_Mod8(&TIMER21->T2MOD.reg, (uint8)TIMER21_T2MOD_T2RHEN_Pos, (uint8)TIMER21_T2MOD_T2RHEN_Msk, 1);
}

/** \brief disable TIMER21 to be started externally
 *
 * \brief <b>Example</b><br>
 * \brief This example sets Rising Edge to External Start.
 * ~~~~~~~~~~~~~~~{.c}
 * void Example_Function(void)
 * {
 *     TIMER21_ExtStart();
 *     TIMER21_ExtStart_RisingEdge_Set();
 *     TIMER21_ExtStop();
 * }
 * ~~~~~~~~~~~~~~~
 * \ingroup TIMER2x_api
 */
INLINE void TIMER21_ExtStop(void)
{
  Field_Mod8(&TIMER21->T2MOD.reg, (uint8)TIMER21_T2MOD_T2RHEN_Pos, (uint8)TIMER21_T2MOD_T2RHEN_Msk, 0);
}

/** \brief reads the TIMER21 count value
 *
 * \return TIMER21 count value
 *
 * \brief <b>Example</b><br>
 * \brief This example reads the TIMER21 count value.
 * ~~~~~~~~~~~~~~~{.c}
 * void Example_Function(void)
 * {
 *     uint16 timer_ticks;
 *
 *     timer_ticks = TIMER21_Get_Count();
 * }
 * ~~~~~~~~~~~~~~~
 * \ingroup TIMER2x_api
 */
INLINE uint16 TIMER21_Get_Count(void)
{
  uint16 res;
  res = (uint16)(((uint16)u8_Field_Rd8(&TIMER21->T2H.reg, (uint8)TIMER21_T2H_T2H_Pos, (uint8)TIMER21_T2H_T2H_Msk)) << (uint8)8);
  res |= (uint16) u8_Field_Rd8(&TIMER21->T2L.reg, (uint8)TIMER21_T2L_T2L_Pos, (uint8)TIMER21_T2L_T2L_Msk);
  return (res);
}

/** \brief reads the TIMER21 count value
 *
 * \return TIMER21 count value
 *
 * \brief <b>Example</b><br>
 * \brief This example reads the TIMER21 count value.
 * ~~~~~~~~~~~~~~~{.c}
 * void Example_Function(void)
 * {
 *     uint16 timer_ticks;
 *
 *     timer_ticks = TIMER21_Value_Get();
 * }
 * ~~~~~~~~~~~~~~~
 * \ingroup TIMER2x_api
 */
INLINE uint16 TIMER21_Value_Get(void)
{
  uint16 res;
  res = (uint16)(((uint16)u8_Field_Rd8(&TIMER21->T2H.reg, (uint8)TIMER21_T2H_T2H_Pos, (uint8)TIMER21_T2H_T2H_Msk)) << (uint8)8);
  res |= (uint16) u8_Field_Rd8(&TIMER21->T2L.reg, (uint8)TIMER21_T2L_T2L_Pos, (uint8)TIMER21_T2L_T2L_Msk);
  return (res);
}

/** \brief reads the TIMER21 capture value
 *
 * \return TIMER21 capture value
 *
 * \brief <b>Example</b><br>
 * \brief This example reads the TIMER21 capture value.
 * ~~~~~~~~~~~~~~~{.c}
 * void Example_Function(void)
 * {
 *     uint16 timer_ticks;
 *
 *     timer_ticks = TIMER21_Get_Capture();
 * }
 * ~~~~~~~~~~~~~~~
 * \ingroup TIMER2x_api
 */
INLINE uint16 TIMER21_Get_Capture(void)
{
  uint16 res;
  res = (uint16)(((uint16)u8_Field_Rd8(&TIMER21->RC2H.reg, (uint8)TIMER21_RC2H_RC2_Pos, (uint8)TIMER21_RC2H_RC2_Msk)) << (uint8)8);
  res |= (uint16) u8_Field_Rd8(&TIMER21->RC2L.reg, (uint8)TIMER21_T2L_T2L_Pos, (uint8)TIMER21_T2L_T2L_Msk);
  return (res);
}

/** \brief reads the TIMER21 capture value
 *
 * \return TIMER21 capture value
 *
 * \brief <b>Example</b><br>
 * \brief This example reads the TIMER21 capture value.
 * ~~~~~~~~~~~~~~~{.c}
 * void Example_Function(void)
 * {
 *     uint16 timer_ticks;
 *
 *     timer_ticks = TIMER21_Capture_Value_Get();
 * }
 * ~~~~~~~~~~~~~~~
 * \ingroup TIMER2x_api
 */
INLINE uint16 TIMER21_Capture_Value_Get(void)
{
  uint16 res;
  res = (uint16)(((uint16)u8_Field_Rd8(&TIMER21->RC2H.reg, (uint8)TIMER21_RC2H_RC2_Pos, (uint8)TIMER21_RC2H_RC2_Msk)) << (uint8)8);
  res |= (uint16) u8_Field_Rd8(&TIMER21->RC2L.reg, (uint8)TIMER21_T2L_T2L_Pos, (uint8)TIMER21_T2L_T2L_Msk);
  return (res);
}

/** \brief clears the TIMER21 count value
 *
 * \brief <b>Example</b><br>
 * \brief This example stops and reset TIMER21 by software.
 * ~~~~~~~~~~~~~~~{.c}
 * void Example_Function(void)
 * {
 *     TIMER21_Stop();
 *     TIMER21_Clear_Count();
 * }
 * ~~~~~~~~~~~~~~~
 * \ingroup TIMER2x_api
 */
INLINE void TIMER21_Clear_Count(void)
{
  Field_Wrt8(&TIMER21->T2H.reg, (uint8)TIMER21_T2H_T2H_Pos, (uint8)TIMER21_T2H_T2H_Msk, 0);
  Field_Wrt8(&TIMER21->T2L.reg, (uint8)TIMER21_T2L_T2L_Pos, (uint8)TIMER21_T2L_T2L_Msk, 0);
}

/** \brief sets TIMER21 reload value
 *
 * \param value reload value in ticks
 *
 * \brief <b>Example</b><br>
 * \brief This example sets TIMER21 Reload Value to 5000 ticks.
 * ~~~~~~~~~~~~~~~{.c}
 * void Example_Function(void)
 * {
 *     TIMER21_Mode_Reload_Set();
 *     TIMER21_Set_Reload(0x1388);
 * }
 * ~~~~~~~~~~~~~~~
 * \ingroup TIMER2x_api
 */
INLINE void TIMER21_Set_Reload(uint16 value)
{
  Field_Wrt8(&TIMER21->RC2H.reg, (uint8)TIMER21_RC2H_RC2_Pos, (uint8)TIMER21_RC2H_RC2_Msk, (uint8)(value >> 8u));
  Field_Wrt8(&TIMER21->RC2L.reg, (uint8)TIMER21_RC2L_RC2_Pos, (uint8)TIMER21_RC2L_RC2_Msk, (uint8)(value & (uint16)0xFFu));
}

/** \brief sets TIMER21 T2IN Pin
 *
 * \param pinsel Pin select, (uint8)TIMER21_T2IN_Pxy
 *
 * \brief <b>Example</b><br>
 * \brief This example sets TIMER2 Input to T21_1(P1.1).
 * ~~~~~~~~~~~~~~~{.c}
 * void Example_Function(void)
 * {
 *     TIMER21_Select_T2IN(TIMER21_T2IN_P11);
 * }
 * ~~~~~~~~~~~~~~~
 * \ingroup TIMER2x_api
 */
INLINE void TIMER21_Select_T2IN(TTIMER21_T2IN_Pxx pinsel)
{
  Field_Mod8(&SCU->MODPISEL2.reg, (uint8)SCU_MODPISEL2_T21IS_Pos, (uint8)SCU_MODPISEL2_T21IS_Msk, (uint8)pinsel);
}

/** \brief sets TIMER21 T2EX Pin
 *
 * \param pinsel Pin select, (uint8)TIMER21_T2EX_Pxy
 *
 * \brief <b>Example</b><br>
 * \brief This example sets TIMER2 External Input to T21EX_0(P0.2).
 * ~~~~~~~~~~~~~~~{.c}
 * void Example_Function(void)
 * {
 *     TIMER21_Select_T2EX(TIMER21_T2EX_P02);
 * }
 * ~~~~~~~~~~~~~~~
 * \ingroup TIMER2x_api
 */
INLINE void TIMER21_Select_T2EX(TTIMER21_T2EX_Pxx pinsel)
{
  if (pinsel < TIMER21_T2EX_CCU6_COUT60)
  {
    Field_Mod8(&SCU->MODPISEL1.reg, (uint8)SCU_MODPISEL1_T21EXCON_Pos, (uint8)SCU_MODPISEL1_T21EXCON_Msk, 0);
    Field_Mod8(&SCU->MODPISEL2.reg, (uint8)SCU_MODPISEL2_T21EXIS_Pos, (uint8)SCU_MODPISEL2_T21EXIS_Msk, (uint8)pinsel);
  }
  else
  {
    Field_Mod8(&SCU->MODPISEL1.reg, (uint8)SCU_MODPISEL1_T21EXCON_Pos, (uint8)SCU_MODPISEL1_T21EXCON_Msk, 1);
  }
}

/** \brief Sets TIMER2 Value.
 *
 * \param t2 TIMER2 Value
 *
 * \brief <b>Example</b><br>
 * \brief This example sets TIMER2 Value to 5000 ticks.
 * ~~~~~~~~~~~~~~~{.c}
 * void Example_Function(void)
 * {
 *     TIMER2_Value_Set(0x1388);
 * }
 * ~~~~~~~~~~~~~~~
 * \ingroup TIMER2x_api
 */
INLINE void TIMER2_Value_Set(uint16 t2)
{
  Field_Wrt8(&TIMER2->T2H.reg, (uint8)TIMER2_T2H_T2H_Pos, (uint8)TIMER2_T2H_T2H_Msk, (uint8)(t2 >> 8u));
  Field_Wrt8(&TIMER2->T2L.reg, (uint8)TIMER2_T2L_T2L_Pos, (uint8)TIMER2_T2L_T2L_Msk, ((uint8)t2 & (uint8)0xFF));
}

/** \brief Sets TIMER2 Reload Value.
 *
 * \param t2 TIMER2 Reload Value
 *
 * \brief <b>Example</b><br>
 * \brief This example sets TIMER2 Reload Value to 5000.
 * ~~~~~~~~~~~~~~~{.c}
 * void Example_Function(void)
 * {
 *     TIMER2_Mode_Reload_Set();
 *     TIMER2_Reload_Value_Set(0x1388);
 * }
 * ~~~~~~~~~~~~~~~
 * \ingroup TIMER2x_api
 */
INLINE void TIMER2_Reload_Value_Set(uint16 t2)
{
  Field_Wrt8(&TIMER2->RC2H.reg, (uint8)TIMER2_RC2H_RC2_Pos, (uint8)TIMER2_RC2H_RC2_Msk, (uint8)(t2 >> 8u));
  Field_Wrt8(&TIMER2->RC2L.reg, (uint8)TIMER2_RC2L_RC2_Pos, (uint8)TIMER2_RC2L_RC2_Msk, ((uint8)t2 & (uint8)0xFF));
}

/** \brief Sets TIMER21 Value.
 *
 * \param t21 TIMER21 Value
 *
 * \brief <b>Example</b><br>
 * \brief This example sets TIMER21 Value to 5000.
 * ~~~~~~~~~~~~~~~{.c}
 * void Example_Function(void)
 * {
 *     TIMER21_Value_Set(0x1388);
 * }
 * ~~~~~~~~~~~~~~~
 * \ingroup TIMER2x_api
 */
INLINE void TIMER21_Value_Set(uint16 t21)
{
  Field_Wrt8(&TIMER21->T2H.reg, (uint8)TIMER21_T2H_T2H_Pos, (uint8)TIMER21_T2H_T2H_Msk, (uint8)(t21 >> 8u));
  Field_Wrt8(&TIMER21->T2L.reg, (uint8)TIMER21_T2L_T2L_Pos, (uint8)TIMER21_T2L_T2L_Msk, ((uint8)t21 & (uint8)0xFF));
}

/** \brief Sets TIMER21 Reload Value.
 *
 * \param t21 TIMER21 Reload Value
 *
 * \brief <b>Example</b><br>
 * \brief This example sets TIMER21 Reload Value to 5000.
 * ~~~~~~~~~~~~~~~{.c}
 * void Example_Function(void)
 * {
 *     TIMER21_Mode_Reload_Set();
 *     TIMER21_Reload_Value_Set(0x1388);
 * }
 * ~~~~~~~~~~~~~~~
 * \ingroup TIMER2x_api
 */
INLINE void TIMER21_Reload_Value_Set(uint16 t21)
{
  Field_Wrt8(&TIMER21->RC2H.reg, (uint8)TIMER21_RC2H_RC2_Pos, (uint8)TIMER21_RC2H_RC2_Msk, (uint8)(t21 >> 8u));
  Field_Wrt8(&TIMER21->RC2L.reg, (uint8)TIMER21_RC2L_RC2_Pos, (uint8)TIMER21_RC2L_RC2_Msk, ((uint8)t21 & (uint8)0xFF));
}

#endif
