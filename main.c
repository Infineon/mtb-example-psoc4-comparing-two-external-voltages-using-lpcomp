/******************************************************************************
 * File Name:   main.c
 *
 * Description: This is the source code for the PSoC 4 Comparing Two
 * External Voltages Using LPComp Example for ModusToolbox.
 *
 * Related Document: See README.md
 *
 *******************************************************************************
 * Copyright 2023-2024, Cypress Semiconductor Corporation (an Infineon company) or
 * an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
 *
 * This software, including source code, documentation and related
 * materials ("Software") is owned by Cypress Semiconductor Corporation
 * or one of its affiliates ("Cypress") and is protected by and subject to
 * worldwide patent protection (United States and foreign),
 * United States copyright laws and international treaty provisions.
 * Therefore, you may use this Software only as provided in the license
 * agreement accompanying the software package from which you
 * obtained this Software ("EULA").
 * If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 * non-transferable license to copy, modify, and compile the Software
 * source code solely for use in connection with Cypress's
 * integrated circuit products.  Any reproduction, modification, translation,
 * compilation, or representation of this Software except as specified
 * above is prohibited without the express written permission of Cypress.
 *
 * Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
 * reserves the right to make changes to the Software without notice. Cypress
 * does not assume any liability arising out of the application or use of the
 * Software or any product or circuit described in the Software. Cypress does
 * not authorize its products for use in any products where a malfunction or
 * failure of the Cypress product may reasonably be expected to result in
 * significant property damage, injury or death ("High Risk Product"). By
 * including Cypress's product in a High Risk Product, the manufacturer
 * of such system or application assumes all risk of such use and in doing
 * so agrees to indemnify Cypress against all liability.
 *******************************************************************************/

/******************************************************************************
 * Include header files
 ******************************************************************************/
#include "cy_pdl.h"
#include "cybsp.h"

/*******************************************************************************
 * Macros
 ********************************************************************************/
/* Start up delay in microseconds */
#define LPCOMP_LP_STARTUP_DELAY_US     (10u)

/* To demonstrate how PDL drivers are used to manually configure the peripherals,
 * set the PDL_CONFIGURATION #define to 1, otherwise leave it set to 0.
 */
#define PDL_CONFIGURATION   (0u)

#if defined COMPONENT_PSOC4100SP256KB
#define GPIO_PORT_NUM GPIO_PRT1
#define GPIO_PIN_NUM 6
#define GPIO_PORT_NUM_CH GPIO_PRT0
#define GPIO_PIN_NUM_CH_Pos 2
#define GPIO_PIN_NUM_CH_Neg 3
#define LPCOMP_CHANNEL CY_LPCOMP_CHANNEL_1
#elif defined COMPONENT_PSOC4HVMS64K || COMPONENT_PSOC4HVMS128K
#define GPIO_PORT_NUM GPIO_PRT0
#define GPIO_PIN_NUM 5
#define GPIO_PORT_NUM_CH GPIO_PRT1
#define GPIO_PIN_NUM_CH_Pos 1
#define GPIO_PIN_NUM_CH_Neg 2
#define LPCOMP_CHANNEL CY_LPCOMP_CHANNEL_0
#else /* COMPONENT_PSOC4100SP, COMPONENT_PSOC4000S */
#define GPIO_PORT_NUM GPIO_PRT2
#define GPIO_PIN_NUM 0
#define GPIO_PORT_NUM_CH GPIO_PRT0
#define GPIO_PIN_NUM_CH_Pos 0
#define GPIO_PIN_NUM_CH_Neg 1
#define LPCOMP_CHANNEL CY_LPCOMP_CHANNEL_0
#endif

/*******************************************************************************
 * Global Variables
 ********************************************************************************/
/* LPComp context structure */
cy_stc_lpcomp_context_t lpcomp_context;

#if PDL_CONFIGURATION
const cy_stc_lpcomp_config_t lpcomp_config =
{
    .outputMode = CY_LPCOMP_OUT_DIRECT,
    .hysteresis = CY_LPCOMP_HYST_ENABLE,
    .power = CY_LPCOMP_MODE_ULP,
    .intType = CY_LPCOMP_INTR_RISING,
};

#if defined COMPONENT_PSOC4100SP256KB
cy_stc_gpio_pin_config_t pin_config = {
    /*.outVal     */ 1UL,                       /* Output = High */
    /*.driveMode  */ CY_GPIO_DM_STRONG_IN_OFF,  /* Strong drive, input buffer off */
    /*.hsiom      */ P1_6_GPIO,                 /* Software controlled pin */
    /*.intEdge    */ CY_GPIO_INTR_DISABLE,      /* Rising edge interrupt */
    /*.vtrip      */ CY_GPIO_VTRIP_CMOS,        /* CMOS voltage trip */
    /*.slewRate   */ CY_GPIO_SLEW_FAST,         /* Fast slew rate */
};

#elif defined COMPONENT_PSOC4HVMS64K || COMPONENT_PSOC4HVMS128K
cy_stc_gpio_pin_config_t pin_config = {
    /*.outVal     */ 1UL,                       /* Output = High */
    /*.driveMode  */ CY_GPIO_DM_STRONG_IN_OFF,  /* Strong drive, input buffer off */
    /*.hsiom      */ P0_5_GPIO,                 /* Software controlled pin */
    /*.intEdge    */ CY_GPIO_INTR_DISABLE,      /* Rising edge interrupt */
    /*.vtrip      */ CY_GPIO_VTRIP_CMOS,        /* CMOS voltage trip */
    /*.slewRate   */ CY_GPIO_SLEW_FAST,         /* Fast slew rate */
};

#else /* COMPONENT_PSOC4100SP, COMPONENT_PSOC4000S */
cy_stc_gpio_pin_config_t pin_config = {
    /*.outVal     */ 1UL,                       /* Output = High */
    /*.driveMode  */ CY_GPIO_DM_STRONG_IN_OFF,  /* Strong drive, input buffer off */
    /*.hsiom      */ P2_0_GPIO,                 /* Software controlled pin */
    /*.intEdge    */ CY_GPIO_INTR_DISABLE,      /* Rising edge interrupt */
    /*.vtrip      */ CY_GPIO_VTRIP_CMOS,        /* CMOS voltage trip */
    /*.slewRate   */ CY_GPIO_SLEW_FAST,         /* Fast slew rate */
};
#endif

#endif

/*******************************************************************************
 * Function Name: main
 ********************************************************************************
 * Summary:
 * System entrance point. This function performs
 *    1. Initializes the BSP.
 *    2. Initialize and enable the LPComp peripheral.
 *    3. Compares the LPComp input voltages.
 *
 * Parameters:
 *  void
 *
 * Return:
 *  int
 *
 *******************************************************************************/
int main(void)
{
    cy_rslt_t result;

    /* Initialize the device and board peripherals */
    result = cybsp_init();

    /* Board init failed. Stop program execution. */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Enable global interrupts */
    __enable_irq();

#if PDL_CONFIGURATION
    /* Initialize GPIO pin for LPComp compare output(comp) */
    Cy_GPIO_Pin_Init(GPIO_PORT_NUM, GPIO_PIN_NUM, &pin_config);

    /* LPComp Positive input (inp) */
    Cy_GPIO_Pin_FastInit(GPIO_PORT_NUM_CH, GPIO_PIN_NUM_CH_Pos, CY_GPIO_DM_ANALOG, 1, HSIOM_SEL_GPIO);

    /* LPComp Negative input (inn) */
    Cy_GPIO_Pin_FastInit(GPIO_PORT_NUM_CH, GPIO_PIN_NUM_CH_Neg, CY_GPIO_DM_ANALOG, 1, HSIOM_SEL_GPIO);

    /* Initialize LPComp block */
    if(CY_LPCOMP_SUCCESS != Cy_LPComp_Init(LPCOMP, LPCOMP_CHANNEL, &lpcomp_config, &lpcomp_context))
    {
        CY_ASSERT(0);
    }

    /* Enable the LPComp channel */
    Cy_LPComp_Enable(LPCOMP, LPCOMP_CHANNEL, &lpcomp_context);

    /* It needs 10us start-up time to settle LPComp channel in LP mode after power up */
    Cy_SysLib_DelayUs(LPCOMP_LP_STARTUP_DELAY_US);

#else
    /* Initialize and enable the LPComp block */
    if(CY_LPCOMP_SUCCESS != Cy_LPComp_Init(LPCOMP, LPCOMP_CHANNEL, &LPComp_1_config, &lpcomp_context))
    {
        CY_ASSERT(0);
    }
    Cy_LPComp_Enable(LPCOMP, LPCOMP_CHANNEL, &lpcomp_context);
#endif


    for (;;)
    {
#if PDL_CONFIGURATION
        /* Control based on the comparison result */
        Cy_GPIO_Write(GPIO_PORT_NUM, GPIO_PIN_NUM, Cy_LPComp_GetCompare(LPCOMP, LPCOMP_CHANNEL));
#endif
    }
}


/* [] END OF FILE */
