/*
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 * 
 * SPDX-License-Identifier: BSD-3-Clause
 */

/*
 * TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
!!GlobalInfo
product: Pins v3.0
processor: QN908XC
package_id: QN9080C
mcu_data: ksdk2_0
processor_version: 0.0.0
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS ***********
 */

#include "fsl_iocon.h"
#include "pin_mux.h"

#define PIN16_IDX                       16u   /*!< Pin number for pin 16 in a port */
#define PIN17_IDX                       17u   /*!< Pin number for pin 17 in a port */
#define PIN22_IDX                       22u   /*!< Pin number for pin 22 in a port */
#define PIN30_IDX                       30u   /*!< Pin number for pin 30 in a port */
#define PIN2_IDX                         2u   /*!< Pin number for pin 2 in a port */
#define PIN3_IDX                         3u   /*!< Pin number for pin 3 in a port */
#define PIN4_IDX                         4u   /*!< Pin number for pin 4 in a port */
#define PIN5_IDX                         5u   /*!< Pin number for pin 5 in a port */
#define PIN8_IDX                         8u   /*!< Pin number for pin 8 in a port */
#define PORTA_IDX                        0u   /*!< Port index */

/*
 * TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
BOARD_InitPins:
- options: {callFromInitBoot: 'true', coreID: cm4}
- pin_list:
  - {pin_num: '8', peripheral: FLEXCOMM2, signal: SDA_SSEL0, pin_signal: GPIOA3/QDEC0_B/SCT0_OUT3/CTIMER0_MAT1/FC2_SDA_SSEL0/RFE_TX_EN, pull_control: High_Z, drive_strength: low}
  - {pin_num: '15', peripheral: FLEXCOMM2, signal: SCK, pin_signal: GPIOA30/ACMP1P/ETM_TRACEDAT3/CTIMER3_MAT1/FC2_SCK/FC3_MOSI/SPIFI_IO3, pull_control: High_Z, drive_strength: low}
  - {pin_num: '7', peripheral: FLEXCOMM2, signal: SDA_MOSI, pin_signal: GPIOA4/ADC2/SCT0_OUT4/CTIMER0_MAT0/FC0_TXD/FC2_SDA_MOSI/SPIFI_IO0, pull_control: High_Z, drive_strength: low}
  - {pin_num: '6', peripheral: FLEXCOMM2, signal: SCL_MISO, pin_signal: GPIOA5/ADC3/SCT0_OUT5/CTIMER0_MAT1/FC0_RXD/FC2_SCL_MISO/SPIFI_IO1, pull_control: High_Z, drive_strength: low}
  - {pin_num: '29', peripheral: FLEXCOMM0, signal: TXD, pin_signal: GPIOA16/CS2/SCT0_OUT1/CTIMER2_MAT0/FC0_TXD/FC3_MOSI/QDEC0_A, pull_control: High_Z, drive_strength: low}
  - {pin_num: '28', peripheral: FLEXCOMM0, signal: RXD, pin_signal: GPIOA17/CS3/SD_DAC/CTIMER2_MAT1/FC0_RXD/FC3_MISO/QDEC0_B, pull_control: High_Z, drive_strength: low}
  - {pin_num: '23', peripheral: SWD, signal: SWCLK, pin_signal: SWCLK/GPIOA22/SCT0_IN2/CTIMER3_MAT0/FC2_SDA_SSEL0/FC3_SSEL3/QDEC1_A, pull_control: Pull_down, drive_strength: low}
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS ***********
 */

/*FUNCTION**********************************************************************
 *
 * Function Name : BOARD_InitPins
 * Description   : Configures pin routing and optionally pin electrical features.
 *
 *END**************************************************************************/
void BOARD_InitPins(void) {
	  const uint32_t portA_pin8_config = (
	    IOCON_FUNC1 |                                            /* Selects pin function 1 */
	    IOCON_MODE_HIGHZ |                                       /* Selects High-Z function */
	    IOCON_DRIVE_LOW                                          /* Enable low drive strength */
	  );
	  IOCON_PinMuxSet(IOCON, PORTA_IDX, PIN8_IDX, portA_pin8_config); /* PORTA PIN8 (coords: 46) is configured as ADC4 */
  const uint32_t portA_pin3_config = (
    IOCON_FUNC5 |                                            /* Selects pin function 5 */
    IOCON_MODE_HIGHZ |                                       /* Selects High-Z function */
    IOCON_DRIVE_LOW                                          /* Enable low drive strength */
  );
  IOCON_PinMuxSet(IOCON, PORTA_IDX, PIN3_IDX, portA_pin3_config); /* PORTA PIN3 (coords: 8) is configured as FC2_SDA_SSEL0 */
  // stuff i added start
  const uint32_t portA_pin2_config = (
    IOCON_FUNC5 |                                            /* Selects pin function 5 */
    IOCON_MODE_HIGHZ |                                       /* Selects High-Z function */
    IOCON_DRIVE_LOW                                          /* Enable low drive strength */
  );
  IOCON_PinMuxSet(IOCON, PORTA_IDX, PIN2_IDX, portA_pin2_config); /* PORTA PIN2 (coords: 9) is configured as FC2_SCL_SSEL1 */
  // stuff i added end

  const uint32_t portA_pin4_config = (
    IOCON_FUNC5 |                                            /* Selects pin function 5 */
    IOCON_MODE_HIGHZ |                                       /* Selects High-Z function */
    IOCON_DRIVE_LOW                                          /* Enable low drive strength */
  );
  IOCON_PinMuxSet(IOCON, PORTA_IDX, PIN4_IDX, portA_pin4_config); /* PORTA PIN4 (coords: 7) is configured as FC2_SDA_MOSI */
  const uint32_t portA_pin5_config = (
    IOCON_FUNC5 |                                            /* Selects pin function 5 */
    IOCON_MODE_HIGHZ |                                       /* Selects High-Z function */
    IOCON_DRIVE_LOW                                          /* Enable low drive strength */
  );
  IOCON_PinMuxSet(IOCON, PORTA_IDX, PIN5_IDX, portA_pin5_config); /* PORTA PIN5 (coords: 6) is configured as FC2_SCL_MISO */
  const uint32_t portA_pin16_config = (
    IOCON_FUNC4 |                                            /* Selects pin function 4 */
    IOCON_MODE_HIGHZ |                                       /* Selects High-Z function */
    IOCON_DRIVE_LOW                                          /* Enable low drive strength */
  );
  const uint32_t portA_pin30_config = (
    IOCON_FUNC4 |                                            /* Selects pin function 4 */
    IOCON_MODE_HIGHZ |                                       /* Selects High-Z function */
    IOCON_DRIVE_LOW                                          /* Enable low drive strength */
  );
  IOCON_PinMuxSet(IOCON, PORTA_IDX, PIN30_IDX, portA_pin30_config); /* PORTA PIN30 (coords: 15) is configured as FC2_SCK */
  IOCON_PinMuxSet(IOCON, PORTA_IDX, PIN16_IDX, portA_pin16_config); /* PORTA PIN16 (coords: 29) is configured as FC0_TXD */
  const uint32_t portA_pin17_config = (
    IOCON_FUNC4 |                                            /* Selects pin function 4 */
    IOCON_MODE_HIGHZ |                                       /* Selects High-Z function */
    IOCON_DRIVE_LOW                                          /* Enable low drive strength */
  );
  IOCON_PinMuxSet(IOCON, PORTA_IDX, PIN17_IDX, portA_pin17_config); /* PORTA PIN17 (coords: 28) is configured as FC0_RXD */
  const uint32_t portA_pin22_config = (
    IOCON_FUNC0 |                                            /* Selects pin function 0 */
    IOCON_MODE_PULLDOWN |                                    /* Selects pull-down function */
    IOCON_DRIVE_LOW                                          /* Enable low drive strength */
  );
  IOCON_PinMuxSet(IOCON, PORTA_IDX, PIN22_IDX, portA_pin22_config); /* PORTA PIN22 (coords: 23) is configured as SWCLK */

}

/*******************************************************************************
 * EOF
 ******************************************************************************/
