/*
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/

#include "fsl_debug_console.h"
#include "board.h"
#include "fsl_rtc.h"
#include "fsl_power.h"
#include "power_mode_switch.h"
#include "clock_config.h"
#include "fsl_iocon.h"
#include "fsl_spi.h"


#include "pin_mux.h"
#include "fsl_calibration.h"
#include "fsl_usart.h"
// adc config
#include "fsl_device_registers.h"
#include "fsl_debug_console.h"
#include "fsl_adc.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define DEMO_ADC_BASE ADC                           /* Demo base address */
#define DEMO_ADC_CHANNEL 8                          /* Select channel */
#define DEMO_ADC_CFG_IDX 0                          /* Select configuration */
#define DEMO_ADC_TRIGGER kADC_TriggerSelectSoftware /* Software trigger */
/*******************************************************************************
 * Variables
 ******************************************************************************/
uint32_t g_AdcVinn;
float g_AdcBandgap;

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
static void ADC_Configuration(void);
/* #include "nvds.h"
#include "fsl_syscon.h" */
// end adc config
/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define DEMO_USART USART0
#define DEMO_USART_BAUDRATE 115200
#define DEMO_USART_CLK_FREQ CLOCK_GetFreq(kCLOCK_BusClk)
#define RTC_INTERRUPT_THRESHOLD (32768U * 5U) //set time to sleep for



//spi
#define EXAMPLE_SPI_MASTER SPI0
#define EXAMPLE_SPI_MASTER_IRQ FLEXCOMM2_IRQn
#define EXAMPLE_SPI_MASTER_CLK_SRC kCLOCK_BusClk
#define EXAMPLE_SPI_MASTER_CLK_FREQ CLOCK_GetFreq(kCLOCK_BusClk)
#define EXAMPLE_SPI_SSEL 1
#define EXAMPLE_SPI_SPOL (kSPI_SpolActiveAllLow | kSPI_Spol1ActiveHigh)



/*******************************************************************************
 * Prototypes
 ******************************************************************************/
extern void BOARD_WakeupRestore(void);

/*******************************************************************************
 * Variables
 ******************************************************************************/
volatile uint8_t g_RtcFreeRunningFlag;
#define BUFFER_SIZE (5)
static uint8_t srcBuff[BUFFER_SIZE];
static uint8_t destBuff[BUFFER_SIZE];
static uint8_t srcBuff2[2];
static uint8_t destBuff2[2];
static uint8_t srcBuff3[6];
static uint8_t destBuff3[6];
static uint8_t srcBuff4[6];
static uint8_t destBuff4[6];
static uint8_t srcBuff5[1];
static uint8_t destBuff5[1];
static uint8_t test;
static uint8_t digit1;
static uint8_t digit2;
static uint8_t seconds;
static uint8_t adc4;
static uint8_t mem_loc;



/*******************************************************************************
 * Code
 ******************************************************************************/
/*flash_config_t g_flash_cfg;*/

/* Reinitialize peripherals after waked up from PD, this function will be called in critical area */
void BOARD_WakeupRestore(void)
{
    //    BUTTON_EnableInterrupt();

    /*
     * config.baudRate_Bps = 115200U;
     * config.parityMode = kUART_ParityDisabled;
     * config.stopBitCount = kUART_OneStopBit;
     * config.loopback = false;
     * config.enableTx = false;
     * config.enableRx = false;
     */
    usart_config_t config;
    USART_GetDefaultConfig(&config);
    config.baudRate_Bps = DEMO_USART_BAUDRATE;
    config.enableTx = true;
    config.enableRx = true;
    USART_Init(DEMO_USART, &config, DEMO_USART_CLK_FREQ);
}


void delay(uint32_t dly)
{
    volatile uint32_t i;

    for (i = 0U; i < dly; ++i)
    {
        __asm("NOP");
    }
}

void RTC_FR_IRQHandler(void)
{
    //    if (RTC_GetStatusFlags(RTC) & (uint32_t)kRTC_FreeRunningInterruptFlag)
    {
        g_RtcFreeRunningFlag = 1U;
    }
}

static void switch_to_OSC32M(void)
{
    POWER_WritePmuCtrl1(SYSCON, SYSCON_PMU_CTRL1_OSC32M_DIS_MASK, SYSCON_PMU_CTRL1_OSC32M_DIS(0U));
    SYSCON->CLK_CTRL = (SYSCON->CLK_CTRL & ~SYSCON_CLK_CTRL_SYS_CLK_SEL_MASK) | SYSCON_CLK_CTRL_SYS_CLK_SEL(0U);
}

static void switch_to_XTAL(void)
{
    /* switch to XTAL after it is stable */
    while (!(SYSCON_SYS_MODE_CTRL_XTAL_RDY_MASK & SYSCON->SYS_MODE_CTRL))
        ;
    SYSCON->CLK_CTRL = (SYSCON->CLK_CTRL & ~SYSCON_CLK_CTRL_SYS_CLK_SEL_MASK) | SYSCON_CLK_CTRL_SYS_CLK_SEL(1U);
    POWER_WritePmuCtrl1(SYSCON, SYSCON_PMU_CTRL1_OSC32M_DIS_MASK, SYSCON_PMU_CTRL1_OSC32M_DIS(1U));
}

void wakeup_by_capa_sensor(void)
{
    /* config PA15 as cs's input */
    IOCON_PinMuxSet(IOCON, 0U, 15U, IOCON_FUNC1);
    /* IOCON_PinMuxSet(IOCON, 0U, 14U, IOCON_FUNC1); */

    /* config PA15 to High-Z */
    SYSCON->PIO_PULL_CFG[0] &= ~0xC0000000U;

    CLOCK_EnableClock(kCLOCK_Cs);

    POWER_WritePmuCtrl1(SYSCON, SYSCON_PMU_CTRL1_CAP_SEN_DIS_MASK, SYSCON_PMU_CTRL1_CAP_SEN_DIS(0U));

    CS->CTRL0 =
        (CS->CTRL0 & ~(CS_CTRL0_CLK_DIV_MASK | CS_CTRL0_OSC_FREQ_MASK | CS_CTRL0_SRST_MASK | CS_CTRL0_ENABLE_MASK)) |
        (79U << CS_CTRL0_CLK_DIV_SHIFT) | (10U << CS_CTRL0_OSC_FREQ_SHIFT) | (0U << CS_CTRL0_CLK_DIV_SHIFT) |
        CS_CTRL0_ENABLE_MASK;

    CS->LP_CTRL =
        (CS->LP_CTRL &
         ~(CS_LP_CTRL_THR_MASK | CS_LP_CTRL_LP_CH_MASK | CS_LP_CTRL_LP_EN_MASK | CS_LP_CTRL_DEBONCE_NUM_MASK)) |
        (600U << CS_LP_CTRL_THR_SHIFT) | (1U << CS_LP_CTRL_LP_CH_SHIFT) | CS_LP_CTRL_LP_EN_MASK |
        (10U << CS_LP_CTRL_DEBONCE_NUM_SHIFT);

    /* after each scan loop, idle for some time. unit: 10ms */
    /* CS->IDLE_PERIOD = 50U; */

    CS->LP_INTEN = CS_LP_INTEN_LP_INTEN_MASK;
}

void CS_WAKEUP_IRQHandler(void)
{
    CS->LP_CTRL &= ~CS_LP_CTRL_LP_EN_MASK;
    NVIC_DisableIRQ(CS_WAKEUP_IRQn);
    NVIC_ClearPendingIRQ(CS_WAKEUP_IRQn);
}

/* #define CAP_SENSOR_WAKEUP */
static void ADC_Configuration(void)
{
    adc_config_t adcConfigStruct;
    adc_sd_config_t adcSdConfigStruct;

    /**
     * Initial ADC to default configuration.
     */
    ADC_GetDefaultConfig(&adcConfigStruct);
    adcConfigStruct.channelEnable = (1U << DEMO_ADC_CHANNEL);
    adcConfigStruct.channelConfig = (DEMO_ADC_CFG_IDX << DEMO_ADC_CHANNEL);
    adcConfigStruct.triggerSource = DEMO_ADC_TRIGGER;
    adcConfigStruct.convMode = kADC_ConvModeSingle;
    ADC_Init(DEMO_ADC_BASE, &adcConfigStruct);

    /* Initial ADC Sigma Delta(SD) configuration */
    ADC_GetSdDefaultConfig(&adcSdConfigStruct);
    ADC_SetSdConfig(DEMO_ADC_BASE, DEMO_ADC_CFG_IDX, &adcSdConfigStruct);

    /* Bandgap voltage */
    g_AdcBandgap = ADC_GetBandgapCalibrationResult(DEMO_ADC_BASE, DEMO_ADC_CFG_IDX);

    /* Calibration VINN value */
    g_AdcVinn = ADC_GetVinnCalibrationResult(DEMO_ADC_BASE, &adcConfigStruct);

    /* Enable ADC */
    ADC_Enable(DEMO_ADC_BASE, true);
}
/*!
 * @brief Main function
 */
int main(void)
{
    uint8_t ch;

    uint32_t msk, val;

    POWER_EnableDCDC(true);

#if 0
    FLASH_GetDefaultConfig(&g_flash_cfg);
    FLASH_Init(FLASH, &g_flash_cfg);
    if (NVDS_OK ==
        nvds_init((uint8_t *)CFG_NVDS_ADDRESS, CFG_NVDS_SIZE, (uint8_t *)CFG_NVDS_BACKUP_ADDRESS, NULL, &g_flash_cfg))
    {
        /* configure load cap for crystal on board */
        uint8_t loadCap;
        nvds_tag_len_t length = NVDS_LEN_XCSEL;
        if (NVDS_OK == nvds_get(NVDS_TAG_XCSEL, (nvds_tag_len_t *)&length, &loadCap))
        {
            SYSCON_SetLoadCap(SYSCON, 1, loadCap);
        }

        length = NVDS_LEN_32K_XCSEL;
        if (NVDS_OK == nvds_get(NVDS_TAG_32K_XCSEL, (nvds_tag_len_t *)&length, &loadCap))
        {
            SYSCON_SetLoadCap(SYSCON, 0, loadCap);
        }
    }
#endif
    CALIB_SystemCalib();

    BOARD_InitPins();
    BOARD_BootClockHSRUN();
    BOARD_InitDebugConsole();

    /* Shut down higher 120K RAM for lower power consumption */
    //POWER_EnablePD(kPDRUNCFG_PD_MEM9);
    //POWER_EnablePD(kPDRUNCFG_PD_MEM8);
   // POWER_EnablePD(kPDRUNCFG_PD_MEM7);
    //POWER_EnablePD(kPDRUNCFG_PD_MEM6);
   // POWER_EnablePD(kPDRUNCFG_PD_MEM5);
  //  POWER_EnablePD(kPDRUNCFG_PD_MEM4);
    //POWER_EnablePD(kPDRUNCFG_PD_MEM3);
  //  POWER_EnablePD(kPDRUNCFG_PD_MEM2);

    POWER_Init();

    POWER_EnablePD(kPDRUNCFG_PD_FIR);
    POWER_EnablePD(kPDRUNCFG_PD_FSP);
    POWER_EnablePD(kPDRUNCFG_PD_OSC32M);

    /* Enable OSC_EN as interrupt and wakeup source */
    //    SYSCON->PMU_CTRL0 = SYSCON->PMU_CTRL0 | SYSCON_PMU_CTRL0_OSC_INT_MSK_MASK;
    /* Power control 1 */
    msk = SYSCON_PMU_CTRL1_XTAL32K_PDM_DIS_MASK | SYSCON_PMU_CTRL1_RCO32K_PDM_DIS_MASK |
          SYSCON_PMU_CTRL1_XTAL32K_DIS_MASK | SYSCON_PMU_CTRL1_RCO32K_DIS_MASK;

#if (defined(BOARD_XTAL1_CLK_HZ) && (BOARD_XTAL1_CLK_HZ == 32768U))
    val = SYSCON_PMU_CTRL1_XTAL32K_PDM_DIS(0U)  /* switch on XTAL32K during power down */
          | SYSCON_PMU_CTRL1_RCO32K_PDM_DIS(1U) /* switch off RCO32K during power down */
          | SYSCON_PMU_CTRL1_XTAL32K_DIS(0U)    /* switch on XTAL32K */
          | SYSCON_PMU_CTRL1_RCO32K_DIS(1U);    /* switch off RCO32K at all time */
#else
    val = SYSCON_PMU_CTRL1_XTAL32K_PDM_DIS(1U)  /* switch off XTAL32K during power down */
          | SYSCON_PMU_CTRL1_RCO32K_PDM_DIS(0U) /* switch on RCO32K during power down */
          | SYSCON_PMU_CTRL1_XTAL32K_DIS(1U)    /* switch off XTAL32K at all time */
          | SYSCON_PMU_CTRL1_RCO32K_DIS(0U);    /* switch on RCO32K */
#endif

    /* The default setting of capacitive sensor, DAC, ADC and USB PLL's power are disabled.
       User should power on these peripherals when using them. */
    POWER_WritePmuCtrl1(SYSCON, msk, val);

    /* Enable GPIO clock */
    CLOCK_EnableClock(kCLOCK_Gpio);
    /* Use Low Frequency Clock source as wdt's clock source */
    CLOCK_AttachClk(k32K_to_WDT_CLK);
    /* Config BLE's high frequency clock to 8MHz */
    CLOCK_AttachClk(k8M_to_BLE_CLK);

    /* Disable ble core's clock. If ble core's clock is enabled, ahb can only be 8M, 16M or 32M */
    CLOCK_DisableClock(kCLOCK_Ble);

    //    BUTTON_Init();
    //    BUTTON_SetGpioWakeupLevel(BOARD_SW_GPIO, BOARD_SW1_GPIO_PIN, 0U);
    //    BUTTON_SetGpioWakeupLevel(BOARD_SW_GPIO, BOARD_SW2_GPIO_PIN, 0U);

    RTC_Init(RTC);

    /* Enable RTC free running interrupt */
    RTC_EnableInterrupts(RTC, kRTC_FreeRunningInterruptEnable);
    /* Enable at the NVIC */
    NVIC_EnableIRQ(RTC_FR_IRQn);

    /* Enable gpio wakeup at button2 */
    SYSCON->PIO_WAKEUP_LVL0 = SYSCON->PIO_WAKEUP_LVL0 | BOARD_SW2_GPIO_PIN_MASK;
    SYSCON->PIO_WAKEUP_EN0 = SYSCON->PIO_WAKEUP_EN0 | BOARD_SW2_GPIO_PIN_MASK;

    g_RtcFreeRunningFlag = 0U;
#if defined(CAP_SENSOR_WAKEUP)
    wakeup_by_capa_sensor();
#endif
    PRINTF("\r\n####################  Power Mode Switch Demo ####################\n\r\n");
    PRINTF("    Sys Clock = %dHz \r\n", CLOCK_GetFreq(kCLOCK_CoreSysClk));
    PRINTF("    Ahb Clock = %dHz \r\n", CLOCK_GetFreq(kCLOCK_BusClk));

    mem_loc = 0x16U;

    while (1)
    {
        if (g_RtcFreeRunningFlag)
        {
            g_RtcFreeRunningFlag = 0U;
            PRINTF("Waked up by rtc free running interrupt.\r\n");
        }
        //delay(1000000);


        		    //read memory
        					spi_master_config_t userConfig = {0};
        					uint32_t srcFreq = 0;
        					spi_transfer_t xfer = {0};


        					SPI_MasterGetDefaultConfig(&userConfig); //this will get default configurations for the SPI master such as baud rate and clock phase
        					srcFreq = EXAMPLE_SPI_MASTER_CLK_FREQ; //gets frequency to run SPI functions
        			        userConfig.sselNum = (spi_ssel_t)0; //this sets the SS pin to active low.
        					userConfig.sselPol = (spi_spol_t)EXAMPLE_SPI_SPOL;	//set the clock polarity
        					SPI_MasterInit(EXAMPLE_SPI_MASTER, &userConfig, srcFreq); //initialises SPI master functionality

    			    						srcBuff3[0] = 0x03U; //read command
    			                            srcBuff3[1] = 0x01U;//24 bit address
    			                            srcBuff3[2] = 0x26U; //24 bit address
    			                            srcBuff3[3] = mem_loc-2; //24 bit address
    			                            srcBuff3[4] = 0x00U;// receive reply from eeprom
    			                            srcBuff3[5] = 0x00U;// receive reply from eeprom
    			                            //srcBuff3[6] = 0x00U;// receive reply from eeprom


    			                            xfer.txData = srcBuff3; //data to be transfered
    			                            xfer.rxData = destBuff3; //data to be received
    			                            xfer.dataSize = sizeof(destBuff3);
    			                            SPI_MasterTransferBlocking(EXAMPLE_SPI_MASTER, &xfer); //data sent
    			            			    SPI_MasterInit(EXAMPLE_SPI_MASTER, &userConfig, srcFreq); //initialises SPI master functionality

    			                test = destBuff3[4];
								digit1 = test & 0xF0;// select first 4 bits
								digit1 = digit1>>4; //shift bits by 4.this is the first digit
								digit2 = test & 0x0F;// this is the second digit
    			                PRINTF("Previous Values: %d%d seconds %d uA \r\n",digit1, digit2,destBuff3[5]); // output data to console
    			                PRINTF("Current Memory Location: %d \r\n", mem_loc); // digit 1 and two output to the console

					//SPI read clock
					userConfig.sselNum = (spi_ssel_t)EXAMPLE_SPI_SSEL; //this sets the SS pin to active high.
					SPI_MasterInit(EXAMPLE_SPI_MASTER, &userConfig, srcFreq); //initialises SPI master functionality
					/* Init Buffer*/

										srcBuff2[0] = 0x92U; // selects the 'seconds' register and asks to read.
										srcBuff2[1] = 0x00U; // selects the 'seconds' register and asks to read.
										xfer.txData = srcBuff2;
										xfer.rxData = destBuff2; //data is read and received at the same time
										xfer.dataSize = sizeof(destBuff2);
										SPI_MasterTransferBlocking(EXAMPLE_SPI_MASTER, &xfer); //'seconds' register selected

										/*Check if the data is right*/
									  //output data to console
										test = destBuff2[1];
										digit1 = test & 0xF0;// select first 4 bits
										digit1 = digit1>>4; //shift bits by 4.this is the first digit
										digit2 = test & 0x0F;// this is the second digit
										PRINTF("Time: %d%d seconds \r\n", digit1, digit2); // digit 1 and two output to the console
										seconds = test;


			POWER_EnableADC(true);
			ADC_Configuration();
				//PRINTF("ADC configured \r\n");

				/**
				 * ADC single mode
				 * (Vext - Vinn) / Vref = Vreg / 2^22  ==> Vext = Vreg * Vref / 2^22 + Vinn
				 */
				float fresult;
				uint32_t adcConvResult;

					/* Software triger */
					ADC_DoSoftwareTrigger(DEMO_ADC_BASE);
					/* Wait for convert complete */
					while (!(ADC_GetStatusFlags(DEMO_ADC_BASE) & kADC_DataReadyFlag))
					{
					}
					/* Get the result */
					adcConvResult = ADC_GetConversionResult(DEMO_ADC_BASE);

					fresult = ADC_ConversionResult2Mv(DEMO_ADC_BASE, DEMO_ADC_CHANNEL, DEMO_ADC_CFG_IDX, g_AdcBandgap, g_AdcVinn,
													  adcConvResult);
					//PRINTF("Original: 0x%x\t Ch: %d\t Result: %f(mv)\r\n", adcConvResult, DEMO_ADC_CHANNEL, fresult);
					fresult = fresult/27;
					//PRINTF("fresult: %f\r\n", fresult);
					adc4 = (uint8_t) fresult;
					//PRINTF("adc4: %d\r\n", adc4);
					fresult = fresult-adc4;
					//PRINTF("round: %f\r\n", fresult);
					if (fresult>0.49)
					{
						adc4=adc4+1;
					}
					PRINTF("Light: %d uA \r\n", adc4);

					//store data in memory
		userConfig.sselNum = (spi_ssel_t)0; //this sets the SS pin to active low.
		SPI_MasterInit(EXAMPLE_SPI_MASTER, &userConfig, srcFreq); //initialises SPI master functionality


		srcBuff5[0] = 0x06U; //Write Enable register written to.


			/*Start Transfer*/
		// data is both transfered and read by the master at the same time.
		// for this case the received data isnt relevent as we have only written to the slave. not read
			xfer.txData = srcBuff5; //data to be transfered
			xfer.rxData = destBuff5; //data to be received
			xfer.dataSize = sizeof(destBuff5);
			SPI_MasterTransferBlocking(EXAMPLE_SPI_MASTER, &xfer); //data sent
			SPI_MasterInit(EXAMPLE_SPI_MASTER, &userConfig, srcFreq); //initialises SPI master functionality

					srcBuff4[0] = 0x02U; //write command
					srcBuff4[1] = 0x01U;	//24 bit address
					srcBuff4[2] = 0x26U; //24 bit address
					srcBuff4[3] = mem_loc; //24 bit address
					srcBuff4[4] = seconds; //value to store
					srcBuff4[5] = adc4; //value to store
					//srcBuff4[6] = 0x03; //value to store


					xfer.txData = srcBuff4; //data to be transfered
					xfer.rxData = destBuff4; //data to be received
					xfer.dataSize = sizeof(destBuff4);
					SPI_MasterTransferBlocking(EXAMPLE_SPI_MASTER, &xfer); //data sent
					SPI_MasterInit(EXAMPLE_SPI_MASTER, &userConfig, srcFreq); //initialises SPI master functionality
					PRINTF("Stored data in memory! \r\n"); // digit 1 and two output to the console

					mem_loc = mem_loc+2;
					PRINTF("Device put to sleep \r\n"); // digit 1 and two output to the console

//        PRINTF("\r\nSelect the desired operation \n\r\n");
//        PRINTF("Press %c to enter: Active        - Normal RUN mode\r\n", kPmActive + 'a');
//        PRINTF("Press %c to enter: Sleep         - Sleep mode\r\n", kPmSleep + 'a');
//        PRINTF("Press %c to enter: Power down 0  - Power down 0 mode\r\n", kPmPowerDown0 + 'a');
//        PRINTF("Press %c to enter: Power down 1  - Power down 1 mode\r\n", kPmPowerDown1 + 'a');
//
//        PRINTF("\r\nWaiting for power mode select...\r\n\r\n");


        RTC_SetFreeRunningInterruptThreshold(RTC, RTC_GetFreeRunningCount(RTC) + RTC_INTERRUPT_THRESHOLD);
        RTC_FreeRunningEnable(RTC, true);


//                PRINTF(" Now in power down 0 mode for about 10 seconds.\r\n");
//                __disable_irq();
//                switch_to_OSC32M();
//#if defined(CAP_SENSOR_WAKEUP)
//                NVIC_ClearPendingIRQ(CS_WAKEUP_IRQn);
//                NVIC_EnableIRQ(CS_WAKEUP_IRQn);
//                CS->LP_CTRL |= CS_LP_CTRL_LP_EN_MASK;
//#endif
//                POWER_LatchIO();
//                CLOCK_DisableClock(kCLOCK_Flexcomm0);
//                POWER_EnterPowerDown(0);
//                CLOCK_EnableClock(kCLOCK_Flexcomm0);
//                POWER_RestoreIO();
//                switch_to_XTAL();
//                /* after waking up from pwoer down, usart config is lost, recover it */
//                BOARD_WakeupRestore();
//                __enable_irq();
        NVIC_ClearPendingIRQ(EXT_GPIO_WAKEUP_IRQn);
                       NVIC_EnableIRQ(EXT_GPIO_WAKEUP_IRQn);
                       __disable_irq();
                       switch_to_OSC32M();
       #if (defined(BOARD_XTAL1_CLK_HZ) && (BOARD_XTAL1_CLK_HZ == CLK_RCO_32KHZ))
                       POWER_WritePmuCtrl1(SYSCON, SYSCON_PMU_CTRL1_RCO32K_DIS_MASK, SYSCON_PMU_CTRL1_RCO32K_DIS(1U));
       #elif(defined(BOARD_XTAL1_CLK_HZ) && (BOARD_XTAL1_CLK_HZ == CLK_XTAL_32KHZ))
                       POWER_WritePmuCtrl1(SYSCON, SYSCON_PMU_CTRL1_XTAL32K_DIS_MASK, SYSCON_PMU_CTRL1_XTAL32K_DIS(1U));
       #endif
                       POWER_LatchIO();
                       POWER_EnterPowerDown(0);
                       POWER_RestoreIO();
                       switch_to_XTAL();

                       NVIC_DisableIRQ(EXT_GPIO_WAKEUP_IRQn);
                       NVIC_ClearPendingIRQ(EXT_GPIO_WAKEUP_IRQn);

       #if (defined(BOARD_XTAL1_CLK_HZ) && (BOARD_XTAL1_CLK_HZ == CLK_RCO_32KHZ))
                       POWER_WritePmuCtrl1(SYSCON, SYSCON_PMU_CTRL1_RCO32K_DIS_MASK, SYSCON_PMU_CTRL1_RCO32K_DIS(0U));
       #elif(defined(BOARD_XTAL1_CLK_HZ) && (BOARD_XTAL1_CLK_HZ == CLK_XTAL_32KHZ))
                       POWER_WritePmuCtrl1(SYSCON, SYSCON_PMU_CTRL1_XTAL32K_DIS_MASK, SYSCON_PMU_CTRL1_XTAL32K_DIS(0U));
       #endif
                       /* after waking up from pwoer down, usart config is lost, recover it */
                       BOARD_WakeupRestore();
                       __enable_irq();

        RTC_FreeRunningEnable(RTC, false);

        //delay(1000000);
        PRINTF("\r\nNext loop\r\n");
    }
}

