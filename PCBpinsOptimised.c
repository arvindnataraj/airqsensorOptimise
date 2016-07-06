/******************************************************************************
 * @file:    airq_sensors_standalone.c
 * @brief:   BreathEZY standalone sensors i/f code
 *-----------------------------------------------------------------------------
 *
 Copyright (c) 2011-2014 Analog Devices, Inc.

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:
  - Redistributions of source code must retain the above copyright notice,
    this list of conditions and the following disclaimer.
  - Redistributions in binary form must reproduce the above copyright notice,
    this list of conditions and the following disclaimer in the documentation
    and/or other materials provided with the distribution.
  - Modified versions of the software must be conspicuously marked as such.
  - This software is licensed solely and exclusively for use with processors
    manufactured by or for Analog Devices, Inc.
  - This software may not be combined or merged with other code in any manner
    that would cause the software to become subject to terms and conditions
    which differ from those listed here.
  - Neither the name of Analog Devices, Inc. nor the names of its
    contributors may be used to endorse or promote products derived
    from this software without specific prior written permission.
  - The use of this software may or may not infringe the patent rights of one
    or more patent holders.  This license does not release you from the
    requirement that you obtain separate licenses from these patent holders
    to use this software.

THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES, INC. AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, NON-
INFRINGEMENT, TITLE, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL ANALOG DEVICES, INC. OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, PUNITIVE OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, DAMAGES ARISING OUT OF
CLAIMS OF INTELLECTUAL PROPERTY RIGHTS INFRINGEMENT; PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.

*****************************************************************************/
//P0 03 - PCB LED

#include <stdint.h>
#include <stdio.h>
#include "common.h"
#include "system.h"
#include <services/tmr/adi_tmr.h>
#include <services/pwr/adi_pwr.h>
#include <services/gpio/adi_gpio.h>
#include <drivers/adc/adi_adc.h>
#include <services/rtc/adi_rtc.h>
#include "airq_sensors_standalone.h"

/* Enter required fanOnTime in seconds */
#define fanOnTime (2u)                                    

#if defined ( __ICCARM__ )
    #define ALIGN4 _Pragma("data_alignment=4")
#elif defined (__CC_ARM)
    #define ALIGN4 __align(4)
#else
    #pragma message("WARNING: NO ALIGHMENT DEFINED FOR MEMORY")
#endif

static const uint16_t GPT1_LOAD_490p5MSEC  = 49816;
static const uint16_t GPT1_LOAD_500MSEC    = 50781;//Calculation for LOAD value: 50795 / (26MHz/256) = 500 ms
static const uint16_t GPT1_LOAD_1SEC       = 25391;
static const uint16_t GPT1_LOAD_0p28MSEC   = 28;
static const uint16_t GPT1_LOAD_0p04MSEC   = 4;
static const uint16_t GPT1_LOAD_9p68MSEC   = 983;
static const uint16_t GPT1_LOAD_2p5MSEC    = 254;
static const uint16_t GPT1_LOAD_14MSEC     = 1422;

/* ADC Device number */
#define ADC_DEV_NUM                (0u)

/* Number of samples to sample for ADC */
#define ADC_NUM_SAMPLES             (4u)

static uint8_t aDeviceMemory1[ADI_TMR_MEMORY_SIZE];
static uint8_t gpioMemory[ADI_GPIO_MEMORY_SIZE];
static uint8_t aRtcDevMem0[ADI_RTC_MEMORY_SIZE];

/* Memory Required for adc driver */
ALIGN4 static uint8_t DeviceMemory[ADI_ADC_MEMORY_SIZE];
/* ADC Data Buffer */
ALIGN4 static uint16_t ADC_DataBuffer[ADC_NUM_SAMPLES] = {0};

ADI_TMR_HANDLE hDevice1;
/* ADC Handle */
ADI_ADC_HANDLE hDevice;
/* Device handle for RTC device-1*/
ADI_RTC_HANDLE hDeviceRTC  = NULL;

ADI_TMR_RESULT result;
ADI_GPIO_RESULT eResult;
ADI_ADC_RESULT adcResult;

#define LED4         ADI_GPIO_PORT1, ADI_GPIO_PIN_12
#define LED5         ADI_GPIO_PORT1, ADI_GPIO_PIN_13


/*EZKIT*/
/*CO sensor heater control: P2.0 / J9-7*/
//#define CO_HEATER    ADI_GPIO_PORT2, ADI_GPIO_PIN_0
/*CO sensor sense control: P2.2 / J9-6*/
//#define CO_SENSE     ADI_GPIO_PORT2, ADI_GPIO_PIN_2
/*PM2.5 sensor LED control: P0.15 / J9-3*/
//#define PM25_LED     ADI_GPIO_PORT0, ADI_GPIO_PIN_15
/*PM2.5 sensor FAN control: P2.11 / J9-4*/
//#define PM25_FAN     ADI_GPIO_PORT2, ADI_GPIO_PIN_11


/*PCB*/
/*CO sensor heater control: P1.7*/
#define CO_HEATER    ADI_GPIO_PORT1, ADI_GPIO_PIN_7
/*CO sensor sense control: P1.6*/
#define CO_SENSE     ADI_GPIO_PORT1, ADI_GPIO_PIN_6
/*PM2.5 sensor LED control: P2.9*/
#define PM25_LED     ADI_GPIO_PORT2, ADI_GPIO_PIN_9
/*PM2.5 sensor FAN control: P1.8*/
#define PM25_FAN     ADI_GPIO_PORT1, ADI_GPIO_PIN_8
#define EN_5V        ADI_GPIO_PORT1, ADI_GPIO_PIN_14

static uint8_t curr_state;
static uint8_t cnt_samples;

static uint8_t cnt_fan_cycles;
static uint8_t cnt_co_heater_cycles;

void ADC_Setup (void);

static void usleep(uint32_t usec);

#define RTC_ALARM_OFFSET   10
#define CNT_SAMPLES_AVG    1

int NUM_FAN_500MS_CYCLES = 0;

int tmp=0,k=0,start=1;

ADI_PWR_RESULT pwrResult;

/* flag for hibernate mode */
bool_t flagHib = false;                                        

ADI_ADC_RESULT  adcResult = ADI_ADC_SUCCESS;
ADI_ADC_BUFFER Buffer;

void enterActiveMode (void);

int main(void)
{
    /* Clock initialization */
    SystemInit();
    
    /* test system initialization */
    test_Init();
    
    /* Enter Active mode for enabling Buck conveter - optimise active current */
    enterActiveMode();
    
  /* Number of 500ms cycles equals ratio of given fanOnTime to 0.5 */     
    NUM_FAN_500MS_CYCLES = fanOnTime/0.5;   
    
    do 
    {
      if(ADI_PWR_SUCCESS != adi_pwr_Init())
        {
          //DEBUG_MESSAGE("Failed to intialize the power service\n");
          break;
        }
      
      if(ADI_PWR_SUCCESS != adi_pwr_SetLFClockMux(ADI_CLOCK_MUX_LFCLK_LFXTAL))
      {
        return(eResult);
      }
      
      if(ADI_PWR_SUCCESS != adi_pwr_EnableClockSource(ADI_CLOCK_SOURCE_HFXTAL, true))
      {
        return(eResult);
      }

      if(ADI_PWR_SUCCESS != adi_pwr_SetRootClockMux(ADI_CLOCK_MUX_ROOT_HFXTAL))
      {
        return(eResult);
      }

      
      if(ADI_PWR_SUCCESS != adi_pwr_EnableClockSource(ADI_CLOCK_SOURCE_LFXTAL,true))
      {
        return(eResult);
      }
   
      if (ADI_PWR_SUCCESS != adi_pwr_SetClockDivider(ADI_CLOCK_HCLK,1))
      {
          //DEBUG_MESSAGE("Failed to intialize the power service\n");
      }
      if (ADI_PWR_SUCCESS != adi_pwr_SetClockDivider(ADI_CLOCK_PCLK,1))
      {
          //DEBUG_MESSAGE("Failed to intialize the power service\n");
      }

  
      if(ADI_RTC_SUCCESS !=rtc_Init())
        {
          //DEBUG_MESSAGE("\nFailed to initialize RTC device \n");
        }
      

      
      if(ADI_GPIO_SUCCESS != adi_gpio_Init(gpioMemory, ADI_GPIO_MEMORY_SIZE))
        {
           // DEBUG_MESSAGE("adi_gpio_Init failed\n");
            break;
        }
      
      //adi_gpio_OutputEnable(LED4, true);
      //adi_gpio_OutputEnable(LED5, true);
      adi_gpio_OutputEnable(CO_HEATER, true);
      adi_gpio_OutputEnable(CO_SENSE, true);
      adi_gpio_OutputEnable(PM25_LED, true);
      adi_gpio_OutputEnable(PM25_FAN, true);
      adi_gpio_OutputEnable(EN_5V, true);
      
      adi_gpio_SetLow(CO_HEATER);
      adi_gpio_SetLow(CO_SENSE);
      adi_gpio_SetLow(PM25_LED);
      adi_gpio_SetLow(PM25_FAN);
      adi_gpio_SetLow(EN_5V);
      
      ADC_Setup();
      
      adi_tmr_Open(TIMER_DEVICE_1,aDeviceMemory1,ADI_TMR_MEMORY_SIZE,&hDevice1); 
      adi_tmr_RegisterCallback( hDevice1, GPTimer1Callback ,hDevice1);
      
      adi_tmr_SetPrescaler(hDevice1, ADI_GPT_PRESCALER_256);
      adi_tmr_SetLoadValue( hDevice1, GPT1_LOAD_1SEC);
     
      //DEBUG_MESSAGE("AQ Sensor initializing!\n");    
        
    }while(0);
    
/* Hibernate label for entering hibernate mode */    
Hibernate :
  if(k==1)  
  {
    /* ADC must be disabled before entering hibernate mode */
    adcResult = adi_adc_Enable (hDevice, false);    
  
    /* Close ADC device before going to hibernate */
    adcResult = adi_adc_Close(hDevice);          
    
    /* Entering hibernate mode */
    pwrResult = adi_pwr_EnterLowPowerMode(ADI_PWR_MODE_HIBERNATE,&flagHib,0x00);  
  
  }
  do
  {
    if(k==1)
     {
   /* Once one cycle of measurements completes - jumps to label 'Hibernate' to switch to hibernate mode till next RTC alarm */
      goto Hibernate;
     } 
   }
   while(1);
    
    
 
}
void enterActiveMode()
{
  /*    SWITCHED TO LOW POWER MODE - ACTIVE MODE    */
  pwrResult = adi_pwr_EnterLowPowerMode(ADI_PWR_MODE_ACTIVE,NULL,0x00);  
  //DEBUG_RESULT("\n Failed to enter active mode %04d",pwrResult,ADI_PWR_SUCCESS);
  
  /*    BUCK CONVERTER ENABLED TO REDUCE POWER      */
  adi_pwr_EnableHPBuck(true); 
}

/* RTC-0 Callback handler */
/*This callback initializes the GPT1 timer, which controls the sensor st8-mc.*/
void rtc0Callback (void *pCBParam, uint32_t nEvent, void *EventArg) {
    
    /* Clear RTC Interrupt status for the next RTC interrupt */
    adi_rtc_ClearInterruptStatus(hDeviceRTC,ADI_RTC_ALARM_INT);
  
    /* LED4 TOGGLE - CHECK FOR RTC INTERRUPTS */
   // adi_gpio_Toggle(LED4);  
  
    
    
    /* flagHib is set to 'true' in adi_pwr_ExitLowPowerMode() to exit hibernate mode */
    pwrResult = adi_pwr_ExitLowPowerMode(&flagHib);   
    
    /* Enter Active mode for enabling Buck conveter - optimise active current */
    enterActiveMode();
   
  /* Components of ADC are powered down in hibernate mode - so must be explicitly powered up after waking up*/
    if(pwrResult==ADI_PWR_SUCCESS)
    {
     k = 0;
     start = 2;  //start is set to '2' inorder to avoid performing ADC calibration again when ADC_Setup is called again 
     
     /* ADC Setup is called again after wakeup to setup the ADC for conversions */
     ADC_Setup();  
    }

   if (ADI_RTC_ALARM_INT & nEvent) 
    {
       // DEBUG_MESSAGE("RTC interrupt");
        
	/*Initialize st8-mc variables*/
	curr_state           = 0;
        cnt_samples          = 0;
        cnt_fan_cycles       = 0;
        cnt_co_heater_cycles = 0;
        
        /* Update RTC alarm */
           rtc_UpdateAlarm();
        
        adi_gpio_SetHigh(EN_5V);
        
	/*kick-start sensor st8-mc*/
        adi_tmr_SetLoadValue( hDevice1, GPT1_LOAD_500MSEC);
        adi_tmr_Enable(hDevice1, true);
    }
}

/**
 * Callback for GPT-1. 
*
 * @return none
 *
 */
/*GPT1 timer callback contains the sensor st8-mc*/
static void GPTimer1Callback(void *pCBParam, uint32_t Event, void *pArg)
{
    adi_tmr_Enable(hDevice1,false); 
  switch(Event)
  {
      case ADI_TMR_EVENT_TIMEOUT:
        switch (curr_state) {
        
	
	/* st8 0 : CO sensor - heater ON for 980 ms*/	
        case 0 : 
          
            if (cnt_samples >= CNT_SAMPLES_AVG) 
             {
               curr_state = 10;//go to PM2.5 sensor - begin by turning on the fan [st8 10]
               cnt_samples = 0;
              
	       adi_gpio_SetLow(CO_HEATER);
               adi_gpio_SetLow(CO_SENSE);
                 
               adi_gpio_SetHigh(PM25_FAN);
               adi_tmr_SetLoadValue( hDevice1, GPT1_LOAD_500MSEC);
               adi_tmr_Enable(hDevice1,true); 
               
             }
            else
            {
              adi_gpio_SetHigh(CO_HEATER);
              
              /*Wait until 2 heater-cycles of 490.5 ms are done - i.e., wait for heater to be ON for 981 ms as per sensor spec.
	        (limitation of GPT1 clock frequency - cannot count 981 ms in one go)*/
	      if (cnt_co_heater_cycles == 1)
              {
                curr_state = 1;  		  
	      }
	      else
	      {            
                curr_state = 0;
	      }
              cnt_co_heater_cycles++;
              adi_tmr_SetLoadValue( hDevice1, GPT1_LOAD_490p5MSEC);
              adi_tmr_Enable(hDevice1,true); 
           }   
           break;
         
       /* st8 1 : CO sensor - sense circuit ON for 2.5 ms*/	
       case 1 :
               adi_gpio_SetHigh(CO_SENSE);
                                 
               curr_state = 2;
               adi_tmr_SetLoadValue( hDevice1, GPT1_LOAD_2p5MSEC);    
               adi_tmr_Enable(hDevice1,true); 
              
         break;
         
       /* st8 2 : CO sensor - trigger ADC sampling on channel-2 */	
       case 2 :
         
                     /* Populate the buffer structure */
             Buffer.nBuffSize = sizeof(ADC_DataBuffer);
             Buffer.nChannels = ADI_ADC_CHANNEL_3;                                           
             Buffer.nNumConversionPasses = ADC_NUM_SAMPLES;
             Buffer.pDataBuffer = ADC_DataBuffer;
    
             /* Submit the buffer to the driver */
             adcResult = adi_adc_SubmitBuffer (hDevice, &Buffer);
             
             curr_state = 3;
             adi_tmr_SetLoadValue( hDevice1, GPT1_LOAD_2p5MSEC);
             adi_tmr_Enable(hDevice1,true);                         
            
         break;         
       
       /* st8 2 : CO sensor - heater and sense circuit OFF - wait for 14ms before taking next CO measurement */	
        case 3 :
             adi_gpio_SetLow(CO_HEATER);
             adi_gpio_SetLow(CO_SENSE);
             cnt_co_heater_cycles = 0;
             
             curr_state = 0;
             adi_tmr_SetLoadValue( hDevice1, GPT1_LOAD_14MSEC);
             adi_tmr_Enable(hDevice1,true); 
                      
         break; 

	/*st8 10: PM2.5 sensor FAN ON */	
        case 10 :
          /*Wait until 4 cycles of 500 ms are done - i.e., wait for fan to be ON for 2s [actually needs to be on for 10s as per sensor spec,
	   * but trying to save power by reducing fan ON time]*/
          if (cnt_fan_cycles == (NUM_FAN_500MS_CYCLES - 1))
          {
            curr_state = 4;  		  
	  }
	  else
	  {            
            curr_state = 10;
	  }
          cnt_fan_cycles++;
          adi_tmr_SetLoadValue( hDevice1, GPT1_LOAD_500MSEC);
          adi_tmr_Enable(hDevice1,true); 
          break;

	/*st8 4: PM2.5 sensor LED ON for 0.28ms */	
       case 4 : 
          
            if (cnt_samples >= CNT_SAMPLES_AVG) 
             {
              
               adi_tmr_Enable(hDevice1, false);
               adi_gpio_SetLow(PM25_LED);
               adi_gpio_SetLow(PM25_FAN);
               adi_gpio_SetLow(EN_5V);
               k=1;                                           
             }
            else
            {
              adi_gpio_SetHigh(PM25_LED);
              
	      curr_state      = 5;
              adi_tmr_SetLoadValue( hDevice1, GPT1_LOAD_0p28MSEC);
              adi_tmr_Enable(hDevice1,true); 
           }   
           break;         
	
	/*st8 5: PM2.5 sensor - trigger ADC sampling on channel-3 */	
        case 5 :
             /* Populate the buffer structure */
             Buffer.nBuffSize = sizeof(ADC_DataBuffer);
             Buffer.nChannels = ADI_ADC_CHANNEL_2;                                             
             Buffer.nNumConversionPasses = ADC_NUM_SAMPLES;
             Buffer.pDataBuffer = ADC_DataBuffer;
    
             /* Submit the buffer to the driver */
             adcResult = adi_adc_SubmitBuffer (hDevice, &Buffer);
           
	     curr_state = 6;  
             adi_tmr_SetLoadValue( hDevice1, GPT1_LOAD_0p04MSEC); 
             adi_tmr_Enable(hDevice1,true); 
             
         break;   

	/*st8 6: PM2.5 sensor - LED OFF, wait for 9.68 ms before next PM2.5 measurement */	
        case 6 :
             adi_gpio_SetLow(PM25_LED);
             
             curr_state = 4;  
             adi_tmr_SetLoadValue( hDevice1, GPT1_LOAD_9p68MSEC);
             adi_tmr_Enable(hDevice1,true);                       

         break;         
         
       default :
         break;
     }
  
        break;
      case ADI_TMR_EVENT_CAPTURED:
        break;
      default:
        break;
  }
  
}

/* Callback from the ADC device */

static void ADCCallback(void *pCBParam, uint32_t Event, void *pArg)
{
  
    switch (Event)
    {
    case ADI_ADC_EVENT_BUFFER_PROCESSED:
             tmp=0;
             do{   
             //DEBUG_MESSAGE("%d,%d\n",cnt_samples,ADC_DataBuffer[tmp]);
             tmp++;
             }while(tmp<4);
             cnt_samples++;
        break;

    default:
        break;
    }
}


void ADC_Setup(void)
{

    bool_t bCalibrationDone = false;	
    
    /* Open the ADC device */
    adcResult = adi_adc_Open(ADC_DEV_NUM, DeviceMemory, sizeof(DeviceMemory), &hDevice);
   // DEBUG_RESULT("Failed to open ADC device",adcResult, ADI_ADC_SUCCESS);

//#ifdef ADC_ENABLE_CALLBACK
    /* Register Callback */
    adcResult = adi_adc_RegisterCallback (hDevice, ADCCallback, NULL);
//#endif

    /* Power up ADC */
    adcResult = adi_adc_PowerUp (hDevice, true);
   // DEBUG_RESULT("Failed to power up ADC", adcResult, ADI_ADC_SUCCESS);

    /* Set ADC reference */
    adcResult = adi_adc_SetVrefSource (hDevice, ADI_ADC_VREF_SRC_INT_2_50_V);
    //adcResult = adi_adc_SetVrefSource (hDevice, ADI_ADC_VREF_SRC_EXT);   
    //DEBUG_RESULT("Failed to set ADC reference", adcResult, ADI_ADC_SUCCESS);

    /* Enable ADC sub system */
    adcResult = adi_adc_EnableADCSubSystem (hDevice, true);
  //  DEBUG_RESULT("Failed to enable ADC sub system", adcResult, ADI_ADC_SUCCESS);

    /* Wait for 5.0ms */
    usleep (5000);
    if(start==1) //purpose of start : ADC Calibration is done only once - as calibration coefficeints are retained in hibernate
    {
      /* Start calibration */
    adcResult = adi_adc_StartCalibration (hDevice);
   // DEBUG_RESULT("Failed to start calibration", adcResult, ADI_ADC_SUCCESS);

    /* Wait until calibration is done */
    while (!bCalibrationDone)
    {
        adcResult = adi_adc_IsCalibrationDone (hDevice, &bCalibrationDone);
       // DEBUG_RESULT("Failed to get the calibration status", adcResult, ADI_ADC_SUCCESS);
    }	
    }
    /* Set the delay time */
    adcResult = adi_adc_SetDelayTime ( hDevice, 0);
    //DEBUG_RESULT("Failed to set the Delay time ", adcResult, ADI_ADC_SUCCESS);

    /* Set the acquisition time. (Application need to change it based on the impedence) */
    adcResult = adi_adc_SetAcquisitionTime ( hDevice, 10);
    //DEBUG_RESULT("Failed to set the acquisition time ", adcResult, ADI_ADC_SUCCESS);	

    /* Populate the buffer structure */
    Buffer.nBuffSize = sizeof(ADC_DataBuffer);
    Buffer.nChannels = ADI_ADC_CHANNEL_0;
    Buffer.nNumConversionPasses = ADC_NUM_SAMPLES;
    Buffer.pDataBuffer = ADC_DataBuffer;
    
     adcResult = adi_adc_Enable (hDevice, true);
    /* Submit the buffer to the driver */
    
    //adi_gpio_SetHigh(DBG_ADC_PIN);
    adcResult = adi_adc_SubmitBuffer (hDevice, &Buffer);
    //DEBUG_RESULT("Failed to submit buffer ", adcResult, ADI_ADC_SUCCESS);
    


    
}
/* Approximately wait for minimum 1 usec */
static void usleep(uint32_t usec)
{
    volatile int y = 0;
    while (y++ < usec) {
        volatile int x = 0;
        while (x < 16) {
        x++;
        }
    }
}

ADI_RTC_RESULT rtc_Init (void) {

    /* callbacks */
    ADI_RTC_INT_TYPE  nInterrupts  = ADI_RTC_ALARM_INT;

    ADI_RTC_RESULT eResult;
        uint32_t nRtc1Count;
    
    
    do
    {
        eResult = adi_rtc_Open(RTC_DEVICE_NUM,aRtcDevMem0,ADI_RTC_MEMORY_SIZE,&hDeviceRTC);
       // DEBUG_RESULT("\n Failed to open the device %04d",eResult,ADI_RTC_SUCCESS);
        
        usleep(10);
        eResult = adi_rtc_RegisterCallback(hDeviceRTC,rtc0Callback,hDeviceRTC);
       // DEBUG_RESULT("\n Failed to open the device",eResult,ADI_RTC_SUCCESS);
        
        usleep(10);
        
        eResult = adi_rtc_SetTrim(hDeviceRTC,ADI_RTC_TRIM_INTERVAL,ADI_RTC_TRIM_VALUE,ADI_RTC_TRIM_DIRECTION);
       // DEBUG_RESULT("Failed to set the trim value",eResult,ADI_RTC_SUCCESS);
        
        usleep(10);
        
        eResult = adi_rtc_EnableTrim(hDeviceRTC,true);
       // DEBUG_RESULT("Failed to enable the trim",eResult,ADI_RTC_SUCCESS);
        
        usleep(10);
        
        eResult = adi_rtc_EnableAlarm(hDeviceRTC,true);
        //DEBUG_RESULT("Failed to enable the alram ",eResult,ADI_RTC_SUCCESS);
        
        usleep(10);
        
        eResult = adi_rtc_EnableInterrupts(hDeviceRTC, nInterrupts,true);
       // DEBUG_RESULT("Failed to enable the specified interrupts",eResult,ADI_RTC_SUCCESS);
      
        usleep(10);
       
          eResult = adi_rtc_GetCount(hDeviceRTC,&nRtc1Count);
   // DEBUG_RESULT("\n Failed to open the device",eResult,ADI_RTC_SUCCESS);
        
          usleep(10);
        
    eResult = adi_rtc_SetAlarm(hDeviceRTC,RTC_ALARM_OFFSET + nRtc1Count);
   // DEBUG_RESULT("\n Failed to open the device",eResult,ADI_RTC_SUCCESS);
     
        usleep(10);
    
        eResult = adi_rtc_Enable(hDeviceRTC,true);
       // DEBUG_RESULT("Failed to enable the device",eResult,ADI_RTC_SUCCESS);
       usleep(10);
       
    }while(0); 
    
    return(eResult);
}

ADI_RTC_RESULT rtc_UpdateAlarm (void) {
    
    ADI_RTC_RESULT eResult;
    uint32_t rtcCount;
    
     /* LED TOGGLE : CHECK FOR RTC ALARM UPDATE */
    //adi_gpio_Toggle(LED5); 
    
     /* Get current RTC count value*/
    eResult = adi_rtc_GetCount(hDeviceRTC,&rtcCount);
    
    /*Set RTC alarm at RTC_ALARM_OFFSET counts from the present RTC count value */
    eResult = adi_rtc_SetAlarm(hDeviceRTC,rtcCount+RTC_ALARM_OFFSET);
   
    return(eResult);    
}
/*
** EOF
*/

/*@}*/
