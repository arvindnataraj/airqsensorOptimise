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

#define preScale (15u)
#define fanOnTime (2u)                                    //Enter required fanOnTime in seconds
#if defined ( __ICCARM__ )
    #define ALIGN4 _Pragma("data_alignment=4")
#elif defined (__CC_ARM)
    #define ALIGN4 __align(4)
#else
    #pragma message("WARNING: NO ALIGHMENT DEFINED FOR MEMORY")
#endif
static const uint16_t GPT1_LOAD_490MSEC  = 49779;
static const uint16_t GPT1_LOAD_500MSEC  = 50795;//Calculation for LOAD value: 50795 / (26MHz/256) = 500 ms
static const uint16_t GPT1_LOAD_1SEC     = 25391;
static const uint16_t GPT1_LOAD_2SEC     = 50782;
static const uint16_t GPT1_LOAD_0p28MSEC = 24;
static const uint16_t GPT1_LOAD_0p04MSEC = 1;
static const uint16_t GPT1_LOAD_9p68MSEC = 979;
static const uint16_t GPT1_LOAD_981MSEC  = 24525;
static const uint16_t GPT1_LOAD_2p5MSEC  = 249;
static const uint16_t GPT1_LOAD_100MSEC  = 10155;
static const uint16_t GPT1_LOAD_50MSEC  = 5050;
static const uint16_t GPT1_LOAD_14MSEC   = 1418;


static volatile bool_t bTimeOutFlag;

static uint8_t aDeviceMemory1[ADI_TMR_MEMORY_SIZE];
static uint8_t gpioMemory[ADI_GPIO_MEMORY_SIZE];
static uint8_t aRtcDevMem0[ADI_RTC_MEMORY_SIZE];


/* Binary flag to indicate RTC interrupt occured */
volatile bool_t bRtcInterrupt;
/* Binary flag to indicate specified number of  RTC interrupt occured */
volatile bool_t bRtcAlarmFlag;
/* Binary flag which is set to true when processor comes out of hibernation */
volatile bool_t bHibernateExitFlag;
/* Alram count*/
volatile uint32_t AlarmCount;

ADI_TMR_HANDLE hDevice1;
ADI_RTC_HANDLE hDeviceRTC  = NULL;

ADI_TMR_RESULT result;
ADI_GPIO_RESULT eResult;

//EZKIT - LED_3 = P0.13, CUSTOM PCB V1 - LED3 = P0.3
//#define LED_3         ADI_GPIO_PORT0, ADI_GPIO_PIN_13
#define LED3         ADI_GPIO_PORT0, ADI_GPIO_PIN_4 //intentionally mapped to P0.4 which is NOT lED on custom board as wanted to free up P0.3 for DBG_ST8_PIN.
#define LED4         ADI_GPIO_PORT1, ADI_GPIO_PIN_12
#define LED5         ADI_GPIO_PORT1, ADI_GPIO_PIN_13
//#define EN_5V         ADI_GPIO_PORT1, ADI_GPIO_PIN_14

/*EZKIT*/
/*CO sensor heater control: P2.0 / J9-7*/
#define CO_HEATER    ADI_GPIO_PORT2, ADI_GPIO_PIN_0
/*CO sensor sense control: P2.2 / J9-6*/
#define CO_SENSE     ADI_GPIO_PORT2, ADI_GPIO_PIN_2
/*PM2.5 sensor LED control: P0.15 / J9-3*/
#define PM25_LED     ADI_GPIO_PORT0, ADI_GPIO_PIN_15
/*PM2.5 sensor FAN control: P2.11 / J9-4*/
#define PM25_FAN     ADI_GPIO_PORT2, ADI_GPIO_PIN_11

/*PCB*/
///*CO sensor heater control: P1.7*/
//#define CO_HEATER    ADI_GPIO_PORT1, ADI_GPIO_PIN_7
///*CO sensor sense control: P1.6*/
//#define CO_SENSE     ADI_GPIO_PORT1, ADI_GPIO_PIN_6
///*PM2.5 sensor LED control: P2.9*/
//#define PM25_LED     ADI_GPIO_PORT2, ADI_GPIO_PIN_9
//*PM2.5 sensor FAN control: P1.8*/
//#define PM25_FAN     ADI_GPIO_PORT1, ADI_GPIO_PIN_8

/*EZKIT: Debug - st8 m/c toggle pin: P0.5 / J6-9*/
#define DBG_ST8_PIN  ADI_GPIO_PORT0, ADI_GPIO_PIN_5
/*EZKIT: Debug - ADC st8 pin: P0.4 / J6-10*/
#define DBG_ADC_PIN  ADI_GPIO_PORT0, ADI_GPIO_PIN_4

/*PCB*/
///*Debug - st8 m/c toggle pin: P0.3*/
//#define DBG_ST8_PIN  ADI_GPIO_PORT0, ADI_GPIO_PIN_3
///*Debug - ADC st8 pin: P2.11*/
//#define DBG_ADC_PIN  ADI_GPIO_PORT2, ADI_GPIO_PIN_11

static uint8_t curr_state;
static uint8_t cnt_samples;

static uint8_t cnt_fan_cycles;
static uint8_t cnt_co_heater_cycles;

#define ST8_PM_FAN  10;



static void usleep(uint32_t usec);

#define RTC_ALARM_OFFSET   10
#define CNT_SAMPLES_AVG    1

int NUM_FAN_500MS_CYCLES = 0;

int tmp=0,k=0,start=0;

ADI_PWR_RESULT pwrResult;
bool_t flagHib = false;                                        // flag for hibernate mode


int main(void)
{
     
    /* Clock initialization */
    SystemInit();

    
    /* test system initialization */
    test_Init();
        //adi_gpio_OutputEnable(EN_5V, true);
        //adi_gpio_SetHigh(EN_5V);
    
    NUM_FAN_500MS_CYCLES = fanOnTime/0.5;//Number of 500ms cycles equals ratio of given fanOnTime to 0.5    
    
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
      //P0.13 --> LED3
     // adi_gpio_OutputEnable(LED3, true);
      //P1.12 --> LED4 
      adi_gpio_OutputEnable(LED4, true);
      adi_gpio_OutputEnable(LED5, true);
      adi_gpio_OutputEnable(CO_HEATER, true);
      adi_gpio_OutputEnable(CO_SENSE, true);
      adi_gpio_OutputEnable(PM25_LED, true);
      adi_gpio_OutputEnable(PM25_FAN, true);
      //adi_gpio_OutputEnable(DBG_ST8_PIN, true);
      //adi_gpio_OutputEnable(DBG_ADC_PIN, true);

      adi_gpio_SetLow(CO_HEATER);
      adi_gpio_SetLow(CO_SENSE);
      adi_gpio_SetLow(PM25_LED);
      adi_gpio_SetLow(PM25_FAN);
     // adi_gpio_SetLow(DBG_ADC_PIN);   
     // adi_gpio_SetHigh(LED3);
      //adi_gpio_SetHigh(LED4);
      
      
      adi_tmr_Open(TIMER_DEVICE_1,aDeviceMemory1,ADI_TMR_MEMORY_SIZE,&hDevice1); 
      adi_tmr_RegisterCallback( hDevice1, GPTimer1Callback ,hDevice1);
      
      adi_tmr_SetPrescaler(hDevice1, ADI_GPT_PRESCALER_256);
      adi_tmr_SetLoadValue( hDevice1, GPT1_LOAD_1SEC);
     
      //DEBUG_MESSAGE("AQ Sensor initializing!\n");    
        
    }while(0);
    
Hibernate :
    flagHib = false;
   
    pwrResult = adi_pwr_EnterLowPowerMode(ADI_PWR_MODE_HIBERNATE,&flagHib,0x00);  //Entering hibernate mode
    //DEBUG_RESULT("\n Failed to enter hibernate %04d",pwrResult,ADI_PWR_SUCCESS);
  do
  {
    if(k==1)
     {
       //if(start==16)
          //adi_gpio_Toggle(LED5); 
       start=0;
      goto Hibernate;// Once one cycle of measurements completes - jumps to label 'Hibernate' to switch to hibernate mode till next RTC alarm
     } 
   }
   while(1);
    
    
 
}
/* RTC-0 Callback handler */
/*This callback initializes the GPT1 timer, which controls the sensor st8-mc.*/
void rtc0Callback (void *pCBParam, uint32_t nEvent, void *EventArg) {
   
  /* Clear RTC Interrupt status for the next RTC interrupt */
   adi_rtc_ClearInterruptStatus(hDeviceRTC,ADI_RTC_ALARM_INT);
    
    adi_gpio_Toggle(ADI_GPIO_PORT1, ADI_GPIO_PIN_12);  
  
  
    /* flagHib is set to 'true' in adi_pwr_ExitLowPowerMode() to exit hibernate mode */
    flagHib = true;
    pwrResult = adi_pwr_ExitLowPowerMode(&flagHib);  
  
    //DEBUG_RESULT("\n Failed to exit hibernate %04d",pwrResult,ADI_PWR_SUCCESS);
   
    k = 0;
     
  /* Components of ADC are powered down in hibernate mode - so must be explicitly powered up after waking up*/
    if(pwrResult==ADI_PWR_SUCCESS)
    {
     k = 0;
    
    }
 
    bRtcInterrupt = true;

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

	/*Initialize debug pins to 0*/
        //adi_gpio_SetLow(DBG_ST8_PIN);
        //di_gpio_SetLow(DBG_ADC_PIN);
        
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
  //if(start==0){ adi_gpio_Toggle(LED5);}        //LED 5 - Toggle
  start++;
  //if(start==15){adi_gpio_Toggle(LED5);}
  
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
               //adi_gpio_SetHigh(LED3);
               //adi_gpio_SetHigh(LED4);
               
	       adi_gpio_SetLow(CO_HEATER);
               adi_gpio_SetLow(CO_SENSE);

               adi_gpio_SetHigh(PM25_FAN);
               adi_tmr_SetLoadValue( hDevice1, GPT1_LOAD_500MSEC);
               adi_tmr_Enable(hDevice1,true); 
               
             }
            else
            {
              adi_gpio_SetHigh(CO_HEATER);
              
	     //adi_gpio_SetLow(LED3);
              //adi_gpio_Toggle(DBG_ST8_PIN);

              /*Wait until 2 heater-cycles of 490 ms are done - i.e., wait for heater to be ON for 980 ms as per sensor spec.
	        (limitation of GPT1 clock frequency - cannot count 980 ms in one go)*/
	      if (cnt_co_heater_cycles == 1)
              {
                curr_state = 1;  		  
	      }
	      else
	      {            
                curr_state = 0;
	      }
              cnt_co_heater_cycles++;
              adi_tmr_SetLoadValue( hDevice1, GPT1_LOAD_490MSEC);
              adi_tmr_Enable(hDevice1,true); 
           }   
           break;
         
       /* st8 1 : CO sensor - sense circuit ON for 2.5 ms*/	
       case 1 :
               adi_gpio_SetHigh(CO_SENSE);
               //adi_gpio_Toggle(DBG_ST8_PIN);
                                              
               curr_state = 2;
               adi_tmr_SetLoadValue( hDevice1, GPT1_LOAD_2p5MSEC);    
               adi_tmr_Enable(hDevice1,true); 
              
         break;
         
       /* st8 2 : CO sensor - trigger ADC sampling on channel-2 */	
       case 2 :
         
            // adi_gpio_Toggle(DBG_ST8_PIN);
             //adi_gpio_SetHigh(DBG_ADC_PIN);
             //adi_gpio_SetLow(LED4);
             
                                         //adi_gpio_Toggle(LED5);
             cnt_samples++;
             curr_state = 3;
             adi_tmr_SetLoadValue( hDevice1, GPT1_LOAD_2p5MSEC);
             adi_tmr_Enable(hDevice1,true);                         
            
         break;         
       
       /* st8 2 : CO sensor - heater and sense circuit OFF - wait for 14ms before taking next CO measurement */	
        case 3 :
             adi_gpio_SetLow(CO_HEATER);
             adi_gpio_SetLow(CO_SENSE);
             
             //adi_gpio_SetHigh(LED3);
            // adi_gpio_Toggle(DBG_ST8_PIN);
             
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
               adi_gpio_SetHigh(LED3);
               //adi_gpio_SetHigh(LED4);
               adi_gpio_SetLow(PM25_LED);
               adi_gpio_SetLow(PM25_FAN);
               k=1;
               
               //cnt_samples = 0;                                                              //change
             }
            else
            {
              adi_gpio_SetHigh(PM25_LED);
              
	     // adi_gpio_SetLow(LED3);
             // adi_gpio_Toggle(DBG_ST8_PIN);

              curr_state      = 5;
              adi_tmr_SetLoadValue( hDevice1, GPT1_LOAD_0p28MSEC);
              adi_tmr_Enable(hDevice1,true); 
           }   
           break;         
	
	/*st8 5: PM2.5 sensor - trigger ADC sampling on channel-3 */	
        case 5 :
          
            // adi_gpio_Toggle(DBG_ST8_PIN);
            // adi_gpio_SetHigh(DBG_ADC_PIN);
             
                                             
             cnt_samples++;
	     curr_state = 6;  
             adi_tmr_SetLoadValue( hDevice1, GPT1_LOAD_0p04MSEC); 
             adi_tmr_Enable(hDevice1,true); 
             
         break;   

	/*st8 6: PM2.5 sensor - LED OFF, wait for 9.68 ms before next PM2.5 measurement */	
        case 6 :
             adi_gpio_SetLow(PM25_LED);
                  
            // adi_gpio_SetHigh(LED3);
            // adi_gpio_Toggle(DBG_ST8_PIN);

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
     
        // eResult = adi_rtc_SetPreScale(hDeviceRTC,preScale);                                   //change
        
        usleep(10);
    
        eResult = adi_rtc_Enable(hDeviceRTC,true);
       // DEBUG_RESULT("Failed to enable the device",eResult,ADI_RTC_SUCCESS);
        
        usleep(10);
       
    }while(0); 
    
    return(eResult);
}

ADI_RTC_RESULT rtc_UpdateAlarm (void) {
 
    uint32_t rtcCount;
    adi_gpio_Toggle(LED5); 
     /* Get current RTC count value*/
    adi_rtc_GetCount(hDeviceRTC,&rtcCount);
       
    /*Set RTC alarm at RTC_ALARM_OFFSET counts from the present RTC count value */
    adi_rtc_SetAlarm(hDeviceRTC,rtcCount+RTC_ALARM_OFFSET);
     
}
/*
** EOF
*/

/*@}*/
