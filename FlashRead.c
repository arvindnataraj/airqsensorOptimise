/*=============  I N C L U D E S   =============*/

/* Managed drivers and/or services include */

#include "common.h"
#include "system.h"
#include "services/pwr/adi_pwr.h"

uint32_t PAGE_ADDR = 0x8000;
//#define BUFF_SIZE  (8)
void main (void)
{

    /* Initialize managed drivers and/or services */
    SystemInit();

    /* test system initialization */
    test_Init();

    if(adi_pwr_Init()!= ADI_PWR_SUCCESS)
    {
        DEBUG_MESSAGE("\n Failed to intialize the power service \n");
    }
    if(ADI_PWR_SUCCESS != adi_pwr_SetClockDivider(ADI_CLOCK_HCLK,1))
    {
        DEBUG_MESSAGE("Failed to intialize the power service\n");
    }
    if(ADI_PWR_SUCCESS != adi_pwr_SetClockDivider(ADI_CLOCK_PCLK,1))
    {
        DEBUG_MESSAGE("Failed to intialize the power service\n");
    }
  
   while(PAGE_ADDR<=0x8430)
   {
     uint16_t* pDataInFlash = (uint16_t*)PAGE_ADDR;
    for (int x=0; x<4; x++)
       {
         DEBUG_MESSAGE("%d",pDataInFlash[x]);
       }
    PAGE_ADDR+=16;
   }
}
