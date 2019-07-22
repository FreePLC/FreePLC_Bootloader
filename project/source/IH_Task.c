/*
* The Clear BSD License
* Copyright (c) 2015, Freescale Semiconductor, Inc.
* Copyright 2016-2017 NXP
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without modification,
* are permitted (subject to the limitations in the disclaimer below) provided
* that the following conditions are met:
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
* NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS LICENSE.
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


#include "IH_Task.h"
#include "IH_Driver.h"
#include "FAC_Port.h"
/*******************************************************************************
* Definitions
******************************************************************************/



/*******************************************************************************
* Prototypes
******************************************************************************/


extern void FAC_Prot(const uint8_t fac_list[], int len);

/*******************************************************************************
* Variables
******************************************************************************/
//设置需要保护的区域，最大支持64个区，每个区代表256K/64 = 4K
const uint8_t fac_list[8] = 
{1, 2, 3, 4, 5, 6, 7, 8};



extern AddressType address;
unsigned long length;
unsigned char write_buffer[256];

//extern uint8_t g_ucMBUF[];
unsigned long timeout_cnt;
unsigned long enableBootMode;



BootloaderIdentOptimType bootloaderIdent = 
{
  /** count of flash blocks */
  LITTLE2BIG(FLASH_BLOCK_CNT),
  /** flash blocks descritor */
  {LITTLE2BIG(USER_FLASH_START), LITTLE2BIG(USER_FLASH_END)},
#if FLASH_BLOCK_CNT == 2
  {LITTLE2BIG(USER_FLASH_START_2), LITTLE2BIG(USER_FLASH_END_2)},
#endif   
  /** Relocated interrupts vestor table */
  LITTLE2BIG(RELOCATED_VECTORS),
  /** Interrupts vestor table */
  LITTLE2BIG(INTERRUPT_VECTORS),
  /** Erase Block Size */
  LITTLE2BIG(FLASH_ERASE_PAGE),
  /** Write Block Size */
  LITTLE2BIG(FLASH_WRITE_PAGE),
  /** Id string */
  KINETIS_MODEL_STR      
};





/*******************************************************************************
* Code
******************************************************************************/


void IH_Task()
{
  //unsigned long systick_cnt;
  unsigned int i, j, flag;
  unsigned char getcommand = 0;
  
  // initialize variables
  timeout_cnt = 0;
  enableBootMode = 0;
  
  INIT_CLOCKS_TO_MODULES;
  
  if(enableBootMode)
  {
    BOOT_PIN_INIT_AS_UART;
    
    UART_Initialization(); 
    
    SysTick->LOAD = (unsigned long)(0.01 * BOOT_BUS_CLOCK); // to do add macro to define the initial timeout
    SysTick->CTRL = (SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk);    
    
    while(1)
    {
      
#if BOOTLOADER_INT_WATCHDOG == 1
      WDOG_Refresh(); /* feeds the dog */
#endif  
      
      if(SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk)
      {
        SysTick->VAL = 0; // Finish the clear of the SysTick CountFlag
        
        // increment timeout value
        
        timeout_cnt++;
        
        if((timeout_cnt & 0x1F) == 0x10)
        {
          SendResult(0);
        }
        
        if((timeout_cnt > BOOT_WAITING_TIMEOUT))
        {
          break;
        }
        
      }
      
      // check the UART activity
      if(UART_IsChar(MODBUS_UART))
      {
        int retry = 0;
        unsigned char getch = 0;
        
        getch = UART_GetChar(MODBUS_UART);
        if(getch != BOOT_CMD_ACK)
        {
          retry++;
        }   
        else
        {
          enableBootMode++;
          SendResult(0);
          break;
        }
        
        if(retry > RETRY_TIMES)
        {
          Boot_ResetMCU();
        }
        
      }  
    }
    
    //  Bootloader protocol runs !
    while(enableBootMode > 1)
    {
      // never ending loop - except quit commnad that disable  enableBootMode
      timeout_cnt = 0;
      
      //如果第一次收到的FC是个异常数，则通过判断PC是否发送'I'命令来超时退出
      if((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk)&&(getcommand == 0))
      {
        SysTick->VAL = 0; // Finish the clear of the SysTick CountFlag
        
        // increment timeout value
        
        timeout_cnt++;		
        
        if((timeout_cnt > 100) )
        {
          break;
        }
        
      }
      
#if BOOTLOADER_INT_WATCHDOG == 1
      WDOG_Refresh(); /* feeds the dog */
#endif
      
      
      switch(UART_GetChar(MODBUS_UART))
      {
      case BOOT_CMD_QUIT:                                 // QUIT command
        
        // send the ack
        SendResult(0);
        // reset MCU
        Boot_ResetMCU();
        
      case BOOT_CMD_ACK:
        SendResult(0);                                   // ACK command
        break;
        
      case BOOT_CMD_IDENT:                               // IDENT command
        //and send the all data with/without CRC 
        getcommand = 1;
        
        UART_PutChar(MODBUS_UART, (BootloaderProtocolType)_BOOTLOADER_PROTOCOL_VERSION);
        
        
        
        UART_PutChar(MODBUS_UART, SIM->SDID >> 8); // high
        //UART_PutChar(MODBUS_UART, 0xC1); // high           
        
        
        
        UART_PutChar(MODBUS_UART, SIM->SDID);  // low
        //UART_PutChar(MODBUS_UART, 0x4A);  // low
        
        for(i=0; i < sizeof(bootloaderIdent); i++)
        {
          UART_PutChar(MODBUS_UART, ((unsigned char*)&bootloaderIdent)[i]);
        }
        
        
        break;
        
      case BOOT_CMD_ERASE:                                // ERASE command
        // Read Address
        ReadAddress();
        // Check the CRC
        // Erase the flash and send result
        //yangliang
        
        flag = FAC_Verify(address.complete);
        
        if(flag)
        {
          SendResult(FLASH_EraseSector(address.complete));
        }
        else
        {
          SendResult(FLASH_EraseEASector(address.complete));
        }
        
        
        break;
        
      case BOOT_CMD_EALOCK:
        {
          FAC_Prot(fac_list, sizeof(fac_list));
          
        }
        break;
        
      case BOOT_CMD_WRITE:
        // Read Address                                   
        ReadAddress();
        // Read length
        length = UART_GetChar(MODBUS_UART);
        
        // Load the data to write
        for(i = 0;i<length; i++)
        {
          write_buffer[i] =  UART_GetChar(MODBUS_UART); 
        }
        
        
        if((length % 4) == 0)
        {
          length >>= 2;    // divide by four
        }
        else
        {
          
          for(j = 0; j < (4 - (length % 4)); j++)
          {
            write_buffer[i + j] =  0xFF;
          }
          length >>= 2;    // divide by four
          length += 1;
        }
        
        
        // check the CRC of input data
        
        //yangliang 
        if(FLASH_ProgramSectionByLongs(address.complete, (uint64_t *)write_buffer, length))
        {            
          SendResult(1);
          break;
        }
        
        // Verify flashed data (if enabled)
#if BOOTLOADER_ENABLE_VERIFY == 1                 
        for(i = 0;i<length; i++)
        {              
          if(((unsigned long*)write_buffer)[i] != ((unsigned long*)address.complete)[i])
          {
            SendResult(1);
            break;
          }
        }
#endif    
        SendResult(0);
        break;
        
#if BOOTLOADER_ENABLE_READ_CMD == 1       
        
      case BOOT_CMD_READ:                                   // READ comamnd    
        ReadAddress();
        length = UART_GetChar(MODBUS_UART);
        
        for(i = 0;i<length; i++)
        {
          UART_PutChar(MODBUS_UART, ((unsigned char*)(address.complete))[i]); 
          
#if BOOTLOADER_INT_WATCHDOG == 1
          WDOG_Refresh(); /* feeds the dog */
#endif
        }
        
        
        break;
        
#endif    
        
      }      
    } 
  }
  
  MODBUS_UART->C2 = 0;
  
  DINIT_CLOCKS_TO_MODULES;
  
  SCB->VTOR = APP_VEC_ADDR;
  
  JumpToUserApplication(APP_STACK, APP_ENTRY);
  
  
}


