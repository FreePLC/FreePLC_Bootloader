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

#include "IH_Driver.h"

/*******************************************************************************
* Definitions
******************************************************************************/

/*******************************************************************************
* Prototypes
******************************************************************************/


/*******************************************************************************
* Variables
******************************************************************************/

typedef void (*void_func_t)();


AddressType address;

AddressType tmp_transpone;

AddressType  crc_res;

FCC0B_STR CommandObj;


/*******************************************************************************
* Code
******************************************************************************/


/*******************************************************************************
* WDT
******************************************************************************/


/*******************************************************************************
* UART
******************************************************************************/


void UART_Initialization(void)
{

  MODBUS_UART->BDH = ((UART_SBR>>8)&0x1f);
  MODBUS_UART->BDL = (UART_SBR&0xff);
  MODBUS_UART->C4  = (UART_BRFA&0x1f);  
  MODBUS_UART->C2  = UART_C2_TE_MASK|UART_C2_RE_MASK;

  while(UART_IsChar(MODBUS_UART))
  {
	(void)UART_GetChar(MODBUS_UART);
  }
    
}   



/*******************************************************************************
* JumpToUserApplication
******************************************************************************/

__ramfunc void JumpToUserApplication(unsigned long userSP, unsigned long userStartup)
{
  // set up stack pointer
  __asm("msr msp, r0");
  __asm("msr psp, r0");
  
  // Jump to PC (r1)
  __asm("BLX r1"); 
}

/*******************************************************************************
* ReadAddress
******************************************************************************/

void ReadAddress(void)
{
  address.Bytes.hh = UART_GetChar(MODBUS_UART);
  address.Bytes.hl = UART_GetChar(MODBUS_UART);
  address.Bytes.lh = UART_GetChar(MODBUS_UART);
  address.Bytes.ll = UART_GetChar(MODBUS_UART);
}



void Boot_ResetMCU(void)
{ 
  SCB->AIRCR = SCB_AIRCR_VECTKEY(0x5FA) | SCB_AIRCR_SYSRESETREQ_Msk;
  while(1)
  {};   
}


void SendResult(unsigned long res)
{ 
  if(!res)
    UART_PutChar(MODBUS_UART,BOOT_CMD_ACK);
  else
    UART_PutChar(MODBUS_UART,BOOT_CMD_NACK);
}



void UART_PutChar(UART_Type *base, unsigned char data)
{
  unsigned int timeout = 0, flag = 1;
  
  while((base->S1 & UART_S1_TC_MASK) == 0)
  {
    timeout++;
    
    if(timeout > 0xFFFFFFF)
    {	flag = 0;
    break;
    }
  }
  if(flag)
  {
    base->D = data;
  }
  
}

unsigned char UART_GetChar(UART_Type *base)
{
  unsigned char ret = 0;
  
  while(UART_IsChar(MODBUS_UART) == 0)
  {
    
#if BOOTLOADER_INT_WATCHDOG == 1
    WDOG_Refresh(); /* feeds the dog */
#endif
    
  };
  ret = base->D;
  return ret;
}

unsigned char UART_IsChar(UART_Type *base)
{
  unsigned char ret = 0;
  
  ret = base->S1 & UART_S1_RDRF_MASK;
  
  return ret;
}




/*****************************************************************
*Flash
*****************************************************************/


/********************************************************
* Function for Programming of one Long Word 
*
********************************************************/
uint64_t FLASH_ProgramLongWord(uint64_t destination, uint64_t data32b)
{
  /* preparing passing parameter to program the flash block */
  
  CommandObj.regsLong.fccob3210 = destination;
  CommandObj.regs.fccob0 = PGM4;
  CommandObj.regsLong.fccob7654 = data32b; 
  
  return FLASH_FlashCommandSequenceStart(PROGRAM_LONGWORD_INDEX);

}

/********************************************************
* Function for Programming of one Long Word 
*
********************************************************/
uint64_t FLASH_ProgramPhrase(uint64_t destination, uint64_t * data64b)
{
  /* preparing passing parameter to program the flash block */
  
  CommandObj.regsLong.fccob3210 = destination;
  CommandObj.regs.fccob0 = PGM8;
  CommandObj.regsLong.fccob7654 = data64b[0];
  CommandObj.regsLong.fccobBA98 = data64b[1];
  
  return FLASH_FlashCommandSequenceStart(PROGRAM_PHRASE_INDEX);
}

/********************************************************
* Function for Programming of section by simple longs
*
********************************************************/
uint64_t FLASH_ProgramSectionByLongs(uint64_t destination, uint64_t* pSource, uint64_t size)
{ 
  while(size--)
  {
    if(FLASH_ProgramLongWord(destination, *pSource++) != FLASH_OK)
      return FLASH_FAIL;
    destination += 4;
  }
  return FLASH_OK;
}




/********************************************************
* Function for erasing of flash memory sector (0x800)
*
********************************************************/
uint64_t FLASH_EraseSector(uint64_t destination)
{  
  CommandObj.regsLong.fccob3210 = destination;
  CommandObj.regs.fccob0 = ERSSCR;

  return FLASH_FlashCommandSequenceStart(ERASE_BLOCK_INDEX);
}


uint64_t FLASH_ReadOnce(uint8_t pDataArray[], uint8_t index)
{
	CommandObj.regs.fccob3= 0;
	CommandObj.regs.fccob2= 0;
	CommandObj.regs.fccob1= index;
	CommandObj.regs.fccob0 = RDONCE;
	FLASH_FlashCommandSequenceStart(ERASE_BLOCK_INDEX);
	pDataArray[3] = FTFA->FCCOB4;
	pDataArray[2] = FTFA->FCCOB5;
	pDataArray[1] = FTFA->FCCOB6;
	pDataArray[0] = FTFA->FCCOB7;
	
	pDataArray[7] = FTFA->FCCOB8;
	pDataArray[6] = FTFA->FCCOB9;
	pDataArray[5] = FTFA->FCCOBA;
	pDataArray[4] = FTFA->FCCOBB;
	return 0;

}

uint64_t FLASH_ProgramOnce(uint8_t pDataArray[],uint8_t index)
{
	CommandObj.regs.fccob0 = PGMONCE;
	CommandObj.regs.fccob1= index;
	CommandObj.regs.fccob2= 0;
	CommandObj.regs.fccob3= 0;

	CommandObj.regs.fccob4= pDataArray[3];
	CommandObj.regs.fccob5= pDataArray[2];
	CommandObj.regs.fccob6= pDataArray[1];
	CommandObj.regs.fccob7= pDataArray[0];
	
	CommandObj.regs.fccob8= pDataArray[7];
	CommandObj.regs.fccob9= pDataArray[6];
	CommandObj.regs.fccobA= pDataArray[5];
	CommandObj.regs.fccobB= pDataArray[4];
	
	return FLASH_FlashCommandSequenceStart(PROGRAM_PHRASE_INDEX);



}



uint64_t FLASH_EraseEASector(uint64_t destination)
{  
  CommandObj.regs.fccob0 = ERSXA;
  FlashSignoff();
  return FLASH_FlashCommandSequenceStart(ERASE_EXE_BLOCK_INDEX);
}


__ramfunc uint64_t FLASH_FlashCommandSequenceStart(uint8_t index)
{
  uint8_t* ptrFccobReg = (uint8_t*)&FTFA->FCCOB3;
  uint8_t* ptrCommandObj = (uint8_t*)&CommandObj;

  /* wait till CCIF bit is set */
  while(!(FTFA->FSTAT & FTFA_FSTAT_CCIF_MASK)){};
  /* clear RDCOLERR & ACCERR & FPVIOL flag in flash status register */
  FTFA->FSTAT = FTFA_FSTAT_ACCERR_MASK | FTFA_FSTAT_FPVIOL_MASK | FTFA_FSTAT_RDCOLERR_MASK;  
  
  /* load FCCOB registers */  
  while(index--)
    *ptrFccobReg++ = *ptrCommandObj++;
  
  //  launch a command 
  FTFA->FSTAT |= FTFA_FSTAT_CCIF_MASK; 
  //  waiting for the finishing of the command
  while(!(FTFA->FSTAT & FTFA_FSTAT_CCIF_MASK)){};
  
   /* Check error bits */
  /* Get flash status register value */
  return (FTFA->FSTAT & (FTFA_FSTAT_ACCERR_MASK | FTFA_FSTAT_FPVIOL_MASK | FTFA_FSTAT_MGSTAT0_MASK));  
} 

uint32_t FlashSignoff(void)
{
  /*Cache Invalidate all four ways
    Bank 0*/
  FMC->PFB0CR = FMC_PFB0CR_CINV_WAY(0xF);
  /*Invalidate (clear) specification buffer and page buffer
    Bank 0*/
  FMC->PFB0CR |= FMC_PFB0CR_S_B_INV_MASK;

  return FLASH_OK;
}





