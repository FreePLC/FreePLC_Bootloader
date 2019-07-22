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
#ifndef _FSL_IH_DRIVER_H_
#define _FSL_IH_DRIVER_H_



#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include <stddef.h>

#include "fsl_device_registers.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
//Clock

#define BOOT_BUS_CLOCK        (32768*640)             


//UART

#define BOOT_UART_BAUD_RATE  115200 
#define MODBUS_UART UART1
#define UART_SBR  (unsigned char)(BOOT_BUS_CLOCK / (16*BOOT_UART_BAUD_RATE))
#define UART_BRFA (unsigned char)((((BOOT_BUS_CLOCK/(16*BOOT_UART_BAUD_RATE))- \
					  UART_SBR)*32.0)+0.5)

//FLASH

#define FLASH_8K    (1<<13)
#define FLASH_16K   (1<<14)
#define FLASH_32K   (1<<15)
#define FLASH_64K   (1<<16)
#define FLASH_128K  (1<<17)
#define FLASH_256K  (1<<18)
#define FLASH_512K  (1<<19)
#define FLASH_1024K (1<<20)
#define FLASH_2048K (1<<21)

#define VERSION_HC08        1
#define VERSION_HC08_LARGE  3
#define VERSION_HCS08       2 
#define VERSION_HCS08_LONG  6
#define VERSION_HCS08_LARGE 10
#define VERSION_CV       4
#define VERSION_KINETIS  8
  
  /** Bootloader commands */
#define BOOT_CMD_IDENT 'I'
#define BOOT_CMD_WRITE 'W'
#define BOOT_CMD_ERASE 'E'
#define BOOT_CMD_ACK  0xFC
#define BOOT_CMD_NACK 0x03
#define BOOT_CMD_QUIT 'Q'
#define BOOT_CMD_READ 'R'

#define BOOT_CMD_EALOCK	 'L'

#define BOOT_CMD_EAERASE 'A'


/** Kinetis Flash memory size */

#define KINETIS_FLASH FLASH_256K

/** Boot timeout after POR (Power On Reset) for wait to connect Master **/
/** BOOT_WAITING_TIMEOUT * 10ms **/
#define BOOT_WAITING_TIMEOUT 1000


#define RD1SEC    0x01
#define PGMCHK    0x02
#define RDRSRC    0x03
#define PGM4      0x06
#define PGM8	  0x07
#define ERSSCR    0x09
#define RD1ALL    0x40
#define RDONCE    0x41
#define PGMONCE   0x43
#define ERSALL    0x44
#define VFYKEY    0x45
#define RD1XA     0x4A
#define ERSXA	  0x4B
#define NORMAL_LEVEL 0x0

#define ERASE_EXE_BLOCK_INDEX   1
#define ERASE_BLOCK_INDEX       4
#define PROGRAM_LONGWORD_INDEX  8
#define PROGRAM_PHRASE_INDEX    12

			  

#define FCCOB_REGS  12
#define FLASH_OK     0
#define FLASH_FAIL   1




/* Flash block count of this MCU */
#define FLASH_BLOCK_CNT 1

/* Start address of interrupt vector table */ 
#define INTERRUPT_VECTORS 0x0000

/* Start address of relocated interrutp vector table */
#define RELOCATED_VECTORS 0x4000 

/* Flash start address */
#define USER_FLASH_START RELOCATED_VECTORS

/* Flash end address */
#define USER_FLASH_END (KINETIS_FLASH - 1)

/* Flash2 start address */
//#define USER_FLASH_START_2 0x00040000

/* Flash2 end address */
//#define USER_FLASH_END_2 0x0005FFFF

/* Size of write block */
#define FLASH_WRITE_PAGE 128

/* Size of erase block */
#define FLASH_ERASE_PAGE 4096

#define KINETIS_MODEL_STR "KV4"
  

#define RETRY_TIMES    100

#define BOOTLOADER_PROTOCOL_VERSION VERSION_KINETIS


#define INIT_CLOCKS_TO_MODULES	  SIM->SCGC4 |= (SIM_SCGC4_UART1_MASK ); \
								  SIM->SCGC5 |= (SIM_SCGC5_PORTE_MASK); \
								  SIM->SCGC6 |= SIM_SCGC6_FTF_MASK;

#define DINIT_CLOCKS_TO_MODULES	  SIM->SCGC4 &= ~(SIM_SCGC4_UART1_MASK ); \
								  SIM->SCGC5 &= ~(SIM_SCGC5_PORTE_MASK); 


/** GPIO & UART pins initialization */

#define BOOT_UART_GPIO_PORT PORTE

/*  setting of multiplexer for UART alternative of pin */
#define BOOT_PIN_UART_ALTERNATIVE 3

/*  setting of multiplexer for GPIO alternative of pin */
#define BOOT_PIN_GPIO_ALTERNATIVE 1

#define BOOT_UART_GPIO_PIN_RX   1  

#define BOOT_UART_GPIO_PIN_TX   0  


//  init macro for multiplexer setting - alternative UART
#define BOOT_PIN_INIT_AS_UART BOOT_UART_GPIO_PORT->PCR[BOOT_UART_GPIO_PIN_RX] |= PORT_PCR_MUX(BOOT_PIN_UART_ALTERNATIVE); \
							  BOOT_UART_GPIO_PORT->PCR[BOOT_UART_GPIO_PIN_TX] |= PORT_PCR_MUX(BOOT_PIN_UART_ALTERNATIVE);


#if (BOOTLOADER_ENABLE_READ_CMD != 0) && (BOOTLOADER_CRC_ENABLE != 0)
  #define _BOOTLOADER_PROTOCOL_VERSION BOOTLOADER_PROTOCOL_VERSION | 0xc0
#elif (BOOTLOADER_ENABLE_READ_CMD != 0)
  #define _BOOTLOADER_PROTOCOL_VERSION BOOTLOADER_PROTOCOL_VERSION | 0x80
#elif (BOOTLOADER_CRC_ENABLE != 0)
  #define _BOOTLOADER_PROTOCOL_VERSION BOOTLOADER_PROTOCOL_VERSION | 0x40
#else
  #define _BOOTLOADER_PROTOCOL_VERSION BOOTLOADER_PROTOCOL_VERSION
#endif

  //  FCOOB register structure
  typedef union 
  {
	uint8_t all[FCCOB_REGS];
	struct
	{
	  uint8_t fccob3;
	  uint8_t fccob2;
	  uint8_t fccob1;
	  uint8_t fccob0;
	  uint8_t fccob7;
	  uint8_t fccob6;
	  uint8_t fccob5;
	  uint8_t fccob4;
	  uint8_t fccobB;
	  uint8_t fccobA;
	  uint8_t fccob9;
	  uint8_t fccob8;
	}regs;
	
	struct
	{
	  uint64_t fccob3210;
	  uint64_t fccob7654;
	  uint64_t fccobBA98;
	}regsLong;
  }FCC0B_STR;

  
  typedef unsigned long addrType;
  
  typedef unsigned char BootloaderProtocolType;
  
  typedef union Address 
  {
    unsigned long complete;
    struct
    {
      unsigned short low;
      unsigned short high;        
    }Words;
    struct
    {
      unsigned char ll;
      unsigned char lh;
      unsigned char hl;
      unsigned char hh;
    }Bytes;
  }AddressType;
  
#pragma pack(1)
#pragma pack(push)
  
  /** Flash block start and end address */
  
  typedef struct FlashBlocksDesc
  {
    addrType startAddr;
    addrType endAddr; 
  }FlashBlocksDescType;
  
  
  /** Bootloader ident structure used for bootloader protocol */
  typedef struct BootloaderIdent
  {
    /** version */
    BootloaderProtocolType version;
    /** Sd Id */
    unsigned short sdid;
    /** count of flash blocks */
    addrType blocksCnt;
    /** flash blocks descritor */
    FlashBlocksDescType blockDesc[FLASH_BLOCK_CNT];
    /** Relocated interrupts vestor table */
    addrType relocatedVectors;
    /** Interrupts vestor table */
    addrType interruptsVectors;
    /** Erase Block Size */
    addrType eraseBlockSize;
    /** Write Block Size */
    addrType writeBlockSize;
    /** Id string */
    unsigned char idString[sizeof(KINETIS_MODEL_STR)];      
  }BootloaderIdentType;
  
  /** Bootloader code optimized ident structure used for bootloader protocol */
  typedef struct BootloaderIdentOptimType
  {
    /** count of flash blocks */
    addrType blocksCnt;
    /** flash blocks descritor */
    FlashBlocksDescType blockDesc[FLASH_BLOCK_CNT];
    /** Relocated interrupts vestor table */
    addrType relocatedVectors;
    /** Interrupts vestor table */
    addrType interruptsVectors;
    /** Erase Block Size */
    addrType eraseBlockSize;
    /** Write Block Size */
    addrType writeBlockSize;
    /** Id string */
    unsigned char idString[sizeof(KINETIS_MODEL_STR)];      
  }BootloaderIdentOptimType;
  
#pragma pack(pop) 
  
#define LITTLE2BIG(x) ((unsigned long)(((unsigned long)((unsigned char)(x)<<24) & 0xff000000) | (unsigned long)(((unsigned short)(x)<<8) & 0x00ff0000) | (unsigned long)(((x)>>8) & 0x0000ff00) | (unsigned long)(((x)>>24) & 0x00000000ff)))
  
typedef void (*vector_entry)(void);
typedef void pointer(void);
  


void UART_PutChar(UART_Type *base, unsigned char data);
unsigned char UART_GetChar(UART_Type *base);
unsigned char UART_IsChar(UART_Type *base);
void ReadAddress(void);
void Boot_ResetMCU(void);
void JumpToUserApplication(unsigned long userSP, unsigned long userStartup);
void IH_Controller();
uint64_t FLASH_ExESector(uint64_t destination);
void SendResult(unsigned long res);
uint64_t FLASH_ProgramSectionByLongs(uint64_t destination, uint64_t* pSource, uint64_t size);
uint32_t FlashSignoff(void);
void UART_Initialization(void);
__ramfunc uint64_t FLASH_FlashCommandSequenceStart(uint8_t index);
uint64_t FLASH_EraseEASector(uint64_t destination);
uint64_t FLASH_EraseSector(uint64_t destination);
uint64_t FLASH_ReadOnce(uint8_t pDataArray[], uint8_t index);
uint64_t FLASH_ProgramOnce(uint8_t pDataArray[],uint8_t index);


/*! @}*/

#endif /* _FSL_PIT_H_ */
