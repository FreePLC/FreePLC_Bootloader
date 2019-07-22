/*
* Copyright (c) 2014, Freescale Semiconductor, Inc.
* All rights reserved.
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
* o Neither the name of Freescale Semiconductor, Inc. nor the names of its
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
/************************************************
*FAC有8个8bit寄存器将Flash分为64个段来进行保护
************************************************/
/*****************************************************************************************************
* Include files
*****************************************************************************************************/


#include "FAC_Port.h"



/*****************************************************************************************************
* Declaration of module wide FUNCTIONs - NOT for use in other modules
*****************************************************************************************************/
void PrintInfo(uint8_t pDataArray[]);
void Read_FAC_Bits(uint8_t pDataArray[]);
void Write_FAC_Bits(uint8_t pDataArray[], const uint8_t fac_list[], int len);
void ErrorTrap(uint32_t returnCode);



/*****************************************************************************************************
* Declaration of module wide TYPEs - NOT for use in other modules
*****************************************************************************************************/


/*****************************************************************************************************
* Definition of module wide VARIABLEs - NOT for use in other modules
*****************************************************************************************************/
uint8_t buf_ifr_fac[16];

/*! @brief Flash driver Structure */

extern const uint8_t fac_list[];


FACragne g_FAC_Range;

/*****************************************************************************************************
* Definition of module wide (CONST-) CONSTANTs - NOT for use in other modules
*****************************************************************************************************/


/* The size of each protected segment is 512KB/64=8KB on K22FN512. */

/*****************************************************************************************************
* Code of project wide FUNCTIONS
*****************************************************************************************************/
/*!
* @brief main function
*/
void FAC_Prot(const uint8_t fac_list[], int len)
{
  Read_FAC_Bits(buf_ifr_fac);
  //PrintInfo(buf_ifr_fac);
  Write_FAC_Bits(buf_ifr_fac, fac_list, len);
  //len = fac_list[0];
}

int8_t FAC_Verify(uint32_t addr)
{
	uint32_t segment_id = 0;
	int index;
    int bit;
	int8_t flag = 0;
  
	Read_FAC_Bits(buf_ifr_fac);

	if(FTFA->FACSS == 5)
	{
		segment_id = (addr>>13);
	}
	else if(FTFA->FACSS == 4)
	{
		segment_id = (addr>>12);
	}
	else
	{
		segment_id = 64;
	}

	if( segment_id > 63 )
	{
	  ;
	}
	else
	{
		bit   = segment_id % 8;
		index = segment_id/8 + 4;
                
		if(index >=8)
                {
                    index -= 8;
                }

		if((((buf_ifr_fac[index] >> bit)&0x01) == 0)||(((buf_ifr_fac[index + 8] >> bit)&0x01) == 0))
		{
			flag = 1;
		}

	}
	

	return flag;
	
}
#if 0

/*!
* @brief print FAC IFR and registers information
*/
void PrintInfo(uint8_t pDataArray[])
{

  int32_t i;
  
  /* print FAC IFR */
  printf("FAC_IFR_A:");
  for(i=3; i>=0; i--)
    printf("0x%02X, ",pDataArray[i]);
  for(i=7; i>=4; i--)
    printf("0x%02X, ",pDataArray[i]);
  printf("\n\r");
  printf("FAC_IFR_B:");
  for(i=0xB; i>=8; i--)
    printf("0x%02X, ",pDataArray[i]);
  for(i=0xF; i>=0xC; i--)
    printf("0x%02X, ",pDataArray[i]);
  printf("\n\r");
  
  /* print XACC registers */
  printf("XACC registers:\n\r");
  printf("XACCH0: 0x%02X\n\r", FTFA_XACCH0);
  printf("XACCH1: 0x%02X\n\r", FTFA_XACCH1);
  printf("XACCH2: 0x%02X\n\r", FTFA_XACCH2);
  printf("XACCH3: 0x%02X\n\r", FTFA_XACCH3);
  printf("XACCL0: 0x%02X\n\r", FTFA_XACCL0);
  printf("XACCL1: 0x%02X\n\r", FTFA_XACCL1);
  printf("XACCL2: 0x%02X\n\r", FTFA_XACCL2);
  printf("XACCL3: 0x%02X\n\r", FTFA_XACCL3);

}
#endif

/*!
* @brief read FAC_IFR_A and FAC_IFR_B
*/
void Read_FAC_Bits(uint8_t pDataArray[])
{
  uint64_t returnCode;      /* Return code variable */
  
  /* Call FlashReadOnce to read FAC_IFR_A */
  //yanglilang
  returnCode = FLASH_ReadOnce(pDataArray, FAC_IFR_A);
  returnCode = FLASH_ReadOnce(pDataArray+8, FAC_IFR_B);
  
  //if(returnCode!=kStatus_FLASH_Success)
  //{
    //ErrorTrap(returnCode);
  //}
  
  /* Call FlashReadOnce to read FAC_IFR_B */
  //yangliang
  //returnCode = FLASH_ReadOnce(&s_flashDriver, FAC_IFR_B, pDataArray+8, 8);
  //if(returnCode!=kStatus_FLASH_Success)
  //{
    //ErrorTrap(returnCode);
  //}
}


void fac_add_one_setment(uint8_t segment_id)
{
  int index;
  int bit;
  
  if( segment_id > 63 )
  {
    return;
  }
  
  bit   = segment_id % 8;
  index = segment_id/8 + 4;
  if(index >=8)
    index -= 8;
  
  buf_ifr_fac[index]   &= ~(1 << bit);
  buf_ifr_fac[index+8] &= ~(1 << bit);
}
void fac_to_data(const uint8_t fac_list[], int len)
{
  int i;
  
  for(i=0; i<len; i++)
  {
    fac_add_one_setment(fac_list[i]);
  }
}


/*!
* @brief program bits of FAC IFR
*/
void Write_FAC_Bits(uint8_t pDataArray[], const uint8_t fac_list[], int len)
{
  uint32_t returnCode;      /* Return code variable */
  int32_t i;
  uint32_t ifr_a_wr_pm, ifr_b_wr_pm;
  
  /* check FAC_IFR_A and FAC_IFR_B */
  ifr_a_wr_pm = 1;
  ifr_b_wr_pm = 1;
  for(i=0; i<8; i++)
  {
    if(pDataArray[i] != 0xFF)
      ifr_a_wr_pm = 0;
    
    if(pDataArray[i+8] != 0xFF)
      ifr_b_wr_pm = 0;
  }
  
  if((ifr_a_wr_pm == 0) && (ifr_b_wr_pm == 0))
  {
    return;
  }
  
  /* calculate the data to be programmed */
  fac_to_data(fac_list, len);
#if 0
  // show buffer content
  printf("Buffer for FAC_IFR_A is ");
  for(i=3; i>=0; i--)
    printf("0x%02X, ",pDataArray[i]);
  for(i=7; i>=4; i--)
    printf("0x%02X, ",pDataArray[i]);
  printf("\n\r");
  
  printf("Buffer for FAC_IFR_B is ");
  for(i=0xB; i>=8; i--)
    printf("0x%02X, ",pDataArray[i]);
  for(i=0xF; i>=0xC; i--)
    printf("0x%02X, ",pDataArray[i]);
  printf("\n\r");
#endif
  
  /* Set FAC by setting IFR A or IFR B */
  if(ifr_a_wr_pm == 1)
  {
  	//yangliang
    returnCode = FLASH_ProgramOnce(pDataArray, FAC_IFR_A);
  }

  if(ifr_b_wr_pm == 1)
  {
    //yangliang
    returnCode = FLASH_ProgramOnce(pDataArray+8, FAC_IFR_B);
  }
  
  
}

/*!
* @brief error trip function
*/
void ErrorTrap(uint32_t returnCode)
{
  uint32_t failedReason = returnCode;
  
  while (1)
  {
    ;
  }
}
#if 0
/* for test */
void print_buf(void *buf, int size)
{
  int i;
  uint8_t *p = (uint8_t*)buf;
  for(i=0;i<size;i++)
  {
    printf("0x%.2x,", p[i]);
    if(((i+1)&0x3) == 0)
      printf("  ");
    if(((i+1)&0x7) == 0)
      printf("\n");
  }
  printf("\n\r");
}
#endif

void fac_test_segment(uint8_t segment_id)
{
  //printf("--i = %d\r\n", segment_id);
  memset(buf_ifr_fac, 0xff, 16);
  fac_add_one_setment(segment_id);
  //print_buf(buf_ifr_fac, 16);
}

void fac_test0(void)
{
  int i;
  for(i=0; i<64; i++)
  {
    fac_test_segment(i);
  }
}
void fac_test1(void)
{
  //printf("fac test 1.\r\n");
  memset(buf_ifr_fac, 0xff, 16);
  fac_add_one_setment(0);
  fac_add_one_setment(1);
  fac_add_one_setment(2);
  fac_add_one_setment(63);
  fac_add_one_setment(62);
  fac_add_one_setment(61);
  //print_buf(buf_ifr_fac, 16);
}

void fac_test(void)
{
  fac_test0();
  fac_test1();
  
}







/********************************************************************/
