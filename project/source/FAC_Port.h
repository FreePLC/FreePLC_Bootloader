#ifndef __FAC_PORT_H__
#define __FAC_PORT_H__

#include "Driver.h"

/*****************************************************************************************************
* Definition of module wide MACROs / #DEFINE-CONSTANTs - NOT for use in other modules
*****************************************************************************************************/
#define DEBUGENABLE             0x00
#define FTFx_REG_BASE           0x40020000
#define PFLASH_BLOCK_BASE       0x00000000
#define PFLASH_SIZE             0x00080000      /* 512 KB size */
#define DEFLASH_BLOCK_BASE      0xFFFFFFFF      /* There is not DFlash */
#define EERAM_BLOCK_BASE        0xFFFFFFFF      /* There is not EERAM */
#define EERAM_BLOCK_SIZE        0x00000800      /* 2 KB size */

#define FAC_IFR_A              0x10            /*Index of IFR for FAC_IFR_A*/
#define FAC_IFR_B              0x11            /*Index of IFR for FAC_IFR_B*/


typedef struct tagFACragne
{
    uint32_t	FAC_portA_range[64];
    uint32_t	FAC_portB_range[64];
}FACragne;


int8_t FAC_Verify(uint32_t addr);


#endif

