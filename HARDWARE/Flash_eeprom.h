#ifndef __STMFLASH_H__
#define __STMFLASH_H__
#include "sys.h"   
//////////////////////////////////////////////////////////////////////////////////	 
 
//STM32??FLASH?? ????	   
//STM32F4????-?????
//????:http://mcudev.taobao.com									  
////////////////////////////////////////////////////////////////////////////////// 



//FLASH????
#define STM32_FLASH_BASE 0x08000000 	//STM32 FLASH?????
 
#define SECTION_ADDR_PID 	  0x080E0000	  //放在扇区4里
#define SECTION_ADDR_OFFSET 0x080E0090
//FLASH ???????
#define ADDR_FLASH_SECTOR_0     ((u32)0x08000000) 	//??0????, 16 Kbytes  
#define ADDR_FLASH_SECTOR_1     ((u32)0x08004000) 	//??1????, 16 Kbytes  
#define ADDR_FLASH_SECTOR_2     ((u32)0x08008000) 	//??2????, 16 Kbytes  
#define ADDR_FLASH_SECTOR_3     ((u32)0x0800C000) 	//??3????, 16 Kbytes  
#define ADDR_FLASH_SECTOR_4     ((u32)0x08010000) 	//??4????, 64 Kbytes  
#define ADDR_FLASH_SECTOR_5     ((u32)0x08020000) 	//??5????, 128 Kbytes  
#define ADDR_FLASH_SECTOR_6     ((u32)0x08040000) 	//??6????, 128 Kbytes  
#define ADDR_FLASH_SECTOR_7     ((u32)0x08060000) 	//??7????, 128 Kbytes  
#define ADDR_FLASH_SECTOR_8     ((u32)0x08080000) 	//??8????, 128 Kbytes  
#define ADDR_FLASH_SECTOR_9     ((u32)0x080A0000) 	//??9????, 128 Kbytes  
#define ADDR_FLASH_SECTOR_10    ((u32)0x080C0000) 	//??10????,128 Kbytes  
#define ADDR_FLASH_SECTOR_11    ((u32)0x080E0000) 	//??11????,128 Kbytes  

void Flash_ReadPID(void);
void Flash_SavePID(void);
void Flash_ReadOFFSET(void);
void Flash_SaveOFFSET(void);
void STMFLASH_Read(u32 ReadAddr,u32 *pBuffer,u32 NumToRead)  ;
#endif

















