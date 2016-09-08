#include "Flash_eeprom.h"
//#include "hmc5883l.h"
#include "imu.h"
#include "control.h"
#include "pid.h"
#include "hmc5883.h"
#include "includes.h"
#define BYTE0(dwTemp)       (*(char *)(&dwTemp))
#define BYTE1(dwTemp)       (*((char *)(&dwTemp) + 1))
#define BYTE2(dwTemp)       (*((char *)(&dwTemp) + 2))
#define BYTE3(dwTemp)       (*((char *)(&dwTemp) + 3))
u16 Flash_SaveData[40];
//////////////////////////////////////////////////////////////////////////////////	 
 
//STM32�ڲ�FLASH��д ��������	   
//STM32F4����ģ��-�⺯���汾
//�Ա����̣�http://mcudev.taobao.com									  
////////////////////////////////////////////////////////////////////////////////// 

 
 
//��ȡָ����ַ�İ���(16λ����) 
//faddr:����ַ 
//����ֵ:��Ӧ����.
u32 STMFLASH_ReadWord(u32 faddr)
{
	return *(vu32*)faddr; 
}  
//��ȡĳ����ַ���ڵ�flash����
//addr:flash��ַ
//����ֵ:0~11,��addr���ڵ�����
uint16_t STMFLASH_GetFlashSector(u32 addr)
{
	if(addr<ADDR_FLASH_SECTOR_1)return FLASH_Sector_0;
	else if(addr<ADDR_FLASH_SECTOR_2)return FLASH_Sector_1;
	else if(addr<ADDR_FLASH_SECTOR_3)return FLASH_Sector_2;
	else if(addr<ADDR_FLASH_SECTOR_4)return FLASH_Sector_3;
	else if(addr<ADDR_FLASH_SECTOR_5)return FLASH_Sector_4;
	else if(addr<ADDR_FLASH_SECTOR_6)return FLASH_Sector_5;
	else if(addr<ADDR_FLASH_SECTOR_7)return FLASH_Sector_6;
	else if(addr<ADDR_FLASH_SECTOR_8)return FLASH_Sector_7;
	else if(addr<ADDR_FLASH_SECTOR_9)return FLASH_Sector_8;
	else if(addr<ADDR_FLASH_SECTOR_10)return FLASH_Sector_9;
	else if(addr<ADDR_FLASH_SECTOR_11)return FLASH_Sector_10; 
	return FLASH_Sector_11;	
}
//��ָ����ַ��ʼд��ָ�����ȵ�����
//�ر�ע��:��ΪSTM32F4������ʵ��̫��,û�취���ر�����������,���Ա�����
//         д��ַ�����0XFF,��ô���Ȳ������������Ҳ�������������.����
//         д��0XFF�ĵ�ַ,�����������������ݶ�ʧ.����д֮ǰȷ��������
//         û����Ҫ����,��������������Ȳ�����,Ȼ����������д. 
//�ú�����OTP����Ҳ��Ч!��������дOTP��!
//OTP�����ַ��Χ:0X1FFF7800~0X1FFF7A0F
//WriteAddr:��ʼ��ַ(�˵�ַ����Ϊ4�ı���!!)
//pBuffer:����ָ��
//NumToWrite:��(32λ)��(����Ҫд���32λ���ݵĸ���.) 
__INLINE void STMFLASH_Write(u32 WriteAddr,u32 *pBuffer,u32 NumToWrite)	
{ 
  FLASH_Status status = FLASH_COMPLETE;
	u32 addrx=0;
	u32 endaddr=0;	
	

  if(WriteAddr<STM32_FLASH_BASE||WriteAddr%4)return;	//�Ƿ���ַ
	FLASH_Unlock();									//���� 
  FLASH_DataCacheCmd(DISABLE);//FLASH�����ڼ�,�����ֹ���ݻ���
 		
	addrx=WriteAddr;				//д�����ʼ��ַ
	endaddr=WriteAddr+NumToWrite*4;	//д��Ľ�����ַ
	if(addrx<0X1FFF0000)			//ֻ�����洢��,����Ҫִ�в�������!!
	{
		while(addrx<endaddr)		//ɨ��һ���ϰ�.(�Է�FFFFFFFF�ĵط�,�Ȳ���)
		{
			if(STMFLASH_ReadWord(addrx)!=0XFFFFFFFF)//�з�0XFFFFFFFF�ĵط�,Ҫ�����������
			{   
				status=FLASH_EraseSector(STMFLASH_GetFlashSector(addrx),VoltageRange_3);//VCC=2.7~3.6V֮��!!
				if(status!=FLASH_COMPLETE)break;	//����������
			}else addrx+=4;
		} 
	}
	if(status==FLASH_COMPLETE)
	{
		while(WriteAddr<endaddr)//д����
		{
			if(FLASH_ProgramWord(WriteAddr,*pBuffer)!=FLASH_COMPLETE)//д������
			{ 
				break;	//д���쳣
			}
			WriteAddr+=4;
			pBuffer++;
		} 
	}
  FLASH_DataCacheCmd(ENABLE);	//FLASH��������,�������ݻ���
	FLASH_Lock();//����

} 

//��ָ����ַ��ʼ����ָ�����ȵ�����
//ReadAddr:��ʼ��ַ
//pBuffer:����ָ��
//NumToRead:��(4λ)��
__INLINE void STMFLASH_Read(u32 ReadAddr,u32 *pBuffer,u32 NumToRead)   	
{
	u32 i;
	OS_CPU_SR cpu_sr=0;
	OS_ENTER_CRITICAL();	

	for(i=0;i<NumToRead;i++)
	{
		pBuffer[i]=STMFLASH_ReadWord(ReadAddr);//��ȡ4���ֽ�.
		ReadAddr+=4;//ƫ��4���ֽ�.	
	}
	OS_EXIT_CRITICAL();	

}



void Flash_ReadPID(void)
{
//	u32 read_data[36];
//	STMFLASH_Read(SECTION_ADDR_PID,(u32*)read_data,36);

	RollRate.Kp=0.05;
	RollRate.Ki=0;
	RollRate.Kd=1.0;
//	
	PitchRate.Kp=0.05;
	PitchRate.Ki=0;
	PitchRate.Kd=1;
	

	YawRate.Kp=0.01;
	YawRate.Ki=0;
	YawRate.Kd=0;
	
	Stabilize_Roll.Kp=80;
	Stabilize_Roll.Ki=0;
	Stabilize_Roll.Kd=0;
	
	Stabilize_Pitch.Kp=80;
	Stabilize_Pitch.Ki=0;
	Stabilize_Pitch.Kd=0;


	Stabilize_Yaw.Kp=0;
	Stabilize_Yaw.Ki=0;
	Stabilize_Yaw.Kd=0;
	
	//Climb.Kp=2;
	Climb.Ki=0;
	Climb.Kd=0;

	AutoHigh_THR.Kp=2;
	AutoHigh_THR.Ki=0;
	AutoHigh_THR.Kd=0;

//	RollAccel.Kp=0;//(float)read_data[24]/100;
//	RollAccel.Ki=0;//(float)read_data[25]/100;
//	RollAccel.Kd=0;//(float)read_data[26]/100;
//	
//	PitchAccel.Kp=0;//(float)read_data[27]/100;
//	PitchAccel.Ki=0;//(float)read_data[28]/100;
//	PitchAccel.Kd=0;//(float)read_data[29]/100;

//	Position_Hold.Kp=0;
//	Position_Hold.Ki=0;
//	Position_Hold.Kd=0;

//	Position_Speed.Kp=0;
//	Position_Speed.Ki=0;
//	Position_Speed.Kd=0;

}

void Flash_SavePID(void)
{
	u32 save_data[36];
	u32 read_data[50];
	int i;
//	save_data[0]=RollRate.Kp*1000; 
//	save_data[1]=RollRate.Ki*1000;
//	save_data[2]=RollRate.Kd*10;
	save_data[0]=1; 
	save_data[1]=2;
	save_data[2]=3;
	save_data[3]=PitchRate.Kp*1000;
	save_data[4]=PitchRate.Ki*1000; 
	save_data[5]=PitchRate.Kd*100;

	save_data[6]=YawRate.Kp*1000;
	save_data[7]=YawRate.Ki*100;
	save_data[8]=YawRate.Kd*100;

	save_data[9]=Stabilize_Roll.Kp*10; 
	save_data[10]=Stabilize_Roll.Ki*1000;
	save_data[11]=Stabilize_Roll.Kd*10;

	save_data[12]=Stabilize_Pitch.Kp*10;
	save_data[13]=Stabilize_Pitch.Ki*1000; 
	save_data[14]=Stabilize_Pitch.Kd*10;

	save_data[15]=0;//Stabilize_Yaw.Kp*10;
	save_data[16]=0;//Stabilize_Yaw.Ki*100;
	save_data[17]=0;//Stabilize_Yaw.Kd*100;

	save_data[18]=Climb.Kp*100;
	save_data[19]=Climb.Ki*100;
	save_data[20]=Climb.Kd*100;

	save_data[21]=AutoHigh_THR.Kp*100;
	save_data[22]=AutoHigh_THR.Ki*10000;
	save_data[23]=AutoHigh_THR.Kd*100;

	save_data[24]=0;//RollAccel.Kp*100;
	save_data[25]=0;//RollAccel.Ki*100;
	save_data[26]=0;//RollAccel.Kd*100;
						
	save_data[27]=0;//PitchAccel.Kp*100;
	save_data[28]=0;//PitchAccel.Ki*100;
	save_data[29]=0;//PitchAccel.Kd*100;

	save_data[30]=Position_Hold.Kp*1000;
	save_data[31]=Position_Hold.Ki*1000;
	save_data[32]=Position_Hold.Kd*100;
	
//	save_data[33]=Position_Speed.Kp*1000;
//	save_data[34]=Position_Speed.Ki*1000;
//	save_data[35]=Position_Speed.Kd*100;
	save_data[33]=4;
	save_data[34]=5;
	save_data[35]=6;		
			
	STMFLASH_Read(SECTION_ADDR_OFFSET,(u32*)read_data,14);
	 for(i=0;i<36;i++)
		read_data[14+i]=save_data[i];			
	STMFLASH_Write(SECTION_ADDR_PID,(u32*)save_data,50);

}
void Flash_ReadOFFSET(void)
{
	 u32 read_data[14];
	STMFLASH_Read(SECTION_ADDR_OFFSET,(u32*)read_data,14);

	 ACC_OFFSET.X=0;
	 ACC_OFFSET.Y=0;
	 ACC_OFFSET.Z=0;
	 GYRO_OFFSET.X=0;
	 GYRO_OFFSET.Y=0;
	 GYRO_OFFSET.Z=0;
	 HMC5883L_OFFSET.X=read_data[5];
	 HMC5883L_OFFSET.Y=read_data[6];
	 HMC5883L_OFFSET.Z=read_data[7];
//	 HMC5883L_Gain.X=(float)(read_data[8])/10000; 
	 HMC5883L_Gain.X=(float)(read_data[8]/10000); 
	 HMC5883L_Gain.Y=(float)(read_data[9])/10000;
	 HMC5883L_Gain.Z=(float)(read_data[10])/10000;
     //XYƯ�Ʋ���ȥ��XY���ٶ�Ư�ƣ���Ϊ���У������Ϊ������ת����INT����FLASH��ᵼ��FLASH��������ԭ��			   
	 //Accel_Offset.X=(float)(read_data[11])/1000; 	
	 //Accel_Offset.Y=(float)(read_data[12])/1000;
	 Accel_Offset.Z=(float)(read_data[13])/10;
}

void Flash_SaveOFFSET(void)
{
	u32 save_data[50];
	u32 read_data[36];
	int i;
//	save_data[0]=ACC_OFFSET.X;
	save_data[0]=10;
	save_data[1]=ACC_OFFSET.Y;
	save_data[11]=ACC_OFFSET.Z;
	
	save_data[2]=GYRO_OFFSET.X;
	save_data[3]=GYRO_OFFSET.Y;
	save_data[4]=GYRO_OFFSET.Z;
	//XYƯ�Ʋ���ȥ��XY���ٶ�Ư�ƣ���Ϊ���У������Ϊ������ת����INT����FLASH��ᵼ��FLASH��������ԭ��
	//save_data[11]=(int)(Accel_Offset.X*100); 
	//save_data[12]=(int)(Accel_Offset.Y*100);
//	save_data[13]=(int)(Accel_Offset.Z*10);
	//HMC5883L_OFFSET.X=-114;
	save_data[5]=HMC5883L_OFFSET.X;
	//HMC5883L_OFFSET.Y=-68;
	save_data[6]=HMC5883L_OFFSET.Y;
	//HMC5883L_OFFSET.Z=-99;
	save_data[7]=HMC5883L_OFFSET.Z;			   
//	save_data[8]=(int)(HMC5883L_Gain.X*10000);//10000;
	save_data[8]=(int)(HMC5883L_Gain.X*10000);//10000;
	save_data[9]=(int)(HMC5883L_Gain.Y*10000);//10106;
	save_data[10]=(int)(HMC5883L_Gain.Z*10000);//11543;
	
	STMFLASH_Read(SECTION_ADDR_PID,(u32*)read_data,36);
	 for(i=0;i<14;i++)
		read_data[36+i]=save_data[i];
	
	STMFLASH_Write(SECTION_ADDR_PID,(u32*)read_data,50);


}


