#include "led.h" 
//////////////////////////////////////////////////////////////////////////////////	 
//STM32F4����ģ��-�⺯���汾
//�Ա����̣�http://mcudev.taobao.com							  
////////////////////////////////////////////////////////////////////////////////// 	 

//��ʼ��PA6��PA7Ϊ�����.��ʹ���������ڵ�ʱ��		    
//LED IO��ʼ��
void LED_Init(void)
{    	 
  GPIO_InitTypeDef  GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_SetBits(GPIOA,GPIO_Pin_6 | GPIO_Pin_7);

}






