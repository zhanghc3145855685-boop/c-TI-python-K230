#include "key.h"

uint8_t click(void)
{
	uint8_t key_num=0;
	if(DL_GPIO_readPins(KEY_PORT,KEY_Pin_18_PIN)>0)
	{
		delay_ms(1);
		while(DL_GPIO_readPins(KEY_PORT,KEY_Pin_18_PIN)>0);
		delay_ms(1);
		key_num=1;
	}
	return key_num;
}



