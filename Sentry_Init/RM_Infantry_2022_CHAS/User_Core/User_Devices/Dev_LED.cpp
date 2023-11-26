#include "Dev_LED.h"
#include "tim.h"

/**
  * @brief  正常运行流水灯
  * @param  None
  * @retval None
  */
uint8_t LED_Cycle = 1;
uint8_t Dir = true;
void Waterfall_LED(void)
{
	static uint8_t LED_Port = LED_1; //--- 引脚
    static uint8_t Dir = true;   //--- 方向

	if (LED_Port == LED_8)
	{
		Dir = false;
	}
	else if (LED_Port == LED_1)
	{
		Dir = true;
	}

	(Dir == true) ? (LED_Port++) : (LED_Port--);

    HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_1 << LED_Port);
}
