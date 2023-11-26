#include "Task_Referee.h"
#include "Task_Init.h"
#include "System_DataPool.h"

/**
 * @brief      裁判系统数据更新
 * @param[in]  None
 * @retval     None
 */
void Task_RefereeRecv(void *arg)
{
    /* Pre-Load for task */
	static USART_COB* referee_pack;
    static TickType_t xLastWakeTime_t = xTaskGetTickCount();
    /* Infinite loop */
    for(;;)
	{
		vTaskDelayUntil(&xLastWakeTime_t,1);
        // Sent_Contorl(&huart1);
		if(xTaskNotifyWait(0x00000000, 0xFFFFFFFF, (uint32_t *) &referee_pack, 0) == pdTRUE)
		{
			Referee.unPackDataFromRF((uint8_t*)referee_pack->address, referee_pack->len);		//更新裁判系统数据
		}
		/* Pass control to the next task */
  }
}

/**
 * @brief      RM客户端UI
 * @param[in]  None
 * @retval     None
 */
WorldTime_t referee_time;
uint16_t referee_fps;
uint16_t armor_x;
uint16_t armor_y;
void Task_RMClientUI(void *argument)
{
    portTickType xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    const TickType_t TimeIncrement = pdMS_TO_TICKS(5);  // --- 5MS 

    for(;;)
		{
					
            vTaskDelayUntil(&xLastWakeTime, TimeIncrement);
        }	

		#if 0//哨兵不用
    static uint8_t init_cnt = 8; //--- 初始化图层

    vTaskDelay(2000);

    Referee.clean_all(); //--- 删除所有图层

    vTaskDelay(500);

    for(;;)
	{
        if(Infantry.Write_Msg[7] == 1)
        {
           init_cnt = 8;
        }

        if(init_cnt != 0)
        {
            init_cnt--;
            Referee.Draw_Robot_Limit(180, 80, 961, 3, YELLOW, ADD_PICTURE);
            Referee.UI_Collimator(5, 961, 538, 25, YELLOW, ADD_PICTURE);
        }

        //--- 摩擦轮
        Referee.Draw_Fric(Infantry.Write_Msg[Attack_Ctrl], Infantry.Write_Msg[Fric_State], init_cnt, 1110, 750, 10, GREEN);
        //--- 视觉
        Referee.Draw_Vision(Infantry.Vision_Mode, Infantry.Write_Msg[Vision_State], Infantry.Get_VisionDepth(), init_cnt, 910, 750, 10, GREEN);
        //--- 弹仓
        Referee.Draw_Magazine(Infantry.Write_Msg[Mag_Ctrl], init_cnt, 1010, 750, 10, GREEN);
        //--- 超电
        Referee.Draw_Cap_Energy(SupCap.Get_Cell(), SupCap.Is_Output(), DevicesMonitor.Get_State(SUPCAP_MONITOR), init_cnt, 1025, 620);
        //--- 小陀螺
        Referee.Draw_Spin((Infantry.Get_ChassisMode()==CHAS_SpinMode), init_cnt, 810, 750, 10, GREEN);
        //--- 剩余子弹
        // Referee.Draw_Bullet(init_cnt,780,620);
        //--- 低血量警告
        Referee.Draw_LowHP(850, 700);

        //--- 动态装甲板 砍掉不用
        // Referee.Draw_Armor(init_cnt, armor_x, armor_y, 10, 10, PINK);
        // Referee.Draw_Armor(init_cnt, 0, abs(Chassis.Target_Vx)/4, abs(Chassis.Target_Vy)/4, 55, 35, 6, PINK);


        //--- 打符提示
        // Referee.Windmill_Icon(7,960,300,80,0,PINK,ADD_PICTURE);

        DevicesMonitor.Get_FPS(&referee_time, &referee_fps);
        
		// vTaskDelayUntil(&xLastWakeTime, TimeIncrement);
    }
		#endif
}

