#include "Task_init.h"
#include "System_DataPool.h"
#include "BSP_CAN.h"

/**
 * @brief      CAN1 接收任务
 * @param[in]  None
 * @retval     None
 */
void Task_CAN1_Recv(void *argument)
{
	static CanRxMsg_t CAN1_Rx;

    for(;;)
    {
        // --- 等待队列不为空
        xQueueReceive(Can1Recv_QueueHandle, &CAN1_Rx, portMAX_DELAY);

        switch(CAN1_Rx.Header.StdId)
        {
				 case 0x206:
            Shoot.Reload_Motor.update(CAN1_Rx.Data);//拨盘电机
            DevicesMonitor.Update(Frame_RELOAD);
            break;
					
        case 0x205:
					Gimbal.Motor[Yaw].update(CAN1_Rx.Data);//Yaw轴电机，和拨盘都是CAN1接收所以封装在一起发送
            DevicesMonitor.Update(Frame_GIMBAL_YAW);
            break;

        case 0x341:
            Infantry.RefereeMsg_Write(0x341, CAN1_Rx.Data);
            DevicesMonitor.Update(Frame_COMMU_0X341);
            break;
				case 0x342:
            Infantry.RefereeMsg_Write(0x342, CAN1_Rx.Data);
            DevicesMonitor.Update(Frame_COMMU_0X342);
            break;
				case 0x343:
            Infantry.RefereeMsg_Write(0x343, CAN1_Rx.Data);
            DevicesMonitor.Update(Frame_COMMU_0X343);
            break;

        }

    }
}


/**
 * @brief      CAN2 接收任务
 * @param[in]  None
 * @retval     None
 */
void Task_CAN2_Recv(void *argument)
{
	static CanRxMsg_t CAN2_Rx;

    for(;;)
    {
        // --- 等待队列不为空
        xQueueReceive(Can2Recv_QueueHandle, &CAN2_Rx, portMAX_DELAY);

        switch(CAN2_Rx.Header.StdId)
        {
        case 0x207:
            Shoot.Fric_Motor[Fric_L].update(CAN2_Rx.Data);//左摩擦轮
            DevicesMonitor.Update(Frame_FRIC_L);
            break;

        case 0x208:
            Shoot.Fric_Motor[Fric_R].update(CAN2_Rx.Data);//右摩擦轮
            DevicesMonitor.Update(Frame_FRIC_R);
            break;
        
        case 0x206:
            Gimbal.Motor[Pit].update(CAN2_Rx.Data);//Pit轴电机
            DevicesMonitor.Update(Frame_GIMBAL_PIT);
            break;
				
				case 0x205:
						Shoot.Barrel_Motor.update(CAN2_Rx.Data);//枪管电机
            break;
				case 0x201:
					Shoot.Card_Motor.update(CAN2_Rx.Data);//拨弹电机，单独封装发送
				break;

        // case 0x342:
        //     Infantry.RefereeMsg_Write(0x342, CAN2_Rx.Data);
        //     DevicesMonitor.Update(Frame_COMMU_0X342);
        //     break;
        }
    }
}

/**
 * @brief      CAN1 发送任务
 * @param[in]  None
 * @retval     None
 */
void Task_CAN1_Transmit(void *argument)
{ 
    static CanTxMsg_t CAN1_Tx;

    for(;;)
    {
        // 等待队列不为空
		xQueueReceive(Can1Send_QueueHandle, &CAN1_Tx, portMAX_DELAY);
        
    }
}

