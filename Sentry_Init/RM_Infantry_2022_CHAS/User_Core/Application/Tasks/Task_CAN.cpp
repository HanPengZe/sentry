#include "Task_CAN.h"
#include "Task_Init.h"
#include "BSP_CAN.h"
#include "System_DataPool.h"
#include "Power_Meter.h"


/**
 * @brief      CAN1 ��������
 * @param[in]  None
 * @retval     None
 */
void Task_CAN1_Recv(void *argument)
{
	static CanRxMsg_t CAN1_Rx;

    for(;;)
    {
        // --- �ȴ����в�Ϊ��
        xQueueReceive(Can1Recv_QueueHandle, &CAN1_Rx, portMAX_DELAY);

        switch(CAN1_Rx.Header.StdId)
        {
        case 0x201:
            Chassis.DRV_Motor[0].update(CAN1_Rx.Data);
            DevicesMonitor.Update(Frame_CHAS_DRV0);
            break;

        case 0x202:
            Chassis.DRV_Motor[1].update(CAN1_Rx.Data);
            DevicesMonitor.Update(Frame_CHAS_DRV1);
            break;
        
        case 0x203:
            Chassis.DRV_Motor[2].update(CAN1_Rx.Data);
            DevicesMonitor.Update(Frame_CHAS_DRV2);
            break;

        case 0x204:
            Chassis.DRV_Motor[3].update(CAN1_Rx.Data);
            DevicesMonitor.Update(Frame_CHAS_DRV3);
            break;
				case 0x340:
            Infantry.WriteMsgFromGimbal(CAN1_Rx.Data);
            DevicesMonitor.Update(Frame_COMMU_0X340);
            break;

        case SCCM_RECEIVE_ID://�ڱ�����
            SupCap.Update(CAN1_Rx.Data);
            DevicesMonitor.Update(Frame_SUPCAP);
            break;

        case POWERMETER_RECV_ID://�ڱ�����
            PowerMeter.Update(CAN1_Rx.Data);
            DevicesMonitor.Update(Frame_PowerMeter);
            break;


        }

    }
}


/**
 * @brief      CAN2 ��������
 * @param[in]  None
 * @retval     None
 */
void Task_CAN2_Recv(void *argument)
{
	static CanRxMsg_t CAN2_Rx;

    for(;;)
    {
        // --- �ȴ����в�Ϊ��
        xQueueReceive(Can2Recv_QueueHandle, &CAN2_Rx, portMAX_DELAY);

        switch(CAN2_Rx.Header.StdId)
        {
        case 0x205://ȫ���ֲ���
            Chassis.RUD_Motor[0].update(CAN2_Rx.Data);
            DevicesMonitor.Update(Frame_CHAS_RUD0);
            break;

        case 0x206://ȫ���ֲ���
            Chassis.RUD_Motor[1].update(CAN2_Rx.Data);
            DevicesMonitor.Update(Frame_CHAS_RUD1);
            break;
        
        case 0x207://ȫ���ֲ���
            Chassis.RUD_Motor[2].update(CAN2_Rx.Data);
            DevicesMonitor.Update(Frame_CHAS_RUD2);
            break;

        case 0x208://ȫ���ֲ���
            Chassis.RUD_Motor[3].update(CAN2_Rx.Data);
            DevicesMonitor.Update(Frame_CHAS_RUD3);
            break;
				
				case 0x201:
            Chassis.DRV_Motor[0].update(CAN2_Rx.Data);
            DevicesMonitor.Update(Frame_CHAS_DRV0);
            break;

        case 0x202:
            Chassis.DRV_Motor[1].update(CAN2_Rx.Data);
            DevicesMonitor.Update(Frame_CHAS_DRV1);
            break;
        
        case 0x203:
            Chassis.DRV_Motor[2].update(CAN2_Rx.Data);
            DevicesMonitor.Update(Frame_CHAS_DRV2);
            break;

        case 0x204:
            Chassis.DRV_Motor[3].update(CAN2_Rx.Data);
            DevicesMonitor.Update(Frame_CHAS_DRV3);
            break;

        // case 0x212:
        //     PowerMeter_Update(CAN2_Rx.Data);
        //     DevicesMonitor.Update(Frame_PowerMeter);
        //     break;



        case POWERMETER_RECV_ID://�ڱ�����
            PowerMeter.Update(CAN2_Rx.Data);
            DevicesMonitor.Update(Frame_PowerMeter);
            break;

        }
    }
}
