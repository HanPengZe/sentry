#include "Dev_Buzzer.h"
// #include "System_DataPool.h"
#include "cmsis_os.h"


Buzzer_classdef Buzzer;


/**
  * @brief  ���������
  */
MusicNote_t SuperMario[] = {
	{M3, 100}, {0, 50}, {M3, 250}, {0, 50}, {M3, 100}, {0, 50}, {0, 150}, {M1, 100}, {0, 50}, 
	{M3, 250}, {0, 50}, {M5, 250}, {0, 50}, {0, 300}, {L5, 250}, {0, 50}, {0, 300},
	/*{M1, 250}, {0, 50}*/
};

/**
  * @brief  ���������
  */
MusicNote_t TheWindRises[] = {
	/* {0,3300}, */
	{M7, 16},{H1, 16},{H2, 16},{H3,16},
	{H3, 16},{M5, 16},{H5, 16},{H3,16},
	{H3,2},
	{M7, 16},{H1, 16},{H2, 16},{H3,16},
	{H3, 16},{M5, 16},{H5, 16},{H3,16},
	{H2, 16},{H3, 16},{H1, 16},{H2,16},
	{M7, 16},{H1, 16},{M5, 4},
	{M7, 16},{H1, 16},{H2, 16},{H3,16},
	{H3, 16},{M5, 16},{H5, 16},{H3,16 },{H3,4},{0,4},
	{M2, 2},{L6, 2},
	{M2, 16 / 3},{M1, 16},{M2, 16 / 3},{M1,16},//��һ·��
	{M2, 8},{M3, 8},{M5, 8},{M3,16},{M2,16},
	{M2,16 / 3},{M1, 16},{M2, 16 / 3},{M1,16},
	{M2, 16},{M3, 16},{M2,16},{M1,16},{L5,4},
	{M2, 16 / 3},{M1, 16},{M2, 16 / 3},{M1,16},//������վ
	{M2, 8},{M3, 8},{M5, 8},{M3,16},{M2,16},
	{M2, 16 / 3},{M3, 16},{M2, 8},{M1,16},{M2,16},{M2,4},{0,4},//����Щ��ԥ
	{M2, 16 / 3},{M1, 16},{M2, 16 / 3},{M1,16},
	{M2, 8},{M3, 8},{M5, 8},{M3,8},
	{M2, 16 / 3},{M3, 16},{M2, 8},{M1,8},{L6,4},//���޿ɱ���
	{M3, 16},{M2, 16},{M1, 16},{M2,16},
	{M1, 4},	{M3, 16},{M2, 16},{M1, 16},{M2,16},//������ôů
	{M1, 16 / 3},	{L5, 16},{M3, 16},{M2, 16},{M1, 16},{M2,16},
	{M1, 2},
	{M1, 8},{M2, 8},{M3, 8},{M1, 8},//��ǰ��ʶ
	{M6, 8},{M5, 16},{M6,4},{M1,16},
	{M7, 8},{M6, 16},{M7, 16 / 5},
	{M7, 8},{M6, 16},{M7, 4},	{M3, 16},//�������
	{H1, 16},{H2, 16},{H1, 16},{M7, 16},{M6, 8},	{M5, 8},//������ǰ
	{M6, 8},	{M5, 16},	{M6, 8},	{M5, 16},{M6, 16},	{M5, 16},//��Ը��������
	{M6, 8},{M5, 16},{M2,16 / 3},{M5, 16},{M3, 16},
	{M3, 2},

	{M1, 8},{M2, 8},{M3, 8},{M1, 8},//����߹�
	{M6, 8},{M5, 16},{M6,4},{M1,16},
	{M7, 8},{M6, 16},{M7, 16 / 5},
	{M7, 8},{M6, 16},{M7, 4},	{M3, 16},//��������
	{H1, 16},{H2, 16},{H1, 16},{M7, 16},{M6, 8},	{M5, 8},//��ͬ��������
	{M6, 8},	{H3, 16},	{H3, 16 / 3},	{M5, 8},//�������������
	{M6, 8} ,{H3, 16},	{H3, 16 / 3} ,{M5, 16},{M6, 16},
	{M6, 2},{0, 4},
	{H1, 8},{H2, 8},//����
	{H3, 8},{H6, 16},{H5, 16 / 3},{H6, 16},{H5, 16 / 3},{H6, 16},{H5, 16 / 3},{H2,16},{H3,16 / 3},//���԰�
	{H6, 16},{H5, 16 / 3},{H6, 16},{H5, 16 / 3},{H6, 16},{H5, 16 / 3},{H2,16},
	{H3,16 },{H2, 8},{H1, 16},{M6, 16 / 3},{H1,8},//������ٲ�������
	{H2, 8},{H1, 16},{M6, 8},{H1, 16 / 3},
	{H3, 16 / 5},{H4, 16},{H3, 16} ,{H2,16},{H3, 16} ,{H2,16 / 3},//����Ц��

	{H1, 8},{H2, 8},//�������ഺ��ӿ����
	{H3, 8},{H6, 16},{H5, 16 / 3},{H6, 16},{H5, 16 / 3},{H6, 16},{H5, 16 / 3},{H3,16},
	{H3,8}, {H6, 16},{H5, 16 / 3},{H6, 16},{H5, 16 / 3},{H6, 16},{H5, 16 / 3},{H3,16},{H3,16 / 3 },//Ҳ��ָ�ⵯ��ʢ��
	{H2, 8},{H1, 16},{M6, 8},{H3,16 / 3},//��֮�����о���Եȥ��
	{H2, 8},{H1,16},{M6,8},{H1, 16 / 3},{H1,2},{0,4},
	{0,0}
};


Buzzer_classdef::Buzzer_classdef()
{

}



/**
 * @brief      ���÷�������������
 * @param[in]  ringtype
 * @retval     None
 */
void Buzzer_classdef::Set_RingType(int8_t ringtype)
{
	Ring_Type = ringtype;
}

void Buzzer_classdef::Process()
{
	static uint8_t state_cnt = 0; // BB�����
	static uint8_t time_cnt = 0;  // BB��Ƶ�μ���
	static int8_t last_times = 0; // ��¼�Ƿ���״̬�л�

	if(Ring_Type != last_times)
	{
		time_cnt = Ring_Type;
		state_cnt = 0;
	}

	switch(Ring_Type)
	{
	case Ring_Stop:	//--- ֹͣ
		HAL_TIM_PWM_Stop(&htim12, TIM_CHANNEL_1);
		state_cnt = 0;
		break;

	case Ring_ContLong: //--- �������� �����
		state_cnt++;
		if((state_cnt%3)==0 && (state_cnt<time_cnt*3))
		{
			HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1);
			__HAL_TIM_SET_AUTORELOAD(&htim12, 3000);
		}
		else if((state_cnt%=5)==0 && (state_cnt<time_cnt*3))
		{
			__HAL_TIM_SET_AUTORELOAD(&htim12, 0);
		}
		break;

	case Ring_ContShort: //--- �������� �̼��
		state_cnt++;
		if((state_cnt%=2)==0 && (state_cnt<time_cnt*2))
		{
			HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1);
			__HAL_TIM_SET_AUTORELOAD(&htim12, 3000);
		}
		else
		{
			__HAL_TIM_SET_AUTORELOAD(&htim12, 0);
		}
		break;

	default: //--- ����n��
    	state_cnt++;
		if((state_cnt%=2)==0 && (state_cnt<time_cnt*2))
		{
			HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1);
			__HAL_TIM_SET_AUTORELOAD(&htim12, 3000);
			time_cnt--;
		}
		else
		{
			__HAL_TIM_SET_AUTORELOAD(&htim12, 0);
		}
		break;
	}

	last_times = Ring_Type;
}

/**
 * @brief      ���������ֲ���
 * @param[in]  music music_len
 * @retval     None
 */
void Buzzer_classdef::Music_Play(MusicNote_t music[], uint16_t len)
{
	HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1);
	for (int i = 0; i < sizeof(SuperMario) / sizeof(MusicNote_t); i++)
	{
		if (music[i].note == 0)
		{
			__HAL_TIM_SET_AUTORELOAD(&htim12, 0);
			osDelay(music[i].time);
		}
		else
		{
			__HAL_TIM_SET_AUTORELOAD(&htim12, 1000000 / music[i].note);
			osDelay(music[i].time);
		}
	}
	HAL_TIM_PWM_Stop(&htim12, TIM_CHANNEL_1);
}




