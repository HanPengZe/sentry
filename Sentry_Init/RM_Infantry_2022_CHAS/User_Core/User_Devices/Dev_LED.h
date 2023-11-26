#ifndef _LED_H_
#define _LED_H_

#ifdef  __cplusplus
extern "C" {
#endif

//--- LEDÒý½ÅÃ¶¾Ù
typedef enum
{
    LED_1,
    LED_2,
    LED_3,
    LED_4,
    LED_5,
    LED_6,
    LED_7,
    LED_8,
    LED_NUMS
}LED_Port_e;


void Waterfall_LED(void);

#ifdef __cplusplus
}
#endif

#endif
