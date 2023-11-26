#ifndef _LED_H_
#define _LED_H_

#include <stdint.h>

#define BLN_LENGHT   6  //--- ºôÎüµÆÊý×é³¤¶È

class LED_classdef
{
private: 
    float delta_alpha, delta_red, delta_green, delta_blue;
    float alpha,red,green,blue;
    uint32_t BLN_Color[BLN_LENGHT + 1] = {0xFF0000FF, 0x0000FF00, 0xFFFF0000, 0x000000FF, 0xFF00FF00, 0x00FF0000, 0xFF0000FF};
    uint32_t RGB;

public:
    LED_classdef();

    void Init();
    void BLN_Ctrl();    /*<! C°åºôÎüµÆ¿ØÖÆ */
    void BLN_Display(uint32_t rgb); /*<! C°åºôÎüµÆÕ¹Ê¾ */
};

#endif
