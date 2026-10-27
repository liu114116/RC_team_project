#include "key.h"

uint8_t KEY1_State      = 0;   
uint8_t KEY1_last_State = 0;   

uint8_t Key_IsDown(void)
{
    return (HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin) == GPIO_PIN_RESET) ? 1U : 0U;
}

/* 返回值：
 *   0 = 无事件
 *   1 = 按下事件（消抖确认后，刚刚按下）
 *   2 = 松开事件（消抖确认后，刚刚松开）
 */
uint8_t Key_Scan(void)
{
    uint8_t current = Key_IsDown(); 
    uint8_t event   = 0;

    
    if (current == KEY1_last_State) {
        if (current != KEY1_State) {
            
            KEY1_State = current;             
            event = (current == 1) ? 1 : 2;   
        }
    }
    KEY1_last_State = current;   
    return event;
}

