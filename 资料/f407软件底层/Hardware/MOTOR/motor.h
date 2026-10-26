#ifndef _MOTOR_H_
#define _MOTOR_H_

#include "main.h"
#include "tim.h"

/*
============================================================
电机编号
============================================================
*/

#define MOTOR_1     1U
#define MOTOR_2     2U
#define MOTOR_3     3U
#define MOTOR_4     4U

/*
============================================================
MOTOR_CAL
============================================================
*/

/*
 * 四个 DRV8323 共用一个 MOTOR_CAL。
 *
 * 你确认：
 * GPIO_PIN_SET   = 正常运行
 * GPIO_PIN_RESET = 校准
 */
#define MOTOR_CAL_PORT              GPIOE
#define MOTOR_CAL_PIN               GPIO_PIN_9
#define MOTOR_CAL_NORMAL_LEVEL      GPIO_PIN_SET

/*
============================================================
电机控制接口
============================================================
*/

/*
 * 初始化四个电机：
 *
 * - 设置 MOTOR_CAL 为正常运行状态；
 * - 初始化四个 ENABLE；
 * - 启动四个定时器的三路 PWM；
 * - 设置初始 PWM；
 * - 默认不使能电机。
 */
void motor_init(void);

/*
 * 运行指定电机。
 *
 * 需要在 while(1) 中反复调用。
 *
 * 示例：
 *
 * motor_run(MOTOR_1);
 *
 * 只运行 M1。
 */
void motor_run(uint8_t motor_id);

#endif

