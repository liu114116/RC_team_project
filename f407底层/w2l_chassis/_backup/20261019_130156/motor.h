#ifndef MOTOR_H
#define MOTOR_H

#include "main.h"
#include "tim.h"
#include <stdint.h>

/* 电机参数：极对数，开环电角度换算用 */
#define MOTOR1_POLE_PAIRS          6U
#define MOTOR2_POLE_PAIRS          6U
#define MOTOR3_POLE_PAIRS          6U
#define MOTOR4_POLE_PAIRS          6U

//电机方向：修改这里即可改变对应电机的正方向
#define MOTOR_DIRECTION_NORMAL      1
#define MOTOR_DIRECTION_REVERSE    (-1)

#define MOTOR1_DIRECTION            MOTOR_DIRECTION_NORMAL
#define MOTOR2_DIRECTION            MOTOR_DIRECTION_NORMAL
#define MOTOR3_DIRECTION            MOTOR_DIRECTION_NORMAL
#define MOTOR4_DIRECTION            MOTOR_DIRECTION_NORMAL

/* M1 -> TIM2 */
#define MOTOR1_PWM_TIMER            (&htim2)
#define MOTOR1_PWM_A_CHANNEL        TIM_CHANNEL_1
#define MOTOR1_PWM_B_CHANNEL        TIM_CHANNEL_2
#define MOTOR1_PWM_C_CHANNEL        TIM_CHANNEL_3

/* M2 -> TIM4 */
#define MOTOR2_PWM_TIMER            (&htim4)
#define MOTOR2_PWM_A_CHANNEL        TIM_CHANNEL_1
#define MOTOR2_PWM_B_CHANNEL        TIM_CHANNEL_2
#define MOTOR2_PWM_C_CHANNEL        TIM_CHANNEL_3

/* M3 -> TIM5 */
#define MOTOR3_PWM_TIMER            (&htim5)
#define MOTOR3_PWM_A_CHANNEL        TIM_CHANNEL_1
#define MOTOR3_PWM_B_CHANNEL        TIM_CHANNEL_2
#define MOTOR3_PWM_C_CHANNEL        TIM_CHANNEL_3

/* M4 -> TIM3 */
#define MOTOR4_PWM_TIMER            (&htim3)
#define MOTOR4_PWM_A_CHANNEL        TIM_CHANNEL_1
#define MOTOR4_PWM_B_CHANNEL        TIM_CHANNEL_2
#define MOTOR4_PWM_C_CHANNEL        TIM_CHANNEL_3

/* M1-ADC */
#define MOTOR1_ADC_A_CHANNEL        ADC_CHANNEL_8
#define MOTOR1_ADC_B_CHANNEL        ADC_CHANNEL_9

/* M2-ADC */
#define MOTOR2_ADC_A_CHANNEL        ADC_CHANNEL_14
#define MOTOR2_ADC_B_CHANNEL        ADC_CHANNEL_15

/* M3-ADC */
#define MOTOR3_ADC_A_CHANNEL        ADC_CHANNEL_12
#define MOTOR3_ADC_B_CHANNEL        ADC_CHANNEL_13

/* M4-ADC */
#define MOTOR4_ADC_A_CHANNEL        ADC_CHANNEL_10
#define MOTOR4_ADC_B_CHANNEL        ADC_CHANNEL_11

#define MOTOR_ENABLE_LEVEL          GPIO_PIN_SET
#define MOTOR_DISABLE_LEVEL         GPIO_PIN_RESET
/* M1-ENABLE */
#define MOTOR1_ENABLE_PORT          GPIOE
#define MOTOR1_ENABLE_PIN           GPIO_PIN_10

/* M2-ENABLE */
#define MOTOR2_ENABLE_PORT          GPIOD
#define MOTOR2_ENABLE_PIN           GPIO_PIN_8

/* M3-ENABLE */
#define MOTOR3_ENABLE_PORT          GPIOE
#define MOTOR3_ENABLE_PIN           GPIO_PIN_7

/* M4-ENABLE */
#define MOTOR4_ENABLE_PORT          GPIOE
#define MOTOR4_ENABLE_PIN           GPIO_PIN_8


//MOTOR_CAL
//calibrate = 0：正常运行
//calibrate = 1：校准模式

#define MOTOR_CAL_PORT              GPIOE
#define MOTOR_CAL_PIN               GPIO_PIN_9

#define MOTOR_CAL_NORMAL_LEVEL      GPIO_PIN_SET
#define MOTOR_CAL_CALIBRATE_LEVEL   GPIO_PIN_RESET

/* 硬件参数：10mΩ 采样电阻、增益 20V/V、参考 3.3V、12 位 ADC */
#define MOTOR_CURRENT_SHUNT      0.0100f
#define MOTOR_CURRENT_GAIN       20.0f
#define MOTOR_CURRENT_VREF       3.3f
#define MOTOR_ADC_FULL_SCALE     4095.0f


/* ============================================================
 * 底层电机控制接口
 * ============================================================ */

/*
 * 初始化电机底层硬件状态：
 * - 关闭四个电机
 * - PWM 输出设置为 50%
 * - MOTOR_CAL 设置为正常运行电平
 */
void MOTOR_Init(void);

/*
 * 启动指定电机的三相 PWM，并打开 ENABLE。
 * motor_id:1、2、3、4
 */
void MOTOR_Start(uint8_t motor_id);

/*
 * 关闭指定电机的 ENABLE。
 */
void MOTOR_Stop(uint8_t motor_id);

/*
 * 设置指定电机三相 PWM 占空比。
 *
 * duty_a、duty_b、duty_c 范围：0.0f ~ 1.0f
 * 例如：
 *     MOTOR_SetPWM(1, 0.5f, 0.5f, 0.5f);
 */
void MOTOR_SetPWM(uint8_t motor_id,float duty_a,float duty_b,float duty_c);

/*
 * 设置 MOTOR_CAL。
 * calibrate = 0：正常运行
 * calibrate = 1：校准模式
 */
void MOTOR_SetCalibrate(uint8_t calibrate);


/* ============================================================
 * 电流采样
 * ============================================================ */

/* 相选择 */
#define MOTOR_PHASE_A                 0U
#define MOTOR_PHASE_B                 1U

/* 硬件参数：10mΩ 采样电阻、增益 20V/V、参考 3.3V、12 位 ADC */
#define MOTOR_CURRENT_SHUNT           0.0100f
#define MOTOR_CURRENT_GAIN            20.0f
#define MOTOR_CURRENT_VREF            3.3f
#define MOTOR_ADC_FULL_SCALE          4095.0f

/* 静态零点默认值（双向采样，位于量程中点附近） */
#define MOTOR_CURRENT_OFFSET_DEFAULT  2048.0f

/* 实测电流方向与采样放大器相反，取 -1；正向电流对应 raw 上升 */
#define MOTOR_CURRENT_SIGN            (-1.0f)

/* 每 1 个 ADC 码对应的电流：3.3 / (4095*20*0.01) ≈ 4.03 mA */
#define MOTOR_CURRENT_SCALE           \
    (MOTOR_CURRENT_VREF / (MOTOR_ADC_FULL_SCALE * MOTOR_CURRENT_GAIN * MOTOR_CURRENT_SHUNT))

/*
 * 上电初始化：先扫一次建立缓存。可选，不调用也能直接读数。
 */
void MOTOR_InitCurrentSensor(void);

/*
 * 读某一相的原始 ADC 码值（0~4095）。
 * 返回 1 成功；0 表示参数非法或采样超时，*raw 不被修改。
 */
uint8_t MOTOR_ReadCurrentRaw(uint8_t motor_id,uint8_t phase,uint16_t *raw);

/*
 * 读某一相电流，单位 A。每次调用都重新采样。
 * 参数非法或采样失败返回 0.0f。
 */
float MOTOR_GetCurrent(uint8_t motor_id,uint8_t phase);

/*
 * 零点校准：相电流为 0 时调用，采样 samples 次取平均。
 * 内部有延时，只能在上电初始化阶段调用。
 */
uint8_t MOTOR_CalibrateCurrentOffset(uint16_t samples);


#endif /* MOTOR_H */
