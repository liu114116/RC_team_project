#ifndef _MOTOR_CONTROL_H_
#define _MOTOR_CONTROL_H_
#include "main.h"
#include <stdint.h>

/* 电机参数 */
#define MOTOR1_POLE_PAIRS          6U
#define MOTOR2_POLE_PAIRS          7U
#define MOTOR3_POLE_PAIRS          7U
#define MOTOR4_POLE_PAIRS          7U
/* 电机参数 */

/* 编码器参数 */
/* 编码器方向：+1/-1，先根据编码器与 PWM 的对应关系确认 */
#define MOTOR1_SENSOR_DIRECTION       1
#define MOTOR2_SENSOR_DIRECTION       1
#define MOTOR3_SENSOR_DIRECTION       1
#define MOTOR4_SENSOR_DIRECTION       1

/*
 * 编码器电角度零位偏移，单位 rad，范围 [0, 2π)。
 *
 * 换算关系（见 DengFOC_CalcElectricalAngle）：
 *     electrical_angle =
 *         SENSOR_DIRECTION * POLE_PAIRS * mechanical_angle - zero;
 *     结果折返到 [0, 2π)。
 *
 * 每个电机单独标定：施加 d 轴电压让转子对齐，读回机械角乘以极对数，
 * 再折返到 [0, 2π)。极对数、编码器方向、PWM 相序或编码器安装位置
 * 改变后需要重新标定。该零位与 MOTOR_CAL 电流检测放大器校准无关。
 *
 * M3 实测：5.7377 rad（328.75 度）。
 */
#define MOTOR1_ZERO_ELECTRICAL_ANGLE   0.0f
#define MOTOR2_ZERO_ELECTRICAL_ANGLE   0.0f
#define MOTOR3_ZERO_ELECTRICAL_ANGLE   5.7377f
#define MOTOR4_ZERO_ELECTRICAL_ANGLE   0.0f
/* 编码器参数 */

/* 开环 SVPWM 控制参数 */
#define OPENLOOP_SUPPLY_VOLTAGE       12.0f

// 注意 OPENLOOP_UQ 不应超过 OPENLOOP_SUPPLY_VOLTAGE 的 1/2
#define OPENLOOP_UQ                   4.0f
#define OPENLOOP_CONTROL_PERIOD_MS    1U
#define OPENLOOP_CONTROL_PERIOD_S     \
				((float)OPENLOOP_CONTROL_PERIOD_MS * 0.001f)


/* 开环 SVPWM 控制参数 */


/*
 * 开环 SVPWM 控制。
 *
 * motor_id：                电机编号，1~4
 * mechanical_speed_rpm：    目标机械转速，单位 RPM。
 *     正负数表示两个旋转方向。
 *
 * 需先调用 MOTOR_Start()，按 OPENLOOP_CONTROL_PERIOD_MS 周期调用。
 * 输入 0 会关闭对应驱动并清除开环角度；重新运行需再次启动驱动。
 * 同一电机每周期只能选择一种控制方式。
 */
void OpenLoopSVPWM(uint8_t motor_id,float mechanical_speed_rpm);


/* 伺服控制：初始化一次，之后只调 Motor_ServoControl */
#define MOTOR_SERVO_MODE_CURRENT   0U
#define MOTOR_SERVO_MODE_SPEED     1U
#define MOTOR_SERVO_MODE_POSITION  2U

/*
 * 初始化：复位电流环与伺服状态，必须在 MOTOR_Start() 之后调用一次。
 */
void Motor_ServoInit(uint8_t motor_id);

/*
 * 三环串接控制，按 foc_period_us 周期调用（现在由 TIM6 中断 1kHz 调用）。
 *   mode 0 CURRENT : target = iq 电流 (A)
 *   mode 1 SPEED   : target = 转速 (RPM)
 *   mode 2 POSITION: target = 机械角 (度)
 * outer_multiple：外环分频，速度环每 outer_multiple 次控制更新一次。
 */
void Motor_ServoControl(uint8_t motor_id,uint8_t mode,float target,uint32_t foc_period_us,uint16_t outer_multiple);

/*
 * 只读状态，给上报用：转速 RPM 与 iq 指令 A。不调用不影响控制。
 */
uint8_t Motor_ServoGetStatus(uint8_t motor_id,float *speed_rpm,float *iq_target);
#endif
