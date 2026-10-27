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
 * 用于：
 *     electrical_angle =
 *         SENSOR_DIRECTION * POLE_PAIRS * mechanical_angle - zero;
 *     将 electrical_angle 折返到 [0, 2π)。
 *
 * 当前 0.0f 仅为占位，尚未校准。
 * 开环控制不使用该值；有感驱动前必须确认实际零位。
 *
 * 校准方法：
 * 1. 确认对应电机的极对数和编码器方向。
 * 2. 调用 Motor_SetAlignmentVoltage()，使转子对齐到 angle = 0。
 * 3. 转子稳定且编码器读取成功后，读取：
 *        Encoder_ReadAngle(motor_id, &mechanical_angle);
 * 4. 计算：
 *        zero = SENSOR_DIRECTION * POLE_PAIRS * mechanical_angle;
 *    将 zero 折返到 [0, 2π)。
 * 5. 从不同初始位置重复对齐，确认结果基本一致。
 *    比较时注意 0 和 2π 附近的数值代表相近方向。
 * 6. 将结果填入对应宏，重新编译并烧录。
 *
 * 注意：
 *     不要直接把任意位置的编码器读数当作零位。
 *     不用 Motor_ReadElectricalAngle() 的结果直接作为新零位，
 *     因为该函数已经减去了原有零位。
 *     四个电机分别校准、分别保存。
 *     编码器与转子的相对安装位置、PWM 相序或编码器方向改变后，
 *     需要重新校准。
 *     此校准与 MOTOR_CAL 电流检测放大器校准不同。
 */
/*
 * 电角度零点（弧度）：编码器读数换算到电角度时的偏移。
 * 由 Motor_CalibrateZeroAngle 标定得出，每个电机不同。
 * 标定方法：施加 d 轴电压让转子对齐，读回机械角乘极对数。
 */
#define MOTOR1_ZERO_ELECTRICAL_ANGLE   0.0f
#define MOTOR2_ZERO_ELECTRICAL_ANGLE   0.0f
#define MOTOR3_ZERO_ELECTRICAL_ANGLE   5.7377f
#define MOTOR4_ZERO_ELECTRICAL_ANGLE   0.0f
/* 编码器参数 */

/* 开环SPWM控制参数 */
#define OPENLOOP_SUPPLY_VOLTAGE       12.0f

// 注意 OPENLOOP_UQ 不应超过 OPENLOOP_SUPPLY_VOLTAGE 的 1/2
#define OPENLOOP_UQ                   4.0f
#define OPENLOOP_CONTROL_PERIOD_MS    1U
#define OPENLOOP_CONTROL_PERIOD_S     \
				((float)OPENLOOP_CONTROL_PERIOD_MS * 0.001f)


/* 开环SPWM控制参数 */

/*
 * 速度 PI 参数：目前未调参，P/I 先设为 0。
 * MOTORx_SPEED_UQ_LIMIT 应不大于母线电压的 1/√3，
 * 否则电压输出层还会再次限幅，PI 的限幅与实际输出就不一致。
 * 该参数是电压上限，不是电流限制。
 */
#define MOTOR1_SPEED_KP          0.0f
#define MOTOR1_SPEED_KI          0.0f
#define MOTOR1_SPEED_UQ_LIMIT    1.0f
#define MOTOR2_SPEED_KP          0.0f
#define MOTOR2_SPEED_KI          0.0f
#define MOTOR2_SPEED_UQ_LIMIT    1.0f
#define MOTOR3_SPEED_KP          0.0f
#define MOTOR3_SPEED_KI          0.0f
#define MOTOR3_SPEED_UQ_LIMIT    1.0f
#define MOTOR4_SPEED_KP          0.0f
#define MOTOR4_SPEED_KI          0.0f
#define MOTOR4_SPEED_UQ_LIMIT    1.0f

/* 调试变量 */
extern float m1a;
extern float m1b;
extern float m1c;
/* 调试变量 */

/*
 * 输出固定方向的转子对齐电压，用于校准编码器电角度零位。
 *
 * motor_id：电机编号，1~4
 * align_voltage：正的对齐电压幅值，单位 V
 *
 * 内部固定使用：
 *     Ud = align_voltage
 *     Uq = 0
 *     angle = 0 rad
 *
 * 使用顺序：
 *     MOTOR_Start(motor_id);
 *     等待驱动就绪；
 *     Motor_SetAlignmentVoltage(motor_id, align_voltage);
 *     等待转子对齐并稳定；
 *     读取 Encoder_ReadAngle(motor_id, &mechanical_angle)，检查返回值；
 *     MOTOR_Stop(motor_id);
 *
 * 调参方法：
 *     空载、限流并监测电流，从低电压开始逐次试验。
 *     每次试验结束先关闭驱动，再按需小幅增加电压。
 *     选择能可靠对齐的较低电压，不要持续增加 Ud。
 *     开环旋转时能使用的电压，不一定适合静止对齐。
 *
 * 对齐现象：
 *     转子可能小幅移动后停住，不应持续旋转或摆动。
 *     不动不代表成功，也可能是电压不足或摩擦卡住。
 *     应确认编码器读数稳定，并从不同初始位置重复验证：
 *     得到的电角度零位应在一圈范围内基本一致。
 *
 * 注意：
 *     本函数只设置电压，不负责启动、等待、读取或停止。
 *     PWM 会保持最后的输出，必须调用 MOTOR_Stop() 结束对齐。
 *     期间不要调用其他电机控制函数覆盖该电机的 PWM。
 *     align_voltage <= 0 时直接返回，不会清除原来的输出。
 *     内部电压限幅仅保护调制范围，不是电流限制。
 *     电流异常、快速发热或持续摆动时，应立即停止。
 */
void Motor_SetAlignmentVoltage(uint8_t motor_id, float align_voltage);


/*
 * 开环 SPWM 控制。
 *
 * motor_id：                电机编号，1~4
 * mechanical_speed_rpm：    目标机械转速，单位 RPM。
 *     正负数表示两个旋转方向。
 *
 * 需先调用 MOTOR_Start()，按 OPENLOOP_CONTROL_PERIOD_MS 周期调用。
 * 输入 0 会关闭对应驱动并清除开环角度；重新运行需再次启动驱动。
 * 同一电机每周期只能选择一种控制方式。
 */
void OpenLoopSPWM(uint8_t motor_id,float mechanical_speed_rpm);


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


/*
 * 有感电压模式，使用 SVPWM。
 *
 * motor_id：电机编号，1~4
 * uq：q 轴电压指令，单位 V，不是 RPM 或电流。
 *     正负表示相反的转矩方向。
 *
 * 需先启动对应驱动，并在控制周期内持续调用。
 * uq = 0 输出零电压指令，不关闭 ENABLE。
 * 启用前必须确认编码器读取、方向和电角度零位正确。
 * 本接口没有电流限制，也不保证实际转速。
 * 返回 1 表示本次读取成功且已提交 PWM 指令；
 * 返回 0 表示参数无效或读取失败，有效编号对应的读取失败会关闭驱动。
 */
uint8_t Motor_SetVoltageFOC(uint8_t motor_id, float uq);


/*
 * 有感速度控制：速度 PI 输出 Uq，使用 SVPWM，无内层电流环。
 * 每次调用更新 FOC 输出，每 speed_loop_multiple 次更新速度 PI。
 *
 * motor_id：电机编号，1~4
 * target_rpm：目标机械转速，单位 RPM
 * foc_period_us：FOC 调用周期，单位 us
 * speed_loop_multiple：速度环周期与 FOC 周期的倍数
 *
 * 例如：foc_period_us = 1000，speed_loop_multiple = 20，
 * 表示 FOC 周期为 1 ms，速度环周期为 20 ms。
 * 此函数不等待、不调度，外部实际调用周期应与 foc_period_us 一致。
 * 两次角度采样之间的实际机械转动必须小于半圈。
 *
 * 需先调用 MOTOR_Start()，再按控制周期持续调用本函数。
 * 正负对应已校准的编码器/电气坐标方向。
 * target_rpm = 0：关闭驱动并清除速度环状态，不做零速保持。
 * 读取失败时关闭驱动；重新运行需显式启动，不自动恢复。
 * 运行中不改变周期和倍数，修改前先用零目标停止并复位。
 * 不要在主循环和中断中同时调用，或与其他模式同时控制同一电机。
 * 当前使用阻塞 I2C，不适合直接放入高优先级或高频中断。
 * 启用前必须确认极对数、编码器方向和电角度零位。
 */
void Motor_SetSpeedFOC(uint8_t motor_id,float target_rpm,uint32_t foc_period_us,uint16_t speed_loop_multiple);



/* 电流环接口 */
void    Motor_ResetCurrentFOC(uint8_t motor_id);
uint8_t Motor_SetCurrentFOC(uint8_t motor_id,float iq_target,uint32_t foc_period_us);
uint8_t Motor_GetCurrentDQ(uint8_t motor_id,float *id,float *iq,float *ud,float *uq);
float Motor_GetCurrentElec(uint8_t motor_id);
void Motor_SetCurrentIdTarget(uint8_t motor_id,float id_target);
float Motor_GetCurrentMech(uint8_t motor_id);
uint8_t Motor_CalibrateZeroAngle(uint8_t motor_id,float voltage,uint16_t settle_ms,float *zero_out,float *mech_out);
#define MOTOR_SERVO_MODE_CURRENT   0U
#define MOTOR_SERVO_MODE_SPEED     1U
#define MOTOR_SERVO_MODE_POSITION  2U

void    Motor_ServoControl(uint8_t motor_id,uint8_t mode,float target,uint32_t foc_period_us,uint16_t outer_multiple);
void    Motor_ServoReset(uint8_t motor_id);
uint8_t Motor_ServoGetStatus(uint8_t motor_id,float *speed_rpm,float *iq_target);
#endif
