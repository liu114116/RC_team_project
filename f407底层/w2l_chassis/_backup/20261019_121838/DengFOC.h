#ifndef DENGFOC_H
#define DENGFOC_H
#include <stdint.h>


#define DENGFOC_PI       3.14159265358979323846f
#define DENGFOC_2PI      6.28318530717958647692f
#define DENGFOC_SQRT3    1.73205080756887729353f


/*
 * 机械角换算成电角度，输入和输出角度单位均为 rad。
 *
 * pole_pairs：极对数
 * direction：编码器方向，必须为 +1 或 -1
 * zero_electrical_angle：电角度零位偏移
 *
 * 返回范围：[0, 2π)
 */
float DengFOC_CalcElectricalAngle(float mechanical_angle,uint8_t pole_pairs,
                             int8_t direction,float zero_electrical_angle);


/*
 * 逆 Park：旋转坐标系 dq -> 静止坐标系 alpha/beta
 *
 * ud、uq：           d/q 轴电压，单位 V
 * angle：            dq 坐标系相对 alpha/beta 的角度，单位 rad
 * u_alpha、u_beta：  输出电压，单位 V
 */
void DengFOC_InvPark(float ud, float uq, float angle,
														float *u_alpha, float *u_beta);

/*
 * 逆 Clarke：静止坐标系 alpha/beta -> a/b/c相电压
 *
 *u_alpha、u_beta  αβ 坐标系中的电压
 *voltage_a        A 相电压输出
 *voltage_b        B 相电压输出
 *voltage_c        C 相电压输出
 */
void DengFOC_InvClarke(float u_alpha,float u_beta,
                       float *voltage_a,float *voltage_b,float *voltage_c);


/*
 * SVPWM：三相电压指令转换为 PWM 并输出。
 *
 * voltage_a/b/c：     逆 Clarke 得到的三相电压，单位 V
 * supply_voltage：    实际母线电压，单位 V
 * 输入电压应在 ±supply_voltage/2 内，超出会被限幅。
 * 本函数不启动电机，需事先调用 MOTOR_Start()。
 */
void DengFOC_SetSVPWM(uint8_t motor_id,
                      float voltage_a,float voltage_b,float voltage_c,
                      float supply_voltage);


#endif /* DENGFOC_H */
