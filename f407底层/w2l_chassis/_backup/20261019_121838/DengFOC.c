#include "DengFOC.h"
#include "motor.h"
#include <math.h>

/*****************************简化处理函数*********************************/
static float DengFOC_Clamp(float value, float min_value, float max_value)
{
    if (value < min_value) value = min_value;
    if (value > max_value) value = max_value;
    return value;
}
/*****************************简化处理函数*********************************/

/*****************************数学运算函数*********************************/
void DengFOC_InvPark(float ud, float uq, float angle,
                    float *u_alpha, float *u_beta)
{
    float sin_angle;
    float cos_angle;
    if (u_alpha == 0 || u_beta == 0) return;
	
    sin_angle = sinf(angle);
    cos_angle = cosf(angle);
    *u_alpha = ud * cos_angle - uq * sin_angle;
    *u_beta  = ud * sin_angle + uq * cos_angle;
}

void DengFOC_InvClarke(float u_alpha,float u_beta,
                       float *voltage_a,float *voltage_b,float *voltage_c)
{
    if (voltage_a == 0 ||voltage_b == 0 ||voltage_c == 0)  return;

    *voltage_a = u_alpha;
    *voltage_b = -0.5f * u_alpha + 0.5f * DENGFOC_SQRT3 * u_beta;
    *voltage_c = -0.5f * u_alpha - 0.5f * DENGFOC_SQRT3 * u_beta;
}

float DengFOC_CalcElectricalAngle(float mechanical_angle,uint8_t pole_pairs,
                                int8_t direction,float zero_electrical_angle)
{
    float electrical_angle;
	
    electrical_angle = mechanical_angle* (float)pole_pairs* (float)direction- zero_electrical_angle;
    electrical_angle = fmodf(electrical_angle, DENGFOC_2PI);
    if (electrical_angle < 0.0f) electrical_angle += DENGFOC_2PI;
    return electrical_angle;
}
/*****************************数学运算函数*********************************/


/*****************************SVPWM*********************************/
void DengFOC_SetSVPWM(uint8_t motor_id,
                     float voltage_a, float voltage_b, float voltage_c,
                     float supply_voltage)
{
    float voltage_max;
    float voltage_min;
    float voltage_zero;
    float duty_a;
    float duty_b;
    float duty_c;

    if (supply_voltage <= 0.0f) return;

    /* 找出三相电压的最大值 */
    voltage_max = voltage_a;
    if (voltage_b > voltage_max) voltage_max = voltage_b;
    if (voltage_c > voltage_max) voltage_max = voltage_c;

    /* 找出三相电压的最小值 */
    voltage_min = voltage_a;
    if (voltage_b < voltage_min) voltage_min = voltage_b;
    if (voltage_c < voltage_min) voltage_min = voltage_c;

    /* 给三相加入相同的零序电压，使最大值和最小值居中 */
    voltage_zero = -0.5f * (voltage_max + voltage_min);

    duty_a = 0.5f + (voltage_a + voltage_zero) / supply_voltage;
    duty_b = 0.5f + (voltage_b + voltage_zero) / supply_voltage;
    duty_c = 0.5f + (voltage_c + voltage_zero) / supply_voltage;

    duty_a = DengFOC_Clamp(duty_a, 0.0f, 1.0f);
    duty_b = DengFOC_Clamp(duty_b, 0.0f, 1.0f);
    duty_c = DengFOC_Clamp(duty_c, 0.0f, 1.0f);

    MOTOR_SetPWM(motor_id, duty_a, duty_b, duty_c);
}
/*****************************SVPWM*********************************/
