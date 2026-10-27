#include "motor_control.h"
#include "DengFOC.h"
#include "motor.h"
#include <math.h>
#include "AS5600.h"
#include "pid.h"


/**********************电机参数相关函数*****************************/
static uint8_t OpenLoop_GetMotorConfig(uint8_t motor_id,uint8_t *pole_pairs,int8_t *direction)
{
    if (pole_pairs == 0 || direction == 0) return 0;
    switch (motor_id)
    {
        case 1:
            *pole_pairs = MOTOR1_POLE_PAIRS;
            *direction = MOTOR1_DIRECTION;
            break;

        case 2:
            *pole_pairs = MOTOR2_POLE_PAIRS;
            *direction = MOTOR2_DIRECTION;
            break;

        case 3:
            *pole_pairs = MOTOR3_POLE_PAIRS;
            *direction = MOTOR3_DIRECTION;
            break;

        case 4:
            *pole_pairs = MOTOR4_POLE_PAIRS;
            *direction = MOTOR4_DIRECTION;
            break;

        default:
            return 0;
    }
    return 1;
}


static uint8_t Motor_ReadElectricalAngle(uint8_t motor_id,float *electrical_angle,float *mechanical_angle_out)
{
    uint8_t pole_pairs;
    int8_t sensor_direction;
    float zero_electrical_angle;
    float mechanical_angle;
    if (electrical_angle == 0)return 0;
    switch (motor_id)
    {
        case 1:
            pole_pairs = MOTOR1_POLE_PAIRS;
            sensor_direction = MOTOR1_SENSOR_DIRECTION;
            zero_electrical_angle = MOTOR1_ZERO_ELECTRICAL_ANGLE;
            break;

        case 2:
            pole_pairs = MOTOR2_POLE_PAIRS;
            sensor_direction = MOTOR2_SENSOR_DIRECTION;
            zero_electrical_angle = MOTOR2_ZERO_ELECTRICAL_ANGLE;
            break;

        case 3:
            pole_pairs = MOTOR3_POLE_PAIRS;
            sensor_direction = MOTOR3_SENSOR_DIRECTION;
            zero_electrical_angle = MOTOR3_ZERO_ELECTRICAL_ANGLE;
            break;

        case 4:
            pole_pairs = MOTOR4_POLE_PAIRS;
            sensor_direction = MOTOR4_SENSOR_DIRECTION;
            zero_electrical_angle = MOTOR4_ZERO_ELECTRICAL_ANGLE;
            break;

        default:
            return 0;
    }
    if (!Encoder_ReadAngle(motor_id, &mechanical_angle))
    {
        return 0;
    }

    if (mechanical_angle_out != 0)
    {
        *mechanical_angle_out = mechanical_angle;
    }

    *electrical_angle = DengFOC_CalcElectricalAngle(mechanical_angle,
                                                    pole_pairs,
                                                    sensor_direction,
                                                    zero_electrical_angle);
    return 1;
}


/**********************电机参数相关函数*****************************/

/**********************开环函数*****************************/
static float open_loop_angle[4] = {0.0f};


void OpenLoopSVPWM(uint8_t motor_id,float mechanical_speed_rpm)
{
    uint8_t pole_pairs;
    int8_t direction;
    float electrical_speed;

    float u_alpha;
    float u_beta;
    float voltage_a;
    float voltage_b;
    float voltage_c;

    if (motor_id < 1U || motor_id > 4U)return;
    if (!OpenLoop_GetMotorConfig(motor_id,&pole_pairs,&direction))return;
    if (mechanical_speed_rpm == 0.0f)
    {
        open_loop_angle[motor_id - 1U] = 0.0f;
        MOTOR_Stop(motor_id);
        return;
    }

    electrical_speed =mechanical_speed_rpm* DENGFOC_2PI/ 60.0f* (float)pole_pairs* (float)direction;

    open_loop_angle[motor_id - 1U] +=electrical_speed * OPENLOOP_CONTROL_PERIOD_S;

    open_loop_angle[motor_id - 1U] =fmodf(open_loop_angle[motor_id - 1U],DENGFOC_2PI);

    if (open_loop_angle[motor_id - 1U] < 0.0f)
    {
        open_loop_angle[motor_id - 1U] += DENGFOC_2PI;
    }
    DengFOC_InvPark(0.0f,OPENLOOP_UQ,open_loop_angle[motor_id - 1U],&u_alpha,&u_beta);
    DengFOC_InvClarke(u_alpha,u_beta,&voltage_a,&voltage_b,&voltage_c);


    DengFOC_SetSVPWM(motor_id,voltage_a,voltage_b,voltage_c,OPENLOOP_SUPPLY_VOLTAGE);
}


/**********************开环函数*****************************/


/************************电流环 FOC*************************/

/*
 * 电流环 PI：输入 d/q 电流误差，输出 d/q 电压，1kHz 运行。
 * 每个电机一组 { kp, ki, 输出限幅(V), 积分初值 }：
 *   kp        ：比例增益，越大响应越快，过大会振荡
 *   ki        ：积分增益，消除稳态误差，过大会超调
 *   输出限幅  ：ud/uq 各自的最大电压(V)，建议不超过 Vbus/√3
 *   积分初值  ：一般填 0
 */
#define CURRENT_LPF_ALPHA   0.500f

static PI motor_current_id_pi[4] =
{
    {0.50f,2.00f,3.0f,0.0f},
    {0.50f,2.00f,3.0f,0.0f},
    {0.50f,2.00f,3.0f,0.0f},
    {0.50f,2.00f,3.0f,0.0f}
};
static PI motor_current_iq_pi[4] =
{
    {1.0f,1.00f,3.0f,0.0f},
    {1.0f,1.00f,3.0f,0.0f},
    {1.0f,1.00f,3.0f,0.0f},
    {1.0f,1.00f,3.0f,0.0f}
};
static float motor_current_id[4];
static float motor_current_iq[4];
static uint8_t motor_current_foc_ready[4];
static float motor_current_mech[4];

static void Motor_ResetCurrentFOC(uint8_t motor_id)
{
    if (motor_id < 1U || motor_id > 4U)return;
    PI_Reset(&motor_current_id_pi[motor_id - 1U]);
    PI_Reset(&motor_current_iq_pi[motor_id - 1U]);
    motor_current_id[motor_id - 1U] = 0.0f;
    motor_current_iq[motor_id - 1U] = 0.0f;
    motor_current_foc_ready[motor_id - 1U] = 0U;
}

/*
 * 电流环：给定 q 轴电流目标，d 轴固定为 0。
 * 必须先把驱动使能（MOTOR_Start），并且零点已校准。
 * 返回 1 正常，0 表示编码器或电流采样失败（此时已关驱动）。
 */
static uint8_t Motor_SetCurrentFOC(uint8_t motor_id,float iq_target,uint32_t foc_period_us)
{
    uint8_t index;
    float mechanical_angle;
    float electrical_angle;
    float current_a;
    float current_b;
    float alpha;
    float beta;
    float cos_angle;
    float sin_angle;
    float id;
    float iq;
    float dt;
    float ud;
    float uq;
    float u_alpha;
    float u_beta;
    float voltage_a;
    float voltage_b;
    float voltage_c;

    if (motor_id < 1U || motor_id > 4U)return 0U;
    if (foc_period_us == 0U)return 0U;

    index = motor_id - 1U;

    if (!Motor_ReadElectricalAngle(motor_id,&electrical_angle,&mechanical_angle))
    {
        Motor_ResetCurrentFOC(motor_id);
        return 0U;
    }
    if (!MOTOR_ReadCurrent(motor_id,&current_a,&current_b))
    {
        Motor_ResetCurrentFOC(motor_id);
        return 0U;
    }

    /* Clarke */
    alpha = current_a;
    beta  = (current_a + 2.0f * current_b) * 0.57735027f;

    /* Park */
    cos_angle = cosf(electrical_angle);
    sin_angle = sinf(electrical_angle);
    id =  alpha * cos_angle + beta * sin_angle;
    iq = -alpha * sin_angle + beta * cos_angle;

    /* first-order low pass on d/q current feedback */
    if (motor_current_foc_ready[index] == 0U)
    {
        motor_current_id[index] = id;
        motor_current_iq[index] = iq;
        motor_current_foc_ready[index] = 1U;
    }
    else
    {
        motor_current_id[index] += CURRENT_LPF_ALPHA *
                                   (id - motor_current_id[index]);
        motor_current_iq[index] += CURRENT_LPF_ALPHA *
                                   (iq - motor_current_iq[index]);
    }
    id = motor_current_id[index];
    iq = motor_current_iq[index];
    motor_current_mech[index] = mechanical_angle;

    dt = (float)foc_period_us * 0.000001f;

    ud = PI_Update(&motor_current_id_pi[index],0.0f - id,dt);
    uq = PI_Update(&motor_current_iq_pi[index],iq_target - iq,dt);

    /* 电压矢量限幅，避免超出线性调制区 */
    {
        float magnitude = sqrtf(ud * ud + uq * uq);
        if (magnitude > 3.0f)
        {
            float scale = 3.0f / magnitude;
            ud *= scale;
            uq *= scale;
        }
    }

    DengFOC_InvPark(ud,uq,electrical_angle,&u_alpha,&u_beta);
    DengFOC_InvClarke(u_alpha,u_beta,&voltage_a,&voltage_b,&voltage_c);
    DengFOC_SetSVPWM(motor_id,voltage_a,voltage_b,voltage_c,OPENLOOP_SUPPLY_VOLTAGE);

    return 1U;
}

/************************电流环 FOC*************************/

static float Motor_GetCurrentMech(uint8_t motor_id)
{
    if (motor_id < 1U || motor_id > 4U)return 0.0f;
    return motor_current_mech[motor_id - 1U];
}


/************************servo: one function, three modes*************************/

#define SERVO_MODE_CURRENT   0U
#define SERVO_MODE_SPEED     1U
#define SERVO_MODE_POSITION  2U

/*
 * Per-motor servo parameters. One entry per motor so all four can be
 * tuned independently.
 */
typedef struct
{
    float iq_limit;      /* A,   q-axis current limit            */
    float speed_kp;      /*      speed loop proportional gain    */
    float speed_ki;      /*      speed loop integral gain        */
    float speed_filter;  /*      speed feedback LPF, 1.0 = off   */
    float speed_limit;   /* rpm, speed target limit (position)   */
    float pos_kp;        /*      position loop proportional gain */
} ServoParams;

/*
 * 伺服参数表，四个电机各自独立。
 * 字段顺序（每个花括号内从左到右）：
 *   [0] iq_limit       q 轴电流限幅，单位 A。决定最大力矩
 *   [1] speed_kp       速度环比例增益。越大跟随越快，过大会振荡
 *   [2] speed_ki       速度环积分增益。消除稳态转速误差，过大会产生极限环
 *   [3] speed_filter   速度反馈一阶低通。1.0=不过滤，越小越平滑但滞后越大
 *                      （当前 0.05 约 1 秒时间常数，偏重，可试 0.15~0.25）
 *   [4] speed_limit    位置模式时，位置环输出的转速上限，单位 RPM
 *   [5] pos_kp         位置环比例增益。越大到点越快，过大会过冲
 */
static ServoParams servo_params[4] =
{
    /* iq_limit  speed_kp  speed_ki  speed_filter  speed_limit  pos_kp */
    {   1.50f,     0.120f,    0.400f,      0.050f,       60.0f,  3.00f},   /* M1 */
    {   1.50f,     0.120f,    0.400f,      0.050f,       60.0f,  3.00f},   /* M2 */
    {   1.50f,     0.120f,    0.400f,      0.050f,       60.0f,  3.00f},   /* M3 */
    {   1.50f,     0.120f,    0.400f,      0.050f,       60.0f,  3.00f}    /* M4 */
};

static uint16_t servo_count[4];
static float    servo_speed[4];
static float    servo_speed_raw[4];
static float    servo_last_mech[4];
static float    servo_angle_sum[4];
static float    servo_integral[4];
static float    servo_iq[4];
static uint8_t  servo_ready[4];

void Motor_ServoInit(uint8_t motor_id)
{
    if (motor_id < 1U || motor_id > 4U)return;
    servo_count[motor_id - 1U] = 0U;
    servo_speed[motor_id - 1U] = 0.0f;
    servo_speed_raw[motor_id - 1U] = 0.0f;
    servo_integral[motor_id - 1U] = 0.0f;
    servo_iq[motor_id - 1U] = 0.0f;
    servo_ready[motor_id - 1U] = 0U;
    Motor_ResetCurrentFOC(motor_id);
}

uint8_t Motor_ServoGetStatus(uint8_t motor_id,float *speed_rpm,float *iq_target)
{
    if (motor_id < 1U || motor_id > 4U)return 0U;
    if (speed_rpm)*speed_rpm = servo_speed[motor_id - 1U];
    if (iq_target)*iq_target = servo_iq[motor_id - 1U];
    return servo_ready[motor_id - 1U];
}

/*
 * One entry point, three chained modes.
 *   mode 0 CURRENT : target = iq (A),      goes straight to the current loop
 *   mode 1 SPEED   : target = rpm,         speed PI -> iq -> current loop
 *   mode 2 POSITION: target = degrees,     position P -> speed -> iq -> current loop
 * The inner current loop always runs at the call rate (1kHz from TIM6).
 */
void Motor_ServoControl(uint8_t motor_id,uint8_t mode,float target,
                        uint32_t foc_period_us,uint16_t outer_multiple)
{
    uint8_t index;
    float dt;
    float outer_dt;
    float mech;
    float delta;

    if (motor_id < 1U || motor_id > 4U)return;
    if (foc_period_us == 0U || outer_multiple == 0U)return;

    index = motor_id - 1U;
    dt = (float)foc_period_us * 0.000001f;
    outer_dt = dt * (float)outer_multiple;

    mech = Motor_GetCurrentMech(motor_id);

    /*
     * Speed measurement: accumulate the angle delta every call (1kHz) and
     * average over the whole outer window. Equivalent to outer_multiple-point
     * averaging, so the encoder quantization error drops by ~sqrt(N).
     */
    if (!servo_ready[index])
    {
        servo_last_mech[index] = mech;
        servo_angle_sum[index] = 0.0f;
        servo_speed[index] = 0.0f;
        servo_ready[index] = 1U;
    }
    else
    {
        delta = mech - servo_last_mech[index];
        while (delta >  3.14159265f)delta -= 6.28318531f;
        while (delta < -3.14159265f)delta += 6.28318531f;
        servo_last_mech[index] = mech;
        servo_angle_sum[index] += delta;
    }

    servo_count[index]++;
    if (servo_count[index] >= outer_multiple)
    {
        servo_count[index] = 0U;

        servo_speed_raw[index] = servo_angle_sum[index] / outer_dt
                                 * 60.0f / 6.28318531f;
        servo_angle_sum[index] = 0.0f;
        servo_speed[index] += servo_params[index].speed_filter *
                              (servo_speed_raw[index] - servo_speed[index]);

        if (mode == SERVO_MODE_POSITION)
        {
            float position_error = target * 0.017453292f - mech;
            while (position_error >  3.14159265f)position_error -= 6.28318531f;
            while (position_error < -3.14159265f)position_error += 6.28318531f;
            float speed_target = servo_params[index].pos_kp * position_error;

            if (speed_target >  servo_params[index].speed_limit)speed_target =  servo_params[index].speed_limit;
            if (speed_target < -servo_params[index].speed_limit)speed_target = -servo_params[index].speed_limit;

            {
                float error = speed_target - servo_speed[index];
                servo_integral[index] += servo_params[index].speed_ki * error * outer_dt;
                if (servo_integral[index] >  servo_params[index].iq_limit)servo_integral[index] =  servo_params[index].iq_limit;
                if (servo_integral[index] < -servo_params[index].iq_limit)servo_integral[index] = -servo_params[index].iq_limit;
                servo_iq[index] = servo_params[index].speed_kp * error + servo_integral[index];
            }
        }
        else if (mode == SERVO_MODE_SPEED)
        {
            float error = target - servo_speed[index];

            servo_integral[index] += servo_params[index].speed_ki * error * outer_dt;
            if (servo_integral[index] >  servo_params[index].iq_limit)servo_integral[index] =  servo_params[index].iq_limit;
            if (servo_integral[index] < -servo_params[index].iq_limit)servo_integral[index] = -servo_params[index].iq_limit;
            servo_iq[index] = servo_params[index].speed_kp * error + servo_integral[index];
        }
        else
        {
            servo_iq[index] = target;
            servo_integral[index] = 0.0f;
        }

        if (servo_iq[index] >  servo_params[index].iq_limit)servo_iq[index] =  servo_params[index].iq_limit;
        if (servo_iq[index] < -servo_params[index].iq_limit)servo_iq[index] = -servo_params[index].iq_limit;
    }

    (void)Motor_SetCurrentFOC(motor_id,servo_iq[index],foc_period_us);
}
