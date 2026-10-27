#include "myfoc.h"
#include "motor.h"
#include <math.h>
#include "mymath.h"
#include "AS5600.h"

/**************************电机参数设置**************************/
typedef struct
{
    uint8_t pole_pairs;
    int8_t  direction;
    float   angle_offset;   // 电角度零点，标定后填
} MotorConfig_t;

static const MotorConfig_t motor_config[4] =
{
    { MOTOR1_POLE_PAIRS, MOTOR1_DIRECTION, 0.0f },
    { MOTOR2_POLE_PAIRS, MOTOR2_DIRECTION, 0.0f },
    { MOTOR3_POLE_PAIRS, MOTOR3_DIRECTION, 0.0f },
    { MOTOR4_POLE_PAIRS, MOTOR4_DIRECTION, 4.98f }
};

//取电机参数 
static const MotorConfig_t *GetMotorConfig(uint8_t motor_id)
{
    if (motor_id < 1U || motor_id > 4U) return 0;
    return &motor_config[motor_id - 1U];
}

/**************************电机参数设置**************************/

/**************************闭环参数设置**************************/
#define CURRENT_LOOP_DT     0.0010f    /* 1 kHz，和 TIM6 一致 */
#define SPEED_LOOP_PERIOD_S (0.001f * (float)SPEED_LOOP_PERIOD_MS)   /* 速度环节拍，见 myfoc.h */

/* ---- 速度前馈系数（A/rpm）：带载后按 iq_steady / target 标定，空载先不用 ---- */
/* #define SPEED_FF_GAIN    0.000125f */

static PID_t pid_d[4] =
{
    {  0.30f,  60.0f, -6.9f, 6.9f, 0.0f },
    {  0.30f,  60.0f, -6.9f, 6.9f, 0.0f },
    {  0.30f,  60.0f, -6.9f, 6.9f, 0.0f },
    {  0.30f,  60.0f, -6.9f, 6.9f, 0.0f }
};

static PID_t pid_q[4] =
{
    {  0.30f,  60.0f, -6.9f, 6.9f, 0.0f },
    {  0.30f,  60.0f, -6.9f, 6.9f, 0.0f },
    {  0.30f,  60.0f, -6.9f, 6.9f, 0.0f },
    {  0.30f,  60.0f, -6.9f, 6.9f, 0.0f }
};

static PID_t pid_speed[4] =
{
    {  0.0003f,  0.0005f, -1.5f, 1.5f, 0.0f },
    {  0.0003f,  0.0005f, -1.5f, 1.5f, 0.0f },
    {  0.0003f,  0.0005f, -1.5f, 1.5f, 0.0f },
    {  0.0003f,  0.0005f, -1.5f, 1.5f, 0.0f }
};

/* 速度环状态 */
typedef struct
{
    float    last_thm;   /* 上一拍的角度 */
    float    iq;         /* 算出的 iq 目标，中断里电流环用 */
    uint32_t tick;       /* 上一次调用的时间 */
    uint8_t  ready;      /* 是否已记录起点 */
} SpeedState_t;

static SpeedState_t spd[4];


/* 速度环 -> 电流环的 iq 目标（A），下标 0~3 = M1~M4 */
volatile float g_iq[4] = {0.0f, 0.0f, 0.0f, 0.0f};
/* 速度环实测转速（rpm），已乘 direction，调试/上报用 */
volatile float g_rpm[4] = {0.0f, 0.0f, 0.0f, 0.0f};

/**************************闭环参数设置**************************/

/**************************开环参数设置**************************/
#define OPENLOOP_SUPPLY_VOLTAGE   12.0f     // 母线电压

#define OPENLOOP_UQ               3.0f     // 交轴电压
#define OPENLOOP_CONTROL_PERIOD_S 0.001f   // 调用周期

// 每个电机的开环电角度
static float open_loop_angle[4] = {0.0f, 0.0f, 0.0f, 0.0f};

/**************************开环参数设置**************************/

/************************坐标变换与 SVPWM****************************/
static void InvParkClarke(float ud, float uq, float angle,
                          float *va, float *vb, float *vc)
{
    float sin_angle;
    float cos_angle;
    float u_alpha;
    float u_beta;
    if (va == 0 || vb == 0 || vc == 0) return;
    sin_angle = sinf(angle);
    cos_angle = cosf(angle);
    // 逆 Park
    u_alpha = ud * cos_angle - uq * sin_angle;
    u_beta  = ud * sin_angle + uq * cos_angle;
    // 逆 Clarke
    *va = u_alpha;
    *vb = -0.5f * u_alpha + 0.5f * SQRT3 * u_beta;
    *vc = -0.5f * u_alpha - 0.5f * SQRT3 * u_beta;
}

// 正 Park + 正 Clarke 合并：两相电流 -> dq 轴电流。
// ia、ib：A、B 两相电流（A）；angle：电角度（rad）；id、iq：输出（A）
// 与 InvParkClarke 严格互为逆变换
void ParkClarke(float ia, float ib, float angle, float *id, float *iq)
{
    float sin_angle;
    float cos_angle;
    float i_alpha;
    float i_beta;

    if (id == 0 || iq == 0) return;

    // 正 Clarke：ic = -(ia + ib) 直接代入
    i_alpha = ia;
    i_beta  = (ia + 2.0f * ib) / SQRT3;

    sin_angle = sinf(angle);
    cos_angle = cosf(angle);

    // 正 Park
    *id =  i_alpha * cos_angle + i_beta * sin_angle;
    *iq = -i_alpha * sin_angle + i_beta * cos_angle;
}



// InvParkClarke输出到电机的接口
static void SetSVPWM(uint8_t motor_id,
          float va, float vb, float vc,float supply_voltage)
{
    float voltage_max;
    float voltage_min;
    float voltage_zero;
    float duty_a;
    float duty_b;
    float duty_c;
    if (supply_voltage <= 0.0f) return;
    voltage_max = va;
    if (vb > voltage_max) voltage_max = vb;
    if (vc > voltage_max) voltage_max = vc;

    voltage_min = va;
    if (vb < voltage_min) voltage_min = vb;
    if (vc < voltage_min) voltage_min = vc;

    /* 加入相同的零序电压，使最大值和最小值居中 */
    voltage_zero = -0.5f * (voltage_max + voltage_min);

    duty_a = 0.5f + (va + voltage_zero) / supply_voltage;
    duty_b = 0.5f + (vb + voltage_zero) / supply_voltage;
    duty_c = 0.5f + (vc + voltage_zero) / supply_voltage;

    duty_a = Clamp(duty_a, 0.0f, 1.0f);
    duty_b = Clamp(duty_b, 0.0f, 1.0f);
    duty_c = Clamp(duty_c, 0.0f, 1.0f);

    MOTOR_SetPWM(motor_id, duty_a, duty_b, duty_c);
}
/************************坐标变换与 SVPWM****************************/


/*************************使用接口设置***************************/
void OpenLoopSVPWM(uint8_t motor_id, float mechanical_speed_rpm)
{
    const MotorConfig_t *cfg = GetMotorConfig(motor_id);
    float electrical_speed;
    float voltage_a;
    float voltage_b;
    float voltage_c;

    if (cfg == 0) return;

    if (mechanical_speed_rpm == 0.0f)
    {
        open_loop_angle[motor_id - 1U] = 0.0f;
        MOTOR_Stop(motor_id);
        return;
    }

    // 机械转速 -> 电角速度
    electrical_speed = mechanical_speed_rpm * _2PI / 60.0f* (float)cfg->pole_pairs * (float)cfg->direction;

    open_loop_angle[motor_id - 1U] += electrical_speed * OPENLOOP_CONTROL_PERIOD_S;
    open_loop_angle[motor_id - 1U] = fmodf(open_loop_angle[motor_id - 1U], _2PI);

    if (open_loop_angle[motor_id - 1U] < 0.0f) open_loop_angle[motor_id - 1U] += _2PI;

    InvParkClarke(0.0f, OPENLOOP_UQ, open_loop_angle[motor_id - 1U],&voltage_a, &voltage_b, &voltage_c);
    SetSVPWM(motor_id, voltage_a, voltage_b, voltage_c, OPENLOOP_SUPPLY_VOLTAGE);
}


void CurrentReset(uint8_t motor_id)
{
    if (motor_id < 1U || motor_id > 4U) return;
    PID_Reset(&pid_d[motor_id - 1U]);
    PID_Reset(&pid_q[motor_id - 1U]);
}

void CurrentLoop(uint8_t motor_id, float theta_m, float id_ref_a, float iq_ref_a)
{
    const MotorConfig_t *cfg = GetMotorConfig(motor_id);
    float theta_e;
    float id = 0.0f;
    float iq = 0.0f;
    float ud;
    float uq;
    float va;
    float vb;
    float vc;

    if (cfg == 0) return;

    theta_e = (float)cfg->direction * (float)cfg->pole_pairs * theta_m + cfg->angle_offset;
    while (theta_e < 0.0f)   theta_e += _2PI;
    while (theta_e >= _2PI)  theta_e -= _2PI;

    ParkClarke(MOTOR_GetCurrent(motor_id, MOTOR_PHASE_A),
               MOTOR_GetCurrent(motor_id, MOTOR_PHASE_B),
               theta_e, &id, &iq);

    ud = PID_Calc(&pid_d[motor_id - 1U], id_ref_a, id, CURRENT_LOOP_DT);
    uq = PID_Calc(&pid_q[motor_id - 1U], iq_ref_a, iq, CURRENT_LOOP_DT);

    InvParkClarke(ud, uq, theta_e, &va, &vb, &vc);
    SetSVPWM(motor_id, va, vb, vc, OPENLOOP_SUPPLY_VOLTAGE);
}


//速度环
float SpeedLoop(uint8_t motor_id, float target_rpm, float theta_m)
{
    const MotorConfig_t *cfg = GetMotorConfig(motor_id);
    SpeedState_t *s;
    uint32_t now;
    float dt;
    float d;
    float rpm;

    if (cfg == 0) return 0.0f;

    s   = &spd[motor_id - 1U];
    now = HAL_GetTick();

    /* 第一拍只记起点，不输出 */
    if (s->ready == 0U)
    {
        s->last_thm = theta_m;
        s->tick     = now;
        s->ready    = 1U;
        return 0.0f;
    }

    dt = (float)(now - s->tick) * 0.001f;

    /* 两拍间隔太短，这一拍不算 */
    if (dt < SPEED_LOOP_PERIOD_S * 0.5f) return s->iq;

    s->tick = now;

    d = theta_m - s->last_thm;
    s->last_thm = theta_m;

    /* 过零处理：差值超过半圈，说明跨了 0/2pi */
    if (d >  3.14159265f) d -= 6.28318531f;
    if (d < -3.14159265f) d += 6.28318531f;

    /*
     * 换算成"iq 为正"的转速：direction = -1 时 iq 为正会让 theta_m 递减，
     * 所以机械角变化要乘 direction，否则速度环变成正反馈。
     */
    rpm = d * (60.0f / 6.28318531f) / dt * (float)cfg->direction;

    g_rpm[motor_id - 1U] = rpm;

    /*
     * 注：这里不再加速度前馈。空载时 200rpm 的稳态电流接近 0，前馈给任何正值都会让转速超调；
     * 等装上负载、有了确定的稳态电流后，可以再按 SPEED_FF_GAIN = iq_steady / target 加回来。
     */
    s->iq = PID_Calc(&pid_speed[motor_id - 1U], target_rpm, rpm, dt);
    g_iq[motor_id - 1U] = s->iq;

    return s->iq;
}

/* 清速度环 + 电流环的状态，开关电机时调用 */
void SpeedReset(uint8_t motor_id)
{
    if (motor_id < 1U || motor_id > 4U) return;

    CurrentReset(motor_id);

    PID_Reset(&pid_speed[motor_id - 1U]);

    spd[motor_id - 1U].last_thm = 0.0f;
    spd[motor_id - 1U].iq       = 0.0f;
    spd[motor_id - 1U].tick     = 0U;
    spd[motor_id - 1U].ready    = 0U;

    g_iq[motor_id - 1U]  = 0.0f;
    g_rpm[motor_id - 1U] = 0.0f;
}

/*************************使用接口设置***************************/

/*************************调试函数***************************/
//角度标定angle_offset
void AlignMotor(uint8_t motor_id, float voltage)
{
    float va;
    float vb;
    float vc;
    if (motor_id < 1U || motor_id > 4U) return;
    if (voltage <= 0.0f) return;
    /* d 轴给电压、电角度固定 0：磁场指向 A 相轴，转子被吸过去 */
    InvParkClarke(voltage, 0.0f, 0.0f, &va, &vb, &vc);
    SetSVPWM(motor_id, va, vb, vc, OPENLOOP_SUPPLY_VOLTAGE);
    HAL_Delay(1000U);   /* 等转子停稳，电压保持输出 */
}
/*************************调试函数***************************/



