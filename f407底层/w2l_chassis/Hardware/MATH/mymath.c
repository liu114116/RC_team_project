#include "mymath.h"

float Clamp(float value, float min_value, float max_value)
{
    if (value < min_value) value = min_value;
    if (value > max_value) value = max_value;
    return value;
}

void PID_Reset(PID_t *pid)
{
    if (pid == 0) return;
    pid->integral = 0.0f;
}

float PID_Calc(PID_t *pid, float ref, float fdb, float dt)
{
    float err;
    float out;

    if (pid == 0 || dt <= 0.0f) return 0.0f;

    err = ref - fdb;

    out = pid->kp * err + pid->ki * pid->integral;
    if ((out < pid->out_max && out > pid->out_min) ||
        (out >= pid->out_max && err < 0.0f) ||
        (out <= pid->out_min && err > 0.0f))
    {
        pid->integral += err * dt;
    }

    out = pid->kp * err + pid->ki * pid->integral;
    if (out > pid->out_max) out = pid->out_max;
    if (out < pid->out_min) out = pid->out_min;

    return out;
}

