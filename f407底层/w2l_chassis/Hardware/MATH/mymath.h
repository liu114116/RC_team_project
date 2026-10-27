#ifndef MYMATH_H
#define MYMATH_H

#define _2PI                      6.28318530717958647692f
#define SQRT3                     1.73205080756887729353f

float Clamp(float value, float min_value, float max_value);

/* 通用 PID（电流环、速度环共用） */
typedef struct
{
    float kp;
    float ki;
    float out_min;
    float out_max;
    float integral; 
} PID_t;

void  PID_Reset(PID_t *pid);
float PID_Calc(PID_t *pid, float ref, float fdb, float dt);

#endif /* MYMATH_H */

