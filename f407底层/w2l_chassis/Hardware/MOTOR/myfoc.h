#ifndef MYFOC_H
#define MYFOC_H

#include <stdint.h>
#define SPEED_LOOP_PERIOD_MS    40U //速度环节拍
extern volatile float g_iq[4]; //电流环的 iq 目标
extern volatile float g_rpm[4]; //转速

/* 开环 SVPWM 控制。 */
void OpenLoopSVPWM(uint8_t motor_id, float mechanical_speed_rpm);

/* 电流环：每 1 ms 调一次，需先 MOTOR_Start() */
void CurrentLoop(uint8_t motor_id, float theta_m, float id_ref_a, float iq_ref_a);
void CurrentReset(uint8_t motor_id);

/* 速度环：主循环每 SPEED_LOOP_PERIOD_MS 调一次，返回 iq 目标（A） */
float SpeedLoop(uint8_t motor_id, float target_rpm, float theta_m);
void SpeedReset(uint8_t motor_id);

// 调试：d 轴加电压把转子吸住，用于标定 angle_offset
void AlignMotor(uint8_t motor_id, float voltage);

#endif /* MYFOC_H */
