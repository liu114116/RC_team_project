#include "adc.h"
#include "motor.h"

/*****************************简化处理函数*********************************/
typedef struct
{
    TIM_HandleTypeDef *timer;
    uint32_t channel_a;
    uint32_t channel_b;
    uint32_t channel_c;
    GPIO_TypeDef *enable_port;
    uint16_t enable_pin;
} MOTOR_Hardware_t;


static uint8_t MOTOR_GetHardware(uint8_t motor_id,MOTOR_Hardware_t *hardware)
{
    if (hardware == 0)
    {
        return 0;
    }
    switch (motor_id)
    {
        case 1:
            hardware->timer = MOTOR1_PWM_TIMER;
            hardware->channel_a = MOTOR1_PWM_A_CHANNEL;
            hardware->channel_b = MOTOR1_PWM_B_CHANNEL;
            hardware->channel_c = MOTOR1_PWM_C_CHANNEL;
            hardware->enable_port = MOTOR1_ENABLE_PORT;
            hardware->enable_pin = MOTOR1_ENABLE_PIN;
            break;
        case 2:
            hardware->timer = MOTOR2_PWM_TIMER;
            hardware->channel_a = MOTOR2_PWM_A_CHANNEL;
            hardware->channel_b = MOTOR2_PWM_B_CHANNEL;
            hardware->channel_c = MOTOR2_PWM_C_CHANNEL;
            hardware->enable_port = MOTOR2_ENABLE_PORT;
            hardware->enable_pin = MOTOR2_ENABLE_PIN;
            break;
        case 3:
            hardware->timer = MOTOR3_PWM_TIMER;
            hardware->channel_a = MOTOR3_PWM_A_CHANNEL;
            hardware->channel_b = MOTOR3_PWM_B_CHANNEL;
            hardware->channel_c = MOTOR3_PWM_C_CHANNEL;
            hardware->enable_port = MOTOR3_ENABLE_PORT;
            hardware->enable_pin = MOTOR3_ENABLE_PIN;
            break;
        case 4:
            hardware->timer = MOTOR4_PWM_TIMER;
            hardware->channel_a = MOTOR4_PWM_A_CHANNEL;
            hardware->channel_b = MOTOR4_PWM_B_CHANNEL;
            hardware->channel_c = MOTOR4_PWM_C_CHANNEL;
            hardware->enable_port = MOTOR4_ENABLE_PORT;
            hardware->enable_pin = MOTOR4_ENABLE_PIN;
            break;
        default:
            return 0;
    }
    return 1;
}

static inline float MOTOR_ClampDuty(float duty)
{
    if (duty < 0.0f) return 0.0f;
    if (duty > 1.0f) return 1.0f;
    return duty;
}

/*
 * 将 0.0f~1.0f 的占空比转换为 CCR 数值
 */
static uint32_t MOTOR_DutyToCompare(TIM_HandleTypeDef *timer,float duty)
{
    uint32_t arr;
    duty = MOTOR_ClampDuty(duty);
    arr = __HAL_TIM_GET_AUTORELOAD(timer);
    return (uint32_t)(duty * (float)arr);
}

void MOTOR_SetCalibrate(uint8_t calibrate)
{
    HAL_GPIO_WritePin(MOTOR_CAL_PORT,MOTOR_CAL_PIN,
                      calibrate ? MOTOR_CAL_CALIBRATE_LEVEL : MOTOR_CAL_NORMAL_LEVEL);
}
/*****************************简化处理函数*********************************/

/******************************电机控制函数*******************************/
void MOTOR_Init(void)
{
    uint8_t motor_id;
    for (motor_id = 1; motor_id <= 4; motor_id++)
    {
        MOTOR_Stop(motor_id);
    }
    MOTOR_SetCalibrate(0);
}

void MOTOR_Start(uint8_t motor_id)
{
    MOTOR_Hardware_t hardware;
    if (!MOTOR_GetHardware(motor_id, &hardware)) return;
		
    HAL_GPIO_WritePin(hardware.enable_port,
								hardware.enable_pin,MOTOR_DISABLE_LEVEL);

    HAL_TIM_PWM_Start(hardware.timer, hardware.channel_a);
    HAL_TIM_PWM_Start(hardware.timer, hardware.channel_b);
    HAL_TIM_PWM_Start(hardware.timer, hardware.channel_c);
    MOTOR_SetPWM(motor_id, 0.5f, 0.5f, 0.5f);
	
    HAL_GPIO_WritePin(hardware.enable_port,
										hardware.enable_pin,MOTOR_ENABLE_LEVEL);
}

void MOTOR_Stop(uint8_t motor_id)
{
    MOTOR_Hardware_t hardware;
    if (!MOTOR_GetHardware(motor_id, &hardware)) return;

    HAL_GPIO_WritePin(hardware.enable_port,
                      hardware.enable_pin,MOTOR_DISABLE_LEVEL);
	
    MOTOR_SetPWM(motor_id, 0.5f, 0.5f, 0.5f);
}

void MOTOR_SetPWM(uint8_t motor_id,float duty_a,float duty_b,float duty_c)
{
    MOTOR_Hardware_t hardware;
    if (!MOTOR_GetHardware(motor_id, &hardware)) return;
	
    __HAL_TIM_SET_COMPARE(hardware.timer,hardware.channel_a,
        MOTOR_DutyToCompare(hardware.timer, duty_a));
	
    __HAL_TIM_SET_COMPARE(hardware.timer,hardware.channel_b,
        MOTOR_DutyToCompare(hardware.timer, duty_b));
	
    __HAL_TIM_SET_COMPARE(hardware.timer,hardware.channel_c,
        MOTOR_DutyToCompare(hardware.timer, duty_c));
}

/******************************电机控制函数*******************************/


