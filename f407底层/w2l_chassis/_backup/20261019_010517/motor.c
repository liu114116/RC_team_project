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

/************************电流采样部分*************************/

/*
 * ADC1 规则组扫描顺序（见 adc.c）：
 *   rank1=CH8  rank2=CH9  rank3=CH14 rank4=CH15
 *   rank5=CH12 rank6=CH13 rank7=CH10 rank8=CH11
 * 每一路采样电阻上接的是哪个电机的哪一相：
 *   M1: A=rank1 B=rank2
 *   M2: A=rank4 B=rank3
 *   M3: A=rank5 B=rank6
 *   M4: A=rank8 B=rank7
 */
static const uint8_t MOTOR_ADC_RANK[4][2] =
{
    {0U,1U},
    {3U,2U},
    {4U,5U},
    {7U,6U}
};

/* 硬件参数：10mΩ 采样电阻、增益 20V/V、参考 3.3V、12 位 ADC */
#define MOTOR_CURRENT_SHUNT      0.0100f
#define MOTOR_CURRENT_GAIN       20.0f
#define MOTOR_CURRENT_VREF       3.3f
#define MOTOR_ADC_FULL_SCALE     4095.0f
/* 实测：电流方向与采样放大器相反，取 -1 */
#define MOTOR_CURRENT_SIGN       (-1.0f)

static uint16_t motor_scan_raw[8];
static uint16_t motor_current_raw[4][2];
static float    motor_current_offset[4][2];
static uint8_t  motor_current_cache_valid = 0U;
static uint8_t  motor_current_last_id = 0xFFU;
static uint8_t  motor_current_stale = 0U;

/*
 * 扫一遍规则组 8 个通道。
 * 不用 HAL_ADC_PollForConversion（它按毫秒计时），
 * 直接查 EOC 标志 + 计数器限界，最坏几十微秒退出，不依赖 SysTick。
 */
static uint8_t MOTOR_CurrentScan(void)
{
    uint8_t i;
    uint8_t ok = 1U;
    uint32_t guard;

    HAL_ADC_Stop(&hadc1);
    if (HAL_ADC_Start(&hadc1) != HAL_OK)
    {
        return 0U;
    }

    for (i = 0U; i < 8U; i++)
    {
        guard = 0U;
        while (__HAL_ADC_GET_FLAG(&hadc1,ADC_FLAG_EOC) == RESET)
        {
            guard++;
            if (guard > 20000U)
            {
                ok = 0U;
                break;
            }
        }
        if (ok == 0U)
        {
            break;
        }
        motor_scan_raw[i] = (uint16_t)HAL_ADC_GetValue(&hadc1);
    }
    HAL_ADC_Stop(&hadc1);

    if (ok == 0U)
    {
        return 0U;
    }

    for (i = 0U; i < 4U; i++)
    {
        motor_current_raw[i][0] = motor_scan_raw[MOTOR_ADC_RANK[i][0]];
        motor_current_raw[i][1] = motor_scan_raw[MOTOR_ADC_RANK[i][1]];
    }
    motor_current_cache_valid = 1U;
    return 1U;
}

void MOTOR_InitCurrentSensor(void)
{
    uint8_t i;
    uint8_t j;

    for (i = 0U; i < 4U; i++)
    {
        for (j = 0U; j < 2U; j++)
        {
            motor_current_offset[i][j] = 2048.0f;
            motor_current_raw[i][j] = 2048U;
        }
    }
    motor_current_cache_valid = 0U;
    motor_current_last_id = 0xFFU;
    motor_current_stale = 0U;

    HAL_ADCEx_InjectedStop(&hadc1);
    (void)MOTOR_CurrentScan();
}

uint8_t MOTOR_ReadCurrentRaw(uint8_t motor_id,uint16_t *raw_a,uint16_t *raw_b)
{
    if (raw_a == 0 || raw_b == 0)return 0U;
    if (motor_id < 1U || motor_id > 4U)return 0U;

    /* 每轮先扫一次，四个电机共用一次扫描结果 */
    if (motor_id <= motor_current_last_id || motor_current_stale >= 4U)
    {
        (void)MOTOR_CurrentScan();
        motor_current_stale = 0U;
    }
    motor_current_stale++;
    motor_current_last_id = motor_id;

    if (motor_current_cache_valid == 0U)return 0U;

    *raw_a = motor_current_raw[motor_id - 1U][0];
    *raw_b = motor_current_raw[motor_id - 1U][1];
    return 1U;
}

uint8_t MOTOR_ReadCurrent(uint8_t motor_id,float *current_a,float *current_b)
{
    uint16_t raw_a;
    uint16_t raw_b;
    float scale;

    if (current_a == 0 || current_b == 0)return 0U;
    if (!MOTOR_ReadCurrentRaw(motor_id,&raw_a,&raw_b))return 0U;

    scale = MOTOR_CURRENT_VREF /
            (MOTOR_ADC_FULL_SCALE * MOTOR_CURRENT_GAIN * MOTOR_CURRENT_SHUNT);

    *current_a = ((motor_current_offset[motor_id - 1U][0] - (float)raw_a) * scale)
                 * MOTOR_CURRENT_SIGN;
    *current_b = ((motor_current_offset[motor_id - 1U][1] - (float)raw_b) * scale)
                 * MOTOR_CURRENT_SIGN;
    return 1U;
}

/*
 * 零点校准：驱动必须已使能、三相占空比相同（此时相电流为 0）。
 * 把 samples 次采样的平均值存成零点。
 */
uint8_t MOTOR_CalibrateCurrentOffset(uint8_t motor_id,uint16_t samples)
{
    uint32_t sum_a = 0U;
    uint32_t sum_b = 0U;
    uint16_t raw_a;
    uint16_t raw_b;
    uint16_t i;

    if (motor_id < 1U || motor_id > 4U)return 0U;
    if (samples == 0U)return 0U;

    MOTOR_SetCalibrate(1U);
    HAL_Delay(2U);
    MOTOR_SetCalibrate(0U);
    HAL_Delay(1U);

    for (i = 0U; i < samples; i++)
    {
        motor_current_stale = 4U;
        if (!MOTOR_ReadCurrentRaw(motor_id,&raw_a,&raw_b))return 0U;
        sum_a += raw_a;
        sum_b += raw_b;
    }

    motor_current_offset[motor_id - 1U][0] = (float)sum_a / (float)samples;
    motor_current_offset[motor_id - 1U][1] = (float)sum_b / (float)samples;
    return 1U;
}

/************************电流采样部分*************************/

/*
 * 多次扫描取平均，抑制 PWM 三角波引起的采样抖动。
 * 一次扫描约 15us，4 次约 60us，1kHz 控制周期内可以接受。
 */
uint8_t MOTOR_ReadCurrentAvg(uint8_t motor_id,float *current_a,float *current_b,
                             uint8_t samples)
{
    uint8_t i;
    float sum_a = 0.0f;
    float sum_b = 0.0f;
    float current_a_value;
    float current_b_value;

    if (current_a == 0 || current_b == 0)return 0U;
    if (samples == 0U)return 0U;

    for (i = 0U; i < samples; i++)
    {
        motor_current_stale = 4U;
        if (!MOTOR_ReadCurrent(motor_id,&current_a_value,&current_b_value))return 0U;
        sum_a += current_a_value;
        sum_b += current_b_value;
    }

    *current_a = sum_a / (float)samples;
    *current_b = sum_b / (float)samples;
    return 1U;
}
