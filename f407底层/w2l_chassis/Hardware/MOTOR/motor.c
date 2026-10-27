#include "adc.h"
#include "motor.h"

/* 电流采样里的零点标定，MOTOR_Init 要调用，先声明 */
static void MOTOR_AdcTriggerInit(void);
static void MOTOR_CurrentStart(void);
static void MOTOR_CalibrateCurrentOffset(uint8_t motor_id);

/*****************************简化处理函数*********************************/

/*
 * 每个电机的底层硬件参数：PWM 定时器/通道、使能引脚、相电流在 ADC 规则组里的位置。
 * 下表在编译期完成初始化，运行时只取地址。
 */
typedef struct
{
    TIM_HandleTypeDef *timer;
    uint32_t channel_a;
    uint32_t channel_b;
    uint32_t channel_c;
    GPIO_TypeDef *enable_port;
    uint16_t enable_pin;
    uint8_t  adc_rank_a;      /* A 相在 ADC 规则组里的位置（0~7） */
    uint8_t  adc_rank_b;      /* B 相在 ADC 规则组里的位置（0~7） */
} MOTOR_Hardware_t;

/* 下标 = 电机号 - 1，只在本文件内部使用 */
static const MOTOR_Hardware_t MOTOR_HW[4] =
{
    /* M1 -> TIM2 */
    {
        MOTOR1_PWM_TIMER, MOTOR1_PWM_A_CHANNEL, MOTOR1_PWM_B_CHANNEL, MOTOR1_PWM_C_CHANNEL,
        M1_ENABLE_GPIO_Port, M1_ENABLE_Pin,
        0U, 1U    /* A=rank1, B=rank2 */
    },
    /* M2 -> TIM4 */
    {
        MOTOR2_PWM_TIMER, MOTOR2_PWM_A_CHANNEL, MOTOR2_PWM_B_CHANNEL, MOTOR2_PWM_C_CHANNEL,
        M2_ENABLE_GPIO_Port, M2_ENABLE_Pin,
        2U, 3U    /* A=rank3, B=rank4 */
    },
    /* M3 -> TIM5 */
    {
        MOTOR3_PWM_TIMER, MOTOR3_PWM_A_CHANNEL, MOTOR3_PWM_B_CHANNEL, MOTOR3_PWM_C_CHANNEL,
        M3_ENABLE_GPIO_Port, M3_ENABLE_Pin,
        4U, 5U    /* A=rank5, B=rank6 */
    },
    /* M4 -> TIM3 */
    {
        MOTOR4_PWM_TIMER, MOTOR4_PWM_A_CHANNEL, MOTOR4_PWM_B_CHANNEL, MOTOR4_PWM_C_CHANNEL,
        M4_ENABLE_GPIO_Port, M4_ENABLE_Pin,
        6U, 7U    /* A=rank7, B=rank8 */
    }
};

/* 取硬件参数表指针，编号非法返回 0 */
static inline const MOTOR_Hardware_t *MOTOR_GetHardware(uint8_t motor_id)
{
    if (motor_id < 1U || motor_id > 4U)
    {
        return 0;
    }
    return &MOTOR_HW[motor_id - 1U];
}

/* 0.0f ~ 1.0f 的占空比换算成 CCR 数值 */
static uint32_t MOTOR_DutyToCompare(TIM_HandleTypeDef *timer,float duty)
{
    if (duty < 0.0f) duty = 0.0f;
    if (duty > 1.0f) duty = 1.0f;

    return (uint32_t)(duty * (float)__HAL_TIM_GET_AUTORELOAD(timer));
}

/* calibrate = 0：正常运行；calibrate = 1：校准模式 */
void MOTOR_SetCalibrate(uint8_t calibrate)
{
    HAL_GPIO_WritePin(MOTOR_CAL_GPIO_Port,MOTOR_CAL_Pin,
                      calibrate ? MOTOR_CAL_CALIBRATE_LEVEL : MOTOR_CAL_NORMAL_LEVEL);
}

/******************************电机控制函数*******************************/

/* motor_id：0 = 四个都初始化；1~4 = 只初始化该电机（影响相电流零点标定） */
void MOTOR_Init(uint8_t motor_id)
{
    uint8_t id;

    for (id = 1U; id <= 4U; id++)
    {
        MOTOR_Stop(id);
    }
    MOTOR_SetCalibrate(0);

    /* 标定相电流零点 */

    MOTOR_AdcTriggerInit();
    MOTOR_CurrentStart();          /* 启动 ADC 同步采样（TIM3 触发） */
    MOTOR_CalibrateCurrentOffset(motor_id);
}

void MOTOR_Start(uint8_t motor_id)
{
    const MOTOR_Hardware_t *hw = MOTOR_GetHardware(motor_id);
    if (hw == 0) return;

    HAL_GPIO_WritePin(hw->enable_port, hw->enable_pin, MOTOR_DISABLE_LEVEL);

    HAL_TIM_PWM_Start(hw->timer, hw->channel_a);
    HAL_TIM_PWM_Start(hw->timer, hw->channel_b);
    HAL_TIM_PWM_Start(hw->timer, hw->channel_c);
    MOTOR_SetPWM(motor_id, 0.5f, 0.5f, 0.5f);

    HAL_GPIO_WritePin(hw->enable_port, hw->enable_pin, MOTOR_ENABLE_LEVEL);
}

void MOTOR_Stop(uint8_t motor_id)
{
    const MOTOR_Hardware_t *hw = MOTOR_GetHardware(motor_id);
    if (hw == 0) return;

    HAL_GPIO_WritePin(hw->enable_port, hw->enable_pin, MOTOR_DISABLE_LEVEL);

    MOTOR_SetPWM(motor_id, 0.5f, 0.5f, 0.5f);
}

void MOTOR_SetPWM(uint8_t motor_id,float duty_a,float duty_b,float duty_c)
{
    const MOTOR_Hardware_t *hw = MOTOR_GetHardware(motor_id);
    if (hw == 0) return;

    __HAL_TIM_SET_COMPARE(hw->timer, hw->channel_a,
        MOTOR_DutyToCompare(hw->timer, duty_a));
    __HAL_TIM_SET_COMPARE(hw->timer, hw->channel_b,
        MOTOR_DutyToCompare(hw->timer, duty_b));
    __HAL_TIM_SET_COMPARE(hw->timer, hw->channel_c,
        MOTOR_DutyToCompare(hw->timer, duty_c));
}

/************************ 电流采样 *************************/

/*
 * ADC1 规则组（见 adc.c）：rank1..rank8 = M1A, M1B, M2A, M2B, M3A, M3B, M4A, M4B。
 * 由 TIM3 更新事件触发，DMA 循环搬进 adc_buf[]；位置放在 MOTOR_HW 表里，
 * 改 CubeMX 的 rank 顺序时，同步改那 4 行的 adc_rank_a/b。
 */
#define MOTOR_ADC_CHANNELS   8U

/* 零点标定的采样次数（每次间隔一个 PWM 周期） */
#define MOTOR_CAL_SAMPLES    64U

/* DMA 目标缓冲区：rank1..rank8，由 DMA 直接写入，必须 volatile */
static volatile uint16_t adc_buf[MOTOR_ADC_CHANNELS];

/* 每路相电流的零点（ADC 码值），MOTOR_Init 上电标定，[电机号 - 1][相位] */
static float motor_current_offset[4][2] =
{
    { 2048.0f, 2048.0f },   /* M1：A、B */
    { 2048.0f, 2048.0f },   /* M2 */
    { 2048.0f, 2048.0f },   /* M3 */
    { 2048.0f, 2048.0f }    /* M4 */
};

/*
 * 采样触发点：用 TIM3 CH4 比较事件做 TRGO，落在 PWM 周期末尾的零矢量窗口
 * （三相下桥同时导通，采样电阻上才有电流）。CH4 只做内部比较，不输出到引脚。
 */
static void MOTOR_AdcTriggerInit(void)
{
    uint32_t arr = __HAL_TIM_GET_AUTORELOAD(&htim3);

    TIM3->CCR4   = arr - (arr / 8U);                /* 周期末尾前 1/8 */
    TIM3->CCMR2 &= ~((3U << 8) | (7U << 12));       /* CC4S = 输出比较，OC4M 清零 */
    TIM3->CCMR2 |=  (6U << 12);                     /* OC4M = 110：PWM2，CNT=CCR4 时 OC4REF 上升 */
    TIM3->CCER  |= TIM_CCER_CC4E;                   /* 打开 CH4 比较 */
    TIM3->CR2    = (TIM3->CR2 & ~TIM_CR2_MMS) | (7U << TIM_CR2_MMS_Pos);   /* MMS = 111：TRGO = OC4REF */
}

static void MOTOR_CurrentStart(void)
{
    (void)HAL_ADC_Start_DMA(&hadc1, (uint32_t *)(void *)adc_buf, MOTOR_ADC_CHANNELS);
}

/*
 * 相电流零点标定。motor_id：0 = 四个全标；1~4 = 只写该电机的零点。
 *
 * 标定时四个电机都会上使能：ADC 由 TIM3 触发，必须保证它一直在触发。
 * 三相都是 50% 占空比，相电压 0、没有电流，所以对其它电机无影响。
 */
static void MOTOR_CalibrateCurrentOffset(uint8_t motor_id)
{
    const MOTOR_Hardware_t *hw;
    uint8_t  first;
    uint8_t  last;
    uint8_t  id;
    uint16_t k;
    uint32_t sum_a;
    uint32_t sum_b;

    if (motor_id == 0U)
    {
        first = 1U;
        last  = 4U;
    }
    else if (motor_id <= 4U)
    {
        first = motor_id;
        last  = motor_id;
    }
    else
    {
        return;
    }

    for (id = 1U; id <= 4U; id++)
    {
        MOTOR_Start(id);
    }
    HAL_Delay(5U);

    for (id = first; id <= last; id++)
    {
        hw = MOTOR_GetHardware(id);
        sum_a = 0U;
        sum_b = 0U;

        for (k = 0U; k < MOTOR_CAL_SAMPLES; k++)
        {
            HAL_Delay(1U);                      /* 等下一个 PWM 周期的新数据 */
            sum_a += adc_buf[hw->adc_rank_a];
            sum_b += adc_buf[hw->adc_rank_b];
        }

        motor_current_offset[id - 1U][0] = (float)sum_a / (float)MOTOR_CAL_SAMPLES;
        motor_current_offset[id - 1U][1] = (float)sum_b / (float)MOTOR_CAL_SAMPLES;
    }

    for (id = 1U; id <= 4U; id++)
    {
        MOTOR_Stop(id);
    }
}

/* 读某一相的原始 ADC 码值（0~4095） */
uint8_t MOTOR_ReadCurrentRaw(uint8_t motor_id,uint8_t phase,uint16_t *raw)
{
    const MOTOR_Hardware_t *hw = MOTOR_GetHardware(motor_id);

    if (hw == 0 || raw == 0 || phase > MOTOR_PHASE_B)
    {
        return 0U;
    }

    *raw = adc_buf[phase ? hw->adc_rank_b : hw->adc_rank_a];
    return 1U;
}

/* 某一相电流，单位 A：减去标定零点后换算 */
float MOTOR_GetCurrent(uint8_t motor_id,uint8_t phase)
{
    uint16_t raw;
    float offset;

    if (!MOTOR_ReadCurrentRaw(motor_id, phase, &raw))
    {
        return 0.0f;
    }

    offset = phase ? motor_current_offset[motor_id - 1U][1]
                   : motor_current_offset[motor_id - 1U][0];

    return (offset - (float)raw) * MOTOR_CURRENT_SCALE * MOTOR_CURRENT_SIGN;
}

