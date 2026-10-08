#include "motor.h"

/*
============================================================
TIM 句柄
============================================================
*/

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;


/*
============================================================
固定测试参数
============================================================
*/

/*
 * PWM 调制深度。
 *
 * 目前先固定为 15%。
 */
#define MOTOR_DUTY_PERCENT       15U

/*
 * 六步换相间隔。
 *
 * 数值越小，换相越快。
 * 如果电机卡顿，可以改成 30。
 */
#define MOTOR_STEP_TIME_MS       20U


/*
============================================================
内部电机结构体
============================================================
*/

typedef struct
{
    TIM_HandleTypeDef *timer;

    GPIO_TypeDef *enable_port;
    uint16_t enable_pin;

    uint8_t step;
    uint8_t running;
    uint32_t last_step_tick;

} motor_info_t;


/*
============================================================
四个电机配置
============================================================
*/

static motor_info_t motor_info[5] =
{
    {0},

    /*
     * M1
     *
     * TIM2_CH1 -> PA15
     * TIM2_CH2 -> PB3
     * TIM2_CH3 -> PB10
     * ENABLE   -> PE10
     */
    {
        &htim2,
        GPIOE,
        GPIO_PIN_10,
        0U,
        0U,
        0U
    },

    /*
     * M2
     *
     * TIM4_CH1 -> PD12
     * TIM4_CH2 -> PD13
     * TIM4_CH3 -> PD14
     * ENABLE   -> PD8
     */
    {
        &htim4,
        GPIOD,
        GPIO_PIN_8,
        0U,
        0U,
        0U
    },

    /*
     * M3
     *
     * TIM5_CH1 -> PA0
     * TIM5_CH2 -> PA1
     * TIM5_CH3 -> PA2
     * ENABLE   -> PE7
     */
    {
        &htim5,
        GPIOE,
        GPIO_PIN_7,
        0U,
        0U,
        0U
    },

    /*
     * M4
     *
     * TIM3_CH1 -> PA6
     * TIM3_CH2 -> PA7
     * TIM3_CH3 -> PC8
     * ENABLE   -> PE8
     */
    {
        &htim3,
        GPIOE,
        GPIO_PIN_8,
        0U,
        0U,
        0U
    }
};


/*
============================================================
内部函数：判断电机编号
============================================================
*/

static uint8_t motor_id_valid(uint8_t motor_id)
{
    if ((motor_id < MOTOR_1) ||
        (motor_id > MOTOR_4))
    {
        return 0U;
    }

    return 1U;
}


/*
============================================================
内部函数：设置三相 PWM
============================================================
*/

static void motor_set_pwm(uint8_t motor_id,
                          uint32_t pwm_a,
                          uint32_t pwm_b,
                          uint32_t pwm_c)
{
    motor_info_t *motor;
    uint32_t period;

    motor = &motor_info[motor_id];

    period =
        __HAL_TIM_GET_AUTORELOAD(motor->timer);

    if (pwm_a > period)
    {
        pwm_a = period;
    }

    if (pwm_b > period)
    {
        pwm_b = period;
    }

    if (pwm_c > period)
    {
        pwm_c = period;
    }

    __HAL_TIM_SET_COMPARE(motor->timer,
                          TIM_CHANNEL_1,
                          pwm_a);

    __HAL_TIM_SET_COMPARE(motor->timer,
                          TIM_CHANNEL_2,
                          pwm_b);

    __HAL_TIM_SET_COMPARE(motor->timer,
                          TIM_CHANNEL_3,
                          pwm_c);
}


/*
============================================================
内部函数：输出当前六步状态
============================================================
*/

static void motor_output_step(uint8_t motor_id,
                              uint8_t step)
{
    motor_info_t *motor;
    uint32_t period;
    uint32_t middle;
    uint32_t amplitude;
    uint32_t high;
    uint32_t low;

    motor = &motor_info[motor_id];

    period =
        __HAL_TIM_GET_AUTORELOAD(motor->timer);

    middle = period / 2U;

    /*
     * 计算调制幅度。
     */
    amplitude =
        (period * MOTOR_DUTY_PERCENT) / 100U / 2U;

    high = middle + amplitude;
    low  = middle - amplitude;

    /*
     * 六步换相顺序。
     *
     * A高 B低 C中
     * A高 B中 C低
     * A中 B高 C低
     * A低 B高 C中
     * A低 B中 C高
     * A中 B低 C高
     */
    switch (step % 6U)
    {
        case 0U:
        {
            motor_set_pwm(motor_id,
                          high,
                          low,
                          middle);
        }
        break;

        case 1U:
        {
            motor_set_pwm(motor_id,
                          high,
                          middle,
                          low);
        }
        break;

        case 2U:
        {
            motor_set_pwm(motor_id,
                          middle,
                          high,
                          low);
        }
        break;

        case 3U:
        {
            motor_set_pwm(motor_id,
                          low,
                          high,
                          middle);
        }
        break;

        case 4U:
        {
            motor_set_pwm(motor_id,
                          low,
                          middle,
                          high);
        }
        break;

        case 5U:
        {
            motor_set_pwm(motor_id,
                          middle,
                          low,
                          high);
        }
        break;

        default:
        {
            motor_set_pwm(motor_id,
                          middle,
                          middle,
                          middle);
        }
        break;
    }
}


/*
============================================================
初始化四个电机
============================================================
*/

void motor_init(void)
{
    uint8_t i;
    uint32_t period;
    uint32_t middle;

    /*
     * MOTOR_CAL = 高电平：
     * 四个驱动器进入正常运行状态。
     */
    HAL_GPIO_WritePin(MOTOR_CAL_PORT,
                      MOTOR_CAL_PIN,
                      MOTOR_CAL_NORMAL_LEVEL);

    /*
     * 依次初始化四个电机。
     */
    for (i = MOTOR_1; i <= MOTOR_4; i++)
    {
        /*
         * 关闭电机使能。
         */
        HAL_GPIO_WritePin(motor_info[i].enable_port,
                          motor_info[i].enable_pin,
                          GPIO_PIN_RESET);

        /*
         * 启动三路 PWM。
         */
        HAL_TIM_PWM_Start(motor_info[i].timer,
                          TIM_CHANNEL_1);

        HAL_TIM_PWM_Start(motor_info[i].timer,
                          TIM_CHANNEL_2);

        HAL_TIM_PWM_Start(motor_info[i].timer,
                          TIM_CHANNEL_3);

        /*
         * 初始三相都输出中点。
         */
        period =
            __HAL_TIM_GET_AUTORELOAD(motor_info[i].timer);

        middle = period / 2U;

        motor_set_pwm(i,
                      middle,
                      middle,
                      middle);

        motor_info[i].step = 0U;
        motor_info[i].running = 0U;
        motor_info[i].last_step_tick = HAL_GetTick();
    }
}


/*
============================================================
运行指定电机
============================================================
*/

void motor_run(uint8_t motor_id)
{
    motor_info_t *motor;
    uint32_t now;

    if (motor_id_valid(motor_id) == 0U)
    {
        return;
    }

    motor = &motor_info[motor_id];

    /*
     * 第一次调用时使能该电机。
     */
    if (motor->running == 0U)
    {
        HAL_GPIO_WritePin(motor->enable_port,
                          motor->enable_pin,
                          GPIO_PIN_SET);

        motor->step = 0U;
        motor->last_step_tick = HAL_GetTick();
        motor->running = 1U;

        motor_output_step(motor_id,
                          motor->step);

        return;
    }

    now = HAL_GetTick();

    /*
     * 尚未到换相时间。
     */
    if ((now - motor->last_step_tick) <
        MOTOR_STEP_TIME_MS)
    {
        return;
    }

    motor->last_step_tick = now;

    /*
     * 输出当前换相状态。
     */
    motor_output_step(motor_id,
                      motor->step);

    /*
     * 进入下一步。
     */
    motor->step++;

    if (motor->step >= 6U)
    {
        motor->step = 0U;
    }
}

