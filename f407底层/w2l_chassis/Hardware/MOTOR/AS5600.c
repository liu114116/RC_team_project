#include "AS5600.h"

/*
 * 非阻塞读取：主循环里不停调 Encoder_StartReadDMA()（忙时返回 0，跳过），
 * DMA 传完之后 HAL_I2C_MemRxCpltCallback 把结果存起来，
 * 再调 Encoder_ReadRaw() / Encoder_ReadAngle() 取最近一次的数据（不阻塞）。
 *
 * 同一条总线上一次只允许一个传输，所以全局只有一个 in-flight 标志；
 * 四个电机的最近一次结果分别缓存，取过之后 enc_new 清零。
 */

static uint8_t dma_buf[2];

static volatile uint8_t dma_busy = 0U;      /* 1 = 有一笔传输还没完 */
static volatile uint8_t dma_id   = 0U;      /* 正在读哪个电机 */

static uint16_t enc_raw[4] = {0U, 0U, 0U, 0U};
static volatile uint8_t enc_new[4] = {0U, 0U, 0U, 0U};

/* 启动一次读取；1 = 已启动，0 = 上一次还没完 / 参数无效 */
uint8_t Encoder_StartReadDMA(uint8_t id)
{
    I2C_HandleTypeDef *hi2c;
    GPIO_TypeDef *sel_port;
    uint16_t sel_pin;
    GPIO_PinState sel_level;

    if (dma_busy) return 0U;

    switch (id)
    {
        case 1:
        case 2:
            hi2c = &hi2c1;
            sel_port = M12_IIC_SEL_GPIO_Port;
            sel_pin  = M12_IIC_SEL_Pin;
            sel_level = (id == 1U) ? GPIO_PIN_SET : GPIO_PIN_RESET;
            break;

        case 3:
        case 4:
            hi2c = &hi2c3;
            sel_port = M34_IIC_SEL_GPIO_Port;
            sel_pin  = M34_IIC_SEL_Pin;
            sel_level = (id == 3U) ? GPIO_PIN_SET : GPIO_PIN_RESET;
            break;

        default:
            return 0U;
    }

    /* 选通通道，传输期间保持不变 */
    HAL_GPIO_WritePin(sel_port, sel_pin, sel_level);

    dma_id   = id;
    dma_busy = 1U;

    if (HAL_I2C_Mem_Read_DMA(hi2c, AS5600_Address << 1, RAW_Angle_Hi,
                             I2C_MEMADD_SIZE_8BIT, dma_buf, 2U) != HAL_OK)
    {
        dma_busy = 0U;
        return 0U;
    }

    return 1U;
}

/*
 * 返回 1：读取成功，*raw 有效，范围 0~4095。
 * 返回 0：还没有新数据，或参数无效，输出变量不修改。
 */
uint8_t Encoder_ReadRaw(uint8_t id, uint16_t *raw)
{
    if (raw == 0 || id < 1U || id > 4U) return 0U;
    if (enc_new[id - 1U] == 0U) return 0U;

    *raw = enc_raw[id - 1U];
    enc_new[id - 1U] = 0U;

    return 1U;
}

/*
 * 返回 1：读取成功，*angle 有效，单位 rad，范围 [0, 2π)。
 * 返回 0：还没有新数据，或参数无效，输出变量不修改。
 */
uint8_t Encoder_ReadAngle(uint8_t id, float *angle)
{
    uint16_t raw;

    if (angle == 0) return 0U;
    if (!Encoder_ReadRaw(id, &raw)) return 0U;

    *angle = (float)raw * (6.28318530718f / 4096.0f);

    return 1U;
}

/* DMA 传输完成：存结果、松开总线 */
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    (void)hi2c;

    if (dma_id >= 1U && dma_id <= 4U)
    {
        enc_raw[dma_id - 1U] = (((uint16_t)dma_buf[0] << 8) | dma_buf[1]) & 0x0FFFU;
        enc_new[dma_id - 1U] = 1U;
    }

    dma_busy = 0U;
}

/* 传输出错：只放开忙标志，下一次重试 */
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
    (void)hi2c;
    dma_busy = 0U;
}
