#include "AS5600.h"

/*
读取时间大概为120~140μs
后期可修改i2c speed查看是否能提高速度，AS5600最高支持1MHZ
*/
uint16_t AS5600_ReadHardware(I2C_HandleTypeDef *hi2c)
{
    uint8_t data[2] = {0};
    static uint16_t last_i2c1 = 0;
    static uint16_t last_i2c3 = 0;
    uint16_t *last_value;
    if (hi2c == &hi2c1) {
        last_value = &last_i2c1;
    } else {
        last_value = &last_i2c3;
    }
    HAL_StatusTypeDef status;
    status = HAL_I2C_Mem_Read(hi2c,AS5600_Address << 1,RAW_Angle_Hi,
					I2C_MEMADD_SIZE_8BIT,data,2,I2C_TIMEOUT
    );
    if (status == HAL_OK) {
        *last_value =(((uint16_t)data[0] << 8) | data[1]) & 0x0FFF;
    }
    return *last_value;
}

/*
目前这个有for (volatile uint32_t i = 0; i < 50; i++) { __NOP(); }
这个是在读取过程中防止切换读取时候避免数据混乱
可先测试，再确定是否保留
*/
uint16_t Encoder_ReadRaw(uint8_t id)
{
    uint16_t raw = 0U;
    switch (id)
    {
        case 1U:
        {
						HAL_GPIO_WritePin(AS5600_12_SEL_PORT,AS5600_12_SEL_PIN,GPIO_PIN_SET);
						for (volatile uint32_t i = 0; i < 50; i++) { __NOP(); }
            raw =AS5600_ReadHardware(&hi2c1);
            return (uint16_t)((raw) &0x0FFFU);
        }
        case 2U:
        {
            HAL_GPIO_WritePin(AS5600_12_SEL_PORT,AS5600_12_SEL_PIN,GPIO_PIN_RESET);
						for (volatile uint32_t i = 0; i < 50; i++) { __NOP(); }
            raw =AS5600_ReadHardware(&hi2c1);
            return (uint16_t)((raw) &0x0FFFU);
        }
        case 3U:
        {
            HAL_GPIO_WritePin(AS5600_34_SEL_PORT,AS5600_34_SEL_PIN,GPIO_PIN_SET);
						for (volatile uint32_t i = 0; i < 50; i++) { __NOP(); }
            raw =AS5600_ReadHardware(&hi2c3);
            return (uint16_t)((raw) &0x0FFFU);
        }
        case 4U:
        {
            HAL_GPIO_WritePin(AS5600_34_SEL_PORT,AS5600_34_SEL_PIN,GPIO_PIN_RESET);
						for (volatile uint32_t i = 0; i < 50; i++) { __NOP(); }
            raw =AS5600_ReadHardware(&hi2c3);
            return (uint16_t)((raw) &0x0FFFU);
        }
        default: return 0U;
    }
}



