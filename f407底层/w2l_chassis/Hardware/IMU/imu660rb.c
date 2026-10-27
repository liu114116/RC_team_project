#include "imu660rb.h"
#include "lsm6dsr_reg.h"

/* ===== 可调参数 ===== */
#define BOOT_TIME         (10)
#define OFFSET_CAL_TIME   (50)

/* ODR 系数：sampleRate = 6667 / ODR_COEFF_xxx */
#define ODR_COEFF_52Hz    (128)
#define ODR_COEFF_104Hz   (64)
#define ODR_COEFF_208Hz   (32)
#define ODR_COEFF_416Hz   (16)
#define ODR_COEFF_833Hz   (8)

/* ===== 当前使用的 ODR，改这三行就能切换速率 ===== */
#define IMU_XL_ODR        LSM6DSR_XL_ODR_104Hz
#define IMU_GY_ODR        LSM6DSR_GY_ODR_104Hz
#define IMU_ODR_COEFF     ODR_COEFF_104Hz

static stmdev_ctx_t dev_ctx;

float acceleration_mg[3];
float angular_rate_mdps[3];

static int16_t data_raw_acceleration[3];
static int16_t data_raw_angular_rate[3];
static uint8_t whoamI, rst;
static float   samplePeriod, sampleRate;

static FusionMatrix gyroscopeMisalignment = {
    1.0f, 0.0f, 0.0f,
    0.0f, 1.0f, 0.0f,
    0.0f, 0.0f, 1.0f
};
static FusionVector gyroscopeSensitivity = {1.0f, 1.0f, 1.0f};
static FusionVector gyroscopeOffset      = {0.0f, 0.0f, 0.0f};

FusionAhrs ahrs;
FusionEuler euler;
static FusionOffset offset;

/* ===== 底层 SPI 读写，直接供 stmdev 回调使用 ===== */
static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len)
{
    uint8_t tx, rx;
    (void)handle;

    HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_RESET);

    tx = reg & 0x7F;
    HAL_SPI_TransmitReceive(&IMU_SPI_HANDLE, &tx, &rx, 1, HAL_MAX_DELAY);

    while (len--)
    {
        tx = *bufp++;
        HAL_SPI_TransmitReceive(&IMU_SPI_HANDLE, &tx, &rx, 1, HAL_MAX_DELAY);
    }

    HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);
    return 0;
}

static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len)
{
    uint8_t tx, rx;
    (void)handle;

    HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_RESET);

    tx = reg | 0x80;
    HAL_SPI_TransmitReceive(&IMU_SPI_HANDLE, &tx, &rx, 1, HAL_MAX_DELAY);

    while (len--)
    {
        tx = 0x00;
        HAL_SPI_TransmitReceive(&IMU_SPI_HANDLE, &tx, &rx, 1, HAL_MAX_DELAY);
        *bufp++ = rx;
    }

    HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);
    return 0;
}

/* ============================================================
 *                        初始化
 * ============================================================ */
void IMU660RB_Init(void)
{
    uint8_t offset_cnt;
    int8_t  freq_fine;

    dev_ctx.write_reg = platform_write;
    dev_ctx.read_reg  = platform_read;
    dev_ctx.mdelay    = HAL_Delay;

    HAL_Delay(BOOT_TIME);

    lsm6dsr_device_id_get(&dev_ctx, &whoamI);
    if (whoamI != LSM6DSR_ID)
        return;   /* ID 不对，SPI 通信有问题，在此断点排查 */

    lsm6dsr_reset_set(&dev_ctx, PROPERTY_ENABLE);
    do {
        lsm6dsr_reset_get(&dev_ctx, &rst);
    } while (rst);

    lsm6dsr_i3c_disable_set(&dev_ctx, LSM6DSR_I3C_DISABLE);
    lsm6dsr_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);

    /* ===== ODR：208Hz ===== */
    lsm6dsr_xl_data_rate_set(&dev_ctx, IMU_XL_ODR);
    lsm6dsr_gy_data_rate_set(&dev_ctx, IMU_GY_ODR);

    lsm6dsr_xl_full_scale_set(&dev_ctx, LSM6DSR_2g);
    lsm6dsr_gy_full_scale_set(&dev_ctx, LSM6DSR_2000dps);

    lsm6dsr_gy_filter_lp1_set(&dev_ctx, 1);

    /* 计算实际采样周期，供 Fusion 积分使用 */
    lsm6dsr_odr_cal_reg_get(&dev_ctx, &freq_fine);
		sampleRate = 100.0f;
		samplePeriod = 0.01f;
//    sampleRate   = (6667 + ((0.0015f * freq_fine) * 6667)) / IMU_ODR_COEFF;
//    samplePeriod = 1.0f / sampleRate;

    FusionAhrsInitialise(&ahrs);
    FusionOffsetInitialise(&offset, sampleRate);

    HAL_Delay(200);

    /* 陀螺仪零偏校准，期间必须静止 */
    offset_cnt = OFFSET_CAL_TIME;
    while (offset_cnt)
    {
        uint8_t reg;
        lsm6dsr_gy_flag_data_ready_get(&dev_ctx, &reg);
        if (reg)
        {
            offset_cnt--;
            lsm6dsr_angular_rate_raw_get(&dev_ctx, data_raw_angular_rate);
            angular_rate_mdps[0] = lsm6dsr_from_fs2000dps_to_mdps(data_raw_angular_rate[0]);
            angular_rate_mdps[1] = lsm6dsr_from_fs2000dps_to_mdps(data_raw_angular_rate[1]);
            angular_rate_mdps[2] = lsm6dsr_from_fs2000dps_to_mdps(data_raw_angular_rate[2]);
            gyroscopeOffset.array[0] += angular_rate_mdps[0] / 1000.0f;
            gyroscopeOffset.array[1] += angular_rate_mdps[1] / 1000.0f;
            gyroscopeOffset.array[2] += angular_rate_mdps[2] / 1000.0f;
        }
    }
    gyroscopeOffset.array[0] /= OFFSET_CAL_TIME;
    gyroscopeOffset.array[1] /= OFFSET_CAL_TIME;
    gyroscopeOffset.array[2] /= OFFSET_CAL_TIME;
}

/* ============================================================
 *                   读取一次并做姿态解算
 * ============================================================ */
void Read_IMU660RB(void)
{
    lsm6dsr_acceleration_raw_get(&dev_ctx, data_raw_acceleration);
    acceleration_mg[0] = lsm6dsr_from_fs2g_to_mg(data_raw_acceleration[0]);
    acceleration_mg[1] = lsm6dsr_from_fs2g_to_mg(data_raw_acceleration[1]);
    acceleration_mg[2] = lsm6dsr_from_fs2g_to_mg(data_raw_acceleration[2]);

    lsm6dsr_angular_rate_raw_get(&dev_ctx, data_raw_angular_rate);
    angular_rate_mdps[0] = lsm6dsr_from_fs2000dps_to_mdps(data_raw_angular_rate[0]);
    angular_rate_mdps[1] = lsm6dsr_from_fs2000dps_to_mdps(data_raw_angular_rate[1]);
    angular_rate_mdps[2] = lsm6dsr_from_fs2000dps_to_mdps(data_raw_angular_rate[2]);

    FusionVector accelerometer = {
        acceleration_mg[0] / 1000.0f,
        acceleration_mg[1] / 1000.0f,
        acceleration_mg[2] / 1000.0f
    };
    FusionVector gyroscope = {
        angular_rate_mdps[0] / 1000.0f,
        angular_rate_mdps[1] / 1000.0f,
        angular_rate_mdps[2] / 1000.0f
    };

    gyroscope = FusionCalibrationInertial(gyroscope,
                                          gyroscopeMisalignment,
                                          gyroscopeSensitivity,
                                          gyroscopeOffset);
    gyroscope = FusionOffsetUpdate(&offset, gyroscope);

    FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer, samplePeriod);
    euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));
}

/* ============================================================
 *                        VOFA+ 打印
 * ============================================================ */
void IMU660RB_Print(void)
{
    usb_printf("roll:%.2f,pitch:%.2f,yaw:%.2f,"
               "ax:%.4f,ay:%.4f,az:%.4f,"
               "gx:%.2f,gy:%.2f,gz:%.2f\n",
               euler.angle.roll,
               euler.angle.pitch,
               euler.angle.yaw,
               acceleration_mg[0] / 1000.0f,
               acceleration_mg[1] / 1000.0f,
               acceleration_mg[2] / 1000.0f,
               angular_rate_mdps[0] / 1000.0f,
               angular_rate_mdps[1] / 1000.0f,
               angular_rate_mdps[2] / 1000.0f);
}
//void IMU660RB_Print(void)
//{
//    usb_printf("roll:%.2f,pitch:%.2f,yaw:%.2f\n",
//               euler.angle.roll,
//               euler.angle.pitch,
//               euler.angle.yaw);
//}




