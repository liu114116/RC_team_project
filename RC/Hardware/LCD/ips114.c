#include "ips114.h"
#include <stddef.h>

/*===========================================================
 * LCD GPIO 控制
 *==========================================================*/

#define IPS114_CS_LOW()                                         \
    HAL_GPIO_WritePin(IPS114_CS_PORT,                           \
                      IPS114_CS_PIN,                            \
                      GPIO_PIN_RESET)

#define IPS114_CS_HIGH()                                        \
    HAL_GPIO_WritePin(IPS114_CS_PORT,                           \
                      IPS114_CS_PIN,                            \
                      GPIO_PIN_SET)

#define IPS114_DC_COMMAND()                                     \
    HAL_GPIO_WritePin(IPS114_DC_PORT,                           \
                      IPS114_DC_PIN,                            \
                      GPIO_PIN_RESET)

#define IPS114_DC_DATA()                                        \
    HAL_GPIO_WritePin(IPS114_DC_PORT,                           \
                      IPS114_DC_PIN,                            \
                      GPIO_PIN_SET)

#define IPS114_RST_LOW()                                        \
    HAL_GPIO_WritePin(IPS114_RST_PORT,                          \
                      IPS114_RST_PIN,                           \
                      GPIO_PIN_RESET)

#define IPS114_RST_HIGH()                                       \
    HAL_GPIO_WritePin(IPS114_RST_PORT,                          \
                      IPS114_RST_PIN,                           \
                      GPIO_PIN_SET)


/*===========================================================
 * 5×7 ASCII 字库
 *
 * 字符范围：
 *   0x20 空格 ～ 0x7E ~
 *
 * 每个字符包含 5 列。
 * 每列使用低 7 位表示像素。
 *==========================================================*/

static const uint8_t IPS114_Font5x7[95][5] =
{
    {0x00,0x00,0x00,0x00,0x00}, /* 0x20 Space */
    {0x00,0x00,0x5F,0x00,0x00}, /* 0x21 ! */
    {0x00,0x07,0x00,0x07,0x00}, /* 0x22 " */
    {0x14,0x7F,0x14,0x7F,0x14}, /* 0x23 # */
    {0x24,0x2A,0x7F,0x2A,0x12}, /* 0x24 $ */
    {0x23,0x13,0x08,0x64,0x62}, /* 0x25 % */
    {0x36,0x49,0x55,0x22,0x50}, /* 0x26 & */
    {0x00,0x05,0x03,0x00,0x00}, /* 0x27 ' */
    {0x00,0x1C,0x22,0x41,0x00}, /* 0x28 ( */
    {0x00,0x41,0x22,0x1C,0x00}, /* 0x29 ) */
    {0x14,0x08,0x3E,0x08,0x14}, /* 0x2A * */
    {0x08,0x08,0x3E,0x08,0x08}, /* 0x2B + */
    {0x00,0x50,0x30,0x00,0x00}, /* 0x2C , */
    {0x08,0x08,0x08,0x08,0x08}, /* 0x2D - */
    {0x00,0x60,0x60,0x00,0x00}, /* 0x2E . */
    {0x20,0x10,0x08,0x04,0x02}, /* 0x2F / */

    {0x3E,0x51,0x49,0x45,0x3E}, /* 0x30 0 */
    {0x00,0x42,0x7F,0x40,0x00}, /* 0x31 1 */
    {0x42,0x61,0x51,0x49,0x46}, /* 0x32 2 */
    {0x21,0x41,0x45,0x4B,0x31}, /* 0x33 3 */
    {0x18,0x14,0x12,0x7F,0x10}, /* 0x34 4 */
    {0x27,0x45,0x45,0x45,0x39}, /* 0x35 5 */
    {0x3C,0x4A,0x49,0x49,0x30}, /* 0x36 6 */
    {0x01,0x71,0x09,0x05,0x03}, /* 0x37 7 */
    {0x36,0x49,0x49,0x49,0x36}, /* 0x38 8 */
    {0x06,0x49,0x49,0x29,0x1E}, /* 0x39 9 */

    {0x00,0x36,0x36,0x00,0x00}, /* 0x3A : */
    {0x00,0x56,0x36,0x00,0x00}, /* 0x3B ; */
    {0x08,0x14,0x22,0x41,0x00}, /* 0x3C < */
    {0x14,0x14,0x14,0x14,0x14}, /* 0x3D = */
    {0x00,0x41,0x22,0x14,0x08}, /* 0x3E > */
    {0x02,0x01,0x51,0x09,0x06}, /* 0x3F ? */
    {0x32,0x49,0x79,0x41,0x3E}, /* 0x40 @ */

    {0x7E,0x11,0x11,0x11,0x7E}, /* 0x41 A */
    {0x7F,0x49,0x49,0x49,0x36}, /* 0x42 B */
    {0x3E,0x41,0x41,0x41,0x22}, /* 0x43 C */
    {0x7F,0x41,0x41,0x22,0x1C}, /* 0x44 D */
    {0x7F,0x49,0x49,0x49,0x41}, /* 0x45 E */
    {0x7F,0x09,0x09,0x09,0x01}, /* 0x46 F */
    {0x3E,0x41,0x49,0x49,0x7A}, /* 0x47 G */
    {0x7F,0x08,0x08,0x08,0x7F}, /* 0x48 H */
    {0x00,0x41,0x7F,0x41,0x00}, /* 0x49 I */
    {0x20,0x40,0x41,0x3F,0x01}, /* 0x4A J */
    {0x7F,0x08,0x14,0x22,0x41}, /* 0x4B K */
    {0x7F,0x40,0x40,0x40,0x40}, /* 0x4C L */
    {0x7F,0x02,0x0C,0x02,0x7F}, /* 0x4D M */
    {0x7F,0x04,0x08,0x10,0x7F}, /* 0x4E N */
    {0x3E,0x41,0x41,0x41,0x3E}, /* 0x4F O */
    {0x7F,0x09,0x09,0x09,0x06}, /* 0x50 P */
    {0x3E,0x41,0x51,0x21,0x5E}, /* 0x51 Q */
    {0x7F,0x09,0x19,0x29,0x46}, /* 0x52 R */
    {0x46,0x49,0x49,0x49,0x31}, /* 0x53 S */
    {0x01,0x01,0x7F,0x01,0x01}, /* 0x54 T */
    {0x3F,0x40,0x40,0x40,0x3F}, /* 0x55 U */
    {0x1F,0x20,0x40,0x20,0x1F}, /* 0x56 V */
    {0x3F,0x40,0x38,0x40,0x3F}, /* 0x57 W */
    {0x63,0x14,0x08,0x14,0x63}, /* 0x58 X */
    {0x07,0x08,0x70,0x08,0x07}, /* 0x59 Y */
    {0x61,0x51,0x49,0x45,0x43}, /* 0x5A Z */

    {0x00,0x7F,0x41,0x41,0x00}, /* 0x5B [ */
    {0x02,0x04,0x08,0x10,0x20}, /* 0x5C \ */
    {0x00,0x41,0x41,0x7F,0x00}, /* 0x5D ] */
    {0x04,0x02,0x01,0x02,0x04}, /* 0x5E ^ */
    {0x40,0x40,0x40,0x40,0x40}, /* 0x5F _ */
    {0x00,0x01,0x02,0x04,0x00}, /* 0x60 ` */

    {0x20,0x54,0x54,0x54,0x78}, /* 0x61 a */
    {0x7F,0x48,0x44,0x44,0x38}, /* 0x62 b */
    {0x38,0x44,0x44,0x44,0x20}, /* 0x63 c */
    {0x38,0x44,0x44,0x48,0x7F}, /* 0x64 d */
    {0x38,0x54,0x54,0x54,0x18}, /* 0x65 e */
    {0x08,0x7E,0x09,0x01,0x02}, /* 0x66 f */
    {0x0C,0x52,0x52,0x52,0x3E}, /* 0x67 g */
    {0x7F,0x08,0x04,0x04,0x78}, /* 0x68 h */
    {0x00,0x44,0x7D,0x40,0x00}, /* 0x69 i */
    {0x20,0x40,0x44,0x3D,0x00}, /* 0x6A j */
    {0x7F,0x10,0x28,0x44,0x00}, /* 0x6B k */
    {0x00,0x41,0x7F,0x40,0x00}, /* 0x6C l */
    {0x7C,0x04,0x18,0x04,0x78}, /* 0x6D m */
    {0x7C,0x08,0x04,0x04,0x78}, /* 0x6E n */
    {0x38,0x44,0x44,0x44,0x38}, /* 0x6F o */
    {0x7C,0x14,0x14,0x14,0x08}, /* 0x70 p */
    {0x08,0x14,0x14,0x18,0x7C}, /* 0x71 q */
    {0x7C,0x08,0x04,0x04,0x08}, /* 0x72 r */
    {0x48,0x54,0x54,0x54,0x20}, /* 0x73 s */
    {0x04,0x3F,0x44,0x40,0x20}, /* 0x74 t */
    {0x3C,0x40,0x40,0x20,0x7C}, /* 0x75 u */
    {0x1C,0x20,0x40,0x20,0x1C}, /* 0x76 v */
    {0x3C,0x40,0x30,0x40,0x3C}, /* 0x77 w */
    {0x44,0x28,0x10,0x28,0x44}, /* 0x78 x */
    {0x0C,0x50,0x50,0x50,0x3C}, /* 0x79 y */
    {0x44,0x64,0x54,0x4C,0x44}, /* 0x7A z */

    {0x00,0x08,0x36,0x41,0x00}, /* 0x7B { */
    {0x00,0x00,0x7F,0x00,0x00}, /* 0x7C | */
    {0x00,0x41,0x36,0x08,0x00}, /* 0x7D } */
    {0x08,0x04,0x08,0x10,0x08}  /* 0x7E ~ */
};


/*===========================================================
 * SPI 底层发送
 *==========================================================*/

static void IPS114_WriteCommand(uint8_t command)
{
    IPS114_CS_LOW();
    IPS114_DC_COMMAND();

    HAL_SPI_Transmit(&IPS114_SPI_HANDLE,
                     &command,
                     1U,
                     IPS114_SPI_TIMEOUT);

    IPS114_CS_HIGH();
}


static void IPS114_WriteData(const uint8_t *data,
                             uint16_t length)
{
    if ((data == NULL) || (length == 0U))
    {
        return;
    }

    IPS114_CS_LOW();
    IPS114_DC_DATA();

    HAL_SPI_Transmit(&IPS114_SPI_HANDLE,
                     (uint8_t *)data,
                     length,
                     IPS114_SPI_TIMEOUT);

    IPS114_CS_HIGH();
}


static void IPS114_WriteDataByte(uint8_t data)
{
    IPS114_WriteData(&data, 1U);
}


/*===========================================================
 * 设置显示区域
 *==========================================================*/

static void IPS114_SetRegion(uint16_t x1,
                             uint16_t y1,
                             uint16_t x2,
                             uint16_t y2)
{
    uint8_t data[4];

    if ((x1 >= IPS114_WIDTH) ||
        (y1 >= IPS114_HEIGHT) ||
        (x2 >= IPS114_WIDTH) ||
        (y2 >= IPS114_HEIGHT) ||
        (x1 > x2) ||
        (y1 > y2))
    {
        return;
    }

    x1 += IPS114_X_OFFSET;
    x2 += IPS114_X_OFFSET;

    y1 += IPS114_Y_OFFSET;
    y2 += IPS114_Y_OFFSET;

    /* 设置列地址 */
    IPS114_WriteCommand(0x2AU);

    data[0] = (uint8_t)(x1 >> 8);
    data[1] = (uint8_t)(x1 & 0xFFU);
    data[2] = (uint8_t)(x2 >> 8);
    data[3] = (uint8_t)(x2 & 0xFFU);

    IPS114_WriteData(data, 4U);

    /* 设置行地址 */
    IPS114_WriteCommand(0x2BU);

    data[0] = (uint8_t)(y1 >> 8);
    data[1] = (uint8_t)(y1 & 0xFFU);
    data[2] = (uint8_t)(y2 >> 8);
    data[3] = (uint8_t)(y2 & 0xFFU);

    IPS114_WriteData(data, 4U);

    /* 开始写显存 */
    IPS114_WriteCommand(0x2CU);
}


/*===========================================================
 * 连续填充颜色
 *==========================================================*/

static void IPS114_WriteRepeatColor(uint16_t color,
                                    uint32_t pixel_count)
{
    uint8_t buffer[128];
    uint32_t i;

    for (i = 0U; i < 64U; i++)
    {
        buffer[i * 2U] =
            (uint8_t)(color >> 8);

        buffer[i * 2U + 1U] =
            (uint8_t)(color & 0xFFU);
    }

    IPS114_CS_LOW();
    IPS114_DC_DATA();

    while (pixel_count > 0U)
    {
        uint16_t count;

        if (pixel_count > 64U)
        {
            count = 64U;
        }
        else
        {
            count = (uint16_t)pixel_count;
        }

        HAL_SPI_Transmit(&IPS114_SPI_HANDLE,
                         buffer,
                         (uint16_t)(count * 2U),
                         IPS114_SPI_TIMEOUT);

        pixel_count -= count;
    }

    IPS114_CS_HIGH();
}


/*===========================================================
 * 内部矩形填充
 *
 * 主要用于放大字体。
 *==========================================================*/

static void IPS114_FillRect(uint16_t x,
                            uint16_t y,
                            uint16_t width,
                            uint16_t height,
                            uint16_t color)
{
    if ((width == 0U) ||
        (height == 0U) ||
        (x >= IPS114_WIDTH) ||
        (y >= IPS114_HEIGHT))
    {
        return;
    }

    if (width > (IPS114_WIDTH - x))
    {
        width = IPS114_WIDTH - x;
    }

    if (height > (IPS114_HEIGHT - y))
    {
        height = IPS114_HEIGHT - y;
    }

    IPS114_SetRegion(x,
                     y,
                     (uint16_t)(x + width - 1U),
                     (uint16_t)(y + height - 1U));

    IPS114_WriteRepeatColor(
        color,
        (uint32_t)width * height);
}


/*===========================================================
 * 初始化
 *==========================================================*/

void IPS114_Init(void)
{
    IPS114_CS_HIGH();
    IPS114_DC_DATA();
    IPS114_RST_HIGH();

    HAL_Delay(5U);

    /* 硬件复位 */
    IPS114_RST_LOW();
    HAL_Delay(5U);

    IPS114_RST_HIGH();
    HAL_Delay(120U);

    /* 软件复位 */
    IPS114_WriteCommand(0x01U);
    HAL_Delay(120U);

    /* 退出睡眠 */
    IPS114_WriteCommand(0x11U);
    HAL_Delay(120U);

    /*
     * 固定横屏 240×135。
     * 该参数来自当前已经正常工作的版本。
     */
    IPS114_WriteCommand(0x36U);
    IPS114_WriteDataByte(0xA0U);

    /* RGB565 */
    IPS114_WriteCommand(0x3AU);
    IPS114_WriteDataByte(0x05U);

    /* Porch Setting */
    IPS114_WriteCommand(0xB2U);
    IPS114_WriteDataByte(0x0CU);
    IPS114_WriteDataByte(0x0CU);
    IPS114_WriteDataByte(0x00U);
    IPS114_WriteDataByte(0x33U);
    IPS114_WriteDataByte(0x33U);

    /* Gate Control */
    IPS114_WriteCommand(0xB7U);
    IPS114_WriteDataByte(0x35U);

    /* VCOM Setting */
    IPS114_WriteCommand(0xBBU);
    IPS114_WriteDataByte(0x37U);

    /* LCM Control */
    IPS114_WriteCommand(0xC0U);
    IPS114_WriteDataByte(0x2CU);

    /* VDV/VRH Command Enable */
    IPS114_WriteCommand(0xC2U);
    IPS114_WriteDataByte(0x01U);

    /* VRH Set */
    IPS114_WriteCommand(0xC3U);
    IPS114_WriteDataByte(0x12U);

    /* VDV Set */
    IPS114_WriteCommand(0xC4U);
    IPS114_WriteDataByte(0x20U);

    /* Frame Rate */
    IPS114_WriteCommand(0xC6U);
    IPS114_WriteDataByte(0x0FU);

    /* Power Control */
    IPS114_WriteCommand(0xD0U);
    IPS114_WriteDataByte(0xA4U);
    IPS114_WriteDataByte(0xA1U);

    /* Positive Gamma */
    IPS114_WriteCommand(0xE0U);
    IPS114_WriteDataByte(0xD0U);
    IPS114_WriteDataByte(0x04U);
    IPS114_WriteDataByte(0x0DU);
    IPS114_WriteDataByte(0x11U);
    IPS114_WriteDataByte(0x13U);
    IPS114_WriteDataByte(0x2BU);
    IPS114_WriteDataByte(0x3FU);
    IPS114_WriteDataByte(0x54U);
    IPS114_WriteDataByte(0x4CU);
    IPS114_WriteDataByte(0x18U);
    IPS114_WriteDataByte(0x0DU);
    IPS114_WriteDataByte(0x0BU);
    IPS114_WriteDataByte(0x1FU);
    IPS114_WriteDataByte(0x23U);

    /* Negative Gamma */
    IPS114_WriteCommand(0xE1U);
    IPS114_WriteDataByte(0xD0U);
    IPS114_WriteDataByte(0x04U);
    IPS114_WriteDataByte(0x0CU);
    IPS114_WriteDataByte(0x11U);
    IPS114_WriteDataByte(0x13U);
    IPS114_WriteDataByte(0x2CU);
    IPS114_WriteDataByte(0x3FU);
    IPS114_WriteDataByte(0x44U);
    IPS114_WriteDataByte(0x51U);
    IPS114_WriteDataByte(0x2FU);
    IPS114_WriteDataByte(0x1FU);
    IPS114_WriteDataByte(0x1FU);
    IPS114_WriteDataByte(0x20U);
    IPS114_WriteDataByte(0x23U);

    /* 颜色反转 */
    IPS114_WriteCommand(0x21U);

    /* 开启显示 */
    IPS114_WriteCommand(0x29U);
    HAL_Delay(20U);

    /* 默认清成黑色 */
    IPS114_Clear(RGB565_BLACK);
}


/*===========================================================
 * 整屏清屏
 *==========================================================*/

void IPS114_Clear(uint16_t color)
{
    IPS114_SetRegion(0U,
                     0U,
                     IPS114_WIDTH - 1U,
                     IPS114_HEIGHT - 1U);

    IPS114_WriteRepeatColor(
        color,
        (uint32_t)IPS114_WIDTH *
        (uint32_t)IPS114_HEIGHT);
}


/*===========================================================
 * 画点
 *==========================================================*/

void IPS114_DrawPoint(uint16_t x,
                      uint16_t y,
                      uint16_t color)
{
    uint8_t data[2];

    if ((x >= IPS114_WIDTH) ||
        (y >= IPS114_HEIGHT))
    {
        return;
    }

    IPS114_SetRegion(x, y, x, y);

    data[0] = (uint8_t)(color >> 8);
    data[1] = (uint8_t)(color & 0xFFU);

    IPS114_WriteData(data, 2U);
}


/*===========================================================
 * 画线
 *
 * Bresenham 直线算法。
 *==========================================================*/

void IPS114_DrawLine(uint16_t x_start,
                     uint16_t y_start,
                     uint16_t x_end,
                     uint16_t y_end,
                     uint16_t color)
{
    int32_t x0 = x_start;
    int32_t y0 = y_start;
    int32_t x1 = x_end;
    int32_t y1 = y_end;

    int32_t dx;
    int32_t dy;
    int32_t sx;
    int32_t sy;
    int32_t error;
    int32_t error2;

    dx = (x1 >= x0) ? (x1 - x0) : (x0 - x1);
    dy = (y1 >= y0) ? (y1 - y0) : (y0 - y1);

    sx = (x0 < x1) ? 1 : -1;
    sy = (y0 < y1) ? 1 : -1;

    error = dx - dy;

    while (1)
    {
        if ((x0 >= 0) &&
            (x0 < (int32_t)IPS114_WIDTH) &&
            (y0 >= 0) &&
            (y0 < (int32_t)IPS114_HEIGHT))
        {
            IPS114_DrawPoint((uint16_t)x0,
                             (uint16_t)y0,
                             color);
        }

        if ((x0 == x1) && (y0 == y1))
        {
            break;
        }

        error2 = error * 2;

        if (error2 > -dy)
        {
            error -= dy;
            x0 += sx;
        }

        if (error2 < dx)
        {
            error += dx;
            y0 += sy;
        }
    }
}


/*===========================================================
 * 显示一个字符
 *==========================================================*/

void IPS114_ShowChar(uint16_t x,
                     uint16_t y,
                     char character,
                     uint16_t foreground_color,
                     uint16_t background_color,
                     uint8_t size)
{
    uint8_t column;
    uint8_t row;
    uint8_t font_data;
    uint16_t color;

    if (size == 0U)
    {
        size = 1U;
    }

    /*
     * 字符超出 ASCII 字库范围时显示问号。
     */
    if (((uint8_t)character < 0x20U) ||
        ((uint8_t)character > 0x7EU))
    {
        character = '?';
    }

    /*
     * 6×8 字符区域：
     *
     * 前 5 列是字形；
     * 第 6 列是字符间隔；
     * 第 8 行是行间隔。
     */
    for (column = 0U; column < 6U; column++)
    {
        if (column < 5U)
        {
            font_data =
                IPS114_Font5x7[
                    (uint8_t)character - 0x20U
                ][column];
        }
        else
        {
            font_data = 0U;
        }

        for (row = 0U; row < 8U; row++)
        {
            if ((font_data & (1U << row)) != 0U)
            {
                color = foreground_color;
            }
            else
            {
                color = background_color;
            }

            IPS114_FillRect(
                (uint16_t)(x + column * size),
                (uint16_t)(y + row * size),
                size,
                size,
                color);
        }
    }
}


/*===========================================================
 * 显示字符串
 *==========================================================*/

void IPS114_ShowString(uint16_t x,
                       uint16_t y,
                       const char *string,
                       uint16_t foreground_color,
                       uint16_t background_color,
                       uint8_t size)
{
    uint16_t current_x;
    uint16_t current_y;
    uint16_t character_width;
    uint16_t character_height;

    if (string == NULL)
    {
        return;
    }

    if (size == 0U)
    {
        size = 1U;
    }

    current_x = x;
    current_y = y;

    character_width = (uint16_t)(6U * size);
    character_height = (uint16_t)(8U * size);

    while (*string != '\0')
    {
        /*
         * 换行。
         */
        if (*string == '\n')
        {
            current_x = x;
            current_y =
                (uint16_t)(current_y +
                           character_height);

            string++;
            continue;
        }

        /*
         * 回到本行起点。
         */
        if (*string == '\r')
        {
            current_x = x;
            string++;
            continue;
        }

        /*
         * 超出右边界时自动换行。
         */
        if ((current_x + character_width) >
            IPS114_WIDTH)
        {
            current_x = x;
            current_y =
                (uint16_t)(current_y +
                           character_height);
        }

        /*
         * 超出下边界时停止显示。
         */
        if ((current_y + character_height) >
            IPS114_HEIGHT)
        {
            break;
        }

        IPS114_ShowChar(current_x,
                        current_y,
                        *string,
                        foreground_color,
                        background_color,
                        size);

        current_x =
            (uint16_t)(current_x +
                       character_width);

        string++;
    }
}
