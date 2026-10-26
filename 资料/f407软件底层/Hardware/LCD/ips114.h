#ifndef _IPS114_H_
#define _IPS114_H_

#include "main.h"
#include "spi.h"

/*
IPS114 接线
    ______________________________________
        IPS114              STM32F407VET6
    |   SCL / SCK           PB13 SPI2_SCK  |
    |   SDA / MOSI          PB15 SPI2_MOSI |
    |   CS                  PD15           |
    |   DC                  PC6            |
    |   RST                 PC7            |
    |   BLK                 3.3V           |
    |   VCC                 3.3V           |
    |   GND                 GND            |
    ______________________________________

IPS114 不需要 MISO。
PC2 和 PB14 可以配置为其他功能。
*/

/* SPI 设置 */
#define IPS114_SPI_HANDLE          hspi2
#define IPS114_SPI_TIMEOUT         HAL_MAX_DELAY

/* LCD 控制引脚 */
#define IPS114_CS_PORT             GPIOD
#define IPS114_CS_PIN              GPIO_PIN_15

#define IPS114_DC_PORT             GPIOC
#define IPS114_DC_PIN              GPIO_PIN_6

#define IPS114_RST_PORT            GPIOC
#define IPS114_RST_PIN             GPIO_PIN_7

/* 横屏分辨率 */
#define IPS114_WIDTH               240U
#define IPS114_HEIGHT              135U

/* 屏幕显存偏移 */
#define IPS114_X_OFFSET            40U
#define IPS114_Y_OFFSET            52U

/* RGB565 常用颜色 */
#define RGB565_BLACK               0x0000U
#define RGB565_WHITE               0xFFFFU
#define RGB565_RED                 0xF800U
#define RGB565_GREEN               0x07E0U
#define RGB565_BLUE                0x001FU
#define RGB565_YELLOW              0xFFE0U
#define RGB565_CYAN                0x07FFU
#define RGB565_MAGENTA             0xF81FU
#define RGB565_ORANGE              0xFD20U
#define RGB565_GRAY                0x8410U
#define RGB565_DARKGRAY            0x4208U
#define RGB565_LIGHTGRAY           0xC618U

/*
 * 将 8 位 RGB 转换为 RGB565。
 *
 * 示例：
 * uint16_t color = RGB565(255, 128, 0);
 */
#define RGB565(r, g, b)                                           \
    ((uint16_t)((((uint16_t)(r) & 0xF8U) << 8) |                 \
                (((uint16_t)(g) & 0xFCU) << 3) |                 \
                (((uint16_t)(b) & 0xF8U) >> 3)))

/* 初始化 */
void IPS114_Init(void);

/* 整屏填充 */
void IPS114_Clear(uint16_t color);

/* 画点 */
void IPS114_DrawPoint(uint16_t x,uint16_t y,uint16_t color);

/* 画线 */
void IPS114_DrawLine(uint16_t x_start,uint16_t y_start,uint16_t x_end,uint16_t y_end,uint16_t color);

/*
 * 显示一个 ASCII 字符。
 * character：
 *   支持 ASCII 0x20～0x7E。
 * foreground_color：
 *   字体颜色。
 * background_color：
 *   背景颜色。
 * size：
 *   字体整数倍数。
 *   size = 1：6×8
 *   size = 2：12×16
 *   size = 3：18×24
 */
void IPS114_ShowChar(uint16_t x,uint16_t y,char character,
			uint16_t foreground_color,uint16_t background_color,uint8_t size);

/*
 * 显示 ASCII 字符串。
 * 支持：
 *   \n：换行
 *   \r：回到行首
 * 超过屏幕右边界后会自动换行。
 */
void IPS114_ShowString(uint16_t x,uint16_t y,const char *string,
							uint16_t foreground_color,uint16_t background_color,uint8_t size);

#endif

