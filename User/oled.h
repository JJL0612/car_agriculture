//
// Created by 36138 on 26-7-29.
//

#ifndef CAR_CODE_7_21_OLED_H
#define CAR_CODE_7_21_OLED_H

#include "main.h"
#include "i2c.h"

// ==================== OLED I2C 配置 ====================
// SSD1306 7 位从机地址（模块默认 0x3C；若模块 SA0 引脚拉高则为 0x3D，
// 屏幕无显示时把地址改成 (0x3D << 1) 再试）
#define OLED_I2C_ADDR        (0x3C << 1)
// I2C 控制字节：0x00=命令、0x40=数据（SSD1306 I2C 协议要求每字节前带控制字节）
#define OLED_CTRL_CMD        0x00
#define OLED_CTRL_DATA       0x40

// ==================== OLED 尺寸 ====================
#define OLED_WIDTH   128
#define OLED_HEIGHT  64
#define OLED_PAGES   8

// ==================== 函数声明 ====================

/**
 * @brief  OLED 初始化（复位、配置 SSD1306 寄存器）
 */
void OLED_Init(void);

/**
 * @brief  清屏（全屏填充 0x00）
 */
void OLED_Clear(void);

/**
 * @brief  在指定位置显示单个数字字符（0~9），字体 8x16
 * @param  x   列坐标（0 ~ 127）
 * @param  y   页坐标（0 ~ 7），字符占 2 页
 * @param  ch  要显示的数字（0 ~ 9）
 */
void OLED_ShowChar(uint8_t x, uint8_t y, uint8_t ch);

/**
 * @brief  在指定位置显示两位数（00 ~ 99），字体 8x16
 *         十位和个位并排显示，总宽度 16 像素
 * @param  x   十位数字的起始列坐标（0 ~ 111）
 * @param  y   页坐标（0 ~ 6），两位数共占 2 页
 * @param  num 要显示的数字（0 ~ 99），自动分解为十位和个位
 */
void OLED_ShowNum(uint8_t x, uint8_t y, uint8_t num);

/**
 * @brief  在第一行显示标题 "CAR TIME:"
 *         使用 6x8 小字体，占 page 0（8 像素高）
 *         OLED 初始化后调用一次即可
 */
void OLED_ShowTitle(void);

/**
 * @brief  在指定位置显示字符串（6x8 小字体，占 1 页）
 * @param  x   起始列坐标（0 ~ 121）
 * @param  y   页坐标（0 ~ 7）
 * @param  str 字符串指针（字库仅覆盖 ' ' ~ 'Z'，超范围字符显示为空格）
 */
void OLED_ShowString(uint8_t x, uint8_t y, const char *str);

/**
 * @brief  显示三个 TOFSense 测距传感器的实时距离（单位 cm）
 *         布局（6x8 小字体，每行占 1 页）：
 *           page 0：标题 "DIST(CM)"
 *           page 1~3：S0 / S1 / S2 的距离，固定宽度 xxx.x，
 *                     无有效数据时显示 "--.-"
 * @note   需要包含 tofsense.h；在主循环中周期调用即可
 */
void OLED_ShowDistance(void);

/**
 * @brief  显示遥控器 CH1~CH8 通道原始值（SBUS，中位 992）
 *         布局（6x8 小字体，每行占 1 页，占用 page 4~7，
 *         不影响上方 page 0~3 的测距显示）：
 *           page 4：CH1  CH2
 *           page 5：CH3  CH4
 *           page 6：CH5  CH6
 *           page 7：CH7  CH8
 *         每个通道固定宽度 "CHx:%4ld"（如 "CH1: 992"），
 *         数值位数变化不会残留旧像素
 * @note   数据来自 yaokong.c 的全局 sbus_ch[]；在主循环中周期调用即可
 */
void OLED_ShowChannels(void);

#endif //CAR_CODE_7_21_OLED_H
