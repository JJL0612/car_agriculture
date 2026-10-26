//
// Created by 36138 on 26-7-29.
//

#include "oled.h"
#include "tofsense.h"
#include "yaokong.h"
#include <string.h>
#include <stdio.h>

// ==================== SSD1306 指令表 ====================
#define SSD1306_SETCONTRAST         0x81
#define SSD1306_DISPLAYALLON_RESUME 0xA4
#define SSD1306_DISPLAYALLON        0xA5
#define SSD1306_NORMALDISPLAY       0xA6
#define SSD1306_INVERTDISPLAY       0xA7
#define SSD1306_DISPLAYOFF          0xAE
#define SSD1306_DISPLAYON           0xAF
#define SSD1306_SETDISPLAYOFFSET    0xD3
#define SSD1306_SETCOMPINS          0xDA
#define SSD1306_SETVCOMDETECT       0xDB
#define SSD1306_SETDISPLAYCLOCKDIV  0xD5
#define SSD1306_SETPRECHARGE        0xD9
#define SSD1306_SETMULTIPLEX        0xA8
#define SSD1306_SETLOWCOLUMN        0x00
#define SSD1306_SETHIGHCOLUMN       0x10
#define SSD1306_SETSTARTLINE        0x40
#define SSD1306_MEMORYMODE          0x20
#define SSD1306_COLUMNADDR          0x21
#define SSD1306_PAGEADDR            0x22
#define SSD1306_COMSCANINC          0xC0
#define SSD1306_COMSCANDEC          0xC8
#define SSD1306_SEGREMAP            0xA0
#define SSD1306_CHARGEPUMP          0x8D
#define SSD1306_SEG_REMAP_REV       0xA1

// ==================== 数字 8x16 字库（0~9，两页，每字符16字节） ====================
// 8x16 字体——每行 8 像素宽，高度 16 像素（占 2 页）
static const uint8_t F8X16_NUM[10][16] = {
    // 0
    {
        0x00, 0xE0, 0x10, 0x08, 0x08, 0x10, 0xE0, 0x00,  // 上半页
        0x00, 0x0F, 0x10, 0x20, 0x20, 0x10, 0x0F, 0x00   // 下半页
    },
    // 1
    {
        0x00, 0x10, 0x10, 0xF8, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x20, 0x20, 0x3F, 0x20, 0x20, 0x00, 0x00
    },
    // 2
    {
        0x00, 0x70, 0x08, 0x08, 0x08, 0x88, 0x70, 0x00,
        0x00, 0x30, 0x28, 0x24, 0x22, 0x21, 0x30, 0x00
    },
    // 3
    {
        0x00, 0x30, 0x08, 0x88, 0x88, 0x48, 0x30, 0x00,
        0x00, 0x18, 0x20, 0x20, 0x20, 0x11, 0x0E, 0x00
    },
    // 4
    {
        0x00, 0x00, 0xC0, 0x20, 0x10, 0xF8, 0x00, 0x00,
        0x00, 0x07, 0x04, 0x24, 0x24, 0x3F, 0x24, 0x00
    },
    // 5
    {
        0x00, 0xF8, 0x08, 0x88, 0x88, 0x08, 0x08, 0x00,
        0x00, 0x19, 0x21, 0x20, 0x20, 0x11, 0x0E, 0x00
    },
    // 6
    {
        0x00, 0xE0, 0x10, 0x88, 0x88, 0x18, 0x00, 0x00,
        0x00, 0x0F, 0x11, 0x20, 0x20, 0x11, 0x0E, 0x00
    },
    // 7
    {
        0x00, 0x38, 0x08, 0x08, 0xC8, 0x38, 0x08, 0x00,
        0x00, 0x00, 0x00, 0x3F, 0x00, 0x00, 0x00, 0x00
    },
    // 8
    {
        0x00, 0x70, 0x88, 0x08, 0x08, 0x88, 0x70, 0x00,
        0x00, 0x1C, 0x22, 0x21, 0x21, 0x22, 0x1C, 0x00
    },
    // 9
    {
        0x00, 0xE0, 0x10, 0x08, 0x08, 0x10, 0xE0, 0x00,
        0x00, 0x00, 0x31, 0x22, 0x22, 0x11, 0x0F, 0x00
    },
};

// ==================== 内部辅助函数 ====================

/**
 * @brief  通过 I2C 向 OLED 写入一个字节（命令或数据由控制字节区分）
 * @param  dat  要发送的字节
 * @param  cmd  1=数据(控制字节 0x40)，0=命令(控制字节 0x00)
 */
static void OLED_WriteByte(uint8_t dat, uint8_t cmd)
{
    uint8_t ctrl = cmd ? OLED_CTRL_DATA : OLED_CTRL_CMD;
    /* SSD1306 I2C：Mem_Write 的"寄存器地址"位置即控制字节 */
    HAL_I2C_Mem_Write(&hi2c1, OLED_I2C_ADDR, ctrl, I2C_MEMADD_SIZE_8BIT, &dat, 1, 100);
}

/**
 * @brief  批量写数据（整段数据一个 I2C 事务写完，比逐字节写快几十倍）
 * @param  data  数据指针
 * @param  len   字节数
 */
static void OLED_WriteData(const uint8_t *data, uint16_t len)
{
    HAL_I2C_Mem_Write(&hi2c1, OLED_I2C_ADDR, OLED_CTRL_DATA,
                      I2C_MEMADD_SIZE_8BIT, (uint8_t *)data, len, 100);
}

/**
 * @brief  设置光标位置（页寻址模式）
 */
static void OLED_SetCursor(uint8_t x, uint8_t y)
{
    OLED_WriteByte(0xB0 + y, 0);                     // 设置页地址
    OLED_WriteByte(SSD1306_SETLOWCOLUMN  | (x & 0x0F), 0);  // 列低4位
    OLED_WriteByte(SSD1306_SETHIGHCOLUMN | (x >> 4),   0);  // 列高4位
}

// ==================== 公开函数 ====================

/**
 * @brief  OLED 初始化（I2C 版）
 * @note   4 脚 I2C 模块没有 RES 引脚，直接发送 SSD1306 初始化命令序列即可
 */
void OLED_Init(void)
{
    // SSD1306 初始化序列
    OLED_WriteByte(SSD1306_DISPLAYOFF, 0);          // AE: 关闭显示
    OLED_WriteByte(SSD1306_SETDISPLAYCLOCKDIV, 0);  // D5: 设置时钟分频
    OLED_WriteByte(0x80, 0);                        // 分频比 / 频率 (默认)
    OLED_WriteByte(SSD1306_SETMULTIPLEX, 0);        // A8: 多路复用率
    OLED_WriteByte(0x3F, 0);                        // 1/64 duty
    OLED_WriteByte(SSD1306_SETDISPLAYOFFSET, 0);    // D3: 显示偏移
    OLED_WriteByte(0x00, 0);                        // 无偏移
    OLED_WriteByte(SSD1306_SETSTARTLINE | 0x00, 0); // 40: 起始行
    OLED_WriteByte(SSD1306_CHARGEPUMP, 0);          // 8D: 电荷泵
    OLED_WriteByte(0x14, 0);                        // 使能电荷泵 (VCC 由内部产生)
    OLED_WriteByte(SSD1306_MEMORYMODE, 0);          // 20: 内存寻址模式
    OLED_WriteByte(0x02, 0);                        // 页寻址模式，与 OLED_SetCursor 的 B0~B7/列地址命令一致
    OLED_WriteByte(SSD1306_SEGREMAP | 0x01, 0);     // A1: 列重映射 (左右翻转)
    OLED_WriteByte(SSD1306_COMSCANDEC, 0);          // C8: COM 扫描方向
    OLED_WriteByte(SSD1306_SETCOMPINS, 0);          // DA: COM 引脚硬件配置
    OLED_WriteByte(0x12, 0);
    OLED_WriteByte(SSD1306_SETCONTRAST, 0);         // 81: 对比度
    OLED_WriteByte(0xCF, 0);
    OLED_WriteByte(SSD1306_SETPRECHARGE, 0);        // D9: 预充电周期
    OLED_WriteByte(0xF1, 0);
    OLED_WriteByte(SSD1306_SETVCOMDETECT, 0);       // DB: VCOMH 电压
    OLED_WriteByte(0x40, 0);
    OLED_WriteByte(SSD1306_DISPLAYALLON_RESUME, 0); // A4: 整体显示关闭 (正常)
    OLED_WriteByte(SSD1306_NORMALDISPLAY, 0);       // A6: 正常显示 (非反相)
    OLED_WriteByte(SSD1306_DISPLAYON, 0);           // AF: 开启显示

    OLED_Clear();
}

/**
 * @brief  清屏
 */
void OLED_Clear(void)
{
    uint8_t page;
    static const uint8_t zero[OLED_WIDTH] = {0};   // 128 字节全 0

    for (page = 0; page < OLED_PAGES; page++) {
        OLED_SetCursor(0, page);
        OLED_WriteData(zero, OLED_WIDTH);          // 每页一次 I2C 事务写完
    }
}

/**
 * @brief  3x 垂直扩展辅助函数
 *         将 8 行原始字模数据按 3x 垂直缩放映射到一个输出字节
 * @param  src   原始字节（8 像素列，bit0=顶行…bit7=底行）
 * @param  group 输出页组号 (0~2)，决定当前输出页映射哪些原始行
 * @return 缩放后的 8 像素列数据
 */
static uint8_t OLED_ScaleByte(uint8_t src, uint8_t group)
{
    // 3x 垂直缩放映射表：3 个输出页分别从原始 bit 的哪些行取样
    // group 0: 输出行 0~7  ← 原始行 0,0,0,1,1,1,2,2
    // group 1: 输出行 8~15 ← 原始行 2,3,3,3,4,4,4,5
    // group 2: 输出行 16~23← 原始行 5,5,6,6,6,7,7,7
    static const uint8_t remap[3][8] = {
        {0, 0, 0, 1, 1, 1, 2, 2},
        {2, 3, 3, 3, 4, 4, 4, 5},
        {5, 5, 6, 6, 6, 7, 7, 7}
    };
    uint8_t out = 0;
    uint8_t b;
    for (b = 0; b < 8; b++) {
        if (src & (1 << remap[group][b])) {
            out |= (1 << b);
        }
    }
    return out;
}

/**
 * @brief  在指定位置显示单个数字字符（原始 8x16 → 3x 放大为 24x48 字体）
 * @param  x  列坐标 (0~104)
 * @param  y  页坐标 (0~2)，数字占 6 页（y 到 y+5），共 48 像素高
 * @param  ch 要显示的数字 (0~9)
 */
void OLED_ShowChar(uint8_t x, uint8_t y, uint8_t ch)
{
    uint8_t i, page, group;

    if (ch > 9) return;
    if (y > OLED_PAGES - 6) return;  // 6 页 (48 像素) 溢出检查

    for (page = 0; page < 6; page++) {
        if (page < 3) {
            group = page;                             // 原始上半页 (page 0)
        } else {
            group = page - 3;                         // 原始下半页 (page 1)
        }
        OLED_SetCursor(x, y + page);
        for (i = 0; i < 8; i++) {                    // 遍历 8 个原始列
            uint8_t src_byte;
            if (page < 3) {
                src_byte = F8X16_NUM[ch][i];         // 原始上半页
            } else {
                src_byte = F8X16_NUM[ch][i + 8];     // 原始下半页
            }
            uint8_t scaled = OLED_ScaleByte(src_byte, group);
            // 3x 水平扩展：每列写 3 次
            OLED_WriteByte(scaled, 1);
            OLED_WriteByte(scaled, 1);
            OLED_WriteByte(scaled, 1);
        }
    }
}

/**
 * @brief  在指定位置显示两位数 (00~99)
 *         调用 OLED_ShowChar 分别显示十位和个位
 * @param  x   十位的起始列 (0~111)
 * @param  y   页坐标 (0~6)
 * @param  num 要显示的数字 (0~99)
 */
void OLED_ShowNum(uint8_t x, uint8_t y, uint8_t num)
{
    uint8_t tens, ones;

    if (num > 99) return;

    tens = num / 10;
    ones = num % 10;

    OLED_ShowChar(x, y, tens);       // 十位
    OLED_ShowChar(x + 24, y, ones);  // 个位（24 像素偏移，3x 放大）
}

// ==================== 6x8 ASCII 字库（空格 ' ' ~ 'Z'） ====================
// 每个字符 6 字节宽、8 像素高（占 1 页）
// 来源：标准 Adafruit GFX glcdfont 字库，已验证正确
static const uint8_t F6X8_FONT[96][6] = {
    {0x00,0x00,0x00,0x00,0x00,0x00}, //   
    {0x00,0x00,0x5F,0x00,0x00,0x00}, // ! 
    {0x00,0x07,0x00,0x07,0x00,0x00}, // " 
    {0x14,0x7F,0x14,0x7F,0x14,0x00}, // # 
    {0x24,0x2A,0x7F,0x2A,0x12,0x00}, // $ 
    {0x23,0x13,0x08,0x64,0x62,0x00}, // % 
    {0x36,0x49,0x55,0x22,0x50,0x00}, // & 
    {0x00,0x05,0x03,0x00,0x00,0x00}, // ' 
    {0x00,0x1C,0x22,0x41,0x00,0x00}, // ( 
    {0x00,0x41,0x22,0x1C,0x00,0x00}, // ) 
    {0x08,0x2A,0x1C,0x2A,0x08,0x00}, // * 
    {0x08,0x08,0x3E,0x08,0x08,0x00}, // + 
    {0x00,0x50,0x30,0x00,0x00,0x00}, // , 
    {0x08,0x08,0x08,0x08,0x08,0x00}, // - 
    {0x00,0x60,0x60,0x00,0x00,0x00}, // . 
    {0x20,0x10,0x08,0x04,0x02,0x00}, // / 
    {0x3E,0x51,0x49,0x45,0x3E,0x00}, // 0 
    {0x00,0x42,0x7F,0x40,0x00,0x00}, // 1 
    {0x42,0x61,0x51,0x49,0x46,0x00}, // 2 
    {0x21,0x41,0x45,0x4B,0x31,0x00}, // 3 
    {0x18,0x14,0x12,0x7F,0x10,0x00}, // 4 
    {0x27,0x45,0x45,0x45,0x39,0x00}, // 5 
    {0x3C,0x4A,0x49,0x49,0x30,0x00}, // 6 
    {0x01,0x71,0x09,0x05,0x03,0x00}, // 7 
    {0x36,0x49,0x49,0x49,0x36,0x00}, // 8 
    {0x06,0x49,0x49,0x29,0x1E,0x00}, // 9 
    {0x00,0x36,0x36,0x00,0x00,0x00}, // : 
    {0x00,0x56,0x36,0x00,0x00,0x00}, // ; 
    {0x00,0x08,0x14,0x22,0x41,0x00}, // < 
    {0x14,0x14,0x14,0x14,0x14,0x00}, // = 
    {0x41,0x22,0x14,0x08,0x00,0x00}, // > 
    {0x02,0x01,0x51,0x09,0x06,0x00}, // ? 
    {0x32,0x49,0x79,0x41,0x3E,0x00}, // @ 
    {0x7E,0x11,0x11,0x11,0x7E,0x00}, // A 
    {0x7F,0x49,0x49,0x49,0x36,0x00}, // B 
    {0x3E,0x41,0x41,0x41,0x22,0x00}, // C 
    {0x7F,0x41,0x41,0x22,0x1C,0x00}, // D 
    {0x7F,0x49,0x49,0x49,0x41,0x00}, // E 
    {0x7F,0x09,0x09,0x01,0x01,0x00}, // F 
    {0x3E,0x41,0x41,0x51,0x32,0x00}, // G 
    {0x7F,0x08,0x08,0x08,0x7F,0x00}, // H 
    {0x00,0x41,0x7F,0x41,0x00,0x00}, // I 
    {0x20,0x40,0x41,0x3F,0x01,0x00}, // J 
    {0x7F,0x08,0x14,0x22,0x41,0x00}, // K 
    {0x7F,0x40,0x40,0x40,0x40,0x00}, // L 
    {0x7F,0x02,0x04,0x02,0x7F,0x00}, // M 
    {0x7F,0x04,0x08,0x10,0x7F,0x00}, // N 
    {0x3E,0x41,0x41,0x41,0x3E,0x00}, // O 
    {0x7F,0x09,0x09,0x09,0x06,0x00}, // P 
    {0x3E,0x41,0x51,0x21,0x5E,0x00}, // Q 
    {0x7F,0x09,0x19,0x29,0x46,0x00}, // R 
    {0x46,0x49,0x49,0x49,0x31,0x00}, // S 
    {0x01,0x01,0x7F,0x01,0x01,0x00}, // T 
    {0x3F,0x40,0x40,0x40,0x3F,0x00}, // U 
    {0x1F,0x20,0x40,0x20,0x1F,0x00}, // V 
    {0x7F,0x20,0x18,0x20,0x7F,0x00}, // W 
    {0x63,0x14,0x08,0x14,0x63,0x00}, // X 
    {0x03,0x04,0x78,0x04,0x03,0x00}, // Y 
    {0x61,0x51,0x49,0x45,0x43,0x00}, // Z
};

/**
 * @brief  在第一行显示标题 "CAR TIME:"
 *         使用 6x8 小字体，占 page 0（仅 8 像素高）
 */
void OLED_ShowTitle(void)
{
    uint8_t x = 0;
    uint8_t i;

    // "CAR TIME:" 依次写每个字符
    // C=67('C'-' ') A=65 R=82 ' '=0 T=84 I=73 M=77 E=69 :=58
    OLED_SetCursor(x, 0);
    for (i = 0; i < 6; i++) {
        OLED_WriteByte(F6X8_FONT['C' - ' '][i], 1);
    }
    x += 6;

    OLED_SetCursor(x, 0);
    for (i = 0; i < 6; i++) {
        OLED_WriteByte(F6X8_FONT['A' - ' '][i], 1);
    }
    x += 6;

    OLED_SetCursor(x, 0);
    for (i = 0; i < 6; i++) {
        OLED_WriteByte(F6X8_FONT['R' - ' '][i], 1);
    }
    x += 6;

    OLED_SetCursor(x, 0);
    for (i = 0; i < 6; i++) {
        OLED_WriteByte(F6X8_FONT[0][i], 1);   // 空格
    }
    x += 6;

    OLED_SetCursor(x, 0);
    for (i = 0; i < 6; i++) {
        OLED_WriteByte(F6X8_FONT['T' - ' '][i], 1);
    }
    x += 6;

    OLED_SetCursor(x, 0);
    for (i = 0; i < 6; i++) {
        OLED_WriteByte(F6X8_FONT['I' - ' '][i], 1);
    }
    x += 6;

    OLED_SetCursor(x, 0);
    for (i = 0; i < 6; i++) {
        OLED_WriteByte(F6X8_FONT['M' - ' '][i], 1);
    }
    x += 6;

    OLED_SetCursor(x, 0);
    for (i = 0; i < 6; i++) {
        OLED_WriteByte(F6X8_FONT['E' - ' '][i], 1);
    }
    x += 6;

    OLED_SetCursor(x, 0);
    for (i = 0; i < 6; i++) {
        OLED_WriteByte(F6X8_FONT[':' - ' '][i], 1);
    }
}

/**
 * @brief  在指定位置显示字符串（6x8 小字体，占 1 页）
 *         先把整行字模拼进缓冲区，再一次性 I2C 事务写入（避免逐字节慢速刷新）
 * @param  x   起始列坐标（0 ~ 121）
 * @param  y   页坐标（0 ~ 7）
 * @param  str 字符串指针（字库仅覆盖 ' ' ~ 'Z'，超范围字符显示为空格）
 */
void OLED_ShowString(uint8_t x, uint8_t y, const char *str)
{
    uint8_t  buf[OLED_WIDTH / 6 * 6];   // 最多 21 个字符 × 6 字节 = 126
    uint16_t n = 0;

    if (y > OLED_PAGES - 1) return;
    if (str == NULL) return;

    while (*str != '\0' && n + 6 <= sizeof(buf))
    {
        uint8_t ch = (uint8_t)(*str);

        // 字库 F6X8_FONT 仅覆盖 ' ' ~ 'Z'，超出范围显示为空格
        if (ch < ' ' || ch > 'Z') ch = ' ';

        memcpy(&buf[n], F6X8_FONT[ch - ' '], 6);
        n += 6;
        str++;
    }

    OLED_SetCursor(x, y);
    if (n > 0) OLED_WriteData(buf, n);
}

/**
 * @brief  显示三个 TOFSense 测距传感器的实时距离（单位 cm）
 *         布局（6x8 小字体）：
 *           page 0：标题 "DIST(CM)"
 *           page 1~3：S0 / S1 / S2，每行固定 8 字符（"S0:123.4" 或 "S0:  -.-"），
 *                     固定宽度保证数值变化时不会残留旧像素
 * @note   在主循环中周期调用；数据来自 tofsense.c 的全局镜像
 */
void OLED_ShowDistance(void)
{
    char   line[16];
    uint8_t s;

    // 标题
    OLED_ShowString(0, 0, "DIST(CM)");

    // 三个槽位 S0/S1/S2 的实时距离
    for (s = 0; s < TOFSENSE_NUM_SENSORS; s++)
    {
        if (tofsense_valid[s] && tofsense_distance_cm[s] >= 0.0f)
        {
            // 定点整数格式，避免使用不支持 %f 的 nano printf
            int32_t deci = (int32_t)(tofsense_distance_cm[s] * 10.0f + 0.5f);
            (void)snprintf(line, sizeof(line), "S%u:%3ld.%ld",
                           s, (long)(deci / 10), (long)(deci % 10));
        }
        else
        {
            // 无目标 / 校验失败：显示 "--.-"
            (void)snprintf(line, sizeof(line), "S%u:  -.-", s);
        }
        OLED_ShowString(0, (uint8_t)(1 + s), line);
    }
}

/**
 * @brief  显示遥控器 CH1~CH8 通道原始值（SBUS，中位 992）
 *         布局（6x8 小字体，每行占 1 页，占用 page 4~7）：
 *           page 4：CH1  CH2
 *           page 5：CH3  CH4
 *           page 6：CH5  CH6
 *           page 7：CH7  CH8
 *         每通道固定宽度 "CHx:%4ld"（如 "CH1: 992"），数值位数
 *         变化（3 位↔4 位）不会残留旧像素
 * @note   数据来自 yaokong.c 的全局 sbus_ch[]；在主循环中周期调用
 */
void OLED_ShowChannels(void)
{
    char    line[24];
    uint8_t i;

    /* 每行两个通道：CH(2i+1) 在左、CH(2i+2) 在右，固定宽度防残留 */
    for (i = 0; i < 4; i++)
    {
        (void)snprintf(line, sizeof(line), "CH%u:%4ld CH%u:%4ld",
                       (unsigned)(i * 2 + 1), (long)sbus_ch[i * 2],
                       (unsigned)(i * 2 + 2), (long)sbus_ch[i * 2 + 1]);
        OLED_ShowString(0, (uint8_t)(4 + i), line);
    }
}
