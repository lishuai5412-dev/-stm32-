#include "oled.h"

// ==========================================
// 极其精简的 6x8 ASCII 软件字库 (只包含常用字符，节省内存)
// ==========================================
const uint8_t OLED_F6x8[][6] = {
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, // ' ' (空格)
    {0x3E, 0x51, 0x49, 0x45, 0x3E, 0x00}, // '0'
    {0x00, 0x42, 0x7F, 0x40, 0x00, 0x00}, // '1'
    {0x42, 0x61, 0x51, 0x49, 0x46, 0x00}, // '2'
    {0x21, 0x41, 0x45, 0x4B, 0x31, 0x00}, // '3'
    {0x18, 0x14, 0x12, 0x7F, 0x10, 0x00}, // '4'
    {0x27, 0x45, 0x45, 0x45, 0x39, 0x00}, // '5'
    {0x3C, 0x4A, 0x49, 0x49, 0x30, 0x00}, // '6'
    {0x01, 0x71, 0x09, 0x05, 0x03, 0x00}, // '7'
    {0x36, 0x49, 0x49, 0x49, 0x36, 0x00}, // '8'
    {0x06, 0x49, 0x49, 0x29, 0x1E, 0x00}, // '9'
    {0x00, 0x36, 0x36, 0x00, 0x00, 0x00}, // ':'
    {0x00, 0x00, 0x60, 0x60, 0x00, 0x00}, // '.'
    {0x3E, 0x41, 0x41, 0x41, 0x22, 0x00}, // 'C'
    {0x01, 0x01, 0x7F, 0x01, 0x01, 0x00}, // 'T'
    {0x38, 0x54, 0x54, 0x54, 0x18, 0x00}, // 'e'
    {0x7C, 0x08, 0x04, 0x04, 0x78, 0x00}, // 'm'
    {0x7C, 0x14, 0x14, 0x14, 0x08, 0x00}, // 'p'
};


// 向 OLED 发送命令
void OLED_WriteCommand(uint8_t cmd) {
    // 使用 STM32 HAL 库的 I2C 硬件发送函数
    HAL_I2C_Mem_Write(OLED_I2C_PORT, OLED_ADDRESS, 0x00, I2C_MEMADD_SIZE_8BIT, &cmd, 1, 100);
}

// 向 OLED 发送数据
void OLED_WriteData(uint8_t data) {
    // 使用 STM32 HAL 库的 I2C 硬件发送函数
    HAL_I2C_Mem_Write(OLED_I2C_PORT, OLED_ADDRESS, 0x40, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);
}

// 初始化 OLED
void OLED_Init(void) {
    HAL_Delay(100); // 等待 OLED 上电稳定

    OLED_WriteCommand(0xAE); // 关闭显示
    OLED_WriteCommand(0x00); // 设置低列地址
    OLED_WriteCommand(0x10); // 设置高列地址
    OLED_WriteCommand(0x40); // 设置起始行地址
    OLED_WriteCommand(0xB0); // 设置页地址
    OLED_WriteCommand(0x81); // 对比度设置
    OLED_WriteCommand(0xFF); // 对比度最大
    OLED_WriteCommand(0xA1); // 段重映射设置
    OLED_WriteCommand(0xA6); // 正常显示 (非反显)
    OLED_WriteCommand(0xA8); // 设置多路复用率
    OLED_WriteCommand(0x3F); // 1/64 duty
    OLED_WriteCommand(0xC8); // COM 扫描方向设置
    OLED_WriteCommand(0xD3); // 设置显示偏移
    OLED_WriteCommand(0x00); 
    OLED_WriteCommand(0xD5); // 设置振荡器频率
    OLED_WriteCommand(0x80); 
    OLED_WriteCommand(0xD8); // 设置区域颜色模式
    OLED_WriteCommand(0x05);
    OLED_WriteCommand(0xD9); // 设置预充电周期
    OLED_WriteCommand(0xF1);
    OLED_WriteCommand(0xDA); // 设置 COM 引脚硬件配置
    OLED_WriteCommand(0x12);
    OLED_WriteCommand(0xDB); // 设置 VCOMH 偏压倍率
    OLED_WriteCommand(0x30);
    OLED_WriteCommand(0x8D); // 电荷泵设置
    OLED_WriteCommand(0x14); // 开启电荷泵
    OLED_WriteCommand(0xAF); // 开启显示
}

// 清屏函数
void OLED_Clear(void) {
    uint8_t i, n;
    for (i = 0; i < 8; i++) {
        OLED_WriteCommand(0xb0 + i); // 设置页地址 (0~7)
        OLED_WriteCommand(0x00);     // 设置显示位置—列低地址
        OLED_WriteCommand(0x10);     // 设置显示位置—列高地址
        for (n = 0; n < 128; n++) {
            OLED_WriteData(0);       // 写入 0，清空像素
        }
    }
}

// 设置光标位置 (x: 0~127, y: 0~7 页)
void OLED_SetCursor(uint8_t x, uint8_t y) {
    OLED_WriteCommand(0xb0 + y);
    OLED_WriteCommand(((x & 0xf0) >> 4) | 0x10);
    OLED_WriteCommand(x & 0x0f);
}

// 显示单个字符 (基于极其精简的字库)
// 这个精简版只支持 6x8 大小的字符
void OLED_ShowChar(uint8_t x, uint8_t y, char chr, uint8_t size) {
    uint8_t i = 0;
    
    // 这里做了一个极简的映射，因为上面的字库不完整
    // 在实际项目中，你应该使用完整的 ASCII 字库数组
    uint8_t index = 0;
    if(chr >= '0' && chr <= '9') index = chr - '0' + 1; // 1-10是数字
    else if(chr == ':') index = 11;
    else if(chr == '.') index = 12;
    else if(chr == 'C') index = 13;
    else if(chr == 'T') index = 14;
    else if(chr == 'e') index = 15;
    else if(chr == 'm') index = 16;
    else if(chr == 'p') index = 17;
    else index = 0; // 空格
    
    if (x > 128 - 6) { x = 0; y++; }
    OLED_SetCursor(x, y);
    for (i = 0; i < 6; i++) {
        OLED_WriteData(OLED_F6x8[index][i]);
    }
}

// 显示字符串
void OLED_ShowString(uint8_t x, uint8_t y, char *chr, uint8_t size) {
    uint8_t j = 0;
    while (chr[j] != '\0') {
        OLED_ShowChar(x, y, chr[j], size);
        x += 6; // 6x8 字体，下一个字符的 X 坐标增加 6
        if (x > 120) { x = 0; y += 2; }
        j++;
    }
}
