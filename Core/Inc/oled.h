#ifndef __OLED_H
#define __OLED_H

#include "main.h" // 包含 HAL 库头文件

// ==========================================
// 硬件接口配置 (根据你实际的 I2C 句柄修改)
// ==========================================
extern I2C_HandleTypeDef hi2c1; // 声明外部的 I2C 句柄
#define OLED_I2C_PORT &hi2c1    // 定义 OLED 使用的 I2C 接口
#define OLED_ADDRESS  0x78      // OLED 的 I2C 设备地址 (默认通常是 0x78 或 0x7A)

// ==========================================
// 函数声明
// ==========================================
void OLED_WriteCommand(uint8_t cmd);
void OLED_WriteData(uint8_t data);
void OLED_Init(void);
void OLED_Clear(void);
void OLED_SetCursor(uint8_t x, uint8_t y);
void OLED_ShowChar(uint8_t x, uint8_t y, char chr, uint8_t size);
void OLED_ShowString(uint8_t x, uint8_t y, char *chr, uint8_t size);

#endif
