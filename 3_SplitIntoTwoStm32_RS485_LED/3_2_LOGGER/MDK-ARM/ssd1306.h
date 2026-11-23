#ifndef SSD1306_H
#define SSD1306_H

#include "main.h"

#define SSD1306_ADDR        (0x3C << 1)

#define SSD1306_WIDTH       128
#define SSD1306_HEIGHT      64

#define SSD1306_COL_OFFSET  0

HAL_StatusTypeDef SSD1306_Init(void);
void SSD1306_Clear(void);
void SSD1306_DrawString(int x, int y, const char *s, uint8_t scale);
void SSD1306_DrawStringCentered(int y, const char *s, uint8_t scale);
HAL_StatusTypeDef SSD1306_Update(void);

#endif
