#include "ssd1306.h"
#include <string.h>

extern I2C_HandleTypeDef hi2c1;

#define SSD1306_CTRL_CMD    0x00    
#define SSD1306_CTRL_DATA   0x40    

static uint8_t fb[SSD1306_WIDTH * (SSD1306_HEIGHT / 8)];

typedef struct { char ch; uint8_t col[5]; } glyph_t;

static const glyph_t font[] = {
    {'0', {0x3E, 0x51, 0x49, 0x45, 0x3E}},
    {'1', {0x00, 0x42, 0x7F, 0x40, 0x00}},
    {'2', {0x42, 0x61, 0x51, 0x49, 0x46}},
    {'3', {0x21, 0x41, 0x45, 0x4B, 0x31}},
    {'4', {0x18, 0x14, 0x12, 0x7F, 0x10}},
    {'5', {0x27, 0x45, 0x45, 0x45, 0x39}},
    {'6', {0x3C, 0x4A, 0x49, 0x49, 0x30}},
    {'7', {0x01, 0x71, 0x09, 0x05, 0x03}},
    {'8', {0x36, 0x49, 0x49, 0x49, 0x36}},
    {'9', {0x06, 0x49, 0x49, 0x29, 0x1E}},
    {'.', {0x00, 0x60, 0x60, 0x00, 0x00}},
    {'%', {0x23, 0x13, 0x08, 0x64, 0x62}},
    {'-', {0x08, 0x08, 0x08, 0x08, 0x08}},
    {' ', {0x00, 0x00, 0x00, 0x00, 0x00}},
    {'H', {0x7F, 0x08, 0x08, 0x08, 0x7F}},
    {'U', {0x3F, 0x40, 0x40, 0x40, 0x3F}},
    {'M', {0x7F, 0x02, 0x0C, 0x02, 0x7F}},
    {'I', {0x00, 0x41, 0x7F, 0x41, 0x00}},
    {'D', {0x7F, 0x41, 0x41, 0x22, 0x1C}},
    {'T', {0x01, 0x01, 0x7F, 0x01, 0x01}},
    {'Y', {0x07, 0x08, 0x70, 0x08, 0x07}},
};

static const uint8_t *find_glyph(char c)
{
    for (uint32_t i = 0; i < sizeof(font) / sizeof(font[0]); i++)
    {
        if (font[i].ch == c)
            return font[i].col;
    }
    return NULL;
}

static HAL_StatusTypeDef ssd1306_cmd(uint8_t cmd)
{
    return HAL_I2C_Mem_Write(&hi2c1, SSD1306_ADDR, SSD1306_CTRL_CMD,
                             I2C_MEMADD_SIZE_8BIT, &cmd, 1, 100);
}

HAL_StatusTypeDef SSD1306_Init(void)
{
    if (HAL_I2C_IsDeviceReady(&hi2c1, SSD1306_ADDR, 3, 100) != HAL_OK)
        return HAL_ERROR;

    HAL_Delay(100);  

    static const uint8_t init_seq[] = {
        0xAE,             
        0xD5, 0x80,      
        0xA8, 0x3F,       
        0xD3, 0x00,       
        0x40,             
        0x8D, 0x14,       
        0x20, 0x02,       
        0xA1,             
        0xC8,             
        0xDA, 0x12,       
        0x81, 0xCF,      
        0xD9, 0xF1,       
        0xDB, 0x40,       
        0xA4,             
        0xA6,             
        0xAF             
    };

    for (uint32_t i = 0; i < sizeof(init_seq); i++)
    {
        if (ssd1306_cmd(init_seq[i]) != HAL_OK)
            return HAL_ERROR;
    }

    SSD1306_Clear();
    return SSD1306_Update();
}

void SSD1306_Clear(void)
{
    memset(fb, 0, sizeof(fb));
}

static void set_pixel(int x, int y)
{
    if ((unsigned)x >= SSD1306_WIDTH || (unsigned)y >= SSD1306_HEIGHT)
        return;
    fb[(y >> 3) * SSD1306_WIDTH + x] |= (uint8_t)(1u << (y & 7));
}

static void draw_char(int x, int y, char c, uint8_t scale)
{
    const uint8_t *g = find_glyph(c);
    if (g == NULL)
        return;

    for (int col = 0; col < 5; col++)
        for (int row = 0; row < 7; row++)
            if (g[col] & (1u << row))
                for (int dy = 0; dy < scale; dy++)
                    for (int dx = 0; dx < scale; dx++)
                        set_pixel(x + col * scale + dx, y + row * scale + dy);
}

void SSD1306_DrawString(int x, int y, const char *s, uint8_t scale)
{
    while (*s)
    {
        draw_char(x, y, *s++, scale);
        x += 6 * scale;                
    }
}

void SSD1306_DrawStringCentered(int y, const char *s, uint8_t scale)
{
    int width = (int)strlen(s) * 6 * scale - scale;
    SSD1306_DrawString((SSD1306_WIDTH - width) / 2, y, s, scale);
}

HAL_StatusTypeDef SSD1306_Update(void)
{
    for (uint8_t page = 0; page < SSD1306_HEIGHT / 8; page++)
    {
        if (ssd1306_cmd(0xB0 | page) != HAL_OK) return HAL_ERROR;
        if (ssd1306_cmd(0x00 | (SSD1306_COL_OFFSET & 0x0F)) != HAL_OK) return HAL_ERROR;
        if (ssd1306_cmd(0x10 | (SSD1306_COL_OFFSET >> 4)) != HAL_OK) return HAL_ERROR;

        if (HAL_I2C_Mem_Write(&hi2c1, SSD1306_ADDR, SSD1306_CTRL_DATA,
                              I2C_MEMADD_SIZE_8BIT,
                              &fb[page * SSD1306_WIDTH],
                              SSD1306_WIDTH, 100) != HAL_OK)
            return HAL_ERROR;
    }
    return HAL_OK;
}
