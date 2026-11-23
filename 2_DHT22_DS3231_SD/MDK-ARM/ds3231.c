#include "ds3231.h"
#include "main.h"

extern I2C_HandleTypeDef hi2c1;

#define DS3231_ADDR (0x68 << 1)

/* DS3231 */
#define DS3231_REG_SEC    0x00
#define DS3231_REG_MIN    0x01
#define DS3231_REG_HOUR   0x02
#define DS3231_REG_DAY    0x03
#define DS3231_REG_DATE   0x04
#define DS3231_REG_MONTH  0x05
#define DS3231_REG_YEAR   0x06
#define DS3231_REG_CTRL   0x0E
#define DS3231_REG_STATUS 0x0F

/* BCD -> decimal */
static uint8_t bcd2dec(uint8_t bcd)
{
    return ((bcd >> 4) * 10) + (bcd & 0x0F);
}

/* decimal -> BCD */
static uint8_t dec2bcd(uint8_t dec)
{
    return ((dec / 10) << 4) | (dec % 10);
}

void DS3231_Init(void)
{
    uint8_t data;

    /* Control (0x0E): INTCN=0 (SQW = ??????), RS2:RS1=00 (1 ??), ?????????? ????????? */
    data = 0x00;
    HAL_I2C_Mem_Write(&hi2c1, DS3231_ADDR, DS3231_REG_CTRL,
                      I2C_MEMADD_SIZE_8BIT, &data, 1, 100);

    /* Status (0x0F): ????? OSF, A1F, A2F */
    data = 0x00;
    HAL_I2C_Mem_Write(&hi2c1, DS3231_ADDR, DS3231_REG_STATUS,
                      I2C_MEMADD_SIZE_8BIT, &data, 1, 100);
}

void DS3231_ReadTime(DS3231_Time_t *time)
{
    uint8_t buf[7];

    HAL_I2C_Mem_Read(&hi2c1, DS3231_ADDR, DS3231_REG_SEC,
                     I2C_MEMADD_SIZE_8BIT, buf, 7, 100);

    time->seconds = bcd2dec(buf[0] & 0x7F);
    time->minutes = bcd2dec(buf[1] & 0x7F);
    time->hours   = bcd2dec(buf[2] & 0x3F);
    time->day     = bcd2dec(buf[3] & 0x07);
    time->date    = bcd2dec(buf[4] & 0x3F);
    time->month   = bcd2dec(buf[5] & 0x1F);
    time->year    = 2000 + bcd2dec(buf[6]);
}

void DS3231_SetTime(DS3231_Time_t *time)
{
    uint8_t buf[7];

    buf[0] = dec2bcd(time->seconds);
    buf[1] = dec2bcd(time->minutes);
    buf[2] = dec2bcd(time->hours);
    buf[3] = dec2bcd(time->day);
    buf[4] = dec2bcd(time->date);
    buf[5] = dec2bcd(time->month);
    buf[6] = dec2bcd(time->year - 2000);

    HAL_I2C_Mem_Write(&hi2c1, DS3231_ADDR, DS3231_REG_SEC,
                      I2C_MEMADD_SIZE_8BIT, buf, 7, 100);
}