#ifndef DS3231_H
#define DS3231_H

#include "main.h"

typedef struct {
    uint8_t seconds;
    uint8_t minutes;
    uint8_t hours;
    uint8_t day;
    uint8_t date;
    uint8_t month;
    uint16_t year;
} DS3231_Time_t;

void DS3231_Init(void);
void DS3231_ReadTime(DS3231_Time_t *time);
void DS3231_SetTime(DS3231_Time_t *time);

#endif