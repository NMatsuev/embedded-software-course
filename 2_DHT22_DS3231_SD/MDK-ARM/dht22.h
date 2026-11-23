#ifndef DHT22_H
#define DHT22_H

#include "main.h"

typedef struct {
    float temperature;
    float humidity;
    uint8_t checksum_ok;
} DHT22_Data_t;

void DHT22_Init(void);
uint8_t DHT22_Read(DHT22_Data_t *data);

#endif