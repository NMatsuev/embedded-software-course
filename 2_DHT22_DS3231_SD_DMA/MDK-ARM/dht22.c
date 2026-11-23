#include "dht22.h"

extern TIM_HandleTypeDef htim2;

static void delay_us(uint16_t us)
{
    uint16_t start = __HAL_TIM_GET_COUNTER(&htim2);
    while ((uint16_t)(__HAL_TIM_GET_COUNTER(&htim2) - start) < us);
}

static void dht22_set_output(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = DHT22_DATA_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(DHT22_DATA_GPIO_Port, &GPIO_InitStruct);
}

static void dht22_set_input(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = DHT22_DATA_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(DHT22_DATA_GPIO_Port, &GPIO_InitStruct);
}

static uint8_t dht22_read_bit(void)
{
    uint32_t start;
    uint32_t timeout;

    /* LOW (~50 us) */
    timeout = 0;
    while (HAL_GPIO_ReadPin(DHT22_DATA_GPIO_Port, DHT22_DATA_Pin) == GPIO_PIN_RESET)
    {
        if (++timeout > 100)
            return 0;

        delay_us(1);
    }

    /* HIGH */
    start = __HAL_TIM_GET_COUNTER(&htim2);

    /* HIGH */
    timeout = 0;
    while (HAL_GPIO_ReadPin(DHT22_DATA_GPIO_Port, DHT22_DATA_Pin) == GPIO_PIN_SET)
    {
        if (++timeout > 100)
            return 0;

        delay_us(1);
    }

    /* HIGH */
    uint16_t high_time =
        (uint16_t)(__HAL_TIM_GET_COUNTER(&htim2) - start);

    /* ~26 us = 0, ~70 us = 1 */
    return (high_time > 40) ? 1 : 0;
}

static uint8_t dht22_read_byte(void)
{
    uint8_t byte = 0;
    for (int i = 0; i < 8; i++)
    {
        byte <<= 1;
        byte |= dht22_read_bit();
    }
    return byte;
}

void DHT22_Init(void)
{
    dht22_set_output();
    HAL_GPIO_WritePin(DHT22_DATA_GPIO_Port, DHT22_DATA_Pin, GPIO_PIN_SET);
    HAL_Delay(1000);
}

uint8_t DHT22_Read(DHT22_Data_t *data)
{
    uint8_t raw[5] = {0};

    /* Start signal */
    dht22_set_output();

    HAL_GPIO_WritePin(
        DHT22_DATA_GPIO_Port,
        DHT22_DATA_Pin,
        GPIO_PIN_RESET
    );

    HAL_Delay(2);

    /* Release bus */
    HAL_GPIO_WritePin(
        DHT22_DATA_GPIO_Port,
        DHT22_DATA_Pin,
        GPIO_PIN_SET
    );

    dht22_set_input();

    delay_us(30);

    /* DHT22 response: LOW ~80 us */
    uint16_t timeout = 0;

    while (HAL_GPIO_ReadPin(
               DHT22_DATA_GPIO_Port,
               DHT22_DATA_Pin) == GPIO_PIN_SET)
    {
        if (++timeout > 100)
            return 0;

        delay_us(1);
    }

    /* HIGH ~80 us */
    timeout = 0;

    while (HAL_GPIO_ReadPin(
               DHT22_DATA_GPIO_Port,
               DHT22_DATA_Pin) == GPIO_PIN_RESET)
    {
        if (++timeout > 100)
            return 0;

        delay_us(1);
    }

    /* LOW before first data bit */
    timeout = 0;

    while (HAL_GPIO_ReadPin(
               DHT22_DATA_GPIO_Port,
               DHT22_DATA_Pin) == GPIO_PIN_SET)
    {
        if (++timeout > 100)
            return 0;

        delay_us(1);
    }

    /* Read 40 bits */
    for (int i = 0; i < 5; i++)
    {
        raw[i] = dht22_read_byte();
    }

    /* Checksum */
    uint8_t checksum =
        raw[0] + raw[1] + raw[2] + raw[3];

    data->checksum_ok = (checksum == raw[4]);

    if (!data->checksum_ok)
        return 0;

    /* Humidity */
    uint16_t hum_raw =
        ((uint16_t)raw[0] << 8) | raw[1];

    /* Temperature */
    uint16_t temp_raw =
        ((uint16_t)raw[2] << 8) | raw[3];

    data->humidity = hum_raw / 10.0f;

    if (temp_raw & 0x8000)
    {
        temp_raw &= 0x7FFF;
        data->temperature = -(temp_raw / 10.0f);
    }
    else
    {
        data->temperature = temp_raw / 10.0f;
    }

    return 1;
}