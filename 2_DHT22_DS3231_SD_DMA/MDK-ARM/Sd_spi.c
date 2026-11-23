/*
 * SD Card SPI Driver
 * Clean version with hot-swap support
 */

#include "Sd_spi.h"
#include "main.h"
#include <string.h>
#include <stdio.h>

extern SPI_HandleTypeDef hspi1;

#define SD_SPI_HANDLE hspi1

#define SD_CS_PORT     GPIOB
#define SD_CS_PIN      GPIO_PIN_0

#define SD_CS_LOW()    HAL_GPIO_WritePin(SD_CS_PORT, SD_CS_PIN, GPIO_PIN_RESET)
#define SD_CS_HIGH()   HAL_GPIO_WritePin(SD_CS_PORT, SD_CS_PIN, GPIO_PIN_SET)

extern void Print(char *message);

static FATFS fs;
static FIL fil;

static char sd_path[4] = "0:/";

static uint8_t sd_card_type = SD_TYPE_UNKNOWN;
volatile uint8_t sd_mounted = 0;
volatile uint8_t sd_use_dma = 1;
extern void disk_reset_drive(BYTE pdrv);
static volatile uint8_t spi_dma_done;     
static uint8_t ff_buf[512];              

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{ if (hspi == &hspi1) spi_dma_done = 1; }

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{ if (hspi == &hspi1) spi_dma_done = 1; }

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{ if (hspi == &hspi1) spi_dma_done = 2; }

static uint8_t spi_wait_dma(uint32_t timeout_ms)
{
    uint32_t t0 = HAL_GetTick();
    while (!spi_dma_done)
    {
        if (HAL_GetTick() - t0 > timeout_ms)
        {
            HAL_SPI_Abort(&hspi1);
            return 1;
        }
    }
    return (spi_dma_done == 1) ? 0 : 1;
}

static uint8_t spi_read_dma(uint8_t *buf, uint16_t len)
{
    spi_dma_done = 0;
    if (HAL_SPI_TransmitReceive_DMA(&hspi1, ff_buf, buf, len) != HAL_OK)
        return 1;
    return spi_wait_dma(100);
}

static uint8_t spi_write_dma(const uint8_t *buf, uint16_t len)
{
    spi_dma_done = 0;
    if (HAL_SPI_Transmit_DMA(&hspi1, (uint8_t *)buf, len) != HAL_OK)
        return 1;
    uint8_t r = spi_wait_dma(100);
    __HAL_SPI_CLEAR_OVRFLAG(&hspi1);      
    return r;
}

/* ============================================================
 * SPI COMMUNICATION
 * ============================================================ */

static uint8_t spi_send(uint8_t data)
{
    uint8_t rx_data = 0xFF;

    if (HAL_SPI_TransmitReceive(&SD_SPI_HANDLE, &data, &rx_data, 1, 10) != HAL_OK)
        return 0xFF;

    return rx_data;
}

static void spi_set_prescaler(uint32_t presc)
{
    __HAL_SPI_DISABLE(&hspi1);
    MODIFY_REG(hspi1.Instance->CR1, SPI_CR1_BR, presc);
    hspi1.Init.BaudRatePrescaler = presc;
    __HAL_SPI_ENABLE(&hspi1);
}


/* ============================================================
 * SD COMMAND
 * ============================================================ */

static uint8_t sd_send_cmd(uint8_t cmd, uint32_t arg)
{
    uint8_t res;
    uint8_t retry;

    spi_send(0xFF);
    spi_send(0x40 | cmd);
    spi_send((uint8_t)(arg >> 24));
    spi_send((uint8_t)(arg >> 16));
    spi_send((uint8_t)(arg >> 8));
    spi_send((uint8_t)(arg));

    if (cmd == CMD0)
        spi_send(0x95);
    else if (cmd == CMD8)
        spi_send(0x87);
    else
        spi_send(0xFF);

    retry = 0;
    do
    {
        res = spi_send(0xFF);
        retry++;
    } while ((res & 0x80) && retry < 20);

    return res;
}


/* ============================================================
 * SD INITIALIZATION
 * ============================================================ */

uint8_t sd_init(void)
{
		memset(ff_buf, 0xFF, sizeof(ff_buf));
		spi_set_prescaler(SPI_BAUDRATEPRESCALER_256);
    uint8_t res;
    uint8_t retry;

    SD_CS_HIGH();
    HAL_Delay(10);

    for (uint32_t i = 0; i < 10; i++)
        spi_send(0xFF);

    SD_CS_LOW();
    HAL_Delay(1);

    /* CMD0 */
    retry = 0;
    do
    {
        res = sd_send_cmd(CMD0, 0);
        retry++;
        if (res != 0x01)
            HAL_Delay(10);
    } while (res != 0x01 && retry < 100);

    if (res != 0x01)
    {
        SD_CS_HIGH();
        return 1;
    }

    /* CMD8 */
    res = sd_send_cmd(CMD8, 0x000001AA);

    if (res == 0x01)
    {
        uint8_t r7[4];
        r7[0] = spi_send(0xFF);
        r7[1] = spi_send(0xFF);
        r7[2] = spi_send(0xFF);
        r7[3] = spi_send(0xFF);

        if (r7[2] != 0x01 || r7[3] != 0xAA)
        {
            SD_CS_HIGH();
            return 2;
        }

        /* ACMD41 */
        retry = 0;
        do
        {
            sd_send_cmd(CMD55, 0);
            res = sd_send_cmd(ACMD41, 0x40000000);
            retry++;
            if (res != 0x00)
                HAL_Delay(10);
        } while (res != 0x00 && retry < 100);

        if (res != 0x00)
        {
            SD_CS_HIGH();
            return 3;
        }

        /* CMD58 */
        res = sd_send_cmd(CMD58, 0);
        if (res != 0x00)
        {
            SD_CS_HIGH();
            return 4;
        }

        {
            uint8_t ocr[4];
            ocr[0] = spi_send(0xFF);
            ocr[1] = spi_send(0xFF);
            ocr[2] = spi_send(0xFF);
            ocr[3] = spi_send(0xFF);

            if (ocr[0] & 0x40)
                sd_card_type = SD_TYPE_V2HC;
            else
                sd_card_type = SD_TYPE_V2;
        }
    }
    else
    {
        sd_card_type = SD_TYPE_V1;

        retry = 0;
        do
        {
            sd_send_cmd(CMD55, 0);
            res = sd_send_cmd(ACMD41, 0);
            retry++;
            if (res != 0x00)
                HAL_Delay(10);
        } while (res != 0x00 && retry < 100);

        if (res != 0x00)
        {
            SD_CS_HIGH();
            return 5;
        }
    }

    /* CMD16 */
    sd_send_cmd(CMD16, 512);

    SD_CS_HIGH();

    for (int i = 0; i < 8; i++)
        spi_send(0xFF);
		
		spi_set_prescaler(SPI_BAUDRATEPRESCALER_8);

    return 0;
}


/* ============================================================
 * READ BLOCK
 * ============================================================ */

uint8_t sd_read_block(uint8_t *buf, uint32_t sector)
{
    uint8_t res;
    uint32_t retry;

    SD_CS_LOW();

    if (sd_card_type != SD_TYPE_V2HC)
        sector *= 512;

    res = sd_send_cmd(CMD17, sector);

    if (res != 0x00)
    {
        SD_CS_HIGH();
        for (int i = 0; i < 8; i++) spi_send(0xFF);
        return 1;
    }

    retry = 0;
    do
    {
        res = spi_send(0xFF);
        retry++;
    } while (res != 0xFE && retry < 5000);

    if (res != 0xFE)
    {
        SD_CS_HIGH();
        for (int i = 0; i < 8; i++) spi_send(0xFF);
        return 2;
    }

    if (sd_use_dma)
    {
        if (spi_read_dma(buf, 512) != 0)
        {
            SD_CS_HIGH();
            for (int i = 0; i < 8; i++) spi_send(0xFF);
            return 3;
        }
    }
    else
    {
        for (int i = 0; i < 512; i++) buf[i] = spi_send(0xFF);
    }

    spi_send(0xFF);
    spi_send(0xFF);

    SD_CS_HIGH();
    for (int i = 0; i < 8; i++) spi_send(0xFF);

    return 0;
}


/* ============================================================
 * WRITE BLOCK
 * ============================================================ */

uint8_t sd_write_block(const uint8_t *buf, uint32_t sector)
{
    uint8_t res;
    uint32_t retry = 0;

    SD_CS_LOW();

    if (sd_card_type != SD_TYPE_V2HC)
        sector *= 512;

    res = sd_send_cmd(CMD24, sector);

    if (res != 0x00)
    {
        SD_CS_HIGH();
        for (int i = 0; i < 8; i++) spi_send(0xFF);
        return 1;
    }

    spi_send(0xFF);
    spi_send(0xFE);

		if (spi_write_dma(buf, 512) != 0)
		{
				SD_CS_HIGH();
				for (int i = 0; i < 8; i++) spi_send(0xFF);
				return 4;
		}

    spi_send(0xFF);
    spi_send(0xFF);

    res = spi_send(0xFF);

    if ((res & 0x1F) != 0x05)
    {
        SD_CS_HIGH();
        for (int i = 0; i < 8; i++) spi_send(0xFF);
        return 2;
    }

    retry = 0;
    do
    {
        res = spi_send(0xFF);
        retry++;
    } while (res == 0x00 && retry < 50000);

    if (res == 0x00)
    {
        SD_CS_HIGH();
        for (int i = 0; i < 8; i++) spi_send(0xFF);
        return 3;
    }

    SD_CS_HIGH();
    for (int i = 0; i < 8; i++) spi_send(0xFF);

    return 0;
}


/* ============================================================
 * MULTI BLOCK READ / WRITE
 * ============================================================ */

uint8_t sd_read_blocks(uint8_t *buf, uint32_t sector, uint32_t count)
{
    for (uint32_t i = 0; i < count; i++)
        if (sd_read_block(buf + (i * 512), sector + i) != 0)
            return 1;
    return 0;
}

uint8_t sd_write_blocks(const uint8_t *buf, uint32_t sector, uint32_t count)
{
    for (uint32_t i = 0; i < count; i++)
        if (sd_write_block(buf + (i * 512), sector + i) != 0)
            return 1;
    return 0;
}


/* ============================================================
 * MOUNT / UNMOUNT
 * ============================================================ */


int sd_mount(void)
{
    FRESULT res;

    sd_mounted = 0;

    f_mount(NULL, sd_path, 0);
    disk_reset_drive(0);

    res = f_mount(&fs, sd_path, 1);

    if (res != FR_OK)
    {
        char msg[64];

        snprintf(msg, sizeof(msg),
                 "[SD] FatFs mount error=%d\r\n",
                 (int)res);
        Print(msg);

        sd_mounted = 0;
        return -1;
    }

    sd_mounted = 1;
    Print("[SD] mounted successfully\r\n");

    return 0;
}

void sd_unmount(void)
{
    sd_mounted = 0;

    f_mount(NULL, sd_path, 0);

    disk_reset_drive(0);

    SD_CS_HIGH();

    Print("[SD] unmounted\r\n");
}



/* ============================================================
 * FILE OPERATIONS
 * ============================================================ */

int sd_write_file(const char *filename, const char *data)
{
    FRESULT res;
    UINT bw;

    res = f_open(&fil, filename, FA_CREATE_ALWAYS | FA_WRITE);

    if (res != FR_OK)
        return -1;

    res = f_write(&fil, data, strlen(data), &bw);
    f_close(&fil);

    HAL_Delay(10);
    SD_CS_HIGH();
    for (int i = 0; i < 8; i++) spi_send(0xFF);

    return (res == FR_OK) ? 0 : -1;
}

int sd_append_file(const char *filename, const char *data)
{
    FRESULT res;
    UINT bw;

    res = f_open(&fil, filename, FA_OPEN_ALWAYS | FA_WRITE);

    if (res != FR_OK)
        return -1;

    res = f_lseek(&fil, f_size(&fil));

    if (res != FR_OK)
    {
        f_close(&fil);
        return -1;
    }

    res = f_write(&fil, data, strlen(data), &bw);
    f_close(&fil);

    HAL_Delay(10);
    SD_CS_HIGH();
    for (int i = 0; i < 8; i++) spi_send(0xFF);

    return (res == FR_OK) ? 0 : -1;
}

int sd_read_file(const char *filename, char *buffer, uint32_t size, UINT *bytes_read)
{
    FRESULT res;

    res = f_open(&fil, filename, FA_READ);

    if (res != FR_OK)
        return -1;

    res = f_read(&fil, buffer, size - 1, bytes_read);
    buffer[*bytes_read] = '\0';
    f_close(&fil);

    return (res == FR_OK) ? 0 : -1;
}

void sd_list_files(void)
{
    DIR dir;
    FILINFO fno;

    Print("\r\n=== SD FILE LIST ===\r\n");

    if (f_opendir(&dir, "/") != FR_OK)
    {
        Print("[SD] opendir FAILED\r\n");
        return;
    }

    while (f_readdir(&dir, &fno) == FR_OK)
    {
        if (fno.fname[0] == 0)
            break;

        Print("FILE: ");
        Print(fno.fname);
        Print("\r\n");
    }

    f_closedir(&dir);

    Print("====================\r\n");
}