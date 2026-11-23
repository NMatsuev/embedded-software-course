/* USER CODE BEGIN DECL */
#include <string.h>
#include "ff_gen_drv.h"
#include "Sd_spi.h"
#include "main.h"

static volatile DSTATUS Stat = STA_NOINIT;


/* USER CODE END DECL */

DSTATUS USER_initialize (BYTE pdrv);
DSTATUS USER_status (BYTE pdrv);
DRESULT USER_read (BYTE pdrv, BYTE *buff, DWORD sector, UINT count);
#if _USE_WRITE == 1
  DRESULT USER_write (BYTE pdrv, const BYTE *buff, DWORD sector, UINT count);
#endif
#if _USE_IOCTL == 1
  DRESULT USER_ioctl (BYTE pdrv, BYTE cmd, void *buff);
#endif

Diskio_drvTypeDef  USER_Driver =
{
  USER_initialize,
  USER_status,
  USER_read,
#if  _USE_WRITE
  USER_write,
#endif
#if  _USE_IOCTL == 1
  USER_ioctl,
#endif
};


DSTATUS USER_initialize(BYTE pdrv)
{
    if (pdrv != 0)
        return STA_NOINIT;

    Stat = STA_NOINIT;

    if (sd_init() != 0)
    {
        Stat = STA_NOINIT;
        return Stat;
    }

    Stat = 0;
    return Stat;
}


DSTATUS USER_status(BYTE pdrv)
{
    if (pdrv != 0)
        return STA_NOINIT;

    return Stat;
}

DRESULT USER_read (BYTE pdrv, BYTE *buff, DWORD sector, UINT count)
{
    if (pdrv != 0 || !count) return RES_PARERR;
    if (Stat & STA_NOINIT) return RES_NOTRDY;

    for (UINT i = 0; i < count; i++)
    {
        if (sd_read_block(buff + (i * 512), sector + i) != 0)
            return RES_ERROR;
    }
    return RES_OK;
}

#if _USE_WRITE == 1
DRESULT USER_write (BYTE pdrv, const BYTE *buff, DWORD sector, UINT count)
{
    if (pdrv != 0 || !count) return RES_PARERR;
    if (Stat & STA_NOINIT) return RES_NOTRDY;

    for (UINT i = 0; i < count; i++)
    {
        if (sd_write_block(buff + (i * 512), sector + i) != 0)
            return RES_ERROR;
    }
    return RES_OK;
}
#endif

#if _USE_IOCTL == 1
DRESULT USER_ioctl (BYTE pdrv, BYTE cmd, void *buff)
{
    DRESULT res = RES_ERROR;

    if (pdrv != 0) return RES_PARERR;
    if (Stat & STA_NOINIT) return RES_NOTRDY;

    switch (cmd)
    {
        case CTRL_SYNC:
            res = RES_OK;
            break;

        case GET_SECTOR_COUNT:
            *(DWORD*)buff = 0;
            res = RES_OK;
            break;

        case GET_SECTOR_SIZE:
            *(WORD*)buff = 512;
            res = RES_OK;
            break;

        case GET_BLOCK_SIZE:
            *(DWORD*)buff = 1;
            res = RES_OK;
            break;

        default:
            res = RES_PARERR;
            break;
    }
    return res;
}
#endif