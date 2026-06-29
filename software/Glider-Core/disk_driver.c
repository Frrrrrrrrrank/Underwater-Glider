/*
 * disk_driver.c
 *
 * SDMMC1 + FatFs helper APIs for creating/writing txt files on SD card.
 */

#include "disk_driver.h"

#include <string.h>

char g_disk_txt_buffer[DISK_TEXT_BUFFER_SIZE] = {0};

static uint32_t s_sd_mounted = 0U;

static UINT Disk_SafeStrnlen(const char *text, UINT max_len)
{
    UINT len = 0U;

    if (text == NULL)
    {
        return 0U;
    }

    while ((len < max_len) && (text[len] != '\0'))
    {
        len++;
    }

    return len;
}

FRESULT Disk_Mount(void)
{
    FRESULT res;

    if (s_sd_mounted != 0U)
    {
        return FR_OK;
    }

    res = f_mount(&SDFatFS, (TCHAR const *)SDPath, 1U);
    if (res == FR_OK)
    {
        s_sd_mounted = 1U;
    }

    return res;
}

FRESULT Disk_Unmount(void)
{
    FRESULT res;

    res = f_mount(NULL, (TCHAR const *)SDPath, 1U);
    if (res == FR_OK)
    {
        s_sd_mounted = 0U;
    }

    return res;
}

void Disk_SetTextBuffer(const char *text)
{
    UINT len;

    memset(g_disk_txt_buffer, 0, sizeof(g_disk_txt_buffer));

    if (text == NULL)
    {
        return;
    }

    len = Disk_SafeStrnlen(text, (UINT)(DISK_TEXT_BUFFER_SIZE - 1U));
    if (len > 0U)
    {
        memcpy(g_disk_txt_buffer, text, len);
    }
}

FRESULT Disk_WriteTextToTxt(const char *filename, const char *text, UINT *written_bytes)
{
    FRESULT res;
    FIL file;
    UINT req_len;
    UINT bw = 0U;

    if (written_bytes != NULL)
    {
        *written_bytes = 0U;
    }

    if ((filename == NULL) || (filename[0] == '\0') || (text == NULL))
    {
        return FR_INVALID_OBJECT;
    }

    res = Disk_Mount();
    if (res != FR_OK)
    {
        return res;
    }

    res = f_open(&file, filename, FA_WRITE | FA_CREATE_ALWAYS);
    if (res != FR_OK)
    {
        return res;
    }

    req_len = Disk_SafeStrnlen(text, 0xFFFFU);
    if (req_len > 0U)
    {
        res = f_write(&file, text, req_len, &bw);
        if ((res == FR_OK) && (bw != req_len))
        {
            res = FR_DISK_ERR;
        }
    }

    if (res == FR_OK)
    {
        res = f_sync(&file);
    }

    (void)f_close(&file);

    if ((res == FR_OK) && (written_bytes != NULL))
    {
        *written_bytes = bw;
    }

    return res;
}

FRESULT Disk_WriteBufferToTxt(const char *filename, UINT *written_bytes)
{
    return Disk_WriteTextToTxt(filename, g_disk_txt_buffer, written_bytes);
}




