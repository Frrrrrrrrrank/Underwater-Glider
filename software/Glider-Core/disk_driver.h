#ifndef _DISK_DRIVER_H_
#define _DISK_DRIVER_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "fatfs.h"

#include <stdint.h>

#define DISK_TEXT_BUFFER_SIZE 512U

/*
 * Writable text buffer interface requested by upper layer.
 * Populate this array, then call Disk_WriteBufferToTxt().
 */
extern char g_disk_txt_buffer[DISK_TEXT_BUFFER_SIZE];

FRESULT Disk_Mount(void);
FRESULT Disk_Unmount(void);

void Disk_SetTextBuffer(const char *text);

FRESULT Disk_WriteTextToTxt(const char *filename, const char *text, UINT *written_bytes);
FRESULT Disk_WriteBufferToTxt(const char *filename, UINT *written_bytes);

#ifdef __cplusplus
}
#endif

#endif /* _DISK_DRIVER_H_ */




