#ifndef OTA_H_
#define OTA_H_

#include <stdint.h>
#include <stdbool.h>
#include "pico.h"
#include "http_rest.h"

// Firmware update over WiFi.
// The portal parses the .uf2, POSTs the raw image to /ota/upload, the image is staged in the top half of
// flash, verified by CRC32, then copied over the running image from RAM and the board reboots.

#define OTA_STAGE_OFFSET        (PICO_FLASH_SIZE_BYTES / 2u)
#define OTA_MAX_IMAGE_BYTES     ((OTA_STAGE_OFFSET < (1536u * 1024u)) ? OTA_STAGE_OFFSET : (1536u * 1024u))
#define OTA_SECTOR_BYTES        4096u

typedef enum {
    OTA_STATE_IDLE = 0,
    OTA_STATE_RECEIVING = 1,
    OTA_STATE_RECEIVED = 2,
    OTA_STATE_VERIFIED = 3,
    OTA_STATE_APPLYING = 4,
    OTA_STATE_ERROR = 5,
} ota_state_t;

#ifdef __cplusplus
extern "C" {
#endif

bool ota_init(void);
bool http_rest_ota_state(struct fs_file *file, int num_params, char *params[], char *values[]);

#ifdef __cplusplus
}
#endif

#endif  // OTA_H_
