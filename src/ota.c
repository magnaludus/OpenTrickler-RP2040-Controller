#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <FreeRTOS.h>
#include <task.h>
#include "pico/stdlib.h"
#include "pico/flash.h"
#include "hardware/flash.h"
#include "hardware/watchdog.h"
#include "hardware/structs/watchdog.h"
#include "hardware/structs/psm.h"
#include "pico/bootrom.h"
#include "boot/picoboot_constants.h"
#include "lwip/apps/httpd.h"
#include "lwip/pbuf.h"

#include "ota.h"
#include "common.h"
#include "session_version.h"


static ota_state_t ota_state = OTA_STATE_IDLE;
static uint32_t ota_expected = 0;
static uint32_t ota_received = 0;
static uint32_t ota_crc = 0;
static char ota_message[40] = "";

// One flash sector of buffered upload data
static uint8_t sector_buf[OTA_SECTOR_BYTES];
static uint32_t sector_fill = 0;
static uint32_t sector_index = 0;

static void * ota_connection = NULL;


static void set_msg(const char * s) {
    strncpy(ota_message, s, sizeof(ota_message) - 1);
    ota_message[sizeof(ota_message) - 1] = '\0';
}


// ---------------------------------------------------------------------------
// Flash helpers, run under flash_safe_execute so the other core is parked
// ---------------------------------------------------------------------------
typedef struct {
    uint32_t offset;
    const uint8_t * data;
    uint32_t len;
} flash_job_t;

static void __no_inline_not_in_flash_func(flash_write_sector_job)(void * p) {
    flash_job_t * job = (flash_job_t *) p;
    flash_range_erase(job->offset, OTA_SECTOR_BYTES);
    flash_range_program(job->offset, job->data, job->len);
}

static bool flash_write_sector(uint32_t offset, const uint8_t * data, uint32_t len) {
    flash_job_t job = { .offset = offset, .data = data, .len = len };
    int rc = flash_safe_execute(flash_write_sector_job, &job, 2000);
    return rc == PICO_OK;
}


static uint32_t crc32_flash(uint32_t offset, uint32_t len) {
    const uint8_t * p = (const uint8_t *)(XIP_BASE + offset);
    uint32_t crc = 0xFFFFFFFFu;
    for (uint32_t i = 0; i < len; i += 1) {
        crc ^= p[i];
        for (int b = 0; b < 8; b += 1) {
            crc = (crc >> 1) ^ (0xEDB88320u & (0u - (crc & 1u)));
        }
    }
    return ~crc;
}


// Does the staged image look like firmware for this board? Vector table sanity only.
static bool staged_image_plausible(uint32_t len) {
    if (len < 4096) return false;
    const uint32_t * v = (const uint32_t *)(XIP_BASE + OTA_STAGE_OFFSET);
    uint32_t sp = v[0];
    uint32_t reset = v[1];
    bool sp_ok = (sp >= 0x20000000u && sp <= 0x20082000u);
    bool reset_ok = (reset >= 0x10000000u && reset < 0x10000000u + len && (reset & 1u) == 1u);
    return sp_ok && reset_ok;
}


// ---------------------------------------------------------------------------
// Apply: copy staging over the running image. Runs entirely from RAM with the
// other core parked and interrupts off, then hits the watchdog. Never returns.
// ---------------------------------------------------------------------------
static uint8_t copy_buf[OTA_SECTOR_BYTES];

static void __no_inline_not_in_flash_func(ota_apply_job)(void * p) {
    uint32_t len = *(uint32_t *) p;
    for (uint32_t off = 0; off < len; off += OTA_SECTOR_BYTES) {
        // Pull the sector out of staging through XIP (XIP is live between flash ops)
        const volatile uint8_t * src = (const volatile uint8_t *)(XIP_BASE + OTA_STAGE_OFFSET + off);
        for (uint32_t i = 0; i < OTA_SECTOR_BYTES; i += 1) {
            copy_buf[i] = src[i];
        }
        flash_range_erase(off, OTA_SECTOR_BYTES);
        flash_range_program(off, copy_buf, OTA_SECTOR_BYTES);
    }
    // The old image is gone. Do not return into flash. Reboot from here.
    // Ask the bootrom for a normal reboot (ROM code, safe to call from RAM).
    rom_reboot(REBOOT2_FLAG_REBOOT_TYPE_NORMAL | REBOOT2_FLAG_NO_RETURN_ON_SUCCESS, 10, 0, 0);

    // Fallback: watchdog with the power domains selected, otherwise the trigger resets nothing on RP2350
    for (int i = 0; i < 8; i += 1) watchdog_hw->scratch[i] = 0;
    psm_hw->wdsel = PSM_WDSEL_BITS & ~(PSM_WDSEL_ROSC_BITS | PSM_WDSEL_XOSC_BITS);
    watchdog_hw->ctrl = WATCHDOG_CTRL_TRIGGER_BITS;
    while (true) { __asm volatile ("nop"); }
}

static void ota_apply_task(void * p) {
    static uint32_t len;
    len = (ota_received + OTA_SECTOR_BYTES - 1) & ~(OTA_SECTOR_BYTES - 1);
    vTaskDelay(pdMS_TO_TICKS(800));   // let the HTTP response go out first
    flash_safe_execute(ota_apply_job, &len, 5000);
    // Only here if the lockout failed
    ota_state = OTA_STATE_ERROR;
    set_msg("Apply failed to start");
    vTaskDelete(NULL);
}


// ---------------------------------------------------------------------------
// lwIP httpd POST hooks
// ---------------------------------------------------------------------------
err_t httpd_post_begin(void *connection, const char *uri, const char *http_request,
                       u16_t http_request_len, int content_len, char *response_uri,
                       u16_t response_uri_len, u8_t *post_auto_wnd) {
    (void) http_request; (void) http_request_len;

    if (strcmp(uri, "/ota/upload") != 0) {
        snprintf(response_uri, response_uri_len, "/404");
        return ERR_VAL;
    }
    if (content_len <= 0 || (uint32_t) content_len > OTA_MAX_IMAGE_BYTES) {
        ota_state = OTA_STATE_ERROR;
        set_msg("Bad image size");
        snprintf(response_uri, response_uri_len, "/rest/ota_state");
        return ERR_VAL;
    }
    if (ota_state == OTA_STATE_APPLYING) {
        snprintf(response_uri, response_uri_len, "/rest/ota_state");
        return ERR_VAL;
    }

    ota_connection = connection;
    ota_state = OTA_STATE_RECEIVING;
    ota_expected = (uint32_t) content_len;
    ota_received = 0;
    ota_crc = 0;
    sector_fill = 0;
    sector_index = 0;
    set_msg("Receiving");
    *post_auto_wnd = 1;
    snprintf(response_uri, response_uri_len, "/rest/ota_state");
    return ERR_OK;
}


static bool flush_sector(void) {
    if (sector_fill == 0) return true;
    if (sector_fill < OTA_SECTOR_BYTES) {
        memset(sector_buf + sector_fill, 0xFF, OTA_SECTOR_BYTES - sector_fill);
    }
    uint32_t offset = OTA_STAGE_OFFSET + sector_index * OTA_SECTOR_BYTES;
    bool ok = flash_write_sector(offset, sector_buf, OTA_SECTOR_BYTES);
    sector_index += 1;
    sector_fill = 0;
    return ok;
}


err_t httpd_post_receive_data(void *connection, struct pbuf *p) {
    if (connection != ota_connection || ota_state != OTA_STATE_RECEIVING) {
        pbuf_free(p);
        return ERR_VAL;
    }

    struct pbuf * q = p;
    while (q != NULL) {
        const uint8_t * d = (const uint8_t *) q->payload;
        uint32_t remaining = q->len;
        while (remaining > 0) {
            uint32_t space = OTA_SECTOR_BYTES - sector_fill;
            uint32_t n = (remaining < space) ? remaining : space;
            memcpy(sector_buf + sector_fill, d, n);
            sector_fill += n;
            d += n;
            remaining -= n;
            ota_received += n;
            if (sector_fill == OTA_SECTOR_BYTES) {
                if (!flush_sector()) {
                    ota_state = OTA_STATE_ERROR;
                    set_msg("Flash write failed");
                    pbuf_free(p);
                    return ERR_VAL;
                }
            }
        }
        q = q->next;
    }
    pbuf_free(p);
    return ERR_OK;
}


void httpd_post_finished(void *connection, char *response_uri, u16_t response_uri_len) {
    if (connection == ota_connection && ota_state == OTA_STATE_RECEIVING) {
        if (flush_sector() && ota_received == ota_expected) {
            ota_state = OTA_STATE_RECEIVED;
            set_msg("Staged, verify next");
        }
        else {
            ota_state = OTA_STATE_ERROR;
            set_msg("Upload incomplete");
        }
    }
    ota_connection = NULL;
    snprintf(response_uri, response_uri_len, "/rest/ota_state");
}


// ---------------------------------------------------------------------------
// REST
// ---------------------------------------------------------------------------
bool http_rest_ota_state(struct fs_file *file, int num_params, char *params[], char *values[]) {
    // a0 (str): "verify" with crc=<hex crc32 of the image> | "apply" | "reset"
    static char json_buffer[320];
    const char * action = NULL;
    uint32_t client_crc = 0;
    bool have_crc = false;

    for (int idx = 0; idx < num_params; idx += 1) {
        if (strcmp(params[idx], "a0") == 0) action = values[idx];
        else if (strcmp(params[idx], "crc") == 0) { client_crc = (uint32_t) strtoul(values[idx], NULL, 16); have_crc = true; }
    }

    if (action != NULL) {
        if (strcmp(action, "reset") == 0) {
            if (ota_state != OTA_STATE_APPLYING) {
                ota_state = OTA_STATE_IDLE;
                ota_received = 0;
                ota_expected = 0;
                set_msg("");
            }
        }
        else if (strcmp(action, "verify") == 0) {
            if (ota_state == OTA_STATE_RECEIVED && have_crc) {
                ota_crc = crc32_flash(OTA_STAGE_OFFSET, ota_received);
                if (ota_crc != client_crc) {
                    ota_state = OTA_STATE_ERROR;
                    set_msg("CRC mismatch");
                }
                else if (!staged_image_plausible(ota_received)) {
                    ota_state = OTA_STATE_ERROR;
                    set_msg("Not a firmware image");
                }
                else {
                    ota_state = OTA_STATE_VERIFIED;
                    set_msg("Verified, ready to apply");
                }
            }
        }
        else if (strcmp(action, "apply") == 0) {
            if (ota_state == OTA_STATE_VERIFIED) {
                ota_state = OTA_STATE_APPLYING;
                set_msg("Flashing, reboots in ~10 s");
                xTaskCreate(ota_apply_task, "OTA Apply", 1024, NULL, configMAX_PRIORITIES - 2, NULL);
            }
        }
    }

    snprintf(json_buffer, sizeof(json_buffer),
             "%s{\"st\":%d,\"rx\":%lu,\"exp\":%lu,\"crc\":\"%08lx\",\"msg\":\"%s\",\"ver\":\"%s\",\"max\":%lu}",
             http_json_header,
             (int) ota_state,
             (unsigned long) ota_received,
             (unsigned long) ota_expected,
             (unsigned long) ota_crc,
             ota_message,
             SESSION_BUILD_TAG,
             (unsigned long) OTA_MAX_IMAGE_BYTES);

    size_t len = strlen(json_buffer);
    file->data = json_buffer; file->len = len; file->index = len;
    file->flags = FS_FILE_FLAGS_HEADER_INCLUDED;
    return true;
}


bool ota_init(void) {
    ota_state = OTA_STATE_IDLE;
    return true;
}
