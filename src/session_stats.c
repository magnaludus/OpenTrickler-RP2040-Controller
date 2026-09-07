#include <string.h>
#include <stdio.h>
#include <math.h>
#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>

#include "session_stats.h"
#include "common.h"

static throw_record_t log_buffer[SESSION_LOG_MAX];
static size_t log_head = 0;     // next write slot
static size_t log_count = 0;    // number of valid records (<= SESSION_LOG_MAX)
static uint32_t seq_counter = 0;

static session_summary_t summary;
static SemaphoreHandle_t stats_mutex = NULL;

extern const char * http_json_header;


static uint32_t uptime_ms(void) {
    return (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
}


static void _reset_locked(void) {
    memset(&summary, 0, sizeof(summary));
    summary.min_time = 0.0f;
    summary.max_time = 0.0f;
    summary.session_start_ms = uptime_ms();
    log_head = 0;
    log_count = 0;
    seq_counter = 0;
}


void session_stats_init(void) {
    if (stats_mutex == NULL) {
        stats_mutex = xSemaphoreCreateMutex();
    }
    _reset_locked();
}


void session_stats_reset(void) {
    xSemaphoreTake(stats_mutex, portMAX_DELAY);
    _reset_locked();
    xSemaphoreGive(stats_mutex);
}


void session_stats_record(float target, float actual, float total_s, float coarse_s,
                          float bracket, throw_result_t result, uint8_t profile_idx, uint8_t mode) {
    xSemaphoreTake(stats_mutex, portMAX_DELAY);

    seq_counter += 1;

    throw_record_t * rec = &log_buffer[log_head];
    rec->seq = seq_counter;
    rec->uptime_ms = uptime_ms();
    rec->target = target;
    rec->actual = actual;
    rec->total_s = total_s;
    rec->coarse_s = coarse_s;
    rec->bracket = bracket;
    rec->result = (uint8_t) result;
    rec->profile_idx = profile_idx;
    rec->mode = mode;
    rec->_pad = 0;

    log_head = (log_head + 1) % SESSION_LOG_MAX;
    if (log_count < SESSION_LOG_MAX) {
        log_count += 1;
    }

    // Running totals
    float err = actual - target;
    summary.total += 1;
    switch (result) {
        case THROW_RESULT_PASS:  summary.pass += 1;  break;
        case THROW_RESULT_UNDER: summary.under += 1; break;
        case THROW_RESULT_OVER:  summary.over += 1;  break;
        default: break;
    }
    summary.sum_time += total_s;
    summary.sum_err += err;
    summary.sum_err_sq += (double) err * (double) err;
    if (summary.total == 1) {
        summary.min_time = total_s;
        summary.max_time = total_s;
    }
    else {
        if (total_s < summary.min_time) summary.min_time = total_s;
        if (total_s > summary.max_time) summary.max_time = total_s;
    }
    summary.last_throw_ms = rec->uptime_ms;

    xSemaphoreGive(stats_mutex);
}


const session_summary_t * session_stats_summary(void) {
    return &summary;
}


float session_stats_avg_time(void) {
    if (summary.total == 0) return 0.0f;
    return (float)(summary.sum_time / (double) summary.total);
}


float session_stats_success_rate(void) {
    if (summary.total == 0) return 0.0f;
    return 100.0f * (float) summary.pass / (float) summary.total;
}


float session_stats_err_mean(void) {
    if (summary.total == 0) return 0.0f;
    return (float)(summary.sum_err / (double) summary.total);
}


float session_stats_err_sd(void) {
    if (summary.total < 2) return 0.0f;
    double n = (double) summary.total;
    double mean = summary.sum_err / n;
    double var = (summary.sum_err_sq - n * mean * mean) / (n - 1.0);
    if (var < 0.0) var = 0.0;
    return (float) sqrt(var);
}


size_t session_stats_count(void) {
    return log_count;
}


bool session_stats_get(size_t idx, throw_record_t * out) {
    if (idx >= log_count || out == NULL) {
        return false;
    }
    // Oldest retained record sits at (head - count)
    size_t start = (log_head + SESSION_LOG_MAX - log_count) % SESSION_LOG_MAX;
    size_t slot = (start + idx) % SESSION_LOG_MAX;
    *out = log_buffer[slot];
    return true;
}


// ---------------------------------------------------------------------------
// REST
// ---------------------------------------------------------------------------

bool http_rest_session_summary(struct fs_file *file, int num_params, char *params[], char *values[]) {
    // Mappings
    // t0 (int): total throws
    // t1 (int): pass
    // t2 (int): under
    // t3 (int): over
    // t4 (float): average time s
    // t5 (float): min time s
    // t6 (float): max time s
    // t7 (float): mean error (actual - target)
    // t8 (float): error standard deviation
    // t9 (float): success rate percent
    // t10 (int): records available for export
    // t11 (int): session length in seconds
    // a0 (str): action. "reset" clears the session

    static char json_buffer[320];

    for (int idx = 0; idx < num_params; idx += 1) {
        if (strcmp(params[idx], "a0") == 0) {
            if (strcmp(values[idx], "reset") == 0) {
                session_stats_reset();
            }
        }
    }

    uint32_t session_len_s = (uptime_ms() - summary.session_start_ms) / 1000;

    snprintf(json_buffer, sizeof(json_buffer),
             "%s"
             "{\"t0\":%lu,\"t1\":%lu,\"t2\":%lu,\"t3\":%lu,"
             "\"t4\":%.2f,\"t5\":%.2f,\"t6\":%.2f,\"t7\":%.4f,\"t8\":%.4f,\"t9\":%.1f,"
             "\"t10\":%u,\"t11\":%lu}",
             http_json_header,
             (unsigned long) summary.total,
             (unsigned long) summary.pass,
             (unsigned long) summary.under,
             (unsigned long) summary.over,
             session_stats_avg_time(),
             summary.min_time,
             summary.max_time,
             session_stats_err_mean(),
             session_stats_err_sd(),
             session_stats_success_rate(),
             (unsigned) log_count,
             (unsigned long) session_len_s);

    size_t data_length = strlen(json_buffer);
    file->data = json_buffer;
    file->len = data_length;
    file->index = data_length;
    file->flags = FS_FILE_FLAGS_HEADER_INCLUDED;

    return true;
}


// Paginated log so the browser can assemble the CSV without a big buffer on the controller.
// GET /rest/session_log?from=N&n=M   (n capped at 20)
bool http_rest_session_log(struct fs_file *file, int num_params, char *params[], char *values[]) {
    static char json_buffer[2200];

    size_t from = 0;
    size_t n = 20;

    for (int idx = 0; idx < num_params; idx += 1) {
        if (strcmp(params[idx], "from") == 0) {
            from = (size_t) strtoul(values[idx], NULL, 10);
        }
        else if (strcmp(params[idx], "n") == 0) {
            n = (size_t) strtoul(values[idx], NULL, 10);
        }
    }
    if (n > 20) n = 20;

    size_t total = log_count;
    int offset = snprintf(json_buffer, sizeof(json_buffer),
                          "%s{\"total\":%u,\"from\":%u,\"rows\":[",
                          http_json_header, (unsigned) total, (unsigned) from);

    bool first = true;
    for (size_t i = 0; i < n; i += 1) {
        throw_record_t rec;
        if (!session_stats_get(from + i, &rec)) {
            break;
        }
        int written = snprintf(json_buffer + offset, sizeof(json_buffer) - offset,
                               "%s[%lu,%lu,%.3f,%.3f,%.2f,%.2f,%.3f,%u,%u,%u]",
                               first ? "" : ",",
                               (unsigned long) rec.seq,
                               (unsigned long) rec.uptime_ms,
                               rec.target,
                               rec.actual,
                               rec.total_s,
                               rec.coarse_s,
                               rec.bracket,
                               (unsigned) rec.result,
                               (unsigned) rec.profile_idx,
                               (unsigned) rec.mode);
        if (written < 0 || (size_t)(offset + written) >= sizeof(json_buffer) - 4) {
            break;  // out of room, the browser will ask for the next page
        }
        offset += written;
        first = false;
    }

    offset += snprintf(json_buffer + offset, sizeof(json_buffer) - offset, "]}");

    file->data = json_buffer;
    file->len = offset;
    file->index = offset;
    file->flags = FS_FILE_FLAGS_HEADER_INCLUDED;

    return true;
}
