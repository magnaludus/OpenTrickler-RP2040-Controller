#ifndef SESSION_STATS_H_
#define SESSION_STATS_H_

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include "http_rest.h"

// Number of throws kept in RAM for the CSV export. 400 x 32 bytes = 12.8 KB.
// The running totals (count, pass, avg time, error SD) are not limited by this.
#define SESSION_LOG_MAX             400

typedef enum {
    THROW_RESULT_PASS = 0,
    THROW_RESULT_UNDER = 1,
    THROW_RESULT_OVER = 2,
} throw_result_t;

typedef struct {
    uint32_t seq;            // 1 based throw number within the session
    uint32_t uptime_ms;      // controller uptime when the throw completed
    float target;            // set point
    float actual;            // settled weight after the cup was read
    float total_s;           // trickle time, start to stop
    float coarse_s;          // time the coarse trickler was running
    float bracket;           // acceptance bracket in effect (+/- grains)
    uint8_t result;          // throw_result_t
    uint8_t profile_idx;
    uint8_t mode;            // bracket mode (0 normal, 1 match)
    uint8_t _pad;
} throw_record_t;

typedef struct {
    uint32_t total;
    uint32_t pass;
    uint32_t under;
    uint32_t over;

    double sum_time;
    double sum_err;
    double sum_err_sq;
    float min_time;
    float max_time;

    uint32_t session_start_ms;
    uint32_t last_throw_ms;
} session_summary_t;


#ifdef __cplusplus
extern "C" {
#endif

void session_stats_init(void);
void session_stats_reset(void);
void session_stats_record(float target, float actual, float total_s, float coarse_s,
                          float bracket, throw_result_t result, uint8_t profile_idx, uint8_t mode);

const session_summary_t * session_stats_summary(void);
float session_stats_avg_time(void);       // seconds, 0 if no throws
float session_stats_success_rate(void);   // 0..100 percent, 0 if no throws
float session_stats_err_mean(void);       // grains, signed (actual - target)
float session_stats_err_sd(void);         // grains
size_t session_stats_count(void);         // number of records available for export
bool session_stats_get(size_t idx, throw_record_t * out);  // idx 0 = oldest retained

// REST
bool http_rest_session_summary(struct fs_file *file, int num_params, char *params[], char *values[]);
bool http_rest_session_log(struct fs_file *file, int num_params, char *params[], char *values[]);

#ifdef __cplusplus
}
#endif

#endif  // SESSION_STATS_H_
