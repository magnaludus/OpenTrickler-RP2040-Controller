#ifndef LEARN_MODE_H_
#define LEARN_MODE_H_

#include <stdint.h>
#include <stdbool.h>
#include "http_rest.h"

// Learn Powder: characterise a powder on both tubes, fit a profile, confirm it.
// No cup handling. The pan stays on the scale and the scale is re-zeroed after each throw settles.

#define LEARN_THROWS_PER_PHASE      12
#define LEARN_COARSE_SPEED_LEVELS   4       // 12 throws = 4 speeds x 3
#define LEARN_FINE_SPEED_LEVELS     3       // 12 throws = 3 speeds x 4
#define LEARN_CONFIRM_THROWS        5

typedef enum {
    LEARN_STATE_IDLE = 0,
    LEARN_STATE_WAIT_FOR_ZERO = 1,
    LEARN_STATE_COARSE = 2,
    LEARN_STATE_FINE = 3,
    LEARN_STATE_FIT = 4,
    LEARN_STATE_CONFIRM = 5,
    LEARN_STATE_DONE = 6,
    LEARN_STATE_ABORTED = 7,
    LEARN_STATE_ERROR = 8,
    LEARN_STATE_EMPTY_CUP = 9,      // waiting on the user to dump the cup and put it back
} learn_state_t;

typedef struct {
    float speed_rps;
    float time_s;           // motor start to stop
    float stop_weight;      // reading when the motor was told to stop
    float settled_weight;   // reading after the scale settled
    float flow_gps;         // stop_weight / time_s
    float tail;             // settled_weight - stop_weight
    float rate_at_stop;     // how fast the reading was climbing when the motor stopped
    float lag_s;            // tail / rate_at_stop, the scale lag measured on this throw
    float dead_time_s;      // motor start to first movement on the scale
} learn_throw_t;

typedef struct {
    // Fitted values
    float coarse_tail_per_rps;      // tail grows with speed (scale lag x flow)
    float fine_tail_per_rps;
    float coarse_lag_s;             // tail per rps / flow per rps = how far behind the scale reads
    float fine_lag_s;
    float lag_used_s;               // what gets written to the charge mode lag compensation
    float lag_sd_s;                 // spread across the 24 throws
    float dead_time_s;              // mean dead time
    bool predict_used;
    float coarse_k;                 // gr/s per rps
    float coarse_tail_at_max;       // mean tail at the chosen coarse max
    float coarse_tail_sd_at_max;
    float fine_k;                   // gr/s per rps
    float fine_tail_at_max;
    float fine_tail_sd_at_max;

    // Profile proposal
    float coarse_min_rps;
    float coarse_max_rps;
    float coarse_kp;
    float fine_min_rps;
    float fine_max_rps;
    float fine_kp;
    float fine_taper_gr;            // fine runs at max until this close, then ramps to min
    float coarse_stop_threshold;

    // Predicted throw at the confirm target with the fitted profile
    float predicted_coarse_s;
    float predicted_fine_s;
    float predicted_total_s;
    bool meets_time_goal;

    // Confirmation
    uint8_t confirm_total;
    uint8_t confirm_pass;
    float confirm_avg_time;
    uint8_t confirm_rounds;         // rounds run, 1 if the first set met the success target
    bool confirm_met;
} learn_result_t;

#define EEPROM_LEARN_CONFIG_REV     4
#define LEARN_MAX_CONFIRM_ROUNDS    3

typedef struct {
    uint16_t learn_config_rev;

    // Targets per throw
    float coarse_target;            // default 8.0
    float fine_target;              // default 1.75
    float coarse_speed_ceiling;     // highest coarse speed the ladder will use (rps)
    float fine_speed_ceiling;       // highest fine speed the ladder will use (rps)
    float confirm_target;           // charge weight for the confirmation throws
    uint8_t confirm_throws;         // default 5
    float time_goal_s;              // upper limit on throw time at the confirm weight, default 8.5
    float cup_capacity_gr;          // powder the cup can hold before it has to be dumped, default 250
    float min_success_pct;          // confirmation has to hit this or the profile backs off and reruns, default 95

    // How much cushion the fitted coarse handoff carries over the coarse tube's own measured stop
    // scatter, as a multiple of that 3 sigma figure. The fit hands the bulk as much of the charge as
    // it can and this is what holds it back, so it is the dial between a tight handoff and margin
    // against an overthrow. 1.0 = bare 3 sigma, 1.5 = default, higher = more conservative.
    float coarse_stop_safety;

    // How many standard deviations of fine landing error have to fit inside the bracket, which is
    // what caps the landing speed. The coarse side already works in 3 sigma; this started life as a
    // bare 1 sigma, which let the fine tube land about three times too fast - roughly a third of
    // throws fell outside the bracket by arithmetic alone. 2.0 is measured-good on tested hardware.
    float land_sigma;
} learn_config_t;

typedef struct {
    learn_config_t config;
    learn_state_t state;
    uint8_t throw_idx;              // within the current phase
    float current_speed;
    learn_throw_t coarse[LEARN_THROWS_PER_PHASE];
    learn_throw_t fine[LEARN_THROWS_PER_PHASE];
    learn_result_t result;
    float cup_load_gr;              // powder in the cup since it was last emptied
    bool result_valid;
    bool applied_to_profile;
    char message[32];
} learn_mode_t;


#ifdef __cplusplus
extern "C" {
#endif

bool learn_mode_init(void);
bool learn_mode_config_save(void);
uint8_t learn_mode_menu(void);              // runs the whole routine, returns the form id to go back to
bool learn_mode_apply_to_profile(void);     // write the fitted values into the selected profile (RAM)

// REST
bool http_rest_learn_state(struct fs_file *file, int num_params, char *params[], char *values[]);
bool http_rest_learn_config(struct fs_file *file, int num_params, char *params[], char *values[]);
bool http_rest_learn_throws(struct fs_file *file, int num_params, char *params[], char *values[]);

extern learn_mode_t learn_mode;

#ifdef __cplusplus
}
#endif

#endif  // LEARN_MODE_H_
