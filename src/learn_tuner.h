#ifndef LEARN_TUNER_H_
#define LEARN_TUNER_H_

// Live profile tuner.
//
// Pure C with no RTOS, EEPROM or scale dependencies, so the exact code that runs on the Pico also
// runs in the host simulation (tests/tuner_sim.c). It sees one record per finished throw and edits
// the profile it is handed.
//
// What it does, in order of authority:
//
//   1. Safety. Misses are judged on the real landing error, never on a model. The landing speed
//      is held to the Landing Sigma margin, the aim point is trimmed to centre the spread, and a
//      probe that misses is thrown out.
//   2. Speed. Handoff, fine Kp, coarse speed and coarse taper are probed one at a time in small steps. A step
//      is kept only if the measured throw time got better and the throws stayed accurate. There is
//      no physics model in the loop to be wrong: the objective is the stopwatch.
//   3. Drift. Once nothing improves it stops probing and watches. Slower throws or a run of misses
//      (a new powder lot, humidity) reopens the search.

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define TUNER_ERR_WINDOW    20      // throws of landing error the margin is judged on
#define TUNER_ERR_MIN_N     8       // samples before the margin means anything
#define TUNER_ARM_N         10      // throws per arm of a probe (baseline vs trial)
#define TUNER_MONITOR_N     16      // throws per drift check once converged
#define TUNER_KNOBS         4

typedef struct {
    float fine_max;
    float fine_min;
    float fine_kp;
    float coarse_max;
    float coarse_min;
    float coarse_kp;
    float handoff;
} tuner_params_t;

// Things the tuner has to respect but does not own. Passed on every call, so a target weight
// change or a motor limit change takes effect on the next throw.
typedef struct {
    float target;
    float bracket;
    float land_sigma;               // sigmas of landing spread that must fit inside the bracket
    float coarse_motor_cap;
    float fine_motor_cap;
    float coarse_motor_min;
    float coarse_sd_floor_gr;       // 3 sigma of the coarse stop scatter from the Learn fit
    float handoff_cap_frac;         // the bulk always carries at least 1 - this share of the charge
    float time_goal_s;              // throws already at or under this are left alone; 0 = always search
    float handoff_floor_gr;         // narrowest handoff the tuner may set; 0 = the default 0.30
    bool aggressive;                // push harder: spend more margin, probe further, tolerate more misses
} tuner_env_t;

typedef enum {
    TUNER_THROW_PASS = 0,
    TUNER_THROW_OVER = 1,
    TUNER_THROW_UNDER = 2,
} tuner_throw_result_t;

typedef struct {
    tuner_throw_result_t result;
    float total_s;
    float error_gr;                 // settled weight - target
    bool coarse_ran_long;           // the bulk carried past its handoff on this throw
} tuner_obs_t;

typedef enum {
    TUNER_MEASURING = 0,            // collecting the baseline the next probe is judged against
    TUNER_PROBING = 1,              // a candidate is live
    TUNER_CONVERGED = 2,            // nothing left to gain, watching for drift
} tuner_mode_t;

typedef struct {
    uint8_t n;
    uint8_t misses;
    float sum;
    float sum_sq;
    float err_sum;                  // landing error of the same throws, for the spread
    float err_sum_sq;
} tuner_arm_t;

typedef struct {
    bool init;
    tuner_params_t anchor;          // the profile as first seen: hard bounds are relative to this
    tuner_params_t good;            // last accepted parameters
    tuner_params_t seen;            // what the tuner left behind, to detect outside edits

    float err[TUNER_ERR_WINDOW];
    uint8_t err_count;
    uint8_t err_next;
    float trim_gr;
    uint8_t land_tick;
    uint32_t miss_bits;             // one bit per recent throw, newest in bit 0, set = miss

    tuner_mode_t mode;
    uint8_t knob;
    int8_t dir[TUNER_KNOBS];
    float step[TUNER_KNOBS];
    bool done[TUNER_KNOBS];
    uint8_t wait;                   // throws to sit out before trying a gated knob again
    tuner_arm_t base;
    tuner_arm_t trial;
    float monitor_ref_s;
    float start_mean_s;             // throw time of the profile as first seen
    uint8_t start_misses;
    bool start_known;

    uint16_t probes;
    uint16_t accepted;
    uint16_t throws;
    float last_gain_s;              // gain of the most recent accepted probe
} tuner_t;

void tuner_init(tuner_t * t);

// Call after every throw. `p` is the live profile on entry and the tuned profile on exit.
void tuner_observe(tuner_t * t, const tuner_env_t * env, const tuner_obs_t * obs, tuner_params_t * p);

// Aim-point correction in grains for the fine tube and the stop check.
float tuner_trim(const tuner_t * t);

// Short human readable state for the portal, "Tuned 7.2s" and the like.
void tuner_status(const tuner_t * t, char * out, size_t out_len);

// Margin of the recent landing spread to the nearer bracket edge, in sigma. 0 until there is data.
float tuner_margin_sigma(const tuner_t * t, float bracket);

#ifdef __cplusplus
}
#endif

#endif  // LEARN_TUNER_H_
