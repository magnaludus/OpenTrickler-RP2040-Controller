// Unit tests for src/learn_tuner.c edge cases the simulation does not reach.
//   cc -std=c11 -Wall -Wextra -Werror tests/tuner_unit.c src/learn_tuner.c -Isrc -lm -o tuner_unit
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "learn_tuner.h"

static tuner_env_t env(void) {
    tuner_env_t e;
    memset(&e, 0, sizeof(e));
    e.target = 26.5f;
    e.bracket = 0.06f;
    e.land_sigma = 2.0f;
    e.coarse_motor_cap = 5.0f;
    e.fine_motor_cap = 5.0f;
    e.coarse_motor_min = 0.05f;
    e.coarse_sd_floor_gr = 0.32f;
    e.handoff_cap_frac = 0.5f;
    e.time_goal_s = 7.0f;
    return e;
}

static tuner_params_t start_params(void) {
    tuner_params_t p = {.fine_max = 4.0f, .fine_min = 0.34f, .fine_kp = 1.5f,
                        .coarse_max = 1.5f, .coarse_min = 0.3f, .coarse_kp = 0.056f, .handoff = 0.57f};
    return p;
}

static void clean_throw(tuner_t * t, const tuner_env_t * e, tuner_params_t * p, int i) {
    tuner_obs_t o = {.result = TUNER_THROW_PASS, .total_s = 9.0f + 0.1f * (float) (i % 3),
                     .error_gr = (i % 2) ? 0.008f : -0.008f, .coarse_ran_long = false};
    tuner_observe(t, e, &o, p);
}

// An outside edit that lands while a candidate is live must roll the candidate back and keep the edit.
static void edit_during_probe_rolls_back_candidate(void) {
    tuner_t t;
    tuner_init(&t);
    tuner_env_t e = env();
    tuner_params_t p = start_params();
    const tuner_params_t original = p;

    for (int i = 0; i < TUNER_ARM_N; i += 1) clean_throw(&t, &e, &p, i);
    assert(t.mode == TUNER_PROBING);
    assert(p.handoff > original.handoff * 1.1f);     // the first probe widens the handoff

    p.coarse_kp = 0.15f;                              // someone edits Kp in the portal
    clean_throw(&t, &e, &p, 11);

    assert(fabsf(p.coarse_kp - 0.15f) < 1e-6f);       // the edit is kept
    assert(fabsf(p.handoff - original.handoff) < 1e-6f);  // the unverified candidate is gone
    assert(t.mode == TUNER_MEASURING);
    printf("outside edit during a probe: candidate rolled back, edit kept: passed\n");
}

// An outside edit while not probing re-anchors and keeps everything as edited.
static void edit_while_measuring_is_adopted(void) {
    tuner_t t;
    tuner_init(&t);
    tuner_env_t e = env();
    tuner_params_t p = start_params();
    for (int i = 0; i < 4; i += 1) clean_throw(&t, &e, &p, i);
    p.handoff = 1.2f;
    clean_throw(&t, &e, &p, 5);
    assert(fabsf(p.handoff - 1.2f) < 1e-6f);
    assert(t.mode == TUNER_MEASURING && t.base.n == 1);
    printf("outside edit while measuring: adopted, baseline restarts: passed\n");
}

// Garbage in must not move anything.
static void nonfinite_input_is_ignored(void) {
    tuner_t t;
    tuner_init(&t);
    tuner_env_t e = env();
    tuner_params_t p = start_params();
    clean_throw(&t, &e, &p, 0);
    tuner_params_t before = p;
    tuner_obs_t bad = {.result = TUNER_THROW_PASS, .total_s = NAN, .error_gr = 0.0f};
    tuner_observe(&t, &e, &bad, &p);
    bad.total_s = 9.0f;
    bad.error_gr = INFINITY;
    tuner_observe(&t, &e, &bad, &p);
    assert(memcmp(&before, &p, sizeof(p)) == 0);
    printf("non-finite observations ignored: passed\n");
}

int main(void) {
    edit_during_probe_rolls_back_candidate();
    edit_while_measuring_is_adopted();
    nonfinite_input_is_ignored();
    return 0;
}
