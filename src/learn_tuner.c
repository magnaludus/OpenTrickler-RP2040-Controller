#include "learn_tuner.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

// Margin precondition for a step that makes a throw riskier, as a share of the Landing Sigma.
#ifndef TUNER_RISKY_MARGIN_FRAC
#define TUNER_RISKY_MARGIN_FRAC 1.25f
#endif
#define RISKY_MARGIN_FRAC   TUNER_RISKY_MARGIN_FRAC
// Margin a probe has to hold, same units, if it logged a single miss.
#define PROBE_MARGIN_FRAC   0.75f

#define STEP_START          0.25f   // first probe size, as a fraction of the knob
#define STEP_MIN            0.06f   // below this the knob counts as converged
#define STEP_MAX            0.50f
#define STEP_REOPEN         0.15f   // probe size after drift reopens the search

#ifndef TUNER_LAND_DEADBAND
#define TUNER_LAND_DEADBAND 1.15f   // leave the landing speed alone while the margin is within this ratio
#endif
#define LAND_EVERY_URGENT   3       // same, when the margin is below 70% of what was asked for
#define LAND_EVERY          6       // passes between landing speed adjustments
#define LAND_STEP           1.25f   // most the landing speed moves in one adjustment
#define TRIM_GAIN           0.15f   // share of the measured bias taken out per throw
#define TRIM_LIMIT          0.50f   // aim trim limit as a fraction of the bracket

enum { KNOB_HANDOFF = 0, KNOB_FINE_KP = 1, KNOB_COARSE_MAX = 2, KNOB_COARSE_TAPER = 3 };


// How hard the tuner pushes. Normal is the behaviour that was tested; Aggressive spends more of the
// accuracy margin, tolerates more misses while probing, and lets knobs travel further from the fit.
typedef struct {
    float risky_margin;     // spare margin (x Landing Sigma) a risky probe needs before it may run
    uint8_t reject_misses;  // misses in one trial that throw the probe out
    float min_gain_s;       // least time gain that keeps a probe
    float bound_mul;        // stretches the upper end of every knob's allowed range
    float step_mul;         // scales probe step sizes
} tuner_policy_t;

static tuner_policy_t policy_for(const tuner_env_t * env) {
    if (env->aggressive) {
        tuner_policy_t a = {0.90f, 3, 0.20f, 1.5f, 1.25f};
        return a;
    }
    tuner_policy_t n = {RISKY_MARGIN_FRAC, 2, 0.30f, 1.0f, 1.0f};
    return n;
}


static float clampf(float v, float lo, float hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}


static bool params_differ(const tuner_params_t * a, const tuner_params_t * b) {
    return fabsf(a->fine_max - b->fine_max) > 1e-4f ||
           fabsf(a->fine_min - b->fine_min) > 1e-4f ||
           fabsf(a->fine_kp - b->fine_kp) > 1e-4f ||
           fabsf(a->coarse_max - b->coarse_max) > 1e-4f ||
           fabsf(a->coarse_min - b->coarse_min) > 1e-4f ||
           fabsf(a->coarse_kp - b->coarse_kp) > 1e-4f ||
           fabsf(a->handoff - b->handoff) > 1e-4f;
}


// Misses among the last `n` throws. One over is ordinary at a 2 sigma bracket (about 2% by
// arithmetic alone), so backing off waits for a cluster rather than reacting to each.
static int recent_misses(const tuner_t * t, int n) {
    uint32_t bits = t->miss_bits & ((n >= 32) ? 0xFFFFFFFFu : ((1u << n) - 1u));
    int c = 0;
    while (bits) { c += (int) (bits & 1u); bits >>= 1; }
    return c;
}


static void arm_reset(tuner_arm_t * a) {
    memset(a, 0, sizeof(*a));
}


static void arm_add(tuner_arm_t * a, float v, float err, bool miss) {
    // Keep a long wait from growing an arm without bound: past 4 arms' worth, halve it, so the
    // oldest throws fade out rather than the newest being drowned
    if (a->n >= 4 * TUNER_ARM_N) {
        a->n = (uint8_t) (a->n / 2);
        a->misses = (uint8_t) (a->misses / 2);
        a->sum *= 0.5f; a->sum_sq *= 0.5f; a->err_sum *= 0.5f; a->err_sum_sq *= 0.5f;
    }
    a->n += 1;
    a->sum += v;
    a->sum_sq += v * v;
    a->err_sum += err;
    a->err_sum_sq += err * err;
    if (miss) a->misses += 1;
}


// Landing spread of an arm, as a standard deviation about zero error: a bias counts against it,
// which is what a bracket sees.
static float arm_err_rms(const tuner_arm_t * a) {
    return (a->n > 0) ? sqrtf(a->err_sum_sq / (float) a->n) : 0.0f;
}


static float arm_mean(const tuner_arm_t * a) {
    return (a->n > 0) ? a->sum / (float) a->n : 0.0f;
}


static float arm_var(const tuner_arm_t * a) {
    if (a->n < 2) return 0.0f;
    float mean = arm_mean(a);
    float var = (a->sum_sq - (float) a->n * mean * mean) / (float) (a->n - 1);
    return (var > 0.0f) ? var : 0.0f;
}


// Landing error centre and spread over the recent window. Both are winsorized at SD_CLIP brackets:
// a throw far outside the bracket is evidence about the bulk or a bad probe as much as about the
// landing, and one such throw must not be able to swing the landing speed law on its own. The clip
// is wide enough that the tail still counts - misses live in the tail, and a spread that ignores it
// reads as comfortable while the miss rate climbs.
#ifndef TUNER_SD_CLIP
#define TUNER_SD_CLIP       2.0f
#endif

static void window_stats(const tuner_t * t, float bracket, float * mean, float * sd) {
    double sum = 0.0, sum_sq = 0.0;
    double clip = (double) TUNER_SD_CLIP * (double) bracket;
    int n = t->err_count;
    for (int i = 0; i < n; i += 1) {
        double v = t->err[i];
        if (v > clip) v = clip;
        if (v < -clip) v = -clip;
        sum += v;
        sum_sq += v * v;
    }
    double m = (n > 0) ? sum / n : 0.0;
    double var = (n > 1) ? (sum_sq - n * m * m) / (n - 1) : 0.0;
    *mean = (float) m;
    *sd = (var > 0.0) ? (float) sqrt(var) : 0.0f;
}


float tuner_margin_sigma(const tuner_t * t, float bracket) {
    if (t->err_count < TUNER_ERR_MIN_N || bracket <= 0.0f) return 0.0f;
    float mean, sd;
    window_stats(t, bracket, &mean, &sd);
    float reach = bracket - fabsf(mean);
    if (reach < 0.0f) reach = 0.0f;
    if (sd < 1e-4f) return 99.0f;
    return reach / sd;
}


float tuner_trim(const tuner_t * t) {
    return t->init ? t->trim_gr : 0.0f;
}


void tuner_init(tuner_t * t) {
    memset(t, 0, sizeof(*t));
}


// Hard limits. Everything the tuner writes goes through here, so no sequence of probes, nudges or
// outside edits can walk a knob out of the range it is allowed to live in.
static void clamp_params(const tuner_t * t, const tuner_env_t * env, tuner_params_t * p) {
    const tuner_params_t * a = &t->anchor;
    float bm = policy_for(env).bound_mul;

    if (a->fine_kp > 0.0f) p->fine_kp = clampf(p->fine_kp, 0.4f * a->fine_kp, 3.0f * bm * a->fine_kp);
    if (a->fine_min > 0.0f) p->fine_min = clampf(p->fine_min, 0.5f * a->fine_min, 2.0f * bm * a->fine_min);
    if (p->fine_min > p->fine_max) p->fine_min = p->fine_max;

    if (a->coarse_max > 0.0f) {
        float hi = 2.0f * bm * a->coarse_max;
        if (env->coarse_motor_cap > 0.0f && hi > env->coarse_motor_cap) hi = env->coarse_motor_cap;
        float lo = 0.5f * a->coarse_max;
        if (lo > hi) lo = hi;
        float before = p->coarse_max;
        p->coarse_max = clampf(p->coarse_max, lo, hi);
        // Keep the taper window (max / Kp) when the clamp moved the speed
        if (before > 0.0f && fabsf(p->coarse_max - before) > 1e-6f) p->coarse_kp *= p->coarse_max / before;
    }
    if (a->coarse_kp > 0.0f) p->coarse_kp = clampf(p->coarse_kp, 0.25f * a->coarse_kp, 12.0f * a->coarse_kp);
    if (p->coarse_min < env->coarse_motor_min) p->coarse_min = env->coarse_motor_min;
    if (p->coarse_min > p->coarse_max) p->coarse_min = p->coarse_max;

    float h_hi = env->handoff_cap_frac * env->target;
    // Hard floor on the handoff: a fixed minimum, and the coarse stop scatter the Learn fit measured
    // times the style's multiple (the adapter folds that multiple into coarse_sd_floor_gr)
    float h_lo = fmaxf(env->handoff_floor_gr > 0.0f ? env->handoff_floor_gr : 0.30f, env->coarse_sd_floor_gr);
    if (h_hi < h_lo) h_lo = h_hi;
    if (h_hi > 0.0f) p->handoff = clampf(p->handoff, h_lo, h_hi);
}


static void anchor_to(tuner_t * t, const tuner_params_t * p) {
    memset(t, 0, sizeof(*t));
    t->init = true;
    t->anchor = *p;
    t->good = *p;
    t->seen = *p;
    t->mode = TUNER_MEASURING;
    for (int k = 0; k < TUNER_KNOBS; k += 1) {
        t->dir[k] = +1;
        t->step[k] = STEP_START;
    }
    t->step[KNOB_FINE_KP] = 0.20f;
    t->step[KNOB_COARSE_MAX] = 0.12f;
    t->step[KNOB_COARSE_TAPER] = 0.40f;
}


static bool all_done(const tuner_t * t) {
    for (int k = 0; k < TUNER_KNOBS; k += 1) {
        if (!t->done[k]) return false;
    }
    return true;
}


// A step that makes the throw riskier is only worth trying while the landing spread still has room.
static bool is_risky(int knob, int dir) {
    switch (knob) {
        case KNOB_HANDOFF: return dir < 0;      // less for the slow tube to do
        case KNOB_FINE_KP: return dir > 0;      // harder push into the target
        default: return dir > 0;                // faster bulk, or a steeper bulk taper
    }
}


static void apply_knob(tuner_params_t * p, int knob, float factor) {
    switch (knob) {
        case KNOB_HANDOFF:
            p->handoff *= factor;
            break;
        case KNOB_FINE_KP:
            p->fine_kp *= factor;
            break;
        case KNOB_COARSE_TAPER:
            // Kp is max / window, so a bigger Kp is a shorter window: the bulk runs flat out for
            // longer and ramps down harder into its stop
            p->coarse_kp *= factor;
            break;
        default: {
            float old = p->coarse_max;
            p->coarse_max *= factor;
            // Keep the coarse taper window the same width as the speed moves
            if (old > 0.0f) p->coarse_kp *= p->coarse_max / old;
            break;
        }
    }
}


// Try to put a candidate live. Walks the knobs from the current one until one has a legal, allowed
// step. Returns true if a probe is now running.
static bool start_probe(tuner_t * t, const tuner_env_t * env, tuner_params_t * p) {
    if (!t->start_known && t->base.n >= TUNER_ARM_N) {
        t->start_known = true;
        t->start_mean_s = arm_mean(&t->base);
        t->start_misses = t->base.misses;
        // Throws that already meet the user's time goal need no search
        if (env->time_goal_s > 0.0f && t->start_mean_s <= env->time_goal_s) {
            for (int k = 0; k < TUNER_KNOBS; k += 1) t->done[k] = true;
            t->monitor_ref_s = t->start_mean_s;
            return false;
        }
    }
    float margin = tuner_margin_sigma(t, env->bracket);
    tuner_policy_t pol = policy_for(env);
    bool margin_ok = (margin >= pol.risky_margin * env->land_sigma);

    for (int tries = 0; tries < TUNER_KNOBS; tries += 1) {
        int k = (t->knob + tries) % TUNER_KNOBS;
        if (t->done[k]) continue;
        int dir = t->dir[k];
        if (is_risky(k, dir) && !margin_ok) continue;

        tuner_params_t cand = t->good;
        apply_knob(&cand, k, 1.0f + (float) dir * t->step[k] * pol.step_mul);
        clamp_params(t, env, &cand);
        if (!params_differ(&cand, &t->good)) {
            // Pinned against a limit in this direction: nowhere left to go that way
            t->dir[k] = (int8_t) -dir;
            t->step[k] *= 0.6f;
            if (t->step[k] < STEP_MIN) t->done[k] = true;
            continue;
        }

        t->knob = (uint8_t) k;
        *p = cand;
        arm_reset(&t->trial);
        t->mode = TUNER_PROBING;
        t->probes += 1;
        return true;
    }
    return false;
}


typedef enum {
    VERDICT_KEEP,
    VERDICT_REVERT,
} verdict_t;


// Decide whether a finished probe is kept. The baseline arm is the throws that ran on the last
// accepted parameters, the trial arm is the candidate's. `margin_sigma` is the recent landing
// spread to the nearer bracket edge, `land_sigma` is what the user asked for.
//
// Keep a probe only if it was clearly faster than noise and did not miss more than chance allows.
// The throw time SD on tested hardware is about 0.9 s, so with 10 throws an arm the standard error
// of the difference is ~0.4 s: demanding a gain well past the noise stops the search from drifting
// on luck, at the price of not resolving gains smaller than ~0.3 s.
//
// TODO(human): this is the policy knob. Tighten it for a more conservative tuner (more evidence,
// zero tolerated misses), loosen it to converge sooner at some risk of drift. The numbers below are
// defaults that held up in simulation, not truths about your hardware.
static verdict_t probe_verdict(const tuner_arm_t * base, const tuner_arm_t * trial,
                               float margin_sigma, float land_sigma, const tuner_policy_t * pol) {
    // Chance alone gives ~5% of throws outside a 2 sigma bracket, so one miss in 10 is ordinary
    // and two is not. A single miss is only forgiven if the spread still has its margin.
    if (trial->misses >= pol->reject_misses) return VERDICT_REVERT;
    if (trial->misses + 1 == pol->reject_misses && trial->misses > 0 && margin_sigma < PROBE_MARGIN_FRAC * land_sigma) return VERDICT_REVERT;

    // A faster throw that lands wider is not a free win: the landing speed law would claw the
    // margin back by slowing the landing, and the time with it. Judge the spread here, where the
    // cause is still attached. Ten throws give the spread to about +-25%, hence the slack.
    float base_rms = arm_err_rms(base);
    if (base_rms > 1e-4f && arm_err_rms(trial) > 1.30f * base_rms) return VERDICT_REVERT;

    float gain = arm_mean(base) - arm_mean(trial);
    float se = sqrtf(arm_var(base) / (float) base->n + arm_var(trial) / (float) trial->n);
    float needed = fmaxf(pol->min_gain_s, 1.0f * se);
    return (gain > needed) ? VERDICT_KEEP : VERDICT_REVERT;
}


// Settle into watching. Before doing so, make sure the search actually earned its keep: the profile
// as first seen was measured for a full arm, so if the one the search ended on is not clearly
// better than that, the original goes back. A search on a profile that was already near its best can
// only wander, and a tuner must not leave a good profile worse than it found it.
static void enter_monitor(tuner_t * t, tuner_params_t * p) {
    if (t->start_known && params_differ(&t->good, &t->anchor) && t->base.n >= TUNER_ARM_N / 2) {
        float now = arm_mean(&t->base);
        bool clearly_faster = (t->start_mean_s - now) > 0.40f;
        bool no_worse_misses = (t->base.misses * 100 / (t->base.n ? t->base.n : 1)) <=
                               (t->start_misses * 100 / TUNER_ARM_N) + 10;
        if (!(clearly_faster && no_worse_misses)) {
            *p = t->anchor;
            t->good = t->anchor;
        }
    }
    t->mode = TUNER_CONVERGED;
    arm_reset(&t->base);
}


static void reopen_search(tuner_t * t) {
    for (int k = 0; k < TUNER_KNOBS; k += 1) {
        t->done[k] = false;
        t->step[k] = STEP_REOPEN;
    }
    t->mode = TUNER_MEASURING;
    t->wait = 0;
    arm_reset(&t->base);
    arm_reset(&t->trial);
}


static void finish_probe(tuner_t * t, const tuner_env_t * env, tuner_params_t * p) {
    float margin = tuner_margin_sigma(t, env->bracket);
    tuner_policy_t pol = policy_for(env);
    verdict_t v = probe_verdict(&t->base, &t->trial, margin, env->land_sigma, &pol);
    int k = t->knob;

    if (v == VERDICT_KEEP) {
        t->last_gain_s = arm_mean(&t->base) - arm_mean(&t->trial);
        t->good = *p;
        t->accepted += 1;
        t->step[k] = fminf(t->step[k] * 1.3f, STEP_MAX);
        // The trial's throws are the next baseline: they ran on the parameters now in force
        t->base = t->trial;
        arm_reset(&t->trial);
        t->mode = TUNER_MEASURING;
        // Keep pushing the same knob while it keeps paying
        if (!start_probe(t, env, p)) {
            *p = t->good;
            if (all_done(t)) enter_monitor(t, p);
        }
    }
    else {
        *p = t->good;
        t->dir[k] = (int8_t) -t->dir[k];
        t->step[k] *= 0.6f;
        if (t->step[k] < STEP_MIN) t->done[k] = true;
        t->knob = (uint8_t) ((k + 1) % TUNER_KNOBS);
        arm_reset(&t->trial);
        t->mode = TUNER_MEASURING;
        if (all_done(t)) enter_monitor(t, p);
    }
}


// A miss outside a probe. Blame is assigned by where the throw went wrong and the matching knob
// backs off, a little - one over is information, not an emergency, and the landing speed law and
// trim do the steady work.
static void nudge_for_over(tuner_t * t, const tuner_env_t * env, const tuner_obs_t * obs, tuner_params_t * p) {
    if (obs->coarse_ran_long) {
        apply_knob(p, KNOB_COARSE_MAX, 0.92f);
        p->coarse_min = fmaxf(env->coarse_motor_min, p->coarse_min * 0.85f);
        p->handoff *= 1.15f;
    }
    else {
        p->fine_min *= 0.85f;
        p->fine_kp *= 0.92f;
    }
    clamp_params(t, env, p);
    t->good = *p;
    // The parameters changed, so what the baseline measured no longer describes them
    arm_reset(&t->base);
}


// Steer the landing speed until the observed margin matches the Landing Sigma. Landing error scales
// linearly with landing speed (halving it halved the measured spread on hardware), so the correction
// is a direct ratio, and it is self-correcting: too fast raises the spread, which lowers the margin,
// which pulls the speed back.
static void landing_law(tuner_t * t, const tuner_env_t * env, tuner_params_t * p) {
    if (t->err_count < TUNER_ERR_MIN_N || env->land_sigma <= 0.0f) return;

    float mean, sd;
    window_stats(t, env->bracket, &mean, &sd);
    if (sd <= 1e-4f) return;
    // A bias bigger than a quarter bracket is the bulk or the trim's business, not the landing
    // speed's: slowing the landing would not fix it, only cost time.
    float reach = fmaxf(0.0f, env->bracket - fminf(fabsf(mean), 0.25f * env->bracket));
    float ratio = (reach / sd) / env->land_sigma;

    // Pace: every few passes normally, but when the margin is plainly gone (a new powder lot, a
    // humid day) waiting out the full interval just means more overcharges while it catches up.
    t->land_tick += 1;
    uint8_t every = (ratio < 0.70f) ? LAND_EVERY_URGENT : LAND_EVERY;
    if (t->land_tick < every) return;
    t->land_tick = 0;

    if (ratio <= TUNER_LAND_DEADBAND && ratio >= 1.0f / TUNER_LAND_DEADBAND) return;
    // Part of the correction only: the spread is estimated from a couple of dozen throws, so a
    // full step would chase the estimate's own noise
    ratio = powf(ratio, 0.6f);
    ratio = clampf(ratio, 1.0f / LAND_STEP, LAND_STEP);

    float before = p->fine_min;
    p->fine_min *= ratio;
    clamp_params(t, env, p);
    t->good.fine_min = p->fine_min;
    // A big landing speed move changes throw time too, so older baseline data is stale
    if (before > 0.0f && fabsf(p->fine_min / before - 1.0f) > 0.05f) arm_reset(&t->base);
}


void tuner_observe(tuner_t * t, const tuner_env_t * env, const tuner_obs_t * obs, tuner_params_t * p) {
    if (!isfinite(obs->total_s) || !isfinite(obs->error_gr) || env->bracket <= 0.0f) return;

    // First throw, or something outside the tuner moved the profile (a Learn fit, a portal edit):
    // whatever was measured before describes a different profile, so start over from here.
    if (t->init && t->mode == TUNER_PROBING && params_differ(p, &t->seen)) {
        // An outside edit landed while a candidate was live. The candidate was never verified, so it
        // must not become the new baseline by accident: put back every field the edit did not touch
        // (those still equal what the tuner left), and keep the fields that were edited.
        if (fabsf(p->fine_max - t->seen.fine_max) <= 1e-4f) p->fine_max = t->good.fine_max;
        if (fabsf(p->fine_min - t->seen.fine_min) <= 1e-4f) p->fine_min = t->good.fine_min;
        if (fabsf(p->fine_kp - t->seen.fine_kp) <= 1e-4f) p->fine_kp = t->good.fine_kp;
        if (fabsf(p->coarse_max - t->seen.coarse_max) <= 1e-4f) p->coarse_max = t->good.coarse_max;
        if (fabsf(p->coarse_min - t->seen.coarse_min) <= 1e-4f) p->coarse_min = t->good.coarse_min;
        if (fabsf(p->coarse_kp - t->seen.coarse_kp) <= 1e-4f) p->coarse_kp = t->good.coarse_kp;
        if (fabsf(p->handoff - t->seen.handoff) <= 1e-4f) p->handoff = t->good.handoff;
    }
    if (!t->init || params_differ(p, &t->seen)) {
        anchor_to(t, p);
    }
    t->throws += 1;

    // Landing error window and aim trim. Every throw counts, misses included.
    t->err[t->err_next] = obs->error_gr;
    t->err_next = (uint8_t) ((t->err_next + 1) % TUNER_ERR_WINDOW);
    if (t->err_count < TUNER_ERR_WINDOW) t->err_count += 1;
    if (t->err_count >= TUNER_ERR_MIN_N) {
        float mean, sd;
        window_stats(t, env->bracket, &mean, &sd);
        float limit = TRIM_LIMIT * env->bracket;
        t->trim_gr = clampf(t->trim_gr - TRIM_GAIN * mean, -limit, limit);
    }

    bool miss = (obs->result != TUNER_THROW_PASS);
    t->miss_bits = (t->miss_bits << 1) | (miss ? 1u : 0u);
    // Back off only on a cluster, and not on a throw that is part of a probe (the probe's own
    // verdict covers that). Clearing the history gives the back-off a chance to work before the
    // next one.
    bool cluster = (obs->result == TUNER_THROW_OVER) && recent_misses(t, 10) >= 3;

    switch (t->mode) {
        case TUNER_MEASURING:
            if (cluster) {
                nudge_for_over(t, env, obs, p);
                t->miss_bits = 0;
            }
            else {
                arm_add(&t->base, obs->total_s, obs->error_gr, miss);
                if (obs->result == TUNER_THROW_PASS) landing_law(t, env, p);
                if (t->wait > 0) t->wait -= 1;
                if (t->base.n >= TUNER_ARM_N && t->wait == 0) {
                    if (!start_probe(t, env, p)) {
                        if (all_done(t)) enter_monitor(t, p);
                        else t->wait = 4;   // gated this time, look again shortly
                    }
                }
            }
            break;

        case TUNER_PROBING:
            // Landing speed is frozen here, so the only thing that differs between the arms is
            // the knob under test.
            arm_add(&t->trial, obs->total_s, obs->error_gr, miss);
            // A gross overcharge means the candidate is wrong, not unlucky: stop paying for it
            tuner_policy_t pol = policy_for(env);
            if (obs->result == TUNER_THROW_OVER && obs->error_gr > 2.0f * env->bracket) t->trial.misses = pol.reject_misses;
            if (t->trial.n >= TUNER_ARM_N || t->trial.misses >= pol.reject_misses) finish_probe(t, env, p);
            break;

        case TUNER_CONVERGED:
            if (cluster) {
                nudge_for_over(t, env, obs, p);
                t->miss_bits = 0;
            }
            arm_add(&t->base, obs->total_s, obs->error_gr, miss);
            if (obs->result == TUNER_THROW_PASS) landing_law(t, env, p);
            if (t->base.n >= TUNER_MONITOR_N) {
                float mean_s = arm_mean(&t->base);
                if (t->base.misses >= 3) {
                    // A run of misses on a profile that was behaving: back off, then search again
                    p->handoff *= 1.15f;
                    p->fine_min *= 0.90f;
                    clamp_params(t, env, p);
                    t->good = *p;
                    reopen_search(t);
                }
                else if (t->monitor_ref_s <= 0.0f) {
                    t->monitor_ref_s = mean_s;
                    arm_reset(&t->base);
                }
                else if (mean_s > 1.15f * t->monitor_ref_s + 0.3f) {
                    t->monitor_ref_s = 0.0f;
                    reopen_search(t);
                }
                else {
                    arm_reset(&t->base);
                }
            }
            break;
    }

    clamp_params(t, env, p);
    // In a probe the live parameters are the candidate; otherwise they are the accepted ones
    if (t->mode != TUNER_PROBING) t->good = *p;
    t->seen = *p;
}


void tuner_status(const tuner_t * t, char * out, size_t out_len) {
    static const char * const names[TUNER_KNOBS] = {"handoff", "fine Kp", "bulk speed", "bulk taper"};
    if (!t->init) {
        snprintf(out, out_len, "Learning starts next throw");
        return;
    }
    switch (t->mode) {
        case TUNER_MEASURING:
            snprintf(out, out_len, "Measuring %u/%u", (unsigned) t->base.n, (unsigned) TUNER_ARM_N);
            break;
        case TUNER_PROBING:
            snprintf(out, out_len, "Trying %s %+d%% %u/%u", names[t->knob % TUNER_KNOBS],
                     (int) lroundf((float) t->dir[t->knob % TUNER_KNOBS] * t->step[t->knob % TUNER_KNOBS] * 100.0f),
                     (unsigned) t->trial.n, (unsigned) TUNER_ARM_N);
            break;
        default:
            if (t->monitor_ref_s > 0.0f) snprintf(out, out_len, "Tuned, %.1fs", (double) t->monitor_ref_s);
            else snprintf(out, out_len, "Tuned, watching");
            break;
    }
}
