// Host simulation of the charge loop around a simulated tube and scale, driving the real
// src/learn_tuner.c. Mirrors the control flow of charge_mode_start_charge_wait_for_complete()
// (rate smoothing, lag-predicted stops, the hold when predicted weight reaches the aim) and
// charge_mode_wait_for_cup_removal() (settle, top-up, judging) closely enough that a tuner which
// misbehaves here would misbehave there.
//
// The plant is a model, not the hardware. Defaults are the numbers measured on the bench:
// coarse 10.7 gr/s/rps, fine 0.32 gr/s/rps, scale lag 0.53 s coarse and 0.74 s fine, 0.02 gr
// resolution at ~4 Hz.
//
//   cc -O2 tests/tuner_sim.c src/learn_tuner.c -Isrc -lm -o tuner_sim
//   ./tuner_sim --throws 120 --tuner 1
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "learn_tuner.h"

#define DT          0.005
#define MAX_T       60.0
#define HIST_N      ((int)(MAX_T / DT) + 16)
#define SAMPLE_S    0.25

static uint64_t rng_state = 88172645463325252ULL;
static double urand(void) {
    rng_state ^= rng_state << 13;
    rng_state ^= rng_state >> 7;
    rng_state ^= rng_state << 17;
    return (double) (rng_state >> 11) / 9007199254740992.0;
}
static double nrand(void) {
    double u1 = urand(), u2 = urand();
    if (u1 < 1e-12) u1 = 1e-12;
    return sqrt(-2.0 * log(u1)) * cos(2.0 * M_PI * u2);
}

typedef struct {
    double kc, kf;              // gr/s per rps
    double lag_c, lag_c_sd;     // true scale lag, per throw draw
    double lag_f, lag_f_sd;
    double eta_c, eta_f;        // relative flow noise (AR(1), 1 s correlation)
    double read_noise;
} plant_t;

static float quantize(float v) {
    return roundf(v / 0.02f) * 0.02f;
}

static float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

typedef struct {
    float total_s, coarse_s, error_gr, coarse_pred;
    tuner_throw_result_t result;
} throw_out_t;

static double hist_c[HIST_N], hist_f[HIST_N];

static double delayed(const double * h, double t, double lag, int n) {
    double x = (t - lag) / DT;
    if (x <= 0.0) return 0.0;
    int i = (int) x;
    if (i >= n - 1) return h[n - 1];
    double f = x - i;
    return h[i] * (1.0 - f) + h[i + 1] * f;
}

typedef struct {
    float coarse_lag_cfg, fine_lag_cfg;     // what the controller believes
    bool predict;
} ctl_cfg_t;

// One whole throw. Returns the throw record. `aim_trim` is the tuner's trim.
static throw_out_t run_throw(const plant_t * pl, const tuner_params_t * p, const ctl_cfg_t * cc,
                             float target, float bracket, float trim, float handoff_cap_frac) {
    throw_out_t out;
    memset(&out, 0, sizeof(out));

    double lag_c = fmax(0.05, pl->lag_c + pl->lag_c_sd * nrand());
    double lag_f = fmax(0.05, pl->lag_f + pl->lag_f_sd * nrand());

    float handoff = p->handoff;
    float cap = handoff_cap_frac * target;
    if (cap > 0.0f && handoff > cap) handoff = cap;
    float coarse_target = fmaxf(0.0f, target - handoff);
    float aim = target + trim;
    float stop_thr = bracket + 0.0005f;

    double Mc = 0.0, Mf = 0.0;
    double ec = 0.0, ef = 0.0;      // AR(1) flow noise state
    int n = 0;
    double t = 0.0;
    hist_c[0] = hist_f[0] = 0.0;

    // Starting speeds: the loop's first command is computed from a reading of zero
    float sc = clampf(p->coarse_kp * coarse_target, p->coarse_min, p->coarse_max);
    float sf = clampf(p->fine_kp * aim, p->fine_min, p->fine_max);
    bool coarse_on = true;
    double next_sample = SAMPLE_S;
    float last_w = 0.0f, rate = 0.0f;
    bool have_last = false;
    double stop_time = -1.0;
    double coarse_stop_time = -1.0;
    float coarse_pred = target;
    double a = exp(-DT / 1.0);

    while (t < MAX_T - 1.0) {
        // Plant step
        ec = a * ec + sqrt(1.0 - a * a) * pl->eta_c * nrand();
        ef = a * ef + sqrt(1.0 - a * a) * pl->eta_f * nrand();
        if (coarse_on) Mc += fmax(0.0, pl->kc * sc * (1.0 + ec)) * DT;
        Mf += fmax(0.0, pl->kf * sf * (1.0 + ef)) * DT;
        t += DT;
        n += 1;
        hist_c[n] = Mc;
        hist_f[n] = Mf;

        if (t < next_sample) continue;
        next_sample += SAMPLE_S;

        float w = quantize((float) (delayed(hist_c, t, lag_c, n + 1) + delayed(hist_f, t, lag_f, n + 1)
                                     + pl->read_noise * nrand()));
        if (have_last) {
            float inst = (w - last_w) / (float) SAMPLE_S;
            if (inst < 0.0f) inst = 0.0f;
            rate = 0.6f * rate + 0.4f * inst;
        }
        last_w = w;
        have_last = true;

        float lag = coarse_on ? cc->coarse_lag_cfg : cc->fine_lag_cfg;
        float pred = cc->predict ? w + rate * lag : w;
        float ce = coarse_target - pred;
        float fe = aim - pred;
        float fre = aim - w;

        if (fre < stop_thr) {
            stop_time = t;
            sc = 0.0f;
            sf = 0.0f;
            break;
        }
        else if (ce <= 0.0f && coarse_on) {
            coarse_on = false;
            coarse_stop_time = t;
            coarse_pred = pred;
            sc = 0.0f;
        }

        float ns = clampf(p->fine_kp * fe, p->fine_min, p->fine_max);
        if (cc->predict && fe <= 0.0f) ns = 0.0f;
        sf = ns;
        if (coarse_on) sc = clampf(p->coarse_kp * ce, p->coarse_min, p->coarse_max);
    }
    if (stop_time < 0.0) stop_time = t;
    if (coarse_stop_time < 0.0) coarse_stop_time = stop_time;

    double total = stop_time;
    double m_final = Mc + Mf;

    // Top up: short by more than the bracket, trickle at the landing speed, twice at most
    for (int attempt = 0; attempt < 2 && (target - m_final) > bracket + 0.0005; attempt += 1) {
        double t2 = 0.0;
        double e2 = 0.0;
        double mf2 = m_final;
        // Reading starts at the settled value, then follows the new powder with the fine lag
        double base_read = m_final;
        double hist2[4096];
        int n2 = 0;
        hist2[0] = mf2;
        double ns2 = SAMPLE_S;
        bool done = false;
        while (t2 < 15.0 && !done) {
            e2 = a * e2 + sqrt(1.0 - a * a) * pl->eta_f * nrand();
            mf2 += fmax(0.0, pl->kf * p->fine_min * (1.0 + e2)) * DT;
            t2 += DT;
            n2 += 1;
            if (n2 >= 4095) break;
            hist2[n2] = mf2;
            if (t2 >= ns2) {
                ns2 += SAMPLE_S;
                double x = (t2 - lag_f) / DT;
                double r = base_read;
                if (x > 0.0) {
                    int i = (int) x;
                    if (i >= n2) i = n2 - 1;
                    r = hist2[i];
                }
                float rw = quantize((float) (r + pl->read_noise * nrand()));
                if (aim - rw < bracket + 0.0005f) done = true;
            }
        }
        m_final = mf2;
        total += t2;
    }

    out.total_s = (float) total;
    out.coarse_s = (float) coarse_stop_time;
    out.coarse_pred = coarse_pred;
    out.error_gr = (float) (m_final - target + 0.004 * nrand());
    float err = out.error_gr;
    if (err < -(bracket + 0.0005f)) out.result = TUNER_THROW_UNDER;
    else if (err > bracket + 0.0005f) out.result = TUNER_THROW_OVER;
    else out.result = TUNER_THROW_PASS;
    return out;
}


// ---- The tuner this one replaced, ported from charge_mode.cpp learn_post_throw() as of v2.3, so the
// new tuner can be measured against what shipped rather than only against a static profile.
typedef struct {
    bool has_baseline;
    tuner_params_t base, seen;
    uint8_t clean_streak;
    float err_window[12];
    uint8_t err_count, err_next;
    float trim;
    uint8_t land_tick;
} old_t;

static float old_bound(float v, float base, float cap) {
    float lo = base * 0.5f, hi = base * 1.4f;
    if (cap > 0.0f && hi > cap) hi = cap;
    return v < lo ? lo : (v > hi ? hi : v);
}
static float old_bound_land(float v, float base) {
    float lo = base * 0.5f, hi = base * 2.0f;
    return v < lo ? lo : (v > hi ? hi : v);
}

static void old_observe(old_t * st, const tuner_env_t * env, const tuner_obs_t * obs, tuner_params_t * p,
                        float coarse_s) {
    float handoff = p->handoff;
    bool moved = st->has_baseline && (fabsf(p->fine_max - st->seen.fine_max) > 1e-4f ||
        fabsf(p->fine_min - st->seen.fine_min) > 1e-4f || fabsf(p->fine_kp - st->seen.fine_kp) > 1e-4f ||
        fabsf(p->coarse_max - st->seen.coarse_max) > 1e-4f || fabsf(handoff - st->seen.handoff) > 1e-4f);
    if (!st->has_baseline || moved) {
        memset(st, 0, sizeof(*st));
        st->has_baseline = true;
        st->base = *p;
    }
    st->err_window[st->err_next] = obs->error_gr;
    st->err_next = (st->err_next + 1) % 12;
    if (st->err_count < 12) st->err_count += 1;
    bool have = (st->err_count >= 8);
    float em = 0, esd = 0;
    if (have) {
        double sum = 0, sq = 0;
        for (int i = 0; i < st->err_count; i += 1) { sum += st->err_window[i]; sq += (double) st->err_window[i] * st->err_window[i]; }
        double m = sum / st->err_count, var = (sq - st->err_count * m * m) / (st->err_count - 1);
        em = (float) m; esd = var > 0 ? (float) sqrt(var) : 0.0f;
        float lim = 0.5f * env->bracket;
        float tr = st->trim - 0.15f * em;
        st->trim = tr > lim ? lim : (tr < -lim ? -lim : tr);
    }
    float fine_s = obs->total_s - coarse_s;
    float taper = (p->fine_kp > 0) ? p->fine_max / p->fine_kp : env->bracket;
    if (obs->result == TUNER_THROW_OVER) {
        st->clean_streak = 0;
        if (obs->coarse_ran_long) {
            float old = p->coarse_max;
            p->coarse_max = old_bound(old * 0.92f, st->base.coarse_max, env->coarse_motor_cap);
            p->coarse_kp *= p->coarse_max / old;
            p->coarse_min = fmaxf(env->coarse_motor_min, p->coarse_min * 0.85f);
            float w = fminf(handoff * 1.15f, 0.5f * env->target);
            p->handoff = old_bound(w, st->base.handoff, 0.0f);
        } else {
            p->fine_min = old_bound_land(p->fine_min * 0.85f, st->base.fine_min);
            p->fine_max = old_bound(p->fine_max * 0.92f, st->base.fine_max, env->fine_motor_cap);
            float nt = taper * 1.2f;
            if (nt > 0) p->fine_kp = old_bound(p->fine_max / nt, st->base.fine_kp, 0.0f);
        }
    } else if (obs->result == TUNER_THROW_PASS) {
        st->clean_streak += 1;
        if (have) {
            st->land_tick += 1;
            if (st->land_tick >= 4) {
                st->land_tick = 0;
                float reach = fmaxf(0.0f, env->bracket - fabsf(em));
                if (esd > 1e-4f) {
                    float ratio = (reach / esd) / env->land_sigma;
                    if (ratio > 1.10f || ratio < 0.90f) {
                        if (ratio > 1.25f) ratio = 1.25f;
                        if (ratio < 0.8f) ratio = 0.8f;
                        p->fine_min = old_bound_land(p->fine_min * ratio, st->base.fine_min);
                    }
                }
            }
        }
        if (st->clean_streak >= 5) {
            st->clean_streak = 0;
            if (coarse_s >= fine_s) {
                float oc = p->coarse_max;
                p->coarse_max = old_bound(oc * 1.06f, st->base.coarse_max, env->coarse_motor_cap);
                if (oc > 0) p->coarse_kp *= p->coarse_max / oc;
            } else {
                float floor_ = 3.0f * 0.107f;
                float tighter = handoff * 0.95f;
                if (tighter < floor_) tighter = fminf(handoff, floor_);
                p->handoff = old_bound(tighter, st->base.handoff, 0.0f);
                if (fine_s > 3.0f) {
                    float om = p->fine_max;
                    p->fine_max = old_bound(om * 1.05f, st->base.fine_max, env->fine_motor_cap);
                    if (taper > 0) p->fine_kp = old_bound(p->fine_max / taper, st->base.fine_kp, 0.0f);
                }
            }
        }
    } else {
        st->clean_streak = 0;
    }
    if (p->fine_min > p->fine_max) p->fine_min = p->fine_max;
    if (p->coarse_min > p->coarse_max) p->coarse_min = p->coarse_max;
    if (p->handoff < 0.30f) p->handoff = 0.30f;
    float cap = 0.5f * env->target;
    if (cap > 0.30f && p->handoff > cap) p->handoff = cap;
    st->seen = *p;
}


// Hard limits the tuner must never break, whatever the plant does. Checked after every throw.
static void check_invariants(const tuner_params_t * p, const tuner_params_t * start, float target, int throw_no) {
    bool ok = isfinite(p->fine_max) && isfinite(p->fine_min) && isfinite(p->fine_kp) &&
              isfinite(p->coarse_max) && isfinite(p->coarse_min) && isfinite(p->coarse_kp) && isfinite(p->handoff);
    float h_hi = 0.5f * target;
    ok = ok && p->handoff >= fminf(0.30f, h_hi) - 1e-4f && p->handoff <= h_hi + 1e-4f;
    ok = ok && p->fine_kp >= 0.4f * start->fine_kp - 1e-4f && p->fine_kp <= 3.0f * start->fine_kp + 1e-4f;
    ok = ok && p->fine_min >= 0.5f * start->fine_min - 1e-4f && p->fine_min <= 2.0f * start->fine_min + 1e-4f;
    ok = ok && p->fine_min <= p->fine_max + 1e-4f;
    ok = ok && p->coarse_max >= 0.5f * start->coarse_max - 1e-4f && p->coarse_max <= 2.0f * start->coarse_max + 1e-4f;
    ok = ok && p->coarse_max <= 5.0f + 1e-4f;
    ok = ok && p->coarse_min <= p->coarse_max + 1e-4f && p->coarse_min >= 0.05f - 1e-4f;
    ok = ok && p->coarse_kp >= 0.25f * start->coarse_kp * 0.5f && p->coarse_kp > 0.0f;
    if (!ok) {
        printf("INVARIANT VIOLATION at throw %d: handoff %.3f fkp %.3f fmin %.3f cmax %.3f cmin %.3f ckp %.4f\n",
               throw_no, p->handoff, p->fine_kp, p->fine_min, p->coarse_max, p->coarse_min, p->coarse_kp);
        exit(2);
    }
}

static bool arg_f(int argc, char ** argv, const char * name, double * v) {
    for (int i = 1; i + 1 < argc; i += 1) {
        if (strcmp(argv[i], name) == 0) { *v = atof(argv[i + 1]); return true; }
    }
    return false;
}

int main(int argc, char ** argv) {
    double v;
    int throws = 120, use_tuner = 1, block = 20, drift_at = -1;
    uint64_t seed = 1;
    if (arg_f(argc, argv, "--throws", &v)) throws = (int) v;
    if (arg_f(argc, argv, "--tuner", &v)) use_tuner = (int) v;
    if (arg_f(argc, argv, "--seed", &v)) seed = (uint64_t) v;
    if (arg_f(argc, argv, "--block", &v)) block = (int) v;
    if (arg_f(argc, argv, "--drift", &v)) drift_at = (int) v;
    rng_state ^= seed * 0x9E3779B97F4A7C15ULL;
    for (int i = 0; i < 8; i += 1) urand();

    plant_t pl = {.kc = 10.7, .kf = 0.32, .lag_c = 0.53, .lag_c_sd = 0.02, .lag_f = 0.74, .lag_f_sd = 0.15,
                  .eta_c = 0.03, .eta_f = 0.10, .read_noise = 0.004};
    arg_f(argc, argv, "--kc", &pl.kc);
    arg_f(argc, argv, "--kf", &pl.kf);
    arg_f(argc, argv, "--lagc", &pl.lag_c);
    arg_f(argc, argv, "--lagf", &pl.lag_f);
    arg_f(argc, argv, "--lagcsd", &pl.lag_c_sd);
    arg_f(argc, argv, "--lagfsd", &pl.lag_f_sd);
    arg_f(argc, argv, "--etac", &pl.eta_c);
    arg_f(argc, argv, "--etaf", &pl.eta_f);

    double target_d = 26.5, bracket_d = 0.06, lsig = 2.0;
    arg_f(argc, argv, "--target", &target_d);
    arg_f(argc, argv, "--bracket", &bracket_d);
    arg_f(argc, argv, "--lsig", &lsig);
    float target = (float) target_d, bracket = (float) bracket_d;
    double lsig_unused = 7.0;

    // NewProfile7 as read off the device
    tuner_params_t p = {.fine_max = 4.410f, .fine_min = 0.343f, .fine_kp = 1.576f,
                        .coarse_max = 1.556f, .coarse_min = 0.310f, .coarse_kp = 0.056f, .handoff = 0.569f};
    double h0;
    if (arg_f(argc, argv, "--handoff", &h0)) p.handoff = (float) h0;
    if (arg_f(argc, argv, "--ckp", &h0)) p.coarse_kp = (float) h0;
    if (arg_f(argc, argv, "--cmax", &h0)) p.coarse_max = (float) h0;
    if (arg_f(argc, argv, "--cmin", &h0)) p.coarse_min = (float) h0;
    if (arg_f(argc, argv, "--fkp", &h0)) p.fine_kp = (float) h0;
    if (arg_f(argc, argv, "--fmin", &h0)) p.fine_min = (float) h0;
    if (arg_f(argc, argv, "--fmax", &h0)) p.fine_max = (float) h0;
    ctl_cfg_t cc = {.coarse_lag_cfg = 0.55f, .fine_lag_cfg = 0.74f, .predict = true};

    tuner_env_t env = {.target = target, .bracket = bracket, .land_sigma = (float) lsig,
                       .coarse_motor_cap = 5.0f, .fine_motor_cap = 4.5f, .coarse_motor_min = 0.05f,
                       .coarse_sd_floor_gr = 0.321f, .handoff_cap_frac = 0.5f, .time_goal_s = 7.0f};
    (void) 0;

    arg_f(argc, argv, "--goal", &lsig_unused);
    env.time_goal_s = (float) lsig_unused;
    const tuner_params_t start_params = p;
    tuner_t tuner;
    tuner_init(&tuner);
    old_t old_state;
    memset(&old_state, 0, sizeof(old_state));

    double bt = 0, bt2 = 0, be = 0, be2 = 0, bc = 0;
    int bn = 0, bmiss = 0, over_total = 0, miss_total = 0;
    double tt = 0;
    printf("throw  time   sd   err_sd  miss  handoff  fkp   fmin  cmax   mode\n");
    for (int i = 1; i <= throws; i += 1) {
        if (i == drift_at) {
            // A different powder lot: slower, laggier fine tube
            pl.kf *= 0.8; pl.lag_f += 0.2; pl.eta_f *= 1.3;
        }
        float trim = (use_tuner == 1 || use_tuner == 2) ? tuner_trim(&tuner) : (use_tuner == 3 ? old_state.trim : 0.0f);
        throw_out_t o = run_throw(&pl, &p, &cc, target, bracket, trim, 0.5f);
        if (use_tuner == 3) {
            tuner_obs_t obs = {.result = o.result, .total_s = o.total_s, .error_gr = o.error_gr,
                               .coarse_ran_long = (o.coarse_pred - (target - fminf(p.handoff, 0.5f * target)) > 0.25f * p.handoff)};
            old_observe(&old_state, &env, &obs, &p, o.coarse_s);
        }
        else if (use_tuner) {
            tuner_obs_t obs = {.result = o.result, .total_s = o.total_s, .error_gr = o.error_gr,
                               .coarse_ran_long = (o.coarse_pred - (target - fminf(p.handoff, 0.5f * target)) > 0.25f * p.handoff)};
            tuner_params_t keep = p;
            tuner_observe(&tuner, &env, &obs, &p);
            if (use_tuner == 1) check_invariants(&p, &start_params, target, i);
            if (use_tuner == 2) {
                // Static profile: only the aim trim and the landing speed law are live
                p.handoff = keep.handoff; p.fine_kp = keep.fine_kp; p.coarse_max = keep.coarse_max;
                p.coarse_kp = keep.coarse_kp; p.coarse_min = keep.coarse_min; p.fine_max = keep.fine_max;
            }
        }
        bt += o.total_s; bt2 += (double) o.total_s * o.total_s;
        be += o.error_gr; be2 += (double) o.error_gr * o.error_gr;
        bn += 1; tt += o.total_s; bc += o.coarse_s;
        if (o.result != TUNER_THROW_PASS) { bmiss += 1; miss_total += 1; }
        if (o.result == TUNER_THROW_OVER) over_total += 1;
        if (bn == block || i == throws) {
            double m = bt / bn, sd = sqrt(fmax(0.0, bt2 / bn - m * m));
            double em = be / bn, esd = sqrt(fmax(0.0, be2 / bn - em * em));
            char st[48];
            tuner_status(&tuner, st, sizeof(st));
            printf("%4d  %5.2f  %4.2f  %5.3f  %2d/%-2d  %5.2f   %4.2f  %4.2f  %4.2f   %s\n",
                   i, m, sd, esd, bmiss, bn, p.handoff, p.fine_kp, p.fine_min, p.coarse_max, use_tuner ? st : "-");
            (void) sd;
            bt = bt2 = be = be2 = bc = 0; bn = 0; bmiss = 0;
        }
    }
    printf("SUMMARY mean_time=%.3f misses=%d overs=%d throws=%d accepted=%u probes=%u handoff=%.3f fkp=%.3f fmin=%.3f cmax=%.3f\n",
           tt / throws, miss_total, over_total, throws, (unsigned) tuner.accepted, (unsigned) tuner.probes,
           p.handoff, p.fine_kp, p.fine_min, p.coarse_max);
    return 0;
}
