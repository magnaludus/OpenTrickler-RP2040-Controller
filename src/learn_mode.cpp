#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <FreeRTOS.h>
#include <queue.h>
#include <task.h>
#include <u8g2.h>

#include "app.h"
#include "FloatRingBuffer.h"
#include "mini_12864_module.h"
#include "display.h"
#include "scale.h"
#include "motors.h"
#include "charge_mode.h"
#include "profile.h"
#include "neopixel_led.h"
#include "session_stats.h"
#include "eeprom.h"
#include "common.h"
#include "learn_mode.h"

extern "C" void scale_write(const char * command, size_t len);


learn_mode_t learn_mode;

extern scale_config_t scale_config;
extern charge_mode_config_t charge_mode_config;
extern neopixel_led_config_t neopixel_led_config;
extern eeprom_profile_data_t profile_data;
extern AppState_t exit_state;
extern QueueHandle_t encoder_event_queue;

// From charge_mode.cpp
void charge_mode_wait_for_complete();
float charge_mode_get_last_elapsed_seconds(void);

static TaskHandle_t learn_render_task_handler = NULL;
static TickType_t throw_start_tick = 0;
static bool throw_running = false;

// Tail limits used when picking the fastest usable speed on each tube
#define LEARN_COARSE_STOP_MARGIN_GR     0.15f
#define LEARN_FINE_TAPER_MIN_GR         0.20f   // shortest taper window
#define LEARN_FINE_TAPER_TAIL_MULT      3.0f    // taper window = this x the fine tail at max, floored above
#define LEARN_SETTLE_ALLOWANCE_S        0.50f
#define LEARN_CUP_REMOVED_GR            -5.0f   // reading below this means the cup came off
#define LEARN_ZERO_RETRY_MS             2500    // resend the zero command if the scale has not taken it by then
#define LEARN_ZERO_MAX_TRIES            12

static const learn_config_t default_learn_config = {
    .learn_config_rev = 0,
    .coarse_target = 8.0f,
    .fine_target = 1.75f,
    .coarse_speed_ceiling = 6.0f,
    .fine_speed_ceiling = 4.0f,
    .confirm_target = 42.5f,
    .confirm_throws = LEARN_CONFIRM_THROWS,
    // The fit spends every second under this goal buying a tighter handoff, so the goal is the dial
    // between a fast throw and a bulk that gets closer before handing over.
    .time_goal_s = 7.0f,
    .cup_capacity_gr = 250.0f,
    .min_success_pct = 95.0f,
    .coarse_stop_safety = 1.5f,
};

// Largest settled throw seen on each tube this run, used to size the cup check
static float max_settled_coarse = 0.0f;
static float max_settled_fine = 0.0f;
#define LEARN_COARSE_STOP_MIN_GR        0.30f
#define LEARN_THROW_TIMEOUT_MS          120000
#define LEARN_MIN_FLOW_GPS              0.02f   // below this the tube is not moving powder, abort


#define LEARN_BRACKET_USE_FRAC          0.80f   // fine landing error has to fit in this much of the bracket
#define LEARN_SEARCH_STEPS              12
#define LEARN_FINE_LAND_FLOW_GPS        0.06f   // fine tube landing flow, a couple of kernels a second
#define LEARN_PREDICT_STOP_MARGIN_GR    0.30f
#define LEARN_COARSE_FLOW_CAP_GPS       18.0f   // no point characterising a bulk rate faster than this
#define LEARN_COARSE_EXTRAP_MULT        1.25f   // fit may pick a coarse speed at most this far past the ladder
#define LEARN_COARSE_STOP_SAFETY_MIN    1.0f    // bare 3 sigma, no cushion
#define LEARN_COARSE_STOP_SAFETY_MAX    5.0f
#define LEARN_COARSE_MIN_RUN_S          1.20f   // has to run well past the lag or the numbers mean nothing
#define LEARN_COARSE_MAX_RUN_S          4.00f
#define LEARN_FINE_MIN_RUN_S            3.00f
#define LEARN_FINE_MAX_RUN_S            12.0f


static void set_message(const char * msg) {
    strncpy(learn_mode.message, msg, sizeof(learn_mode.message) - 1);
    learn_mode.message[sizeof(learn_mode.message) - 1] = '\0';
}


// ---------------------------------------------------------------------------
// Display
// ---------------------------------------------------------------------------
static void learn_render_task(void *p) {
    char buf[32];
    u8g2_t *display_handler = get_display_handler();

    while (true) {
        TickType_t last_render_tick = xTaskGetTickCount();
        u8g2_ClearBuffer(display_handler);
        uint8_t screen_width = u8g2_GetDisplayWidth(display_handler);

        // Title
        u8g2_SetFont(display_handler, u8g2_font_helvB08_tr);
        switch (learn_mode.state) {
            case LEARN_STATE_WAIT_FOR_ZERO: snprintf(buf, sizeof(buf), "Learn: Zeroing"); break;
            case LEARN_STATE_COARSE:  snprintf(buf, sizeof(buf), "Learn C %d/%d", learn_mode.throw_idx + 1, LEARN_THROWS_PER_PHASE); break;
            case LEARN_STATE_FINE:    snprintf(buf, sizeof(buf), "Learn F %d/%d", learn_mode.throw_idx + 1, LEARN_THROWS_PER_PHASE); break;
            case LEARN_STATE_FIT:     snprintf(buf, sizeof(buf), "Learn: Fitting"); break;
            case LEARN_STATE_CONFIRM: snprintf(buf, sizeof(buf), "Confirm %d/%d", learn_mode.throw_idx + 1, learn_mode.config.confirm_throws); break;
            case LEARN_STATE_DONE:    snprintf(buf, sizeof(buf), "Learn: Done"); break;
            case LEARN_STATE_ABORTED: snprintf(buf, sizeof(buf), "Learn: Stopped"); break;
            case LEARN_STATE_ERROR:   snprintf(buf, sizeof(buf), "Learn: Error"); break;
            case LEARN_STATE_EMPTY_CUP: snprintf(buf, sizeof(buf), "EMPTY THE CUP"); break;
            default:                  snprintf(buf, sizeof(buf), "Learn"); break;
        }
        u8g2_DrawStr(display_handler, 5, 10, buf);

        // Timer on the right while a throw is running
        if (throw_running) {
            float elapsed = (float)((xTaskGetTickCount() - throw_start_tick) * portTICK_PERIOD_MS) / 1000.0f;
            snprintf(buf, sizeof(buf), "%.1f s", elapsed);
            uint8_t w = u8g2_GetStrWidth(display_handler, buf);
            u8g2_DrawStr(display_handler, screen_width - w - 5, 10, buf);
        }
        u8g2_DrawHLine(display_handler, 0, 13, screen_width);

        if (learn_mode.state == LEARN_STATE_DONE && learn_mode.result_valid) {
            // Result summary
            u8g2_SetFont(display_handler, u8g2_font_profont11_tf);
            snprintf(buf, sizeof(buf), "C max %.2f stop %.2f", learn_mode.result.coarse_max_rps, learn_mode.result.coarse_stop_threshold);
            u8g2_DrawStr(display_handler, 3, 24, buf);
            snprintf(buf, sizeof(buf), "F max %.2f taper %.2f", learn_mode.result.fine_max_rps, learn_mode.result.fine_taper_gr);
            u8g2_DrawStr(display_handler, 3, 34, buf);
            snprintf(buf, sizeof(buf), "R%d %d/%d avg %.1fs %s", learn_mode.result.confirm_rounds, learn_mode.result.confirm_pass, learn_mode.result.confirm_total, learn_mode.result.confirm_avg_time, learn_mode.result.confirm_met ? "OK" : "MISS");
            u8g2_DrawStr(display_handler, 3, 44, buf);
            snprintf(buf, sizeof(buf), "Pred %.1fs goal %.1fs", learn_mode.result.predicted_total_s, learn_mode.config.time_goal_s);
            u8g2_DrawStr(display_handler, 3, 53, buf);
            u8g2_SetFont(display_handler, u8g2_font_helvR08_tr);
            u8g2_DrawStr(display_handler, 3, 61, "Knob: Save   RST: Skip");
        }
        else if (learn_mode.state == LEARN_STATE_ERROR || learn_mode.state == LEARN_STATE_ABORTED) {
            u8g2_SetFont(display_handler, u8g2_font_helvR08_tr);
            u8g2_DrawStr(display_handler, 5, 30, learn_mode.message);
            u8g2_DrawStr(display_handler, 5, 61, "Press RST to exit");
        }
        else if (learn_mode.state == LEARN_STATE_EMPTY_CUP) {
            u8g2_SetFont(display_handler, u8g2_font_helvR08_tr);
            snprintf(buf, sizeof(buf), "In cup %.0f of %.0f gr", learn_mode.cup_load_gr, learn_mode.config.cup_capacity_gr);
            u8g2_DrawStr(display_handler, 5, 27, buf);
            u8g2_DrawStr(display_handler, 5, 39, "Dump it, put it back.");
            u8g2_DrawStr(display_handler, 5, 51, "Zeroes when it returns.");
            uint8_t w = u8g2_GetStrWidth(display_handler, learn_mode.message);
            u8g2_DrawStr(display_handler, screen_width - w - 3, 61, learn_mode.message);
        }
        else {
            // Weight
            float m = scale_get_current_measurement();
            if (m > -1.0f) {
                float_to_string(buf, m, charge_mode_config.eeprom_charge_mode_data.decimal_places);
            }
            else {
                strcpy(buf, "---");
            }
            u8g2_SetFont(display_handler, u8g2_font_profont22_tf);
            u8g2_DrawStr(display_handler, 26, 35, buf);

            // Bottom: speed and message
            u8g2_SetFont(display_handler, u8g2_font_helvR08_tr);
            if (learn_mode.state == LEARN_STATE_COARSE || learn_mode.state == LEARN_STATE_FINE) {
                snprintf(buf, sizeof(buf), "%.2f rps", learn_mode.current_speed);
                u8g2_DrawStr(display_handler, 5, 61, buf);
            }
            uint8_t w = u8g2_GetStrWidth(display_handler, learn_mode.message);
            u8g2_DrawStr(display_handler, screen_width - w - 3, 61, learn_mode.message);
        }

        u8g2_SendBuffer(display_handler);
        vTaskDelayUntil(&last_render_tick, pdMS_TO_TICKS(50));
    }
}


// ---------------------------------------------------------------------------
// Scale helpers
// ---------------------------------------------------------------------------

// Returns false if the user pressed RST
static bool check_abort(void) {
    ButtonEncoderEvent_t ev = button_wait_for_input(false);
    if (ev == BUTTON_RST_PRESSED) {
        learn_mode.state = LEARN_STATE_ABORTED;
        set_message("Stopped by user");
        return false;
    }
    return true;
}


// Wait until the scale reading holds still. Returns the settled reading.
static bool wait_for_stable(float * settled) {
    FloatRingBuffer data_buffer(8);
    TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(30000);

    while (true) {
        TickType_t last_tick = xTaskGetTickCount();
        if (!check_abort()) return false;
        if (last_tick > deadline) {
            learn_mode.state = LEARN_STATE_ERROR;
            set_message("Scale not settling");
            return false;
        }

        float m;
        if (scale_block_wait_for_next_measurement(300, &m)) {
            data_buffer.enqueue(m);
        }
        if (data_buffer.getCounter() >= 8 &&
            data_buffer.getSd() < charge_mode_config.eeprom_charge_mode_data.set_point_sd_margin) {
            *settled = data_buffer.getMean();
            return true;
        }
        vTaskDelayUntil(&last_tick, pdMS_TO_TICKS(200));
    }
}


// Re-zero on the loaded pan, then wait for a stable zero
static bool auto_zero(void) {
    set_message("Zeroing");
    float settled;

    // Let it settle first so the zero is taken on a still reading
    if (!wait_for_stable(&settled)) return false;

    // The A&D ignores a zero command while it reports unstable, so send it, watch the reading,
    // and send it again if it has not taken. The knob also sends one by hand.
    FloatRingBuffer data_buffer(6);
    int tries = 0;
    TickType_t next_send = 0;

    while (true) {
        TickType_t last_tick = xTaskGetTickCount();

        ButtonEncoderEvent_t ev = button_wait_for_input(false);
        if (ev == BUTTON_RST_PRESSED) {
            learn_mode.state = LEARN_STATE_ABORTED;
            set_message("Stopped by user");
            return false;
        }
        if (ev == BUTTON_ENCODER_PRESSED) {
            next_send = 0;   // force a resend now
        }

        if (last_tick >= next_send) {
            if (tries >= LEARN_ZERO_MAX_TRIES) {
                learn_mode.state = LEARN_STATE_ERROR;
                set_message("Zero failed");
                return false;
            }
            // Alternate re-zero and tare. Re-zero on an A&D only works within a small window either
            // side of the calibration zero, and after a cup of powder has been tared off we are well
            // outside it. Tare covers the full range, so it is what actually takes at that point.
            if ((tries % 2) == 0) {
                if (scale_config.scale_handle->force_zero != NULL) {
                    scale_config.scale_handle->force_zero();
                }
            }
            else {
                const char tare_cmd[] = "T\r\n";
                scale_write(tare_cmd, sizeof(tare_cmd) - 1);
            }
            tries += 1;
            next_send = last_tick + pdMS_TO_TICKS(LEARN_ZERO_RETRY_MS);
            data_buffer.reset();
            if (tries > 2) set_message("Zeroing, knob to retry");
        }

        float m;
        if (scale_block_wait_for_next_measurement(300, &m)) {
            data_buffer.enqueue(m);
        }
        if (data_buffer.getCounter() >= 6 &&
            data_buffer.getSd() < charge_mode_config.eeprom_charge_mode_data.set_point_sd_margin &&
            fabsf(data_buffer.getMean()) < charge_mode_config.eeprom_charge_mode_data.set_point_mean_margin) {
            return true;
        }
        vTaskDelayUntil(&last_tick, pdMS_TO_TICKS(200));
    }
}


// ---------------------------------------------------------------------------
// Cup handling. Stops and waits for the user to dump the cup. Zero is taken on the empty cup when it comes back.
// ---------------------------------------------------------------------------
static bool empty_cup_and_zero(void) {
    learn_state_t resume_state = learn_mode.state;
    learn_mode.state = LEARN_STATE_EMPTY_CUP;
    set_message("Remove cup");
    neopixel_led_set_colour(charge_mode_config.eeprom_charge_mode_data.neopixel_session_backlight_colour,
                            charge_mode_config.eeprom_charge_mode_data.neopixel_not_ready_colour,
                            charge_mode_config.eeprom_charge_mode_data.neopixel_not_ready_colour, true);

    // Wait for the cup to come off
    while (true) {
        if (!check_abort()) return false;
        float m;
        if (scale_block_wait_for_next_measurement(200, &m) && m < LEARN_CUP_REMOVED_GR) break;
    }

    set_message("Return cup");
    // Wait for it to come back
    while (true) {
        if (!check_abort()) return false;
        float m;
        if (scale_block_wait_for_next_measurement(200, &m) && m > LEARN_CUP_REMOVED_GR) break;
    }
    vTaskDelay(pdMS_TO_TICKS(500));

    // Zero on the empty cup. A whole cup of powder has been tared off by now, so lead with tare.
    {
        const char tare_cmd[] = "T\r\n";
        scale_write(tare_cmd, sizeof(tare_cmd) - 1);
        vTaskDelay(pdMS_TO_TICKS(600));
    }
    if (!auto_zero()) return false;
    learn_mode.cup_load_gr = 0.0f;
    learn_mode.state = resume_state;
    return true;
}


// Call before any throw with the amount that will actually land, not the target.
static bool ensure_cup_room(float expected_gr) {
    if (learn_mode.cup_load_gr + expected_gr > learn_mode.config.cup_capacity_gr) {
        return empty_cup_and_zero();
    }
    return true;
}


// ---------------------------------------------------------------------------
// One throw on one tube: run at a fixed speed for a fixed time, then settle.
// Duration based, not weight based. The scale reads behind the powder, so stopping on a
// reading corrupts both the flow and the lag. Mass in the cup over motor run time is the
// true flow no matter how far behind the display is.
// ---------------------------------------------------------------------------
static bool throw_for_time(motor_select_t motor, float speed, float run_s, float expected_gr,
                           float * max_settled, learn_throw_t * out) {
    if (!ensure_cup_room(fmaxf(expected_gr, *max_settled) * 1.15f)) return false;

    set_message("Running");
    learn_mode.current_speed = speed;

    // Reading before the motor starts, so the throw mass is measured from here
    float base = scale_get_current_measurement();
    if (base < -1.0f) base = 0.0f;

    throw_start_tick = xTaskGetTickCount();
    throw_running = true;
    TickType_t stop_target = throw_start_tick + pdMS_TO_TICKS((uint32_t)(run_s * 1000.0f));
    motor_set_speed(motor, speed);

    float latest = base;
    float start_w = base;
    bool motion_seen = false;
    float dead_time = 0.0f;
    bool ok = true;

    while (xTaskGetTickCount() < stop_target) {
        if (!check_abort()) { ok = false; break; }
        float m;
        if (scale_block_wait_for_next_measurement(150, &m)) {
            latest = m;
            if (!motion_seen && m > start_w + 0.10f) {
                motion_seen = true;
                dead_time = (float)((xTaskGetTickCount() - throw_start_tick) * portTICK_PERIOD_MS) / 1000.0f;
            }
        }
    }

    motor_set_speed(motor, 0);
    TickType_t stop_tick = xTaskGetTickCount();
    throw_running = false;
    if (!ok) return false;

    float actual_run_s = (float)((stop_tick - throw_start_tick) * portTICK_PERIOD_MS) / 1000.0f;
    if (actual_run_s <= 0.0f) actual_run_s = 0.001f;
    float stop_weight = latest;

    set_message("Settling");
    float settled;
    if (!wait_for_stable(&settled)) return false;

    float mass = settled - base;
    if (mass < 0.0f) mass = 0.0f;

    out->speed_rps = speed;
    out->time_s = actual_run_s;
    out->stop_weight = stop_weight - base;
    out->settled_weight = mass;
    out->flow_gps = mass / actual_run_s;                  // what actually landed over the motor run time
    out->tail = mass - out->stop_weight;                  // still in the air when the motor stopped
    out->rate_at_stop = out->flow_gps;                    // steady state, the flow is the rate
    out->dead_time_s = dead_time;
    // Lag straight out: the tail is exactly the flow multiplied by however far the scale reads behind
    out->lag_s = (out->flow_gps > 0.02f) ? (out->tail / out->flow_gps) : 0.0f;
    if (out->lag_s < 0.0f) out->lag_s = 0.0f;
    if (out->lag_s > 5.0f) out->lag_s = 5.0f;

    if (mass > *max_settled) *max_settled = mass;
    learn_mode.cup_load_gr += mass;

    if (out->flow_gps < LEARN_MIN_FLOW_GPS) {
        learn_mode.state = LEARN_STATE_ERROR;
        set_message("No flow, check tube");
        return false;
    }

    return auto_zero();
}


// ---------------------------------------------------------------------------
// Fit
// ---------------------------------------------------------------------------
// Predict a throw at the confirm target for one coarse level and one fine level.
// The charge loop runs both tubes during the bulk, then only the fine after the coarse stops.
// After the coarse stops, its tail lands, so the fine only has to trickle (stop threshold - tail).
// Flow per rps, least squares through the origin
static float fit_flow_per_rps(const learn_throw_t * throws, int count) {
    double num = 0.0, den = 0.0;
    for (int i = 0; i < count; i += 1) {
        if (throws[i].time_s <= 0.0f) continue;
        num += (double) throws[i].flow_gps * throws[i].speed_rps;
        den += (double) throws[i].speed_rps * throws[i].speed_rps;
    }
    return (den > 0.0) ? (float)(num / den) : 0.0f;
}


// Sample standard deviation from running sums. Returns 0 for n < 2.
static float sample_sd(double sum, double sum_sq, int n) {
    if (n < 2) return 0.0f;
    double mean = sum / n;
    double var = (sum_sq - n * mean * mean) / (n - 1);
    return (var > 0.0) ? (float) sqrt(var) : 0.0f;
}


// Mean and SD of the per throw lag, both combined and per phase. Coarse and fine consistently
// measure different lag, so a combined SD is inflated by the gap between their means, not just
// noise - that pooled number is only for display. Anything driving a safety margin must use the
// per-phase SD instead, or a fresh, consistent tube reads as if it were a noisy one.
static void lag_stats(float * mean_out, float * sd_out, float * coarse_out, float * fine_out,
                      float * coarse_sd_out, float * fine_sd_out, float * dead_out) {
    double sum = 0.0, sum_sq = 0.0;
    double c_sum = 0.0, c_sum_sq = 0.0, f_sum = 0.0, f_sum_sq = 0.0, d_sum = 0.0;
    int n = 0, c_n = 0, f_n = 0, d_n = 0;
    for (int phase = 0; phase < 2; phase += 1) {
        const learn_throw_t * t = (phase == 0) ? learn_mode.coarse : learn_mode.fine;
        for (int i = 0; i < LEARN_THROWS_PER_PHASE; i += 1) {
            if (t[i].lag_s <= 0.0f) continue;
            double v = (double) t[i].lag_s;
            sum += v;
            sum_sq += v * v;
            n += 1;
            if (phase == 0) { c_sum += v; c_sum_sq += v * v; c_n += 1; }
            else { f_sum += v; f_sum_sq += v * v; f_n += 1; }
            if (t[i].dead_time_s > 0.0f) { d_sum += t[i].dead_time_s; d_n += 1; }
        }
    }
    *mean_out = (n > 0) ? (float)(sum / n) : 0.0f;
    *sd_out = sample_sd(sum, sum_sq, n);
    *coarse_out = (c_n > 0) ? (float)(c_sum / c_n) : 0.0f;
    *fine_out = (f_n > 0) ? (float)(f_sum / f_n) : 0.0f;
    *coarse_sd_out = sample_sd(c_sum, c_sum_sq, c_n);
    *fine_sd_out = sample_sd(f_sum, f_sum_sq, f_n);
    *dead_out = (d_n > 0) ? (float)(d_sum / d_n) : 0.0f;
}


// Predict a throw at the confirm target.
// Bulk: both tubes running until the coarse hands off. Fine: full speed down to the taper window,
// then a ramp from fine max to fine min through the window.
static void predict_throw(float target, float coarse_flow, float fine_flow_max, float fine_flow_min,
                          float handoff, float taper_gr, float * coarse_s, float * fine_s) {
    float bulk_rate = coarse_flow + fine_flow_max;
    float bulk_gr = fmaxf(0.0f, target - handoff);
    *coarse_s = (bulk_rate > 0.0f) ? bulk_gr / bulk_rate : 999.0f;

    float full_gr = fmaxf(0.0f, handoff - taper_gr);
    float taper_run = fminf(handoff, taper_gr);
    float taper_rate = 0.5f * (fine_flow_max + fine_flow_min);
    float t = 999.0f;
    if (fine_flow_max > 0.0f && taper_rate > 0.0f) {
        t = full_gr / fine_flow_max + taper_run / taper_rate;
    }
    *fine_s = t + LEARN_SETTLE_ALLOWANCE_S;
}


static void fit_profile(void) {
    learn_result_t * r = &learn_mode.result;
    memset(r, 0, sizeof(*r));

    float bracket = charge_mode_get_active_bracket();
    float target = learn_mode.config.confirm_target;
    float goal = learn_mode.config.time_goal_s;

    r->coarse_k = fit_flow_per_rps(learn_mode.coarse, LEARN_THROWS_PER_PHASE);
    r->fine_k = fit_flow_per_rps(learn_mode.fine, LEARN_THROWS_PER_PHASE);

    float lag_mean, lag_sd, coarse_lag_sd, fine_lag_sd, dead;
    lag_stats(&lag_mean, &lag_sd, &r->coarse_lag_s, &r->fine_lag_s, &coarse_lag_sd, &fine_lag_sd, &dead);
    r->lag_sd_s = lag_sd;
    r->dead_time_s = dead;
    r->lag_used_s = fminf(3.0f, fmaxf(0.0f, lag_mean));
    r->predict_used = (r->lag_used_s > 0.05f);
    r->coarse_tail_per_rps = r->coarse_lag_s * r->coarse_k;
    r->fine_tail_per_rps = r->fine_lag_s * r->fine_k;

    // With compensation on, the handoff only has to cover the error in the prediction, not the whole
    // tail. That error is the flow multiplied by how much the lag itself varies - each phase by its
    // own measured spread, not the combined figure (pooling coarse and fine's different mean lag
    // inflates that number well past either phase's real noise). Without compensation the handoff
    // has to swallow the entire tail, which is why it gets big and slow.
    float coarse_lag_err = r->predict_used ? fmaxf(coarse_lag_sd, 0.02f) : fmaxf(r->coarse_lag_s, 0.05f);
    float fine_lag_err = r->predict_used ? fmaxf(fine_lag_sd, 0.02f) : fmaxf(r->fine_lag_s, 0.05f);

    // The extra safety factor belongs on a scatter, not on a tail. With prediction on, the handoff
    // covers 3 sigma of stop scatter and the factor buys a cushion on top of a session's worth of
    // measurements. With prediction off it already has to swallow the whole tail at 3x, so stacking
    // the factor there would just make an uncompensated profile needlessly slow.
    float coarse_margin_mult = r->predict_used ? learn_mode.config.coarse_stop_safety : 1.0f;
    if (coarse_margin_mult < LEARN_COARSE_STOP_SAFETY_MIN) coarse_margin_mult = LEARN_COARSE_STOP_SAFETY_MIN;
    if (coarse_margin_mult > LEARN_COARSE_STOP_SAFETY_MAX) coarse_margin_mult = LEARN_COARSE_STOP_SAFETY_MAX;

    float c_motor_min = fmaxf(get_motor_min_speed(SELECT_COARSE_TRICKLER_MOTOR), 0.05f);
    float f_motor_min = fmaxf(get_motor_min_speed(SELECT_FINE_TRICKLER_MOTOR), 0.05f);
    float c_hi = fminf(learn_mode.config.coarse_speed_ceiling, (float) get_motor_max_speed(SELECT_COARSE_TRICKLER_MOTOR));
    float f_hi = fminf(learn_mode.config.fine_speed_ceiling, (float) get_motor_max_speed(SELECT_FINE_TRICKLER_MOTOR));

    // Fine landing speed: a couple of kernels a second, and slow enough that whatever is still
    // in the air at the very end fits well inside the bracket.
    float fmin_land = (r->fine_k > 0.0f) ? LEARN_FINE_LAND_FLOW_GPS / r->fine_k : f_motor_min;
    float land_budget = LEARN_BRACKET_USE_FRAC * bracket;
    float fmin_err = (r->fine_k > 0.0f && fine_lag_err > 0.0f) ? land_budget / (r->fine_k * fine_lag_err) : fmin_land;
    float fmin = fminf(fmin_land, fmin_err);
    if (fmin < f_motor_min) fmin = f_motor_min;
    if (fmin > f_hi) fmin = f_hi;
    r->fine_min_rps = fmin;
    float fine_flow_min = r->fine_k * fmin;

    // The fit may only pick a coarse speed the ladder actually characterised, plus a little. The
    // ladder stops at LEARN_COARSE_FLOW_CAP_GPS on purpose, so anything past that is coarse_k
    // extrapolated well outside its data - on tested hardware the ladder topped out near 1.7 rps
    // while the motor allows 5, and a fit reaching that far assumes a bulk rate three times faster
    // than anything measured.
    float c_measured_max = 0.0f;
    for (int i = 0; i < LEARN_THROWS_PER_PHASE; i += 1) {
        if (learn_mode.coarse[i].time_s > 0.0f && learn_mode.coarse[i].speed_rps > c_measured_max) {
            c_measured_max = learn_mode.coarse[i].speed_rps;
        }
    }
    if (c_measured_max > 0.0f) c_hi = fminf(c_hi, c_measured_max * LEARN_COARSE_EXTRAP_MULT);
    if (c_hi < c_motor_min) c_hi = c_motor_min;

    // Sweep both tubes. Running the fine flat out is close to free in time but expensive in handoff:
    // the taper window scales with fine flow, so the fine phase takes about the same number of
    // seconds at any fine speed (taper / mean taper rate ~ 6 x lag either way) while the handoff it
    // forces grows in proportion. Sweeping fine max alongside coarse max lets the fit spend fine
    // speed only where it actually buys bulk rate.
    //
    // Every point on the grid already carries the same margin - the handoff covers the configured
    // safety factor x the 3 sigma coarse stop scatter at that bulk rate - so the goal is to hand the
    // bulk as much of the charge as it can, i.e. the smallest handoff, and the time goal is what
    // holds that back. Take the tightest handoff that still makes the goal, fastest on a tie. Only
    // if nothing makes the goal does the fastest throw win instead.
    float best_c = c_motor_min, best_f = fmin, best_handoff = 0.0f, best_taper = LEARN_FINE_TAPER_MIN_GR;
    float best_cs = 0.0f, best_fs = 0.0f, best_total = 1e9f;
    bool found = false;

    // Fallback for when no combination meets the time goal: the fastest throw on the grid.
    float fb_c = c_motor_min, fb_f = fmin, fb_handoff = 0.0f, fb_taper = LEARN_FINE_TAPER_MIN_GR;
    float fb_cs = 0.0f, fb_fs = 0.0f, fb_total = 1e9f;

    for (int fi = 1; fi <= LEARN_SEARCH_STEPS; fi += 1) {
        float fmax = f_hi * (float) fi / (float) LEARN_SEARCH_STEPS;
        if (fmax < fmin) continue;
        float fine_flow_max = r->fine_k * fmax;

        // Taper window: wide enough to cover what is in the air at this fine speed, so the ramp
        // starts before the powder already committed would carry past the target. Sized on the fine
        // tube's own lag - the combined figure is pulled down by the coarse tube's shorter lag.
        float taper = fmaxf(LEARN_FINE_TAPER_MIN_GR, LEARN_FINE_TAPER_TAIL_MULT * r->fine_lag_s * fine_flow_max);

        for (int ci = 1; ci <= LEARN_SEARCH_STEPS; ci += 1) {
            float cmax = c_hi * (float) ci / (float) LEARN_SEARCH_STEPS;
            if (cmax < c_motor_min) continue;
            float coarse_flow = r->coarse_k * cmax;

            // Handoff has to cover the coarse stop scatter at this bulk speed, with a margin, and be
            // wide enough that the fine can run its ramp. Below the taper the fine simply hands off
            // part way down the ramp, which costs nothing, so the taper is a floor at 1x not 1.5x.
            float handoff = fmaxf(LEARN_COARSE_STOP_MIN_GR,
                                  coarse_margin_mult * 3.0f * coarse_flow * coarse_lag_err + LEARN_COARSE_STOP_MARGIN_GR);
            if (handoff < taper) handoff = taper;
            if (handoff >= 0.5f * target) continue;  // bulk has to carry at least half the charge

            float cs, fs;
            predict_throw(target, coarse_flow, fine_flow_max, fine_flow_min, handoff, taper, &cs, &fs);
            float total = cs + fs;

            if (total <= goal &&
                (!found || handoff < best_handoff - 0.001f ||
                 (handoff < best_handoff + 0.001f && total < best_total))) {
                found = true;
                best_c = cmax; best_f = fmax; best_handoff = handoff; best_taper = taper;
                best_cs = cs; best_fs = fs; best_total = total;
            }
            if (total < fb_total) {
                fb_c = cmax; fb_f = fmax; fb_handoff = handoff; fb_taper = taper;
                fb_cs = cs; fb_fs = fs; fb_total = total;
            }
        }
    }
    if (!found) {
        best_c = fb_c; best_f = fb_f; best_handoff = fb_handoff; best_taper = fb_taper;
        best_cs = fb_cs; best_fs = fb_fs; best_total = fb_total;
    }
    if (best_total >= 1e9f) {
        // Nothing on the grid was usable - every handoff the margins demanded was more than half the
        // charge. Fall back to the slowest bulk and the narrowest legal handoff rather than leaving
        // the threshold at zero, which would let the coarse tube run the whole way to target.
        best_c = c_motor_min;
        best_f = fmin;
        best_taper = fmaxf(LEARN_FINE_TAPER_MIN_GR, LEARN_FINE_TAPER_TAIL_MULT * r->fine_lag_s * r->fine_k * fmin);
        best_handoff = fmaxf(LEARN_COARSE_STOP_MIN_GR, best_taper);
        predict_throw(target, r->coarse_k * best_c, r->fine_k * best_f, fine_flow_min,
                      best_handoff, best_taper, &best_cs, &best_fs);
        best_total = best_cs + best_fs;
    }
    found = (best_total <= goal);

    float best_fine_flow_max = r->fine_k * best_f;
    r->fine_max_rps = best_f;
    r->fine_taper_gr = best_taper;
    r->fine_kp = (best_taper > 0.0f) ? (best_f / best_taper) : best_f;

    r->coarse_max_rps = best_c;
    r->coarse_min_rps = fminf(c_motor_min, best_c);
    r->coarse_stop_threshold = best_handoff;
    r->coarse_tail_at_max = r->coarse_lag_s * r->coarse_k * best_c;
    r->coarse_tail_sd_at_max = coarse_lag_sd * r->coarse_k * best_c;
    r->fine_tail_at_max = r->fine_lag_s * best_fine_flow_max;
    r->fine_tail_sd_at_max = fine_lag_sd * best_fine_flow_max;

    // Hold full coarse speed until the last half grain before the handoff
    r->coarse_kp = r->coarse_max_rps / 0.5f;

    r->predicted_coarse_s = best_cs;
    r->predicted_fine_s = best_fs;
    r->predicted_total_s = best_total;
    r->meets_time_goal = found;

    learn_mode.result_valid = true;
}


// Confirmation missed the success target: slow the landing, widen the handoff, rerun.
static void back_off_profile(void) {
    learn_result_t * r = &learn_mode.result;
    float f_motor_min = fmaxf(get_motor_min_speed(SELECT_FINE_TRICKLER_MOTOR), 0.05f);

    r->fine_min_rps = fmaxf(f_motor_min, r->fine_min_rps * 0.7f);
    r->fine_max_rps = fmaxf(r->fine_min_rps, r->fine_max_rps * 0.85f);
    r->fine_taper_gr = r->fine_taper_gr * 1.4f;
    r->fine_kp = r->fine_max_rps / r->fine_taper_gr;
    r->coarse_stop_threshold = r->coarse_stop_threshold * 1.3f;

    predict_throw(learn_mode.config.confirm_target,
                  r->coarse_k * r->coarse_max_rps,
                  r->fine_k * r->fine_max_rps,
                  r->fine_k * r->fine_min_rps,
                  r->coarse_stop_threshold, r->fine_taper_gr,
                  &r->predicted_coarse_s, &r->predicted_fine_s);
    r->predicted_total_s = r->predicted_coarse_s + r->predicted_fine_s;
    r->meets_time_goal = (r->predicted_total_s <= learn_mode.config.time_goal_s);
}


bool learn_mode_apply_to_profile(void) {
    if (!learn_mode.result_valid) return false;
    learn_result_t * r = &learn_mode.result;
    profile_t * p = profile_get_selected();

    p->coarse_kp = r->coarse_kp;
    p->coarse_ki = 0.0f;
    p->coarse_kd = 0.0f;
    p->coarse_min_flow_speed_rps = r->coarse_min_rps;
    p->coarse_max_flow_speed_rps = r->coarse_max_rps;

    p->fine_kp = r->fine_kp;
    p->fine_ki = 0.0f;
    p->fine_kd = 0.0f;
    p->fine_min_flow_speed_rps = r->fine_min_rps;
    p->fine_max_flow_speed_rps = r->fine_max_rps;

    charge_mode_config.eeprom_charge_mode_data.coarse_stop_threshold = r->coarse_stop_threshold;
    charge_mode_config.eeprom_charge_mode_data.predict_enable = r->predict_used;
    if (r->predict_used) {
        // Each phase gets its own measured lag rather than one blended value - coarse and fine
        // have consistently measured different lag on tested hardware (~0.53s vs ~0.70-0.74s).
        charge_mode_config.eeprom_charge_mode_data.coarse_lag_s = r->coarse_lag_s;
        charge_mode_config.eeprom_charge_mode_data.fine_lag_s = r->fine_lag_s;
    }
    charge_mode_config.eeprom_charge_mode_data.auto_lag_enable = true;

    // Floor for live per-throw tightening: it can narrow the handoff, but not past what the
    // coarse tube's own measured spread needs.
    charge_mode_config.eeprom_charge_mode_data.coarse_tail_sd_gr = r->coarse_tail_sd_at_max;

    learn_mode.applied_to_profile = true;
    return true;
}


// ---------------------------------------------------------------------------
// Confirmation: real charges with the fitted profile, pan stays on, auto zero between
// ---------------------------------------------------------------------------
static bool confirm_throws(void) {
    learn_result_t * r = &learn_mode.result;
    float bracket = charge_mode_get_active_bracket();
    uint8_t n = learn_mode.config.confirm_throws;
    if (n == 0) return true;
    if (n > 20) n = 20;

    charge_mode_config.target_charge_weight = learn_mode.config.confirm_target;
    r->confirm_total = 0;
    r->confirm_pass = 0;
    double sum_time = 0.0;

    for (uint8_t i = 0; i < n; i += 1) {
        learn_mode.throw_idx = i;
        if (!ensure_cup_room(learn_mode.config.confirm_target + 1.0f)) return false;
        char msg[24];
        snprintf(msg, sizeof(msg), "Charging R%d", (int) learn_mode.result.confirm_rounds);
        set_message(msg);

        charge_mode_config.charge_mode_state = CHARGE_MODE_WAIT_FOR_COMPLETE;
        charge_mode_wait_for_complete();
        if (charge_mode_config.charge_mode_state == CHARGE_MODE_EXIT) {
            learn_mode.state = LEARN_STATE_ABORTED;
            set_message("Stopped by user");
            return false;
        }

        set_message("Settling");
        float settled;
        if (!wait_for_stable(&settled)) return false;

        learn_mode.cup_load_gr += settled;
        float err = settled - learn_mode.config.confirm_target;
        throw_result_t result = THROW_RESULT_PASS;
        if (err > bracket + 0.0005f) result = THROW_RESULT_OVER;
        else if (err < -(bracket + 0.0005f)) result = THROW_RESULT_UNDER;

        float t = charge_mode_get_last_elapsed_seconds();
        r->confirm_total += 1;
        if (result == THROW_RESULT_PASS) r->confirm_pass += 1;
        sum_time += t;

        session_stats_record(learn_mode.config.confirm_target, settled, t, 0.0f, bracket, result,
                             (uint8_t) profile_data.current_profile_idx,
                             charge_mode_config.eeprom_charge_mode_data.bracket_mode);

        // Flash the result on the LEDs
        rgbw_u32_t c = (result == THROW_RESULT_PASS) ? charge_mode_config.eeprom_charge_mode_data.neopixel_normal_charge_colour
                     : (result == THROW_RESULT_OVER) ? charge_mode_config.eeprom_charge_mode_data.neopixel_over_charge_colour
                     : charge_mode_config.eeprom_charge_mode_data.neopixel_under_charge_colour;
        neopixel_led_set_colour(charge_mode_config.eeprom_charge_mode_data.neopixel_session_backlight_colour, c, c, true);

        if (!auto_zero()) return false;
    }

    r->confirm_avg_time = (r->confirm_total > 0) ? (float)(sum_time / r->confirm_total) : 0.0f;
    return true;
}


// ---------------------------------------------------------------------------
// Main routine
// ---------------------------------------------------------------------------
uint8_t learn_mode_menu(void) {
    // Reset run state
    learn_mode.state = LEARN_STATE_WAIT_FOR_ZERO;
    learn_mode.throw_idx = 0;
    learn_mode.current_speed = 0.0f;
    learn_mode.result_valid = false;
    learn_mode.applied_to_profile = false;
    learn_mode.cup_load_gr = 0.0f;
    max_settled_coarse = 0.0f;
    max_settled_fine = 0.0f;
    memset(learn_mode.coarse, 0, sizeof(learn_mode.coarse));
    memset(learn_mode.fine, 0, sizeof(learn_mode.fine));
    set_message("");

    if (learn_render_task_handler == NULL) {
        UBaseType_t prio = uxTaskPriorityGet(xTaskGetCurrentTaskHandle());
        xTaskCreate(learn_render_task, "Learn Render Task", configMINIMAL_STACK_SIZE, NULL, prio - 1, &learn_render_task_handler);
    }
    else {
        vTaskResume(learn_render_task_handler);
    }

    neopixel_led_set_colour(charge_mode_config.eeprom_charge_mode_data.neopixel_session_backlight_colour,
                            charge_mode_config.eeprom_charge_mode_data.neopixel_not_ready_colour,
                            charge_mode_config.eeprom_charge_mode_data.neopixel_not_ready_colour, true);

    motor_enable(SELECT_COARSE_TRICKLER_MOTOR, true);
    motor_enable(SELECT_FINE_TRICKLER_MOTOR, true);

    float coarse_cap = fminf(learn_mode.config.coarse_speed_ceiling, (float) get_motor_max_speed(SELECT_COARSE_TRICKLER_MOTOR));
    float fine_cap = fminf(learn_mode.config.fine_speed_ceiling, (float) get_motor_max_speed(SELECT_FINE_TRICKLER_MOTOR));
    float c_motor_min = fmaxf(get_motor_min_speed(SELECT_COARSE_TRICKLER_MOTOR), 0.05f);
    float f_motor_min = fmaxf(get_motor_min_speed(SELECT_FINE_TRICKLER_MOTOR), 0.05f);

    bool ok = auto_zero();

    // Probe throw on each tube first. One short run tells us the flow per rps, which is what sets
    // the speeds for the rest of the phase. Without it the ladder is guessing at how fast the tube is.
    learn_throw_t probe;
    float coarse_k = 0.0f, fine_k = 0.0f;

    if (ok) {
        learn_mode.state = LEARN_STATE_COARSE;
        learn_mode.throw_idx = 0;
        set_message("Probe");
        float probe_speed = fmaxf(c_motor_min, 0.25f * coarse_cap);
        ok = throw_for_time(SELECT_COARSE_TRICKLER_MOTOR, probe_speed, 1.0f, 10.0f, &max_settled_coarse, &probe);
        if (ok) {
            coarse_k = probe.flow_gps / probe_speed;
            if (coarse_k <= 0.0f) coarse_k = 1.0f;
        }
    }

    // Coarse ladder in flow, not in speed. Four bulk rates spread across what the tube can do,
    // capped so a throw stays a sensible size for the cup.
    if (ok) {
        float flow_top = fminf(coarse_k * coarse_cap, LEARN_COARSE_FLOW_CAP_GPS);
        const float flow_frac[LEARN_COARSE_SPEED_LEVELS] = {0.20f, 0.45f, 0.70f, 1.00f};
        int per_level = LEARN_THROWS_PER_PHASE / LEARN_COARSE_SPEED_LEVELS;

        for (int i = 0; i < LEARN_THROWS_PER_PHASE && ok; i += 1) {
            learn_mode.throw_idx = i;
            float want_flow = flow_top * flow_frac[i / per_level];
            float speed = want_flow / coarse_k;
            if (speed < c_motor_min) speed = c_motor_min;
            if (speed > coarse_cap) speed = coarse_cap;
            float flow = coarse_k * speed;

            // Long enough that the reading is well past the lag, short enough to keep the cup sane
            float run_s = learn_mode.config.coarse_target / fmaxf(flow, 0.1f);
            if (run_s < LEARN_COARSE_MIN_RUN_S) run_s = LEARN_COARSE_MIN_RUN_S;
            if (run_s > LEARN_COARSE_MAX_RUN_S) run_s = LEARN_COARSE_MAX_RUN_S;

            ok = throw_for_time(SELECT_COARSE_TRICKLER_MOTOR, speed, run_s, flow * run_s, &max_settled_coarse, &learn_mode.coarse[i]);
            if (ok && learn_mode.coarse[i].flow_gps > 0.0f) {
                coarse_k = 0.7f * coarse_k + 0.3f * (learn_mode.coarse[i].flow_gps / speed);
            }
        }
    }

    if (ok) {
        learn_mode.state = LEARN_STATE_FINE;
        learn_mode.throw_idx = 0;
        set_message("Probe");
        float probe_speed = fmaxf(f_motor_min, 0.40f * fine_cap);
        ok = throw_for_time(SELECT_FINE_TRICKLER_MOTOR, probe_speed, 4.0f, 2.0f, &max_settled_fine, &probe);
        if (ok) {
            fine_k = probe.flow_gps / probe_speed;
            if (fine_k <= 0.0f) fine_k = 0.1f;
        }
    }

    if (ok) {
        float f_flow_top = fine_k * fine_cap;
        const float flow_frac[LEARN_FINE_SPEED_LEVELS] = {0.15f, 0.45f, 1.00f};
        int per_level = LEARN_THROWS_PER_PHASE / LEARN_FINE_SPEED_LEVELS;

        for (int i = 0; i < LEARN_THROWS_PER_PHASE && ok; i += 1) {
            learn_mode.throw_idx = i;
            float want_flow = f_flow_top * flow_frac[i / per_level];
            float speed = want_flow / fine_k;
            if (speed < f_motor_min) speed = f_motor_min;
            if (speed > fine_cap) speed = fine_cap;
            float flow = fine_k * speed;

            float run_s = learn_mode.config.fine_target / fmaxf(flow, 0.01f);
            if (run_s < LEARN_FINE_MIN_RUN_S) run_s = LEARN_FINE_MIN_RUN_S;
            if (run_s > LEARN_FINE_MAX_RUN_S) run_s = LEARN_FINE_MAX_RUN_S;

            ok = throw_for_time(SELECT_FINE_TRICKLER_MOTOR, speed, run_s, flow * run_s, &max_settled_fine, &learn_mode.fine[i]);
            if (ok && learn_mode.fine[i].flow_gps > 0.0f) {
                fine_k = 0.7f * fine_k + 0.3f * (learn_mode.fine[i].flow_gps / speed);
            }
        }
    }

    // Fit and apply to the selected profile in RAM
    if (ok) {
        learn_mode.state = LEARN_STATE_FIT;
        set_message("Fitting");
        fit_profile();
        learn_mode_apply_to_profile();
        vTaskDelay(pdMS_TO_TICKS(800));
    }

    // Confirmation throws with the fitted profile. Cup gets dumped first, every time.
    // If the set misses the success target, back the profile off and run another set, up to 3 rounds.
    if (ok) {
        learn_mode.state = LEARN_STATE_CONFIRM;
        learn_mode.throw_idx = 0;
        ok = empty_cup_and_zero();
    }
    learn_mode.result.confirm_rounds = 0;
    learn_mode.result.confirm_met = false;
    while (ok && learn_mode.result.confirm_rounds < LEARN_MAX_CONFIRM_ROUNDS) {
        learn_mode.result.confirm_rounds += 1;
        ok = confirm_throws();
        if (!ok) break;
        if (learn_mode.config.confirm_throws == 0) { learn_mode.result.confirm_met = true; break; }
        float rate = 100.0f * (float) learn_mode.result.confirm_pass / (float) learn_mode.result.confirm_total;
        if (rate + 0.01f >= learn_mode.config.min_success_pct) {
            learn_mode.result.confirm_met = true;
            break;
        }
        if (learn_mode.result.confirm_rounds < LEARN_MAX_CONFIRM_ROUNDS) {
            back_off_profile();
            learn_mode_apply_to_profile();
            set_message("Backing off");
            vTaskDelay(pdMS_TO_TICKS(800));
        }
    }

    motor_set_speed(SELECT_COARSE_TRICKLER_MOTOR, 0);
    motor_set_speed(SELECT_FINE_TRICKLER_MOTOR, 0);

    if (ok) {
        learn_mode.state = LEARN_STATE_DONE;
        set_message("Done");
        neopixel_led_set_colour(charge_mode_config.eeprom_charge_mode_data.neopixel_session_backlight_colour,
                                charge_mode_config.eeprom_charge_mode_data.neopixel_normal_charge_colour,
                                charge_mode_config.eeprom_charge_mode_data.neopixel_normal_charge_colour, true);

        // Knob saves profile + coarse stop threshold to EEPROM, RST leaves them in RAM only
        while (true) {
            ButtonEncoderEvent_t ev = button_wait_for_input(true);
            if (ev == BUTTON_ENCODER_PRESSED) {
                profile_data_save();
                charge_mode_config_save();
                set_message("Saved");
                vTaskDelay(pdMS_TO_TICKS(600));
                break;
            }
            else if (ev == BUTTON_RST_PRESSED || ev == OVERRIDE_FROM_REST) {
                break;
            }
        }
    }
    else {
        // Error or abort: wait for RST so the message can be read
        neopixel_led_set_colour(charge_mode_config.eeprom_charge_mode_data.neopixel_session_backlight_colour,
                                charge_mode_config.eeprom_charge_mode_data.neopixel_over_charge_colour,
                                charge_mode_config.eeprom_charge_mode_data.neopixel_over_charge_colour, true);
        while (true) {
            ButtonEncoderEvent_t ev = button_wait_for_input(true);
            if (ev == BUTTON_RST_PRESSED || ev == BUTTON_ENCODER_PRESSED || ev == OVERRIDE_FROM_REST) break;
        }
    }

    neopixel_led_set_colour(neopixel_led_config.eeprom_neopixel_led_metadata.default_led_colours.mini12864_backlight_colour,
                            neopixel_led_config.eeprom_neopixel_led_metadata.default_led_colours.led1_colour,
                            neopixel_led_config.eeprom_neopixel_led_metadata.default_led_colours.led2_colour, true);

    vTaskSuspend(learn_render_task_handler);
    motor_enable(SELECT_COARSE_TRICKLER_MOTOR, false);
    motor_enable(SELECT_FINE_TRICKLER_MOTOR, false);

    charge_mode_config.charge_mode_state = CHARGE_MODE_EXIT;
    learn_mode.state = ok ? LEARN_STATE_DONE : learn_mode.state;

    return 1;
}


bool learn_mode_config_save(void) {
    return save_config(EEPROM_LEARN_CONFIG_BASE_ADDR, &learn_mode.config, sizeof(learn_mode.config));
}


bool learn_mode_init(void) {
    memset(&learn_mode, 0, sizeof(learn_mode));
    learn_mode.state = LEARN_STATE_IDLE;

    bool is_ok = load_config(EEPROM_LEARN_CONFIG_BASE_ADDR, &learn_mode.config, &default_learn_config, sizeof(learn_mode.config), EEPROM_LEARN_CONFIG_REV);
    if (!is_ok) {
        learn_mode.config = default_learn_config;
    }
    if (learn_mode.config.cup_capacity_gr < 20.0f) learn_mode.config.cup_capacity_gr = default_learn_config.cup_capacity_gr;
    if (learn_mode.config.time_goal_s < 1.0f) learn_mode.config.time_goal_s = default_learn_config.time_goal_s;
    if (learn_mode.config.coarse_target <= 0.0f) learn_mode.config.coarse_target = default_learn_config.coarse_target;
    if (learn_mode.config.fine_target <= 0.0f) learn_mode.config.fine_target = default_learn_config.fine_target;
    if (learn_mode.config.min_success_pct < 50.0f || learn_mode.config.min_success_pct > 100.0f) learn_mode.config.min_success_pct = default_learn_config.min_success_pct;
    if (learn_mode.config.coarse_stop_safety < LEARN_COARSE_STOP_SAFETY_MIN ||
        learn_mode.config.coarse_stop_safety > LEARN_COARSE_STOP_SAFETY_MAX) {
        learn_mode.config.coarse_stop_safety = default_learn_config.coarse_stop_safety;
    }

    eeprom_register_handler(learn_mode_config_save);
    return true;
}


// ---------------------------------------------------------------------------
// REST
// ---------------------------------------------------------------------------
bool http_rest_learn_config(struct fs_file *file, int num_params, char *params[], char *values[]) {
    // l0 (float): coarse target
    // l1 (float): fine target
    // l2 (float): coarse speed ceiling rps
    // l3 (float): fine speed ceiling rps
    // l4 (float): confirm target
    // l5 (int): confirm throws
    // l6 (float): time goal s
    // l7 (float): cup capacity gr
    // l8 (float): min success pct
    // l9 (float): coarse stop safety factor
    // ee (bool): save to eeprom
    static char json_buffer[256];
    bool save_to_eeprom = false;

    for (int idx = 0; idx < num_params; idx += 1) {
        if (strcmp(params[idx], "l0") == 0)      learn_mode.config.coarse_target = strtof(values[idx], NULL);
        else if (strcmp(params[idx], "l1") == 0) learn_mode.config.fine_target = strtof(values[idx], NULL);
        else if (strcmp(params[idx], "l2") == 0) learn_mode.config.coarse_speed_ceiling = strtof(values[idx], NULL);
        else if (strcmp(params[idx], "l3") == 0) learn_mode.config.fine_speed_ceiling = strtof(values[idx], NULL);
        else if (strcmp(params[idx], "l4") == 0) learn_mode.config.confirm_target = strtof(values[idx], NULL);
        else if (strcmp(params[idx], "l5") == 0) learn_mode.config.confirm_throws = (uint8_t) atoi(values[idx]);
        else if (strcmp(params[idx], "l6") == 0) learn_mode.config.time_goal_s = strtof(values[idx], NULL);
        else if (strcmp(params[idx], "l7") == 0) learn_mode.config.cup_capacity_gr = strtof(values[idx], NULL);
        else if (strcmp(params[idx], "l8") == 0) learn_mode.config.min_success_pct = strtof(values[idx], NULL);
        else if (strcmp(params[idx], "l9") == 0) learn_mode.config.coarse_stop_safety = strtof(values[idx], NULL);
        else if (strcmp(params[idx], "ee") == 0) save_to_eeprom = string_to_boolean(values[idx]);
    }
    if (learn_mode.config.time_goal_s < 1.0f) learn_mode.config.time_goal_s = 1.0f;
    if (learn_mode.config.cup_capacity_gr < 20.0f) learn_mode.config.cup_capacity_gr = 20.0f;
    if (learn_mode.config.min_success_pct < 50.0f) learn_mode.config.min_success_pct = 50.0f;
    if (learn_mode.config.min_success_pct > 100.0f) learn_mode.config.min_success_pct = 100.0f;
    if (learn_mode.config.coarse_stop_safety < LEARN_COARSE_STOP_SAFETY_MIN) learn_mode.config.coarse_stop_safety = LEARN_COARSE_STOP_SAFETY_MIN;
    if (learn_mode.config.coarse_stop_safety > LEARN_COARSE_STOP_SAFETY_MAX) learn_mode.config.coarse_stop_safety = LEARN_COARSE_STOP_SAFETY_MAX;

    // Clamp before saving, so a bad value can't be written to EEPROM and reloaded next boot
    if (save_to_eeprom) {
        learn_mode_config_save();
    }

    snprintf(json_buffer, sizeof(json_buffer),
             "%s{\"l0\":%.3f,\"l1\":%.3f,\"l2\":%.2f,\"l3\":%.2f,\"l4\":%.3f,\"l5\":%d,\"l6\":%.1f,\"l7\":%.0f,\"l8\":%.0f,\"l9\":%.2f}",
             http_json_header,
             learn_mode.config.coarse_target, learn_mode.config.fine_target,
             learn_mode.config.coarse_speed_ceiling, learn_mode.config.fine_speed_ceiling,
             learn_mode.config.confirm_target, (int) learn_mode.config.confirm_throws,
             learn_mode.config.time_goal_s, learn_mode.config.cup_capacity_gr,
             learn_mode.config.min_success_pct, learn_mode.config.coarse_stop_safety);

    size_t len = strlen(json_buffer);
    file->data = json_buffer; file->len = len; file->index = len;
    file->flags = FS_FILE_FLAGS_HEADER_INCLUDED;
    return true;
}


// Every throw from the last learn run, for the portal table and CSV
bool http_rest_learn_throws(struct fs_file *file, int num_params, char *params[], char *values[]) {
    static char json_buffer[3200];
    int off = snprintf(json_buffer, sizeof(json_buffer), "%s{\"rows\":[", http_json_header);
    bool first = true;
    for (int phase = 0; phase < 2; phase += 1) {
        const learn_throw_t * t = (phase == 0) ? learn_mode.coarse : learn_mode.fine;
        for (int i = 0; i < LEARN_THROWS_PER_PHASE; i += 1) {
            if (t[i].time_s <= 0.0f) continue;
            int w = snprintf(json_buffer + off, sizeof(json_buffer) - off,
                             "%s[%d,%d,%.2f,%.2f,%.3f,%.3f,%.3f,%.3f,%.3f,%.2f,%.2f]",
                             first ? "" : ",", phase, i + 1, t[i].speed_rps, t[i].time_s,
                             t[i].stop_weight, t[i].settled_weight, t[i].flow_gps, t[i].tail,
                             t[i].rate_at_stop, t[i].lag_s, t[i].dead_time_s);
            if (w < 0 || (size_t)(off + w) >= sizeof(json_buffer) - 4) break;
            off += w;
            first = false;
        }
    }
    off += snprintf(json_buffer + off, sizeof(json_buffer) - off, "]}");
    file->data = json_buffer; file->len = off; file->index = off;
    file->flags = FS_FILE_FLAGS_HEADER_INCLUDED;
    return true;
}


bool http_rest_learn_state(struct fs_file *file, int num_params, char *params[], char *values[]) {
    // a0 (str): "start" | "stop" | "save"
    // Response: state, phase progress, current speed, live weight, message, result block
    static char json_buffer[1000];

    for (int idx = 0; idx < num_params; idx += 1) {
        if (strcmp(params[idx], "a0") == 0) {
            if (strcmp(values[idx], "start") == 0 &&
                learn_mode.state != LEARN_STATE_WAIT_FOR_ZERO && learn_mode.state != LEARN_STATE_COARSE &&
                learn_mode.state != LEARN_STATE_FINE && learn_mode.state != LEARN_STATE_CONFIRM &&
                charge_mode_config.charge_mode_state == CHARGE_MODE_EXIT) {
                exit_state = APP_STATE_ENTER_LEARN_MODE;
                ButtonEncoderEvent_t ev = OVERRIDE_FROM_REST;
                xQueueSend(encoder_event_queue, &ev, portMAX_DELAY);
            }
            else if (strcmp(values[idx], "stop") == 0) {
                ButtonEncoderEvent_t ev = BUTTON_RST_PRESSED;
                xQueueSend(encoder_event_queue, &ev, portMAX_DELAY);
            }
            else if (strcmp(values[idx], "save") == 0 && learn_mode.result_valid) {
                learn_mode_apply_to_profile();
                profile_data_save();
                charge_mode_config_save();
            }
        }
    }

    float m = scale_get_current_measurement();
    if (isnanf(m) || isinff(m)) m = -1.0f;

    const learn_result_t * r = &learn_mode.result;
    snprintf(json_buffer, sizeof(json_buffer),
             "%s{\"st\":%d,\"i\":%d,\"n\":%d,\"sp\":%.2f,\"w\":%.3f,\"msg\":\"%s\",\"ok\":%s,"
             "\"ck\":%.3f,\"ct\":%.3f,\"cts\":%.3f,\"fk\":%.4f,\"ft\":%.3f,\"fts\":%.3f,"
             "\"cmin\":%.2f,\"cmax\":%.2f,\"ckp\":%.3f,\"fmin\":%.2f,\"fmax\":%.2f,\"fkp\":%.3f,\"ftw\":%.3f,\"cst\":%.3f,"
             "\"cp\":%d,\"cn\":%d,\"cavg\":%.2f,\"pc\":%.2f,\"pf\":%.2f,\"pt\":%.2f,\"goal\":%s,\"cup\":%.1f,"
             "\"cr\":%d,\"met\":%s,\"ctk\":%.3f,\"ftk\":%.4f,\"clag\":%.2f,\"flag\":%.2f,\"lagu\":%.2f,\"pred\":%s,\"lagsd\":%.2f,\"dead\":%.2f}",
             http_json_header,
             (int) learn_mode.state,
             (int) learn_mode.throw_idx,
             (learn_mode.state == LEARN_STATE_CONFIRM) ? (int) learn_mode.config.confirm_throws : LEARN_THROWS_PER_PHASE,
             learn_mode.current_speed,
             m,
             learn_mode.message,
             boolean_to_string(learn_mode.result_valid),
             r->coarse_k, r->coarse_tail_at_max, r->coarse_tail_sd_at_max,
             r->fine_k, r->fine_tail_at_max, r->fine_tail_sd_at_max,
             r->coarse_min_rps, r->coarse_max_rps, r->coarse_kp,
             r->fine_min_rps, r->fine_max_rps, r->fine_kp, r->fine_taper_gr, r->coarse_stop_threshold,
             (int) r->confirm_pass, (int) r->confirm_total, r->confirm_avg_time,
             r->predicted_coarse_s, r->predicted_fine_s, r->predicted_total_s,
             boolean_to_string(r->meets_time_goal), learn_mode.cup_load_gr,
             (int) r->confirm_rounds, boolean_to_string(r->confirm_met), r->coarse_tail_per_rps, r->fine_tail_per_rps,
             r->coarse_lag_s, r->fine_lag_s, r->lag_used_s, boolean_to_string(r->predict_used),
             r->lag_sd_s, r->dead_time_s);

    size_t len = strlen(json_buffer);
    file->data = json_buffer; file->len = len; file->index = len;
    file->flags = FS_FILE_FLAGS_HEADER_INCLUDED;
    return true;
}
