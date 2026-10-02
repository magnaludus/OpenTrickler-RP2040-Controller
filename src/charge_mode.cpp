#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <FreeRTOS.h>
#include <queue.h>
#include <task.h>
#include <semphr.h>
#include <u8g2.h>
#include <math.h>

#include "app.h"
#include "FloatRingBuffer.h"
#include "mini_12864_module.h"
#include "display.h"
#include "scale.h"
#include "motors.h"
#include "charge_mode.h"
#include "eeprom.h"
#include "neopixel_led.h"
#include "profile.h"
#include "common.h"
#include "servo_gate.h"
#include "session_stats.h"
#include "learn_tuner.h"


uint8_t charge_weight_digits[] = {0, 0, 0, 0, 0};

charge_mode_config_t charge_mode_config;

// Scale related
extern scale_config_t scale_config;
extern servo_gate_t servo_gate;


const eeprom_charge_mode_data_t default_charge_mode_data = {
    .charge_mode_data_rev = 0,

    .coarse_stop_threshold = 5,
    .fine_stop_threshold = 0.03,

    .set_point_sd_margin = 0.02,
    .set_point_mean_margin = 0.02,
    .coarse_stop_gate_ratio = 0,   // NEW
    .coarse_stop_backoff_enable = false,
    .coarse_stop_backoff_turns = 0.25f,
    .coarse_stop_backoff_speed_rps = 1.0f,
    .decimal_places = DP_2,

    // Precharges
    .precharge_enable = false,
    .precharge_time_ms = 1000,
    .precharge_speed_rps = 2,

    // LED related
    .neopixel_normal_charge_colour = RGB_COLOUR_GREEN,        // green
    .neopixel_under_charge_colour = RGB_COLOUR_YELLOW,        // yellow
    .neopixel_over_charge_colour = RGB_COLOUR_RED,            // red
    .neopixel_not_ready_colour = RGB_COLOUR_BLUE,             // blue

    // Session / bracket
    .neopixel_session_backlight_colour = RGB_COLOUR_SESSION_AMBER,
    .bracket_mode = BRACKET_MODE_NORMAL,
    .normal_bracket_steps = 3,      // +/- 0.06 gr
    .match_bracket_steps = 1,       // +/- 0.02 gr
    .learn_enable = false,

    .predict_enable = false,
    .coarse_lag_s = 0.35f,
    .fine_lag_s = 0.35f,
    .auto_lag_enable = false,
    .coarse_tail_sd_gr = 0.0f,
};

// Configures
TaskHandle_t scale_measurement_render_task_handler = NULL;
static char title_string[30];

static TickType_t charge_start_tick = 0;
static float last_charge_elapsed_seconds = 0.0f;
static float last_coarse_elapsed_seconds = 0.0f;
static float last_coarse_stop_weight = 0.0f;
static bool throw_pending_record = false;   // a completed throw that has not been logged yet

// Lag measurement. At the moment the motors stop, the reading is climbing at rate_at_stop and is short
// by whatever is still in the air. lag = tail / rate_at_stop. One clean number per throw.
static float weight_at_stop = 0.0f;
static float rate_at_stop = 0.0f;
static float last_measured_lag = 0.0f;
static float last_dead_time_s = 0.0f;

float charge_mode_get_measured_lag(void) { return last_measured_lag; }
float charge_mode_get_dead_time(void) { return last_dead_time_s; }


// ---------------------------------------------------------------------------
// Bracket helpers
// ---------------------------------------------------------------------------
static uint8_t clamp_steps(uint8_t steps) {
    if (steps < BRACKET_STEPS_MIN) return BRACKET_STEPS_MIN;
    if (steps > BRACKET_STEPS_MAX) return BRACKET_STEPS_MAX;
    return steps;
}

float charge_mode_get_active_bracket(void) {
    uint8_t steps;
    if (charge_mode_config.eeprom_charge_mode_data.bracket_mode == BRACKET_MODE_MATCH) {
        steps = charge_mode_config.eeprom_charge_mode_data.match_bracket_steps;
    }
    else {
        steps = charge_mode_config.eeprom_charge_mode_data.normal_bracket_steps;
    }
    return clamp_steps(steps) * BRACKET_STEP_GRAINS;
}

void charge_mode_toggle_bracket_mode(void) {
    if (charge_mode_config.eeprom_charge_mode_data.bracket_mode == BRACKET_MODE_MATCH) {
        charge_mode_config.eeprom_charge_mode_data.bracket_mode = BRACKET_MODE_NORMAL;
    }
    else {
        charge_mode_config.eeprom_charge_mode_data.bracket_mode = BRACKET_MODE_MATCH;
    }
}

float charge_mode_get_last_elapsed_seconds(void) {
    return last_charge_elapsed_seconds;
}

const char * charge_mode_bracket_mode_name(void) {
    return charge_mode_config.eeprom_charge_mode_data.bracket_mode == BRACKET_MODE_MATCH ? "Match" : "Normal";
}

static rgbw_u32_t session_backlight(void) {
    return charge_mode_config.eeprom_charge_mode_data.neopixel_session_backlight_colour;
}


// ---------------------------------------------------------------------------
// Learn: live tuning of the selected profile after each throw.
//
// The tuner itself is src/learn_tuner.c, pure C with no hardware in it, so the same code runs in
// tests/tuner_sim.c against a simulated tube. This is only the adapter: it hands the profile over,
// takes the tuned profile back, and supplies the limits the tuner has to respect.
// Changes live in RAM. Save the profile from the portal to keep them.
// ---------------------------------------------------------------------------
#define HANDOFF_MAX_FRAC    0.50f   // the bulk always carries at least this share of the charge

extern "C" float learn_mode_get_land_sigma(void);
extern "C" float learn_mode_get_time_goal(void);
extern "C" bool learn_mode_is_aggressive(void);

static tuner_t learn_tuner[MAX_PROFILE_CNT];

// Learn Powder's confirmation throws measure the fitted profile. Tuning it while it is being
// measured means the pass rate no longer describes the profile the fit produced, and the profile
// that ends up saved is not the one on the results screen.
static bool learn_tuning_suppressed = false;

void charge_mode_learn_set_suppressed(bool suppressed) {
    learn_tuning_suppressed = suppressed;
}

// Learned landing trim for the profile in use. The charge loop ends the throw as soon as the raw
// reading is inside the bracket under the target, so a landing can finish anywhere in that window
// but can never be pulled back from above it: results sit low rather than centred. Shifting the
// fine tube's aim point by the bias the throws actually show centres the spread. Zero when Learn
// is off, and during Learn Powder's confirmation throws, which measure the profile as it stands.
static float learn_landing_trim(void) {
    if (!charge_mode_config.eeprom_charge_mode_data.learn_enable) return 0.0f;
    if (learn_tuning_suppressed) return 0.0f;
    extern eeprom_profile_data_t profile_data;
    uint8_t idx = (uint8_t) profile_data.current_profile_idx;
    if (idx >= MAX_PROFILE_CNT) return 0.0f;
    return tuner_trim(&learn_tuner[idx]);
}

// What the tuner is doing, for the portal. Empty when Learn is off.
extern "C" const char * charge_mode_learn_status(void) {
    static char status[40];
    status[0] = '\0';
    if (!charge_mode_config.eeprom_charge_mode_data.learn_enable) return status;
    extern eeprom_profile_data_t profile_data;
    uint8_t idx = (uint8_t) profile_data.current_profile_idx;
    if (idx >= MAX_PROFILE_CNT) return status;
    tuner_status(&learn_tuner[idx], status, sizeof(status));
    return status;
}

// The handoff can never be allowed to swallow the charge. At or above the target, the coarse target
// works out to zero, the coarse stop fires on the very first reading, and the fine tube delivers
// the whole charge on its own - roughly twice as slow, and the only sign is coarse_s logging 0.00.
// That happened on hardware: a handoff fitted or tuned at one charge weight carried over to a
// lighter one. Capped where it is used, so no stored value can idle the bulk.
static float handoff_for_target(float target) {
    float handoff = charge_mode_config.eeprom_charge_mode_data.coarse_stop_threshold;
    float cap = HANDOFF_MAX_FRAC * target;
    if (cap > 0.0f && handoff > cap) handoff = cap;
    if (handoff < 0.0f) handoff = 0.0f;
    return handoff;
}

// Runs after every throw when Learn is on.
static void learn_post_throw(uint8_t profile_idx, throw_result_t result, float bracket,
                             float total_s, float coarse_s, float coarse_stop_predicted,
                             float error_gr) {
    (void) coarse_s;
    if (!charge_mode_config.eeprom_charge_mode_data.learn_enable) {
        return;
    }
    if (learn_tuning_suppressed) {
        return;
    }
    if (profile_idx >= MAX_PROFILE_CNT) {
        return;
    }

    profile_t * profile = profile_get_selected();
    float target = charge_mode_config.target_charge_weight;
    float handoff = charge_mode_config.eeprom_charge_mode_data.coarse_stop_threshold;

    tuner_params_t params;
    params.fine_max = profile->fine_max_flow_speed_rps;
    params.fine_min = profile->fine_min_flow_speed_rps;
    params.fine_kp = profile->fine_kp;
    params.coarse_max = profile->coarse_max_flow_speed_rps;
    params.coarse_min = profile->coarse_min_flow_speed_rps;
    params.coarse_kp = profile->coarse_kp;
    params.handoff = handoff;

    tuner_env_t env;
    env.target = target;
    env.bracket = bracket;
    env.land_sigma = learn_mode_get_land_sigma();
    env.coarse_motor_cap = get_motor_max_speed(SELECT_COARSE_TRICKLER_MOTOR);
    env.fine_motor_cap = get_motor_max_speed(SELECT_FINE_TRICKLER_MOTOR);
    env.coarse_motor_min = get_motor_min_speed(SELECT_COARSE_TRICKLER_MOTOR);
    // Aggressive lets the handoff come down to 2 sigma of the coarse stop scatter (Normal: 3) and to
    // 0.15 gr (Normal: 0.30). Landing Sigma already arrives scaled for the style.
    const bool aggressive = learn_mode_is_aggressive();
    env.coarse_sd_floor_gr = (aggressive ? 2.0f : 3.0f) * charge_mode_config.eeprom_charge_mode_data.coarse_tail_sd_gr;
    env.handoff_floor_gr = aggressive ? 0.15f : 0.30f;
    env.aggressive = aggressive;
    env.handoff_cap_frac = HANDOFF_MAX_FRAC;
    env.time_goal_s = learn_mode_get_time_goal();

    // Where the bulk actually handed off, measured against where it was meant to. Compared on the
    // predicted weight, since that is what the coarse stop decision is made on. Comparing the raw
    // reading would blame the fine tube for every over, because the reading always trails.
    float handoff_target = target - handoff_for_target(target);
    tuner_obs_t obs;
    obs.result = (result == THROW_RESULT_OVER) ? TUNER_THROW_OVER
               : (result == THROW_RESULT_UNDER) ? TUNER_THROW_UNDER : TUNER_THROW_PASS;
    obs.total_s = total_s;
    obs.error_gr = error_gr;
    obs.coarse_ran_long = ((coarse_stop_predicted - handoff_target) > 0.25f * handoff);

    tuner_observe(&learn_tuner[profile_idx], &env, &obs, &params);

    profile->fine_max_flow_speed_rps = params.fine_max;
    profile->fine_min_flow_speed_rps = params.fine_min;
    profile->fine_kp = params.fine_kp;
    profile->coarse_max_flow_speed_rps = params.coarse_max;
    profile->coarse_min_flow_speed_rps = params.coarse_min;
    profile->coarse_kp = params.coarse_kp;
    charge_mode_config.eeprom_charge_mode_data.coarse_stop_threshold = params.handoff;
}

// Coarse trickler end-of-trickle backoff
#define COARSE_STOP_BACKOFF_MAX_MS 15000  // safety cap

static bool coarse_backoff_in_progress = false;
static TickType_t coarse_backoff_end_tick = 0;

static void coarse_trickler_backoff(float min_output_speed_rps) {
    if (!charge_mode_config.eeprom_charge_mode_data.coarse_stop_backoff_enable) {
        return;
    }

    float backoff_turns = charge_mode_config.eeprom_charge_mode_data.coarse_stop_backoff_turns;
    if (backoff_turns <= 0.0f) {
        return;
    }

    float configured_speed = charge_mode_config.eeprom_charge_mode_data.coarse_stop_backoff_speed_rps;
    float backoff_speed = configured_speed;
    if (backoff_speed <= 0.0f) {
        backoff_speed = min_output_speed_rps;
    }

    if (backoff_speed <= 0.0f) {
        return;
    }

    float backoff_time_s = backoff_turns / fabsf(backoff_speed);
    uint32_t backoff_time_ms = (uint32_t)(backoff_time_s * 1000.0f);

    if (backoff_time_ms == 0) {
        return;
    }

    if (backoff_time_ms > COARSE_STOP_BACKOFF_MAX_MS) {
        backoff_time_ms = COARSE_STOP_BACKOFF_MAX_MS;
    }

    motor_set_speed(SELECT_COARSE_TRICKLER_MOTOR, -fabsf(backoff_speed));
    coarse_backoff_in_progress = true;
    coarse_backoff_end_tick = xTaskGetTickCount() + pdMS_TO_TICKS(backoff_time_ms);
}


// Menu system
extern AppState_t exit_state;
extern QueueHandle_t encoder_event_queue;
extern neopixel_led_config_t neopixel_led_config;


// Definitions
typedef enum {
    CHARGE_MODE_EVENT_NO_EVENT = (1 << 0),
    CHARGE_MODE_EVENT_UNDER_CHARGE = (1 << 1),
    CHARGE_MODE_EVENT_OVER_CHARGE = (1 << 2),
} ChargeModeEventBit_t;


static void format_elapsed_time(char *buffer, size_t len, TickType_t start_tick) {
    TickType_t now = xTaskGetTickCount();
    uint32_t elapsed_ticks = now - start_tick;

    // Tick to milliseconds
    float elapsed_seconds = (float)(elapsed_ticks * portTICK_PERIOD_MS) / 1000.0f;

    snprintf(buffer, len, "%.2f s", elapsed_seconds);
}


void scale_measurement_render_task(void *p) {
    char current_weight_string[WEIGHT_STRING_LEN];
    char time_buffer[16];

    u8g2_t *display_handler = get_display_handler();

    while (true) {
        TickType_t last_render_tick = xTaskGetTickCount();

        u8g2_ClearBuffer(display_handler);

        // Set font for title and timer
        u8g2_SetFont(display_handler, u8g2_font_helvB08_tr);

        // Format the timer string based on current state
        if (charge_mode_config.charge_mode_state == CHARGE_MODE_WAIT_FOR_COMPLETE) {
            format_elapsed_time(time_buffer, sizeof(time_buffer), charge_start_tick);
        } else if (charge_mode_config.charge_mode_state == CHARGE_MODE_WAIT_FOR_CUP_REMOVAL ||
                   charge_mode_config.charge_mode_state == CHARGE_MODE_WAIT_FOR_CUP_RETURN ||
                   charge_mode_config.charge_mode_state == CHARGE_MODE_WAIT_FOR_ZERO) {
            snprintf(time_buffer, sizeof(time_buffer), "%.2f s", last_charge_elapsed_seconds);
        } else {
            snprintf(time_buffer, sizeof(time_buffer), "--.- s");
        }

        // Calculate x positions
        uint8_t screen_width = u8g2_GetDisplayWidth(display_handler);
        uint8_t time_width = u8g2_GetStrWidth(display_handler, time_buffer);

        // Draw title on left
        u8g2_DrawStr(display_handler, 5, 10, title_string);

        // Draw timer on right edge
        u8g2_DrawStr(display_handler, screen_width - time_width - 5, 10, time_buffer);  // 5 px padding from edge

        // Draw line under title
        u8g2_DrawHLine(display_handler, 0, 13, screen_width);

        // Auto-tune indicator: a small bullseye in the gap left of the weight digits,
        // shown only when both lag compensation and Learn's per-throw tuning are active -
        // together they're what's actually adjusting the profile while you charge.
        if (charge_mode_config.eeprom_charge_mode_data.predict_enable &&
            charge_mode_config.eeprom_charge_mode_data.learn_enable) {
            u8g2_DrawCircle(display_handler, 12, 24, 6, U8G2_DRAW_ALL);
            u8g2_DrawDisc(display_handler, 12, 24, 3, U8G2_DRAW_ALL);
        }

        // Current weight (only show values > -1.0)
        memset(current_weight_string, 0x0, sizeof(current_weight_string));
        float scale_measurement = scale_get_current_measurement();
        if (scale_measurement > -1.0) {
            float_to_string(current_weight_string, scale_measurement, charge_mode_config.eeprom_charge_mode_data.decimal_places);
        } else {
            strcpy(current_weight_string, "---");
        }

        // Draw current weight value
        u8g2_SetFont(display_handler, u8g2_font_profont22_tf);
        u8g2_DrawStr(display_handler, 26, 35, current_weight_string);

        // Bottom line: profile name on the left, session stats on the right
        profile_t *current_profile = profile_get_selected();
        u8g2_SetFont(display_handler, u8g2_font_helvR08_tr);
        u8g2_DrawStr(display_handler, 5, 61, current_profile->name);

        char stats_buffer[24];
        if (session_stats_summary()->total > 0) {
            snprintf(stats_buffer, sizeof(stats_buffer), "%.1fs %.0f%%",
                     session_stats_avg_time(), session_stats_success_rate());
        }
        else {
            snprintf(stats_buffer, sizeof(stats_buffer), "%c %.2f",
                     charge_mode_config.eeprom_charge_mode_data.bracket_mode == BRACKET_MODE_MATCH ? 'M' : 'N',
                     charge_mode_get_active_bracket());
        }
        uint8_t stats_width = u8g2_GetStrWidth(display_handler, stats_buffer);
        u8g2_DrawStr(display_handler, screen_width - stats_width - 3, 61, stats_buffer);

        // Bracket mode tag under the weight, small, so it is visible from the bench
        char mode_buffer[12];
        snprintf(mode_buffer, sizeof(mode_buffer), "%c%.2f",
                 charge_mode_config.eeprom_charge_mode_data.bracket_mode == BRACKET_MODE_MATCH ? 'M' : 'N',
                 charge_mode_get_active_bracket());
        u8g2_SetFont(display_handler, u8g2_font_5x7_tr);
        uint8_t mode_width = u8g2_GetStrWidth(display_handler, mode_buffer);
        u8g2_DrawStr(display_handler, screen_width - mode_width - 3, 47, mode_buffer);

        u8g2_SendBuffer(display_handler);

        vTaskDelayUntil(&last_render_tick, pdMS_TO_TICKS(20));
    }
}


void charge_mode_wait_for_zero() {
    // Set colour to not ready
    neopixel_led_set_colour(
        session_backlight(),
        charge_mode_config.eeprom_charge_mode_data.neopixel_not_ready_colour, 
        charge_mode_config.eeprom_charge_mode_data.neopixel_not_ready_colour, 
        true
    );
    
    // Wait for 5 measurements and wait for stable
    FloatRingBuffer data_buffer(10);

    // Update current status
    snprintf(title_string, sizeof(title_string), "Waiting for Zero");

    // Stop condition: 10 stable measurements in 200ms apart (2 seconds minimum)
    while (true) {
        TickType_t last_measurement_tick = xTaskGetTickCount();

        // Non block waiting for the input
        ButtonEncoderEvent_t button_encoder_event = button_wait_for_input(false);
        if (button_encoder_event == BUTTON_RST_PRESSED) {
            charge_mode_config.charge_mode_state = CHARGE_MODE_EXIT;
            return;
        }
        else if (button_encoder_event == BUTTON_ENCODER_PRESSED) {
            scale_config.scale_handle->force_zero();
        }

        // Perform measurement (max delay 300 seconds   )
        float current_measurement;
        if (scale_block_wait_for_next_measurement(300, &current_measurement)){
            data_buffer.enqueue(current_measurement);
        }

        // Generate stop condition
        if (data_buffer.getCounter() >= 10){
            if (data_buffer.getSd() < charge_mode_config.eeprom_charge_mode_data.set_point_sd_margin && 
                abs(data_buffer.getMean()) < charge_mode_config.eeprom_charge_mode_data.set_point_mean_margin) {
                break;
            }
        }

        // Wait for minimum 300 ms (but can skip if previously wait already)
        vTaskDelayUntil(&last_measurement_tick, pdMS_TO_TICKS(300));
    }

    charge_mode_config.charge_mode_state = CHARGE_MODE_WAIT_FOR_COMPLETE;
}

void charge_mode_wait_for_complete() {

    charge_start_tick = xTaskGetTickCount();
    coarse_backoff_in_progress = false;

    // Set colour to under charge
    neopixel_led_set_colour(
        session_backlight(),
        charge_mode_config.eeprom_charge_mode_data.neopixel_under_charge_colour, 
        charge_mode_config.eeprom_charge_mode_data.neopixel_under_charge_colour, 
        true
    );

    // If the servo gate is used then it has to be opened
    if (servo_gate.eeprom_servo_gate_config.servo_gate_enable) {
        servo_gate_set_ratio(SERVO_GATE_RATIO_OPEN, false);
    }
    // Update current status
    char target_weight_string[WEIGHT_STRING_LEN];
    float_to_string(target_weight_string, charge_mode_config.target_charge_weight, charge_mode_config.eeprom_charge_mode_data.decimal_places);

    snprintf(title_string, sizeof(title_string), 
             "Target: %s", 
             target_weight_string);

    // Read trickling parameter from the current profile
    profile_t * current_profile = profile_get_selected();

    // Find the minimum of max speed from the motor and the profile
    float coarse_trickler_max_speed = fmin(get_motor_max_speed(SELECT_COARSE_TRICKLER_MOTOR),
                                           current_profile->coarse_max_flow_speed_rps);
    float coarse_trickler_min_speed = fmax(get_motor_min_speed(SELECT_COARSE_TRICKLER_MOTOR),
                                           current_profile->coarse_min_flow_speed_rps);
    float fine_trickler_max_speed = fmin(get_motor_max_speed(SELECT_FINE_TRICKLER_MOTOR),
                                         current_profile->fine_max_flow_speed_rps);
    float fine_trickler_min_speed = fmax(get_motor_min_speed(SELECT_FINE_TRICKLER_MOTOR),
                                         current_profile->fine_min_flow_speed_rps);

    // Define PID terms
    float coarse_trickler_integral = 0.0f;
    float fine_trickler_integral = 0.0f;
    float coarse_trickler_last_error = 0.0f;
    float fine_trickler_last_error = 0.0f;

    // Calculate target weight for coarse trickler
    // The coarse trickler is suppose to stop ahead of the target weight by an offset
    float coarse_trickler_target_charge_weight = fmaxf(0.0f, charge_mode_config.target_charge_weight - handoff_for_target(charge_mode_config.target_charge_weight));

    // Where the fine tube aims: the target, plus whatever landing bias live tuning has learned
    float fine_aim_weight = charge_mode_config.target_charge_weight + learn_landing_trim();

    TickType_t last_sample_tick = xTaskGetTickCount();
    TickType_t last_valid_measurement_tick = last_sample_tick;
    TickType_t current_sample_tick = last_sample_tick;
    bool should_coarse_trickler_move = true;

    // The fine trickler stops once the reading is inside the bracket under the target (edge inclusive).
    // The small epsilon keeps float rounding from missing a reading that sits exactly on the edge.
    float fine_stop_threshold = charge_mode_get_active_bracket() + 0.0005f;
    TickType_t coarse_stop_tick = charge_start_tick;
    last_coarse_stop_weight = 0.0f;

    // Lag compensation: rate of climb from the readings, predicted weight = reading + rate x lag.
    // Coarse and fine measure differently (coarse ~0.53s, fine ~0.70-0.74s on tested hardware), so
    // each phase uses its own value rather than one blended number.
    bool predict = charge_mode_config.eeprom_charge_mode_data.predict_enable;
    float coarse_lag_s = fmaxf(0.0f, fminf(charge_mode_config.eeprom_charge_mode_data.coarse_lag_s, 3.0f));
    float fine_lag_s = fmaxf(0.0f, fminf(charge_mode_config.eeprom_charge_mode_data.fine_lag_s, 3.0f));
    float rate_gps = 0.0f;
    float last_weight = 0.0f;
    bool have_last_weight = false;
    float start_weight = 0.0f;
    bool have_start_weight = false;
    bool motion_seen = false;
    last_dead_time_s = 0.0f;

    while (true) {
        // Non block waiting for the input
        ButtonEncoderEvent_t button_encoder_event = button_wait_for_input(false);
        if (button_encoder_event == BUTTON_RST_PRESSED) {
            motor_set_speed(SELECT_BOTH_MOTOR, 0);
            charge_mode_config.charge_mode_state = CHARGE_MODE_EXIT;
            return;
        }

        // Run the PID controlled loop to start charging
        // Perform the measurement
        float current_weight;
        if (!scale_block_wait_for_next_measurement(200, &current_weight) || !isfinite(current_weight)) {
            if (xTaskGetTickCount() - last_valid_measurement_tick >= pdMS_TO_TICKS(CHARGE_SCALE_TIMEOUT_MS)) {
                motor_set_speed(SELECT_BOTH_MOTOR, 0);
                charge_mode_config.charge_mode_state = CHARGE_MODE_EXIT;
                return;
            }
            continue;
        }
        current_sample_tick = xTaskGetTickCount();
        last_valid_measurement_tick = current_sample_tick;

        // Handle coarse trickler backoff stop
        if (coarse_backoff_in_progress && (current_sample_tick >= coarse_backoff_end_tick)) {
            motor_set_speed(SELECT_COARSE_TRICKLER_MOTOR, 0);
            coarse_backoff_in_progress = false;
        }

        // Dead time: motor start to the first real movement on the scale
        if (!have_start_weight) {
            start_weight = current_weight;
            have_start_weight = true;
        }
        else if (!motion_seen && current_weight > start_weight + 0.10f) {
            motion_seen = true;
            last_dead_time_s = (float)((current_sample_tick - charge_start_tick) * portTICK_PERIOD_MS) / 1000.0f;
        }

        // Rate estimate, smoothed, never negative
        float dt_s = (float)((current_sample_tick - last_sample_tick) * portTICK_PERIOD_MS) / 1000.0f;
        if (have_last_weight && dt_s > 0.0f) {
            float inst = (current_weight - last_weight) / dt_s;
            if (inst < 0.0f) inst = 0.0f;
            rate_gps = 0.6f * rate_gps + 0.4f * inst;
        }
        last_weight = current_weight;
        have_last_weight = true;

        // Coarse is still moving until it hits its own stop condition below; use its lag until then.
        float active_lag_s = should_coarse_trickler_move ? coarse_lag_s : fine_lag_s;
        float predicted_weight = predict ? (current_weight + rate_gps * active_lag_s) : current_weight;

        // Motor decisions run on the predicted weight, the stop decision on the real reading
        float coarse_trickler_error = coarse_trickler_target_charge_weight - predicted_weight;
        // Both use the trimmed aim point. Trimming only the motor's aim would be undone by the stop
        // check, which would still end the throw at the untrimmed edge.
        float fine_trickler_error = fine_aim_weight - predicted_weight;
        float fine_reading_error = fine_aim_weight - current_weight;

        // Fine & Coarse trickler stop condition
        if (fine_reading_error < fine_stop_threshold) {
            // Stop all motors
            motor_set_speed(SELECT_FINE_TRICKLER_MOTOR, 0);
            motor_set_speed(SELECT_COARSE_TRICKLER_MOTOR, 0);

            weight_at_stop = current_weight;
            rate_at_stop = rate_gps;
            break;
        }

        // Coarse trickler stop condition
        else if (coarse_trickler_error <= 0 &&
                 should_coarse_trickler_move) {

            should_coarse_trickler_move = false;
            motor_set_speed(SELECT_COARSE_TRICKLER_MOTOR, 0);
            coarse_stop_tick = current_sample_tick;
            last_coarse_stop_weight = predicted_weight;   // what the bulk is believed to have delivered

            // NEW: When the coarse trickler stops, move the servo gate to a configured ratio
            // Ratio convention: 0.0 = open, 1.0 = close
            if (servo_gate.eeprom_servo_gate_config.servo_gate_enable) {
                float r = charge_mode_config.eeprom_charge_mode_data.coarse_stop_gate_ratio;

                servo_gate_set_ratio(r, false); // don't block the charge loop
            }

            // NEW: Reverse the coarse trickler for a short backoff (quarter turn).
            coarse_trickler_backoff(coarse_trickler_min_speed);
        }
    

        // Update fine trickler speed
        float elapse_time_ms = (current_sample_tick - last_sample_tick) / portTICK_RATE_MS;
        fine_trickler_integral += fine_trickler_error;
        float fine_trickler_derivative = (elapse_time_ms > 0.0f)
            ? (fine_trickler_error - fine_trickler_last_error) / elapse_time_ms : 0.0f;

        // Update fine trickler speed
        float new_p = current_profile->fine_kp * fine_trickler_error;
        float new_i = current_profile->fine_ki * fine_trickler_integral;
        float new_d = current_profile->fine_kd * fine_trickler_derivative;
        float new_speed = fmaxf(fine_trickler_min_speed, fminf(new_p + new_i + new_d, fine_trickler_max_speed));
        if (predict && fine_trickler_error <= 0.0f) {
            // Powder already in the air is expected to reach the target. Hold and let the reading catch up.
            // If it settles short, the rate decays, the prediction drops back to the reading, and this resumes.
            new_speed = 0.0f;
        }
        motor_set_speed(SELECT_FINE_TRICKLER_MOTOR, new_speed);

        // Update coarse trickler speed
        if (should_coarse_trickler_move && !coarse_backoff_in_progress) {
            coarse_trickler_integral += coarse_trickler_error;
            float coarse_trickler_derivative = (elapse_time_ms > 0.0f)
                ? (coarse_trickler_error - coarse_trickler_last_error) / elapse_time_ms : 0.0f;

            new_p = current_profile->coarse_kp * coarse_trickler_error;
            new_i = current_profile->coarse_ki * coarse_trickler_integral;
            new_d = current_profile->coarse_kd * coarse_trickler_derivative ;

            new_speed = fmaxf(coarse_trickler_min_speed, fminf(new_p + new_i + new_d, coarse_trickler_max_speed));

            motor_set_speed(SELECT_COARSE_TRICKLER_MOTOR, new_speed);
        }

        // Record state
        last_sample_tick = current_sample_tick;
        fine_trickler_last_error = fine_trickler_error;
        coarse_trickler_last_error = coarse_trickler_error;
    }

    // Stop the timer 
    TickType_t now = xTaskGetTickCount();
    TickType_t elapsed_ticks = now - charge_start_tick;
    last_charge_elapsed_seconds = (float)(elapsed_ticks * portTICK_PERIOD_MS) / 1000.0f;
    if (should_coarse_trickler_move) {
        // Coarse never reached its stop point (small charge), count the whole throw as coarse
        coarse_stop_tick = now;
        last_coarse_stop_weight = charge_mode_config.target_charge_weight;
    }
    last_coarse_elapsed_seconds = (float)((coarse_stop_tick - charge_start_tick) * portTICK_PERIOD_MS) / 1000.0f;
    throw_pending_record = true;

    // Close the gate if the servo gate is present
    if (servo_gate.eeprom_servo_gate_config.servo_gate_enable) {
    servo_gate_set_ratio(SERVO_GATE_RATIO_CLOSED, true);
    }

    // Precharge
    if (charge_mode_config.eeprom_charge_mode_data.precharge_enable &&
    servo_gate.eeprom_servo_gate_config.servo_gate_enable) {
        // Set a fixed delay between closing the gate and precharge to allow the gate to fully close
        vTaskDelay(pdMS_TO_TICKS(500));

        // Start the pre-charge
        motor_set_speed(SELECT_COARSE_TRICKLER_MOTOR, charge_mode_config.eeprom_charge_mode_data.precharge_speed_rps);
        vTaskDelay(pdMS_TO_TICKS(charge_mode_config.eeprom_charge_mode_data.precharge_time_ms));

        motor_set_speed(SELECT_COARSE_TRICKLER_MOTOR, 0);
    }
    else {
        vTaskDelay(pdMS_TO_TICKS(20));  // Wait for other tasks to complete  
    }

    charge_mode_config.charge_mode_state = CHARGE_MODE_WAIT_FOR_CUP_REMOVAL;
}

// Wait for the reading to hold still before judging a throw. Returns the settled mean.
// Gives up after the timeout and returns the latest reading, so a twitchy scale cannot hang the loop.
static bool charge_mode_wait_for_settle(float * settled, uint32_t timeout_ms) {
    FloatRingBuffer data_buffer(6);
    TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(timeout_ms);
    float latest = scale_get_current_measurement();

    while (xTaskGetTickCount() < deadline) {
        TickType_t last_tick = xTaskGetTickCount();
        ButtonEncoderEvent_t ev = button_wait_for_input(false);
        if (ev == BUTTON_RST_PRESSED) {
            charge_mode_config.charge_mode_state = CHARGE_MODE_EXIT;
            return false;
        }
        float m;
        if (scale_block_wait_for_next_measurement(200, &m)) {
            latest = m;
            data_buffer.enqueue(m);
        }
        if (data_buffer.getCounter() >= 6 &&
            data_buffer.getSd() < charge_mode_config.eeprom_charge_mode_data.set_point_sd_margin) {
            *settled = data_buffer.getMean();
            return true;
        }
        vTaskDelayUntil(&last_tick, pdMS_TO_TICKS(150));
    }
    *settled = latest;
    return true;
}


// Short throw: run the fine tube at its landing speed until the reading is inside the bracket.
// Same rule the charge loop stops on, just slower. Returns false on RST.
static bool charge_mode_top_up(float bracket) {
    profile_t * current_profile = profile_get_selected();
    float speed = fmaxf(get_motor_min_speed(SELECT_FINE_TRICKLER_MOTOR), current_profile->fine_min_flow_speed_rps);
    if (speed <= 0.0f) speed = 0.1f;

    snprintf(title_string, sizeof(title_string), "Top Up");
    float aim = charge_mode_config.target_charge_weight + learn_landing_trim();
    TickType_t start = xTaskGetTickCount();
    TickType_t last_valid_measurement_tick = start;
    TickType_t deadline = start + pdMS_TO_TICKS(15000);
    motor_set_speed(SELECT_FINE_TRICKLER_MOTOR, speed);

    bool ok = true;
    while (true) {
        ButtonEncoderEvent_t ev = button_wait_for_input(false);
        if (ev == BUTTON_RST_PRESSED) {
            charge_mode_config.charge_mode_state = CHARGE_MODE_EXIT;
            ok = false;
            break;
        }
        if (xTaskGetTickCount() > deadline) break;
        float m;
        if (!scale_block_wait_for_next_measurement(200, &m) || !isfinite(m)) {
            if (xTaskGetTickCount() - last_valid_measurement_tick >= pdMS_TO_TICKS(CHARGE_SCALE_TIMEOUT_MS)) {
                charge_mode_config.charge_mode_state = CHARGE_MODE_EXIT;
                ok = false;
                break;
            }
            continue;
        }
        last_valid_measurement_tick = xTaskGetTickCount();
        if (aim - m < bracket + 0.0005f) break;
    }
    motor_set_speed(SELECT_FINE_TRICKLER_MOTOR, 0);
    last_charge_elapsed_seconds += (float)((xTaskGetTickCount() - start) * portTICK_PERIOD_MS) / 1000.0f;
    return ok;
}


void charge_mode_wait_for_cup_removal() {
    // Update current status
    snprintf(title_string, sizeof(title_string), "Settling");

    FloatRingBuffer data_buffer(5);
    float bracket = charge_mode_get_active_bracket();

    // Let the scale finish reporting what is already in the cup before judging anything
    float current_measurement;
    if (!charge_mode_wait_for_settle(&current_measurement, 5000)) return;
    float error = charge_mode_config.target_charge_weight - current_measurement;

    // Measure the lag from this throw: what landed after the motors stopped, divided by how fast
    // the reading was climbing at that moment. Needs a real rate to divide by, so skip slow endings.
    if (rate_at_stop > 0.05f) {
        float tail = current_measurement - weight_at_stop;
        if (tail >= 0.0f) {
            float measured = tail / rate_at_stop;
            if (measured < 5.0f) {
                last_measured_lag = measured;
                // Only meaningful with lag compensation OFF. With it on, the fine motor is
                // stopped early on purpose - when predicted weight reaches target, not when the
                // reading does - so by the time the loop exits, the motor has been off for a
                // while and most of the tail has already landed. What is left is the residual
                // *after* compensation did its job, not the transport lag. Feeding that back in
                // makes the learned lag shrink, which weakens compensation, which shrinks it
                // further: the better compensation works, the smaller the lag looks. On tested
                // hardware it walked fine_lag_s from a fitted 0.70s down to 0.49s, which put
                // +0.056gr of uncompensated powder into every charge - measured mean error was
                // +0.060gr, and 13 of 20 throws went over with not one landing under.
                if (charge_mode_config.eeprom_charge_mode_data.auto_lag_enable &&
                    !charge_mode_config.eeprom_charge_mode_data.predict_enable) {
                    // Taken at the throw's final settle, downstream of the fine phase, so it reads
                    // fine's lag rather than coarse's. Only fine_lag_s tracks it; coarse_lag_s
                    // moves only when Learn Powder is re-run.
                    // Slow tracking so one odd throw cannot move it far.
                    float current = charge_mode_config.eeprom_charge_mode_data.fine_lag_s;
                    float updated = 0.8f * current + 0.2f * measured;
                    if (updated < 0.0f) updated = 0.0f;
                    if (updated > 3.0f) updated = 3.0f;
                    charge_mode_config.eeprom_charge_mode_data.fine_lag_s = updated;
                }
            }
        }
    }
    rate_at_stop = 0.0f;

    // Short by more than the bracket: trickle it up, up to twice, then judge for real
    for (int attempt = 0; attempt < 2 && error > bracket + 0.0005f; attempt += 1) {
        if (!charge_mode_top_up(bracket)) return;
        if (!charge_mode_wait_for_settle(&current_measurement, 5000)) return;
        error = charge_mode_config.target_charge_weight - current_measurement;
    }

    snprintf(title_string, sizeof(title_string), "Remove Cup");
    const float bracket_eps = 0.0005f;   // keep a reading sitting exactly on the edge inside the bracket
    throw_result_t throw_result = THROW_RESULT_PASS;

    // Update LED colour before moving to the next stage
    // Over charged
    if (error < -(bracket + bracket_eps)) {
        throw_result = THROW_RESULT_OVER;
        neopixel_led_set_colour(
            session_backlight(),
            charge_mode_config.eeprom_charge_mode_data.neopixel_over_charge_colour, 
            charge_mode_config.eeprom_charge_mode_data.neopixel_over_charge_colour, 
            true
        );

        // Set over charge
        charge_mode_config.charge_mode_event |= CHARGE_MODE_EVENT_OVER_CHARGE;
    }
    // Under charged
    else if (error > (bracket + bracket_eps)) {
        throw_result = THROW_RESULT_UNDER;
        neopixel_led_set_colour(
            session_backlight(), 
            charge_mode_config.eeprom_charge_mode_data.neopixel_under_charge_colour, 
            charge_mode_config.eeprom_charge_mode_data.neopixel_under_charge_colour, 
            true
        );

        // Set under charge flag
        charge_mode_config.charge_mode_event |= CHARGE_MODE_EVENT_UNDER_CHARGE;

    }
    // Normal
    else {
        neopixel_led_set_colour(
            session_backlight(), 
            charge_mode_config.eeprom_charge_mode_data.neopixel_normal_charge_colour, 
            charge_mode_config.eeprom_charge_mode_data.neopixel_normal_charge_colour, 
            true
        );

        // Clear over and under charge bit
        charge_mode_config.charge_mode_event &= ~(CHARGE_MODE_EVENT_UNDER_CHARGE | CHARGE_MODE_EVENT_OVER_CHARGE);
    }

    // Log the throw once, then let the learner nudge the profile for the next one
    if (throw_pending_record) {
        throw_pending_record = false;
        extern eeprom_profile_data_t profile_data;
        uint8_t profile_idx = (uint8_t) profile_data.current_profile_idx;
        session_stats_record(charge_mode_config.target_charge_weight,
                             current_measurement,
                             last_charge_elapsed_seconds,
                             last_coarse_elapsed_seconds,
                             bracket,
                             throw_result,
                             profile_idx,
                             charge_mode_config.eeprom_charge_mode_data.bracket_mode);
        learn_post_throw(profile_idx, throw_result, bracket,
                         last_charge_elapsed_seconds, last_coarse_elapsed_seconds, last_coarse_stop_weight,
                         current_measurement - charge_mode_config.target_charge_weight);
    }

    // Stop condition: 5 stable measurements in 300ms apart (1.5 seconds minimum)
    while (true) {
        TickType_t last_sample_tick = xTaskGetTickCount();

        // Non block waiting for the input
        ButtonEncoderEvent_t button_encoder_event = button_wait_for_input(false);
        if (button_encoder_event == BUTTON_RST_PRESSED) {
            charge_mode_config.charge_mode_state = CHARGE_MODE_EXIT;
            return;
        }

        // Perform measurement
        float current_weight;
        if (!scale_block_wait_for_next_measurement(200, &current_weight)) {
            // If no measurement within 200ms then poll the button and retry
            continue;
        }
        data_buffer.enqueue(current_weight);

        // Generate stop condition
        if (data_buffer.getCounter() >= 5) {
            if (data_buffer.getSd() < charge_mode_config.eeprom_charge_mode_data.set_point_sd_margin && 
                data_buffer.getMean() + 10 < charge_mode_config.eeprom_charge_mode_data.set_point_mean_margin){
                break;
            }
        }

        // Wait for next measurement
        vTaskDelayUntil(&last_sample_tick, pdMS_TO_TICKS(300));
    }

    // Back to session lighting between throws
    neopixel_led_set_colour(session_backlight(),
                            neopixel_led_config.eeprom_neopixel_led_metadata.default_led_colours.led1_colour,
                            neopixel_led_config.eeprom_neopixel_led_metadata.default_led_colours.led2_colour,
                            true);

    charge_mode_config.charge_mode_state = CHARGE_MODE_WAIT_FOR_CUP_RETURN;
}

void charge_mode_wait_for_cup_return() { 
    // Set colour to not ready
    neopixel_led_set_colour(
        session_backlight(), 
        charge_mode_config.eeprom_charge_mode_data.neopixel_not_ready_colour, 
        charge_mode_config.eeprom_charge_mode_data.neopixel_not_ready_colour, 
        true
    );

    snprintf(title_string, sizeof(title_string), "Return Cup");


    FloatRingBuffer data_buffer(5);

    while (true) {
        TickType_t last_sample_tick = xTaskGetTickCount();

        // Non block waiting for the input
        ButtonEncoderEvent_t button_encoder_event = button_wait_for_input(false);
        if (button_encoder_event == BUTTON_RST_PRESSED) {
            charge_mode_config.charge_mode_state = CHARGE_MODE_EXIT;
            return;
        }
        else if (button_encoder_event == BUTTON_ENCODER_PRESSED) {
            scale_config.scale_handle->force_zero();
        }

        // Perform measurement
        float current_weight;
        if (!scale_block_wait_for_next_measurement(200, &current_weight)) {
            // If no measurement within 200ms then poll the button and retry
            continue;
        }

        if (current_weight >= 0) {
            break;
        }

        // Wait for next measurement
        vTaskDelayUntil(&last_sample_tick, pdMS_TO_TICKS(20));
    }

    charge_mode_config.charge_mode_state = CHARGE_MODE_WAIT_FOR_ZERO;
}


uint8_t charge_mode_menu(bool charge_mode_skip_user_input) {
    // Create target weight, if the charge mode weight is built by charge_weight_digits
    if (!charge_mode_skip_user_input) {
        switch (charge_mode_config.eeprom_charge_mode_data.decimal_places) {
            case DP_2:
                charge_mode_config.target_charge_weight = charge_weight_digits[4] * 100 + \
                                                charge_weight_digits[3] * 10 + \
                                                charge_weight_digits[2] * 1 + \
                                                charge_weight_digits[1] * 0.1 + \
                                                charge_weight_digits[0] * 0.01;
                break;
            case DP_3:
                charge_mode_config.target_charge_weight = charge_weight_digits[4] * 10 + \
                                                charge_weight_digits[3] * 1 + \
                                                charge_weight_digits[2] * 0.1 + \
                                                charge_weight_digits[1] * 0.01 + \
                                                charge_weight_digits[0] * 0.001;
                break;
            default:
                charge_mode_config.target_charge_weight = 0;
                break;
        }
    }

    // If the display task is never created then we shall create one, otherwise we shall resume the task
    if (scale_measurement_render_task_handler == NULL) {
        // The render task shall have lower priority than the current one
        UBaseType_t current_task_priority = uxTaskPriorityGet(xTaskGetCurrentTaskHandle());
        xTaskCreate(scale_measurement_render_task, "Scale Measurement Render Task", configMINIMAL_STACK_SIZE, NULL, current_task_priority - 1, &scale_measurement_render_task_handler);
    }
    else {
        vTaskResume(scale_measurement_render_task_handler);
    }

    // Enable motor on entering the charge mode
    motor_enable(SELECT_COARSE_TRICKLER_MOTOR, true);
    motor_enable(SELECT_FINE_TRICKLER_MOTOR, true);
    
    charge_mode_config.charge_mode_state = CHARGE_MODE_WAIT_FOR_ZERO;

    bool quit = false;
    while (quit == false) {
        switch (charge_mode_config.charge_mode_state) {
            case CHARGE_MODE_WAIT_FOR_ZERO:
                charge_mode_wait_for_zero();
                break;
            case CHARGE_MODE_WAIT_FOR_COMPLETE:
                charge_mode_wait_for_complete();
                break;
            case CHARGE_MODE_WAIT_FOR_CUP_REMOVAL:
                charge_mode_wait_for_cup_removal();
                break;
            case CHARGE_MODE_WAIT_FOR_CUP_RETURN:
                charge_mode_wait_for_cup_return();
                break;
            case CHARGE_MODE_EXIT:
            default:
                quit = true;
                break;
        }
    }

    // Reset LED to default colour
    neopixel_led_set_colour(neopixel_led_config.eeprom_neopixel_led_metadata.default_led_colours.mini12864_backlight_colour,
                            neopixel_led_config.eeprom_neopixel_led_metadata.default_led_colours.led1_colour,
                            neopixel_led_config.eeprom_neopixel_led_metadata.default_led_colours.led2_colour,
                            true);

    // vTaskDelete(scale_measurement_render_handler);
    vTaskSuspend(scale_measurement_render_task_handler);

    // Diable motors on exiting the mode
    motor_enable(SELECT_COARSE_TRICKLER_MOTOR, false);
    motor_enable(SELECT_FINE_TRICKLER_MOTOR, false);

    return 1;  // return back to main menu
}


bool charge_mode_config_init(void) {
    bool is_ok = false;

    // Read charge mode config from EEPROM
    is_ok = load_config(EEPROM_CHARGE_MODE_BASE_ADDR, &charge_mode_config.eeprom_charge_mode_data, &default_charge_mode_data, sizeof(charge_mode_config.eeprom_charge_mode_data), EEPROM_CHARGE_MODE_DATA_REV);
    if (!is_ok) {
        printf("Unable to read charge mode configuration\n");
        return is_ok;
    }

    // Sanity on the bracket steps in case the EEPROM held garbage
    charge_mode_config.eeprom_charge_mode_data.normal_bracket_steps = clamp_steps(charge_mode_config.eeprom_charge_mode_data.normal_bracket_steps);
    charge_mode_config.eeprom_charge_mode_data.match_bracket_steps = clamp_steps(charge_mode_config.eeprom_charge_mode_data.match_bracket_steps);
    if (charge_mode_config.eeprom_charge_mode_data.bracket_mode > BRACKET_MODE_MATCH) {
        charge_mode_config.eeprom_charge_mode_data.bracket_mode = BRACKET_MODE_NORMAL;
    }

    session_stats_init();

    // Register to eeprom save all
    eeprom_register_handler(charge_mode_config_save);

    return true;
}


bool charge_mode_config_save(void) {
    bool is_ok = save_config(EEPROM_CHARGE_MODE_BASE_ADDR, &charge_mode_config.eeprom_charge_mode_data, sizeof(eeprom_charge_mode_data_t));
    return is_ok;
}



bool http_rest_charge_mode_config(struct fs_file *file, int num_params, char *params[], char *values[]) {
    // Mappings
    // c1 (str): neopixel_normal_charge_colour
    // c2 (str): neopixel_under_charge_colour
    // c3 (str): neopixel_over_charge_colour
    // c4 (str): neopixel_not_ready_colour

    // c5 (float): coarse_stop_threshold
    // c6 (float): fine_stop_threshold
    // c7 (float): set_point_sd_margin
    // c8 (float): set_point_mean_margin
    // c9 (int): decimal point enum
    // c10 (bool): precharge_enable
    // c11 (int): precharge_time_ms
    // c12 (float): precharge_speed_rps
    // c13 (float): coarse_stop_gate_ratio
    // c14 (bool): coarse_stop_backoff_enable
    // c15 (float): coarse_stop_backoff_turns
    // c16 (float): coarse_stop_backoff_speed_rps
    // c17 (str): neopixel_session_backlight_colour
    // c18 (int): bracket_mode (0 normal, 1 match)
    // c19 (int): normal_bracket_steps (x 0.02 gr)
    // c20 (int): match_bracket_steps (x 0.02 gr)
    // c21 (bool): learn_enable
    // c22 (bool): predict_enable
    // c23 (float): coarse_lag_s
    // c24 (bool): auto_lag_enable
    // c25 (float): fine_lag_s
    // c26 (float): coarse_tail_sd_gr
    // ee (bool): save to eeprom

    static char charge_mode_json_buffer[512];
    bool save_to_eeprom = false;

    // Control
    for (int idx = 0; idx < num_params; idx += 1) {
        if (strcmp(params[idx], "c5") == 0) {
            charge_mode_config.eeprom_charge_mode_data.coarse_stop_threshold = strtof(values[idx], NULL);
        }
        else if (strcmp(params[idx], "c6") == 0) {
            charge_mode_config.eeprom_charge_mode_data.fine_stop_threshold = strtof(values[idx], NULL);
        }
        else if (strcmp(params[idx], "c7") == 0) {
            charge_mode_config.eeprom_charge_mode_data.set_point_sd_margin = strtof(values[idx], NULL);
        }
        else if (strcmp(params[idx], "c8") == 0) {
            charge_mode_config.eeprom_charge_mode_data.set_point_mean_margin = strtof(values[idx], NULL);
        }
        else if (strcmp(params[idx], "c9") == 0) {
            charge_mode_config.eeprom_charge_mode_data.decimal_places = (decimal_places_t) atoi(values[idx]);
        }
        
        // Pre charge related settings
        else if (strcmp(params[idx], "c10") == 0) {
            charge_mode_config.eeprom_charge_mode_data.precharge_enable = string_to_boolean(values[idx]);
        }
        else if (strcmp(params[idx], "c11") == 0) {
            charge_mode_config.eeprom_charge_mode_data.precharge_time_ms = strtol(values[idx], NULL, 10);
        }
        else if (strcmp(params[idx], "c12") == 0) {
            charge_mode_config.eeprom_charge_mode_data.precharge_speed_rps = strtof(values[idx], NULL);
        }
        else if (strcmp(params[idx], "c13") == 0) {
            charge_mode_config.eeprom_charge_mode_data.coarse_stop_gate_ratio = strtof(values[idx], NULL);
        }
        else if (strcmp(params[idx], "c14") == 0) {
            charge_mode_config.eeprom_charge_mode_data.coarse_stop_backoff_enable = string_to_boolean(values[idx]);
        }
        else if (strcmp(params[idx], "c15") == 0) {
            charge_mode_config.eeprom_charge_mode_data.coarse_stop_backoff_turns = strtof(values[idx], NULL);
        }
        else if (strcmp(params[idx], "c16") == 0) {
            charge_mode_config.eeprom_charge_mode_data.coarse_stop_backoff_speed_rps = strtof(values[idx], NULL);
        }
        else if (strcmp(params[idx], "c17") == 0) {
            charge_mode_config.eeprom_charge_mode_data.neopixel_session_backlight_colour._raw_colour = hex_string_to_decimal(values[idx]);
        }
        else if (strcmp(params[idx], "c18") == 0) {
            charge_mode_config.eeprom_charge_mode_data.bracket_mode = (atoi(values[idx]) == 1) ? BRACKET_MODE_MATCH : BRACKET_MODE_NORMAL;
        }
        else if (strcmp(params[idx], "c19") == 0) {
            charge_mode_config.eeprom_charge_mode_data.normal_bracket_steps = clamp_steps((uint8_t) atoi(values[idx]));
        }
        else if (strcmp(params[idx], "c20") == 0) {
            charge_mode_config.eeprom_charge_mode_data.match_bracket_steps = clamp_steps((uint8_t) atoi(values[idx]));
        }
        else if (strcmp(params[idx], "c21") == 0) {
            charge_mode_config.eeprom_charge_mode_data.learn_enable = string_to_boolean(values[idx]);
        }
        else if (strcmp(params[idx], "c22") == 0) {
            charge_mode_config.eeprom_charge_mode_data.predict_enable = string_to_boolean(values[idx]);
        }
        else if (strcmp(params[idx], "c23") == 0) {
            charge_mode_config.eeprom_charge_mode_data.coarse_lag_s = strtof(values[idx], NULL);
        }
        else if (strcmp(params[idx], "c24") == 0) {
            charge_mode_config.eeprom_charge_mode_data.auto_lag_enable = string_to_boolean(values[idx]);
        }
        else if (strcmp(params[idx], "c25") == 0) {
            charge_mode_config.eeprom_charge_mode_data.fine_lag_s = strtof(values[idx], NULL);
        }
        else if (strcmp(params[idx], "c26") == 0) {
            charge_mode_config.eeprom_charge_mode_data.coarse_tail_sd_gr = strtof(values[idx], NULL);
        }


        // LED related settings
        else if (strcmp(params[idx], "c1") == 0) {
            charge_mode_config.eeprom_charge_mode_data.neopixel_normal_charge_colour._raw_colour = hex_string_to_decimal(values[idx]);
        }
        else if (strcmp(params[idx], "c2") == 0) {
            charge_mode_config.eeprom_charge_mode_data.neopixel_under_charge_colour._raw_colour = hex_string_to_decimal(values[idx]);
        }
        else if (strcmp(params[idx], "c3") == 0) {
            charge_mode_config.eeprom_charge_mode_data.neopixel_over_charge_colour._raw_colour = hex_string_to_decimal(values[idx]);
        }
        else if (strcmp(params[idx], "c4") == 0) {
            charge_mode_config.eeprom_charge_mode_data.neopixel_not_ready_colour._raw_colour = hex_string_to_decimal(values[idx]);
        }
        else if (strcmp(params[idx], "ee") == 0) {
            save_to_eeprom = string_to_boolean(values[idx]);
        }
    }
    
    // Perform action
    if (save_to_eeprom) {
        charge_mode_config_save();
    }

    // Response
    snprintf(charge_mode_json_buffer, 
             sizeof(charge_mode_json_buffer),
             "HTTP/1.1 200 OK\r\nContent-Type: application/json\r\n\r\n"
             "{\"c1\":\"#%06lx\",\"c2\":\"#%06lx\",\"c3\":\"#%06lx\",\"c4\":\"#%06lx\","
             "\"c5\":%.3f,\"c6\":%.3f,\"c7\":%.3f,\"c8\":%.3f,\"c9\":%d,\"c10\":%s,\"c11\":%ld,\"c12\":%0.3f,\"c13\":%0.3f,\"c14\":%s,\"c15\":%0.3f,\"c16\":%0.3f,"
             "\"c17\":\"#%06lx\",\"c18\":%d,\"c19\":%d,\"c20\":%d,\"c21\":%s,\"c22\":%s,\"c23\":%.2f,\"c24\":%s,\"c25\":%.2f,\"c26\":%.3f}",
             charge_mode_config.eeprom_charge_mode_data.neopixel_normal_charge_colour._raw_colour,
             charge_mode_config.eeprom_charge_mode_data.neopixel_under_charge_colour._raw_colour,
             charge_mode_config.eeprom_charge_mode_data.neopixel_over_charge_colour._raw_colour,
             charge_mode_config.eeprom_charge_mode_data.neopixel_not_ready_colour._raw_colour,
             charge_mode_config.eeprom_charge_mode_data.coarse_stop_threshold,
             charge_mode_config.eeprom_charge_mode_data.fine_stop_threshold,
             charge_mode_config.eeprom_charge_mode_data.set_point_sd_margin,
             charge_mode_config.eeprom_charge_mode_data.set_point_mean_margin,
             charge_mode_config.eeprom_charge_mode_data.decimal_places,
             boolean_to_string(charge_mode_config.eeprom_charge_mode_data.precharge_enable),
             charge_mode_config.eeprom_charge_mode_data.precharge_time_ms,
             charge_mode_config.eeprom_charge_mode_data.precharge_speed_rps,
             charge_mode_config.eeprom_charge_mode_data.coarse_stop_gate_ratio,
             boolean_to_string(charge_mode_config.eeprom_charge_mode_data.coarse_stop_backoff_enable),
             charge_mode_config.eeprom_charge_mode_data.coarse_stop_backoff_turns,
             charge_mode_config.eeprom_charge_mode_data.coarse_stop_backoff_speed_rps,
             charge_mode_config.eeprom_charge_mode_data.neopixel_session_backlight_colour._raw_colour,
             (int) charge_mode_config.eeprom_charge_mode_data.bracket_mode,
             (int) charge_mode_config.eeprom_charge_mode_data.normal_bracket_steps,
             (int) charge_mode_config.eeprom_charge_mode_data.match_bracket_steps,
             boolean_to_string(charge_mode_config.eeprom_charge_mode_data.learn_enable),
             boolean_to_string(charge_mode_config.eeprom_charge_mode_data.predict_enable),
             charge_mode_config.eeprom_charge_mode_data.coarse_lag_s,
             boolean_to_string(charge_mode_config.eeprom_charge_mode_data.auto_lag_enable),
             charge_mode_config.eeprom_charge_mode_data.fine_lag_s,
             charge_mode_config.eeprom_charge_mode_data.coarse_tail_sd_gr);

    size_t data_length = strlen(charge_mode_json_buffer);
    file->data = charge_mode_json_buffer;
    file->len = data_length;
    file->index = data_length;
    file->flags = FS_FILE_FLAGS_HEADER_INCLUDED;

    return true;
}


bool http_rest_charge_mode_state(struct fs_file *file, int num_params, char *params[], char *values[]) {
    // Mappings
    // s0 (float): Charge weight set point (unitless)
    // s1 (float): Current weight (unitless)
    // s2 (charge_mode_state_t | int): Charge mode state
    // s3 (uint32_t): Charge mode event
    // s4 (string): Profile Name
    // s5 (string): Elapsed time in seconds, live during charging
    // s6 (float): active bracket +/- grains
    // s7 (int): bracket mode (0 normal, 1 match). Settable.
    // s8 (float): session average time s
    // s9 (float): session success rate percent
    // s10 (int): session throw count
    // s11 (float): coarse lag in use
    // s12 (float): lag measured on the last throw (fine phase, feeds auto lag)
    // s13 (float): dead time on the last throw
    // s14 (float): fine lag in use
    // s15 (string): what the live tuner is doing

    static char charge_mode_json_buffer[420];
    char elapsed_time_buffer[16] = {0};

    // Control
    for (int idx = 0; idx < num_params; idx += 1) {
        if (strcmp(params[idx], "s0") == 0) {
            charge_mode_config.target_charge_weight = strtof(values[idx], NULL);
        }
        else if (strcmp(params[idx], "s2") == 0) {
            charge_mode_state_t new_state = (charge_mode_state_t) atoi(values[idx]);

            // Exit
            if (new_state == CHARGE_MODE_EXIT && charge_mode_config.charge_mode_state != CHARGE_MODE_EXIT) {
                ButtonEncoderEvent_t button_event = BUTTON_RST_PRESSED;
                xQueueSend(encoder_event_queue, &button_event, portMAX_DELAY);
            }
            // Enter
            else if (new_state == CHARGE_MODE_WAIT_FOR_ZERO && charge_mode_config.charge_mode_state == CHARGE_MODE_EXIT) {
                // Set exit_status for the menu
                exit_state = APP_STATE_ENTER_CHARGE_MODE_FROM_REST;

                // Then signal the menu to stop
                ButtonEncoderEvent_t button_event = OVERRIDE_FROM_REST;
                xQueueSend(encoder_event_queue, &button_event, portMAX_DELAY);
            }

            charge_mode_config.charge_mode_state = new_state;
        }
        else if (strcmp(params[idx], "s7") == 0) {
            charge_mode_config.eeprom_charge_mode_data.bracket_mode = (atoi(values[idx]) == 1) ? BRACKET_MODE_MATCH : BRACKET_MODE_NORMAL;
        }
    }

    // Handle the special case
    float current_measurement = scale_get_current_measurement();
    char weight_string[16];
    if (isnanf(current_measurement)) {
        sprintf(weight_string, "\"nan\"");
    }
    else if (isinff(current_measurement)) {
        sprintf(weight_string, "\"inf\"");
    }
    else {
        sprintf(weight_string, "%0.3f", current_measurement);
    }

    // Format elapsed time
    if (charge_mode_config.charge_mode_state == CHARGE_MODE_WAIT_FOR_COMPLETE) {
        TickType_t now = xTaskGetTickCount();
        float elapsed_seconds = (float)((now - charge_start_tick) * portTICK_PERIOD_MS) / 1000.0f;
        snprintf(elapsed_time_buffer, sizeof(elapsed_time_buffer), "%.2f", elapsed_seconds);
    } else {
        snprintf(elapsed_time_buffer, sizeof(elapsed_time_buffer), "%.2f", last_charge_elapsed_seconds);
    }

    // Response
    snprintf(charge_mode_json_buffer, 
             sizeof(charge_mode_json_buffer),
             "%s"
             "{\"s0\":%0.3f,\"s1\":%s,\"s2\":%d,\"s3\":%lu,\"s4\":\"%s\",\"s5\":\"%s\","
             "\"s6\":%0.2f,\"s7\":%d,\"s8\":%0.2f,\"s9\":%0.1f,\"s10\":%lu,"
             "\"s11\":%0.2f,\"s12\":%0.2f,\"s13\":%0.2f,\"s14\":%0.2f,\"s15\":\"%s\"}",
             http_json_header,
             charge_mode_config.target_charge_weight,
             weight_string,
             (int) charge_mode_config.charge_mode_state,
             charge_mode_config.charge_mode_event,
             profile_get_selected()->name,
             elapsed_time_buffer,
             charge_mode_get_active_bracket(),
             (int) charge_mode_config.eeprom_charge_mode_data.bracket_mode,
             session_stats_avg_time(),
             session_stats_success_rate(),
             (unsigned long) session_stats_summary()->total,
             charge_mode_config.eeprom_charge_mode_data.coarse_lag_s,
             last_measured_lag,
             last_dead_time_s,
             charge_mode_config.eeprom_charge_mode_data.fine_lag_s,
             charge_mode_learn_status());

    // Clear events
    charge_mode_config.charge_mode_event = 0;

    size_t data_length = strlen(charge_mode_json_buffer);
    file->data = charge_mode_json_buffer;
    file->len = data_length;
    file->index = data_length;
    file->flags = FS_FILE_FLAGS_HEADER_INCLUDED;

    return true;
}
