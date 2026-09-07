#ifndef CHARGE_MODE_H_
#define CHARGE_MODE_H_

#include <stdint.h>
#include "http_rest.h"
#include "common.h"
#include "neopixel_led.h"


#define EEPROM_CHARGE_MODE_DATA_REV                     11             // 16 byte 

#define WEIGHT_STRING_LEN 8

// Acceptance bracket. Tolerance = steps * BRACKET_STEP_GRAINS, selectable from the screen and the portal.
#define BRACKET_STEP_GRAINS         0.02f
#define BRACKET_STEPS_MIN           1       // +/- 0.02
#define BRACKET_STEPS_MAX           10      // +/- 0.20

typedef enum {
    BRACKET_MODE_NORMAL = 0,
    BRACKET_MODE_MATCH = 1,
} bracket_mode_t;

typedef enum {
    CHARGE_MODE_EXIT = 0,
    CHARGE_MODE_WAIT_FOR_ZERO = 1,
    CHARGE_MODE_WAIT_FOR_COMPLETE = 2,
    CHARGE_MODE_WAIT_FOR_CUP_REMOVAL = 3,
    CHARGE_MODE_WAIT_FOR_CUP_RETURN = 4,
} charge_mode_state_t;

typedef struct {
    uint16_t charge_mode_data_rev;

    float coarse_stop_threshold;
    float fine_stop_threshold;

    float set_point_sd_margin;
    float set_point_mean_margin;
    float coarse_stop_gate_ratio; // 0.0=open, 1.0=close, -1.0=disabled (optional)

    bool coarse_stop_backoff_enable;
    float coarse_stop_backoff_turns; // 0.125, 0.25, 0.5, 1.0
    float coarse_stop_backoff_speed_rps; // if <=0 uses min speed

    decimal_places_t decimal_places;

    // Precharge
    bool precharge_enable;
    uint32_t precharge_time_ms;
    float precharge_speed_rps;

    // LED related settings
    rgbw_u32_t neopixel_normal_charge_colour;
    rgbw_u32_t neopixel_under_charge_colour;
    rgbw_u32_t neopixel_over_charge_colour;
    rgbw_u32_t neopixel_not_ready_colour;

    // Session / bracket (rev 9)
    rgbw_u32_t neopixel_session_backlight_colour;   // mini12864 backlight while in charge mode
    uint8_t bracket_mode;                           // bracket_mode_t
    uint8_t normal_bracket_steps;                   // x 0.02 gr
    uint8_t match_bracket_steps;                    // x 0.02 gr
    bool learn_enable;                              // adaptive profile tuning after each throw

    // Lag compensation (rev 10). The scale reads behind the powder. Predicted weight = reading + rate x lag.
    // The motors are driven off the predicted weight, the final stop is still on the real reading.
    bool predict_enable;
    float scale_lag_s;
    bool auto_lag_enable;           // re-measure the lag from every throw and track it

} eeprom_charge_mode_data_t;

typedef struct {
    eeprom_charge_mode_data_t eeprom_charge_mode_data;
    float target_charge_weight;
    uint32_t charge_mode_event;
    charge_mode_state_t charge_mode_state;
} charge_mode_config_t;


// C Functions
#ifdef __cplusplus
extern "C" {
#endif


bool charge_mode_config_init(void);
uint8_t charge_mode_menu(bool charge_mode_skip_user_input);
bool charge_mode_config_save(void);

// Bracket helpers
float charge_mode_get_active_bracket(void);     // +/- grains for the active mode
void charge_mode_toggle_bracket_mode(void);
const char * charge_mode_bracket_mode_name(void);
float charge_mode_get_last_elapsed_seconds(void);
float charge_mode_get_measured_lag(void);       // lag from the last throw, 0 if not measurable
float charge_mode_get_dead_time(void);          // motor start to first movement on the scale, last throw

// REST interface
bool http_rest_charge_mode_config(struct fs_file *file, int num_params, char *params[], char *values[]);
bool http_rest_charge_mode_state(struct fs_file *file, int num_params, char *params[], char *values[]);


#ifdef __cplusplus
}  // __cplusplus
#endif


#endif  // CHARGE_MODE_H_