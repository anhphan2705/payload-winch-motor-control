#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include <math.h>
#include <stdint.h>


// ----------------- USER CONFIG -----------------
#define PWM_PIN   15
#define DIR_PIN   14
#define FG_PIN    16
#define PWM_WRAP  6249        // 20 kHz at 125 MHz (for RP2040 default clk)

static constexpr float    GEAR_RATIO              = 14.0f; // 24V 570RPM version (14:1)
static constexpr uint32_t FG_PULSES_PER_MOTOR_REV = 6;     // datasheet: FG = 6 pulses / motor rev
static constexpr float    DRUM_DIAMETER_M         = 0.050f; // 50mm drum diameter
// ------------------------------------------------

// Unsigned pulse count (FG has no direction info)
static volatile uint32_t g_fg_pulses = 0;

static void fg_irq_handler(uint gpio, uint32_t events) {
    if (gpio == FG_PIN && (events & GPIO_IRQ_EDGE_RISE)) {
        g_fg_pulses++;
    }
}

// --- PWM init (20 kHz) ---
static void pwm_init_motor() {
    gpio_set_function(PWM_PIN, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(PWM_PIN);
    pwm_set_wrap(slice, PWM_WRAP);
    pwm_set_chan_level(slice, pwm_gpio_to_channel(PWM_PIN), 0);
    pwm_set_enabled(slice, true);
}

// percent: 0..100
static void set_speed(float percent) {
    if (percent < 0) percent = 0;
    if (percent > 100) percent = 100;
    uint level = (uint)(percent / 100.0f * PWM_WRAP);
    pwm_set_chan_level(pwm_gpio_to_slice_num(PWM_PIN),
                       pwm_gpio_to_channel(PWM_PIN),
                       level);
}

// Smooth ramp with memory of current speed
static void ramp_to(float target_percent, int step_delay_ms = 10, float step_percent = 1.0f) {
    static float current = 0.0f;

    if (target_percent > current) {
        for (float p = current; p <= target_percent; p += step_percent) {
            set_speed(p);
            sleep_ms(step_delay_ms);
        }
    } else {
        for (float p = current; p >= target_percent; p -= step_percent) {
            set_speed(p);
            sleep_ms(step_delay_ms);
        }
    }
    current = target_percent;
}

static void brake_to_stop(int settle_ms = 300) {
    ramp_to(0);
    sleep_ms(settle_ms);
}

static void set_direction_cw(bool cw) {
    // Your wiring convention: LOW=CW, HIGH=CCW
    gpio_put(DIR_PIN, cw ? 0 : 1);
}

static float drum_circumference_m() {
    return (float)M_PI * DRUM_DIAMETER_M; // πD
}

static uint32_t target_pulses_for_meters(float meters) {
    if (meters <= 0) return 0;

    // pulses per drum rev = gear_ratio * pulses_per_motor_rev = 14 * 6 = 84
    const float pulses_per_output_rev = GEAR_RATIO * (float)FG_PULSES_PER_MOTOR_REV;
    const float meters_per_output_rev = drum_circumference_m(); // ≈ 0.1571 m
    const float pulses_per_meter      = pulses_per_output_rev / meters_per_output_rev; // ≈ 535

    return (uint32_t)(meters * pulses_per_meter + 0.5f);
}

// Move by distance (meters) using FG pulse counting with stall/timeout + end slowdown
bool move_meters(bool cw, float meters,
                 float cruise_percent,
                 uint32_t timeout_ms)
{
    uint32_t target = target_pulses_for_meters(meters);
    if (target == 0) return true;

    // reset pulse counter
    g_fg_pulses = 0;
    set_direction_cw(cw);

    // soft start
    ramp_to(cruise_percent);

    uint32_t last_pulses = 0;
    absolute_time_t t0 = get_absolute_time();
    absolute_time_t last_progress = get_absolute_time();
    bool slowdown_triggered = false;

    while (g_fg_pulses < target) {
        uint32_t now = g_fg_pulses;

        // Stall/progress check (no pulses for 300ms while commanded to move)
        if (now != last_pulses) {
            last_pulses = now;
            last_progress = get_absolute_time();
        } else {
            if (absolute_time_diff_us(last_progress, get_absolute_time()) > 300000) {
                // no FG progress => stalled or FG broken/noisy
                brake_to_stop();
                return false;
            }
        }

        // Timeout safety
        if (absolute_time_diff_us(t0, get_absolute_time()) > (int64_t)timeout_ms * 1000) {
            brake_to_stop();
            return false;
        }

        // Slowdown near end (last 20%)
        uint32_t remaining = target - now;
        if (!slowdown_triggered && remaining < target / 5) {       // last 20%
            slowdown_triggered = true;
            ramp_to(50.0f);                 // stable low speed, jerk if below 50%
        }

        tight_loop_contents();
    }

    brake_to_stop();
    return true;
}

// HOLD: watches FG pulses; if slip occurs, it "nudges" upward a little then stops again.
// NOTE: FG has no direction, so we treat ANY pulses during hold as "movement happened".
bool hold_payload_ms(uint32_t hold_ms,
                     bool tow_up_cw,
                     float nudge_speed_percent = 50.0f,
                     uint32_t deadband_pulses = 1,
                     uint32_t nudge_pulses = 80,
                     uint32_t min_nudge_gap_ms = 250)
{
    brake_to_stop(200);

    // Reset pulse counter at start of hold
    g_fg_pulses = 0;

    absolute_time_t t0 = get_absolute_time();
    absolute_time_t last_nudge = get_absolute_time();

    while (absolute_time_diff_us(t0, get_absolute_time()) < (int64_t)hold_ms * 1000) {

        // If we see pulses while "stopped", the drum is moving (slipping/backdriving)
        if (g_fg_pulses > deadband_pulses) {

            // Rate-limit nudges so it doesn't chatter too fast
            if (absolute_time_diff_us(last_nudge, get_absolute_time()) > (int64_t)min_nudge_gap_ms * 1000) {

                // Nudge UP a bit
                g_fg_pulses = 0;
                set_direction_cw(tow_up_cw);

                ramp_to(nudge_speed_percent, 8, 10.0f);

                while (g_fg_pulses < nudge_pulses) {
                    tight_loop_contents();
                }

                brake_to_stop(200);
                g_fg_pulses = 0;
                last_nudge = get_absolute_time();
            }
        }

        sleep_ms(10);
    }

    return true;
}

// Public API
bool unwind_payload_m(float meters, float speed_percent = 40.0f) {
    // unwind = CCW (cw=false). Flip if your wiring/spool is opposite.
    return move_meters(false, meters, speed_percent, 60000);
}

bool wind_payload_m(float meters, float speed_percent = 60.0f) {
    // wind = CW (cw=true). Flip if your wiring/spool is opposite.
    return move_meters(true, meters, speed_percent, 60000);
}

int main() {
    stdio_init_all();

    // DIR pin
    gpio_init(DIR_PIN);
    gpio_set_dir(DIR_PIN, GPIO_OUT);

    // FG pin input with pull-up (external 10k to 3.3V recommended)
    gpio_init(FG_PIN);
    gpio_set_dir(FG_PIN, GPIO_IN);
    gpio_pull_up(FG_PIN);
    // gpio_disable_pulls(FG_PIN);

    // IRQ on rising edge
    gpio_set_irq_enabled_with_callback(FG_PIN, GPIO_IRQ_EDGE_RISE, true, &fg_irq_handler);

    // PWM init
    pwm_init_motor();

    sleep_ms(5000);

    while (true) {
        // Example cycle:
        // Unwind 0.5m, hold 5s, wind 0.5m, pause
        unwind_payload_m(0.3f, 100.0f);
        
        // Hold 5 seconds (wind/tow up direction is CW=true)
        hold_payload_ms(2000, /*tow_up_cw=*/true);

        wind_payload_m(0.3f, 100.0f);

        sleep_ms(5000);
    }
}