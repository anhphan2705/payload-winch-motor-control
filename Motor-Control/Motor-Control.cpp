#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include <math.h>
#include <stdint.h>
#include <stdio.h>


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

// Slew Rate Limiter

static struct {
    float current = 0.0f;
    float target = 0.0f;
    float rate = 200.0f; // percent per second
    absolute_time_t last_t = {0};
    bool initialized = false;
} g_slew;

static void slew_init(float start_percent = 0.0f) {
    g_slew.current = start_percent;
    g_slew.target  = start_percent;
    g_slew.last_t  = get_absolute_time();
    g_slew.initialized = true;
    set_speed(start_percent);
}

static void slew_set_target(float target_percent, float rate_percent_per_sec = 200.0f) {
    if (!g_slew.initialized) slew_init(0.0f);

    if (target_percent < 0) target_percent = 0;
    if (target_percent > 100) target_percent = 100;

    g_slew.target = target_percent;
    g_slew.rate   = (rate_percent_per_sec <= 0) ? 1.0f : rate_percent_per_sec;
}

static void slew_update() {
    if (!g_slew.initialized) slew_init(0.0f);

    absolute_time_t now_t = get_absolute_time();
    int64_t dt_us = absolute_time_diff_us(g_slew.last_t, now_t);
    if (dt_us <= 0) return;

    g_slew.last_t = now_t;

    float dt_s = (float)dt_us / 1e6f;
    float max_step = g_slew.rate * dt_s;

    float error = g_slew.target - g_slew.current;

    if (fabsf(error) <= max_step) {
        g_slew.current = g_slew.target;
    } else {
        g_slew.current += (error > 0 ? max_step : -max_step);
    }

    set_speed(g_slew.current);
}

static bool slew_at_target(float eps = 0.5f) {
    return fabsf(g_slew.current - g_slew.target) <= eps;
}

static void brake_to_stop(int settle_ms = 300) {
    // command a stop (non-blocking)
    slew_set_target(0.0f, 400.0f);

    // wait a short settle time while still updating PWM smoothly
    absolute_time_t t0 = get_absolute_time();
    while (absolute_time_diff_us(t0, get_absolute_time()) < (int64_t)settle_ms * 1000) {
        slew_update();
        tight_loop_contents();
    }
}

static void set_direction_cw(bool cw) {
    // Wiring convention: LOW=CW, HIGH=CCW
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
bool move_meters (
    bool cw, float meters,
    float cruise_percent,
    uint32_t timeout_ms,
    float padding_m = 0.2f,
    float padding_speed = 50.0f, // padding in meter
    int64_t stall_window_us = 500000)   // 500k us = 500 ms
{
    uint32_t target = target_pulses_for_meters(meters);
    if (target == 0) return true;

    uint32_t pad_pulses = target_pulses_for_meters(padding_m);

    // If move is too short for padding, just go slow entire way
    if (pad_pulses * 2 >= target) {
        pad_pulses = target / 2;
    }

    gpio_set_irq_enabled(FG_PIN, GPIO_IRQ_EDGE_RISE, false);
    g_fg_pulses = 0;
    gpio_set_irq_enabled(FG_PIN, GPIO_IRQ_EDGE_RISE, true);

    set_direction_cw(cw);

    float last_speed = -1.0f;
    float start_speed = (pad_pulses > 0) ? padding_speed : cruise_percent;
    slew_set_target(start_speed, 200.0f);
    last_speed = start_speed;
    
    absolute_time_t t0 = get_absolute_time();
    absolute_time_t stall_ref_time = get_absolute_time();
    uint32_t stall_ref_pulses = g_fg_pulses;

    while (g_fg_pulses < target) {
        slew_update();
        uint32_t now = g_fg_pulses;

        // ---- Stall detection ----
        if (absolute_time_diff_us(stall_ref_time, get_absolute_time()) > stall_window_us) {
            uint32_t dp = g_fg_pulses - stall_ref_pulses;
            float cmd = g_slew.target;

            uint32_t min_pulses;
            if (cmd < 15.0f)      min_pulses = 0;
            else if (cmd < 40.0f) min_pulses = 1;
            else if (cmd < 70.0f) min_pulses = 2;
            else                  min_pulses = 3;

            if (min_pulses > 0 && dp < min_pulses) {
                brake_to_stop();
                return false;
            }

            stall_ref_pulses = g_fg_pulses;
            stall_ref_time = get_absolute_time();
        }

        // ---- Timeout ----
        if (absolute_time_diff_us(t0, get_absolute_time()) > (int64_t)timeout_ms * 1000) {
            brake_to_stop();
            return false;
        }

        uint32_t remaining = (now < target) ? (target - now) : 0;

        // ---- Speed Selection ----
        float desired_speed;

        if (now < pad_pulses) desired_speed = padding_speed;
        else if (remaining < pad_pulses) desired_speed = padding_speed;
        else desired_speed = cruise_percent;

        if (fabsf(desired_speed - last_speed) > 0.01f) {
            slew_set_target(desired_speed, 200.0f);  // rate = 200 %/s (tune)
            last_speed = desired_speed;
        }

        tight_loop_contents();
    }

    brake_to_stop();
    return true;
}

// HOLD: watches FG pulses; if slip occurs, it "nudges" upward a little then stops again.
// NOTE: FG has no direction, so treat ANY pulses during hold as "movement happened".
bool hold_payload_ms(uint32_t hold_ms,
                     bool tow_up_cw,
                     float nudge_speed_percent = 50.0f,
                     uint32_t deadband_pulses = 1,
                     uint32_t nudge_pulses = 80,
                     uint32_t min_nudge_gap_ms = 250)
{
    brake_to_stop(200);

    // Reset pulse counter at start of hold
    gpio_set_irq_enabled(FG_PIN, GPIO_IRQ_EDGE_RISE, false);
    g_fg_pulses = 0;
    gpio_set_irq_enabled(FG_PIN, GPIO_IRQ_EDGE_RISE, true);

    absolute_time_t t0 = get_absolute_time();
    absolute_time_t last_nudge = get_absolute_time();

    while (absolute_time_diff_us(t0, get_absolute_time()) < (int64_t)hold_ms * 1000) {
        slew_update();

        // If see pulses while "stopped", the drum is moving (slipping/backdriving)
        if (g_fg_pulses > deadband_pulses) {

            // Rate-limit nudges so it doesn't chatter too fast
            if (absolute_time_diff_us(last_nudge, get_absolute_time()) > (int64_t)min_nudge_gap_ms * 1000) {

                // Nudge UP a bit

                gpio_set_irq_enabled(FG_PIN, GPIO_IRQ_EDGE_RISE, false);
                g_fg_pulses = 0;
                gpio_set_irq_enabled(FG_PIN, GPIO_IRQ_EDGE_RISE, true);

                set_direction_cw(tow_up_cw);

                slew_set_target(nudge_speed_percent, 400.0f);

                while (g_fg_pulses < nudge_pulses) {
                    slew_update();
                    tight_loop_contents();
                }

                brake_to_stop(200);

                gpio_set_irq_enabled(FG_PIN, GPIO_IRQ_EDGE_RISE, false);
                g_fg_pulses = 0;
                gpio_set_irq_enabled(FG_PIN, GPIO_IRQ_EDGE_RISE, true);

                last_nudge = get_absolute_time();
            }
        }

        sleep_ms(10);
    }

    return true;
}

static void monitor_fg_for_ms(uint32_t ms, const char* tag = "MON") {
    // reset pulses atomically
    gpio_set_irq_enabled(FG_PIN, GPIO_IRQ_EDGE_RISE, false);
    g_fg_pulses = 0;
    gpio_set_irq_enabled(FG_PIN, GPIO_IRQ_EDGE_RISE, true);

    absolute_time_t t0 = get_absolute_time();
    uint32_t last = 0;

    printf("[%s] Start monitoring for %u ms. Spin by hand now...\n", tag, ms);

    while (absolute_time_diff_us(t0, get_absolute_time()) < (int64_t)ms * 1000) {
        // keep motor command stable during monitor window
        slew_update();

        // sample every 200ms
        sleep_ms(200);

        uint32_t cur = g_fg_pulses;
        uint32_t dp  = cur - last;
        last = cur;

        int lvl = gpio_get(FG_PIN);

        printf("[%s] pulses=%lu  dp=%lu  FG_lvl=%d\n",
               tag,
               (unsigned long)cur,
               (unsigned long)dp,
               lvl);
    }

    printf("[%s] Done. Total pulses=%lu\n", tag, (unsigned long)g_fg_pulses);
}

static void fg_hand_spin_test(uint32_t ms,
                              float wake_percent = 3.0f,   // try 1–5%
                              float wake_rate = 400.0f)    // fast slew
{
    // Make sure driver is in a known state
    set_direction_cw(true);

    // Keep the driver "awake" (some drivers only output FG when enabled/commutating)
    slew_set_target(wake_percent, wake_rate);

    // Reset pulses safely
    gpio_set_irq_enabled(FG_PIN, GPIO_IRQ_EDGE_RISE, false);
    g_fg_pulses = 0;
    gpio_set_irq_enabled(FG_PIN, GPIO_IRQ_EDGE_RISE, true);

    absolute_time_t t0 = get_absolute_time();
    uint32_t last = 0;

    printf("[FG_HAND] Driver awake at %.1f%%. Spin by hand now for %u ms...\n",
           wake_percent, ms);

    while (absolute_time_diff_us(t0, get_absolute_time()) < (int64_t)ms * 1000) {
        slew_update();
        sleep_ms(200);

        uint32_t cur = g_fg_pulses;
        printf("[FG_HAND] pulses=%lu dp=%lu lvl=%d cmd=%.1f cur=%.1f\n",
               (unsigned long)cur,
               (unsigned long)(cur - last),
               gpio_get(FG_PIN),
               g_slew.target,
               g_slew.current);

        last = cur;
    }

    brake_to_stop(200);
    printf("[FG_HAND] Done. Total pulses=%lu\n", (unsigned long)g_fg_pulses);
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
        unwind_payload_m(0.6f, 100.0f);
        
        // Hold 5 seconds (wind/tow up direction is CW=true)
        hold_payload_ms(2000, /*tow_up_cw=*/true);

        wind_payload_m(0.6f, 100.0f);

        sleep_ms(5000);
        
        // // Test to see if pulses are detected when hand-spinning the drum with driver "awake" at low speed
        // bool ok = unwind_payload_m(0.3f, 100.0f);
        // printf("Unwind done. OK=%d  Pulses=%lu\n", ok, (unsigned long)g_fg_pulses);

        // // Stop so you're not fighting the motor
        // brake_to_stop(200);

        // // Now test slip / hand spin for 8 seconds
        // monitor_fg_for_ms(8000, "BETWEEN");

        // fg_hand_spin_test(8000, 3.0f);

        // // Then do wind
        // ok = wind_payload_m(0.3f, 100.0f);
        // printf("Wind done. OK=%d  Pulses=%lu\n", ok, (unsigned long)g_fg_pulses);
    }
}