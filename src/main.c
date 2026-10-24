#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>

#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/timer.h"
#include "hardware/adc.h"
#include "hardware/i2c.h"
#include "pico/stdio.h"

#include "ssd1306.h"

// Single sine output on GPIO 7 (PWM slice 3, channel B).
#define GPIO_OUT        7u

// Frequency potentiometer: wiper -> GP26 (ADC0), outer legs -> 3V3 and GND.
#define FREQ_ADC_GPIO   26u
#define FREQ_ADC_CHAN   0u

// OLED on I2C1: SDA = GP14 (pin 19), SCL = GP15 (pin 20).
#define OLED_I2C        i2c1
#define OLED_SDA        14u
#define OLED_SCL        15u
#define OLED_ADDR       0x3Cu
#define OLED_COL_OFFSET 0u    // 0 = SSD1306, 2 = SH1106

#define PWM_TOP          255u
#define SAMPLE_RATE_HZ   43478u   // repeating_timer_us uses integer microseconds (23 us)
#define TWO_PI           6.2831853071795864769f

#define FREQ_MIN_HZ      80.0f
#define FREQ_MAX_HZ      1000.0f
#define DEFAULT_HZ       440.0f

// Number of ADC reads averaged per control sample.
#define ADC_AVG_SAMPLES  8u

// 2 full cycles across the 128px screen = 2^32 / 64.
#define WAVE_PHASE_STEP  67108864u

// Written by the main loop, read by the audio callback. A single aligned 32-bit
// store is atomic on the M0+, so no lock is needed.
static volatile uint32_t g_phase_step;
static uint32_t g_phase;
static ssd1306_t g_display;
static uint32_t g_disp_phase;

// Map a [-1, +1] waveform value to a PWM duty cycle, leaving headroom at both rails.
static inline uint16_t wave_to_duty(float v) {
    float duty = (0.5f + 0.48f * v) * (float)PWM_TOP + 0.5f;
    if (duty < 0.0f) duty = 0.0f;
    if (duty > (float)PWM_TOP) duty = (float)PWM_TOP;
    return (uint16_t)duty;
}

// Called by the SDK alarm pool at ~43.5 kHz. Keep it short and allocation-free.
static bool audio_sample_callback(struct repeating_timer *timer) {
    (void)timer;

    const uint32_t step = g_phase_step;
    const float phase = (float)g_phase * (TWO_PI / 4294967296.0f);

    pwm_set_gpio_level(GPIO_OUT, wave_to_duty(sinf(phase)));

    g_phase += step;
    return true;
}

static void update_display(unsigned freq_hz) {
    ssd1306_clear(&g_display);

    // Label + big frequency number + unit.
    ssd1306_draw_text(&g_display, 0, 0, "FREQ", 1);

    char buf[8];
    snprintf(buf, sizeof(buf), "%u", freq_hz);
    ssd1306_draw_text(&g_display, 0, 8, buf, 3);

    int num_width = (int)strlen(buf) * 6 * 3;
    ssd1306_draw_text(&g_display, num_width + 4, 24, "Hz", 1);

    // Live sine trace, slowly scrolling.
    g_disp_phase += 0x08000000u; // ~1/32 cycle per redraw
    for (int px = 0; px < SSD1306_WIDTH; px++) {
        uint32_t ph = g_disp_phase + (uint32_t)((uint64_t)px * WAVE_PHASE_STEP);
        float v = sinf((float)ph * (TWO_PI / 4294967296.0f));
        int y = 52 - (int)(11.0f * v);
        ssd1306_set_pixel(&g_display, px, y, true);
    }

    ssd1306_flush(&g_display);
}

int main(void) {
    // Default tone so there is output before the first ADC read settles.
    g_phase_step = (uint32_t)((DEFAULT_HZ * 4294967296.0) / (double)SAMPLE_RATE_HZ + 0.5);

    // Route GP7 to the PWM peripheral (slice 3, channel B).
    gpio_set_function(GPIO_OUT, GPIO_FUNC_PWM);
    pwm_config config = pwm_get_default_config();
    pwm_config_set_clkdiv(&config, 1.0f);
    pwm_config_set_wrap(&config, PWM_TOP);
    pwm_init(pwm_gpio_to_slice_num(GPIO_OUT), &config, true);

    // Potentiometer ADC.
    adc_init();
    adc_gpio_init(FREQ_ADC_GPIO);
    adc_select_input(FREQ_ADC_CHAN);

    // OLED on I2C1 (GP14 SDA / GP15 SCL).
    i2c_init(OLED_I2C, 400 * 1000);
    gpio_set_function(OLED_SDA, GPIO_FUNC_I2C);
    gpio_set_function(OLED_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(OLED_SDA);
    gpio_pull_up(OLED_SCL);
    ssd1306_init(&g_display, OLED_I2C, OLED_ADDR, OLED_COL_OFFSET);

    // USB CDC serial (printf). Safe: non-blocking, init does not wait for a host.
    stdio_init_all();

    // Negative interval -> deadlines scheduled from the previous deadline (no drift).
    struct repeating_timer audio_timer;
    const int64_t sample_period_us = -23;
    if (!add_repeating_timer_us(sample_period_us, audio_sample_callback, NULL, &audio_timer)) {
        while (true) tight_loop_contents();
    }

    // Control loop: sample the pot at ~100 Hz, smooth it, then update the phase step.
    float freq = DEFAULT_HZ;
    uint32_t tick = 0;
    while (true) {
        uint32_t sum = 0;
        for (uint i = 0; i < ADC_AVG_SAMPLES; i++) {
            sum += adc_read();
        }
        const uint16_t raw = (uint16_t)(sum / ADC_AVG_SAMPLES);

        const float target = FREQ_MIN_HZ + ((float)raw / 4095.0f) * (FREQ_MAX_HZ - FREQ_MIN_HZ);
        freq += 0.15f * (target - freq); // exponential smoothing

        g_phase_step = (uint32_t)((freq * 4294967296.0) / (double)SAMPLE_RATE_HZ + 0.5);

        if ((tick % 10) == 0) { // display refresh at ~10 Hz
            update_display((unsigned)freq);
        }
        if ((tick % 20) == 0) { // log every ~200 ms
            printf("ADC=%4u  freq=%4u Hz  phase_step=%u\n",
                   raw, (unsigned)freq, (unsigned)g_phase_step);
        }
        tick++;
        sleep_ms(10);
    }
}
