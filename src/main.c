#include <math.h>
#include <stdio.h>
#include <stdint.h>

#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/timer.h"
#include "hardware/adc.h"
#include "hardware/i2c.h"
#include "pico/stdio.h"

#include "ssd1306.h"

// ---------------------------------------------------------------------------
// Single mixed audio output (PWM).
// ---------------------------------------------------------------------------
#define AUDIO_GPIO      7u    // PWM slice 3, channel B

// Frequency potentiometers (one per voice). RP2040 has 3 external ADCs.
#define ADC_SINE_GPIO    26u
#define ADC_SINE_CHAN    0u
#define ADC_SQUARE_GPIO  27u
#define ADC_SQUARE_CHAN  1u
#define ADC_SAW_GPIO     28u
#define ADC_SAW_CHAN     2u

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

typedef enum {
    WAVE_SINE = 0,
    WAVE_SQUARE,
    WAVE_SAW,
} wave_kind_t;

typedef struct {
    const char *label;
    uint        adc_gpio;   // pot input
    uint        adc_chan;   // ADC mux select
    wave_kind_t wave;
    uint32_t    phase;      // audio phase accumulator (owned by callback)
    volatile uint32_t phase_step; // written by main loop, read by callback
    float       freq;       // smoothed frequency (owned by main loop)
} voice_t;

static voice_t g_voices[3] = {
    { "SIN", ADC_SINE_GPIO,   ADC_SINE_CHAN,   WAVE_SINE,     0, 0, DEFAULT_HZ },
    { "SQR", ADC_SQUARE_GPIO, ADC_SQUARE_CHAN, WAVE_SQUARE,   0, 0, DEFAULT_HZ },
    { "SAW", ADC_SAW_GPIO,    ADC_SAW_CHAN,    WAVE_SAW,       0, 0, DEFAULT_HZ },
};

#define VOICE_COUNT (sizeof(g_voices) / sizeof(g_voices[0]))

static ssd1306_t g_display;
static uint32_t g_disp_base; // slow-scrolling phase base for the mixed trace

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

// Map a [-1, +1] waveform value to a PWM duty cycle, leaving headroom at both rails.
static inline uint16_t wave_to_duty(float v) {
    float duty = (0.5f + 0.48f * v) * (float)PWM_TOP + 0.5f;
    if (duty < 0.0f) duty = 0.0f;
    if (duty > (float)PWM_TOP) duty = (float)PWM_TOP;
    return (uint16_t)duty;
}

// Sample a waveform at normalized phase p in [0, 1).
static inline float waveform_sample(wave_kind_t wave, float p) {
    switch (wave) {
        case WAVE_SINE:     return sinf(p * TWO_PI);
        case WAVE_SQUARE:   return (p < 0.5f) ? 1.0f : -1.0f;
        case WAVE_SAW:      return 2.0f * p - 1.0f;
        default:            return 0.0f;
    }
}

// Read one ADC channel, discarding the first sample after the mux switch and
// averaging the rest (rejects real noise).
static uint16_t read_adc_avg(uint chan) {
    adc_select_input(chan);
    (void)adc_read(); // discard first sample (mux settling)
    uint32_t sum = 0;
    for (uint i = 0; i < ADC_AVG_SAMPLES; i++) {
        sum += adc_read();
    }
    return (uint16_t)(sum / ADC_AVG_SAMPLES);
}

// Map a 12-bit ADC reading to a frequency in [FREQ_MIN_HZ, FREQ_MAX_HZ].
static inline float adc_to_freq(uint16_t raw) {
    return FREQ_MIN_HZ + ((float)raw / 4095.0f) * (FREQ_MAX_HZ - FREQ_MIN_HZ);
}

// ---------------------------------------------------------------------------
// Audio: mix all voices into a single PWM output.
// ---------------------------------------------------------------------------

static bool audio_sample_callback(struct repeating_timer *timer) {
    (void)timer;

    float mix = 0.0f;
    for (uint i = 0; i < VOICE_COUNT; i++) {
        voice_t *v = &g_voices[i];
        const float p = (float)v->phase / 4294967296.0f;
        mix += waveform_sample(v->wave, p);
        v->phase += v->phase_step;
    }
    mix /= (float)VOICE_COUNT; // normalize back to [-1, +1]

    pwm_set_gpio_level(AUDIO_GPIO, wave_to_duty(mix));
    return true;
}

// ---------------------------------------------------------------------------
// Display
// ---------------------------------------------------------------------------

static void update_display(void) {
    ssd1306_clear(&g_display);

    char buf[8];
    uint32_t step[VOICE_COUNT];

    for (uint i = 0; i < VOICE_COUNT; i++) {
        const voice_t *v = &g_voices[i];
        const int y = (int)i * 9;

        ssd1306_draw_text(&g_display, 0, y, v->label, 1);
        snprintf(buf, sizeof(buf), "%u", (unsigned)v->freq);
        ssd1306_draw_text(&g_display, 22, y, buf, 1);
        ssd1306_draw_text(&g_display, 48, y, "Hz", 1);

        // Per-voice per-pixel phase step so the trace reflects the real
        // frequency ratio (440 Hz -> 2 cycles across the screen).
        float cycles = v->freq / 220.0f;
        if (cycles < 0.5f) cycles = 0.5f;
        if (cycles > 6.0f) cycles = 6.0f;
        step[i] = (uint32_t)(cycles * 4294967296.0 / (double)SSD1306_WIDTH);
    }

    // Mixed trace: sum of the three waveforms at their current frequencies.
    g_disp_base += 0x08000000u;
    for (int x = 0; x < SSD1306_WIDTH; x++) {
        float mix = 0.0f;
        for (uint i = 0; i < VOICE_COUNT; i++) {
            uint32_t ph = g_disp_base + (uint32_t)((uint64_t)x * step[i]);
            mix += waveform_sample(g_voices[i].wave, (float)ph / 4294967296.0f);
        }
        mix /= (float)VOICE_COUNT;
        int y = 47 - (int)(15.0f * mix);
        ssd1306_set_pixel(&g_display, x, y, true);
    }

    ssd1306_flush(&g_display);
}

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------

int main(void) {
    const uint32_t default_step = (uint32_t)((DEFAULT_HZ * 4294967296.0) / (double)SAMPLE_RATE_HZ + 0.5);
    for (uint i = 0; i < VOICE_COUNT; i++) {
        g_voices[i].phase_step = default_step;
    }

    // Single mixed output on GP7 (slice 3, channel B).
    gpio_set_function(AUDIO_GPIO, GPIO_FUNC_PWM);
    pwm_config config = pwm_get_default_config();
    pwm_config_set_clkdiv(&config, 1.0f);
    pwm_config_set_wrap(&config, PWM_TOP);
    pwm_init(pwm_gpio_to_slice_num(AUDIO_GPIO), &config, true);

    // Potentiometer ADCs.
    adc_init();
    adc_gpio_init(ADC_SINE_GPIO);
    adc_gpio_init(ADC_SQUARE_GPIO);
    adc_gpio_init(ADC_SAW_GPIO);

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

    // Control loop: sample the pots at ~100 Hz, smooth, update phase steps.
    uint32_t tick = 0;
    while (true) {
        for (uint i = 0; i < VOICE_COUNT; i++) {
            voice_t *v = &g_voices[i];
            const uint16_t raw = read_adc_avg(v->adc_chan);
            const float target = adc_to_freq(raw);
            v->freq += 0.15f * (target - v->freq); // exponential smoothing
            v->phase_step = (uint32_t)((v->freq * 4294967296.0) / (double)SAMPLE_RATE_HZ + 0.5);
        }

        if ((tick % 10) == 0) { // display refresh at ~10 Hz
            update_display();
        }
        if ((tick % 20) == 0) { // log every ~200 ms
            printf("SIN=%4u SQR=%4u SAW=%4u Hz\n",
                   (unsigned)g_voices[0].freq,
                   (unsigned)g_voices[1].freq,
                   (unsigned)g_voices[2].freq);
        }
        tick++;
        sleep_ms(10);
    }
}
