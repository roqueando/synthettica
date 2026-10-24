#include <math.h>
#include <stdio.h>
#include <stdint.h>

#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/timer.h"
#include "hardware/adc.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"
#include "pico/stdio.h"

#include "ssd1306.h"

// ---------------------------------------------------------------------------
// Single mixed audio output (PWM).
// ---------------------------------------------------------------------------
#define AUDIO_GPIO      7u    // PWM slice 3, channel B

// Push buttons (active low, internal pull-up).
#define BUTTON_KICK_GPIO   4u   // toggles the kick drum on/off
#define BUTTON_SNARE_GPIO  5u   // fires the snare immediately (free-form)

// Frequency potentiometers (one per voice).
#define ADC_SINE_GPIO    26u
#define ADC_SINE_CHAN    0u
#define ADC_SQUARE_GPIO  27u
#define ADC_SQUARE_CHAN  1u
#define ADC_SAW_GPIO     28u
#define ADC_SAW_CHAN     2u

// BPM potentiometer on GP29 = ADC3 (RP2040-Zero exposes GP29).
#define BPM_ADC_GPIO     29u
#define BPM_ADC_CHAN     3u

// OLED on I2C1: SDA = GP14, SCL = GP15.
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

#define BPM_MIN          10.0f
#define BPM_MAX          120.0f
#define DEFAULT_BPM      120.0f

#define ADC_AVG_SAMPLES  8u

// 2 full cycles across the 128px screen = 2^32 / 64.
#define WAVE_PHASE_STEP  67108864u

// Sine wavetable (avoids sinf/division inside the audio callback).
#define SINE_TABLE_SIZE  1024u
#define SINE_TABLE_SHIFT 22u    // 32 - log2(1024)
#define PHASE_PER_HZ     (4294967296.0f / SAMPLE_RATE_HZ)

// Kick drum synthesis (808-style: sine with pitch + amplitude decay).
#define KICK_START_HZ     180.0f
#define KICK_END_HZ       45.0f
#define KICK_SWEEP_TAU_S  0.02f
#define KICK_AMP_TAU_S    0.08f
#define KICK_DURATION_S   0.5f
#define KICK_SAMPLES      ((uint32_t)(SAMPLE_RATE_HZ * KICK_DURATION_S))

// Snare drum synthesis: body tone + white-noise burst, fast decay.
#define SNARE_TONE_HZ      180.0f
#define SNARE_TONE_AMP     0.5f
#define SNARE_NOISE_AMP    0.7f
#define SNARE_TONE_TAU_S   0.04f
#define SNARE_NOISE_TAU_S  0.03f
#define SNARE_DURATION_S   0.3f
#define SNARE_SAMPLES      ((uint32_t)(SAMPLE_RATE_HZ * SNARE_DURATION_S))

// Output mix levels.
#define VOICE_MIX_LEVEL    0.45f
#define KICK_LEVEL         0.60f
#define SNARE_LEVEL        0.70f

// Display indicator ticks (10 ms each).
#define BEAT_FLASH_TICKS   10u
#define SNARE_FLASH_TICKS  5u

typedef enum {
    WAVE_SINE = 0,
    WAVE_SQUARE,
    WAVE_SAW,
} wave_kind_t;

typedef struct {
    const char *label;
    uint        adc_gpio;
    uint        adc_chan;
    wave_kind_t wave;
    uint32_t    phase;      // owned by callback
    volatile uint32_t phase_step; // written by main loop, read by callback
    float       freq;       // owned by main loop
} voice_t;

static voice_t g_voices[3] = {
    { "SIN", ADC_SINE_GPIO,   ADC_SINE_CHAN,   WAVE_SINE,   0, 0, DEFAULT_HZ },
    { "SQR", ADC_SQUARE_GPIO, ADC_SQUARE_CHAN, WAVE_SQUARE, 0, 0, DEFAULT_HZ },
    { "SAW", ADC_SAW_GPIO,    ADC_SAW_CHAN,    WAVE_SAW,    0, 0, DEFAULT_HZ },
};

#define VOICE_COUNT (sizeof(g_voices) / sizeof(g_voices[0]))

static ssd1306_t g_display;
static uint32_t g_disp_base; // slow-scrolling phase base for the mixed trace

// Tempo / step clock shared between main loop and audio callback.
// Steps are half-beats (8th notes): even = downbeat (kick), odd = offbeat (snare).
static volatile uint32_t g_samples_per_half_beat;
static volatile bool     g_kick_enabled;
static volatile uint32_t g_beat_flash;
static uint32_t          g_step_countdown;
static bool              g_step_even;

// Kick synthesis state (owned by the audio callback).
static uint32_t g_kick_time = KICK_SAMPLES; // start inactive
static uint32_t g_kick_phase;
static float    g_kick_freq;
static float    g_kick_amp;
static float    g_kick_sweep_coeff;
static float    g_kick_amp_coeff;

// Snare state.
static volatile bool     g_snare_enabled;   // toggled by button, fires on beat
static volatile uint32_t g_snare_flash;
static uint32_t g_snare_time = SNARE_SAMPLES; // start inactive
static uint32_t g_snare_tone_phase;
static uint32_t g_snare_lfsr = 0x12345678u;
static float    g_snare_tone_amp;
static float    g_snare_noise_amp;
static float    g_snare_tone_coeff;
static float    g_snare_noise_coeff;

static float g_sine_table[SINE_TABLE_SIZE];

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

static inline uint16_t wave_to_duty(float v) {
    float duty = (0.5f + 0.48f * v) * (float)PWM_TOP + 0.5f;
    if (duty < 0.0f) duty = 0.0f;
    if (duty > (float)PWM_TOP) duty = (float)PWM_TOP;
    return (uint16_t)duty;
}

static inline float sine_lookup(uint32_t phase) {
    return g_sine_table[(phase >> SINE_TABLE_SHIFT) & (SINE_TABLE_SIZE - 1)];
}

static inline float waveform_sample(wave_kind_t wave, uint32_t phase) {
    switch (wave) {
        case WAVE_SINE:   return sine_lookup(phase);
        case WAVE_SQUARE: return (phase & 0x80000000u) ? 1.0f : -1.0f;
        case WAVE_SAW:    return ((float)(phase >> 16) * (2.0f / 65536.0f)) - 1.0f;
        default:          return 0.0f;
    }
}

static uint16_t read_adc_avg(uint chan) {
    adc_select_input(chan);
    (void)adc_read(); // discard first sample (mux settling)
    uint32_t sum = 0;
    for (uint i = 0; i < ADC_AVG_SAMPLES; i++) {
        sum += adc_read();
    }
    return (uint16_t)(sum / ADC_AVG_SAMPLES);
}

static inline float adc_to_freq(uint16_t raw) {
    return FREQ_MIN_HZ + ((float)raw / 4095.0f) * (FREQ_MAX_HZ - FREQ_MIN_HZ);
}

static inline float adc_to_bpm(uint16_t raw) {
    return BPM_MIN + ((float)raw / 4095.0f) * (BPM_MAX - BPM_MIN);
}

// ---------------------------------------------------------------------------
// Kick drum
// ---------------------------------------------------------------------------

static void trigger_kick(void) {
    g_kick_time  = 0;
    g_kick_phase = 0;
    g_kick_freq  = KICK_START_HZ;
    g_kick_amp   = 1.0f;
}

static inline float kick_sample(void) {
    if (g_kick_time >= KICK_SAMPLES) {
        return 0.0f;
    }
    const float s = sine_lookup(g_kick_phase) * g_kick_amp;

    g_kick_phase += (uint32_t)(g_kick_freq * PHASE_PER_HZ);
    if (g_kick_freq > KICK_END_HZ) g_kick_freq *= g_kick_sweep_coeff;
    else                           g_kick_freq  = KICK_END_HZ;
    g_kick_amp *= g_kick_amp_coeff;
    g_kick_time++;
    return s;
}

static void trigger_snare(void) {
    g_snare_time       = 0;
    g_snare_tone_phase = 0;
    g_snare_tone_amp   = 1.0f;
    g_snare_noise_amp  = 1.0f;
    if (g_snare_lfsr == 0) g_snare_lfsr = 0x12345678u;
}

static inline float snare_sample(void) {
    if (g_snare_time >= SNARE_SAMPLES) {
        return 0.0f;
    }

    // White noise via a 32-bit xorshift LFSR -> [-1, 1).
    g_snare_lfsr ^= g_snare_lfsr << 13;
    g_snare_lfsr ^= g_snare_lfsr >> 17;
    g_snare_lfsr ^= g_snare_lfsr << 5;
    const float noise = ((float)(g_snare_lfsr & 0x00FFFFFFu) * (2.0f / 16777216.0f)) - 1.0f;

    const float tone = sine_lookup(g_snare_tone_phase);

    g_snare_tone_phase += (uint32_t)(SNARE_TONE_HZ * PHASE_PER_HZ);
    g_snare_tone_amp   *= g_snare_tone_coeff;
    g_snare_noise_amp  *= g_snare_noise_coeff;
    g_snare_time++;

    return SNARE_TONE_AMP  * tone  * g_snare_tone_amp
         + SNARE_NOISE_AMP * noise * g_snare_noise_amp;
}

// ---------------------------------------------------------------------------
// Audio callback
// ---------------------------------------------------------------------------

static bool audio_sample_callback(struct repeating_timer *timer) {
    (void)timer;

    float mix = 0.0f;
    for (uint i = 0; i < VOICE_COUNT; i++) {
        voice_t *v = &g_voices[i];
        mix += waveform_sample(v->wave, v->phase);
        v->phase += v->phase_step;
    }
    mix *= (1.0f / (float)VOICE_COUNT); // normalized mix (multiply, no divide)

    // Step clock: downbeat fires the kick, offbeat fires the snare.
    if (g_step_countdown == 0) {
        if (g_step_even) {
            if (g_kick_enabled) {
                trigger_kick();
            }
            g_beat_flash = BEAT_FLASH_TICKS;
        } else {
            if (g_snare_enabled) {
                trigger_snare();
                g_snare_flash = SNARE_FLASH_TICKS;
            }
        }
        g_step_even = !g_step_even;
        g_step_countdown = g_samples_per_half_beat;
    }
    g_step_countdown--;

    const float out = VOICE_MIX_LEVEL * mix
                    + KICK_LEVEL      * kick_sample()
                    + SNARE_LEVEL     * snare_sample();
    pwm_set_gpio_level(AUDIO_GPIO, wave_to_duty(out));
    return true;
}

// ---------------------------------------------------------------------------
// Buttons (debounced)
// ---------------------------------------------------------------------------

static void poll_kick_button(void) {
    static bool     last  = true;   // idle high
    static uint32_t count = 0;

    const bool now = gpio_get(BUTTON_KICK_GPIO);
    if (now == last) {
        count = 0;
    } else if (++count >= 3) {      // stable for ~30 ms
        last  = now;
        count = 0;
        if (!now) {                 // pressed (active low)
            g_kick_enabled = !g_kick_enabled;
        }
    }
}

static void poll_snare_button(void) {
    static bool     last  = true;
    static uint32_t count = 0;

    const bool now = gpio_get(BUTTON_SNARE_GPIO);
    if (now == last) {
        count = 0;
    } else if (++count >= 3) {
        last  = now;
        count = 0;
        if (!now) {                 // pressed (active low)
            g_snare_enabled = !g_snare_enabled;
        }
    }
}

// ---------------------------------------------------------------------------
// Display
// ---------------------------------------------------------------------------

static void update_display(float bpm) {
    ssd1306_clear(&g_display);

    char buf[8];
    uint32_t step[VOICE_COUNT];

    for (uint i = 0; i < VOICE_COUNT; i++) {
        const voice_t *v = &g_voices[i];
        const int y = (int)i * 8;

        ssd1306_draw_text(&g_display, 0, y, v->label, 1);
        snprintf(buf, sizeof(buf), "%u", (unsigned)v->freq);
        ssd1306_draw_text(&g_display, 22, y, buf, 1);
        ssd1306_draw_text(&g_display, 48, y, "Hz", 1);

        float cycles = v->freq / 220.0f;
        if (cycles < 0.5f) cycles = 0.5f;
        if (cycles > 6.0f) cycles = 6.0f;
        step[i] = (uint32_t)(cycles * 4294967296.0 / (double)SSD1306_WIDTH);
    }

    // BPM + kick status row.
    ssd1306_draw_text(&g_display, 0, 24, "BPM", 1);
    snprintf(buf, sizeof(buf), "%u", (unsigned)bpm);
    ssd1306_draw_text(&g_display, 22, 24, buf, 1);
    ssd1306_draw_text(&g_display, 64, 24,  g_kick_enabled  ? "KON"  : "KOFF", 1);
    ssd1306_draw_text(&g_display, 100, 24, g_snare_enabled ? "SON"  : "SOFF", 1);

    // Beat flash indicator (top-right, left square).
    if (g_beat_flash > 0) {
        for (int x = 112; x < 120; x++) {
            for (int y = 0; y < 8; y++) {
                ssd1306_set_pixel(&g_display, x, y, true);
            }
        }
    }
    // Snare hit indicator (top-right, right square).
    if (g_snare_flash > 0) {
        for (int x = 120; x < SSD1306_WIDTH; x++) {
            for (int y = 0; y < 8; y++) {
                ssd1306_set_pixel(&g_display, x, y, true);
            }
        }
    }

    // Mixed trace of the three voices.
    g_disp_base += 0x08000000u;
    for (int x = 0; x < SSD1306_WIDTH; x++) {
        float mix = 0.0f;
        for (uint i = 0; i < VOICE_COUNT; i++) {
            uint32_t ph = g_disp_base + (uint32_t)((uint64_t)x * step[i]);
            mix += waveform_sample(g_voices[i].wave, ph);
        }
        mix /= (float)VOICE_COUNT;
        int y = 48 - (int)(14.0f * mix);
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

    g_samples_per_half_beat = (uint32_t)(SAMPLE_RATE_HZ * 30.0 / DEFAULT_BPM + 0.5);
    g_step_countdown        = g_samples_per_half_beat;
    g_step_even             = true;

    // Precompute the sine wavetable (no sinf inside the audio callback).
    for (uint i = 0; i < SINE_TABLE_SIZE; i++) {
        g_sine_table[i] = sinf((float)i * TWO_PI / (float)SINE_TABLE_SIZE);
    }

    // Kick exponential decay coefficients (computed once).
    g_kick_sweep_coeff = expf(-1.0f / (SAMPLE_RATE_HZ * KICK_SWEEP_TAU_S));
    g_kick_amp_coeff   = expf(-1.0f / (SAMPLE_RATE_HZ * KICK_AMP_TAU_S));

    // Snare exponential decay coefficients (computed once).
    g_snare_tone_coeff  = expf(-1.0f / (SAMPLE_RATE_HZ * SNARE_TONE_TAU_S));
    g_snare_noise_coeff = expf(-1.0f / (SAMPLE_RATE_HZ * SNARE_NOISE_TAU_S));

    // Audio output on GP7 (slice 3, channel B).
    gpio_set_function(AUDIO_GPIO, GPIO_FUNC_PWM);
    pwm_config config = pwm_get_default_config();
    pwm_config_set_clkdiv(&config, 1.0f);
    pwm_config_set_wrap(&config, PWM_TOP);
    pwm_init(pwm_gpio_to_slice_num(AUDIO_GPIO), &config, true);

    // Potentiometer ADCs (voices + BPM).
    adc_init();
    adc_gpio_init(ADC_SINE_GPIO);
    adc_gpio_init(ADC_SQUARE_GPIO);
    adc_gpio_init(ADC_SAW_GPIO);
    adc_gpio_init(BPM_ADC_GPIO);

    // Push buttons (active low, internal pull-up).
    gpio_init(BUTTON_KICK_GPIO);
    gpio_set_dir(BUTTON_KICK_GPIO, GPIO_IN);
    gpio_pull_up(BUTTON_KICK_GPIO);

    gpio_init(BUTTON_SNARE_GPIO);
    gpio_set_dir(BUTTON_SNARE_GPIO, GPIO_IN);
    gpio_pull_up(BUTTON_SNARE_GPIO);

    // OLED on I2C1 (GP14 SDA / GP15 SCL).
    i2c_init(OLED_I2C, 400 * 1000);
    gpio_set_function(OLED_SDA, GPIO_FUNC_I2C);
    gpio_set_function(OLED_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(OLED_SDA);
    gpio_pull_up(OLED_SCL);
    ssd1306_init(&g_display, OLED_I2C, OLED_ADDR, OLED_COL_OFFSET);

    // USB CDC serial (printf).
    stdio_init_all();

    // Negative interval -> deadlines scheduled from the previous deadline (no drift).
    struct repeating_timer audio_timer;
    const int64_t sample_period_us = -23;
    if (!add_repeating_timer_us(sample_period_us, audio_sample_callback, NULL, &audio_timer)) {
        while (true) tight_loop_contents();
    }

    // Control loop (~100 Hz): pots, button, display, log.
    float bpm = DEFAULT_BPM;
    uint32_t tick = 0;
    while (true) {
        for (uint i = 0; i < VOICE_COUNT; i++) {
            voice_t *v = &g_voices[i];
            const uint16_t raw = read_adc_avg(v->adc_chan);
            const float target = adc_to_freq(raw);
            v->freq += 0.15f * (target - v->freq);
            v->phase_step = (uint32_t)((v->freq * 4294967296.0) / (double)SAMPLE_RATE_HZ + 0.5);
        }

        const uint16_t raw_bpm = read_adc_avg(BPM_ADC_CHAN);
        const float bpm_target = adc_to_bpm(raw_bpm);
        bpm += 0.15f * (bpm_target - bpm);
        g_samples_per_half_beat = (uint32_t)(SAMPLE_RATE_HZ * 30.0 / bpm + 0.5);

        poll_kick_button();
        poll_snare_button();
        if (g_beat_flash  > 0) g_beat_flash--;
        if (g_snare_flash > 0) g_snare_flash--;

        if ((tick % 10) == 0) {
            update_display(bpm);
        }
        if ((tick % 20) == 0) {
            printf("SIN=%4u SQR=%4u SAW=%4u BPM=%3u KICK=%s SNR=%s\n",
                   (unsigned)g_voices[0].freq,
                   (unsigned)g_voices[1].freq,
                   (unsigned)g_voices[2].freq,
                   (unsigned)bpm,
                   g_kick_enabled  ? "ON" : "OFF",
                   g_snare_enabled ? "ON" : "OFF");
        }
        tick++;
        sleep_ms(10);
    }
}
