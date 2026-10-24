# RP2040 440 Hz sine starter (PWM + RC)

This is a small Pico SDK C project for Raspberry Pi Pico and Pico W. It generates a 440 Hz sine by updating the duty cycle of a fast PWM carrier at 44.1 kHz. An RC low-pass removes most of the carrier. The GPIO output is unamplified, unipolar (centered near 1.65 V), and intended for a scope or a high-impedance amplifier input.

## Files

```text
rp2040-sine-pwm/
├── CMakeLists.txt
└── src/
    └── main.c
```

## Wiring

Use a breadboard and a common ground. Connect:

```text
Pico GP16 ─── 1 kΩ ───●─── filtered output (scope probe tip)
                       │
                     100 nF
                       │
Pico GND ──────────────●──── scope ground clip
```

Put the 1 kΩ resistor in series from GP16, then connect the 100 nF capacitor from the resistor/output junction to GND. This gives a nominal cutoff of `1/(2πRC) ≈ 1.59 kHz`. Use a non-polar ceramic capacitor. Do not connect the scope ground clip to GP16. Do not connect headphones or a speaker directly to the Pico pin; use an audio amplifier. The filtered signal is still biased around half the 3.3 V supply; for an amplifier input, use AC coupling (for example a 10 µF capacitor in series) and follow the amplifier input requirements.

The PWM carrier is approximately 488 kHz with the default 125 MHz system clock. The starter schedules samples every 23 µs (about 43.48 kHz), because the SDK repeating-microsecond timer accepts integer microseconds; the DDS phase increment compensates for this and keeps the tone near 440 Hz. The software callback is suitable for this first experiment; later stages replace it with DMA paced by PWM for steadier audio timing.

## Install and build

Install CMake, Ninja, an Arm GNU embedded toolchain and the Raspberry Pi Pico SDK following Raspberry Pi's current [C/C++ SDK setup guide](https://www.raspberrypi.com/documentation/microcontrollers/c_sdk.html). Clone `pico-sdk` and initialize its submodules once:

```sh
git clone https://github.com/raspberrypi/pico-sdk.git
cd pico-sdk
git submodule update --init
```

From this project's root (set the SDK path to the actual clone location):

```sh
cmake -S . -B build -G Ninja -DPICO_SDK_PATH=/absolute/path/to/pico-sdk -DPICO_BOARD=pico
cmake --build build
```

For a Pico W, replace `-DPICO_BOARD=pico` with `-DPICO_BOARD=pico_w`. CMake creates `build/sine_pwm.uf2`.

## Flash

1. Disconnect the Pico from USB.
2. Hold **BOOTSEL** while connecting USB. Release the button after the `RPI-RP2` drive appears.
3. Copy `build/sine_pwm.uf2` onto that drive. It will disconnect and reboot automatically.

Alternatively, use the Pico SDK's `picotool` load command if installed and the board is accessible in BOOTSEL mode.

## Oscilloscope check

1. Attach the probe tip to the node after the 1 kΩ resistor, and its ground clip to Pico GND. Start with a 1× or 10× probe, DC coupling, and a 1 V/div scale.
2. Set the timebase near 500 µs/div to view several cycles. You should see a sine-like waveform centered near 1.65 V, with a period near 2.27 ms (about 440 Hz).
3. Use the scope's frequency/measurement function or cursors: expect roughly 440 Hz. Small error can come from the clock tolerance and timing jitter.
4. Zoom in to 1–5 µs/div. Some residual carrier ripple is normal. The carrier is far above the 440 Hz tone and should be around 488 kHz before filtering.
5. Compare the signal at GP16 (before the resistor/capacitor) with the filtered node. Before the filter it should look like fast PWM whose duty changes smoothly; after it the carrier should be much smaller.

The simple one-pole RC is a teaching filter, not a high-fidelity DAC. Reduce the capacitor value if the wanted tone looks overly attenuated; increase filter order (two or more buffered RC stages, or a proper active filter) to reduce carrier ripple while preserving audio bandwidth.

## Staged development plan

### 1. Stable tone controls

Add one analog input first and measure its raw range. The RP2040 has three external ADC inputs on GP26–GP28 (ADC0–ADC2); its fourth ADC input is connected internally to the chip temperature sensor. Six independent knobs therefore need an external ADC or an analog multiplexer. A practical option is MCP3008 (8 channels, SPI). Wire its analog supply/reference and grounds as required by its data sheet, connect SCLK/MOSI/MISO/CS to free Pico GPIOs, and wire each pot's outer legs to 3V3 and GND with its wiper to a separate MCP3008 channel. Use 50 kΩ pots; sample controls at about 100–500 Hz, average a few readings, and smooth parameter changes rather than reading ADCs inside the audio callback.

Map the first knob to frequency, e.g. 80–1000 Hz, and another to output amplitude. Keep audio work and control scanning separate. Avoid `sleep_ms`, I²C display writes, printing, and floating-point mapping in the time-critical sample callback.

### 2. Four waveform controls and mixing

Define four normalized oscillator sources (sine, triangle, sawtooth and pulse/square). Give each a dedicated pot for its level; normalize the knob reading to 0–1, apply a taper in control code if desired, then mix:

```c
mix = level_sine * sine + level_triangle * triangle
    + level_saw * saw + level_pulse * pulse;
sample = 0.5f + output_gain * mix;
```

Clamp or softly limit `mix` so duty remains within a safe range, and keep headroom when multiple knobs are fully up. Here “pulse width” is a separate waveform-shape parameter (duty within the square/pulse oscillator), not frequency. Keep frequency and amplitude on the remaining two controls as planned.

### 3. OLED waveform display

Identify whether the OLED is SSD1306 or SH1106 and its I²C address (commonly `0x3C` or `0x3D`). The Pico SDK provides I²C through `hardware_i2c`; add `hardware_i2c` to the target link libraries and initialize an available I²C instance and GPIO pair (for example I2C0 on GP4/GP5, SDA/SCL). Connect OLED VCC to the module's supported supply, GND to GND, SDA to GP4 and SCL to GP5; confirm module voltage compatibility. Add a display driver and keep the display refresh (say 10–20 Hz) outside audio generation.

Draw from a slow, copied snapshot of oscillator parameters or a small display buffer; never let I²C transactions run inside the sample callback. Show the four source traces and the final mixed trace with a shared time/voltage scale.

### 4. Robust audio timing and quality

The starter's repeating timer and `sinf` are intentionally simple. For a robust instrument, precompute wavetable(s) and use a phase accumulator in integer/fixed-point arithmetic. Use PWM wrap interrupts only to pace DMA transfers into the PWM compare register (one sample per wrap), with a double buffer/ring buffer and an underrun counter. Keep USB/controls/OLED on foreground code or the other core, communicate through small atomic snapshots, and verify with the scope that sample spacing is stable under UI activity. For better audio quality, use an external I²S DAC or an audio codec rather than relying on single-pole-filtered PWM.

## Current SDK API notes

The starter uses the documented `pwm_gpio_to_slice_num`, `pwm_get_default_config`, `pwm_config_set_clkdiv`, `pwm_config_set_wrap`, `pwm_init`, `pwm_set_gpio_level`, and `add_repeating_timer_us` APIs. See the official [hardware PWM API](https://www.raspberrypi.com/documentation/pico-sdk/hardware.html) and the Raspberry Pi [pico-sdk repository/setup instructions](https://github.com/raspberrypi/pico-sdk).
