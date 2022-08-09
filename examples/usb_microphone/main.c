/*
 * Copyright (c) 2021 Arm Limited and Contributors. All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * This examples creates a USB Microphone device using the TinyUSB
 * library and captures data from a PDM microphone using a sample
 * rate of 16 kHz, to be sent the to PC.
 *
 * The USB microphone code is based on the TinyUSB audio_test example.
 *
 * https://github.com/hathach/tinyusb/tree/master/examples/device/audio_test
 */

#include "pico/pdm_microphone.h"

#include "pico/stdlib.h"

#include "hardware/irq.h"
#include "hardware/pwm.h"

#include "usb_microphone.h"

// configuration
const struct pdm_microphone_config config = {
    .gpio_data = 13,
    .gpio_clk = 16,
    .pio = pio0,
    .pio_sm = 0,
    .sample_rate = SAMPLE_RATE,
    .sample_buffer_size = SAMPLE_BUFFER_SIZE,
    .sample_depth = CFG_TUD_AUDIO_FUNC_1_N_BYTES_PER_SAMPLE_TX * 8,
};

// variables
int16_t sample_buffer[SAMPLE_BUFFER_SIZE];

static const int PWM_PIN = 23;

// callback functions
void on_pdm_samples_ready();
void on_usb_microphone_tx_ready();

void on_pwm() {
  uint32_t l = 0;

  for (int i = 0; i < SAMPLE_BUFFER_SIZE; i++) {
    uint16_t d =
        sample_buffer[i] < 0 ? -1 * sample_buffer[i] : sample_buffer[i];

    l += d;
  }

  l = l / SAMPLE_BUFFER_SIZE;

  l = (((double)l - 300) / (1800 - 300)) * 255;

  pwm_set_gpio_level(PWM_PIN, l * l);
}

int main(void) {
  // initialize and start the PDM microphone
  pdm_microphone_init(&config);
  pdm_microphone_set_samples_ready_handler(on_pdm_samples_ready);
  pdm_microphone_set_filter_gain(1);
  pdm_microphone_start();

  {
    gpio_set_function(PWM_PIN, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(PWM_PIN);
    pwm_config config = pwm_get_default_config();
    pwm_config_set_clkdiv(&config, 1.f);
    pwm_init(slice_num, &config, true);
  }

  // initialize the USB microphone interface
  usb_microphone_init();
  usb_microphone_set_tx_ready_handler(on_usb_microphone_tx_ready);

  while (1) {
    // run the USB microphone task continuously
    usb_microphone_task();
  }

  return 0;
}

void on_pdm_samples_ready() {
  // Callback from library when all the samples in the library
  // internal sample buffer are ready for reading.
  //
  // Read new samples into local buffer.
  pdm_microphone_read(sample_buffer, SAMPLE_BUFFER_SIZE);

  on_pwm();
}

static uint8_t usb_buffer[768] = {0};
void on_usb_microphone_tx_ready() {
  // Callback from TinyUSB library when all data is ready
  // to be transmitted.
  //
  // Write local buffer to the USB microphone
  // for (int i = 0; i < SAMPLE_BUFFER_SIZE; i++) {
    // *(uint16_t *)(usb_buffer + (i * 3)) = sample_buffer[i];
  // }
  // usb_microphone_write(usb_buffer, sizeof(usb_buffer));
  usb_microphone_write(sample_buffer, sizeof(sample_buffer));
}
