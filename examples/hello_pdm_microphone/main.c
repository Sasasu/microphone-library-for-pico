/*
 * Copyright (c) 2021 Arm Limited and Contributors. All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * This examples captures data from a PDM microphone using a sample
 * rate of 8 kHz and prints the sample values over the USB serial
 * connection.
 */

#include <stdio.h>
#include <string.h>

#include "pico/pdm_microphone.h"
#include "pico/stdlib.h"
#include "tusb.h"

#include "hardware/irq.h"
#include "hardware/pwm.h"

// configuration
const struct pdm_microphone_config config = {
    // GPIO pin for the PDM DAT signal
    .gpio_data = 13,

    // GPIO pin for the PDM CLK signal
    .gpio_clk = 16,

    // PIO instance to use
    .pio = pio0,

    // PIO State Machine instance to use
    .pio_sm = 0,

    // sample rate in Hz
    .sample_rate = 8000,

    // number of samples to buffer
    .sample_buffer_size = 256,
};

#define SAMPLE_BUFFER_SIZE 256
// variables
uint16_t sample_buffer[256];
volatile int samples_read = 0;

void on_pwm();

static const int PWM_PIN = 23;
static uint16_t pwm_l = 0;
static const uint16_t pwm_l_min = 900, pwm_l_max = 1200;
bool l_d = 0;
void on_pwm() {
  uint16_t l = 0;

  if (samples_read == 0)
    return;
  samples_read = 0;

  for (int i = 0; i < samples_read * 8; i++) {
    int bytes = i / 8;
    int bits = i % 8;

    int d = (sample_buffer[bytes] & 0b1 << bits) >> bits;

    l += (l_d != d);

    l_d = d;
  }

  if (l == 0)
    return;

  l = ((double)(l - pwm_l_min) / (pwm_l_max - pwm_l_min)) * 255;
  pwm_l = l * l;

  // printf("%d \n", pwm_l);
  pwm_set_gpio_level(PWM_PIN, pwm_l);
}

void on_pdm_samples_ready() {
  // callback from library when all the samples in the library
  // internal sample buffer are ready for reading
  samples_read = pdm_microphone_read((int16_t *)sample_buffer, 256);

  on_pwm();
}

int main(void) {
  // initialize stdio and wait for USB CDC connect
  stdio_init_all();

  while (!tud_cdc_connected()) {
    tight_loop_contents();
  }

  {
    gpio_set_function(PWM_PIN, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(PWM_PIN);

    pwm_config config = pwm_get_default_config();
    pwm_config_set_clkdiv(&config, 100);
    pwm_init(slice_num, &config, true);
  }

  // initialize the PDM microphone
  if (pdm_microphone_init(&config) < 0) {
    printf("PDM microphone initialization failed!\n");
    while (1) {
      tight_loop_contents();
    }
  }

  // set callback that is called when all the samples in the library
  // internal sample buffer are ready for reading
  pdm_microphone_set_samples_ready_handler(on_pdm_samples_ready);

  // start capturing data from the PDM microphone
  if (pdm_microphone_start() < 0) {
    printf("PDM microphone start failed!\n");
  }

  printf("hello PDM microphone\n");

  return 0;
}
