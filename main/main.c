#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include "hardware/clocks.h"
#include "FreeRTOS.h"
#include "task.h"

#define PAN_GPIO   14
#define TILT_GPIO  15
#define LDR1_GPIO  26
#define LDR2_GPIO  27

typedef struct {
    uint slice;
    uint channel;
    float clkdiv;
    uint32_t wrap;
} servo_t;

static uint16_t ldr_to_us(uint16_t adc_val) {
    const uint16_t MIN_US = 400;
    const uint16_t MAX_US = 2400;
    uint32_t pos = MIN_US + ((uint32_t)adc_val * (MAX_US - MIN_US) / 4095);
    return pos > MAX_US ? MAX_US : pos;
}

static servo_t servo_init(int gpio, float start_us) {
    servo_t s;
    gpio_set_function(gpio, GPIO_FUNC_PWM);
    s.slice   = pwm_gpio_to_slice_num(gpio);
    s.channel = pwm_gpio_to_channel(gpio);
    pwm_config cfg = pwm_get_default_config();
    uint64_t clk_hz = clock_get_hz(clk_sys);
    s.clkdiv = 64.0f;
    while (clk_hz / s.clkdiv / 50 > 0xFFFF && s.clkdiv < 256.0f) {
        s.clkdiv += 64.0f;
    }
    s.wrap = (uint32_t)(clk_hz / s.clkdiv / 50);
    pwm_config_set_clkdiv(&cfg, s.clkdiv);
    pwm_config_set_wrap  (&cfg, s.wrap);
    pwm_init(s.slice, &cfg, true);
    pwm_set_chan_level(s.slice, s.channel, (start_us / 20000.0f) * s.wrap);
    return s;
}

static void ldr1_task(void *pv) {
    servo_t *pan = (servo_t *)pv;
    adc_gpio_init(LDR1_GPIO);
    for (;;) {
        adc_select_input(0);
        uint16_t v = adc_read();
        uint32_t lvl = (ldr_to_us(v) / 20000.0f) * pan->wrap;
        pwm_set_chan_level(pan->slice, pan->channel, lvl);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

static void ldr2_task(void *pv) {
    servo_t *tilt = (servo_t *)pv;
    adc_gpio_init(LDR2_GPIO);
    for (;;) {
        adc_select_input(1);
        uint16_t v = adc_read();
        uint32_t lvl = (ldr_to_us(v) / 20000.0f) * tilt->wrap;
        pwm_set_chan_level(tilt->slice, tilt->channel, lvl);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

int main() {
    stdio_init_all();
    adc_init();
    servo_t pan  = servo_init(PAN_GPIO, 1500.0f);
    servo_t tilt = servo_init(TILT_GPIO,1500.0f);
    xTaskCreate(ldr1_task, "LDR1", 256, &pan,  1, NULL);
    xTaskCreate(ldr2_task, "LDR2", 256, &tilt, 1, NULL);
    vTaskStartScheduler();
    for (;;);
    return 0;
}
