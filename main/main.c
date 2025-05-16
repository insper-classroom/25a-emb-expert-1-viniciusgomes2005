#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"
#include "FreeRTOS.h"
#include "task.h"

const int PAN_GPIO   = 14;
const int TILT_GPIO  = 15;
const int LDR_GPIO =  26;
const int LDR_GPIO2  =27;


float clockDiv = 64;
float wrap = 39062;

void setMillis(int servoPin, float millis)
{
    pwm_set_gpio_level(servoPin, (millis/20000.f)*wrap);
}

void setServo(int servoPin, float startMillis)
{
    gpio_set_function(servoPin, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(servoPin);

    pwm_config config = pwm_get_default_config();
    
    uint64_t clockspeed = clock_get_hz(5);
    clockDiv = 64;
    wrap = 39062;

    while (clockspeed/clockDiv/50 > 65535 && clockDiv < 256) clockDiv += 64; 
    wrap = clockspeed/clockDiv/50;

    pwm_config_set_clkdiv(&config, clockDiv);
    pwm_config_set_wrap(&config, wrap);

    pwm_init(slice_num, &config, true);

    setMillis(servoPin, startMillis);
}


static uint16_t ldr_to_us(uint16_t adc_val) {
    const uint16_t min_us = 400;
    const uint16_t max_us = 2400;
    uint16_t pos = min_us + ((uint32_t)adc_val * (max_us - min_us) / 4095);
    if (pos < min_us) pos = min_us;
    if (pos > max_us) pos = max_us;
    return pos;
}

// void joystick_pwm_task(void *pvParameters) {
//     uint slice_pan  = pwm_gpio_to_slice_num(PAN_GPIO);
//     uint chan_pan   = pwm_gpio_to_channel(PAN_GPIO);
//     uint slice_tilt = pwm_gpio_to_slice_num(TILT_GPIO);
//     uint chan_tilt  = pwm_gpio_to_channel(TILT_GPIO);

//     for (;;) {
//         adc_select_input(0);
//         uint16_t x = adc_read();
//         adc_select_input(1);
//         uint16_t y = adc_read();

//         uint16_t pulse_pan  = adc_to_pwm(x);
//         uint16_t pulse_tilt = adc_to_pwm(y);

//         pwm_set_chan_level(slice_pan,  chan_pan,  pulse_pan);
//         pwm_set_chan_level(slice_tilt, chan_tilt, pulse_tilt);

//         vTaskDelay(pdMS_TO_TICKS(20));
//     }
// }
void ldr_1_task(void *pvParameters) {
    adc_gpio_init(LDR_GPIO);
    while(true){
        adc_select_input(0);
        uint16_t ldr_val = adc_read();
        uint16_t value = ldr_to_us(ldr_val);
        setMillis(PAN_GPIO, value);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
void ldr_2_task(void *pvParameters) {
    adc_gpio_init(LDR_GPIO2);
    while(true){
        adc_select_input(1);
        uint16_t ldr_val = adc_read();
        uint16_t value = ldr_to_us(ldr_val);
        setMillis(TILT_GPIO, value);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

int main() {
    stdio_init_all();
    adc_init();
    setServo(PAN_GPIO, 1500);
    setServo(TILT_GPIO, 1500);
    gpio_set_function(0, GPIO_FUNC_UART);
    gpio_set_function(1, GPIO_FUNC_UART);

    xTaskCreate(ldr_1_task, "LDR_1", 256, NULL, 1, NULL);
    xTaskCreate(ldr_2_task, "LDR_2", 256, NULL, 1, NULL);
    vTaskStartScheduler();

    for (;;);
    return 0;
}
