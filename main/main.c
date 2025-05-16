#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include "hardware/clocks.h"
#include "FreeRTOS.h"
#include "task.h"

const int PAN_GPIO   = 14;
const int TILT_GPIO  = 15;
const int LDR_GPIO   = 26;
const int LDR_GPIO2  = 27;

float clockDiv;
float wrap;

void setMillis(int servoPin, float millis){
    pwm_set_gpio_level(servoPin,(millis/20000.f)*wrap);
}

void setServo(int servoPin, float startMillis){
    gpio_set_function(servoPin,GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(servoPin);
    pwm_config cfg = pwm_get_default_config();
    uint64_t clk = clock_get_hz(5);
    clockDiv = 64;
    while(clk/clockDiv/50 > 65535 && clockDiv < 256) clockDiv += 64;
    wrap = clk/clockDiv/50;
    pwm_config_set_clkdiv(&cfg,clockDiv);
    pwm_config_set_wrap(&cfg,wrap);
    pwm_init(slice,&cfg,true);
    setMillis(servoPin,startMillis);
}

static uint16_t ldr_to_us(uint16_t adc_val){
    const uint16_t min_us = 400;
    const uint16_t max_us = 2400;
    uint16_t pos = min_us + ((uint32_t)adc_val * (max_us - min_us) / 4095);
    if(pos > max_us) pos = max_us;
    return pos;
}

void ldr_1_task(void *pv){
    adc_gpio_init(LDR_GPIO);
    for(;;){
        adc_select_input(0);
        uint16_t v = adc_read();
        setMillis(PAN_GPIO, ldr_to_us(v));
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void ldr_2_task(void *pv){
    adc_gpio_init(LDR_GPIO2);
    for(;;){
        adc_select_input(1);
        uint16_t v = adc_read();
        setMillis(TILT_GPIO, ldr_to_us(v));
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

int main(){
    stdio_init_all();
    adc_init();
    setServo(PAN_GPIO, 1500);
    setServo(TILT_GPIO, 1500);
    gpio_set_function(0, GPIO_FUNC_UART);
    gpio_set_function(1, GPIO_FUNC_UART);
    xTaskCreate(ldr_1_task, "LDR1", 256, NULL, 1, NULL);
    xTaskCreate(ldr_2_task, "LDR2", 256, NULL, 1, NULL);
    vTaskStartScheduler();
    for(;;);
    return 0;
}
