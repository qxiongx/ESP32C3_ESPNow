#ifndef LEDC_PWM_H
#define LEDC_PWM_H

#include "driver/ledc.h"
#include "esp_err.h"

// PWM配置参数
#define PWM_GPIO 9
#define PWM_FREQ 5000
#define PWM_RESOLUTION LEDC_TIMER_8_BIT
#define PWM_CHANNEL LEDC_CHANNEL_0
#define PWM_MODE LEDC_LOW_SPEED_MODE

// 函数声明
esp_err_t pwm_init(void);
esp_err_t pwm_set_duty(uint8_t duty);
esp_err_t pwm_stop(void);

#endif