#include "LEDC_PWM.h"

#include "LEDC_PWM.h"

esp_err_t pwm_init(void) {
    // 配置 LEDC timer
    ledc_timer_config_t ledc_timer = {
        .duty_resolution = PWM_RESOLUTION,
        .freq_hz = PWM_FREQ,
        .speed_mode = PWM_MODE,
        .timer_num = LEDC_TIMER_0,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // 配置 LEDC channel，直接设置 duty 为 0
    ledc_channel_config_t ledc_channel = {
        .channel = PWM_CHANNEL,
        .duty = 0,                  // 设置为0，确保不震动
        .gpio_num = PWM_GPIO,
        .speed_mode = PWM_MODE,
        .hpoint = 0,
        .timer_sel = LEDC_TIMER_0
    };
    return ledc_channel_config(&ledc_channel);
}

esp_err_t pwm_set_duty(uint8_t duty) {
    ESP_ERROR_CHECK(ledc_set_duty(PWM_MODE, PWM_CHANNEL, duty));
    return ledc_update_duty(PWM_MODE, PWM_CHANNEL);
}

esp_err_t pwm_stop(void) {
    return pwm_set_duty(0);
}