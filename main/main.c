/**
 * @file main.c
 * @brief Main application file for ESP32C3_ESPNow_V2.0 project.
 */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include "esp_log.h"
#include "ADS1256.h"
#include "esp32c3/rom/ets_sys.h"
#include "mpu9250.h"
#include <esp_now.h>
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "string.h"
#include "CW2015.h"
#include "LEDC_PWM.h"

static const char *TAG = "MAIN";

// 目标设备的 MAC 地址（ESP8266_EMG 的 MAC 地址）
uint8_t broadcastAddress[] = {0xd8, 0xbc, 0x38, 0xad, 0x24, 0x41};

// 定义要发送的数据结构
typedef struct {
    float voltages[8];
    float acc_x;      // X轴加速度
    float acc_y;      // Y轴加速度
    float acc_z;      // Z轴加速度
    float gyro_x;     // X轴角速度
    float gyro_y;     // Y轴角速度
    float gyro_z;     // Z轴角速度
    float battery_level;
} SensorData;

SensorData sensorData;

// 回调函数，当发送消息时会调用该函数
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    ESP_LOGI(TAG, "Last Packet Send Status: %s", status == ESP_NOW_SEND_SUCCESS ? "发送成功" : "发送失败");
}

void setup() {
    // 初始化I2C
    ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGI(TAG, "I2C初始化成功");
    vTaskDelay(100 / portTICK_PERIOD_MS);

    // 初始化PWM
    ESP_ERROR_CHECK(pwm_init());
    ESP_LOGI(TAG, "PWM初始化成功");
    vTaskDelay(100 / portTICK_PERIOD_MS);

    // 初始化MPU9250
    ESP_ERROR_CHECK(mpu9250_init());
    ESP_LOGI(TAG, "MPU9250初始化成功");
    vTaskDelay(100 / portTICK_PERIOD_MS);
    
    // 初始化其他外设
    soft_spi_init();
    ads1256_gpio_init();
    ads1256_init();
    initCW2015();
    wakeUp();

    ESP_LOGI(TAG, "系统初始化中...");

    // 初始化NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // 设置WiFi模式
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_11B|WIFI_PROTOCOL_11G|WIFI_PROTOCOL_11N|WIFI_PROTOCOL_LR));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE)); // 禁用省电模式

    // 初始化ESP-NOW
    if (esp_now_init() != ESP_OK) {
        ESP_LOGE(TAG, "ESP-NOW初始化失败");
        return;
    }
    
    // 注册发送回调
    esp_now_register_send_cb(OnDataSent);

    // 添加对等设备
    esp_now_peer_info_t peerInfo;
    memcpy(peerInfo.peer_addr, broadcastAddress, 6);
    peerInfo.channel = 0;  
    peerInfo.ifidx = ESP_IF_WIFI_STA;
    peerInfo.encrypt = false;

    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        ESP_LOGE(TAG, "添加对等设备失败");
        return;
    }

    ESP_LOGI(TAG, "系统初始化完成");
    vTaskDelay(1000 / portTICK_PERIOD_MS);
}

void loop() {
    int ulResult = 0;
    float tempVoltages[8];
    float acc_x, acc_y, acc_z;
    float gyro_x, gyro_y, gyro_z;
    
    // 采集ADS1256数据
    for (int i = 0; i < 8; i++) {
        switch(i) {
            case 0: write_reg(REG_MUX, MUXP_AIN0 | MUXN_AINCOM); break;
            case 1: write_reg(REG_MUX, MUXP_AIN1 | MUXN_AINCOM); break;
            case 2: write_reg(REG_MUX, MUXP_AIN2 | MUXN_AINCOM); break;
            case 3: write_reg(REG_MUX, MUXP_AIN3 | MUXN_AINCOM); break;
            case 4: write_reg(REG_MUX, MUXP_AIN4 | MUXN_AINCOM); break;
            case 5: write_reg(REG_MUX, MUXP_AIN5 | MUXN_AINCOM); break;
            case 6: write_reg(REG_MUX, MUXP_AIN6 | MUXN_AINCOM); break;
            case 7: write_reg(REG_MUX, MUXP_AIN7 | MUXN_AINCOM); break;
        }
        ulResult = ads1256_read_data();
        tempVoltages[i] = (ulResult * 0.59604644775390625) / 1000000.0;
    }

    // 重新排列电压数据
    sensorData.voltages[0] = tempVoltages[1];
    sensorData.voltages[1] = tempVoltages[0];
    sensorData.voltages[2] = tempVoltages[5];
    sensorData.voltages[3] = tempVoltages[4];
    sensorData.voltages[4] = tempVoltages[7];
    sensorData.voltages[5] = tempVoltages[2];
    sensorData.voltages[6] = tempVoltages[3];
    sensorData.voltages[7] = tempVoltages[6];

    // 获取MPU9250数据
    if (mpu9250_get_accel(&acc_x, &acc_y, &acc_z) == ESP_OK) {
        sensorData.acc_x = acc_x;
        sensorData.acc_y = acc_y;
        sensorData.acc_z = acc_z;
    }

    if (mpu9250_get_gyro(&gyro_x, &gyro_y, &gyro_z) == ESP_OK) {
        sensorData.gyro_x = gyro_x;
        sensorData.gyro_y = gyro_y;
        sensorData.gyro_z = gyro_z;
    }
    
    // 获取电池电量
    sensorData.battery_level = readBatQuantityHighPre();

    // 发送数据
    esp_now_send(broadcastAddress, (uint8_t *) &sensorData, sizeof(sensorData));
}

void app_main(void)
{
    setup();
    while (1) {
        loop();
    }
}