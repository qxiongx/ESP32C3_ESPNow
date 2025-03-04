#include "cw2015.h"
#include "driver/i2c.h"
#include "esp_log.h"

static const char *TAG = "CW2015";
#define I2C_MASTER_SDA_IO GPIO_NUM_0 
#define I2C_MASTER_SCL_IO GPIO_NUM_1
#define I2C_MASTER_NUM I2C_NUM_0
#define CW_INT GPIO_NUM_3
#define I2C_MASTER_FREQ_HZ 400000

void IRAM_ATTR timer_isr(void *arg) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR((TaskHandle_t)arg, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
    }
}

void initCW2015(void) {
    // Initialize I2C
    int i2c_master_port = I2C_MASTER_NUM;
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ
    };
    i2c_param_config(i2c_master_port, &conf);
    i2c_driver_install(i2c_master_port, conf.mode, 0, 0, 0);
}

uint16_t readAnalogVoltage(void) {
    uint16_t AnalogVoltage = 0x0000;
    uint8_t data[2];
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (CW_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, CW_VCELL_H, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (CW_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, data, 2, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    AnalogVoltage = (data[0] << 8) | data[1];
    return (AnalogVoltage & 0x3fff);
}

float readBatVoltage(void) {
    float Voltage = 0.0;
    uint16_t AnalogVoltage;
    AnalogVoltage = readAnalogVoltage();
    Voltage = AnalogVoltage * 0.000305;

    return Voltage;
}

uint8_t readBatQuantityLowPre(void) {
    uint8_t soc_integer = readRegister(CW_SOC_integer);
    ESP_LOGI(TAG, "SOC Integer: %d%%", soc_integer);

    return soc_integer;
}

float readBatQuantityHighPre(void) {
    float BatQuantity = 0.0;
    uint8_t BatQuantityH = readRegister(CW_SOC_integer);
    uint8_t BatQuantityL = readRegister(CW_SOC_decimal);
    BatQuantity = BatQuantityH + BatQuantityL / 256.0;
    return BatQuantity;
}

void checkCW2015(void) {
    uint8_t check = readRegister(CW_VERSION);
    if (check != 0x70) {
        ESP_LOGE(TAG, "cw2015 error!");
    } else {
        ESP_LOGI(TAG, "cw2015 ok!");
    }
}

void setSleep(void) {
    writeRegister(0xc0, CW_MODE);
}

void wakeUp(void) {
    writeRegister(0x00, CW_MODE);
}

void quickStart(void) {
    writeRegister(0x30, CW_MODE);
}

void powerReset(void) {
    writeRegister(0x0f, CW_MODE);
}

uint8_t readRegister(uint8_t reg) {
    uint8_t rdat;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (CW_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (CW_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, &rdat, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return rdat;
}

void writeRegister(uint8_t val, uint8_t reg) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (CW_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write_byte(cmd, val, true);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
}


uint8_t readCW2015Version(void) {
    uint8_t version = 0;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (CW_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, CW_VERSION, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (CW_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, &version, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return version;
}

uint16_t readRemainingRunTime(void) {
    uint8_t data[2] = {0};
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (CW_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, CW_RRT_H, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (CW_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, data, 2, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    uint16_t remainingRunTime = (data[0] << 8) | data[1];
    return remainingRunTime;
}

void uart_event_task(void *pvParameters) {
    // Initialize CW2015
    initCW2015();
    vTaskDelay(100 / portTICK_PERIOD_MS);
    printf("CW2015 init finish!\n");
    wakeUp();
}

esp_err_t i2c_master_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    
    esp_err_t err = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (err != ESP_OK) {
        return err;
    }
    
    return i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

