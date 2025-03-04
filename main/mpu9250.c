#include "mpu9250.h"
#include "math.h"
#include "esp_log.h"

static const char *TAG = "MPU9250";

#define I2C_MASTER_SCL_IO    1    
#define I2C_MASTER_SDA_IO    0    
#define I2C_MASTER_NUM       I2C_NUM_0
#define I2C_MASTER_FREQ_HZ   400000

// 定义偏移量数组
static int16_t accel_offset[3] = {0};
static int16_t gyro_offset[3] = {0};
static float mag_sensitivity[3] = {0};

static esp_err_t write_byte(uint8_t addr, uint8_t reg, uint8_t data) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

esp_err_t mpu9250_init(void) {

    write_byte(MPU9250_ADDRESS, MPU9250_PWR_MGMT_1, 0x00);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    write_byte(MPU9250_ADDRESS, MPU9250_GYRO_CONFIG, 0x00);
    write_byte(MPU9250_ADDRESS, MPU9250_ACCEL_CONFIG, 0x00);
    write_byte(MPU9250_ADDRESS, MPU9250_CONFIG, 0x03);
    // 启用MPU9250的I2C旁路模式以访问AK8963
    write_byte(MPU9250_ADDRESS, 0x37, 0x02);
    vTaskDelay(10 / portTICK_PERIOD_MS);
    
    // 初始化AK8963
    write_byte(AK8963_ADDRESS, AK8963_CNTL1, 0x00); // 断电模式
    vTaskDelay(10 / portTICK_PERIOD_MS);
    write_byte(AK8963_ADDRESS, AK8963_CNTL1, 0x0F); // Fuse ROM访问模式
    vTaskDelay(10 / portTICK_PERIOD_MS);
    
    // 读取灵敏度调整值
    uint8_t asa[3];
    read_bytes(AK8963_ADDRESS, AK8963_ASAX, 3, asa);
    
    mag_sensitivity[0] = (float)(asa[0] - 128) / 256.0f + 1.0f;
    mag_sensitivity[1] = (float)(asa[1] - 128) / 256.0f + 1.0f;
    mag_sensitivity[2] = (float)(asa[2] - 128) / 256.0f + 1.0f;
    
    // 设置为连续测量模式2 (100Hz) + 16位输出
    write_byte(AK8963_ADDRESS, AK8963_CNTL1, 0x16);
    vTaskDelay(10 / portTICK_PERIOD_MS);
    
    return ESP_OK;
}

esp_err_t mpu9250_read_id(uint8_t *id) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU9250_ADDRESS << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, MPU9250_WHO_AM_I, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU9250_ADDRESS << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, id, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    
    return ret;
}

esp_err_t mpu9250_read_accel(float *ax, float *ay, float *az) {
    uint8_t data[6];
    
    // 读取加速度计数据
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU9250_ADDRESS << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, MPU9250_ACCEL_XOUT_H, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU9250_ADDRESS << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, data, 6, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    if (ret == ESP_OK) {
        // 转换为实际的g值 (±2g量程下，16384 LSB/g)
        *ax = (int16_t)((data[0] << 8) | data[1]) / 16384.0f;
        *ay = (int16_t)((data[2] << 8) | data[3]) / 16384.0f;
        *az = (int16_t)((data[4] << 8) | data[5]) / 16384.0f;
    }
    
    return ret;
}


esp_err_t mpu9250_read_raw_data(int16_t* ax, int16_t* ay, int16_t* az, 
                            int16_t* gx, int16_t* gy, int16_t* gz) {
    uint8_t buffer[14];

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU9250_ADDRESS << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, MPU9250_ACCEL_XOUT_H, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU9250_ADDRESS << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, buffer, 14, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    if (ret == ESP_OK) {
    *ax = ((int16_t)buffer[0] << 8) | buffer[1];
    *ay = ((int16_t)buffer[2] << 8) | buffer[3];
    *az = ((int16_t)buffer[4] << 8) | buffer[5];
    *gx = ((int16_t)buffer[8] << 8) | buffer[9];
    *gy = ((int16_t)buffer[10] << 8) | buffer[11];
    *gz = ((int16_t)buffer[12] << 8) | buffer[13];
    }

    return ret;
}

void mpu9250_get_angles(float *roll, float *pitch, float *yaw) {
    int16_t ax, ay, az;
    int16_t gx, gy, gz;

    // 添加参数检查
    if (!roll || !pitch || !yaw) {
    ESP_LOGE(TAG, "无效的参数指针");
    return;
    }

    // 读取原始数据并检查错误
    if (mpu9250_read_raw_data(&ax, &ay, &az, &gx, &gy, &gz) != ESP_OK) {
    ESP_LOGE(TAG, "读取传感器数据失败");
    return;
    }

    // 将原始数据转换为实际物理量
    float accel_x = ax / 16384.0f;  // ±2g量程下
    float accel_y = ay / 16384.0f;
    float accel_z = az / 16384.0f;

    // 计算欧拉角
    *roll = atan2(accel_y, accel_z) * 180.0f / M_PI;
    *pitch = atan2(-accel_x, sqrt(accel_y * accel_y + accel_z * accel_z)) * 180.0f / M_PI;

    // 使用陀螺仪数据计算yaw角
    static float yaw_last = 0;
    float gyro_z = gz / 131.0f;  // ±250°/s量程下
    *yaw = yaw_last + gyro_z * 0.01f;  // 积分计算，0.01是采样周期
    yaw_last = *yaw;
}

void mpu9250_calibrate(void) {
    int32_t ax_sum = 0, ay_sum = 0, az_sum = 0;
    int32_t gx_sum = 0, gy_sum = 0, gz_sum = 0;
    int16_t ax, ay, az, gx, gy, gz;
    
    ESP_LOGI(TAG, "开始校准，请保持传感器静止...");
    
    for(int i = 0; i < 1000; i++) {
        mpu9250_read_raw_data(&ax, &ay, &az, &gx, &gy, &gz);
        ax_sum += ax;
        ay_sum += ay;
        az_sum += az;
        gx_sum += gx;
        gy_sum += gy;
        gz_sum += gz;
        vTaskDelay(1);
    }
    
    accel_offset[0] = ax_sum / 1000;
    accel_offset[1] = ay_sum / 1000;
    accel_offset[2] = az_sum / 1000 - 16384;
    gyro_offset[0] = gx_sum / 1000;
    gyro_offset[1] = gy_sum / 1000;
    gyro_offset[2] = gz_sum / 1000;
    
    ESP_LOGI(TAG, "校准完成");
}

esp_err_t read_bytes(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *data) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_READ, true);
    if(len > 1) {
        i2c_master_read(cmd, data, len-1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, data + len-1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

// 添加磁力计读取函数
void mpu9250_read_mag(int16_t* mx, int16_t* my, int16_t* mz) {
    uint8_t st1;
    uint8_t data[7];
    
    // 读取状态寄存器1
    read_bytes(AK8963_ADDRESS, 0x02, 1, &st1);
    
    // 检查数据准备好
    if(st1 & 0x01) {
        read_bytes(AK8963_ADDRESS, 0x03, 7, data);
        
        // 检查磁力计溢出
        if(!(data[6] & 0x08)) {
            *mx = ((int16_t)data[1] << 8) | data[0];
            *my = ((int16_t)data[3] << 8) | data[2];
            *mz = ((int16_t)data[5] << 8) | data[4];
        }
    }
}

// 获取加速度数据
esp_err_t mpu9250_get_accel(float *ax, float *ay, float *az) {
    int16_t raw_ax, raw_ay, raw_az;
    int16_t raw_gx, raw_gy, raw_gz;  // 虽然不使用但需要传入read_raw_data函数
    
    esp_err_t ret = mpu9250_read_raw_data(&raw_ax, &raw_ay, &raw_az, 
                                         &raw_gx, &raw_gy, &raw_gz);
    
    if (ret == ESP_OK) {
        // 转换为实际的g值 (±2g量程下，16384 LSB/g)
        // 考虑校准偏移量
        *ax = (float)(raw_ax - accel_offset[0]) / 16384.0f;
        *ay = (float)(raw_ay - accel_offset[1]) / 16384.0f;
        *az = (float)(raw_az - accel_offset[2]) / 16384.0f;
    }
    
    return ret;
}

// 获取角速度数据
esp_err_t mpu9250_get_gyro(float *gx, float *gy, float *gz) {
    int16_t raw_ax, raw_ay, raw_az;  // 虽然不使用但需要传入read_raw_data函数
    int16_t raw_gx, raw_gy, raw_gz;
    
    esp_err_t ret = mpu9250_read_raw_data(&raw_ax, &raw_ay, &raw_az, 
                                         &raw_gx, &raw_gy, &raw_gz);
    
    if (ret == ESP_OK) {
        // 转换为实际的度/秒值 (±250°/s量程下，131 LSB/°/s)
        // 考虑校准偏移量
        *gx = (float)(raw_gx - gyro_offset[0]) / 131.0f;
        *gy = (float)(raw_gy - gyro_offset[1]) / 131.0f;
        *gz = (float)(raw_gz - gyro_offset[2]) / 131.0f;
    }
    
    return ret;
}