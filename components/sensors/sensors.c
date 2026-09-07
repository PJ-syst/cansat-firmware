#include "sensors.h"

#include <math.h>
#include <string.h>

#include "driver/i2c_master.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "sdkconfig.h"

#define BMP_REG_ID 0xD0U
#define BMP_REG_CALIB 0x88U
#define BMP_REG_CTRL_MEAS 0xF4U
#define BMP_REG_CONFIG 0xF5U
#define BMP_REG_DATA 0xF7U
#define MPU_REG_WHO_AM_I 0x75U
#define MPU_REG_PWR_MGMT_1 0x6BU
#define MPU_REG_CONFIG 0x1AU
#define MPU_REG_GYRO_CONFIG 0x1BU
#define MPU_REG_ACCEL_CONFIG 0x1CU
#define MPU_REG_ACCEL_DATA 0x3BU

typedef struct {
    uint16_t t1;
    int16_t t2, t3;
    uint16_t p1;
    int16_t p2, p3, p4, p5, p6, p7, p8, p9;
} bmp_calibration_t;

static const char *TAG = "sensors";
static i2c_master_bus_handle_t s_bus;
static i2c_master_dev_handle_t s_bmp;
static i2c_master_dev_handle_t s_mpu;
static bmp_calibration_t s_cal;
static bool s_bmp_ready;
static bool s_mpu_ready;

static void release_bus(void)
{
    s_bmp_ready = false;
    s_mpu_ready = false;
    if (s_bmp != NULL) {
        (void)i2c_master_bus_rm_device(s_bmp);
        s_bmp = NULL;
    }
    if (s_mpu != NULL) {
        (void)i2c_master_bus_rm_device(s_mpu);
        s_mpu = NULL;
    }
    if (s_bus != NULL) {
        (void)i2c_del_master_bus(s_bus);
        s_bus = NULL;
    }
}

static uint16_t u16le(const uint8_t *p)
{
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8U));
}

static int16_t i16le(const uint8_t *p)
{
    return (int16_t)u16le(p);
}

static int16_t i16be(const uint8_t *p)
{
    return (int16_t)(((uint16_t)p[0] << 8U) | (uint16_t)p[1]);
}

static esp_err_t reg_read(i2c_master_dev_handle_t device, uint8_t reg,
                          uint8_t *data, size_t length)
{
    return i2c_master_transmit_receive(device, &reg, 1U, data, length, 100);
}

static esp_err_t reg_write(i2c_master_dev_handle_t device, uint8_t reg, uint8_t value)
{
    const uint8_t data[2] = {reg, value};
    return i2c_master_transmit(device, data, sizeof(data), 100);
}

static esp_err_t init_bmp(void)
{
    i2c_device_config_t device_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = CONFIG_CANSAT_BAROMETER_ADDRESS,
        .scl_speed_hz = CONFIG_CANSAT_I2C_FREQUENCY_HZ,
    };
    ESP_RETURN_ON_ERROR(i2c_master_bus_add_device(s_bus, &device_config, &s_bmp),
                        TAG, "add barometer");
    uint8_t id = 0U;
    ESP_RETURN_ON_ERROR(reg_read(s_bmp, BMP_REG_ID, &id, 1U), TAG, "read barometer id");
    if (id != 0x58U && id != 0x60U) {
        ESP_LOGE(TAG, "unexpected barometer id 0x%02x", id);
        return ESP_ERR_NOT_FOUND;
    }
    uint8_t raw[24];
    ESP_RETURN_ON_ERROR(reg_read(s_bmp, BMP_REG_CALIB, raw, sizeof(raw)),
                        TAG, "read barometer calibration");
    s_cal.t1 = u16le(&raw[0]); s_cal.t2 = i16le(&raw[2]); s_cal.t3 = i16le(&raw[4]);
    s_cal.p1 = u16le(&raw[6]); s_cal.p2 = i16le(&raw[8]); s_cal.p3 = i16le(&raw[10]);
    s_cal.p4 = i16le(&raw[12]); s_cal.p5 = i16le(&raw[14]); s_cal.p6 = i16le(&raw[16]);
    s_cal.p7 = i16le(&raw[18]); s_cal.p8 = i16le(&raw[20]); s_cal.p9 = i16le(&raw[22]);
    if (s_cal.p1 == 0U) return ESP_ERR_INVALID_RESPONSE;
    ESP_RETURN_ON_ERROR(reg_write(s_bmp, BMP_REG_CONFIG, 0x10U), TAG, "configure barometer");
    ESP_RETURN_ON_ERROR(reg_write(s_bmp, BMP_REG_CTRL_MEAS, 0x27U), TAG, "start barometer");
    s_bmp_ready = true;
    return ESP_OK;
}

static esp_err_t init_mpu(void)
{
    i2c_device_config_t device_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = CONFIG_CANSAT_MPU6050_ADDRESS,
        .scl_speed_hz = CONFIG_CANSAT_I2C_FREQUENCY_HZ,
    };
    ESP_RETURN_ON_ERROR(i2c_master_bus_add_device(s_bus, &device_config, &s_mpu),
                        TAG, "add MPU6050");
    uint8_t id = 0U;
    ESP_RETURN_ON_ERROR(reg_read(s_mpu, MPU_REG_WHO_AM_I, &id, 1U), TAG, "read MPU6050 id");
    if ((id & 0x7EU) != 0x68U) return ESP_ERR_NOT_FOUND;
    ESP_RETURN_ON_ERROR(reg_write(s_mpu, MPU_REG_PWR_MGMT_1, 0x00U), TAG, "wake MPU6050");
    ESP_RETURN_ON_ERROR(reg_write(s_mpu, MPU_REG_CONFIG, 0x03U), TAG, "configure MPU6050 filter");
    ESP_RETURN_ON_ERROR(reg_write(s_mpu, MPU_REG_GYRO_CONFIG, 0x00U), TAG, "configure gyro");
    ESP_RETURN_ON_ERROR(reg_write(s_mpu, MPU_REG_ACCEL_CONFIG, 0x00U), TAG, "configure accel");
    s_mpu_ready = true;
    return ESP_OK;
}

esp_err_t sensors_init(sensors_health_t *health)
{
    if (health == NULL) return ESP_ERR_INVALID_ARG;
    if (s_bus != NULL) release_bus();
    memset(health, 0, sizeof(*health));
    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = CONFIG_CANSAT_I2C_SDA,
        .scl_io_num = CONFIG_CANSAT_I2C_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7U,
        .flags.enable_internal_pullup = true,
    };
    ESP_RETURN_ON_ERROR(i2c_new_master_bus(&bus_config, &s_bus), TAG, "create I2C bus");
    const esp_err_t bmp_result = init_bmp();
    health->barometer_available = bmp_result == ESP_OK;
    const esp_err_t mpu_result = init_mpu();
    health->imu_available = mpu_result == ESP_OK;
    if (mpu_result != ESP_OK) ESP_LOGW(TAG, "MPU6050 unavailable: %s", esp_err_to_name(mpu_result));
    return bmp_result;
}

esp_err_t sensors_recover(sensors_health_t *health)
{
    release_bus();
    return sensors_init(health);
}

static esp_err_t read_bmp(float *temperature_c, float *pressure_kpa)
{
    uint8_t raw[6];
    ESP_RETURN_ON_ERROR(reg_read(s_bmp, BMP_REG_DATA, raw, sizeof(raw)), TAG, "barometer read");
    const int32_t adc_p = (int32_t)(((uint32_t)raw[0] << 12U) |
                                   ((uint32_t)raw[1] << 4U) | ((uint32_t)raw[2] >> 4U));
    const int32_t adc_t = (int32_t)(((uint32_t)raw[3] << 12U) |
                                   ((uint32_t)raw[4] << 4U) | ((uint32_t)raw[5] >> 4U));
    if (adc_p == 0x80000 || adc_t == 0x80000) return ESP_ERR_INVALID_RESPONSE;

    const int32_t var1 = ((((adc_t >> 3) - ((int32_t)s_cal.t1 << 1))) *
                         (int32_t)s_cal.t2) >> 11;
    const int32_t var2 = (((((adc_t >> 4) - (int32_t)s_cal.t1) *
                            ((adc_t >> 4) - (int32_t)s_cal.t1)) >> 12) *
                          (int32_t)s_cal.t3) >> 14;
    const int32_t t_fine = var1 + var2;
    *temperature_c = (float)((t_fine * 5 + 128) >> 8) / 100.0f;

    int64_t p_var1 = (int64_t)t_fine - 128000;
    int64_t p_var2 = p_var1 * p_var1 * (int64_t)s_cal.p6;
    p_var2 += (p_var1 * (int64_t)s_cal.p5) << 17;
    p_var2 += (int64_t)s_cal.p4 << 35;
    p_var1 = ((p_var1 * p_var1 * (int64_t)s_cal.p3) >> 8) +
             ((p_var1 * (int64_t)s_cal.p2) << 12);
    p_var1 = (((((int64_t)1 << 47) + p_var1) * (int64_t)s_cal.p1) >> 33);
    if (p_var1 == 0) return ESP_ERR_INVALID_RESPONSE;
    int64_t pressure = 1048576 - adc_p;
    pressure = (((pressure << 31) - p_var2) * 3125) / p_var1;
    p_var1 = ((int64_t)s_cal.p9 * (pressure >> 13) * (pressure >> 13)) >> 25;
    p_var2 = ((int64_t)s_cal.p8 * pressure) >> 19;
    pressure = ((pressure + p_var1 + p_var2) >> 8) + ((int64_t)s_cal.p7 << 4);
    *pressure_kpa = (float)pressure / 256000.0f;
    return (*pressure_kpa >= 30.0f && *pressure_kpa <= 120.0f) ? ESP_OK : ESP_ERR_INVALID_RESPONSE;
}

float sensors_altitude_from_pressure_kpa(float pressure_kpa)
{
    if (!isfinite(pressure_kpa) || pressure_kpa <= 0.0f) return NAN;
    return 44330.0f * (1.0f - powf(pressure_kpa / 101.325f, 0.19029495f));
}

esp_err_t sensors_read(sensor_reading_t *reading)
{
    if (reading == NULL) return ESP_ERR_INVALID_ARG;
    memset(reading, 0, sizeof(*reading));
    reading->timestamp_ms = (uint32_t)(esp_timer_get_time() / 1000LL);
    esp_err_t critical_result = ESP_ERR_INVALID_STATE;
    if (s_bmp_ready) {
        critical_result = read_bmp(&reading->temperature_c, &reading->pressure_kpa);
        if (critical_result == ESP_OK) {
            reading->altitude_m = sensors_altitude_from_pressure_kpa(reading->pressure_kpa);
            reading->barometer_valid = isfinite(reading->altitude_m);
            reading->altitude_valid = reading->barometer_valid;
            reading->temperature_valid = reading->barometer_valid;
            reading->pressure_valid = reading->barometer_valid;
        }
    }
    if (s_mpu_ready) {
        uint8_t raw[14];
        if (reg_read(s_mpu, MPU_REG_ACCEL_DATA, raw, sizeof(raw)) == ESP_OK) {
            for (size_t i = 0U; i < 3U; ++i) {
                reading->accel_g[i] = (float)i16be(&raw[i * 2U]) / 16384.0f;
                reading->gyro_dps[i] = (float)i16be(&raw[8U + i * 2U]) / 131.0f;
            }
            reading->imu_valid = true;
        }
    }
    return critical_result;
}
