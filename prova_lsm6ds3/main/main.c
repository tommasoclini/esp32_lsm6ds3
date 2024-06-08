#include <stdio.h>
#include <lsm6ds3_reg.h>
#include <driver/i2c_master.h>

#include <esp_err.h>
#include <esp_log.h>
#include <esp_check.h>

#include <string.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <iot_button.h>

#include <platform_st.h>

static const char *TAG = "main";

#define I2C_MASTER_SCL_IO GPIO_NUM_7 /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO GPIO_NUM_6 /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_FREQ_HZ 100000    /*!< I2C master clock frequency */
#define I2C_MASTER_TIMEOUT_MS 5000

#define IMU_ADDR 0x6a

typedef struct {
    int16_t data_raw_acceleration[3];
    int16_t data_raw_angular_rate[3];
} imu_data_t;

static esp_err_t i2c_master_init(void);

static esp_err_t add_lsm6ds3(i2c_master_bus_handle_t bus_handle, i2c_master_dev_handle_t *imu_dev_handle);
static esp_err_t lsm6ds3_init(stmdev_ctx_t *imu_dev);

static i2c_master_bus_handle_t bus_handle;
static i2c_master_dev_handle_t imu_dev_handle;

static int16_t data_raw_acceleration[3];
static int16_t data_raw_angular_rate[3];
static int16_t data_raw_temperature;
static float acceleration_mg[3];
static float angular_rate_mdps[3];
static float temperature_degC;
static uint8_t tx_buffer[1000];

static int16_t data_raw_angular_rate_offsets[3];

static uint8_t whoamI, rst;
static uint8_t tx_buffer[1000];

void app_main(void)
{
    ESP_ERROR_CHECK(i2c_master_init());

    ESP_ERROR_CHECK(add_lsm6ds3(bus_handle, &imu_dev_handle));

    platform_delay(30);

    stmdev_ctx_t imu_dev = {
        .read_reg = platform_read,
        .write_reg = platform_write,
        .mdelay = platform_delay,
        .handle = imu_dev_handle};

    ESP_ERROR_CHECK(lsm6ds3_init(&imu_dev));

    /* Read samples in polling mode (no int) */
    while (1)
    {
        uint8_t reg;
        /* Read output only if new value is available */
        lsm6ds3_xl_flag_data_ready_get(&imu_dev, &reg);

        if (reg) {
          // Read acceleration field data
          memset(data_raw_acceleration, 0x00, 3 * sizeof(int16_t));
          lsm6ds3_acceleration_raw_get(&imu_dev, data_raw_acceleration);
          acceleration_mg[0] =
            lsm6ds3_from_fs2g_to_mg(data_raw_acceleration[0]);
          acceleration_mg[1] =
            lsm6ds3_from_fs2g_to_mg(data_raw_acceleration[1]);
          acceleration_mg[2] =
            lsm6ds3_from_fs2g_to_mg(data_raw_acceleration[2]);
          sprintf((char *)tx_buffer,
                  "Acceleration [mg]:%4.2f\t%4.2f\t%4.2f\r\n",
                  acceleration_mg[0], acceleration_mg[1], acceleration_mg[2]);
          tx_com(tx_buffer, strlen((char const *)tx_buffer));
        }

        lsm6ds3_gy_flag_data_ready_get(&imu_dev, &reg);

        if (reg)
        {
            /* Read angular rate field data */
            memset(data_raw_angular_rate, 0x00, 3 * sizeof(int16_t));
            lsm6ds3_angular_rate_raw_get(&imu_dev, data_raw_angular_rate);
            angular_rate_mdps[0] =
                lsm6ds3_from_fs2000dps_to_mdps(data_raw_angular_rate[0]);
            angular_rate_mdps[1] =
                lsm6ds3_from_fs2000dps_to_mdps(data_raw_angular_rate[1]);
            angular_rate_mdps[2] =
                lsm6ds3_from_fs2000dps_to_mdps(data_raw_angular_rate[2]);
            sprintf((char *)tx_buffer,
                    "Angular rate [mdps]:%4.2f\t%4.2f\t%4.2f\r\n",
                    angular_rate_mdps[0],
                    angular_rate_mdps[1],
                    angular_rate_mdps[2]);
            tx_com(tx_buffer, strlen((char const *)tx_buffer));
        }

        /*lsm6ds3_temp_flag_data_ready_get(&imu_dev, &reg);

        if (reg) {
          // Read temperature data
          memset(&data_raw_temperature, 0x00, sizeof(int16_t));
          lsm6ds3_temperature_raw_get(&imu_dev, &data_raw_temperature);
          temperature_degC =
            lsm6ds3_from_lsb_to_celsius(data_raw_temperature);
          sprintf((char *)tx_buffer,
                  "Temperature [degC]:%6.2f\r\n",
                  temperature_degC);
          tx_com(tx_buffer, strlen((char const *)tx_buffer));
        }*/

        platform_delay(5);
    }
}

/**
 * @brief i2c master initialization
 */
static esp_err_t i2c_master_init(void)
{
    i2c_master_bus_config_t i2c_mst_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = -1,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &bus_handle));

    return ESP_OK;
}

static esp_err_t add_lsm6ds3(i2c_master_bus_handle_t bus_handle, i2c_master_dev_handle_t *imu_dev_handle_p)
{
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = IMU_ADDR,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };

    ESP_RETURN_ON_ERROR(i2c_master_bus_add_device(bus_handle, &dev_cfg, imu_dev_handle_p), TAG, "Failed to add imu");

    return ESP_OK;
}

static esp_err_t lsm6ds3_init(stmdev_ctx_t *imu_dev)
{
    while (1)
    {
        lsm6ds3_device_id_get(imu_dev, &whoamI);

        if (whoamI != LSM6DS3_ID)
        {
            ESP_LOGE(TAG, "Did not find lsm6ds3(%u)", whoamI);
        }
        else
        {
            break;
        }
        platform_delay(250);
    }

    /* Restore default configuration */
    lsm6ds3_reset_set(imu_dev, PROPERTY_ENABLE);

    do
    {
        lsm6ds3_reset_get(imu_dev, &rst);
        platform_delay(50);
    } while (rst);

    lsm6ds3_int1_route_t int_1_reg;

    /*  Enable Block Data Update */
    lsm6ds3_block_data_update_set(imu_dev, PROPERTY_ENABLE);
    /* Set full scale */
    lsm6ds3_xl_full_scale_set(imu_dev, LSM6DS3_2g);
    lsm6ds3_gy_full_scale_set(imu_dev, LSM6DS3_2000dps);
    /* Set Output Data Rate for Acc and Gyro */
    lsm6ds3_xl_data_rate_set(imu_dev, LSM6DS3_XL_ODR_12Hz5);
    lsm6ds3_gy_data_rate_set(imu_dev, LSM6DS3_GY_ODR_12Hz5);

    return ESP_OK;
}
