#include <platform_st.h>

#include <driver/i2c_master.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <string.h>

int32_t platform_write(void *handle, uint8_t reg,
                              const uint8_t *bufp, uint16_t len)
{
    size_t size = len + 1;
    uint8_t *write_buf = malloc(size);
    if (write_buf == NULL)
    {
        return -1;
    }

    write_buf[0] = reg;
    memcpy(&write_buf[1], bufp, len);

    esp_err_t err = i2c_master_transmit(handle, write_buf, size, PLATFORM_ST_I2C_TIMEOUT_MS);

    free(write_buf);

    return (int32_t)err;
}

int32_t platform_read(void *handle, uint8_t reg,
                             uint8_t *bufp, uint16_t len)
{
    return (int32_t)i2c_master_transmit_receive(handle, &reg, 1, bufp, len, PLATFORM_ST_I2C_TIMEOUT_MS);
}

void platform_delay(uint32_t ms)
{
    vTaskDelay(pdMS_TO_TICKS(ms));
}

void tx_com(uint8_t *tx_buffer, uint16_t len)
{
    printf("%.*s", len, (char *)tx_buffer);
}

