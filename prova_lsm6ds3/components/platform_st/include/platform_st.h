#pragma once

#include <stdint.h>

#define PLATFORM_ST_I2C_TIMEOUT_MS 1000

int32_t platform_write(void *handle, uint8_t reg,
                              const uint8_t *bufp, uint16_t len);
int32_t platform_read(void *handle, uint8_t reg,
                             uint8_t *bufp, uint16_t len);
void platform_delay(uint32_t ms);
void tx_com(uint8_t *tx_buffer, uint16_t len);
