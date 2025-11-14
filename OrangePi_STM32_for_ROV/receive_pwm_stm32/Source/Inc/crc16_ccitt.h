// crc16_ccitt.h
#ifndef CRC16_CCITT_H
#define CRC16_CCITT_H
#include <stdint.h>
static inline uint16_t crc16_init(void) { return 0xFFFF; }
uint16_t crc16_update(uint16_t crc, const uint8_t *data, uint16_t len);
static inline uint16_t crc16_ccitt(const uint8_t *data, uint16_t len)
{
    return crc16_update(0xFFFF, data, len);
}
#endif
