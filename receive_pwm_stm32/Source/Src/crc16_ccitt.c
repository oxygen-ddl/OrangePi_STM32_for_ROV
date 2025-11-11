// crc16_ccitt.c
#include "crc16_ccitt.h"
uint16_t crc16_update(uint16_t crc, const uint8_t *data, uint16_t len)
{
    while (len--)
    {
        crc ^= (uint16_t)(*data++) << 8;
        for (int i = 0; i < 8; i++)
            crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : (crc << 1);
    }
    return crc;
}
