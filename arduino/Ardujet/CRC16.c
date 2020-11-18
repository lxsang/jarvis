#include <stdint.h>

uint16_t crc16_ccitt(const uint8_t *data, unsigned int data_len)
{
    uint16_t crc = 0xFFFF;

    if (data_len == 0)
        return 0;

    for (unsigned int i = 0; i < data_len; ++i)
    {
        uint16_t dbyte = data[i];
        crc ^= dbyte << 8;

        for (unsigned char j = 0; j < 8; ++j)
        {
            uint16_t mix = crc & 0x8000;
            crc = (crc << 1);
            if (mix)
                crc = crc ^ 0x1021;
        }
    }

    return crc;
}