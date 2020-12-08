#ifndef CRC16_H
#define CRC16_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

uint16_t crc16_ccitt(const uint8_t *data, unsigned int data_len);

#ifdef __cplusplus
}
#endif

#endif