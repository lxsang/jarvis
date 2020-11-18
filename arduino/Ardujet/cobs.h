#ifndef COBS_H
#define COBS_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * cobs_encode stuffs "length" bytes of data
 * at the location pointed to by "ptr", writing
 * the output to the location pointed to by "dst".
 *
 * Returns the length of the encoded data.
 */
size_t cobs_encode(const uint8_t *ptr, size_t length, uint8_t *dst);


/*
 * cobs_decode decodes "length" bytes of data at
 * the location pointed to by "ptr", writing the
 * output to the location pointed to by "dst".
 *
 * Returns the length of the decoded data
 * (which is guaranteed to be <= length).
 */
size_t cobs_decode(const uint8_t *ptr, size_t length, uint8_t *dst);

#ifdef __cplusplus
}
#endif
#endif