
#include "cobs.h"

size_t cobs_encode(const uint8_t *ptr, size_t length, uint8_t *dst)
{
    uint8_t *start = dst;
    uint8_t code = 1, *code_ptr = dst++; /* Where to insert the leading count */

    while (length--)
    {
        if (*ptr) /* Input byte not zero */
            *dst++ = *ptr, ++code;

        if (!*ptr++ || code == 0xFF) /* Input is zero or complete block */
            *code_ptr = code, code = 1, code_ptr = dst++;
    }
    *code_ptr = code; /* Final code */

    return dst - start;
}

size_t cobs_decode(uint8_t *ptr, size_t length)
{
    size_t offset = 0, next_offset = 0;
    while (offset < length && ptr[offset] != 0)
    {
        next_offset = ptr[offset];
        ptr[offset] = 0;
        offset = offset + next_offset;
    }
    return length - 2;
}