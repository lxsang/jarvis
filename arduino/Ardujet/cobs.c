
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

size_t cobs_decode(const uint8_t *ptr, size_t length, uint8_t *dst)
{
    const uint8_t *start = dst, *end = ptr + length;
    uint8_t code = 0xFF, copy = 0;

    for (; ptr < end; copy--)
    {
        if (copy != 0)
        {
            *dst++ = *ptr++;
        }
        else
        {
            if (code != 0xFF)
                *dst++ = 0;
            copy = code = *ptr++;
            if (code == 0)
                break; /* Source length too long */
        }
    }
    return dst - start;
}