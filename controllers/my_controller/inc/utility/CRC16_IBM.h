#ifndef _CRC16_IBM_H_
#define _CRC16_IBM_H_

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

uint16_t crc16_ibm_calc(const uint8_t *buffer, size_t len);
void     crc16_ibm_update(uint8_t *const buffer, const size_t len);
uint8_t  crc16_ibm_verify(const uint8_t *const buffer, const size_t len);

#ifdef __cplusplus
}
#endif

#endif //_CRC16_IBM_H_
