#ifndef _SBUS_H_
#define _SBUS_H_

#ifdef __cplusplus
extern "C" {
#endif

typedef struct{
    uint8_t  start;
    uint32_t ch1  : 11;
    uint32_t ch2  : 11;
    uint64_t ch3  : 11;
    uint32_t ch4  : 11;
    uint32_t ch5  : 11;
    uint32_t ch6  : 11;
    uint32_t ch7  : 11;
    uint32_t ch8  : 11;
    uint32_t ch9  : 11;
    uint32_t ch10 : 11;
    uint32_t ch11 : 11;
    uint32_t ch12 : 11;
    uint32_t ch13 : 11;
    uint32_t ch14 : 11;
    uint32_t ch15 : 11;
    uint32_t ch16 : 11;
    uint8_t  ch17 : 1;
    uint8_t  ch18 : 1;
    uint8_t  frame_lost : 1;
    uint8_t  failsafe   : 1;
    uint8_t  shit       : 4; //useless
    uint8_t  end;
} __attribute__((packed)) SBUS_t;

#ifdef __cplusplus
}
#endif

#endif