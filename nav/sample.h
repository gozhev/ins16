#ifndef _SAMPLE_H_
#define _SAMPLE_H_

struct sample_t {
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
    int64_t tstmp;
} __attribute__((__packed__));

#define MAKE16(H,L) ((int16_t)((H)<<8|(L)))

#endif
