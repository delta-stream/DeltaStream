

#ifndef AVBUFFERDATA_H
#define AVBUFFERDATA_H
#include <cstdint>

struct BufferData
{
    const uint8_t* data;
    int size;
    int offset;
};
#endif //AVBUFFERDATA_H
