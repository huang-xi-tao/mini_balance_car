#include "Parameter_Device.hpp"
#include "CRC16_IBM.h"

bool Param_Device::load(const uint8_t data[], const uint8_t head)
{
    if(data[0] != head )
        return false;
    
    memcpy(this->data, data + 2, size);

    return true;
}

void Param_Device::OTA_Fill(uint8_t output[]) 
{
    memcpy(&output[0], data, size);

}

void Param_Device::complete(uint8_t output[], const uint8_t head) const
{
    output[0] = head;
    output[1] = this->store_size();
    memcpy(&output[2], data, size);

}