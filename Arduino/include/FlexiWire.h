#pragma once

#include <Wire.h>

/*
 * Simple wrapper for the I2C Wire library to allow changing I2C pins on the fly
 * to allow for multiple I2C buses.
 */ 


class FlexiWire: public TwoWire
{   
    public:
        FlexiWire(uint8_t bus_num): TwoWire(bus_num) {}
        ~FlexiWire() {}

        bool setSdaPin(int sdaPin)
        {   
            if (sdaPin < 0) {
                return false;
            }

#if !CONFIG_DISABLE_HAL_LOCKS
            if(lock == NULL){
                lock = xSemaphoreCreateMutex();
                if(lock == NULL){
                    log_e("xSemaphoreCreateMutex failed");
                    return false;
                }
            }
            //acquire lock
            if(xSemaphoreTake(lock, portMAX_DELAY) != pdTRUE){
                log_e("could not acquire lock");
                return false;
            }
 #endif

            sda = sdaPin;

#if !CONFIG_DISABLE_HAL_LOCKS
    //release lock
    xSemaphoreGive(lock);
#endif

            return true;
        }
};
