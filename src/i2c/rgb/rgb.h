#ifndef _RGB_H
#define _RGB_H

#include <stdint.h>

/* Amount of sensors */
#define rgbSENSORCOUNT 1

/* struct for rgb */
struct RGB
{
    uint16_t usRed;
    uint16_t usGreen;
    uint16_t usBlue;
};

/* functions */
void RGBInit();
uint8_t getSensorCount();
struct RGB xRGBgetRGB(uint8_t ucPosition);
uint8_t ucRGBGetRed(uint8_t ucPosition);
uint8_t ucRGBGetGreen(uint8_t ucPosition);
uint8_t ucRGBGetBlue(uint8_t ucPosition);

#endif	/* _RGB_H */
