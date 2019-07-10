#include <unistd.h>
#include "stm32f103xb.h"

#include "rgb.h"
#include "../i2c.h"

/* free rtos */
#include "FreeRTOS.h"
#include "task.h"
#include "octo.h"

/* addr */
#define TCS34725_ADDRESS (0x29)     /* < I2C address */
#define TCS34725_COMMAND_BIT (0x80) /* < Command bit */
#define TCS34725_ID (0x12)          /* < 0x44 = TCS34721/TCS34725, 0x4D = TCS34723/TCS34727 */
#define TCS34725_ENABLE (0x00)      /* Enable Register (0x00) */
#define TCS34725_RGBCTIME (0x01)    /* RGBC Timing Register  (0x01) */
#define TCS34725_WAITTIME (0X03)    /* Wait Time Register  (0x03) */
#define TCS34725_CNTRLREG (0X0F)    /* Control Register  (0x0F) */

/* readable registers voor determining colours */
#define TCS34725_CDATA (0x14)       /* < Clear channel data low byte */
#define TCS34725_RDATA (0x16)       /* < Red channel data low byte */
#define TCS34725_GDATA (0x18)       /* < Green channel data low byte */
#define TCS34725_BDATA (0x1A)       /* < Blue channel data low byte */

/* rgb pins for controlling led and reading */
/* GPIOx_CRL  x= A..G */
#define TCS34725_SENSOR1_8_CONFIG  (*((volatile uint32_t *) 0x40010800)) /*  define location addr of sensor 1-8  low input gpioA(0 - 7)*/
#define TCS34725_SENSOR1_8_VALUES  (*((volatile uint32_t *) 0x40010810)) /* define value addrs of sensor 1-8 */
#define TCS34725_SENSOR9_12_CONFIG (*((volatile uint32_t *) 0x40010c04)) /* define location addr of sensor 9-12  high input gpioB(12 - 15)*/
#define TCS34725_SENSOR9_12_VALUES (*((volatile uint32_t *) 0x40010c10)) /* define value addrs of sensor 9-12 */



/* Local variables */
unsigned long time = 0;

/* private functions */
static _Bool prvCheckDeviceID();
static _Bool prvInitPins();
static void  prvSetPin(uint8_t ucSensorPosition);
static void  prvRGBResetPin(uint8_t ucSensorPosition);
static void  prvRGBResetPins();

/* Enable register */
union EnableRegister
{
    struct
    {
        uint8_t powerOn    : 1;
        uint8_t ADCEnable  : 1;
        uint8_t : 1;
        uint8_t waitEnable : 1;
        uint8_t RGBCIntEnb : 1;
        uint8_t : 3;
    } Bits;
    uint8_t value;
};

/* RGBC timing register */
/* The RGBC timing register controls the internal integration time of
 * the RGBC clear and IR channel ADCs in2.4-ms increments.
 * Max RGBC Count = (256 − ATIME) × 1024 up to a maximum of 65535.
 */
union RGBCTimingRegister
{
    struct
    {
        uint8_t ATIME: 8;
    } Bits;
    uint8_t value;
};

/* Wait Time Register */
/* Wait time is set 2.4 ms increments unless the WLONG bit is asserted,
 * in which case the wait times are 12×longer.
 * WTIME is programmed as a 2’s complement number.
 */
union WaitTimeRegister
{
    struct
    {
        uint8_t WTIME: 8;
    } Bits;
    uint8_t value;
};

/* Control Register */
union ControlRegister
{
    struct
    {
        uint8_t AGAIN: 2;
        uint8_t : 6;
    } Bits;
    uint8_t value;
};

/*
 * initialize function for TCS34725.
 */
void RGBInit()
{
    /*initialize pins*/
    prvInitPins();
    int i = 0;
    for(i = 0; i < rgbSENSORCOUNT; i++) {
        prvSetPin(i);
        if(prvCheckDeviceID())
        {
        /* Enables the adc */
        /* Select enable register(0x00) */
        /* Power ON, RGBC enable, wait time disable(0x03) */
        union EnableRegister enRegister;
        enRegister.Bits.ADCEnable = 1;
        enRegister.Bits.powerOn = 1;
        i2c_begin_transmission(TCS34725_ADDRESS, (TCS34725_ENABLE | TCS34725_COMMAND_BIT));
        i2c_send_byte(enRegister.value);
        i2c_stop_transmission();
        while(time < 1000)
            time++;
        time = 0;

        /* Set intergration time */
        /* Select ALS time register(0x81) */
        /* Atime = 700 ms(0x00) */
        union RGBCTimingRegister timeReg;
        timeReg.value = 0x00;
        i2c_begin_transmission(TCS34725_ADDRESS, (TCS34725_RGBCTIME | TCS34725_COMMAND_BIT));
        i2c_send_byte(timeReg.value);
        i2c_stop_transmission();
        while(time < 1000)
            time++;
        time = 0;

        /* Set gain */
        /* Select Wait Time register(0x83) */
        /* WTIME : 2.4ms(0xFF) */
        union WaitTimeRegister waitReg;
        waitReg.value = 0xFF;
        i2c_begin_transmission(TCS34725_ADDRESS, (TCS34725_WAITTIME | TCS34725_COMMAND_BIT));
        i2c_send_byte(waitReg.value);
        i2c_stop_transmission();
        while(time < 1000)
            time++;
        time = 0;

        /* Select control register(0x8F) */
        /* AGAIN = 1x(0x00) */
        /* The gain register determines the sensitivity of the diodes */
        /* 00=1x, 01=4x, 10=16x, 11=60x Gain*/
        union ControlRegister cntrlReg;
        cntrlReg.value = 0x00;
        i2c_begin_transmission(TCS34725_ADDRESS, (TCS34725_CNTRLREG | TCS34725_COMMAND_BIT));
        i2c_send_byte(cntrlReg.value);
        i2c_stop_transmission();
        while(time < 1000)
            time++;
        time = 0;
        }
        else
        {
            TCS34725_SENSOR1_8_VALUES = 0x00ff0000;
        }
    }
}

/*
 * Returns the amount of sensor connected to the stm or cks.
 */
uint8_t getSensorCount()
{
    return rgbSENSORCOUNT;
}

/*
 * Returns the value of of each colour including clear (0-255) of the given rgb sensor.
 */
struct RGB xRGBgetRGB(uint8_t ucPosition)
{
    prvSetPin(ucPosition);
    while(time < 200000)
        time++;
    time = 0;
    volatile struct RGB tmp_RGB;
    tmp_RGB.usRed = ucRGBGetRed(ucPosition);
    while(time < 10)
        time++;
    time = 0;
    tmp_RGB.usGreen = ucRGBGetGreen(ucPosition);
    while(time < 10)
        time++;
    time = 0;
    tmp_RGB.usBlue = ucRGBGetBlue(ucPosition);
    while(time < 100000)
        time++;
    time = 0;
    prvRGBResetPin(ucPosition);
    return tmp_RGB;
}

/*
 * Returns the value of red(0-255) of the given rgb sensor.
 */
uint8_t ucRGBGetRed(uint8_t ucPosition)
{
    volatile uint16_t usTmpRed = 0, usTmpClear = 0;
    float fRed = 0;
    uint8_t ucRed = 0;

    /* read low Byte clear */
    i2c_begin_transmission(TCS34725_ADDRESS, TCS34725_CDATA | TCS34725_COMMAND_BIT);
    i2c_stop_transmission();
    usTmpClear = i2c_read_2_bytes(TCS34725_ADDRESS);

    /* read low Byte red */
    i2c_begin_transmission(TCS34725_ADDRESS, TCS34725_RDATA | TCS34725_COMMAND_BIT);
    i2c_stop_transmission();
    usTmpRed = i2c_read_2_bytes(TCS34725_ADDRESS);

    fRed = (float)((float)usTmpRed / (float)usTmpClear) * 255.0;
    ucRed = fRed;

    return ucRed;
}

/*
 * Returns the value of green(0-255) of the given rgb sensor.
 */
uint8_t ucRGBGetGreen(uint8_t ucPosition)
{
    volatile uint16_t usTmpGreen = 0, usTmpClear = 0;
    float fGreen = 0;
    uint8_t ucGreen = 0;

    /* read low Byte clear */
    i2c_begin_transmission(TCS34725_ADDRESS, TCS34725_CDATA | TCS34725_COMMAND_BIT);
    i2c_stop_transmission();
    usTmpClear = i2c_read_2_bytes(TCS34725_ADDRESS);

    /* read low Byte Green */
    i2c_begin_transmission(TCS34725_ADDRESS, TCS34725_GDATA | TCS34725_COMMAND_BIT);
    i2c_stop_transmission();
    usTmpGreen = i2c_read_2_bytes(TCS34725_ADDRESS);

    fGreen = (float)((float)usTmpGreen / (float)usTmpClear) * 255.0;
    ucGreen = fGreen;

    return ucGreen;
}

/*
 * Returns the value of blue(0-255) of the given rgb sensor.
 */
uint8_t ucRGBGetBlue(uint8_t ucPosition)
{
    volatile uint16_t usTmpBlue = 0, usTmpClear = 0;
    float fBlue = 0;
    uint8_t ucBlue = 0;

    /* read low Byte clear */
    i2c_begin_transmission(TCS34725_ADDRESS, TCS34725_CDATA | TCS34725_COMMAND_BIT);
    i2c_stop_transmission();
    usTmpClear = i2c_read_2_bytes(TCS34725_ADDRESS);

    /* read low Byte red */
    i2c_begin_transmission(TCS34725_ADDRESS, TCS34725_BDATA | TCS34725_COMMAND_BIT);
    i2c_stop_transmission();
    usTmpBlue = i2c_read_2_bytes(TCS34725_ADDRESS);

    fBlue = (float)((float)usTmpBlue / (float)usTmpClear) * 255.0;
    ucBlue = fBlue;

    return ucBlue;
}

/* Private functions */
extern QueueHandle_t i2c_to_isr;
extern QueueHandle_t i2c_from_isr;
/*
 *
 * Checks if ID is equal to 0x44 so the corresponding sensor is either TCS34721 or TCS34725.
 */
static _Bool prvCheckDeviceID()
{
    /* Read id and check if rgb sensor is a TCS34725 */
    //i2c_begin_transmission(TCS34725_ADDRESS, TCS34725_ID | TCS34725_COMMAND_BIT);
    //i2c_stop_transmission();

    struct i2c_message m;
    m.address = TCS34725_ADDRESS;
    m.byte = TCS34725_ID | TCS34725_COMMAND_BIT;
    m.write_finished = 0;
    m.read = 1;

    xQueueSend(i2c_to_isr, &m, portMAX_DELAY);
    _I2C_CR1 |= (1 << 8);
    xQueueReceive(i2c_from_isr, &m, portMAX_DELAY);

    volatile int i = 0;
    i++;
    volatile uint8_t ret = i2c_read_byte(TCS34725_ADDRESS);
    if (ret == 0x44) {
        return 1;

    }

    return 0;
}

/*
 * Function for initializing all gpio for the rgb sensors.
 */
static _Bool prvInitPins()
{
    /* set pins 0 and 1 of A as output */
    /* 0x1 == 0001 push pull 10mhz */
    /* Set A0 to A7 sensor 1 to 8 */
    TCS34725_SENSOR1_8_CONFIG = 0x11111111;
    /* Set B12 to B15*/
    TCS34725_SENSOR9_12_CONFIG = 0x11110000;
    return 1;
}

/*
 * Used for resetting all rgb sensor pins.
 */
static void prvRGBResetPins()
{
    TCS34725_SENSOR1_8_VALUES = 0x00FF0000;
    TCS34725_SENSOR9_12_VALUES = 0xF0000000;
}

/*
 * Used for resetting rgb sensor pin.
 */
static void prvRGBResetPin(uint8_t ucSensorPosition)
{
    if(ucSensorPosition == 0)
    {
        TCS34725_SENSOR1_8_VALUES = (0x00010000);
    }
    if(ucSensorPosition > 0 &&  ucSensorPosition < 8)
    {
        TCS34725_SENSOR1_8_VALUES = (0x00010000 << ucSensorPosition);
    }
    else
    {
        TCS34725_SENSOR9_12_VALUES = (0x10000000 << (ucSensorPosition - 8));
    }
}

/*
 * Used for setting rgb sensor pin.
 */
static void prvSetPin(uint8_t ucSensorPosition)
{
    /* Reset pins */
    prvRGBResetPins();
    /* Set pin */
    if(ucSensorPosition == 0)
    {
        TCS34725_SENSOR1_8_VALUES = (0x00000001);
    }
    if(ucSensorPosition > 0 &&  ucSensorPosition < 8)
    {
        TCS34725_SENSOR1_8_VALUES = (0x00000001 << ucSensorPosition);
    }
    else
    {
        TCS34725_SENSOR9_12_VALUES = (0x00001000 << (ucSensorPosition - 8));
    }
}
