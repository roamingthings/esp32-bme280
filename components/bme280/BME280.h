/*
 * BME280.h
 *
 *  Created on: Sep 10, 2017
 *      Author: Alexander Sparkowsky
 */
#ifndef COMPONENTS_BME280_BME280_H_
#define COMPONENTS_BME280_BME280_H_
#include <I2C.h>

/*=========================================================================
I2C ADDRESS/BITS
-----------------------------------------------------------------------*/
#define BME280_ADDRESS (0x77)
/*=========================================================================*/

/*=========================================================================
    REGISTERS
    -----------------------------------------------------------------------*/
enum
{
    BME280_REGISTER_DIG_T1 = 0x88,
    BME280_REGISTER_DIG_T2 = 0x8A,
    BME280_REGISTER_DIG_T3 = 0x8C,

    BME280_REGISTER_DIG_P1 = 0x8E,
    BME280_REGISTER_DIG_P2 = 0x90,
    BME280_REGISTER_DIG_P3 = 0x92,
    BME280_REGISTER_DIG_P4 = 0x94,
    BME280_REGISTER_DIG_P5 = 0x96,
    BME280_REGISTER_DIG_P6 = 0x98,
    BME280_REGISTER_DIG_P7 = 0x9A,
    BME280_REGISTER_DIG_P8 = 0x9C,
    BME280_REGISTER_DIG_P9 = 0x9E,

    BME280_REGISTER_DIG_H1 = 0xA1,
    BME280_REGISTER_DIG_H2 = 0xE1,
    BME280_REGISTER_DIG_H3 = 0xE3,
    BME280_REGISTER_DIG_H4 = 0xE4,
    BME280_REGISTER_DIG_H5 = 0xE5,
    BME280_REGISTER_DIG_H6 = 0xE7,

    BME280_REGISTER_CHIPID = 0xD0,
    BME280_REGISTER_VERSION = 0xD1,
    BME280_REGISTER_SOFTRESET = 0xE0,

    BME280_REGISTER_CAL26 = 0xE1, // R calibration stored in 0xE1-0xF0

    BME280_REGISTER_CONTROLHUMID = 0xF2,
    BME280_REGISTER_STATUS = 0xF3,
    BME280_REGISTER_CONTROL = 0xF4,
    BME280_REGISTER_CONFIG = 0xF5,
    BME280_REGISTER_PRESSUREDATA = 0xF7,
    BME280_REGISTER_TEMPDATA = 0xFA,
    BME280_REGISTER_HUMIDDATA = 0xFD,
};

/*=========================================================================*/

/*=========================================================================
    CALIBRATION DATA
    -----------------------------------------------------------------------*/
typedef struct
{
    uint16_t dig_T1;
    int16_t dig_T2;
    int16_t dig_T3;

    uint16_t dig_P1;
    int16_t dig_P2;
    int16_t dig_P3;
    int16_t dig_P4;
    int16_t dig_P5;
    int16_t dig_P6;
    int16_t dig_P7;
    int16_t dig_P8;
    int16_t dig_P9;

    uint8_t dig_H1;
    int16_t dig_H2;
    uint8_t dig_H3;
    int16_t dig_H4;
    int16_t dig_H5;
    int8_t dig_H6;
} bme280_calib_data;
/*=========================================================================*/

typedef uint32_t BME280_U32_t;
typedef int32_t BME280_S32_t;
typedef int64_t BME280_S64_t;

typedef struct
{
    float temperature;
    float humidity;
    float pressure;
} bme280_reading_data;

union bme280_adc_data
{
    struct {
        BME280_S32_t adc_P;
        BME280_S32_t adc_T;
        BME280_S32_t adc_H;
    } adc_data;
    struct {
        struct {
            uint8_t xlsb;
            uint8_t lsb;
            uint8_t msb;
            uint8_t xmsb;
        } pressure;
        struct {
            uint8_t xlsb;
            uint8_t lsb;
            uint8_t msb;
            uint8_t xmsb;
        } temperature;
        struct {
            uint8_t xlsb;
            uint8_t lsb;
            uint8_t msb;
            uint8_t xmsb;
        } humidity;
    } buffer;
};

/**
 * @brief Encapsulate a %BME280 device.
 *
 * The %BME280 is a sensor to measure temperature, humidity and preassure.
 *
 * @see [BM280 home page](https://www.bosch-sensortec.com/bst/products/all_products/bme280)
 */
class BME280 {
public:
	BME280(uint8_t address=BME280_ADDRESS);
	virtual ~BME280();
	void setDebug(bool enabled);
	esp_err_t init(gpio_num_t sdaPin=I2C::DEFAULT_SDA_PIN, gpio_num_t clkPin=I2C::DEFAULT_CLK_PIN);

    float altitudeOfPressure(float pressure, float seaLevel);
    float seaLevelForAltitude(float altitude, float atmospheric);
    bme280_reading_data readSensorData();

    /**
	 * Read the device id of the module.
	 */
    uint8_t readChipId(void);

private:
	I2C i2c = I2C();

    void readCoefficients(void);
    uint8_t readRegister8(uint8_t reg);
    uint16_t read16BitBigEndianRegister(uint8_t reg);
    uint16_t read16BitLittleEndianRegister(uint8_t register_address);
    int16_t read16BitSignedLittleEndianRegister(uint8_t register_address);
    uint32_t readRegister24(uint8_t reg);
    esp_err_t writeRegister8(uint8_t register_address, uint8_t data);
    bme280_adc_data burstReadMeasurement();

    BME280_S32_t compensate_T(BME280_S32_t adc_T);
    BME280_U32_t compensate_P(BME280_S32_t adc_P);
    BME280_U32_t compensate_H(BME280_S32_t adc_H);

    float convertUncompensatedTemperature(int32_t adc_T);
    float convertUncompensatedPressure(BME280_S32_t adc_P);
    float convertUncompensatedHumidity(BME280_S32_t adc_H);

    int32_t _sensor_id;
    int32_t _t_fine;

    bme280_calib_data _bme280_calib;
};

 #endif /* COMPONENTS_BME280_BME280_H_ */
