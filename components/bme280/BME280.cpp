/*
* BME280.cpp
*
*  Created on: Sep 10, 2017
*      Author: Alexander Sparkowsky
*/

#include "BME280.h"
#include <esp_log.h>
#include <math.h>
#include <cstdint>
#include <I2C.h>

static char LOG_TAG[] = "bme280";
static bool debug = false;

/**
* @brief Class constructor.
*
* The address is the address of the device on the %I2C bus. By default this address is set to 0x77.
*
* @param [in] address The %I2C address of the device on the %I2C bus.
*/
BME280::BME280(uint8_t address) {
    i2c.setAddress(address);
}

/**
* @brief Class instance destructor.
*/
BME280::~BME280() {
}

uint8_t BME280::readRegister8(uint8_t reg) {
    i2c.beginTransaction();
    i2c.write(reg, true);
    i2c.endTransaction();

	uint8_t value;
    i2c.beginTransaction();
    i2c.read(&value, false);
    i2c.endTransaction();

	return value;
}

uint16_t BME280::read16BitLittleEndianRegister(uint8_t register_address) {
    uint16_t temp = read16BitBigEndianRegister(register_address);

    return (temp >> 8) | (temp << 8);
}

int16_t BME280::read16BitSignedLittleEndianRegister(uint8_t register_address) {
    return (int16_t) read16BitLittleEndianRegister(register_address);
}

uint16_t BME280::read16BitBigEndianRegister(uint8_t reg) {
    i2c.beginTransaction();
    i2c.write(reg, true);
    i2c.endTransaction();

    uint8_t msb;
	uint8_t lsb;
    i2c.beginTransaction();
    i2c.read(&msb, true);
    i2c.read(&lsb, false);
    i2c.endTransaction();

    uint16_t ret = (uint16_t)((msb << 8) | lsb);
	return ret;
}

uint32_t BME280::readRegister24(uint8_t reg) {
    i2c.beginTransaction();
    i2c.write(reg, true);
    i2c.endTransaction();

    uint8_t msb;
	uint8_t lsb;
	uint8_t xlsb;
    i2c.beginTransaction();
    i2c.read(&msb, true);
    i2c.read(&lsb, true);
    i2c.read(&xlsb, false);
    i2c.endTransaction();

    uint32_t ret = (uint32_t)((msb << 16) | (lsb << 8) | xlsb);
	return ret;
}

esp_err_t BME280::writeRegister8(uint8_t register_address, uint8_t data) {
    i2c.beginTransaction();
    i2c.write(register_address, true);
    i2c.write(data, true);
    i2c.endTransaction();

    return ESP_OK;
}

bme280_adc_data BME280::burstReadMeasurement() {
    uint8_t buffer[8];

    uint8_t reg = BME280_REGISTER_PRESSUREDATA;

    i2c.beginTransaction();
    i2c.write(reg, true);
    i2c.endTransaction();

    i2c.beginTransaction();
    i2c.read(buffer, 7, true);
    i2c.read(&buffer[7], false);
    i2c.endTransaction();

    bme280_adc_data adc_data;

    adc_data.buffer.pressure.xmsb = 0;
    adc_data.buffer.pressure.msb = buffer[0];
    adc_data.buffer.pressure.lsb = buffer[1];
    adc_data.buffer.pressure.xlsb = buffer[2];

    adc_data.buffer.temperature.xmsb = 0;
    adc_data.buffer.temperature.msb = buffer[3];
    adc_data.buffer.temperature.lsb = buffer[4];
    adc_data.buffer.temperature.xlsb = buffer[5];

    adc_data.buffer.humidity.xmsb = 0;
    adc_data.buffer.humidity.msb = 0;
    adc_data.buffer.humidity.lsb = buffer[6];
    adc_data.buffer.humidity.xlsb = buffer[7];
    // adc_data.adc_data.adc_P = (BME280_S32_t)((buffer[0] << 16) | (buffer[1] << 8) | buffer[2]);
    // adc_data.adc_data.adc_T = (BME280_S32_t)((buffer[3] << 16) | (buffer[4] << 8) | buffer[5]);
    // adc_data.adc_data.adc_H = (BME280_S32_t)((buffer[6] << 8) | buffer[7]);
    return adc_data;
}

uint8_t BME280::readChipId() {
    if (debug) {
        ESP_LOGD(LOG_TAG, "readChipId()");
    }

    uint8_t chip_id = readRegister8(BME280_REGISTER_CHIPID);

    return chip_id;
}

// Returns temperature in DegC, resolution is 0.01 DegC. Output value of “5123” equals 51.23 DegC.
// t_fine carries fine temperature as global value
BME280_S32_t BME280::compensate_T(BME280_S32_t adc_T) {
    BME280_S32_t var1, var2, T;

    var1 = ((((adc_T>>3) - ((BME280_S32_t)_bme280_calib.dig_T1<<1))) * ((BME280_S32_t)_bme280_calib.dig_T2)) >> 11;
    var2 = (((((adc_T>>4) - ((BME280_S32_t)_bme280_calib.dig_T1)) * ((adc_T>>4) - ((BME280_S32_t)_bme280_calib.dig_T1))) >> 12) *
        ((BME280_S32_t)_bme280_calib.dig_T3)) >> 14;

    _t_fine = var1 + var2;

    T = (_t_fine * 5 + 128) >> 8;

    return T;
}

// Returns pressure in Pa as unsigned 32 bit integer in Q24.8 format (24 integer bits and 8 fractional bits).
// Output value of “24674867” represents 24674867/256 = 96386.2 Pa = 963.862 hPa
BME280_U32_t BME280::compensate_P(BME280_S32_t adc_P) {
    BME280_S64_t var1, var2, p;

    var1 = ((BME280_S64_t)_t_fine) - 128000;
    var2 = var1 * var1 * (BME280_S64_t)_bme280_calib.dig_P6;
    var2 = var2 + ((var1*(BME280_S64_t)_bme280_calib.dig_P5) << 17);
    var2 = var2 + (((BME280_S64_t)_bme280_calib.dig_P4) << 35);
    var1 = ((var1 * var1 * (BME280_S64_t)_bme280_calib.dig_P3) >> 8) + ((var1 * (BME280_S64_t)_bme280_calib.dig_P2) << 12);
    var1 = (((((BME280_S64_t)1) << 47) + var1)) * ((BME280_S64_t)_bme280_calib.dig_P1 ) >> 33;

    if (var1 == 0)
    {
        return 0; // avoid exception caused by division by zero
    }

    p = 1048576 - adc_P;
    p = (((p<<31) - var2) * 3125) / var1;
    var1 = (((BME280_S64_t)_bme280_calib.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((BME280_S64_t)_bme280_calib.dig_P8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((BME280_S64_t)_bme280_calib.dig_P7) << 4);

    return (BME280_U32_t)p;
}

// Returns humidity in %RH as unsigned 32 bit integer in Q22.10 format (22 integer and 10 fractional bits).
// Output value of “47445” represents 47445/1024 = 46.333 %RH
BME280_U32_t BME280::compensate_H(BME280_S32_t adc_H) {
    BME280_S32_t v_x1_u32r;

    v_x1_u32r = (_t_fine - ((BME280_S32_t)76800));
    v_x1_u32r = (((((adc_H << 14) - (((BME280_S32_t)_bme280_calib.dig_H4) << 20) - (((BME280_S32_t)_bme280_calib.dig_H5) * v_x1_u32r)) +
        ((BME280_S32_t)16384)) >> 15) * (((((((v_x1_u32r * ((BME280_S32_t)_bme280_calib.dig_H6)) >> 10) * (((v_x1_u32r *
        ((BME280_S32_t)_bme280_calib.dig_H3)) >> 11) + ((BME280_S32_t)32768))) >> 10) + ((BME280_S32_t)2097152)) *
        ((BME280_S32_t)_bme280_calib.dig_H2) + 8192) >> 14));
    v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) * ((BME280_S32_t)_bme280_calib.dig_H1)) >> 4));
    v_x1_u32r = (v_x1_u32r < 0 ? 0 : v_x1_u32r);
    v_x1_u32r = (v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r);

    return (BME280_U32_t)(v_x1_u32r >> 12);
}

float BME280::convertUncompensatedTemperature(int32_t adc_T) {
    adc_T >>= 4;
    BME280_S32_t T = compensate_T(adc_T);

    return (float) T / 100.0;
}

float BME280::convertUncompensatedPressure(BME280_S32_t adc_P) {
    adc_P >>= 4;
    BME280_U32_t p = compensate_P(adc_P);

    return (float) p / 256;
}

float BME280::convertUncompensatedHumidity(BME280_S32_t adc_H) {
    BME280_U32_t h = compensate_H(adc_H);

    return h / 1024.0;
}

/**************************************************************************/
/*!
Calculates the altitude (in meters) from the specified atmospheric
pressure (in hPa), and sea-level pressure (in hPa).
@param  pressure     preassure in Pa
@param  seaLevel      Sea-level pressure in hPa
@param  atmospheric   Atmospheric pressure in hPa
*/
/**************************************************************************/
float BME280::altitudeOfPressure(float pressure, float seaLevel) {
    // Equation taken from BMP180 datasheet (page 16):
    //  http://www.adafruit.com/datasheets/BST-BMP180-DS000-09.pdf

    // Note that using the equation from wikipedia can give bad results
    // at high altitude.  See this thread for more information:
    //  http://forums.adafruit.com/viewtopic.php?f=22&t=58064

    float atmospheric = pressure / 100.0F;
    return 44330.0 * (1.0 - pow((double) atmospheric / seaLevel, (double) 0.1903));
}

/**************************************************************************/
/*!
Calculates the pressure at sea level (in hPa) from the specified altitude
(in meters), and atmospheric pressure (in hPa).
@param  altitude      Altitude in meters
@param  atmospheric   Atmospheric pressure in hPa
*/
/**************************************************************************/
float BME280::seaLevelForAltitude(float altitude, float atmospheric) {
    // Equation taken from BMP180 datasheet (page 17):
    //  http://www.adafruit.com/datasheets/BST-BMP180-DS000-09.pdf

    // Note that using the equation from wikipedia can give bad results
    // at high altitude.  See this thread for more information:
    //  http://forums.adafruit.com/viewtopic.php?f=22&t=58064

    return atmospheric / pow(1.0 - (altitude / 44330.0), 5.255);
}

bme280_reading_data BME280::readSensorData() {
    if (debug) {
        ESP_LOGD(LOG_TAG, "readSensorData()");
    }
    bme280_reading_data reading_data;

    bme280_adc_data adc_data = burstReadMeasurement();

    reading_data.temperature = convertUncompensatedTemperature(adc_data.adc_data.adc_T);
    reading_data.pressure = convertUncompensatedPressure(adc_data.adc_data.adc_P);
    reading_data.humidity = convertUncompensatedHumidity(adc_data.adc_data.adc_H);

    return reading_data;
}

void BME280::readCoefficients(void) {
    if (debug) {
        ESP_LOGD(LOG_TAG, "readCoefficients()");
    }

    _bme280_calib.dig_T1 = (uint16_t)read16BitLittleEndianRegister(BME280_REGISTER_DIG_T1);
    _bme280_calib.dig_T2 = (int16_t)read16BitSignedLittleEndianRegister(BME280_REGISTER_DIG_T2);
    _bme280_calib.dig_T3 = (int16_t)read16BitSignedLittleEndianRegister(BME280_REGISTER_DIG_T3);
    if (debug) {
       ESP_LOGD(LOG_TAG, "T1: %u, T2: %d, T3: %d", _bme280_calib.dig_T1, _bme280_calib.dig_T2, _bme280_calib.dig_T3);
    }

    _bme280_calib.dig_P1 = (uint16_t)read16BitLittleEndianRegister(BME280_REGISTER_DIG_P1);
    _bme280_calib.dig_P2 = (int16_t)read16BitSignedLittleEndianRegister(BME280_REGISTER_DIG_P2);
    _bme280_calib.dig_P3 = (int16_t)read16BitSignedLittleEndianRegister(BME280_REGISTER_DIG_P3);
    _bme280_calib.dig_P4 = (int16_t)read16BitSignedLittleEndianRegister(BME280_REGISTER_DIG_P4);
    _bme280_calib.dig_P5 = (int16_t)read16BitSignedLittleEndianRegister(BME280_REGISTER_DIG_P5);
    _bme280_calib.dig_P6 = (int16_t)read16BitSignedLittleEndianRegister(BME280_REGISTER_DIG_P6);
    _bme280_calib.dig_P7 = (int16_t)read16BitSignedLittleEndianRegister(BME280_REGISTER_DIG_P7);
    _bme280_calib.dig_P8 = (int16_t)read16BitSignedLittleEndianRegister(BME280_REGISTER_DIG_P8);
    _bme280_calib.dig_P9 = (int16_t)read16BitSignedLittleEndianRegister(BME280_REGISTER_DIG_P9);
    if (debug) {
        ESP_LOGD(LOG_TAG, "P1: %u, P2: %d, P3: %d, P4: %d, P5: %d, P6: %d, P7: %d, P8: %d, P9: %d",
            _bme280_calib.dig_P1,
            _bme280_calib.dig_P2,
            _bme280_calib.dig_P3,
            _bme280_calib.dig_P4,
            _bme280_calib.dig_P5,
            _bme280_calib.dig_P6,
            _bme280_calib.dig_P7,
            _bme280_calib.dig_P8,
            _bme280_calib.dig_P9);
    }

    _bme280_calib.dig_H1 = readRegister8(BME280_REGISTER_DIG_H1);
    _bme280_calib.dig_H2 = read16BitSignedLittleEndianRegister(BME280_REGISTER_DIG_H2);
    _bme280_calib.dig_H3 = readRegister8(BME280_REGISTER_DIG_H3);
    _bme280_calib.dig_H4 = (readRegister8(BME280_REGISTER_DIG_H4) << 4) |
    (readRegister8(BME280_REGISTER_DIG_H4 + 1) & 0xF);
    _bme280_calib.dig_H5 = (readRegister8(BME280_REGISTER_DIG_H5 + 1) << 4) |
    (readRegister8(BME280_REGISTER_DIG_H5) >> 4);
    _bme280_calib.dig_H6 = (int8_t) readRegister8(BME280_REGISTER_DIG_H6);
}

/**
 * @brief enable or disable debugging.
 * @param [in] enabled Should debugging be enabled or disabled.
 * @return N/A.
 */
 void BME280::setDebug(bool enabled) {
	debug = enabled;
} // setDebug

/**
* @brief Initialize the BME280 device.
*
* @param [in] sdaPin The pin to use for the %I2C SDA functions.
* @param [in] clkPin The pin to use for the %I2C CLK functions.
*/
esp_err_t BME280::init(gpio_num_t sdaPin, gpio_num_t clkPin) {
    if (debug) {
        ESP_LOGD(LOG_TAG, "init()");
    }

    i2c.setDebug(false);
    i2c.init(BME280_ADDRESS, sdaPin, clkPin);

    _sensor_id = readChipId();
    if (_sensor_id != 0x60) {
        if (debug) {
            ESP_LOGD(LOG_TAG, "Could not read chip id during initialization");
        }

        return ESP_ERR_NOT_FOUND;
    }

    if (debug) {
        ESP_LOGD(LOG_TAG, "Sucessfully read chipId");
    }

    vTaskDelay(1000/portTICK_PERIOD_MS);

    readCoefficients();

    // Set before CONTROL_meas (DS 5.4.3)
    writeRegister8(BME280_REGISTER_CONTROLHUMID, 0x05); // 16x oversampling
    writeRegister8(BME280_REGISTER_CONTROL, 0xB7);      // 16x oversampling, normal mode

    return ESP_OK;
} // init
