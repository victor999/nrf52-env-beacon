#ifndef SENSOR_DRV_H_
#define SENSOR_DRV_H_

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include "hal_twi.h"

/* Common addresses definition for temperature sensor. */
/*=========================================================================
I2C ADDRESS/BITS
-----------------------------------------------------------------------*/
#define BME280_ADDRESS                (0x76)
/*=========================================================================*/

/*=========================================================================
    REGISTERS
-----------------------------------------------------------------------*/
enum
{
		BME280_REGISTER_DIG_T1              = 0x88,
		BME280_REGISTER_DIG_T2              = 0x8A,
		BME280_REGISTER_DIG_T3              = 0x8C,

		BME280_REGISTER_DIG_P1              = 0x8E,
		BME280_REGISTER_DIG_P2              = 0x90,
		BME280_REGISTER_DIG_P3              = 0x92,
		BME280_REGISTER_DIG_P4              = 0x94,
		BME280_REGISTER_DIG_P5              = 0x96,
		BME280_REGISTER_DIG_P6              = 0x98,
		BME280_REGISTER_DIG_P7              = 0x9A,
		BME280_REGISTER_DIG_P8              = 0x9C,
		BME280_REGISTER_DIG_P9              = 0x9E,

		BME280_REGISTER_DIG_H1              = 0xA1,
		BME280_REGISTER_DIG_H2              = 0xE1,
		BME280_REGISTER_DIG_H3              = 0xE3,
		BME280_REGISTER_DIG_H4              = 0xE4,
		BME280_REGISTER_DIG_H5              = 0xE5,
		BME280_REGISTER_DIG_H6              = 0xE7,

		BME280_REGISTER_CHIPID             = 0xD0,
		BME280_REGISTER_VERSION            = 0xD1,
		BME280_REGISTER_SOFTRESET          = 0xE0,

		BME280_REGISTER_CAL26              = 0xE1,  // R calibration stored in 0xE1-0xF0

		BME280_REGISTER_CONTROLHUMID       = 0xF2,
		BME280_REGISTER_STATUS             = 0XF3,
		BME280_REGISTER_CONTROL            = 0xF4,
		BME280_REGISTER_CONFIG             = 0xF5,
		BME280_REGISTER_PRESSUREDATA       = 0xF7,
		BME280_REGISTER_TEMPDATA           = 0xFA,
		BME280_REGISTER_HUMIDDATA          = 0xFD
};

/*=========================================================================*/


typedef enum 
{
		SAMPLING_NONE = 0, //0b000,
		SAMPLING_X1   = 1, //0b001,
		SAMPLING_X2   = 2, //0b010,
		SAMPLING_X4   = 3, //0b011,
		SAMPLING_X8   = 4, //0b100,
		SAMPLING_X16  = 5 //0b101
}sensor_sampling;

typedef enum 
{
		MODE_SLEEP  = 0, //0b00,
		MODE_FORCED = 1, //0b01,
		MODE_NORMAL = 3 //0b11
}sensor_mode;

typedef enum 
{
		FILTER_OFF = 0, //0b000,
		FILTER_X2  = 1, //0b001,
		FILTER_X4  = 2, //0b010,
		FILTER_X8  = 3, //0b011,
		FILTER_X16 = 4 //0b100
}sensor_filter;

// standby durations in ms 
typedef enum 
{
		STANDBY_MS_0_5  = 0, //0b000,
		STANDBY_MS_10   = 6, //0b110,
		STANDBY_MS_20   = 7, //0b111,
		STANDBY_MS_62_5 = 1, //0b001,
		STANDBY_MS_125  = 2, //0b010,
		STANDBY_MS_250  = 3, //0b011,
		STANDBY_MS_500  = 4, //0b100,
		STANDBY_MS_1000 = 5, //0b101
}standby_duration;

/**@brief The bme280 status codes.
 */
enum
{
    DRV_BME280_STATUS_CODE_SUCCESS,         ///< Successfull.
    DRV_BME280_STATUS_CODE_DISALLOWED,      ///< Disallowed.
    DRV_BME280_STATUS_CODE_INVALID_PARAM,   ///< Invalid parameter.
};


/**@brief The access modes.
 */
typedef enum
{
    DRV_BME280_ACCESS_MODE_CPU_INACTIVE,    ///< The CPU is inactive while waiting for the access to complete.
    DRV_BME280_ACCESS_MODE_CPU_ACTIVE,      ///< The CPU is active while waiting for the access to complete.
} drv_bme280_access_mode_t;


/**@brief The type of the signal callback conveying signals from the driver.
 */
typedef void (*drv_bme280_sleep_hook_t)(void);


/**@brief The bme280 configuration.
 */
typedef struct
{
    hal_twi_id_t            twi_id;         ///< The ID of TWI master to be used for transactions.
    hal_twi_cfg_t           twi_cfg;        ///< The TWI configuration to use while the driver is opened.
    drv_bme280_sleep_hook_t p_sleep_hook;   ///< Pointer to a function for CPU power down to be used in the CPU inactive mode.
} drv_bme280_cfg_t;

/**@brief Initializes the bme280 interface.
 */
void drv_bme280_init(void);

bool sensor_init(void);
void sensor_reset(void);

/**@brief Opens access to the bme280 driver.
 *
 * @param{in] id    The id of the HW peripheral to open the driver for.
 * @param{in] cfg   The driver configuration.
 *
 * @retval ::DRV_BME280_STATUS_CODE_SUCCESS     if successful.
 * @retval ::DRV_BME280_STATUS_CODE_DISALLOWED  if the driver could not be opened.
 */
uint32_t drv_bme280_open(drv_bme280_cfg_t const * const p_drv_bme280_cfg);


/**@brief Sets the access mode.
 *
 * @nore Reading and writing data will be blocking calls if no callback is set.
 *
 * @param{in] access_mode   The mode to be used while accessing the bme280 chip.
 *
 * @return DRV_BME280_STATUS_CODE_SUCCESS          If the call was successful.
 * @return DRV_BME280_STATUS_CODE_DISALLOWED       If the call was not allowed at this time.
 * @return DRV_BME280_STATUS_CODE_INVALID_PARAM    If specified mode is not compatible with the configuration.
 */
uint32_t drv_bme280_access_mode_set(drv_bme280_access_mode_t access_mode);

/**@brief Opens access to the mcp9808 driver.
 *
 * @retval ::DRV_MCP9808_STATUS_CODE_SUCCESS     if successful.
 * @retval ::DRV_MCP9808_STATUS_CODE_DISALLOWED  if the driver could not be opened.
 */
uint32_t drv_bme280_close(void);

bool is_reading_calibration(void);

void read_coefficients(void);

void sensor_set_sampling(sensor_mode mode,
			 sensor_sampling tempSampling,
			 sensor_sampling pressSampling,
			 sensor_sampling humSampling,
			 sensor_filter filter,
			 standby_duration duration);

void take_forced_measurement(void);
float sensor_read_temperature(void);
float sensor_read_pressure(void);
float sensor_read_humidity(void);

float sensor_read_altitude(float seaLevel);
float sensor_sea_level_for_altitude(float altitude, float pressure);

#endif
