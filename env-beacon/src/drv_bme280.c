#include "drv_bme280.h"

#include <math.h>

int32_t   t_fine = 0;

/*=========================================================================
CALIBRATION DATA
-----------------------------------------------------------------------*/
struct bme280_calib_data
{
		uint16_t dig_T1;
		int16_t  dig_T2;
		int16_t  dig_T3;

		uint16_t dig_P1;
		int16_t  dig_P2;
		int16_t  dig_P3;
		int16_t  dig_P4;
		int16_t  dig_P5;
		int16_t  dig_P6;
		int16_t  dig_P7;
		int16_t  dig_P8;
		int16_t  dig_P9;

		uint8_t  dig_H1;
		int16_t  dig_H2;
		uint8_t  dig_H3;
		int16_t  dig_H4;
		int16_t  dig_H5;
		int8_t   dig_H6;
} bme280_calib;

// The config register
struct config 
{
		// inactive duration (standby time) in normal mode
		// 000 = 0.5 ms
		// 001 = 62.5 ms
		// 010 = 125 ms
		// 011 = 250 ms
		// 100 = 500 ms
		// 101 = 1000 ms
		// 110 = 10 ms
		// 111 = 20 ms
		unsigned char t_sb : 3;

		// filter settings
		// 000 = filter off
		// 001 = 2x filter
		// 010 = 4x filter
		// 011 = 8x filter
		// 100 and above = 16x filter
		unsigned char filter : 3;

		// unused - don't set
		unsigned char none : 1;
		unsigned char spi3w_en : 1;
}_config_reg;

// The ctrl_meas register
struct ctrl_meas 
{
		// temperature oversampling
		// 000 = skipped
		// 001 = x1
		// 010 = x2
		// 011 = x4
		// 100 = x8
		// 101 and above = x16
		unsigned char osrs_t : 3;

		// pressure oversampling
		// 000 = skipped
		// 001 = x1
		// 010 = x2
		// 011 = x4
		// 100 = x8
		// 101 and above = x16
		unsigned char osrs_p : 3;

		// device mode
		// 00       = sleep
		// 01 or 10 = forced
		// 11       = normal
		unsigned char mode : 2;
}_meas_reg;


// The ctrl_hum register
struct ctrl_hum {
		// unused - don't set
		unsigned char none : 5;

		// pressure oversampling
		// 000 = skipped
		// 001 = x1
		// 010 = x2
		// 011 = x4
		// 100 = x8
		// 101 and above = x16
		unsigned char osrs_h : 3;
}_hum_reg;

unsigned char config_reg_get() {return (_config_reg.t_sb << 5) | (_config_reg.filter << 3) | _config_reg.spi3w_en;}
unsigned char meas_reg_get() {return (_meas_reg.osrs_t << 5) | (_meas_reg.osrs_p << 3) | _meas_reg.mode;}
unsigned char hum_reg_get() {return (_hum_reg.osrs_h);}

/* Driver properties. */
static struct
{
    drv_bme280_cfg_t const  *   p_drv_bme280_cfg;       ///< Pointer to the device configuration.
    drv_bme280_access_mode_t    current_access_mode;    ///< The currently used access mode.
    volatile bool               twi_sig_callback_called;///< Indicates whether the signal callback was called.
} m_drv_bme280;

/**************************************************************************/
/*!
    @brief  Writes an 8 bit value over I2C
*/
/**************************************************************************/
void write8(uint8_t reg_addr, uint8_t val)
{
	if ( m_drv_bme280.p_drv_bme280_cfg != NULL )
    {
        hal_twi_id_t twi_id  = m_drv_bme280.p_drv_bme280_cfg->twi_id;
        uint8_t tx_buffer[2] = {reg_addr, val};
    
        hal_twi_stop_mode_set(twi_id, HAL_TWI_STOP_MODE_STOP_ON_TX_BUF_END);
    
        m_drv_bme280.twi_sig_callback_called = false;
        if ( hal_twi_write(twi_id, 2, &(tx_buffer[0])) == HAL_TWI_STATUS_CODE_SUCCESS )
        {
            while ( (m_drv_bme280.current_access_mode == DRV_BME280_ACCESS_MODE_CPU_INACTIVE)
            &&      (!m_drv_bme280.twi_sig_callback_called) )
            {
                m_drv_bme280.p_drv_bme280_cfg->p_sleep_hook();
            }
        }
    }
}

/**************************************************************************/
/*!
    @brief  Reads an 8 bit value over I2C
*/
/**************************************************************************/
uint8_t read8(uint8_t reg_addr)
{
	uint8_t read_val = 0;
	
	if ( m_drv_bme280.p_drv_bme280_cfg != NULL )
	{
		hal_twi_id_t twi_id = m_drv_bme280.p_drv_bme280_cfg->twi_id;

		hal_twi_stop_mode_set(twi_id, HAL_TWI_STOP_MODE_STOP_ON_RX_BUF_END);

		m_drv_bme280.twi_sig_callback_called = false;
		if ( hal_twi_write(twi_id, 1, &reg_addr)== HAL_TWI_STATUS_CODE_SUCCESS )
		{
			while ( (m_drv_bme280.current_access_mode == DRV_BME280_ACCESS_MODE_CPU_INACTIVE)
			&&      (!m_drv_bme280.twi_sig_callback_called) )
			{
					m_drv_bme280.p_drv_bme280_cfg->p_sleep_hook();
			}

			m_drv_bme280.twi_sig_callback_called = false;
			if ( hal_twi_read(twi_id, 1, &read_val) == HAL_TWI_STATUS_CODE_SUCCESS )
			{
					while ( (m_drv_bme280.current_access_mode == DRV_BME280_ACCESS_MODE_CPU_INACTIVE)
					&&      (!m_drv_bme280.twi_sig_callback_called) )
					{
							m_drv_bme280.p_drv_bme280_cfg->p_sleep_hook();
					}
			}
		}
	}
	
	return read_val;
}

/**************************************************************************/
/*!
    @brief  Reads a 16 bit value over I2C
*/
/**************************************************************************/
uint16_t read16(uint8_t reg_addr)
{
	uint16_t read_val = 0;
	
	uint8_t temp_val[2] = "";

	if ( m_drv_bme280.p_drv_bme280_cfg != NULL )
	{
		hal_twi_id_t twi_id = m_drv_bme280.p_drv_bme280_cfg->twi_id;

		hal_twi_stop_mode_set(twi_id, HAL_TWI_STOP_MODE_STOP_ON_RX_BUF_END);

		m_drv_bme280.twi_sig_callback_called = false;
		if ( hal_twi_write(twi_id, 1, &reg_addr)== HAL_TWI_STATUS_CODE_SUCCESS )
		{
			while ( (m_drv_bme280.current_access_mode == DRV_BME280_ACCESS_MODE_CPU_INACTIVE)
			&&      (!m_drv_bme280.twi_sig_callback_called) )
			{
					m_drv_bme280.p_drv_bme280_cfg->p_sleep_hook();
			}

			m_drv_bme280.twi_sig_callback_called = false;
			if ( hal_twi_read(twi_id, 2, temp_val) == HAL_TWI_STATUS_CODE_SUCCESS )
			{
					while ( (m_drv_bme280.current_access_mode == DRV_BME280_ACCESS_MODE_CPU_INACTIVE)
					&&      (!m_drv_bme280.twi_sig_callback_called) )
					{
							m_drv_bme280.p_drv_bme280_cfg->p_sleep_hook();
					}

					read_val = temp_val[1];
					read_val |= ((uint16_t)temp_val[0] << 8);
			}
		}
	}

	return read_val;
}


/**************************************************************************/
/*!
    
*/
/**************************************************************************/
uint16_t read16_LE(uint8_t reg_addr) 
{
	uint16_t temp = read16(reg_addr);
	return (temp >> 8) | (temp << 8);
}


/**************************************************************************/
/*!
    @brief  Reads a signed 16 bit value over I2C or SPI
*/
/**************************************************************************/
int16_t readS16(uint8_t reg_addr)
{
	return (int16_t)read16(reg_addr);
}


/**************************************************************************/
/*!
   
*/
/**************************************************************************/
int16_t readS16_LE(uint8_t reg_addr)
{
	return (int16_t)read16_LE(reg_addr);
}


/**************************************************************************/
/*!
    @brief  Reads a 24 bit value over I2C
*/
/**************************************************************************/
uint32_t read24(uint8_t reg_addr)
{
	uint32_t read_val = 0;
	
	uint8_t temp_val[3] = "";

	if ( m_drv_bme280.p_drv_bme280_cfg != NULL )
	{
		hal_twi_id_t twi_id = m_drv_bme280.p_drv_bme280_cfg->twi_id;

		hal_twi_stop_mode_set(twi_id, HAL_TWI_STOP_MODE_STOP_ON_RX_BUF_END);

		m_drv_bme280.twi_sig_callback_called = false;
		if ( hal_twi_write(twi_id, 1, &reg_addr)== HAL_TWI_STATUS_CODE_SUCCESS )
		{
			while ( (m_drv_bme280.current_access_mode == DRV_BME280_ACCESS_MODE_CPU_INACTIVE)
			&&      (!m_drv_bme280.twi_sig_callback_called) )
			{
					m_drv_bme280.p_drv_bme280_cfg->p_sleep_hook();
			}

			m_drv_bme280.twi_sig_callback_called = false;
			if ( hal_twi_read(twi_id, 3, temp_val) == HAL_TWI_STATUS_CODE_SUCCESS )
			{
					while ( (m_drv_bme280.current_access_mode == DRV_BME280_ACCESS_MODE_CPU_INACTIVE)
					&&      (!m_drv_bme280.twi_sig_callback_called) )
					{
							m_drv_bme280.p_drv_bme280_cfg->p_sleep_hook();
					}

					read_val = temp_val[2];
					read_val |= ((uint16_t)temp_val[1] << 8);
					read_val |= ((uint32_t)temp_val[0] << 16);
			}
		}
	}

	return read_val;
}

void sensor_set_sampling(sensor_mode mode,
			 sensor_sampling tempSampling,
			 sensor_sampling pressSampling,
			 sensor_sampling humSampling,
			 sensor_filter filter,
			 standby_duration duration)
{
	_meas_reg.mode     = mode;
	_meas_reg.osrs_t   = tempSampling;
	_meas_reg.osrs_p   = pressSampling;
			
	
	_hum_reg.osrs_h    = humSampling;
	_config_reg.filter = filter;
	_config_reg.t_sb   = duration;

	
	// you must make sure to also set REGISTER_CONTROL after setting the
	// CONTROLHUMID register, otherwise the values won't be applied (see DS 5.4.3)
	write8(BME280_REGISTER_CONTROLHUMID, hum_reg_get());
	write8(BME280_REGISTER_CONFIG, config_reg_get());
	write8(BME280_REGISTER_CONTROL, meas_reg_get());
}

/**************************************************************************/
/*!
    @brief return true if chip is busy reading cal data
*/
/**************************************************************************/
bool is_reading_calibration(void)
{
  uint8_t const rStatus = read8(BME280_REGISTER_STATUS);

  return (rStatus & (1 << 0)) != 0;
}

/**************************************************************************/
/*!
    @brief  Reads the factory-set coefficients
*/
/**************************************************************************/
void read_coefficients(void)
{
    bme280_calib.dig_T1 = read16_LE(BME280_REGISTER_DIG_T1);
    bme280_calib.dig_T2 = readS16_LE(BME280_REGISTER_DIG_T2);
    bme280_calib.dig_T3 = readS16_LE(BME280_REGISTER_DIG_T3);

    bme280_calib.dig_P1 = read16_LE(BME280_REGISTER_DIG_P1);
    bme280_calib.dig_P2 = readS16_LE(BME280_REGISTER_DIG_P2);
    bme280_calib.dig_P3 = readS16_LE(BME280_REGISTER_DIG_P3);
    bme280_calib.dig_P4 = readS16_LE(BME280_REGISTER_DIG_P4);
    bme280_calib.dig_P5 = readS16_LE(BME280_REGISTER_DIG_P5);
    bme280_calib.dig_P6 = readS16_LE(BME280_REGISTER_DIG_P6);
    bme280_calib.dig_P7 = readS16_LE(BME280_REGISTER_DIG_P7);
    bme280_calib.dig_P8 = readS16_LE(BME280_REGISTER_DIG_P8);
    bme280_calib.dig_P9 = readS16_LE(BME280_REGISTER_DIG_P9);

    bme280_calib.dig_H1 = read8(BME280_REGISTER_DIG_H1);
    bme280_calib.dig_H2 = readS16_LE(BME280_REGISTER_DIG_H2);
    bme280_calib.dig_H3 = read8(BME280_REGISTER_DIG_H3);
    bme280_calib.dig_H4 = (read8(BME280_REGISTER_DIG_H4) << 4) | (read8(BME280_REGISTER_DIG_H4+1) & 0xF);
    bme280_calib.dig_H5 = (read8(BME280_REGISTER_DIG_H5+1) << 4) | (read8(BME280_REGISTER_DIG_H5) >> 4);
    bme280_calib.dig_H6 = (int8_t)read8(BME280_REGISTER_DIG_H6);
}

/* Handles the signals from the TWI driver. */
static void hal_twi_sig_callback(hal_twi_signal_type_t hal_twi_signal_type)
{
    (void)hal_twi_signal_type;
    
    m_drv_bme280.twi_sig_callback_called = true;
}


void drv_bme280_init(void)
{
    m_drv_bme280.p_drv_bme280_cfg = NULL;
}


uint32_t drv_bme280_open(drv_bme280_cfg_t const * const p_drv_bme280_cfg)
{
    if ( hal_twi_open(p_drv_bme280_cfg->twi_id, &(p_drv_bme280_cfg->twi_cfg)) == HAL_TWI_STATUS_CODE_SUCCESS )
    {
        m_drv_bme280.p_drv_bme280_cfg    = p_drv_bme280_cfg;
        m_drv_bme280.current_access_mode = DRV_BME280_ACCESS_MODE_CPU_ACTIVE;
        
        return ( DRV_BME280_STATUS_CODE_SUCCESS );
    }
    
    return ( DRV_BME280_STATUS_CODE_DISALLOWED );
}

uint32_t drv_bme280_access_mode_set(drv_bme280_access_mode_t access_mode)
{
    if ( m_drv_bme280.p_drv_bme280_cfg == NULL )
    {
        return ( DRV_BME280_STATUS_CODE_DISALLOWED );
    }
    
    if ( access_mode == DRV_BME280_ACCESS_MODE_CPU_ACTIVE )
    {
        m_drv_bme280.current_access_mode = DRV_BME280_ACCESS_MODE_CPU_ACTIVE;
        hal_twi_callback_set(m_drv_bme280.p_drv_bme280_cfg->twi_id, NULL);
    }
    else if ( (access_mode == DRV_BME280_ACCESS_MODE_CPU_INACTIVE)
    &&        (m_drv_bme280.p_drv_bme280_cfg->p_sleep_hook != NULL) )
    {
        m_drv_bme280.current_access_mode = DRV_BME280_ACCESS_MODE_CPU_INACTIVE;
        hal_twi_callback_set(m_drv_bme280.p_drv_bme280_cfg->twi_id, hal_twi_sig_callback);
    }
    else
    {
        return ( DRV_BME280_STATUS_CODE_INVALID_PARAM );
    }
    
    return ( DRV_BME280_STATUS_CODE_SUCCESS );
}

bool sensor_init(void)
{
	// check if sensor, i.e. the chip ID is correct
	if (read8(BME280_REGISTER_CHIPID) != 0x60)
			return false;
	
	return true;
}

void sensor_reset(void)
{
	// reset the device using soft-reset
	// this makes sure the IIR is off, etc.
	write8(BME280_REGISTER_SOFTRESET, 0xB6);
}



/**************************************************************************/
/*!
    @brief  Take a new measurement (only possible in forced mode)
*/
/**************************************************************************/
void take_forced_measurement(void)
{   
    // If we are in forced mode, the BME sensor goes back to sleep after each
    // measurement and we need to set it to forced mode once at this point, so
    // it will take the next measurement and then return to sleep again.
    // In normal mode simply does new measurements periodically.
	
		int i = 0;
	
    if (_meas_reg.mode == MODE_FORCED) 
		{
        // set to forced mode, i.e. "take next measurement"
        write8(BME280_REGISTER_CONTROL, meas_reg_get());
        // wait until measurement has been completed, otherwise we would read
        // the values from the last measurement
        while (read8(BME280_REGISTER_STATUS) & 0x08)
					for(i = 0; i < 100000; i ++);
    }
}


/**************************************************************************/
/*!
    @brief  Returns the temperature from the sensor
*/
/**************************************************************************/
float sensor_read_temperature(void)
{
    int32_t var1, var2;

    int32_t adc_T = read24(BME280_REGISTER_TEMPDATA);
    if (adc_T == 0x800000) // value in case temp measurement was disabled
        return NAN;
    adc_T >>= 4;

    var1 = ((((adc_T>>3) - ((int32_t)bme280_calib.dig_T1 <<1))) *
            ((int32_t)bme280_calib.dig_T2)) >> 11;
             
    var2 = (((((adc_T>>4) - ((int32_t)bme280_calib.dig_T1)) *
              ((adc_T>>4) - ((int32_t)bme280_calib.dig_T1))) >> 12) *
            ((int32_t)bme280_calib.dig_T3)) >> 14;

    t_fine = var1 + var2;

    float T = (t_fine * 5 + 128) >> 8;
    return T/100;
}


/**************************************************************************/
/*!
    @brief  Returns the temperature from the sensor
*/
/**************************************************************************/
float sensor_read_pressure(void) {
    int64_t var1, var2, p;

    sensor_read_temperature(); // must be done first to get t_fine

    int32_t adc_P = read24(BME280_REGISTER_PRESSUREDATA);
    if (adc_P == 0x800000) // value in case pressure measurement was disabled
        return NAN;
    adc_P >>= 4;

    var1 = ((int64_t)t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)bme280_calib.dig_P6;
    var2 = var2 + ((var1*(int64_t)bme280_calib.dig_P5)<<17);
    var2 = var2 + (((int64_t)bme280_calib.dig_P4)<<35);
    var1 = ((var1 * var1 * (int64_t)bme280_calib.dig_P3)>>8) +
           ((var1 * (int64_t)bme280_calib.dig_P2)<<12);
    var1 = (((((int64_t)1)<<47)+var1))*((int64_t)bme280_calib.dig_P1)>>33;

    if (var1 == 0) {
        return 0; // avoid exception caused by division by zero
    }
    p = 1048576 - adc_P;
    p = (((p<<31) - var2)*3125) / var1;
    var1 = (((int64_t)bme280_calib.dig_P9) * (p>>13) * (p>>13)) >> 25;
    var2 = (((int64_t)bme280_calib.dig_P8) * p) >> 19;

    p = ((p + var1 + var2) >> 8) + (((int64_t)bme280_calib.dig_P7)<<4);
    return (float)p/256;
}


/**************************************************************************/
/*!
    @brief  Returns the humidity from the sensor
*/
/**************************************************************************/
float sensor_read_humidity(void) {
    sensor_read_temperature(); // must be done first to get t_fine

    int32_t adc_H = read16(BME280_REGISTER_HUMIDDATA);
    if (adc_H == 0x8000) // value in case humidity measurement was disabled
        return NAN;
        
    int32_t v_x1_u32r;

    v_x1_u32r = (t_fine - ((int32_t)76800));

    v_x1_u32r = (((((adc_H << 14) - (((int32_t)bme280_calib.dig_H4) << 20) -
                    (((int32_t)bme280_calib.dig_H5) * v_x1_u32r)) + ((int32_t)16384)) >> 15) *
                 (((((((v_x1_u32r * ((int32_t)bme280_calib.dig_H6)) >> 10) *
                      (((v_x1_u32r * ((int32_t)bme280_calib.dig_H3)) >> 11) + ((int32_t)32768))) >> 10) +
                    ((int32_t)2097152)) * ((int32_t)bme280_calib.dig_H2) + 8192) >> 14));

    v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) *
                               ((int32_t)bme280_calib.dig_H1)) >> 4));

    v_x1_u32r = (v_x1_u32r < 0) ? 0 : v_x1_u32r;
    v_x1_u32r = (v_x1_u32r > 419430400) ? 419430400 : v_x1_u32r;
    float h = (v_x1_u32r>>12);
    return  h / 1024.0f;
}


/**************************************************************************/
/*!
    Calculates the altitude (in meters) from the specified atmospheric
    pressure (in hPa), and sea-level pressure (in hPa).

    @param  seaLevel      Sea-level pressure in hPa
    @param  atmospheric   Atmospheric pressure in hPa
*/
/**************************************************************************/
float readAltitude(float seaLevel)
{
    // Equation taken from BMP180 datasheet (page 16):
    //  http://www.adafruit.com/datasheets/BST-BMP180-DS000-09.pdf

    // Note that using the equation from wikipedia can give bad results
    // at high altitude. See this thread for more information:
    //  http://forums.adafruit.com/viewtopic.php?f=22&t=58064

    float atmospheric = sensor_read_pressure() / 100.0F;
    return 44330.0 * (1.0 - pow(atmospheric / seaLevel, 0.1903));
}


/**************************************************************************/
/*!
    Calculates the pressure at sea level (in hPa) from the specified altitude 
    (in meters), and atmospheric pressure (in hPa).  
    @param  altitude      Altitude in meters
    @param  atmospheric   Atmospheric pressure in hPa
*/
/**************************************************************************/
float sensor_sea_level_for_altitude(float altitude, float atmospheric)
{
    // Equation taken from BMP180 datasheet (page 17):
    //  http://www.adafruit.com/datasheets/BST-BMP180-DS000-09.pdf

    // Note that using the equation from wikipedia can give bad results
    // at high altitude. See this thread for more information:
    //  http://forums.adafruit.com/viewtopic.php?f=22&t=58064

    return atmospheric / pow(1.0f - (altitude/44330.0f), 5.255f);
}

uint32_t drv_bme280_close(void)
{
    if ( hal_twi_close(m_drv_bme280.p_drv_bme280_cfg->twi_id) == HAL_TWI_STATUS_CODE_SUCCESS )
    {
        m_drv_bme280.p_drv_bme280_cfg = NULL;
        
        return ( DRV_BME280_STATUS_CODE_SUCCESS );
    }
    
    return ( DRV_BME280_STATUS_CODE_DISALLOWED );
}
