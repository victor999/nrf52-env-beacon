#include "drv_mcp9808.h"
#include <stdlib.h>

/* Register addresses of the mcp9808 device. */

#define MCP9808_REG_CONFIG					(0x01U)

#define MCP9808_REG_UPPER_TEMP			(0x02U)
#define MCP9808_REG_LOWER_TEMP			(0x03U)
#define MCP9808_REG_CRIT_TEMP				(0x04U)
#define MCP9808_REG_AMBIENT_TEMP		(0x05U)
#define MCP9808_REG_MANUF_ID				(0x06U)
#define MCP9808_REG_DEVICE_ID				(0x07U)

#define MCP9808_REG_CONFIG_SHUTDOWN    	(0x0100U)
#define MCP9808_REG_CONFIG_NORMAL    		(0x0U)

/* Driver properties. */
static struct
{
    drv_mcp9808_cfg_t const  *   p_drv_mcp9808_cfg;       ///< Pointer to the device configuration.
    drv_mcp9808_access_mode_t    current_access_mode;    ///< The currently used access mode.
    volatile bool               twi_sig_callback_called;///< Indicates whether the signal callback was called.
} m_drv_mcp9808;


/* Gets the value of the specified register. */
static bool reg_get(uint8_t reg_addr, uint8_t *p_value)
{
    if ( m_drv_mcp9808.p_drv_mcp9808_cfg != NULL )
    {
        hal_twi_id_t twi_id = m_drv_mcp9808.p_drv_mcp9808_cfg->twi_id;

        hal_twi_stop_mode_set(twi_id, HAL_TWI_STOP_MODE_STOP_ON_RX_BUF_END);

        m_drv_mcp9808.twi_sig_callback_called = false;
        if ( hal_twi_write(twi_id, 1, &reg_addr)== HAL_TWI_STATUS_CODE_SUCCESS )
        {
            while ( (m_drv_mcp9808.current_access_mode == DRV_MCP9808_ACCESS_MODE_CPU_INACTIVE)
            &&      (!m_drv_mcp9808.twi_sig_callback_called) )
            {
                m_drv_mcp9808.p_drv_mcp9808_cfg->p_sleep_hook();
            }

            m_drv_mcp9808.twi_sig_callback_called = false;
            if ( hal_twi_read(twi_id, 1, p_value) == HAL_TWI_STATUS_CODE_SUCCESS )
            {
                while ( (m_drv_mcp9808.current_access_mode == DRV_MCP9808_ACCESS_MODE_CPU_INACTIVE)
                &&      (!m_drv_mcp9808.twi_sig_callback_called) )
                {
                    m_drv_mcp9808.p_drv_mcp9808_cfg->p_sleep_hook();
                }

                return ( true );
            }
        }
    }
    
    return ( false );
}

/* Gets the 16 bit value of the specified register. */
static bool reg_get_16(uint8_t reg_addr, uint16_t *p_value)
{
		uint16_t read_value = 0;
	
		uint8_t temp_val[2] = "";
	
    if ( m_drv_mcp9808.p_drv_mcp9808_cfg != NULL )
    {
        hal_twi_id_t twi_id = m_drv_mcp9808.p_drv_mcp9808_cfg->twi_id;

        hal_twi_stop_mode_set(twi_id, HAL_TWI_STOP_MODE_STOP_ON_RX_BUF_END);

        m_drv_mcp9808.twi_sig_callback_called = false;
        if ( hal_twi_write(twi_id, 1, &reg_addr)== HAL_TWI_STATUS_CODE_SUCCESS )
        {
            while ( (m_drv_mcp9808.current_access_mode == DRV_MCP9808_ACCESS_MODE_CPU_INACTIVE)
            &&      (!m_drv_mcp9808.twi_sig_callback_called) )
            {
                m_drv_mcp9808.p_drv_mcp9808_cfg->p_sleep_hook();
            }

            m_drv_mcp9808.twi_sig_callback_called = false;
            if ( hal_twi_read(twi_id, 2, temp_val) == HAL_TWI_STATUS_CODE_SUCCESS )
            {
                while ( (m_drv_mcp9808.current_access_mode == DRV_MCP9808_ACCESS_MODE_CPU_INACTIVE)
                &&      (!m_drv_mcp9808.twi_sig_callback_called) )
                {
                    m_drv_mcp9808.p_drv_mcp9808_cfg->p_sleep_hook();
                }
								
								read_value = temp_val[1];
								read_value |= ((uint16_t)temp_val[0] << 8);
								
								*p_value = read_value;

                return ( true );
            }
        }
    }
    
    return ( false );
}

/* Sets the specified register to the specified value. */
static bool reg_set(uint8_t reg_addr, uint8_t value)
{
    if ( m_drv_mcp9808.p_drv_mcp9808_cfg != NULL )
    {
        hal_twi_id_t twi_id  = m_drv_mcp9808.p_drv_mcp9808_cfg->twi_id;
        uint8_t tx_buffer[2] = {reg_addr, value};
    
        hal_twi_stop_mode_set(twi_id, HAL_TWI_STOP_MODE_STOP_ON_TX_BUF_END);
    
        m_drv_mcp9808.twi_sig_callback_called = false;
        if ( hal_twi_write(twi_id, 2, &(tx_buffer[0])) == HAL_TWI_STATUS_CODE_SUCCESS )
        {
            while ( (m_drv_mcp9808.current_access_mode == DRV_MCP9808_ACCESS_MODE_CPU_INACTIVE)
            &&      (!m_drv_mcp9808.twi_sig_callback_called) )
            {
                m_drv_mcp9808.p_drv_mcp9808_cfg->p_sleep_hook();
            }

            return ( true );
        }
    }
    
    return ( false );
}

/* Sets the specified register to the specified value. */
static bool reg_set_16(uint8_t reg_addr, uint16_t value)
{
		uint8_t value1 = value >> 8;
		uint8_t value2 = value & 0xFF;
    if ( m_drv_mcp9808.p_drv_mcp9808_cfg != NULL )
    {
        hal_twi_id_t twi_id  = m_drv_mcp9808.p_drv_mcp9808_cfg->twi_id;
        uint8_t tx_buffer[3] = {reg_addr, value1, value2};
    
        hal_twi_stop_mode_set(twi_id, HAL_TWI_STOP_MODE_STOP_ON_TX_BUF_END);
    
        m_drv_mcp9808.twi_sig_callback_called = false;
        if ( hal_twi_write(twi_id, 3, &(tx_buffer[0])) == HAL_TWI_STATUS_CODE_SUCCESS )
        {
            while ( (m_drv_mcp9808.current_access_mode == DRV_MCP9808_ACCESS_MODE_CPU_INACTIVE)
            &&      (!m_drv_mcp9808.twi_sig_callback_called) )
            {
                m_drv_mcp9808.p_drv_mcp9808_cfg->p_sleep_hook();
            }

            return ( true );
        }
    }
    
    return ( false );
}

/* Handles the signals from the TWI driver. */
static void hal_twi_sig_callback(hal_twi_signal_type_t hal_twi_signal_type)
{
    (void)hal_twi_signal_type;
    
    m_drv_mcp9808.twi_sig_callback_called = true;
}


void drv_mcp9808_init(void)
{
    m_drv_mcp9808.p_drv_mcp9808_cfg = NULL;
}


uint32_t drv_mcp9808_open(drv_mcp9808_cfg_t const * const p_drv_mcp9808_cfg)
{
    if ( hal_twi_open(p_drv_mcp9808_cfg->twi_id, &(p_drv_mcp9808_cfg->twi_cfg)) == HAL_TWI_STATUS_CODE_SUCCESS )
    {
        m_drv_mcp9808.p_drv_mcp9808_cfg    = p_drv_mcp9808_cfg;
        m_drv_mcp9808.current_access_mode = DRV_MCP9808_ACCESS_MODE_CPU_ACTIVE;
        
        return ( DRV_MCP9808_STATUS_CODE_SUCCESS );
    }
    
    return ( DRV_MCP9808_STATUS_CODE_DISALLOWED );
}


uint32_t drv_mcp9808_access_mode_set(drv_mcp9808_access_mode_t access_mode)
{
    if ( m_drv_mcp9808.p_drv_mcp9808_cfg == NULL )
    {
        return ( DRV_MCP9808_STATUS_CODE_DISALLOWED );
    }
    
    if ( access_mode == DRV_MCP9808_ACCESS_MODE_CPU_ACTIVE )
    {
        m_drv_mcp9808.current_access_mode = DRV_MCP9808_ACCESS_MODE_CPU_ACTIVE;
        hal_twi_callback_set(m_drv_mcp9808.p_drv_mcp9808_cfg->twi_id, NULL);
    }
    else if ( (access_mode == DRV_MCP9808_ACCESS_MODE_CPU_INACTIVE)
    &&        (m_drv_mcp9808.p_drv_mcp9808_cfg->p_sleep_hook != NULL) )
    {
        m_drv_mcp9808.current_access_mode = DRV_MCP9808_ACCESS_MODE_CPU_INACTIVE;
        hal_twi_callback_set(m_drv_mcp9808.p_drv_mcp9808_cfg->twi_id, hal_twi_sig_callback);
    }
    else
    {
        return ( DRV_MCP9808_STATUS_CODE_INVALID_PARAM );
    }
    
    return ( DRV_MCP9808_STATUS_CODE_SUCCESS );
}

uint32_t drv_mcp9808_config_reg_set(uint16_t reg_value)
{    
    if ( reg_set_16(MCP9808_REG_CONFIG, reg_value) )
    {
        return ( DRV_MCP9808_STATUS_CODE_SUCCESS );
    }
    
    return ( DRV_MCP9808_STATUS_CODE_DISALLOWED );
}

uint32_t drv_mcp9808_config_reg_get(uint16_t *p_reg_value)
{
		if ( reg_get_16(MCP9808_REG_CONFIG, p_reg_value) )
		{
				return ( DRV_MCP9808_STATUS_CODE_SUCCESS );
		}
		
		return ( DRV_MCP9808_STATUS_CODE_DISALLOWED );
}

uint32_t drv_mcp9808_temperature_get(int32_t * p_temperature_milli_deg)
{
    int16_t tmp_16 = 0;
    
    if ( reg_get_16(MCP9808_REG_AMBIENT_TEMP, (uint16_t *)(&tmp_16)) )
    {
				float temp = tmp_16 & 0x0FFF;
				temp /=  16.0f;
				if (tmp_16 & 0x1000) temp -= 256;
			
        *p_temperature_milli_deg = temp * 1000;
        
        return ( DRV_MCP9808_STATUS_CODE_SUCCESS );
    }
    
    return ( DRV_MCP9808_STATUS_CODE_DISALLOWED );
}


uint32_t drv_mcp9808_close(void)
{
    if ( hal_twi_close(m_drv_mcp9808.p_drv_mcp9808_cfg->twi_id) == HAL_TWI_STATUS_CODE_SUCCESS )
    {
        m_drv_mcp9808.p_drv_mcp9808_cfg = NULL;
        
        return ( DRV_MCP9808_STATUS_CODE_SUCCESS );
    }
    
    return ( DRV_MCP9808_STATUS_CODE_DISALLOWED );
}

uint32_t set_normal_mode(void)
{
	return drv_mcp9808_config_reg_set(MCP9808_REG_CONFIG_NORMAL);
}

uint32_t set_shutdown_mode(void)
{
	return drv_mcp9808_config_reg_set(MCP9808_REG_CONFIG_SHUTDOWN);
}
