/* Copyright (c) Nordic Semiconductor ASA
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 
 *   1. Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 * 
 *   2. Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 * 
 *   3. Neither the name of Nordic Semiconductor ASA nor the names of other
 *   contributors to this software may be used to endorse or promote products
 *   derived from this software without specific prior written permission.
 * 
 *   4. This software must only be used in a processor manufactured by Nordic
 *   Semiconductor ASA, or in a processor manufactured by a third party that
 *   is used in combination with a processor manufactured by Nordic Semiconductor.
 * 
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#include "hal_radio.h"
#include "hal_timer.h"
#include "hal_clock.h"
#include "drv_bme280.h"
#include "hal_serial.h"
#include "hal_twi.h"

#include "nrf.h"

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>

#include "nrf_drv_saadc.h"

#include "nrf_error.h"

#define APP_IRQ_PRIORITY_LOW 6


/* #define DBG_RADIO_ACTIVE_ENABLE      */

/* #define DBG_BEACON_START_PIN    (16) */
/* #define DBG_HFCLK_ENABLED_PIN   (27) */
/* #define DBG_HFCLK_DISABLED_PIN  (26) */
/* #define DBG_PKT_SENT_PIN        ( 9) */
/* #define DBG_CALCULATE_BEGIN_PIN (15) */
/* #define DBG_CALCULATE_END_PIN   (15) */
/* #define DBG_RADIO_ACTIVE_PIN    (24) */
/* #define DBG_WFE_BEGIN_PIN       (25) */
/* #define DBG_WFE_END_PIN         (25) */


/* #define DBG_BEACON_START                           \
{                                                     \
    NRF_GPIO->OUTSET = (1 << DBG_BEACON_START_PIN);   \
    NRF_GPIO->DIRSET = (1 << DBG_BEACON_START_PIN);   \
    NRF_GPIO->DIRCLR = (1 << DBG_BEACON_START_PIN);   \
    NRF_GPIO->OUTCLR = (1 << DBG_BEACON_START_PIN);   \
} */
/* #define DBG_HFCLK_ENABLED                          \
{                                                     \
    NRF_GPIO->OUTSET = (1 << DBG_HFCLK_ENABLED_PIN);  \
    NRF_GPIO->DIRSET = (1 << DBG_HFCLK_ENABLED_PIN);  \
    NRF_GPIO->DIRCLR = (1 << DBG_HFCLK_ENABLED_PIN);  \
    NRF_GPIO->OUTCLR = (1 << DBG_HFCLK_ENABLED_PIN);  \
} */
/* #define DBG_HFCLK_DISABLED                         \
{                                                     \
    NRF_GPIO->OUTSET = (1 << DBG_HFCLK_DISABLED_PIN); \
    NRF_GPIO->DIRSET = (1 << DBG_HFCLK_DISABLED_PIN); \
    NRF_GPIO->DIRCLR = (1 << DBG_HFCLK_DISABLED_PIN); \
    NRF_GPIO->OUTCLR = (1 << DBG_HFCLK_DISABLED_PIN); \
} */
/* #define DBG_PKT_SENT                               \
{                                                     \
    NRF_GPIO->OUTSET = (1 << DBG_PKT_SENT_PIN);       \
    NRF_GPIO->DIRSET = (1 << DBG_PKT_SENT_PIN);       \
    NRF_GPIO->DIRCLR = (1 << DBG_PKT_SENT_PIN);       \
    NRF_GPIO->OUTCLR = (1 << DBG_PKT_SENT_PIN);       \
} */
/* #define DBG_WFE_BEGIN                              \
{                                                     \
    NRF_GPIO->OUTCLR = (1 << DBG_WFE_BEGIN_PIN);      \
    NRF_GPIO->DIRSET = (1 << DBG_WFE_BEGIN_PIN);      \
    NRF_GPIO->OUTSET = (1 << DBG_WFE_BEGIN_PIN);      \
    for ( int i = 0; i < 0xFF; i++ ) __NOP();         \
    NRF_GPIO->OUTCLR = (1 << DBG_WFE_BEGIN_PIN);      \
} */
/* #define DBG_WFE_END                                \
{                                                     \
    for ( int i = 0; i < 0xFF; i++ ) __NOP();         \
    NRF_GPIO->OUTSET = (1 << DBG_WFE_END_PIN);        \
} */


#ifndef DBG_BEACON_START
#define DBG_BEACON_START
#endif
#ifndef DBG_PKT_SENT
#define DBG_PKT_SENT
#endif
#ifndef DBG_HFCLK_ENABLED
#define DBG_HFCLK_ENABLED
#endif
#ifndef DBG_HFCLK_DISABLED
#define DBG_HFCLK_DISABLED
#endif
#ifndef DBG_WFE_BEGIN
#define DBG_WFE_BEGIN
#endif
#ifndef DBG_WFE_END
#define DBG_WFE_END
#endif


#ifdef PCA20014
static const hal_serial_cfg_t serial_cfg =
{
    .twi0.psel.scl = 27,
    .twi0.psel.sda = 26,
};    
#else
#ifdef PCA10036
static const hal_serial_cfg_t serial_cfg =
{
    .twi0.psel.scl = 25,
    .twi0.psel.sda = 23,
};
#else
static const hal_serial_cfg_t serial_cfg =
{
    .twi0.psel.scl = 29,
    .twi0.psel.sda = 25,    
};
#endif  
#endif


#define HFCLK_STARTUP_TIME_US                       (1600)              /* The time in microseconds it takes to start up the HF clock*. */
#define INTERVAL_US                                 (8000000)           /* The time in microseconds between advertising events. */
#define INITIAL_TIMEOUT                             (INTERVAL_US)       /* The time in microseconds until adverising the first time. */
#define START_OF_INTERVAL_TO_SENSOR_READ_TIME_US    (INTERVAL_US / 2)   /* The time from the start of the latest advertising event until reading the sensor. */
#define SENSOR_SKIP_READ_COUNT                      (5)                /* The number of advertising events between reading the sensor. */

#define POWERUP_DELAY_US														(400000)							/*delay after powerup the sensor*/


#if INITIAL_TIMEOUT - HFCLK_STARTUP_TIME_US < 400
#error "Initial timeout too short!"
#endif


#define BD_ADDR_OFFS                (3)     /* BLE device address offest of the beacon advertising pdu. */
#define M_BD_ADDR_SIZE              (6)     /* BLE device address size. */

/* Begin - Definitions for beacons with both temperature and pressure. */
#define SINT16_TEMPERATURE_OFFS     (20)    /* The offset of the temperature in the beacon advertising pdu */
#define UINT32_PRESSURE_OFFS        (26)    /* The offset of the pressure in the beacon advertising pdu */
#define UINT32_HUMIDITY_OFFS        (34)    /* The offset of the humidity in the beacon advertising pdu */
#define UINT8_BATT_OFFS        			(35)    /* The offset of the battery in the beacon advertising pdu */
/* End - Definitions for beacons with both temperature and pressure. */

/* Begin - Definitions for beacons with only temperature. */
#define FLOAT32_TEMPERATURE_OFFS    (36)    /* The offset of the temperature in the beacon advertising pdu */
/* End - Definitions for beacons with only temperature. */

#define SAADC_CALIBRATION_INTERVAL 5              //Determines how often the SAADC should be calibrated relative to NRF_DRV_SAADC_EVT_DONE event. E.g. value 5 will make the SAADC calibrate every fifth time the NRF_DRV_SAADC_EVT_DONE is received.
#define SAADC_SAMPLES_IN_BUFFER 1                 //Number of SAADC samples in RAM before returning a SAADC event. For low power SAADC set this constant to 1. Otherwise the EasyDMA will be enabled for an extended time which consumes high current.
#define SAADC_OVERSAMPLE NRF_SAADC_OVERSAMPLE_4X  //Oversampling setting for the SAADC. Setting oversample to 4x This will make the SAADC output a single averaged value when the SAMPLE task is triggered 4 times. Enable BURST mode to make the SAADC sample 4 times when triggering SAMPLE task once.
#define SAADC_BURST_MODE 1                        //Set to 1 to enable BURST mode, otherwise set to 0.

#define ADC_REF_VOLTAGE_IN_MILLIVOLTS     600
#define ADC_RES_12BIT                     4096   
#define ADC_PRE_SCALING_COMPENSATION      6   

#define ADC_RESULT_IN_MILLI_VOLTS(ADC_VALUE)\
        ((((ADC_VALUE) * ADC_REF_VOLTAGE_IN_MILLIVOLTS) / ADC_RES_12BIT) * ADC_PRE_SCALING_COMPENSATION)

void saadc_init(void);

static nrf_saadc_value_t       m_buffer_pool[2][SAADC_SAMPLES_IN_BUFFER];
static uint32_t                m_adc_evt_counter = 0;
static bool                    m_saadc_calibrate = false; 

int g_batteryVoltage_mv = 0;


/* The beacon types.
 */
typedef enum
{
    M_BEACON_PDU_TYPE_TEMP_ONLY = 0,    ///< Beacon with only temperature data.
		M_BEACON_PDU_TYPE_TEMP_BATT					///< Beacon with both temperature and battery data.
} m_beacon_pdu_type_t;


static bool volatile m_radio_isr_called;    /* Indicates that the radio ISR has executed. */
static bool volatile m_rtc_isr_called;      /* Indicates that the RTC ISR has executed. */
static uint32_t m_time_us;                  /* Keeps track of the latest scheduled point in time. */
static uint32_t m_skip_read_counter = 0;    /* Keeps track on when to read the sensor. */
static uint8_t m_adv_pdu[40];               /* The RAM representation of the advertising PDU. */


/* Initializes the beacon advertising PDU.
 */
static void m_beacon_pdu_init(uint8_t * p_beacon_pdu)
{
    p_beacon_pdu[0] = 0x42;
    p_beacon_pdu[1] = 0;
    p_beacon_pdu[2] = 0;
}


/* Sets the BLE address field in the sensor beacon PDU.
 */
static void m_beacon_pdu_bd_addr_default_set(uint8_t * p_beacon_pdu)
{
    if ( ( NRF_FICR->DEVICEADDR[0]           != 0xFFFFFFFF)
    ||   ((NRF_FICR->DEVICEADDR[1] & 0xFFFF) != 0xFFFF) )
    {
        p_beacon_pdu[BD_ADDR_OFFS    ] = (NRF_FICR->DEVICEADDR[0]      ) & 0xFF;
        p_beacon_pdu[BD_ADDR_OFFS + 1] = (NRF_FICR->DEVICEADDR[0] >>  8) & 0xFF;
        p_beacon_pdu[BD_ADDR_OFFS + 2] = (NRF_FICR->DEVICEADDR[0] >> 16) & 0xFF;
        p_beacon_pdu[BD_ADDR_OFFS + 3] = (NRF_FICR->DEVICEADDR[0] >> 24)       ;
        p_beacon_pdu[BD_ADDR_OFFS + 4] = (NRF_FICR->DEVICEADDR[1]      ) & 0xFF;
        p_beacon_pdu[BD_ADDR_OFFS + 5] = (NRF_FICR->DEVICEADDR[1] >>  8) & 0xFF;
    }
    else
    {
        static const uint8_t random_bd_addr[M_BD_ADDR_SIZE] = {0xE2, 0xA3, 0x01, 0xE7, 0x61, 0xF7};
        memcpy(&(p_beacon_pdu[3]), &(random_bd_addr[0]), M_BD_ADDR_SIZE);
    }
    
    p_beacon_pdu[1] = (p_beacon_pdu[1] == 0) ? M_BD_ADDR_SIZE : p_beacon_pdu[1];
}


/* Resets the sensor data of the sensor beacon PDU.
 */
static void m_beacon_pdu_sensor_data_reset(uint8_t * p_beacon_pdu)
{
    static const uint8_t beacon_temp_press_hum[27] = 
			{
					0x02,
					0x01, 0x04,
					0x03,
					0x03, 0xE5, 0xFE,
					/* Entry for temperature characteristics. Ref:
						 sint16, https://developer.bluetooth.org/gatt/characteristics/Pages/CharacteristicViewer.aspx?u=org.bluetooth.characteristic.temperature.xml */
					0x05,
					0x16, 0x6E, 0x2A, 0x00, 0x00,               
					/* Entry for pressure characteristics. Ref:
                       uint32, https://developer.bluetooth.org/gatt/characteristics/Pages/CharacteristicViewer.aspx?u=org.bluetooth.characteristic.pressure.xml */
					0x07,
					0x16, 0x6D, 0x2A, 0x00, 0x00, 0x00, 0x00 ,
					/* Entry for humidity characteristics.*/
					0x05,
					0x16, 0x6F, 0x2A, 0x00, 0x00
					
			};
			memcpy(&(p_beacon_pdu[3 + M_BD_ADDR_SIZE]), &(beacon_temp_press_hum[0]), sizeof(beacon_temp_press_hum));
			p_beacon_pdu[1] = M_BD_ADDR_SIZE + sizeof(beacon_temp_press_hum);
}


/* Sets the sensor data of the sensor beacon PDU.
 */
static void m_beacon_pdu_sensor_data_set(uint8_t * p_beacon_pdu, int32_t *p_temperature, uint32_t *p_pressure, uint16_t *p_humidity, uint8_t *p_battery_level)
{
		if ( (p_pressure != NULL) && (p_temperature != NULL) && (p_humidity != NULL) )
    {
        /* Pressure (Bluetooth standard) is in Pascal with one decimal, so
           multiplying the value by 10. */
        p_beacon_pdu[UINT32_PRESSURE_OFFS    ] = ((*p_pressure * 10)      ) & 0xFF;
        p_beacon_pdu[UINT32_PRESSURE_OFFS + 1] = ((*p_pressure * 10) >>  8) & 0xFF;
        p_beacon_pdu[UINT32_PRESSURE_OFFS + 2] = ((*p_pressure * 10) >> 16) & 0xFF;
        /* The temperature (Bluetooth standard) is in Celsius with two decimals, so
           deviding the value by an additional 10. */
        p_beacon_pdu[SINT16_TEMPERATURE_OFFS    ] = ((*p_temperature / 10)     ) & 0xFF;
        p_beacon_pdu[SINT16_TEMPERATURE_OFFS + 1] = ((*p_temperature / 10) >> 8) & 0xFF;
				/*Humidity. */
        p_beacon_pdu[UINT32_HUMIDITY_OFFS    ] = ((*p_humidity)     ) & 0xFF;
				/* Battery (Bluetooth standard) is in Percents*/
        p_beacon_pdu[UINT8_BATT_OFFS] = ((*p_battery_level)      ) & 0xFF;
    }
}


/* Waits for the next NVIC event.
 */
#ifdef __GNUC__
static void __INLINE cpu_wfe(void)
#else
static void __forceinline cpu_wfe(void)
#endif
{
    DBG_WFE_BEGIN;
    __WFE();
    __SEV();
    __WFE();
    DBG_WFE_END;
}


/* Hook for the access mode feature of the bme280 driver.
 */
static void cpu_sleep_hook(void)
{
    cpu_wfe();
}

void wait_for_timer(void)
{
		while ( !m_rtc_isr_called )
		{
				cpu_wfe();
		}
}


/* Powers up the the mcp9808 device and TWI pull-up resistors.
 */
static void sensor_chip_powerup(void)
{
    static const drv_bme280_cfg_t m_drv_bme280_cfg =
    {
        .twi_id = HAL_TWI_ID_TWI0,
        .twi_cfg.address   = (0x77 << TWI_ADDRESS_ADDRESS_Pos),
        .twi_cfg.frequency = (TWI_FREQUENCY_FREQUENCY_K400 << TWI_FREQUENCY_FREQUENCY_Pos),
        .p_sleep_hook = cpu_sleep_hook,
    };
    
    if ( drv_bme280_open(&m_drv_bme280_cfg) == DRV_BME280_STATUS_CODE_SUCCESS )
    {
        drv_bme280_access_mode_set(DRV_BME280_ACCESS_MODE_CPU_INACTIVE);
    
        sensor_init();
			
				sensor_reset();
    
				m_time_us += POWERUP_DELAY_US;
			
				m_rtc_isr_called = false;
				hal_timer_timeout_set(m_time_us);
				wait_for_timer();
			
				read_coefficients(); // read trimming parameters, see DS 4.2.2

				sensor_set_sampling(MODE_NORMAL,
					SAMPLING_X1,
					SAMPLING_X1,
					SAMPLING_X1,
					FILTER_OFF,
					STANDBY_MS_1000); // use defaults
	
				m_time_us += 100000;

				m_rtc_isr_called = false;
				hal_timer_timeout_set(m_time_us);
				wait_for_timer();
        hal_clock_hfclk_disable();
    }
    else
    {
        (void)drv_bme280_close();
    }
}


/* Ends after measuring temperature and pressure.
 */
static void sensor_chip_measurement_done(void)
{
    (void)drv_bme280_close();
}


/* Sends an advertising PDU on the given channel index.
 */
static void send_one_packet(uint8_t channel_index)
{
    uint8_t i;
    
    m_radio_isr_called = false;
    hal_radio_channel_index_set(channel_index);
    hal_radio_send(m_adv_pdu);
    while ( !m_radio_isr_called )
    {
        cpu_wfe();
    }
    
    for ( i = 0; i < 9; i++ )
    {
        __NOP();
    }
}


void get_battery_level(uint8_t* bat_level)
{
		*bat_level = g_batteryVoltage_mv / 100 * 2;
}


/* Handles sensor managing.
 */
static void sensor_handler(uint32_t start_time_us)
{
    int32_t temperature = 0;	
		uint32_t pressure = 1000;
		uint16_t humidity = 80;
		uint8_t battery_level = 28;
		
		float real_temp = sensor_read_temperature();
		temperature = real_temp * 1000;
	
		float real_hum = sensor_read_humidity();
		humidity = real_hum;
	
		float real_press = sensor_read_pressure();
		pressure = real_press;
		

		get_battery_level(&battery_level);
				
		m_beacon_pdu_sensor_data_set(&(m_adv_pdu[0]), &temperature, &pressure, &humidity, &battery_level);   
}

void saadc_callback(nrf_drv_saadc_evt_t const * p_event)
{
    ret_code_t err_code;
    if (p_event->type == NRF_DRV_SAADC_EVT_DONE)                                                        //Capture offset calibration complete event
    {
	
        if((m_adc_evt_counter % SAADC_CALIBRATION_INTERVAL) == 0)                                  //Evaluate if offset calibration should be performed. Configure the SAADC_CALIBRATION_INTERVAL constant to change the calibration frequency
        {
            nrf_drv_saadc_abort();                                                                      // Abort all ongoing conversions. Calibration cannot be run if SAADC is busy
            m_saadc_calibrate = true;                                                                   // Set flag to trigger calibration in main context when SAADC is stopped
        }  
        
        if(m_saadc_calibrate == false)
        {
						float measured_voltage = (float)(p_event->data.done.p_buffer[0]) * 0.6 * 6.0 / 16384.0; 
				
						g_batteryVoltage_mv = measured_voltage * 1000; //ADC_RESULT_IN_MILLI_VOLTS(p_event->data.done.p_buffer[0]);
					
            err_code = nrf_drv_saadc_buffer_convert(p_event->data.done.p_buffer, SAADC_SAMPLES_IN_BUFFER);             //Set buffer so the SAADC can write to it again. 						
        }
        
        m_adc_evt_counter++;
  
    }
    else if (p_event->type == NRF_DRV_SAADC_EVT_CALIBRATEDONE)
    {
     
        err_code = nrf_drv_saadc_buffer_convert(m_buffer_pool[0], SAADC_SAMPLES_IN_BUFFER);             //Set buffer so the SAADC can write to it again. 
 
        err_code = nrf_drv_saadc_buffer_convert(m_buffer_pool[1], SAADC_SAMPLES_IN_BUFFER);             //Need to setup both buffers, as they were both removed with the call to nrf_drv_saadc_abort before calibration.
        
    }
}

void calibration_handler(void)
{
	if(m_saadc_calibrate == true)
	{
			while(nrf_drv_saadc_calibrate_offset() != NRF_SUCCESS); //Trigger calibration task
			m_saadc_calibrate = false;
	}
}

void saadc_init(void)
{
    ret_code_t err_code;
    nrf_drv_saadc_config_t saadc_config;
    nrf_saadc_channel_config_t channel_config;


	
    //Configure SAADC
    saadc_config.low_power_mode = true;                                                   //Enable low power mode.
    saadc_config.resolution = NRF_SAADC_RESOLUTION_14BIT;                                 //Set SAADC resolution to 12-bit. This will make the SAADC output values from 0 (when input voltage is 0V) to 2^14=16384 (when input voltage is 3.6V for channel gain setting of 1/6).
    saadc_config.oversample = SAADC_OVERSAMPLE;                                           //Set oversample to 4x. This will make the SAADC output a single averaged value when the SAMPLE task is triggered 4 times.
    saadc_config.interrupt_priority = APP_IRQ_PRIORITY_LOW;                               //Set SAADC interrupt to low priority.
	
    //Initialize SAADC
    err_code = nrf_drv_saadc_init(&saadc_config, saadc_callback);                         //Initialize the SAADC with configuration and callback function. The application must then implement the saadc_callback function, which will be called when SAADC interrupt is triggered
    
    //Configure SAADC channel
    channel_config.reference = NRF_SAADC_REFERENCE_INTERNAL;                              //Set internal reference of fixed 0.6 volts
    channel_config.gain = NRF_SAADC_GAIN1_6;                                              //Set input gain to 1/6. The maximum SAADC input voltage is then 0.6V/(1/6)=3.6V. The single ended input range is then 0V-3.6V
    channel_config.acq_time = NRF_SAADC_ACQTIME_10US;                                     //Set acquisition time. Set low acquisition time to enable maximum sampling frequency of 200kHz. Set high acquisition time to allow maximum source resistance up to 800 kohm, see the SAADC electrical specification in the PS. 
    channel_config.mode = NRF_SAADC_MODE_SINGLE_ENDED;                                    //Set SAADC as single ended. This means it will only have the positive pin as input, and the negative pin is shorted to ground (0V) internally.
    channel_config.pin_p = NRF_SAADC_INPUT_AIN0;                                          //Select the input pin for the channel. AIN0 pin maps to physical pin P0.02.
    channel_config.pin_n = NRF_SAADC_INPUT_DISABLED;                                      //Since the SAADC is single ended, the negative pin is disabled. The negative pin is shorted to ground internally.
    channel_config.resistor_p = NRF_SAADC_RESISTOR_DISABLED;                              //Disable pullup resistor on the input pin
    channel_config.resistor_n = NRF_SAADC_RESISTOR_DISABLED;                              //Disable pulldown resistor on the input pin

	
    //Initialize SAADC channel
    err_code = nrf_drv_saadc_channel_init(0, &channel_config);                            //Initialize SAADC channel 0 with the channel configuration
    
    if(SAADC_BURST_MODE)
    {
        NRF_SAADC->CH[0].CONFIG |= 0x01000000;                                            //Configure burst mode for channel 0. Burst is useful together with oversampling. When triggering the SAMPLE task in burst mode, the SAADC will sample "Oversample" number of times as fast as it can and then output a single averaged value to the RAM buffer. If burst mode is not enabled, the SAMPLE task needs to be triggered "Oversample" number of times to output a single averaged value to the RAM buffer.		
    }

    err_code = nrf_drv_saadc_buffer_convert(m_buffer_pool[0],SAADC_SAMPLES_IN_BUFFER);    //Set SAADC buffer 1. The SAADC will start to write to this buffer
    
    err_code = nrf_drv_saadc_buffer_convert(m_buffer_pool[1],SAADC_SAMPLES_IN_BUFFER);    //Set SAADC buffer 2. The SAADC will write to this buffer when buffer 1 is full. This will give the applicaiton time to process data in buffer 1.
    
}


/* Handles beacon managing.
 */
static void beacon_handler(void)
{
    hal_radio_reset();
    hal_timer_start();
		sensor_chip_powerup();
    
    m_time_us = INITIAL_TIMEOUT - HFCLK_STARTUP_TIME_US; 
	
		m_time_us += 40000;	

    do
    {
        if ( m_skip_read_counter == 0 )
        {											
            sensor_handler(m_time_us);
						nrf_drv_saadc_sample();                                        //Trigger the SAADC SAMPLE task
						calibration_handler();
//						sensor_chip_measurement_done();
        }
        m_skip_read_counter = ( (m_skip_read_counter + 1) < SENSOR_SKIP_READ_COUNT ) ? (m_skip_read_counter + 1) : 0;
				
        hal_clock_hfclk_enable();
        DBG_HFCLK_ENABLED;
        
        m_rtc_isr_called = false;
        m_time_us += HFCLK_STARTUP_TIME_US; 
        hal_timer_timeout_set(m_time_us);
        wait_for_timer();
				
        send_one_packet(37);
        DBG_PKT_SENT;
        send_one_packet(38);
        DBG_PKT_SENT;
        send_one_packet(39);
        DBG_PKT_SENT;
        
        hal_clock_hfclk_disable();
        
        DBG_HFCLK_DISABLED;
        
        m_time_us = m_time_us + (INTERVAL_US - HFCLK_STARTUP_TIME_US); 
    } while ( 1 );
}  


int main(void)
{        
    DBG_BEACON_START;

    NRF_GPIO->OUTCLR = 0xFFFFFFFF;
    NRF_GPIO->DIRCLR = 0xFFFFFFFF;
        
        
#ifdef DBG_WFE_BEGIN_PIN
    NRF_GPIO->OUTSET = (1 << DBG_WFE_BEGIN_PIN);
    NRF_GPIO->DIRSET = (1 << DBG_WFE_BEGIN_PIN);
#endif
	
		NRF_GPIO->OUTSET = (1 << 27) | (1 << 26);
		NRF_GPIO->DIRSET = (1 << 27) | (1 << 26);
	
		NRF_GPIO->PIN_CNF[27] |=  GPIO_PIN_CNF_PULL_Pullup << GPIO_PIN_CNF_PULL_Pos;
		NRF_GPIO->PIN_CNF[26] |=  GPIO_PIN_CNF_PULL_Pullup << GPIO_PIN_CNF_PULL_Pos;
    
    hal_serial_init(&serial_cfg);
    hal_twi_init();
    
    drv_bme280_init();
	
		saadc_init();                                    //Initialize and start SAADC
		
    m_beacon_pdu_init(&(m_adv_pdu[0]));
    m_beacon_pdu_bd_addr_default_set(&(m_adv_pdu[0]));
    m_beacon_pdu_sensor_data_reset(&(m_adv_pdu[0]));
    
#ifdef DBG_RADIO_ACTIVE_ENABLE
    NRF_GPIOTE->CONFIG[0] = (GPIOTE_CONFIG_POLARITY_Toggle << GPIOTE_CONFIG_POLARITY_Pos) 
                          | (DBG_RADIO_ACTIVE_PIN << GPIOTE_CONFIG_PSEL_Pos) 
                          | (GPIOTE_CONFIG_MODE_Task << GPIOTE_CONFIG_MODE_Pos); 

    NRF_PPI->CH[5].TEP = (uint32_t)&(NRF_GPIOTE->TASKS_OUT[0]); 
    NRF_PPI->CH[5].EEP = (uint32_t)&(NRF_RADIO->EVENTS_READY); 
    NRF_PPI->CH[6].TEP = (uint32_t)&(NRF_GPIOTE->TASKS_OUT[0]); 
    NRF_PPI->CH[6].EEP = (uint32_t)&(NRF_RADIO->EVENTS_DISABLED); 

    NRF_PPI->CHENSET = (PPI_CHEN_CH5_Enabled << PPI_CHEN_CH5_Pos)
                     | (PPI_CHEN_CH6_Enabled << PPI_CHEN_CH6_Pos); 
#endif

    for (;;)
    {
        beacon_handler();
    }
}


void RADIO_IRQHandler(void)
{
    NRF_RADIO->EVENTS_DISABLED = 0;
    m_radio_isr_called = true;    
}


void RTC0_IRQHandler(void)
{
    NRF_RTC0->EVTENCLR = (RTC_EVTENCLR_COMPARE0_Enabled << RTC_EVTENCLR_COMPARE0_Pos);
    NRF_RTC0->INTENCLR = (RTC_INTENCLR_COMPARE0_Enabled << RTC_INTENCLR_COMPARE0_Pos);
    NRF_RTC0->EVENTS_COMPARE[0] = 0;
    
    m_rtc_isr_called = true;    
}
