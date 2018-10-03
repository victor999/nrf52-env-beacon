#ifndef DRV_MCP9808_H__
#define DRV_MCP9808_H__

#include "hal_twi.h"


/**@brief The mcp9808 status codes.
 */
enum
{
    DRV_MCP9808_STATUS_CODE_SUCCESS,         ///< Successfull.
    DRV_MCP9808_STATUS_CODE_DISALLOWED,      ///< Disallowed.
    DRV_MCP9808_STATUS_CODE_INVALID_PARAM,   ///< Invalid parameter.
};


/**@brief The access modes.
 */
typedef enum
{
    DRV_MCP9808_ACCESS_MODE_CPU_INACTIVE,    ///< The CPU is inactive while waiting for the access to complete.
    DRV_MCP9808_ACCESS_MODE_CPU_ACTIVE,      ///< The CPU is active while waiting for the access to complete.
} drv_mcp9808_access_mode_t;


/**@brief The type of the signal callback conveying signals from the driver.
 */
typedef void (*drv_mcp9808_sleep_hook_t)(void);


/**@brief The mcp9808 configuration.
 */
typedef struct
{
    hal_twi_id_t            twi_id;         ///< The ID of TWI master to be used for transactions.
    hal_twi_cfg_t           twi_cfg;        ///< The TWI configuration to use while the driver is opened.
    drv_mcp9808_sleep_hook_t p_sleep_hook;   ///< Pointer to a function for CPU power down to be used in the CPU inactive mode.
} drv_mcp9808_cfg_t;


/**@brief Initializes the mcp9808 interface.
 */
void drv_mcp9808_init(void);


/**@brief Opens access to the mcp9808 driver.
 *
 * @param{in] id    The id of the HW peripheral to open the driver for.
 * @param{in] cfg   The driver configuration.
 *
 * @retval ::DRV_MCP9808_STATUS_CODE_SUCCESS     if successful.
 * @retval ::DRV_MCP9808_STATUS_CODE_DISALLOWED  if the driver could not be opened.
 */
uint32_t drv_mcp9808_open(drv_mcp9808_cfg_t const * const p_drv_mcp9808_cfg);


/**@brief Sets the access mode.
 *
 * @nore Reading and writing data will be blocking calls if no callback is set.
 *
 * @param{in] access_mode   The mode to be used while accessing the mcp9808 chip.
 *
 * @return DRV_MCP9808_STATUS_CODE_SUCCESS          If the call was successful.
 * @return DRV_MCP9808_STATUS_CODE_DISALLOWED       If the call was not allowed at this time.
 * @return DRV_MCP9808_STATUS_CODE_INVALID_PARAM    If specified mode is not compatible with the configuration.
 */
uint32_t drv_mcp9808_access_mode_set(drv_mcp9808_access_mode_t access_mode);

/**@brief Gets the config register of the mcp9808 device.
 *
 * @param[in]   p_reg_value    A pointer to where the config is to be stored.
 *
 * @return DRV_MCP9808_STATUS_CODE_SUCCESS      If the call was successful.
 * @return DRV_MCP9808_STATUS_CODE_DISALLOWED   If the call was not allowed at this time. 
 */
uint32_t drv_mcp9808_config_reg_get(uint16_t *p_reg_value);

/**@brief Sets the config register of the mcp9808 device.
 *
 * @param[in]   reg_value    A value for the register.
 *
 * @return DRV_MCP9808_STATUS_CODE_SUCCESS      If the call was successful.
 * @return DRV_MCP9808_STATUS_CODE_DISALLOWED   If the call was not allowed at this time. 
 */
uint32_t drv_mcp9808_config_reg_set(uint16_t reg_value);


/**@brief Gets the temperature in milli degrees Celcius.
 *
 * @param[in]   p_temperature_milli_deg A pointer to where value of the temperature is to be stored.
 *
 * @return DRV_MCP9808_STATUS_CODE_SUCCESS      If the call was successful.
 * @return DRV_MCP9808_STATUS_CODE_DISALLOWED   If the call was not allowed at this time.
 */
uint32_t drv_mcp9808_temperature_get(int32_t * p_temperature_milli_deg);

/**@brief Opens access to the mcp9808 driver.
 *
 * @retval ::DRV_MCP9808_STATUS_CODE_SUCCESS     if successful.
 * @retval ::DRV_MCP9808_STATUS_CODE_DISALLOWED  if the driver could not be opened.
 */
uint32_t drv_mcp9808_close(void);


#endif // DRV_MCP9808_H__
