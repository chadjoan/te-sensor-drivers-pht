/// \file ms8607.c
///
/// \brief MS8607 Temperature sensor driver source file
///
/// Copyright (c) 2016 Measurement Specialties. All rights reserved.
///
/// For details on programming, refer to ms8607 datasheet :
/// http://www.meas-spec.com/downloads/MS8607D.pdf
///
///

#include "ms8607.h"

#include <assert.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

// HSENSOR device address
#define HSENSOR_ADDR                                        0x40 //0b1000000

// HSENSOR device commands
#define HSENSOR_RESET_COMMAND                               0xFE
#define HSENSOR_READ_HUMIDITY_W_HOLD_COMMAND                0xE5
#define HSENSOR_READ_HUMIDITY_WO_HOLD_COMMAND               0xF5
#define HSENSOR_READ_SERIAL_FIRST_8BYTES_COMMAND            0xFA0F
#define HSENSOR_READ_SERIAL_LAST_6BYTES_COMMAND             0xFCC9
#define HSENSOR_WRITE_USER_REG_COMMAND                      0xE6
#define HSENSOR_READ_USER_REG_COMMAND                       0xE7

// Processing constants
#define HSENSOR_TEMPERATURE_COEFFICIENT                     (float)(-0.15)
#define HSENSOR_CONSTANT_A                                  (float)(8.1332)
#define HSENSOR_CONSTANT_B                                  (float)(1762.39)
#define HSENSOR_CONSTANT_C                                  (float)(235.66)

// Coefficients for temperature computation
#define TEMPERATURE_COEFF_MUL                               (175.72)
#define TEMPERATURE_COEFF_ADD                               (-46.85)

// Coefficients for relative humidity computation
#define HUMIDITY_COEFF_MUL                                  (125)
#define HUMIDITY_COEFF_ADD                                  (-6)

// Conversion timings
#define HSENSOR_CONVERSION_TIME_12b                         16000
#define HSENSOR_CONVERSION_TIME_10b                         5000
#define HSENSOR_CONVERSION_TIME_8b                          3000
#define HSENSOR_CONVERSION_TIME_11b                         9000

#define HSENSOR_RESET_TIME                                  15       // ms value

// HSENSOR User Register masks and bit position
#define HSENSOR_USER_REG_RESOLUTION_MASK                    0x81
#define HSENSOR_USER_REG_END_OF_BATTERY_MASK                0x40
#define HSENSOR_USER_REG_ENABLE_ONCHIP_HEATER_MASK          0x4
#define HSENSOR_USER_REG_DISABLE_OTP_RELOAD_MASK            0x2
#define HSENSOR_USER_REG_RESERVED_MASK                      (~(     HSENSOR_USER_REG_RESOLUTION_MASK           \
                                                                |   HSENSOR_USER_REG_END_OF_BATTERY_MASK       \
                                                                |   HSENSOR_USER_REG_ENABLE_ONCHIP_HEATER_MASK \
                                                                |   HSENSOR_USER_REG_DISABLE_OTP_RELOAD_MASK   ))

// HTU User Register values
// Resolution
#define HSENSOR_USER_REG_RESOLUTION_12b                     0x00
#define HSENSOR_USER_REG_RESOLUTION_11b                     0x81
#define HSENSOR_USER_REG_RESOLUTION_10b                     0x80
#define HSENSOR_USER_REG_RESOLUTION_8b                      0x01

// End of battery status
#define HSENSOR_USER_REG_END_OF_BATTERY_VDD_ABOVE_2_25V     0x00
#define HSENSOR_USER_REG_END_OF_BATTERY_VDD_BELOW_2_25V     0x40
// Enable on chip heater
#define HSENSOR_USER_REG_ONCHIP_HEATER_ENABLE               0x04
#define HSENSOR_USER_REG_OTP_RELOAD_DISABLE                 0x02

// PSENSOR device address
#define PSENSOR_ADDR                                        0x76 //0b1110110

// PSENSOR device commands
#define PSENSOR_RESET_COMMAND                               0x1E
#define PSENSOR_START_PRESSURE_ADC_CONVERSION               0x40
#define PSENSOR_START_TEMPERATURE_ADC_CONVERSION            0x50
#define PSENSOR_READ_ADC                                    0x00

#define PSENSOR_CONVERSION_OSR_MASK                         0x0F

#define PSENSOR_CONVERSION_TIME_OSR_256                     1000
#define PSENSOR_CONVERSION_TIME_OSR_512                     2000
#define PSENSOR_CONVERSION_TIME_OSR_1024                    3000
#define PSENSOR_CONVERSION_TIME_OSR_2048                    5000
#define PSENSOR_CONVERSION_TIME_OSR_4096                    9000
#define PSENSOR_CONVERSION_TIME_OSR_8192                    18000

// PSENSOR commands
#define PROM_ADDRESS_READ_ADDRESS_0                         0xA0

// Coefficients indexes for temperature and pressure computation
#define CRC_INDEX                                           0
#define PRESSURE_SENSITIVITY_INDEX                          1
#define PRESSURE_OFFSET_INDEX                               2
#define TEMP_COEFF_OF_PRESSURE_SENSITIVITY_INDEX            3
#define TEMP_COEFF_OF_PRESSURE_OFFSET_INDEX                 4
#define REFERENCE_TEMPERATURE_INDEX                         5
#define TEMP_COEFF_OF_TEMPERATURE_INDEX                     6

#define MAX_CONVERSION_TIME                                 HSENSOR_CONVERSION_TIME_12b

// Global constants
static uint32_t psensor_conversion_time[6] = {
		PSENSOR_CONVERSION_TIME_OSR_256,
		PSENSOR_CONVERSION_TIME_OSR_512,
		PSENSOR_CONVERSION_TIME_OSR_1024,
		PSENSOR_CONVERSION_TIME_OSR_2048,
		PSENSOR_CONVERSION_TIME_OSR_4096,
		PSENSOR_CONVERSION_TIME_OSR_8192
	};

// Static functions
// humidity sensor functions
static tepht_error_info  hsensor_reset(ms8607_sensor *sensor, void *caller_context);
static bool              hsensor_is_connected(ms8607_sensor *sensor, void *caller_context);
static tepht_status      hsensor_write_command(ms8607_sensor *sensor, uint8_t, void *caller_context);
static tepht_status      hsensor_write_command_no_stop(ms8607_sensor *sensor, uint8_t, void *caller_context);
static tepht_status      hsensor_crc_check( uint16_t, uint8_t);
static tepht_status      hsensor_read_user_register(ms8607_sensor *sensor, uint8_t *value, void *caller_context);
static tepht_status      hsensor_write_user_register(ms8607_sensor *sensor, uint8_t value, void *caller_context);
static tepht_error_info  hsensor_humidity_conversion_and_read_adc(ms8607_sensor *sensor, uint16_t *, void *caller_context);
static tepht_error_info  hsensor_read_relative_humidity(ms8607_sensor *sensor, int32_t *, void *caller_context);

// Pressure sensor functions
static tepht_error_info  psensor_reset(ms8607_sensor *sensor, void *caller_context);
static bool              psensor_is_connected(ms8607_sensor *sensor, void *caller_context);
static tepht_status      psensor_write_command(ms8607_sensor *sensor, uint8_t, void *caller_context);
static tepht_error_info  psensor_read_eeprom_coeff(ms8607_sensor *sensor, uint8_t, uint16_t*, void *caller_context);
static tepht_error_info  psensor_read_eeprom(ms8607_sensor *sensor, void *caller_context);
static tepht_error_info  psensor_conversion_and_read_adc(ms8607_sensor *sensor, uint8_t, uint32_t *, void *caller_context);
static bool              psensor_crc_check (uint16_t *n_prom, uint8_t crc);
static tepht_error_info  psensor_read_pressure_and_temperature(ms8607_sensor *sensor, int32_t *, int32_t *, void *caller_context);

static const char *ms8607_get_device_model_name(void *ms8607_sensor) {
	(void)ms8607_sensor;
	return "MS8607";
}

static const char *ms8607_get_driver_prefix(void *ms8607_sensor) {
	(void)ms8607_sensor;
	return "ms8607";
}

static tepht_driver_context_accessor_vtable  ms8607_dca_vtable =
	{
		.get_device_model_name = ms8607_get_device_model_name,
		.get_driver_prefix     = ms8607_get_driver_prefix
	};

static tepht_driver_context_accessor  ms8607_default_dca =
	{
		.self   = NULL,
		.vtable = &ms8607_dca_vtable
	};

static inline
	const tepht_driver_context_accessor  *ms8607_get_dca(ms8607_sensor *sensor)
{
	if ( sensor != NULL )
		return &(sensor->context_accessor);
	else
		return &ms8607_default_dca;
}

static inline tepht_error_info  ms8607_success(ms8607_sensor *sensor) {
	return tepht_success(ms8607_get_dca(sensor));
}

// This must be a macro to preserve __FILE__ and __LINE__ information.
#define  ms8607_error(sensor, status)  (tepht_error(ms8607_get_dca(sensor), (status)))

/// \brief    Initializes a new `ms8607_sensor` object.
///
/// \details  This function's purpose is to place the given `new_sensor`
///           argument into a predictable state and provide it with some
///           default configuration. This function does not actually communicate
///           with the sensor in any way.
///
///           This must be called after `ms8607_init_and_assign_host_functions`
///           has been called at least once (to obtain the argument for the
///           `depends_to_use` parameter), and it must be called before any
///           other function that requires a `ms8607_sensor*` type parameter
///           (usually `ms8607_reset`).
///
///           This function is reentrant, idempotent, non-blocking,
///           and does not perform any I/O. This function is thread-safe
///           as long as, during this function's execution, no other threads
///           read from or write to that call's `new_sensor` instance, and
///           no other threads write to that call's `depends_to_use` instance.
///
/// \param[out] ms8607_sensor *new_sensor : The new sensor object.
/// \param[in]  const tepht_host_functions *depends_to_use : Specifies the
///           functions that this sensor can call to do things such as I2C I/O
///           and timing.
///
/// \return tepht_status : status of MS8607
///       - tepht_status_ok : Sensor object was initailized successfully
///       - tepht_status_null_sensor : The pointer provided for the `new_sensor` parameter was NULL.
///       - tepht_status_null_argument : The pointer provided for the `depends_to_use` parameter was NULL.
///       - tepht_status_i2c_read_unimplemented : Returned if the `i2c_controller_read` function was not assigned.
///       - tepht_status_i2c_write_unimplemented : Returned if the `i2c_controller_write` function was not assigned.
///       - tepht_status_sleep_ms_unimplemented : Returned if the `sleep_ms` function was not assigned.
///
tepht_error_info  ms8607_init_sensor(ms8607_sensor *new_sensor,  tepht_host_functions *depends_to_use)
{
	tepht_status status;

	if ( new_sensor == NULL )
		return ms8607_error(new_sensor, tepht_status_null_sensor);

	if ( depends_to_use == NULL )
		return ms8607_error(new_sensor, tepht_status_null_argument);

	status = tepht_validate_mandatory_depends(depends_to_use);
	if ( status != tepht_status_ok )
		return ms8607_error(new_sensor, status);

	ms8607_sensor  const_init_sensor = {
		.host_funcs                  = depends_to_use,
		.context_accessor.self       = new_sensor,
		.context_accessor.vtable     = &ms8607_dca_vtable,

		// Defaults
		.hsensor_conversion_time     = HSENSOR_CONVERSION_TIME_12b,
		.hsensor_i2c_controller_mode = ms8607_i2c_no_hold,
		.psensor_resolution_osr      = ms8607_pressure_resolution_osr_8192,
		.hsensor_heater_on           = false,
		.psensor_coeff_read          = false
	};

	memcpy(new_sensor, &const_init_sensor, sizeof(ms8607_sensor));

	uint8_t i = 0;
	for (; i < MS8607_COEFFICIENT_COUNT+1; i++)
		new_sensor->eeprom_coeff[i] = 0;

	return ms8607_success(new_sensor);
}

/// \brief Check whether MS8607 device is connected
///
/// \param[in] ms8607_sensor *sensor : Sensor object to test for connectivity.
/// \param[in] void* caller_context : When this function calls any callbacks
///         from the `tepht_host_functions` structure, this will be passed
///         directly to those callbacks' `caller_context` parameter.
///
/// \return bool : status of MS8607
///       - true : Device is present
///       - false : Device is not acknowledging I2C address
///
tepht_bool ms8607_is_connected(ms8607_sensor *sensor,  void *caller_context)
{
	return (
		hsensor_is_connected(sensor, caller_context) &&
		psensor_is_connected(sensor, caller_context)
	);
}

static tepht_bool  ms8607_is_connected__virtual(void *self,  void *caller_context) {
	return ms8607_is_connected(self, caller_context);
}

// TODO:
///          If this function is not called before any other functions that
///          communicate with the sensor sensor (ex: `ms8607_<TODO_function_name>`),
///          then it will be called automatically by the other function.
///
///          Configuration that is stored locally within the `ms8607_sensor`
///          object, such as pressure resolution settings, will not be affected
///          by this function.
///
///          Configuration that is stored remotely on the physical sensor,
///          such as humidity resolution settings or heater activation, will
///          experience temporary changes during this function's execution,
///          but by the time this function returns, the sensor will (normally)
///          be brought into the (configuration) state that was described by
///          the `ms8607_sensor` object at the beginning of the function call.
///          (The physical RESET causes the sensor to revert to factory settings,
///          so the `ms8607_reset` will write the `ms8607_sensor` object's
///          settings to the sensor after the RESET completes.) (TODO: Verify. Implement.)


/// \brief Reset the MS8607 device
///
/// \details
///           This function is reentrant. It blocks and performs I2C I/O.
///           It is thread-safe as long as, during this function's execution,
///           no other threads read from or write to the given `sensor` instance.
///
/// \param[in] ms8607_sensor *sensor : Object representing the sensor to be reset.
/// \param[in] void* caller_context : When this function calls any callbacks
///         from the `tepht_host_functions` structure, this will be passed
///         directly to those callbacks' `caller_context` parameter.
///
/// \return tepht_status : status of MS8607
///       - tepht_status_ok : I2C transfer completed successfully
///       - tepht_status_null_sensor : The pointer provided for the `sensor` parameter was NULL.
///       - tepht_status_callback_error : Error occurred within a tepht_host_functions function
///
tepht_error_info  ms8607_reset(ms8607_sensor *sensor,  void *caller_context)
{
	if ( sensor == NULL )
		return ms8607_error(sensor, tepht_status_null_sensor);

	tepht_error_info  einfo;

	einfo = hsensor_reset(sensor, caller_context);
	if( tepht_is_error(einfo) )
		return einfo;
	einfo = psensor_reset(sensor, caller_context);
	if( tepht_is_error(einfo) )
		return einfo;

	return ms8607_success(sensor);
}

static tepht_error_info  ms8607_reset__virtual(void *self,  void *caller_context) {
	return ms8607_reset(self, caller_context);
}

/// \brief Set humidity ADC resolution.
///
/// \param[in] ms8607_sensor *sensor : Object representing the sensor to set humidity resolution on.
/// \param[in] ms8607_humidity_resolution : Resolution requested
/// \param[in] void* caller_context : When this function calls any callbacks
///         from the `tepht_host_functions` structure, this will be passed
///         directly to those callbacks' `caller_context` parameter.
///
/// \return tepht_status : status of MS8607
///       - tepht_status_ok : I2C transfer completed successfully
///       - tepht_status_null_sensor : The pointer provided for the `sensor` parameter was NULL.
///       - tepht_status_callback_error : Error occurred within a tepht_host_functions function
///
tepht_error_info  ms8607_set_humidity_resolution(
	ms8607_sensor                    *sensor,
	enum ms8607_humidity_resolution  res,
	void                             *caller_context)
{
	if ( sensor == NULL )
		return ms8607_error(sensor, tepht_status_null_sensor);

	tepht_status status;
	uint8_t reg_value, tmp=0;
	uint32_t conversion_time = HSENSOR_CONVERSION_TIME_12b;

	if( res == ms8607_humidity_resolution_12b) {
		tmp = HSENSOR_USER_REG_RESOLUTION_12b;
		conversion_time = HSENSOR_CONVERSION_TIME_12b;
	}
	else if( res == ms8607_humidity_resolution_10b) {
		tmp = HSENSOR_USER_REG_RESOLUTION_10b;
		conversion_time = HSENSOR_CONVERSION_TIME_10b;
	}
	else if( res == ms8607_humidity_resolution_8b) {
		tmp = HSENSOR_USER_REG_RESOLUTION_8b;
		conversion_time = HSENSOR_CONVERSION_TIME_8b;
	}
	else if( res == ms8607_humidity_resolution_11b) {
		tmp = HSENSOR_USER_REG_RESOLUTION_11b;
		conversion_time = HSENSOR_CONVERSION_TIME_11b;
	}

	status = hsensor_read_user_register(sensor, &reg_value, caller_context);
	if( status != tepht_status_ok )
		return ms8607_error(sensor, status);

	// Clear the resolution bits
	reg_value &= ~HSENSOR_USER_REG_RESOLUTION_MASK;
	reg_value |= tmp & HSENSOR_USER_REG_RESOLUTION_MASK;

	sensor->hsensor_conversion_time = conversion_time;

	status = hsensor_write_user_register(sensor, reg_value, caller_context);
	if ( status != tepht_status_ok )
		return ms8607_error(sensor, status);

	return ms8607_success(sensor);
}

/// \brief Set Humidity sensor ADC resolution.
///
/// \param[in] ms8607_sensor *sensor : Object representing the sensor to set controller mode on.
/// \param[in] ms8607_i2c_controller_mode : I2C mode
/// \param[in] void* caller_context : When this function calls any callbacks
///         from the `tepht_host_functions` structure, this will be passed
///         directly to those callbacks' `caller_context` parameter.
///         (As of this writing, this function does not do any I2C I/O and
///         does not call any host functions, so `caller_context` is unused here,
///         but nonetheless provided for sake of future-proofing.)
///
/// \return tepht_status : status of MS8607
///       - tepht_status_ok
///       - tepht_status_null_sensor : The pointer provided for the `sensor` parameter was NULL.
///
tepht_error_info  ms8607_set_humidity_i2c_controller_mode(ms8607_sensor *sensor,  enum ms8607_humidity_i2c_controller_mode mode, void *caller_context)
{
	(void)caller_context;
	if ( sensor == NULL )
		return ms8607_error(sensor, tepht_status_null_sensor);

	sensor->hsensor_i2c_controller_mode = mode;
	return ms8607_success(sensor);
}

/// \brief    Reads the temperature, pressure and relative humidity values.
///
/// \details  The results are returned in thousanths as a way to preserve
///           the sensor's resolution while using integer types.
///
/// \param[in] ms8607_sensor* sensor : The sensor to use for measuring
/// \param[out] int32_t* : Thousanths of degC temperature value
/// \param[out] int32_t* : Microbar pressure value (thousanths of millibar)
/// \param[out] int32_t* : Thousanths of %RH Relative Humidity value
/// \param[in]  void* caller_context : When this function calls any callbacks
///         from the `tepht_host_functions` structure, this will be passed
///         directly to those callbacks' `caller_context` parameter.
///
/// \return tepht_status : status of MS8607
///       - tepht_status_ok : I2C transfer completed successfully
///       - tepht_status_null_sensor : The pointer provided for the `sensor` parameter was NULL.
///       - tepht_status_null_argument : One or more of the `t`, `p`, or `h` pointers were NULL.
///       - tepht_status_callback_error : Error occurred within a tepht_host_functions function
///       - tepht_status_eeprom_is_zero : One or more EEPROM coefficients were received as 0, preventing measurement.
///       - tepht_status_eeprom_crc_error : CRC check error on the sensor's EEPROM coefficients
///       - tepht_status_measurement_invalid : EEPROM is OK and I2C transfer completed, but data received was invalid
///
tepht_error_info  ms8607_read_temperature_pressure_humidity_int32(ms8607_sensor *sensor, int32_t *t, int32_t *p, int32_t *h,  void *caller_context)
{
	if ( sensor == NULL )
		return ms8607_error(sensor, tepht_status_null_sensor);

	if ( t == NULL || p == NULL || h == NULL )
		return ms8607_error(sensor, tepht_status_null_argument);

	tepht_error_info  einfo;
	einfo = psensor_read_pressure_and_temperature(sensor, t, p, caller_context);
	if( tepht_is_error(einfo) )
		return einfo;

	einfo = hsensor_read_relative_humidity(sensor, h, caller_context);
	if( tepht_is_error(einfo) )
		return einfo;

	return ms8607_success(sensor);
}

/// \brief    Reads the temperature and pressure values.
///
/// \details  The results are returned in thousanths as a way to preserve
///           the sensor's resolution while using integer types.
///
/// \param[in] ms8607_sensor* sensor : The sensor to use for measuring
/// \param[out] int32_t* : Thousanths of degC temperature value
/// \param[out] int32_t* : Microbar pressure value (thousanths of millibar)
/// \param[in]  void* caller_context : When this function calls any callbacks
///         from the `tepht_host_functions` structure, this will be passed
///         directly to those callbacks' `caller_context` parameter.
///
/// \return tepht_status : status of MS8607
///       - tepht_status_ok : I2C transfer completed successfully
///       - tepht_status_null_sensor : The pointer provided for the `sensor` parameter was NULL.
///       - tepht_status_null_argument : One or more of the `t` or `p` pointers were NULL.
///       - tepht_status_callback_error : Error occurred within a tepht_host_functions function
///       - tepht_status_eeprom_is_zero : One or more EEPROM coefficients were received as 0, preventing measurement.
///       - tepht_status_eeprom_crc_error : CRC check error on the sensor's EEPROM coefficients
///       - tepht_status_measurement_invalid : EEPROM is OK and I2C transfer completed, but data received was invalid
///
tepht_error_info  ms8607_read_temperature_pressure_int32(ms8607_sensor *sensor, int32_t *t, int32_t *p, void *caller_context)
{
	if ( sensor == NULL )
		return ms8607_error(sensor, tepht_status_null_sensor);

	if ( t == NULL || p == NULL )
		return ms8607_error(sensor, tepht_status_null_argument);

	tepht_error_info  einfo;
	einfo = psensor_read_pressure_and_temperature(sensor, t, p, caller_context);
	if( tepht_is_error(einfo) )
		return einfo;

	return ms8607_success(sensor);
}

static tepht_error_info  ms8607_read_temperature_pressure_int32__virtual(void *self, int32_t *t, int32_t *p,  void *caller_context) {
	return ms8607_read_temperature_pressure_int32(self, t, p, caller_context);
}

/// \brief Reads the temperature, pressure and relative humidity values.
///
/// \param[in] ms8607_sensor* sensor : The sensor to use for measuring
/// \param[out] float* : degC temperature value
/// \param[out] float* : mbar pressure value
/// \param[out] float* : %RH Relative Humidity value
/// \param[in]  void* caller_context : When this function calls any callbacks
///         from the `tepht_host_functions` structure, this will be passed
///         directly to those callbacks' `caller_context` parameter.
///
/// \return tepht_status : status of MS8607
///       - tepht_status_ok : I2C transfer completed successfully
///       - tepht_status_null_sensor : The pointer provided for the `sensor` parameter was NULL.
///       - tepht_status_null_argument : One or more of the `t`, `p`, or `h` pointers were NULL.
///       - tepht_status_callback_error : Error occurred within a tepht_host_functions function
///       - tepht_status_eeprom_is_zero : One or more EEPROM coefficients were received as 0, preventing measurement.
///       - tepht_status_eeprom_crc_error : CRC check error on the sensor's EEPROM coefficients
///       - tepht_status_measurement_invalid : EEPROM is OK and I2C transfer completed, but data received was invalid
///
tepht_error_info  ms8607_read_temperature_pressure_humidity_float32(ms8607_sensor *sensor, float *t, float *p, float *h, void *caller_context)
{
	tepht_error_info  einfo;
	int32_t t_;
	int32_t p_;
	int32_t h_;

	// NULL status of the `sensor` parameter will be checked by `ms8607_read_temperature_pressure_humidity_i32`.

	if ( t == NULL || p == NULL || h == NULL )
		return ms8607_error(sensor, tepht_status_null_argument);

	einfo = ms8607_read_temperature_pressure_humidity_int32(sensor, &t_, &p_, &h_, caller_context);
	*t = ((float)t_) / 1000;
	*p = ((float)p_) / 1000;
	*h = ((float)h_) / 1000;

	return einfo;
}

/// \brief Provide battery status
///
/// \param[in] ms8607_sensor *sensor : Object representing the sensor to get battery status from
/// \param[out] ms8607_battery_status* : Battery status
///                      - ms8607_battery_ok,
///                      - ms8607_battery_low
/// \param[in]  void* caller_context : When this function calls any callbacks
///         from the `tepht_host_functions` structure, this will be passed
///         directly to those callbacks' `caller_context` parameter.
///
/// \return tepht_status : status of MS8607
///       - tepht_status_ok : I2C transfer completed successfully
///       - tepht_status_null_sensor : The pointer provided for the `sensor` parameter was NULL.
///       - tepht_status_callback_error : Error occurred within a tepht_host_functions function
///
tepht_error_info  ms8607_get_battery_status(ms8607_sensor *sensor, enum ms8607_battery_status *bat, void *caller_context)
{
	if ( sensor == NULL )
		return ms8607_error(sensor, tepht_status_null_sensor);

	if ( bat == NULL )
		return ms8607_error(sensor, tepht_status_null_argument);

	tepht_status  status;
	uint8_t reg_value;

	status = hsensor_read_user_register(sensor, &reg_value, caller_context);
	if( status != tepht_status_ok)
		return ms8607_error(sensor, status);

	if( reg_value & HSENSOR_USER_REG_END_OF_BATTERY_VDD_BELOW_2_25V )
		*bat = ms8607_battery_low;
	else
		*bat = ms8607_battery_ok;

	return ms8607_success(sensor);
}

/// \brief Enable heater
///
/// \param[in] ms8607_sensor *sensor : Object representing the sensor whose heater shall be enabled
/// \param[in] void* caller_context : When this function calls any callbacks
///         from the `tepht_host_functions` structure, this will be passed
///         directly to those callbacks' `caller_context` parameter.
///
/// \return tepht_status : status of MS8607
///       - tepht_status_ok : I2C transfer completed successfully
///       - tepht_status_null_sensor : The pointer provided for the `sensor` parameter was NULL.
///       - tepht_status_callback_error : Error occurred within a tepht_host_functions function
///
tepht_error_info  ms8607_enable_heater(ms8607_sensor *sensor, void *caller_context)
{
	if ( sensor == NULL )
		return ms8607_error(sensor, tepht_status_null_sensor);

	tepht_status status;
	uint8_t reg_value;

	status = hsensor_read_user_register(sensor, &reg_value, caller_context);
	if( status != tepht_status_ok )
		return ms8607_error(sensor, status);

	// Clear the resolution bits
	reg_value |= HSENSOR_USER_REG_ONCHIP_HEATER_ENABLE;
	sensor->hsensor_heater_on = true;

	status = hsensor_write_user_register(sensor, reg_value, caller_context);
	if ( status != tepht_status_ok )
		return ms8607_error(sensor, status);

	return ms8607_success(sensor);
}

/// \brief Disable heater
///
/// \param[in] ms8607_sensor *sensor : Object representing the sensor whose heater shall be disabled
/// \param[in] void* caller_context : When this function calls any callbacks
///         from the `tepht_host_functions` structure, this will be passed
///         directly to those callbacks' `caller_context` parameter.
///
/// \return tepht_status : status of MS8607
///       - tepht_status_ok : I2C transfer completed successfully
///       - tepht_status_null_sensor : The pointer provided for the `sensor` parameter was NULL.
///       - tepht_status_callback_error : Error occurred within a tepht_host_functions function
///
tepht_error_info  ms8607_disable_heater(ms8607_sensor *sensor, void *caller_context)
{
	if ( sensor == NULL )
		return ms8607_error(sensor, tepht_status_null_sensor);

	tepht_status status;
	uint8_t reg_value;

	status = hsensor_read_user_register(sensor, &reg_value, caller_context);
	if( status != tepht_status_ok )
		return ms8607_error(sensor, status);

	// Clear the resolution bits
	reg_value &= ~HSENSOR_USER_REG_ONCHIP_HEATER_ENABLE;
	sensor->hsensor_heater_on = false;

	status = hsensor_write_user_register(sensor, reg_value, caller_context);
	if ( status != tepht_status_ok )
		return ms8607_error(sensor, status);

	return ms8607_success(sensor);
}

/// \brief Get heater status
///
/// \param[in] ms8607_sensor *sensor : Object representing the sensor  to get heater status from
/// \param[in] ms8607_heater_status* : Return heater status (above or below 2.5V)
///                      - ms8607_heater_off,
///                      - ms8607_heater_on
/// \param[in] void* caller_context : When this function calls any callbacks
///         from the `tepht_host_functions` structure, this will be passed
///         directly to those callbacks' `caller_context` parameter.
///
/// \return tepht_status : status of MS8607
///       - tepht_status_ok : I2C transfer completed successfully
///       - tepht_status_null_sensor : The pointer provided for the `sensor` parameter was NULL.
///       - tepht_status_callback_error : Error occurred within a tepht_host_functions function
///
tepht_error_info  ms8607_get_heater_status(ms8607_sensor *sensor, enum ms8607_heater_status *heater, void *caller_context)
{
	if ( sensor == NULL )
		return ms8607_error(sensor, tepht_status_null_sensor);

	tepht_status status;
	uint8_t reg_value;

	status = hsensor_read_user_register(sensor, &reg_value, caller_context);
	if( status != tepht_status_ok )
		return ms8607_error(sensor, status);

	// Get the heater enable bit in reg_value
	if( reg_value & HSENSOR_USER_REG_ONCHIP_HEATER_ENABLE)
		*heater = ms8607_heater_on;
	else
		*heater = ms8607_heater_off;

	return ms8607_success(sensor);
}

/******************** Functions from humidity sensor ********************/

/// \brief Check whether humidity sensor is connected
///
/// \param[in] ms8607_sensor *sensor : Object representing the sensor to check for connectivity
/// \param[in] void* caller_context : When this function calls any callbacks
///         from the `tepht_host_functions` structure, this will be passed
///         directly to those callbacks' `caller_context` parameter.
///
/// \return bool : status of humidity sensor
///       - true : Device is present
///       - false : Device is not acknowledging I2C address
///
static bool hsensor_is_connected(ms8607_sensor *sensor, void *caller_context)
{
	assert( sensor != NULL );

	tepht_status status;

	tepht_i2c_controller_packet  transfer = {
		.address     = HSENSOR_ADDR,
		.data_length = 0,
		.data        = NULL,
	};
	/* Do the transfer */
	status = sensor->host_funcs->i2c_controller_write(caller_context, &transfer);
	if( status != tepht_status_ok)
		return false;

	return true;
}

/// \brief Reset the humidity sensor part
///
/// \param[in] ms8607_sensor *sensor : Object representing the sensor to reset
/// \param[in] void* caller_context : When this function calls any callbacks
///         from the `tepht_host_functions` structure, this will be passed
///         directly to those callbacks' `caller_context` parameter.
///
/// \return tepht_status : status of MS8607
///       - tepht_status_ok : I2C transfer completed successfully
///       - tepht_status_callback_error : Error occurred within a tepht_host_functions function
///
static tepht_error_info  hsensor_reset(ms8607_sensor *sensor, void *caller_context)
{
	assert( sensor != NULL );

	tepht_status status;

	status = hsensor_write_command(sensor, HSENSOR_RESET_COMMAND, caller_context);
	if( status != tepht_status_ok )
		return ms8607_error(sensor, status);

	sensor->hsensor_conversion_time = HSENSOR_CONVERSION_TIME_12b;
	sensor->host_funcs->sleep_ms(caller_context, HSENSOR_RESET_TIME);

	return ms8607_success(sensor);
}

/// \brief Writes the Humidity sensor 8-bits command with the value passed
///
/// \param[in] uint8_t : Command value to be written.
///
/// \return tepht_status : status of MS8607
///       - tepht_status_ok : I2C transfer completed successfully
///       - tepht_status_null_sensor : The pointer provided for the `sensor` parameter was NULL.
///       - tepht_status_callback_error : Error occurred within a tepht_host_functions function
///
static tepht_status hsensor_write_command(ms8607_sensor *sensor, uint8_t cmd, void *caller_context)
{
	assert( sensor != NULL );

	uint8_t data[1];

	data[0] = cmd;

	tepht_i2c_controller_packet  transfer = {
		.address     = HSENSOR_ADDR,
		.data_length = 1,
		.data        = data,
	};

	/* Do the transfer */
	return sensor->host_funcs->i2c_controller_write(caller_context, &transfer);
}

/// \brief Writes the Humidity Sensor 8-bits command with the value passed
///        Do not send the STOP bit in the I2C transfer
///
/// \param[in] uint8_t : Command value to be written.
///
/// \return tepht_status : status of MS8607
///       - tepht_status_ok : I2C transfer completed successfully
///       - tepht_status_null_sensor : The pointer provided for the `sensor` parameter was NULL.
///       - tepht_status_callback_error : Error occurred within a tepht_host_functions function
///
tepht_status hsensor_write_command_no_stop(ms8607_sensor *sensor, uint8_t cmd, void *caller_context)
{
	assert( sensor != NULL );

	uint8_t data[1];

	data[0] = cmd;

	tepht_i2c_controller_packet  transfer = {
		.address     = HSENSOR_ADDR,
		.data_length = 1,
		.data        = data,
	};

	/* Do the transfer */
	return sensor->host_funcs->i2c_controller_write_no_stop(caller_context, &transfer);
}

/// \brief Check CRC
///
/// \param[in] uint16_t : variable on which to check CRC
/// \param[in] uint8_t : CRC value
///
/// \return tepht_status : status of MS8607
///       - tepht_status_ok : CRC check is OK
///       - tepht_status_measurement_invalid : CRC check error
///
static tepht_status hsensor_crc_check( uint16_t value, uint8_t crc)
{
	uint32_t polynom = 0x988000; // x^8 + x^5 + x^4 + 1
	uint32_t msb     = 0x800000;
	uint32_t mask    = 0xFF8000;
	uint32_t result  = (uint32_t)value<<8; // Pad with zeros as specified in spec

	while( msb != 0x80 ) {

		// Check if msb of current value is 1 and apply XOR mask
		if( result & msb )
			result = ((result ^ polynom) & mask) | ( result & ~mask);

		// Shift by one
		msb >>= 1;
		mask >>= 1;
		polynom >>=1;
	}
	if( result == crc )
		return tepht_status_ok;
	else
		return tepht_status_measurement_invalid;
}

/// \brief   Reads the MS8607 humidity user register.
///
/// \details The "user" register is described in the datasheet.
///          It is unnecessary to call this function to use the ms8607 sensor,
///          but it could be useful as a diagnostic tool or for better
///          understanding the behavior of the ms8607.
///
/// \param[in] ms8607_sensor *sensor : Object representing the sensor to read the user register from
/// \param[out] uint8_t* : Storage of user register value
/// \param[in] void* caller_context : When this function calls any callbacks
///         from the `tepht_host_functions` structure, this will be passed
///         directly to those callbacks' `caller_context` parameter.
///
/// \return tepht_status : status of MS8607
///       - tepht_status_ok : I2C transfer completed successfully
///       - tepht_status_null_sensor : The pointer provided for the `sensor` parameter was NULL.
///       - tepht_status_callback_error : Error occurred within a tepht_host_functions function
tepht_error_info  ms8607_hsensor_read_user_register(ms8607_sensor *sensor, uint8_t *value, void *caller_context)
{
	if ( sensor == NULL )
		return ms8607_error(sensor, tepht_status_null_sensor);

	if ( value == NULL )
		return ms8607_error(sensor, tepht_status_null_argument);

	tepht_status  status;
	status = hsensor_read_user_register(sensor, value, caller_context);
	if ( status != tepht_status_ok )
		return ms8607_error(sensor, status);

	return ms8607_success(sensor);
}

static tepht_status hsensor_read_user_register(ms8607_sensor *sensor, uint8_t *value, void *caller_context)
{
	assert( sensor != NULL );
	assert( value != NULL );

	tepht_status status;
	uint8_t buffer[1];
	buffer[0] = 0;

	/* Read data */
	tepht_i2c_controller_packet  read_transfer = {
		.address     = HSENSOR_ADDR,
		.data_length = 1,
		.data        = buffer,
	};

	// Send the Read Register Command
	status = hsensor_write_command(sensor, HSENSOR_READ_USER_REG_COMMAND, caller_context);
	if( status != tepht_status_ok )
		return status;

	status = sensor->host_funcs->i2c_controller_read(caller_context, &read_transfer);
	if ( status != tepht_status_ok )
		return status;

	*value = buffer[0];

	return tepht_status_ok;
}

/// \brief Writes the MS8607 humidity user register with value
///        Will read and keep the unreserved bits of the register
///
/// \param[in] ms8607_sensor *sensor : Object representing the sensor to write to
/// \param[in] uint8_t : Register value to be set.
/// \param[in] void* caller_context : When this function calls any callbacks
///         from the `tepht_host_functions` structure, this will be passed
///         directly to those callbacks' `caller_context` parameter.
///
/// \return tepht_status : status of MS8607
///       - tepht_status_ok : I2C transfer completed successfully
///       - tepht_status_callback_error : Error occurred within a tepht_host_functions function
///
static tepht_status hsensor_write_user_register(ms8607_sensor *sensor, uint8_t value, void *caller_context)
{
	assert( sensor != NULL );

	tepht_status status;
	uint8_t reg;
	uint8_t data[2];

	status = hsensor_read_user_register(sensor, &reg, caller_context);
	if( status != tepht_status_ok )
		return status;

	// Clear bits of reg that are not reserved
	reg &= HSENSOR_USER_REG_RESERVED_MASK;
	// Set bits from value that are not reserved
	reg |= (value & ~HSENSOR_USER_REG_RESERVED_MASK);

	data[0] = HSENSOR_WRITE_USER_REG_COMMAND;
	data[1] = reg;

	tepht_i2c_controller_packet  transfer = {
		.address     = HSENSOR_ADDR,
		.data_length = 2,
		.data        = data,
	};

	/* Do the transfer */
	return sensor->host_funcs->i2c_controller_write(caller_context, &transfer);
}

/// \brief Reads the relative humidity ADC value
///
/// \param[in] ms8607_sensor *sensor : Object representing the sensor to read the adc from
/// \param[out] uint16_t* : Relative humidity ADC value.
/// \param[in] void* caller_context : When this function calls any callbacks
///         from the `tepht_host_functions` structure, this will be passed
///         directly to those callbacks' `caller_context` parameter.
///
/// \return tepht_status : status of MS8607
///       - tepht_status_ok : I2C transfer completed successfully
///       - tepht_status_callback_error : Error occurred within a tepht_host_functions function
///       - tepht_status_measurement_invalid : EEPROM is OK and I2C transfer completed, but data received was invalid
///
static tepht_error_info  hsensor_humidity_conversion_and_read_adc(ms8607_sensor *sensor, uint16_t *adc, void *caller_context)
{
	assert( sensor != NULL );
	assert( adc != NULL );

	tepht_status  status = tepht_status_ok;
	uint16_t _adc;
	uint8_t buffer[3];
	uint8_t crc;

	buffer[0] = 0;
	buffer[1] = 0;
	buffer[2] = 0;

	/* Read data */
    tepht_i2c_controller_packet  read_transfer = {
		.address     = HSENSOR_ADDR,
		.data_length = 3,
		.data        = buffer,
	};

	// TODO: There are 3 ways to do this. Implement them all.
	// (1) Hold mode.
	// (2) Delay mode.
	// (3) Polling mode.
	// There might even be a 4th, which is to use Delay Mode, but then retry
	// if we recieve a NACK. It's like a hybrid between 2 and 3, but with
	// the purpose being redundancy. It would be auto-detected if the
	// polling-friendly read function is available.
	// Maybe have another mode: automatic. Hold mode is used preferentially
	// because it's event based. (This doesn't make it universally better, but
	// if the caller has already accepted and committed to a blocking call, then
	// it's probably the best option of the blocking methods.) Then polling,
	// if the poll-friendly-read is available. Last would be to just use
	// the delay. TODO: If I implement this, make sure to NOT recommend
	// that the user put in their own stubs for the dependencies. That would
	// defeat the capability-detection mechanism!
	if( sensor->hsensor_i2c_controller_mode == ms8607_i2c_hold) {
		status = hsensor_write_command_no_stop(sensor, HSENSOR_READ_HUMIDITY_W_HOLD_COMMAND, caller_context);
		if( status != tepht_status_ok)
			return ms8607_error(sensor, status);

		status = sensor->host_funcs->i2c_controller_read(caller_context, &read_transfer);
		if( status != tepht_status_ok)
			return ms8607_error(sensor, status);
	}
	else {
		status = hsensor_write_command(sensor, HSENSOR_READ_HUMIDITY_WO_HOLD_COMMAND, caller_context);
		if( status != tepht_status_ok)
			return ms8607_error(sensor, status);

		// `max_tries` is the number of times we will call `i2c_controller_read`
		// before assuming the worst and giving up (timeout condition).
		//
		// The choice of this number is somewhat arbitrary. As it is, the loop
		// is set up so that it is extremely likely to complete on its first
		// iteration. So setting `max_tries` to something low is reasonable.
		//
		// Rationale:
		// Unfortunately, the Not Acknowledge bit that the ms8607 uses for
		// telling the controller to "wait a little longer please" could also
		// indicate that there is something wrong at a wire-level (ex: the
		// data line is latched into the "Not Acknowledge" state). And if we're
		// really paranoid, we can't expect the ms8607 to ever respond if it
		// is damaged or defective. Most importantly: we should NEVER halt/hang
		// the caller's code. So we give up if we receive NACK for too long.
		//
		int16_t max_tries = 3;
		int16_t tries_remaining = max_tries;
		while(true)
		{
			// delay depending on resolution
			sensor->host_funcs->sleep_ms(caller_context, sensor->hsensor_conversion_time/1000);

			status = sensor->host_funcs->i2c_controller_read(caller_context, &read_transfer);
			if ( status == tepht_status_callback_i2c_nack )
			{
				tries_remaining--;
				if ( tries_remaining > 0 )
					continue; // Not ready yet. That's OK, just wait longer.
				else
					return ms8607_error(sensor, status); // Timeout: we waited too long while receiving only NACK.
			}
			else
			if ( status == tepht_status_ok )
				break;
			else
			// if( status != tepht_status_ok)
				return ms8607_error(sensor, status); // Other errors indicate something is *actually* wrong.
		}
	}

	_adc = (buffer[0] << 8) | buffer[1];
	crc = buffer[2];

	// Compute CRC
	// This is where we can get the `tepht_status_measurement_invalid` error.
	// Notably, this is not EEPROM/coefficient related. We're checking the CRC of the measurement itself.
	status = hsensor_crc_check(_adc,crc);
	if( status != tepht_status_ok)
		return ms8607_error(sensor, status);

	*adc = _adc;

	return ms8607_success(sensor);
}

/// \brief Reads the relative humidity value.
///
/// \param[in] ms8607_sensor *sensor : Object representing the sensor to read relative humidity from
/// \param[out] int32_t* : %RH Relative Humidity value
/// \param[in] void* caller_context : When this function calls any callbacks
///         from the `tepht_host_functions` structure, this will be passed
///         directly to those callbacks' `caller_context` parameter.
///
/// \return tepht_status : status of MS8607
///       - tepht_status_ok : I2C transfer completed successfully
///       - tepht_status_callback_error : Error occurred within a tepht_host_functions function
///       - tepht_status_measurement_invalid : EEPROM is OK and I2C transfer completed, but data received was invalid
///
static tepht_error_info  hsensor_read_relative_humidity(ms8607_sensor *sensor, int32_t *humidity, void *caller_context)
{
	assert( sensor != NULL );
	assert( humidity != NULL );

	tepht_error_info  einfo;
	uint16_t adc;

	einfo = hsensor_humidity_conversion_and_read_adc(sensor, &adc, caller_context);
	if( tepht_is_error(einfo) )
		return einfo;

	// Perform conversion function
	//*humidity = (float)adc * HUMIDITY_COEFF_MUL / (1UL<<16) + HUMIDITY_COEFF_ADD;
	*humidity = 1000 * adc * HUMIDITY_COEFF_MUL / (1UL<<16) + HUMIDITY_COEFF_ADD;

	return ms8607_success(sensor);
}

#if 0
// TODO: remove?
/*static*/ tepht_status hsensor_poll_relative_humidity(ms8607_sensor *sensor, float *humidity, void *caller_context)
{
	assert( sensor != NULL );
	assert( humidity != NULL );

	tepht_status  status;
	uint16_t adc;
	uint8_t buffer[3];
	uint8_t crc;

	buffer[0] = 0;
	buffer[1] = 0;
	buffer[2] = 0;

	/* Read data */
    tepht_i2c_controller_packet  read_transfer = {
		.address     = HSENSOR_ADDR,
		.data_length = 3,
		.data        = buffer,
	};

	// TODO: Move success-path stuff into separate function.
	// This would allow the compiler to optimize inlining/etc for i-cache salvation.
	status = sensor->host_funcs->i2c_controller_read(caller_context, &read_transfer);
	if ( status == tepht_status_callback_i2c_nack )
		return tepht_status_waiting; // Not ready yet. That's OK, just wait longer. TODO: Timeout calculations?
	else
	if( status != tepht_status_ok)
		return status; // Other errors indicate something is *actually* wrong.

	adc = (buffer[0] << 8) | buffer[1];
	crc = buffer[2];

	// Compute CRC
	// This is where we can get the `tepht_status_measurement_invalid` error.
	// Notably, this is not EEPROM/coefficient related. We're checking the CRC of the measurement itself.
	status = hsensor_crc_check(adc,crc);
	if( status != tepht_status_ok)
		return status;

	// Perform conversion function
	*humidity = (float)adc * HUMIDITY_COEFF_MUL / (1UL<<16) + HUMIDITY_COEFF_ADD;

	return status;
}
#endif

/// \brief Returns result of compensated humidity
///        Note : This function shall only be used when the heater is OFF. It will return an error otherwise.
///
/// \param[in] ms8607_sensor *sensor : Object representing the sensor that earlier obtained the temperature and uncompensated %RH
/// \param[in] float - Actual temperature measured (degC)
/// \param[in] float - Actual relative humidity measured (%RH)
/// \param[out] float *- Compensated humidity (%RH).
/// \param[in] void* caller_context : When this function calls any callbacks
///         from the `tepht_host_functions` structure, this will be passed
///         directly to those callbacks' `caller_context` parameter.
///
/// \return tepht_status : status of MS8607
///       - tepht_status_ok : I2C transfer completed successfully
///       - tepht_status_null_sensor : The pointer provided for the `sensor` parameter was NULL.
///       - tepht_status_heater_on_error : Cannot compute compensated humidity because heater is on
///
tepht_error_info  ms8607_get_compensated_humidity(ms8607_sensor *sensor, float temperature, float relative_humidity, float *compensated_humidity)
{
	if ( sensor == NULL )
		return ms8607_error(sensor, tepht_status_null_sensor);

	if ( compensated_humidity == NULL )
		return ms8607_error(sensor, tepht_status_null_argument);

	if( sensor->hsensor_heater_on )
		return ms8607_error(sensor, tepht_status_heater_on_error);

	*compensated_humidity = ( relative_humidity + (25 - temperature) * HSENSOR_TEMPERATURE_COEFFICIENT);

	return ms8607_success(sensor);
}

/// \brief Returns the computed dew point
///        Note : This function shall only be used when the heater is OFF. It will return an error otherwise.
///
/// \param[in] ms8607_sensor *sensor : Object representing the sensor that earlier obtained the temperature and uncompensated %RH
/// \param[in] float - Actual temperature measured (degC)
/// \param[in] float - Actual relative humidity measured (%RH)
/// \param[out] float *- Dew point temperature (DegC).
/// \param[in] void* caller_context : When this function calls any callbacks
///         from the `tepht_host_functions` structure, this will be passed
///         directly to those callbacks' `caller_context` parameter.
///
/// \return tepht_status : status of MS8607
///       - tepht_status_ok : I2C transfer completed successfully
///       - tepht_status_null_sensor : The pointer provided for the `sensor` parameter was NULL.
///       - tepht_status_heater_on_error : Cannot compute compensated humidity because heater is on
///
tepht_error_info  ms8607_get_dew_point(ms8607_sensor *sensor, float temperature, float relative_humidity, float *dew_point)
{
	if ( sensor == NULL )
		return ms8607_error(sensor, tepht_status_null_sensor);

	if ( dew_point == NULL )
		return ms8607_error(sensor, tepht_status_null_argument);

	double partial_pressure;

	if( sensor->hsensor_heater_on )
		return ms8607_error(sensor, tepht_status_heater_on_error);

	// Missing power of 10
	partial_pressure = pow( 10, HSENSOR_CONSTANT_A - HSENSOR_CONSTANT_B / (temperature + HSENSOR_CONSTANT_C) );

	*dew_point =  - HSENSOR_CONSTANT_B / (log10( relative_humidity * partial_pressure / 100) - HSENSOR_CONSTANT_A) - HSENSOR_CONSTANT_C;

	return ms8607_success(sensor);
}

/******************** Functions from Pressure sensor ********************/

/// \brief Check whether Pressure sensor device is connected
///
/// \param[in] ms8607_sensor *sensor : Object representing the sensor to check for connectivity
/// \param[in] void* caller_context : When this function calls any callbacks
///         from the `tepht_host_functions` structure, this will be passed
///         directly to those callbacks' `caller_context` parameter.
///
/// \return bool : status of Pressure sensor
///       - true : Device is present
///       - false : Device is not acknowledging I2C address
///
static bool psensor_is_connected(ms8607_sensor *sensor, void *caller_context)
{
	assert( sensor != NULL );

	tepht_status status;

	tepht_i2c_controller_packet  transfer = {
		.address     = PSENSOR_ADDR,
		.data_length = 0,
		.data        = NULL,
	};

	/* Do the transfer */
	status = sensor->host_funcs->i2c_controller_write(caller_context, &transfer);
	if( status != tepht_status_ok)
		return false;

	return true;
}

/// \brief Reset the Pressure sensor part
///
/// \param[in] ms8607_sensor *sensor : Object representing the sensor to reset
/// \param[in] void* caller_context : When this function calls any callbacks
///         from the `tepht_host_functions` structure, this will be passed
///         directly to those callbacks' `caller_context` parameter.
///
/// \return tepht_status : status of MS8607
///       - tepht_status_ok : I2C transfer completed successfully
///       - tepht_status_callback_error : Error occurred within a tepht_host_functions function
///
static tepht_error_info  psensor_reset(ms8607_sensor *sensor, void *caller_context)
{
	assert( sensor != NULL );
	
	tepht_status status;
	status = psensor_write_command(sensor, PSENSOR_RESET_COMMAND, caller_context);
	if ( status != tepht_status_ok )
		return ms8607_error(sensor, status);

	return ms8607_success(sensor);
}

///
/// \brief Writes the Pressure Sensor 8-bits command with the value passed
///
/// \param[in] ms8607_sensor *sensor : Object representing the sensor to send the command to
/// \param[in] uint8_t : Command value to be written.
/// \param[in] void* caller_context : When this function calls any callbacks
///         from the `tepht_host_functions` structure, this will be passed
///         directly to those callbacks' `caller_context` parameter.
///
/// \return tepht_status : status of MS8607
///       - tepht_status_ok : I2C transfer completed successfully
///       - tepht_status_callback_error : Error occurred within a tepht_host_functions function
///
static tepht_status psensor_write_command(ms8607_sensor *sensor, uint8_t cmd, void *caller_context)
{
	assert( sensor != NULL );
	uint8_t data[1];

	data[0] = cmd;

	tepht_i2c_controller_packet  transfer = {
		.address     = PSENSOR_ADDR,
		.data_length = 1,
		.data        = data,
	};

	/* Do the transfer */
	return sensor->host_funcs->i2c_controller_write(caller_context, &transfer);
}

/// \brief Set pressure ADC resolution.
///
/// \param[in] ms8607_sensor *sensor : Object representing the sensor to configure
/// \param[in] ms8607_pressure_resolution : Resolution requested
/// \param[in] void* caller_context : When this function calls any callbacks
///         from the `tepht_host_functions` structure, this will be passed
///         directly to those callbacks' `caller_context` parameter.
///         (As of this writing, this function does not do any I2C I/O and
///         does not call any host functions, so `caller_context` is unused here,
///         but nonetheless provided for sake of future-proofing.)
///
/// \return tepht_status : status of MS8607
///       - tepht_status_ok
///       - tepht_status_null_sensor : The pointer provided for the `sensor` parameter was NULL.
///
///
tepht_error_info  ms8607_set_pressure_resolution(ms8607_sensor *sensor, enum ms8607_pressure_resolution res, void *caller_context)
{
	(void)caller_context;
	if ( sensor == NULL )
		return ms8607_error(sensor, tepht_status_null_sensor);

	sensor->psensor_resolution_osr = res;
	return ms8607_success(sensor);
}

/// \brief Reads the psensor EEPROM coefficient stored at address provided.
///
/// \param[in] ms8607_sensor *sensor : Object representing the sensor to retrieve EEPROM coefficients from
/// \param[in] uint8_t : Address of coefficient in EEPROM
/// \param[out] uint16_t* : Value read in EEPROM
/// \param[in] void* caller_context : When this function calls any callbacks
///         from the `tepht_host_functions` structure, this will be passed
///         directly to those callbacks' `caller_context` parameter.
///
/// \return tepht_status : status of MS8607
///       - tepht_status_ok : All operations completed successfully
///       - tepht_status_callback_error : Error occurred within a tepht_host_functions function
///       - tepht_status_eeprom_is_zero : One or more EEPROM coefficients were received as 0, preventing measurement.
///
static tepht_error_info  psensor_read_eeprom_coeff(ms8607_sensor *sensor, uint8_t command, uint16_t *coeff, void *caller_context)
{
	assert( sensor != NULL );
	assert( coeff != NULL );

	tepht_status status;
	uint8_t buffer[2];

	buffer[0] = 0;
	buffer[1] = 0;

	/* Read data */
	tepht_i2c_controller_packet  read_transfer = {
		.address     = PSENSOR_ADDR,
		.data_length = 2,
		.data        = buffer,
	};

	// Send the conversion command
	status = psensor_write_command(sensor, command, caller_context);
	if(status != tepht_status_ok)
		return ms8607_error(sensor, status);

	status = sensor->host_funcs->i2c_controller_read(caller_context, &read_transfer);
	if(status != tepht_status_ok)
		return ms8607_error(sensor, status);

	*coeff = (buffer[0] << 8) | buffer[1];

	if (*coeff == 0)
		return ms8607_error(sensor, tepht_status_eeprom_is_zero);

	return ms8607_success(sensor);
}

/// \brief Reads the ms8607 EEPROM coefficients to store them for computation.
///
/// \param[in] ms8607_sensor *sensor : Object representing the sensor to retrieve EEPROM coefficients from
/// \param[in] void* caller_context : When this function calls any callbacks
///         from the `tepht_host_functions` structure, this will be passed
///         directly to those callbacks' `caller_context` parameter.
///
/// \return tepht_status : status of MS8607
///       - tepht_status_ok : All operations completed successfully
///       - tepht_status_callback_error : Error occurred within a tepht_host_functions function
///       - tepht_status_eeprom_is_zero : One or more EEPROM coefficients were received as 0, preventing measurement.
///       - tepht_status_eeprom_crc_error : CRC check error on the sensor's EEPROM coefficients
///
static tepht_error_info  psensor_read_eeprom(ms8607_sensor *sensor, void *caller_context)
{
	assert( sensor != NULL );
	tepht_error_info  einfo;
	uint8_t i;

	for( i=0 ; i< MS8607_COEFFICIENT_COUNT ; i++)
	{
		einfo = psensor_read_eeprom_coeff(
			sensor, PROM_ADDRESS_READ_ADDRESS_0 + i*2, sensor->eeprom_coeff+i, caller_context);
		if( tepht_is_error(einfo) )
			return einfo;
	}

	if( !psensor_crc_check( sensor->eeprom_coeff, (sensor->eeprom_coeff[CRC_INDEX] & 0xF000)>>12 ) )
		return ms8607_error(sensor, tepht_status_eeprom_crc_error);

	sensor->psensor_coeff_read = true;

	return ms8607_success(sensor);
}

/// \brief Triggers conversion and reading of ADC value
///
/// \param[in] ms8607_sensor *sensor : Object representing the sensor to read the adc from
/// \param[in] uint8_t : Command used for conversion (will determine Temperature vs Pressure and osr)
/// \param[out] uint32_t* : ADC value.
/// \param[in] void* caller_context : When this function calls any callbacks
///         from the `tepht_host_functions` structure, this will be passed
///         directly to those callbacks' `caller_context` parameter.
///
/// \return tepht_status : status of MS8607
///       - tepht_status_ok : All operations completed successfully
///       - tepht_status_callback_error : Error occurred within a tepht_host_functions function
///       - tepht_status_measurement_invalid : I2C transfer(s) completed, but data received was invalid
///
static tepht_error_info  psensor_conversion_and_read_adc(ms8607_sensor *sensor, uint8_t cmd, uint32_t *adc, void *caller_context)
{
	assert( sensor != NULL );
	assert( adc != NULL );

	tepht_status status;
	uint8_t buffer[3];

	buffer[0] = 0;
	buffer[1] = 0;
	buffer[2] = 0;

	/* Read data */
    tepht_i2c_controller_packet  read_transfer = {
		.address     = PSENSOR_ADDR,
		.data_length = 3,
		.data        = buffer,
	};

	// TODO: implement?
#if 0
	status = psensor_write_command(sensor, cmd, caller_context);
	if( status != tepht_status_ok)
		return status;

	while ( microsecs() < psensor_conversion_time[ (cmd & PSENSOR_CONVERSION_OSR_MASK)/2 ] )
		spin();

	// Send the read command
	status = psensor_write_command(sensor, PSENSOR_READ_ADC, caller_context);
	if( status != tepht_status_ok)
		return status;

    status = sensor->host_funcs->i2c_controller_read(caller_context, &read_transfer);
	if( status != tepht_status_ok )
		return status;

	// TODO: If failure, should probably send reset sequence?
	// API seems to leak problems, because datasheet is saying that all bets are
	// off if we talk to the thing while it's converting. But... what if we didn't
	// wait long enough? How do we recover? It might not be enough to say,
	// "It didn't work" and move on with life, because now we might get bogus
	// data from the thing. It'd be a good idea to reset and reload EEPROM at
	// that point.
	// TODO: 1-deep command queueing: the driver always knows how long it will
	//   be before it can perform another action, and will not allow other
	//   actions to be performed. Probably make sure the state-machine is
	//   simplified compared to the datasheet.
#endif
	status = psensor_write_command(sensor, cmd, caller_context);
	// 20ms wait for conversion
	sensor->host_funcs->sleep_ms(caller_context, psensor_conversion_time[ (cmd & PSENSOR_CONVERSION_OSR_MASK)/2 ]/1000);
	if( status != tepht_status_ok)
		return ms8607_error(sensor, status);

	// Send the read command
	status = psensor_write_command(sensor, PSENSOR_READ_ADC, caller_context);
	if( status != tepht_status_ok)
		return ms8607_error(sensor, status);

    status = sensor->host_funcs->i2c_controller_read(caller_context, &read_transfer);
	if( status != tepht_status_ok )
		return ms8607_error(sensor, status);

	*adc = ((uint32_t)buffer[0] << 16) | ((uint32_t)buffer[1] << 8) | buffer[2];

	return ms8607_success(sensor);
}

/// \brief Compute temperature and pressure
///
/// \param[in] ms8607_sensor *sensor : Object representing the sensor to read temperature and pressure from
/// \param[out] float* : Celsius Degree temperature value
/// \param[out] float* : mbar pressure value
/// \param[in] void* caller_context : When this function calls any callbacks
///         from the `tepht_host_functions` structure, this will be passed
///         directly to those callbacks' `caller_context` parameter.
///
/// \return tepht_status : status of MS8607
///       - tepht_status_ok : All operations completed successfully
///       - tepht_status_callback_error : Error occurred within a tepht_host_functions function
///       - tepht_status_eeprom_is_zero : One or more EEPROM coefficients were received as 0, preventing measurement.
///       - tepht_status_eeprom_crc_error : CRC check error on the sensor's EEPROM coefficients
///       - tepht_status_measurement_invalid : EEPROM is OK and I2C transfer completed, but data received was invalid
///
static tepht_error_info  psensor_read_pressure_and_temperature(ms8607_sensor *sensor, int32_t *temperature, int32_t *pressure, void *caller_context)
{
	assert( sensor      != NULL );
	assert( temperature != NULL );
	assert( pressure    != NULL );

	tepht_error_info  einfo;
	uint32_t adc_temperature, adc_pressure;
	int32_t dT, TEMP;
	int64_t OFF, SENS, P, T2, OFF2, SENS2;
	uint8_t cmd;

	// If first time adc is requested, get EEPROM coefficients
	einfo = ms8607_success(sensor);
	if( sensor->psensor_coeff_read == false )
		einfo = psensor_read_eeprom(sensor, caller_context);
	if( tepht_is_error(einfo) )
		return einfo;

	// First read temperature
	cmd = sensor->psensor_resolution_osr*2;
	cmd |= PSENSOR_START_TEMPERATURE_ADC_CONVERSION;
	einfo = psensor_conversion_and_read_adc(sensor, cmd, &adc_temperature, caller_context);
	if( tepht_is_error(einfo) )
		return einfo;

	// Now read pressure
	cmd = sensor->psensor_resolution_osr*2;
	cmd |= PSENSOR_START_PRESSURE_ADC_CONVERSION;
	einfo = psensor_conversion_and_read_adc(sensor, cmd, &adc_pressure, caller_context);
	if( tepht_is_error(einfo) )
		return einfo;

	if (adc_temperature == 0 || adc_pressure == 0)
		return ms8607_error(sensor, tepht_status_measurement_invalid);

	// Difference between actual and reference temperature = D2 - Tref
	dT = (int32_t)adc_temperature - ( (int32_t)sensor->eeprom_coeff[REFERENCE_TEMPERATURE_INDEX] <<8 );

	// Actual temperature = 2000 + dT * TEMPSENS
	TEMP = 2000 + ((int64_t)dT * (int64_t)sensor->eeprom_coeff[TEMP_COEFF_OF_TEMPERATURE_INDEX] >> 23) ;

	// Second order temperature compensation
	if( TEMP < 2000 )
	{
		T2 = ( 3 * ( (int64_t)dT  * (int64_t)dT  ) ) >> 33;
		OFF2 = 61 * ((int64_t)TEMP - 2000) * ((int64_t)TEMP - 2000) / 16 ;
		SENS2 = 29 * ((int64_t)TEMP - 2000) * ((int64_t)TEMP - 2000) / 16 ;

		if( TEMP < -1500 )
		{
			OFF2 += 17 * ((int64_t)TEMP + 1500) * ((int64_t)TEMP + 1500) ;
			SENS2 += 9 * ((int64_t)TEMP + 1500) * ((int64_t)TEMP + 1500) ;
		}
	}
	else
	{
		T2 = ( 5 * ( (int64_t)dT  * (int64_t)dT  ) ) >> 38;
		OFF2 = 0 ;
		SENS2 = 0 ;
	}

	// OFF = OFF_T1 + TCO * dT
	OFF = ( (int64_t)(sensor->eeprom_coeff[PRESSURE_OFFSET_INDEX]) << 17 )
		+ ( ( (int64_t)(sensor->eeprom_coeff[TEMP_COEFF_OF_PRESSURE_OFFSET_INDEX]) * dT ) >> 6 ) ;
	OFF -= OFF2 ;

	// Sensitivity at actual temperature = SENS_T1 + TCS * dT
	SENS = ( (int64_t)sensor->eeprom_coeff[PRESSURE_SENSITIVITY_INDEX] << 16 )
		+ ( ((int64_t)sensor->eeprom_coeff[TEMP_COEFF_OF_PRESSURE_SENSITIVITY_INDEX] * dT) >> 7 ) ;
	SENS -= SENS2 ;

	// Temperature compensated pressure = D1 * SENS - OFF
	P = ( ( (adc_pressure * SENS) >> 21 ) - OFF ) >> 15 ;

	//*temperature = ( (float)TEMP - T2 ) / 100;
	//*pressure = (float)P / 100;

	*temperature = 10*( TEMP - T2 );
	*pressure = 10*P;

	return ms8607_success(sensor);
}

#if 0
// TODO: remove?
// (I was trying to write a polling interface for the driver.
// It has merits, at least from the perspective of an ideal generalized
// sensor API or Hardware Abstraction Layer (HAL).
// I don't have time to write said HAL right now, so I'm going to avoid it.
// The other potential merit was that polling the sensor for NACK responses
// might yield values faster than waiting for a constant amount of time
// between request and read. In reality, I don't think it helped much,
// and it might have even been a hindrance, since the polling frequency
// was severely limited and could easily overshoot, IIRC.
// It'd still be good at an API level, but the driver would need a state-machine
// (probably), and it would want to use the well-tuned time-constants as
// it's first guess for when to actually communicate with the sensor and
// ask "are you done yet?". Then you'd get a bunch of delicious non-blocking
// functions that could operate in parallel, which is great, even if it
// doesn't make individual readings happen any faster. And it'd dovetail
// into a nice API that could be used for other sensors. Maybe someday.)
//
// Other important note: this was hacked together quickly so I could run
// experiments, and so it has global state outside of the ms8607_sensor struct.
// That's bad and mean and shouldn't be part of the experience when using
// this driver, so please leave this reference code commented out unless
// you are going to finish my original visions, or at least remove the
// global state and deduplicate the contents of these functions from
// the places I copied them from. (The global state makes it impossible
// to, without spurious unexpected behavior, use more than one ms8607 sensor
// at the same time.)
//
static uint64_t ms8607_time_of_last_temperature_request = 0;

tepht_status psensor_request_temperature(ms8607_sensor *sensor, uint64_t microsecs, void *caller_context)
{
	assert( sensor != NULL );

	tepht_status status = tepht_status_ok;
	uint8_t cmd;

	cmd = sensor->psensor_resolution_osr*2;
	cmd |= PSENSOR_START_TEMPERATURE_ADC_CONVERSION;

	status = psensor_write_command(sensor, cmd, caller_context);
	if( status != tepht_status_ok)
		return status;

	ms8607_time_of_last_temperature_request = microsecs;

	return status;
}

tepht_status psensor_poll_raw_temperature(ms8607_sensor *sensor, uint64_t microsecs, uint32_t *temperature, void *caller_context)
{
	assert( sensor != NULL );
	assert( temperature != NULL );

	tepht_status status = tepht_status_ok;
	uint8_t cmd;

	cmd = sensor->psensor_resolution_osr*2;
	cmd |= PSENSOR_START_TEMPERATURE_ADC_CONVERSION;

	uint64_t delay_us = psensor_conversion_time[ (cmd & PSENSOR_CONVERSION_OSR_MASK)/2 ];
	if ( ((int64_t)microsecs - ms8607_time_of_last_temperature_request) < delay_us )
		return tepht_status_waiting;

	uint8_t buffer[3];

	buffer[0] = 0;
	buffer[1] = 0;
	buffer[2] = 0;

	/* Read data */
    tepht_i2c_controller_packet  read_transfer = {
		.address     = PSENSOR_ADDR,
		.data_length = 3,
		.data        = buffer,
	};

	// Send the read command
	status = psensor_write_command(sensor, PSENSOR_READ_ADC, caller_context);
	if( status != tepht_status_ok)
		return status;

    status = sensor->host_funcs->i2c_controller_read(caller_context, &read_transfer);
	if( status != tepht_status_ok )
		return status;

	*temperature = ((uint32_t)buffer[0] << 16) | ((uint32_t)buffer[1] << 8) | buffer[2];

	return status;
}

static int64_t ms8607_time_of_last_pressure_request = 0;

tepht_status psensor_request_pressure(ms8607_sensor *sensor, uint64_t microsecs, void *caller_context)
{
	assert( sensor != NULL );

	tepht_status status = tepht_status_ok;
	uint8_t cmd;

	cmd = sensor->psensor_resolution_osr*2;
	cmd |= PSENSOR_START_PRESSURE_ADC_CONVERSION;

	status = psensor_write_command(sensor, cmd, caller_context);
	if( status != tepht_status_ok)
		return status;

	ms8607_time_of_last_pressure_request = microsecs;

	return status;
}

tepht_status psensor_poll_raw_pressure(ms8607_sensor *sensor, uint64_t microsecs, uint32_t *pressure, void *caller_context)
{
	assert( sensor != NULL );
	assert( pressure != NULL );

	tepht_status status = tepht_status_ok;
	uint8_t cmd;

	cmd = sensor->psensor_resolution_osr*2;
	cmd |= PSENSOR_START_PRESSURE_ADC_CONVERSION;

	int64_t delay_us = psensor_conversion_time[ (cmd & PSENSOR_CONVERSION_OSR_MASK)/2 ];
	if ( ((int64_t)microsecs - ms8607_time_of_last_pressure_request) < (int64_t)delay_us )
		return tepht_status_waiting;

	uint8_t buffer[3];

	buffer[0] = 0;
	buffer[1] = 0;
	buffer[2] = 0;

	/* Read data */
    tepht_i2c_controller_packet  read_transfer = {
		.address     = PSENSOR_ADDR,
		.data_length = 3,
		.data        = buffer,
	};

	// Send the read command
	status = psensor_write_command(sensor, PSENSOR_READ_ADC, caller_context);
	if( status != tepht_status_ok)
		return status;

    status = sensor->host_funcs->i2c_controller_read(caller_context, &read_transfer);
	if( status != tepht_status_ok )
		return status;

	*pressure = ((uint32_t)buffer[0] << 16) | ((uint32_t)buffer[1] << 8) | buffer[2];

	return status;
}
#endif

/// \brief CRC check
///
/// \param[in] uint16_t *: List of EEPROM coefficients
/// \param[in] uint8_t : crc to compare
///
/// \return bool : TRUE if CRC is OK, FALSE if KO
///
static bool psensor_crc_check (uint16_t *n_prom, uint8_t crc)
{
	assert( n_prom != NULL );
	uint8_t cnt, n_bit;
	uint16_t n_rem, crc_read;

	n_rem = 0x00;
	crc_read = n_prom[0];
	n_prom[MS8607_COEFFICIENT_COUNT] = 0;
	n_prom[0] = (0x0FFF & (n_prom[0]));    // Clear the CRC byte

	for( cnt = 0 ; cnt < (MS8607_COEFFICIENT_COUNT+1)*2 ; cnt++ ) {

		// Get next byte
		if (cnt%2 == 1)
			n_rem ^=  n_prom[cnt>>1] & 0x00FF ;
		else
			n_rem ^=  n_prom[cnt>>1]>>8 ;

		for( n_bit = 8; n_bit > 0 ; n_bit-- ) {

			if( n_rem & 0x8000 )
				n_rem = (n_rem << 1) ^ 0x3000;
			else
				n_rem <<= 1;
		}
	}
	n_rem >>= 12;
	n_prom[0] = crc_read;

	return  ( n_rem == crc );
}

static tepht_pt_sensor_vtable  ms8607_pt_sensor_vtable =
	{
		.is_connected                    = ms8607_is_connected__virtual,
		.reset                           = ms8607_reset__virtual,
		.read_temperature_pressure_int32 = ms8607_read_temperature_pressure_int32__virtual
	};

tepht_pt_sensor   ms8607_to_pt_sensor_interface(ms8607_sensor *sensor)
{
	tepht_pt_sensor  generic;
	assert(sensor != NULL);
	generic.self   = sensor;
	generic.vtable = &ms8607_pt_sensor_vtable;
	return generic;
}

#ifdef __cplusplus
}
#endif
