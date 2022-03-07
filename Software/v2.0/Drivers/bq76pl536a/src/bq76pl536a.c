#include "bq76pl536a.h"
#include "crcLUT.h"
#include "main.h"

/**
 * @func calculate_ott
 * @brief calculates the ott_config value given the delay time in a uint16_t
 * 		  variable
 * @params[in] delay_time: uint16_t variable that holds the desired delay time
 * 			   in miliseconds
 * @return uint8_t ott value
 */
static uint8_t calculate_ott(uint16_t delay_time)
{
	delay_time = (delay_time > MAX_OTT_DELAY) ? MAX_OTT_DELAY : delay_time;
	return delay_time / OTT_LSB_VALUE;
}

/**
 * @func  calculate_ot
 * @brief calculates the ot_config temperature threshold value given the desired
 * 		  temperature threshold
 * @params[in] temperature_threshold: uint8_t variable that holds the desired
 * 			   temperature threshold value
 * @note the temperature has to be multiple of 5
 * @return uint8_t ot value
 */

static uint8_t calculate_ot(uint8_t temperature_threshold)
{
	if (temperature_threshold > MAX_CAL_TEMP)
	{
		temperature_threshold = MAX_CAL_TEMP;
	}
	if (temperature_threshold < MIN_CAL_TEMP)
	{
		temperature_threshold = MIN_CAL_TEMP;
	}
	return (1 + ((temperature_threshold - MIN_CAL_TEMP) / CAL_TEMP_LSB));
}

/**
 * @func  calculate_cuv
 * @brief calculates the cuv_config voltage threshold value given the desired
 * 		  floating undervoltage value
 * @params[in] undervoltage_value: float32_t variable that holds the desired
 * 			   undervoltage value detection
 * @return uint8_t cuv register value
 */
static uint8_t calculate_cuv(float32_t undervoltage_value)
{
	if (undervoltage_value < MIN_CUV_VALUE)
	{
		undervoltage_value = MIN_CUV_VALUE;
	}
	if (undervoltage_value > MAX_CUV_VALUE)
	{
		undervoltage_value = MAX_CUV_VALUE;
	}
	return (uint8_t) ((undervoltage_value - MIN_CUV_VALUE) / CUV_LSB_VALUE);
}

/**
 * @func  calculate_cov
 * @brief calculates the cov_config voltage threshold value given the desired 
 * 		  floating overvoltage value
 * @params[in] overvoltage_value: float32_t variable that holds the desired
 * 			   overvoltage value detection
 * @return uint8_t cov register value
 */
static uint8_t calculate_cov(float32_t overvoltage_value)
{
	if (overvoltage_value < MIN_COV_VALUE)
	{
		overvoltage_value = MIN_COV_VALUE;
	}
	if (overvoltage_value > MAX_COV_VALUE)
	{
		overvoltage_value = MAX_COV_VALUE;
	}
	return (uint8_t) ((overvoltage_value - MIN_COV_VALUE) / COV_LSB_VALUE);
}

/**
 * @func  calculate_covt
 * @brief calculates the covt_config delay time given the desired uint16_t 
 * 		  desired time delay
 * @params[in] delay_time: uint16_t variable that holds the desired
 * 			   delay_time
 * @return uint8_t covt register value
 */
static uint8_t calculate_delay(uint16_t delay_time)
{
	if (delay_time > MAX_DELAY_VALUE)
	{
		return MAX_DELAY_VALUE / DELAY_LSB_VALUE;
	}
	return delay_time / DELAY_LSB_VALUE;
}

/**
 * @func init_write_packet
 * @brief It generates a BQ76_write_packet_format struct given the device 
 * 	  	  address, register address and register data
 * @params[in] device_address: uint8_t bq76 device address
 * @params[in] reg_address: uint8_t register address to write
 * @params[in] reg_data: uint8_t packet data to write
 * @return BQ76_write_packet_format
 */
static struct BQ76_write_packet_format init_write_packet(uint8_t device_address, uint8_t reg_address, uint8_t reg_data)
{
	struct BQ76_write_packet_format packet;
	packet.device_address = (device_address << 1) | 0x01;
	packet.reg_address = reg_address;
	packet.reg_data = reg_data;
	packet.crc = calculate_crc((uint8_t*) &packet, sizeof(struct BQ76_write_packet_format) - 1, CRC_SMBUS_LUT);
	return packet;
}

/**
 * @func BQ76_read_packet_format
 * @brief It generates a BQ76_read_packet_format struct given the device 
 * 	  	  address, start register address and the length of the packet 
 * @params[in] device_address: uint8_t bq76 device address
 * @params[in] start_reg_address: uint8_t start register address to read
 * @params[in] read_length: uint8_t length of data to read
 * @return BQ76_read_packet_format
 */
static struct BQ76_read_packet_format init_read_packet(uint8_t device_address, uint8_t start_reg_addr, uint8_t read_length)
{
	struct BQ76_read_packet_format packet;
	packet.device_address = device_address << 1;
	packet.start_reg_address = start_reg_addr;
	packet.read_length = read_length;
	return packet;
}

/**
 * @func writespi
 * @brief Sends a write command from the SPI interface to the BQ76PL536 
 * 		  device. The user of this library is responsible of defining this 
 * 		  function. As an example, we have only defined functionality for 
 * 		  STM32F407xx.
 * @params[in] spi_address: BQ76 device address
 * @params[in] reg_address: register address to write
 * @params[in] reg_data: data to write
 * @return BQ76_status [BQ76_OK|BQ76_SPI_TRANSMISSION_ERROR]
 */
static enum BQ76_status writespi(uint8_t spi_address, uint8_t reg_address, uint8_t reg_data)
{
	if (reg_address >= FUNCTION_CONFIG_REG && reg_address <= USER4_REG)
	{
		if (writespi(spi_address, SHADOW_CONTROL_REG, ENABLE_REG_3) != BQ76_OK)
		{
			return BQ76_SPI_TRANSMISSION_ERROR;
		}
	}
	struct BQ76_write_packet_format packet = init_write_packet(spi_address, reg_address, reg_data);
	while (HAL_SPI_GetState(&hspi2) != HAL_SPI_STATE_READY);

	//Actual SPI Transmit

	HAL_GPIO_WritePin(BQ_NSS_GPIO_Port, BQ_NSS_Pin, GPIO_PIN_RESET);
	if (HAL_SPI_Transmit(&hspi2, (uint8_t*) &packet, BQ76_TX_BUFF_SIZE, BQ76_TIMEOUT) != HAL_OK)
	{
		HAL_GPIO_WritePin(BQ_NSS_GPIO_Port, BQ_NSS_Pin, GPIO_PIN_SET);
		return BQ76_SPI_TRANSMISSION_ERROR;
	}
	HAL_GPIO_WritePin(BQ_NSS_GPIO_Port, BQ_NSS_Pin, GPIO_PIN_SET);
	return BQ76_OK;
}

/**
 * @func readspi
 * @brief Write a register of the BQ76PL536 device.
 * 		  The user of this library is responsible of defining this function. As
 * 		  an example, we have only defined functionality for STM32F407xx.
 * 		  Make sure that data has the size of read_length, otherwise it would be
 * 		  unsafe because the SPI receive function will overwrite over the
 * 		  overflowed address
 * @params[in] spi_address: BQ76 device address
 * @params[in] reg: register address to write
 * @params[in] value: value to write
 * @return BQ76_status [BQ76_OK|BQ76_SPI_TRANSMISSION_ERROR]
 */
static enum BQ76_status readspi(uint8_t spi_address, uint8_t reg_address, uint8_t read_length, uint8_t *data)
{
	struct BQ76_read_packet_format packet = init_read_packet(spi_address, reg_address, read_length);

	while (HAL_SPI_GetState(&hspi2) != HAL_SPI_STATE_READY);

	HAL_GPIO_WritePin(BQ_NSS_GPIO_Port, BQ_NSS_Pin, GPIO_PIN_RESET);

	if (HAL_SPI_Transmit(&hspi2, (uint8_t*) &packet, BQ76_TX_BUFF_SIZE - 1, BQ76_TIMEOUT) != HAL_OK)
	{
		HAL_GPIO_WritePin(BQ_NSS_GPIO_Port, BQ_NSS_Pin, GPIO_PIN_SET);
		return BQ76_SPI_TRANSMISSION_ERROR;
	}

	volatile uint8_t temp = 0;

	if (HAL_SPI_TransmitReceive(&hspi2, &temp, packet.buffer, read_length + 1, BQ76_TIMEOUT) != HAL_OK)
	{
		HAL_GPIO_WritePin(BQ_NSS_GPIO_Port, BQ_NSS_Pin, GPIO_PIN_SET);
		return BQ76_SPI_TRANSMISSION_ERROR;
	}
	HAL_GPIO_WritePin(BQ_NSS_GPIO_Port, BQ_NSS_Pin, GPIO_PIN_SET);

	if (calculate_crc((uint8_t*) &packet, BQ76_RX_BUFF_LENGTH(read_length) - 1, CRC_SMBUS_LUT) != packet.buffer[read_length])
	{
		return BQ76_CRC_MISMATCH;
	}
	memcpy(data, packet.buffer, read_length);
	return BQ76_OK;
}

/**
 *  @func   readspi_dma
 *  @brief  Sends a read command from the SPI interface of the current MCU to
 *  the BQ76PL536A using a DMA channel dedicated for it
 *
 *  @params[in] spi_address: BQ76 device address
 *  @params[in] reg_address: register address to read
 *  @todo the callback handle when the data has returned from the device needs
 *  to be handled
 */
static enum BQ76_status readspi_dma(uint8_t spi_address, uint8_t reg_address, uint8_t read_length, uint8_t *data)
{
	struct BQ76_read_packet_format packet = init_read_packet(spi_address, reg_address, read_length);

	// first write a small packet indicating that we are going to read a
	// register with the specified length
	while (HAL_SPI_GetState(&hspi2) != HAL_SPI_STATE_READY);

	HAL_GPIO_WritePin(BQ_NSS_GPIO_Port, BQ_NSS_Pin, GPIO_PIN_RESET);

	if (HAL_SPI_Transmit(&hspi2, (uint8_t*) &packet, BQ76_TX_BUFF_SIZE - 1, BQ76_TIMEOUT) != HAL_OK)
	{
		HAL_GPIO_WritePin(BQ_NSS_GPIO_Port, BQ_NSS_Pin, GPIO_PIN_SET);
		return BQ76_SPI_TRANSMISSION_ERROR;
	}
	if (HAL_SPI_Receive_DMA(&hspi2, data, read_length + 1) != HAL_OK)
	{
		HAL_GPIO_WritePin(BQ_NSS_GPIO_Port, BQ_NSS_Pin, GPIO_PIN_SET);
		return BQ76_SPI_RECEIVE_ERROR;
	}
	// At this stage, the user needs to handle the RX callback interrupt from
	// the DMA and
	return BQ76_OK;
}

static void calc_battery_voltages(struct BQ76 *device)
{
	for (int i = 0; i < device->adc_control.CELL_SEL + 1; ++i)
	{
		uint16_t voltage_buffer = device->raw_v_cells[i * 2] << 8 | device->raw_v_cells[2 * i + 1];
		device->v_cells[i] = ((float32_t) voltage_buffer * 6250.0f) / 16383.0f;
	}
}

enum BQ76_status bq76_init(struct BQ76 *device, uint8_t spi_address, uint8_t balancing_time, float32_t cov_threshold, uint8_t covt_delay, float32_t cuv_threshold, uint8_t cuvt_delay, float32_t temp1_threshold, float32_t temp2_threshold, uint16_t temp_delay)
{
	// Send broadcast reset
	if (bq76_broadcast_reset() != BQ76_OK)
	{
		return BQ76_BROADCAST_RESET_FAIL;
	}

	// set address to device
	if (bq76_set_address(device, spi_address) != BQ76_OK)
	{
		return BQ76_ADDRESS_CONFIG_FAIL;
	}

	// config adc control register
	if (bq76_set_adc_control(device) != BQ76_OK)
	{
		return BQ76_ADC_CONFIG_FAIL;
	}

	// config balancing time outputs
	if (bq76_set_cb_time(device, balancing_time) != BQ76_OK)
	{
		return BQ76_CB_TIME_CONFIG_FAIL;
	}

	// set function configuration
	if (bq76_set_function_config(device) != BQ76_OK)
	{
		return BQ76_FUNCTION_CONFIG_FAIL;
	}

	// set I/O configuration
	if (bq76_set_io_config(device) != BQ76_OK)
	{
		return BQ76_IO_CONFIG_FAIL;
	}

	// set cov config
	if (bq76_set_cov_config(device, cov_threshold) != BQ76_OK)
	{
		return BQ76_COV_CONFIG_FAIL;
	}

	// set covt config
	if (bq76_set_covt_config(device, covt_delay) != BQ76_OK)
	{
		return BQ76_COVT_CONFIG_FAIL;
	}

	// set cuv config
	if (bq76_set_cuv_config(device, cuv_threshold) != BQ76_OK)
	{
		return BQ76_CUV_CONFIG_FAIL;
	}

	// set cuvt config
	if (bq76_set_cuvt_config(device, cuvt_delay) != BQ76_OK)
	{
		return BQ76_CUVT_CONFIG_FAIL;
	}

	// set ot config
	if (bq76_set_ot_config(device, temp1_threshold, temp2_threshold) != BQ76_OK)
	{
		return BQ76_OT_CONFIG_FAIL;
	}

	// set ott config
	if (bq76_set_ott_config(device, temp_delay) != BQ76_OK)
	{
		return BQ76_OTT_CONFIG_FAIL;
	}
	return BQ76_OK;
}

enum BQ76_status bq76_broadcast_reset()
{
	if (writespi(BROADCAST_ADDRESS, RESET_REG, RESET_DEVICE_VALUE) != BQ76_OK)
	{
		return BQ76_SPI_TRANSMISSION_ERROR;
	}
	return BQ76_OK;
}

enum BQ76_status bq76_reset(struct BQ76 *device)
{
	if (writespi((uint8_t) device->address_control.ADDR, RESET_REG,
	RESET_DEVICE_VALUE) != BQ76_OK)
	{
		return BQ76_SPI_TRANSMISSION_ERROR;
	}
	return BQ76_OK;
}

enum BQ76_status bq76_set_address(struct BQ76 *device, uint8_t address)
{
	// Write new address to device
	// Here we are assuming that the device is previously reset, if you want to
	// modify the address call bq76_change_address
	if (writespi(0x00, ADDRESS_CONTROL_REG, address) != BQ76_OK)
	{
		return BQ76_SPI_TRANSMISSION_ERROR;
	}

	uint8_t address_buffer;
	// 1 - read new address
	if (readspi(address, ADDRESS_CONTROL_REG, 1, &address_buffer) != BQ76_OK)
	{
		return BQ76_SPI_TRANSMISSION_ERROR;
	}
	device->address_control.ADDR_RQST = address_buffer >> 7;
	device->address_control.ADDR = address_buffer & 0x01F;

	if (device->address_control.ADDR != address)
	{
		return BQ76_ADDRESS_CONFIG_FAIL;
	}
	// 2 - clear alert reg
	// First write a 1 to the AR bit
	uint8_t alert_clear = 0x80;
	if (writespi(device->address_control.ADDR, ALERT_STATUS_REG, alert_clear) != BQ76_OK)
	{
		return BQ76_SPI_TRANSMISSION_ERROR;
	}

	// then write a 0 to the AR bit
	alert_clear = 0x00;
	if (writespi(device->address_control.ADDR, ALERT_STATUS_REG, alert_clear) != BQ76_OK)
	{
		return BQ76_SPI_TRANSMISSION_ERROR;
	}

	return BQ76_OK;
}

enum BQ76_status bq76_set_adc_control(struct BQ76 *device)
{
	uint8_t adc_control_data = (device->adc_control.ADC_ON << 6) | (device->adc_control.GPAI << 5) | (device->adc_control.TS << 4) | (device->adc_control.GPAI << 3) | device->adc_control.CELL_SEL;
	// write address control
	if (writespi((uint8_t) device->address_control.ADDR, ADC_CONTROL_REG, adc_control_data) != BQ76_OK)
	{
		return BQ76_SPI_TRANSMISSION_ERROR;
	}

	return BQ76_OK;
}

enum BQ76_status bq76_set_cb_time(struct BQ76 *device, uint8_t balancing_time)
{
	device->cb_time.CBT = (balancing_time > 63) ? 63 : balancing_time;
	uint8_t cb_time_data = device->cb_time.CBCT << 7 | device->cb_time.CBT;

	if (writespi((uint8_t) device->address_control.ADDR, CB_TIME_REG, cb_time_data) != BQ76_OK)
	{
		return BQ76_SPI_TRANSMISSION_ERROR;
	}

	return BQ76_OK;
}

enum BQ76_status bq76_set_function_config(struct BQ76 *device)
{
	uint8_t function_config_data = (device->function_config.GPAI_REF << 5) | (device->function_config.GPAI_SRC << 4) | (device->function_config.CN << 2);
	if (writespi((uint8_t) device->address_control.ADDR, FUNCTION_CONFIG_REG, (uint8_t) function_config_data) != BQ76_OK)
	{
		return BQ76_SPI_TRANSMISSION_ERROR;
	}
	return BQ76_OK;
}

enum BQ76_status bq76_set_io_config(struct BQ76 *device)
{
	uint8_t io_config_data = (device->io_config.CRCNOFLT << 7) | (device->io_config.CRC_DIS);
	if (writespi((uint8_t) device->address_control.ADDR, IO_CONFIG_REG, io_config_data) != BQ76_OK)
	{
		return BQ76_SPI_TRANSMISSION_ERROR;
	}
	return BQ76_OK;
}

enum BQ76_status bq76_set_cov_config(struct BQ76 *device, float32_t voltage_threshold)
{
	device->cov_config.VTH = calculate_cov(voltage_threshold);
	uint8_t cov_config_data = (device->cov_config.DISABLE << 7) | (device->cov_config.VTH);
	if (writespi((uint8_t) device->address_control.ADDR, COV_CONFIG_REG, cov_config_data) != BQ76_OK)
	{
		return BQ76_SPI_TRANSMISSION_ERROR;
	}

	uint8_t cov_config_data_buffer;
	if (readspi((uint8_t) device->address_control.ADDR, COV_CONFIG_REG, 1, &cov_config_data_buffer) != BQ76_OK)
	{
		return BQ76_SPI_TRANSMISSION_ERROR;
	}
	return BQ76_OK;
}

enum BQ76_status bq76_set_cuv_config(struct BQ76 *device, float32_t voltage_threshold)
{
	device->cov_config.VTH = calculate_cuv(voltage_threshold);
	uint8_t cov_config_data = (device->cov_config.DISABLE << 7) | device->cov_config.VTH;
	if (writespi((uint8_t) device->address_control.ADDR, CUV_CONFIG_REG, cov_config_data) != BQ76_OK)
	{
		return BQ76_SPI_TRANSMISSION_ERROR;
	}
	return BQ76_OK;
}

enum BQ76_status bq76_set_covt_config(struct BQ76 *device, uint16_t delay)
{
	device->covt_config.DELAY = calculate_delay(delay);
	uint8_t covt_config_data = (device->covt_config.US_MS << 7) | device->covt_config.DELAY;
	if (writespi((uint8_t) device->address_control.ADDR, COVT_CONFIG_REG, covt_config_data) != BQ76_OK)
	{
		return BQ76_SPI_TRANSMISSION_ERROR;
	}
	return BQ76_OK;
}

enum BQ76_status bq76_set_cuvt_config(struct BQ76 *device, uint16_t delay)
{
	device->cuvt_config.DELAY = calculate_delay(delay);
	uint8_t cuvt_config_data = (device->cuvt_config.US_MS << 7) | (device->cuvt_config.DELAY);
	if (writespi((uint8_t) device->address_control.ADDR, CUVT_CONFIG_REG, cuvt_config_data) != BQ76_OK)
	{
		return BQ76_SPI_TRANSMISSION_ERROR;
	}
	return BQ76_OK;
}

enum BQ76_status bq76_set_ot_config(struct BQ76 *device, float32_t ot1_threshold, float32_t ot2_threshold)
{
	device->ot_config.OT1 = calculate_ot(ot1_threshold);
	device->ot_config.OT2 = calculate_ot(ot2_threshold);
	uint8_t ot_config_data = (device->ot_config.OT2 << 4) | (device->ot_config.OT1);
	if (writespi((uint8_t) device->address_control.ADDR, OT_CONFIG_REG, ot_config_data) != BQ76_OK)
	{
		return BQ76_SPI_TRANSMISSION_ERROR;
	}
	return BQ76_OK;
}

enum BQ76_status bq76_set_ott_config(struct BQ76 *device, uint16_t delay_value)
{
	// the LSB of ott_config representes 10mS
	device->ott_config = calculate_ott(delay_value);
	if (writespi((uint8_t) device->address_control.ADDR, OTT_CONFIG_REG, device->ott_config) != BQ76_OK)
	{
		return BQ76_SPI_TRANSMISSION_ERROR;
	}
	return BQ76_OK;
}

enum BQ76_status bq76_read_v_cells(struct BQ76 *device)
{
	// the amount of cells to be read from a device is automatically set by the
	// adc_control registerp
	if (readspi((uint8_t) device->address_control.ADDR, VCELL1_LOW_REG, 2 * (device->adc_control.CELL_SEL + 1), device->raw_v_cells) != BQ76_OK)
	{
		return BQ76_SPI_TRANSMISSION_ERROR;
	}
	calc_battery_voltages(device);
	device->data_conversion_ongoing = 0;
	return BQ76_OK;
}

enum BQ76_status bq76_read_v_cells_dma(struct BQ76 *device)
{
	uint8_t n_cells = 2 * (device->adc_control.CELL_SEL + 1);
	if (readspi_dma((uint8_t) device->address_control.ADDR, VCELL1_LOW_REG, n_cells, device->raw_v_cells) != BQ76_OK)
	{
		return BQ76_SPI_TRANSMISSION_ERROR;
	}
	device->current_dma_request = BQ76_V_CELLS;
	return BQ76_OK;
}

enum BQ76_status bq76_swrqst_adc_convert(struct BQ76 *device)
{
	device->data_conversion_ongoing = 1;
	if (writespi((uint8_t) device->address_control.ADDR, ADC_CONVERT_REG, 0x01) != BQ76_OK)
	{
		return BQ76_SPI_TRANSMISSION_ERROR;
	}
	return BQ76_OK;
}

void bq76_hwrqst_adc_convert(struct BQ76 *device)
{
#if defined(USE_HAL_DRIVER) && defined(STM32F407xx)
    HAL_GPIO_WritePin(BQ76_CONV_GPIO, BQ76_CONV_PIN, GPIO_PIN_SET);
    device->data_conversion_ongoing = 1;
#endif
}

void bq76_assert_end_adc_convert()
{
#if defined(USE_HAL_DRIVER) && defined(STM32F407xx)
    HAL_GPIO_WritePin(BQ76_CONV_GPIO, BQ76_CONV_PIN, GPIO_PIN_RESET);
#endif
}

enum BQ76_status bq76_brdcst_adc_convert()
{
	if (writespi(BROADCAST_ADDRESS, ADC_CONVERT_REG, 0x01) != BQ76_OK)
	{
		return BQ76_SPI_TRANSMISSION_ERROR;
	}
	return BQ76_OK;
}

enum BQ76_status bq76_read_alert_reg(struct BQ76 *device)
{
	uint8_t buffer;
	if (readspi((uint8_t) device->address_control.ADDR, ALERT_STATUS_REG, 1, (uint8_t*) &buffer) != BQ76_OK)
	{
		return BQ76_SPI_TRANSMISSION_ERROR;
	}
	device->alert_status.OT1 = buffer & 0x01;
	device->alert_status.OT2 = buffer >> 1 & 0x01;
	device->alert_status.SLEEP = buffer >> 2 & 0x01;
	device->alert_status.TSD = buffer >> 3 & 0x01;
	device->alert_status.FORCE = buffer >> 4 & 0x01;
	device->alert_status.ECC_ERR = buffer >> 5 & 0x01;
	device->alert_status.PARITY = buffer >> 6 & 0x01;
	device->alert_status.AR = buffer >> 7 & 0x01;
	return BQ76_OK;
}

enum BQ76_status bq76_read_fault_reg(struct BQ76 *device)
{
	uint8_t buffer = 0x00;
	if (readspi((uint8_t) device->address_control.ADDR, FAULT_STATUS_REG, 1, &buffer) != BQ76_OK)
	{
		return BQ76_SPI_TRANSMISSION_ERROR;
	}
	device->fault_status.COV_BIT = buffer & 0x01;
	device->fault_status.CUV_BIT = buffer >> 1 & 0x01;
	device->fault_status.CRC_BIT = buffer >> 2 & 0x01;
	device->fault_status.POR_BIT = buffer >> 3 & 0x01;
	device->fault_status.FORCE_BIT = buffer >> 4 & 0x01;
	device->fault_status.I_FAULT_BIT = buffer >> 5 & 0x01;
	return BQ76_OK;
}

enum BQ76_status bq76_clear_fault_reg(struct BQ76 *device, uint8_t flags)
{
	if (writespi((uint8_t) device->address_control.ADDR, FAULT_STATUS_REG, flags) != BQ76_OK)
	{
		return BQ76_SPI_TRANSMISSION_ERROR;
	}
	if (writespi((uint8_t) device->address_control.ADDR, FAULT_STATUS_REG, 0x00) != BQ76_OK)
	{
		return BQ76_SPI_TRANSMISSION_ERROR;
	}
	return BQ76_OK;
}

enum BQ76_status bq76_clear_alert_reg(struct BQ76 *device, uint8_t flags)
{
	if (writespi((uint8_t) device->address_control.ADDR, ALERT_STATUS_REG, flags) != BQ76_OK)
	{
		return BQ76_SPI_TRANSMISSION_ERROR;
	}
	if (writespi((uint8_t) device->address_control.ADDR, ALERT_STATUS_REG, 0x00) != BQ76_OK)
	{
		return BQ76_SPI_TRANSMISSION_ERROR;
	}
	return BQ76_OK;
}

enum BQ76_status bq76_read_cov_fault_reg(struct BQ76 *device)
{
	uint8_t buffer = 0x00;
	if (readspi((uint8_t) device->address_control.ADDR, COV_FAULT_REG, 1, &buffer) != BQ76_OK)
	{
		return BQ76_SPI_TRANSMISSION_ERROR;
	}
	device->cov_fault.OV1 = buffer & 0x01;
	device->cov_fault.OV2 = buffer >> 1 & 0x01;
	device->cov_fault.OV3 = buffer >> 2 & 0x01;
	device->cov_fault.OV4 = buffer >> 3 & 0x01;
	device->cov_fault.OV5 = buffer >> 4 & 0x01;
	device->cov_fault.OV6 = buffer >> 5 & 0x01;
	return BQ76_OK;
}

enum BQ76_status bq76_read_cuv_fault_reg(struct BQ76 *device)
{
	uint8_t buffer;
	if (readspi((uint8_t) device->address_control.ADDR, CUV_FAULT_REG, 1, &buffer) != BQ76_OK)
	{
		return BQ76_SPI_TRANSMISSION_ERROR;
	}
	device->cuv_fault.UV1 = buffer & 0x01;
	device->cuv_fault.UV2 = buffer >> 1 & 0x01;
	device->cuv_fault.UV3 = buffer >> 2 & 0x01;
	device->cuv_fault.UV4 = buffer >> 3 & 0x01;
	device->cuv_fault.UV5 = buffer >> 4 & 0x01;
	device->cuv_fault.UV6 = buffer >> 5 & 0x01;
	return BQ76_OK;
}

enum BQ76_status bq76_set_balancing_output(struct BQ76 *device, uint8_t transistors)
{
	device->cb_ctrl.CBAL = transistors & 0x3F;
	if (writespi((uint8_t) device->address_control.ADDR, CB_CONTROL_REG, (uint8_t) device->cb_ctrl.CBAL) != BQ76_OK)
	{
		return BQ76_SPI_TRANSMISSION_ERROR;
	}
	uint8_t buffer;
	if (readspi(device->address_control.ADDR, CB_CONTROL_REG, 1, &buffer) != BQ76_OK)
	{
		return BQ76_SPI_TRANSMISSION_ERROR;
	}
	return BQ76_OK;
}

enum BQ76_status handle_bq76_dma_callback(struct BQ76 *monitor)
{
	// Disable the SPI interface for this chip

	switch (monitor->current_dma_request)
	{
	case BQ76_V_CELLS:
		;
		// before calculating new data and updating it, we need to check if
		// the crc is valid
		HAL_GPIO_WritePin(BQ_NSS_GPIO_Port, BQ_NSS_Pin, GPIO_PIN_SET);
		//uint8_t read_length = 2*(monitor->adc_control.CELL_SEL + 1);
		/*
		 struct BQ76_read_packet_format packet =
		 init_read_packet(monitor->address_control.ADDR, VCELL1_LOW_REG,
		 read_length);
		 */
		//memcpy(packet.buffer, monitor->raw_v_cells, read_length - 1);
		/*
		 if(calculate_crc((uint8_t *) &packet,
		 BQ76_RX_BUFF_LENGTH(read_length) - 1,
		 CRC_SMBUS_LUT) != packet.buffer[read_length]){
		 return BQ76_CRC_MISMATCH;
		 }
		 */
		calc_battery_voltages(monitor);
		monitor->current_dma_request = BQ76_NO_CONV;
		monitor->data_conversion_ongoing = 0;
		return BQ76_OK;
	case BQ76_T_SENSORS:

		// TODO: implement calc temp sensors
		break;
	case BQ76_FAULT_REG:

		// TODO: implement fault reg dma reading
		break;
	case BQ76_ALERT_REG:

		// TODO: implement alert reg dma reading
		break;
	default:
		break;
	}
	return BQ76_OK;
}
