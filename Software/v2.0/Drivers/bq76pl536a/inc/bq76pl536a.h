/******************************************************************************
 *
 * @file	bq76pl536a.h
 * @brief 	Texas Instruments BQ76PL536A high level data structures definitions
 * @version	v1.00f00
 * @date	27, Dec. 2020
 * @author	CECARELLI, Federico (fededc88@gmail.com),
 * 			MOYA, Martin		(moyamartin1@gmail.com),
 * 			SANTOS, Lucio		(lusho2206@gmail.com)
 * @copyright GPL license, all text here must be included in any redistribution
 *****************************************************************************/
#ifndef _BQ76PL536A_H_
#define _BQ76PL536A_H_

#include "bq76pl536a_defs.h"

#include "stm32g4xx_hal.h"
#include "main.h"

/*
 * All the instances of these drivers share only one SPI bus, that we asume
 * that it is already initialized on the main before the infinite loop.
 * This design takes into account that the project has been created with
 * the STM32CubeMX which creates the function needed to initialize the i2c
 * bus.
 *
 * We also asume that the SPI bus used is the hspi1
 */
extern SPI_HandleTypeDef hspi1;
#define BQ76_INTERFACE hspi1
#define BQ76_TIMEOUT 1000

// STM32F407xx handles the SPI via SPI_NSS_SOFT, change the following if
// it's handled by hw
#define BQ76_CS_GPIO BQ76_CS_GPIO_Port
#define BQ76_CS_PIN BQ76_CS_Pin
#define BQ76_CONV_GPIO BQ76_CONV_GPIO_Port
#define BQ76_CONV_PIN BQ76_CONV_Pin

enum BQ76_status
{
	BQ76_OK = 0, BQ76_SPI_TRANSMISSION_ERROR = -1, BQ76_BROADCAST_RESET_FAIL = -2, BQ76_DEVICE_RESET_FAIL = -3, BQ76_ADDRESS_CONFIG_FAIL = -4, BQ76_ADC_CONFIG_FAIL = -5, BQ76_CB_TIME_CONFIG_FAIL = -6, BQ76_FUNCTION_CONFIG_FAIL = -7, BQ76_IO_CONFIG_FAIL = -8, BQ76_COV_CONFIG_FAIL = -9, BQ76_COVT_CONFIG_FAIL = -10, BQ76_CUV_CONFIG_FAIL = -11, BQ76_CUVT_CONFIG_FAIL = -12, BQ76_OT_CONFIG_FAIL = -13, BQ76_OTT_CONFIG_FAIL = -14, BQ76_CRC_MISMATCH = -15, BQ76_SPI_RECEIVE_ERROR = -16,
};

enum BQ76_DMA_request
{
	BQ76_NO_CONV = 0, BQ76_V_CELLS, BQ76_T_SENSORS, BQ76_FAULT_REG, BQ76_ALERT_REG,
};

struct BQ76_write_packet_format
{
	uint8_t device_address;
	uint8_t reg_address;
	uint8_t reg_data;
	uint8_t crc;
};

struct BQ76_read_packet_format
{
	uint8_t device_address;
	uint8_t start_reg_address;
	uint8_t read_length;
	uint8_t buffer[16];
};

#define BQ76_TX_BUFF_SIZE sizeof(struct BQ76_write_packet_format)
#define BQ76_RX_BUFF_LENGTH(length) \
    (sizeof(struct BQ76_write_packet_format) + length)
#define BQ76_RX_BUFF_SIZE 12

struct BQ76
{
	struct adc_control adc_control;
	struct address_control address_control;
	struct alert_status alert_status;
	struct cb_ctrl cb_ctrl;
	struct cb_time cb_time;
	struct cov_fault cov_fault;
	struct cuv_fault cuv_fault;
	struct delay_config covt_config;
	struct delay_config cuvt_config;
	struct device_status device_status;
	struct fault_status fault_status;
	struct function_config function_config;
	struct io_config io_config;
	struct io_control io_control;
	struct ot_config ot_config;
	struct presult_a presult_a;
	struct presult_b presult_b;
	struct vth_config cov_config;
	struct vth_config cuv_config;
	uint8_t ott_config;
	uint8_t user_register[4];
	uint8_t data_conversion_ongoing;
	uint8_t raw_v_cells[13]; /* <- don't forget that this needs one bit extra for th CRC calculation, that's why the double of size */
	float32_t v_cells[6];
	enum BQ76_DMA_request current_dma_request;
};

/**
 * @func bq76_init
 * @brief Initializes a bq76 struct with the users desired configuration
 * @params[in] device: BQ76 pointer referencing to the desired device to be
 * 					   initialized 
 * @params[in] spi_address: uint8_t device's new spi address
 * @params[in] balancing_time: uint8_t device's balancing maximum period of time
 * 			   where balancing can be active
 * @params[in] cov_threshold: float32_t 
 * @return BQ76_status [OK|SPI_TRANSMISSION_ERROR]
 */
enum BQ76_status bq76_init(struct BQ76 *device, uint8_t spi_address, uint8_t balancing_time, float32_t cov_threshold, uint8_t covt_delay, float32_t cuv_threshold, uint8_t cuvt_delay, float32_t temp1_threshold, float32_t temp2_threshold, uint16_t temp_delay);

/**
 * @func bq76_broadcast_reset
 * @brief Sends a reset broadcast to all connected devices with a valid address
 * @returns bq76_status [OK|SPI_TRANSMISSION_ERROR]
 */
enum BQ76_status bq76_broadcast_reset();

/**
 * @func bq76_reset
 * @brief sends a reset command to a specified device
 * @params[in] device: BQ76 pointer referencing to the desired device to be
 * 					   modified
 * @return BQ76_status [OK|SPI_TRANSMISSION_ERROR]
 */
enum BQ76_status bq76_reset(struct BQ76 *device);

/**
 * @func bq76_set_address
 * @brief sets a device's address and check if it was properly set
 * @params[in] device: BQ76 pointer referencing to the desired device to be
 * 					   modified
 * @params[in] new_address: uint8_t variable that holds the new address of the
 * 			   device
 * @return BQ76_status [OK|ADDRESS_CONFIG_FAIL|SPI_TRANSMISSION_ERROR]
 */
enum BQ76_status bq76_set_address(struct BQ76 *device, uint8_t address);

/**
 * @func bq76_set_adc_control
 * @brief sets a device's adc control register data
 * @params[in] device: BQ76 pointer referencing to the desired device to be
 * 					   modified
 * @return BQ76_status [OK|ADDRESS_CONFIG_FAIL|SPI_TRANSMISSION_ERROR]
 */
enum BQ76_status bq76_set_adc_control(struct BQ76 *device);

/**
 * @func  bq76_set_cb_time
 * @brief sets a device's balancing timeout, maximum timeout is 63
 * 		  (seconds/minutes depending the time unit time)
 * @params[in] device: BQ76 pointer referencing to the desired device to be
 * 					   modified
 * @params[in] balancing_time: uint8_t variable holding the number of
 * 			   seconds/minutes that the balalncers should be on
 * @return BQ76_status [OK|SPI_TRANSMISSION_ERROR]
 */
enum BQ76_status bq76_set_cb_time(struct BQ76 *device, uint8_t balancing_time);

/**
 * @func  bq76_set_function_config
 * @brief sets a device's function config register
 * @params[in] device: BQ76 pointer referencing to the desired device to be
 * 					   modified
 * @return BQ76_status [OK|SPI_TRANSMISSION_ERROR]
 */
enum BQ76_status bq76_set_function_config(struct BQ76 *device);

/**
 * @func  bq76_set_io_config
 * @brief sets a device's I/O config register
 * @params[in] device: BQ76 pointer referencing to the desired device to be
 * 					   modified
 * @return BQ76_status [OK|SPI_TRANSMISSION_ERROR]
 */
enum BQ76_status bq76_set_io_config(struct BQ76 *device);

/**
 * @func  bq76_set_cov_config
 * @brief sets a device's Cell Overvoltage Threshold value
 * @params[in] device: BQ76 pointer referencing to the desired device to be
 * 			   resetted
 * @params[in] voltage_threshold: float32_t that holds the voltage_threshold
 * 			   value
 * @return BQ76_status [OK|SPI_TRANSMISSION_ERROR]
 */
enum BQ76_status bq76_set_cov_config(struct BQ76 *device, float32_t voltage_threshold);

/**
 * @func  bq76_set_cuv_config
 * @brief sets a device's Cell undervoltage threshold value
 * @params[in] device: BQ76 pointer referencing to the desired device to be
 * 					   modified
 */
enum BQ76_status bq76_set_cuv_config(struct BQ76 *device, float32_t voltage_threshold);

/**
 * @func  bq76_set_covt_config
 * @brief sets a device's Cell Undervoltage delay time
 * @params[in] device: BQ76 pointer referencing to the desired device to be
 * 			   modified
 * @params[in] delay: uint16_t variable that holds the 
 * 					  delay time
 * @return BQ76_status [OK|SPI_TRANSMISSION_ERROR]
 */
enum BQ76_status bq76_set_covt_config(struct BQ76 *device, uint16_t delay);

/**
 * @func  bq76_set_cuvt_config
 * @brief sets a device's Cell Undervoltage delay time
 * @params[in] device: BQ76 pointer referencing to the desired device to be
 * 			   modified
 * @params[in] delay: uint16_t variable that holds the 
 * 					  delay time
 * @return BQ76_status [OK|SPI_TRANSMISSION_ERROR]
 */
enum BQ76_status bq76_set_cuvt_config(struct BQ76 *device, uint16_t delay);

/**
 * @func  bq76_set_ot_config
 * @brief sets a device's temperature sensor threshold value for sensors 1 and 2
 * @params[in] device: BQ76 pointer referencing to the desired device to be
 * 			   modified
 * @params[in] ot1: float32_t value that sets the temp sensor 1 threshold value
 * @params[in] ot2: float32_t value that sets the temp sensor 2 threshold value
 * @return BQ76_status indicating if the process failed or not
 */
enum BQ76_status bq76_set_ot_config(struct BQ76 *device, float32_t ot1, float32_t ot2);

/**
 * @func  bq76_set_ott_config
 * @brief sets a device's temperature sensor delay time detection for temp.
 * 		  sensors 1 and 2
 * @params[in] device: BQ76 pointer referencing to the desired device to be
 * 					   modified
 * @params[in] delay_time: uint16_t that holds the amount of time until the
 * 			   overtemperature is triggered
 * @return BQ76_status indicating if the process failed or not
 */
enum BQ76_status bq76_set_ott_config(struct BQ76 *device, uint16_t delay_time);

/**
 * @func  bq76_read_v_cells()
 * @brief reads the voltage value of n cells, this function always takes as a
 * starting point the cell 1 and this function should always be called when DRDY
 * pin from the chip is asserted
 * @params[in] device: BQ76 pointer referencing to the desired device to be
 *             modified
 * @params[in] n_cells: amount of cells to be read by the user, the result is
 * stored in BQ76->v_cells
 * @return BQ76_status indicating if the process failed or not
 * @see BQ76
 */
enum BQ76_status bq76_read_v_cells(struct BQ76 *device);

/**
 * @func  bq76_read_v_cells_dma()
 * @brief reads the voltage value of n cells, this function always takes as a
 * starting point the cell 1 and this function should always be called when 
 * DRDY pin from the chip is asserted. This function is similar to
 * bq76_read_v_cells but instead it uses the MCUs DMA to fullfil the transaction
 * @params[in] device: BQ76 pointer referencing to the desired device to be
 *             modified
 * @params[in] n_cells: amount of cells to be read by the user, the result is
 * stored in BQ76->v_cells
 * @return BQ76_status indicating if the process failed or not
 * @see BQ76
 */
enum BQ76_status bq76_read_v_cells_dma(struct BQ76 *device);

/**
 * @func bq76_rqst_adc_convert
 * @brief Request and ADC conversion by writing a 1 to the ADC_CONVERT register
 *        This bit starts a conversion, using the settings programmed into the
 *        adc_control register
 * @params[in] device: BQ76 pointer referencing to the desired device to be
 *             modified
 * @return BQ76_status indicating if the process failed or not
 */
enum BQ76_status bq76_swrqst_adc_convert(struct BQ76 *device);

/**
 * @func bq76_rqst_adc_convert
 * @brief Request and ADC conversion by writing a 1 to the ADC_CONVERT register
 *        to all the connected devices
 * @return BQ76_status indicating if the process failed or not
 */
enum BQ76_status bq76_bdcst_adc_convert();

/**
 * @func  bq76_read_alert_reg
 * @brief Reads the ALERT_STATUS register and updates the alert_status variable
 * @param[in] device: BQ76 pointer referencing to the desired device to be
 *            modified
 * @return BQ76_status indicating if the process failed or not
 */
enum BQ76_status bq76_read_alert_reg(struct BQ76 *device);

/**
 * @func  bq76_read_fault_reg
 * @brief Reads the FAULT_STATUS register and updates the fault_status variable
 * @param[in] device: BQ76 pointer referencing to the desired device to be
 *            modified
 * @return BQ76_status indicating if the process failed or not
 */
enum BQ76_status bq76_read_fault_reg(struct BQ76 *device);

/**
 * @func bq76_clear_fault_flags
 * @brief clear the flags from the fault register by writing a one to them
 * @param[in] device: BQ76 pointer referencing to the desired device to be
 *                    modified
 * @return BQ76_status indicating if the process failed or not
 */
enum BQ76_status bq76_clear_fault_reg(struct BQ76 *device, uint8_t flags);

/**
 * @func bq76_clear_alert_flags
 * @brief clear the flags from the alert register by writing a one to them
 * @param[in] device: BQ76 pointer referencing to the desired device to be
 *                    modified
 * @return BQ76_status indicating if the process failed or not
 */
enum BQ76_status bq76_clear_alert_reg(struct BQ76 *device, uint8_t flags);

/**
 * @func bq76_read_cov_faut_reg
 * @brief Reads the cov register to see which cell is the faulty one
 * @param[in] device: BQ76 pointer referencing to the desired device to be
 *            modified
 * @return BQ76_status indicating if the operation failed or not
 */
enum BQ76_status bq76_read_cov_fault_reg(struct BQ76 *device);

/**
 * @func bq76_read_cuv_fault_reg
 * @brief Reads the cuv register to see which cell is the faulty one
 * @params[in] device: BQ76 pointer referencing to the desired device to be
 *             modified
 * @return BQ76_status indiciating if the operation failed or not
 */
enum BQ76_status bq76_read_cuv_fault_reg(struct BQ76 *device);

/**
 * @func bq76_set_balancing_output
 * @brief Turns on the balancing transistors given a uint8_t value indicating
 *        which transistors should be turned on and which transistors should 
 *        be turned off
 * @params[in] device: BQ76 pointer referencing to the desired device to be
 *             modified
 * @params[in] uint8_t flag indicating which transistors to turn off and on
 * @return BQ76_status indicating if the operation failed or not
 */
enum BQ76_status bq76_set_balancing_output(struct BQ76 *device, uint8_t transistors);

/**
 * @func    handle_bq76_dma_callback
 * @brief   handles the bq76 device DMA callback defined by the
 *          current_dma_request, this function must be called as callback from
 *          the dma hardware interrupt in the main flow application
 */
enum BQ76_status handle_bq76_dma_callback(struct BQ76 *monitor);

#endif /* bq76pl536a.h */
