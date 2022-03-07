/******************************************************************************
 *
 * @file	bq76pl536a_defs.h
 * @brief 	Texas Instruments BQ76PL536A register and constants definitions
 * @version	v1.00f00
 * @date	27, Dec. 2020
 * @author	CECARELLI, Federico (fededc88@gmail.com),
 * 			MOYA, Martin		(moyamartin1@gmail.com),
 * 			SANTOS, Lucio		(lusho2206@gmail.com)
 * @copyright GPL license, all text here must be included in any redistribution
 *****************************************************************************/

#ifndef __BQ76PL536A_DEFS_H_
#define __BQ76PL536A_DEFS_H_

#include <stdint.h>

/** 
 * BQ76PL536A Status Register address 
 *	Default value: 0b00000000 0x00
 *	Type: R - GROUP 1
 */
#define DEVICE_STATUS_REG		(0x00)

/** 
 * BQ76PL536A GPAI Low Data Register address 
 *	Default value: 0b00000000 0x00
 *	Type: R - GROUP 1
 */
#define GPAI_LOW_REG			(0x01)

/** 
 * BQ76PL536A GPAI High Data Register address 
 *	Default value: 0b00000000 0x00
 *	Type: R - GROUP 1
 */
#define GPAI_HIGH_REG			(0x02)

/** 
 * BQ76PL536A VCELL1 Low Data Register address 
 *	Default value: 0b00000000 0x00
 *	Type: R - GROUP 1
 */
#define VCELL1_LOW_REG			(0x03)

/** 
 * BQ76PL536A VCELL1 High Data Register address 
 *	Default value: 0b00000000 0x00
 *	Type: R - GROUP 1
 */
#define VCELL1_HIGH_REG			(0x04)

/** 
 * BQ76PL536A VCELL2 Low Data Register address 
 *	Default value: 0b00000000 0x00
 *	Type: R - GROUP 1
 */
#define VCELL2_LOW_REG			(0x05)

/** 
 * BQ76PL536A VCELL2 High Data Register address 
 *	Default value: 0b00000000 0x00
 *	Type: R - GROUP 1
 */
#define VCELL2_HIGH_REG			(0x06)

/** 
 * BQ76PL536A VCELL3 Low Data Register address 
 *	Default value: 0b00000000 0x00
 *	Type: R - GROUP 1
 */
#define VCELL3_LOW_REG			(0x07)

/** 
 * BQ76PL536A VCELL3 High Data Register address 
 *	Default value: 0b00000000 0x00
 *	Type: R - GROUP 1
 */
#define VCELL3_HIGH_REG			(0x08)

/** 
 * BQ76PL536A VCELL4 Low Data Register address 
 *	Default value: 0b00000000 0x00
 *	Type: R - GROUP 1
 */
#define VCELL4_LOW_REG			(0x09)

/** 
 * BQ76PL536A VCELL4 High Data Register address 
 *	Default value: 0b00000000 0x00
 *	Type: R - GROUP 1
 */
#define VCELL4_HIGH_REG			(0x0a)

/** 
 * BQ76PL536A VCELL5 Low Data Register address 
 *	Default value: 0b00000000 0x00
 *	Type: R - GROUP 1
 */
#define VCELL5_LOW_REG			(0x0b)

/** 
 * BQ76PL536A VCELL5 High Data Register address 
 *	Default value: 0b00000000 0x00
 *	Type: R - GROUP 1
 */
#define VCELL5_HIGH_REG			(0x0c)

/** 
 * BQ76PL536A VCELL6 Low Data Register address 
 *	Default value: 0b00000000 0x00 
 *	Type: R - GROUP 1
 */
#define VCELL6_LOW_REG			(0x0d)

/** 
 * BQ76PL536A VCELL6 High Data Register address 
 *	Default value: 0b00000000 0x00
 *	Type: R - GROUP 1
 */
#define VCELL6_HIGH_REG			(0x0e)

/** 
 * BQ76PL536A TEMPERATURE1 Low Data Register address 
 *	Default value: 0b00000000 0x00
 *	Type: R - GROUP 1
 */
#define TEMPERATURE1_LOW_REG 	(0x0f)

/** 
 * BQ76PL536A TEMPERATURE1 High Data Register address 
 *	Default value: 0b00000000 0x00
 *	Type: R - GROUP 1
 */
#define TEMPERATURE1_HIGH_REG	(0x10)

/** 
 * BQ76PL536A TEMPERATURE2 Low Data Register address 
 *	Default value: 0b00000000 0x00
 *	Type: R - GROUP 1
 */
#define TEMPERATURE2_LOW_REG 	(0x11)

/** 
 * BQ76PL536A TEMPERATURE2 High Data Register address 
 :	Default value: 0b00000000 0x00
 *	Type: R - GROUP 1
 */
#define TEMPERATURE2_HIGH_REG	(0x12)

/** 
 * BQ76PL536A Alert Status Register address 
 *	Default value: 0b10000000 0x80
 *	Type: R/W - GROUP 2
 */
#define ALERT_STATUS_REG		(0x20)

/** 
 * BQ76PL536A Fault Status Register address 
 *	Default value: 0b00001000 0x08
 *	Type: R/W - GROUP 2
 */
#define FAULT_STATUS_REG		(0x21)

/** 
 * BQ76PL536A COV Fault Status Register address 
 *	Default value: 0b00000000 0x00
 *	Type: R - GROUP 1
 */
#define COV_FAULT_REG			(0x22) 

/** 
 * BQ76PL536A CUV Fault Status Register address 
 *	Default value: 0b00000000 0x00
 *	Type: R - GROUP 1
 */
#define CUV_FAULT_REG			(0x23)

/** 
 * BQ76PL536A Preresult A Status Register address 
 *	Default value: 0b00000000 0x00
 *	Type: R - GROUP 1
 */
#define PRESULT_A_REG			(0x24)

/** 
 * BQ76PL536A Preresult B Status Register address 
 *	Default value: 0b00000000 0x00
 *	Type: R - GROUP 1
 */
#define PRESULT_B_REG			(0x25)

/** 
 * BQ76PL536A ADC Control Register address 
 *	Default value: 0b00000000 0x00
 *	Type: R/W - GROUP 2
 */
#define ADC_CONTROL_REG			(0x30)

/** 
 * BQ76PL536A IO Control Register address 
 *	Default value: 0b00000000 0x00
 *	Type: R/W - GROUP 2
 */
#define IO_CONTROL_REG			(0x31)

/** 
 * BQ76PL536A CB Control Register address 
 *	Default value: 0b00000000 0x00
 *	Type: R/W - GROUP 2
 */
#define CB_CONTROL_REG			(0x32)

/** 
 * BQ76PL536A CB Time Register address 
 *	Default value: 0b00000000 0x00
 *	Type: R/W - GROUP 2
 */
#define CB_TIME_REG				(0x33)

/** 
 * BQ76PL536A ADC Convert address 
 *	Default value: 0b00000000 0x00
 *	Type: R/W - GROUP 2
 */
#define ADC_CONVERT_REG			(0x34)

/** 
 * BQ76PL536A Shadow Control address 
 *	Default value: 0b00000000 0x00
 *	Type: R/W - GROUP 2
 */
#define SHADOW_CONTROL_REG		(0x3a)

/** 
 * BQ76PL536A Address Control address 
 *	Default value: 0b00000000 0x00
 *	Type: R/W - GROUP 2
 */
#define ADDRESS_CONTROL_REG		(0x3b)

/** 
 * BQ76PL536A Reset address 
 *	Default value: 0b00000000 0x00
 *	Type: W - GROUP 2
 */
#define RESET_REG				(0x3c)

/** 
 * BQ76PL536A Test Select address 
 *	Default value: 0b00000000 0x00
 *	Type: R/W - GROUP 2
 */
#define TEST_SELECT_REG			(0x3d)

/** 
 * BQ76PL536A EPROM programming mode enable address 
 *	Default value: 0b00000000 0x00
 *	Type: R/W - GROUP 3
 */
#define E_EN_REG				(0x3f)

/** 
 * BQ76PL536A Function Configuration address 
 *	Default value: 0b00000000 0x00
 *	Type: R/W - GROUP 3
 */
#define FUNCTION_CONFIG_REG		(0x40)

/** 
 * BQ76PL536A I/O Pin Configuration address 
 *	Default value: 0b00000000 0x00
 *	Type: R/W - GROUP 3
 */
#define IO_CONFIG_REG			(0x41)

/** 
 * BQ76PL536A COV Configuration address 
 *	Default value: 0b00000000 0x00
 *	Type: R/W - GROUP 3
 */
#define COV_CONFIG_REG			(0x42)

/** 
 * BQ76PL536A COVT Config Configuration address 
 *	Default value: 0b00000000 0x00
 *	Type: R/W - GROUP 3
 */
#define COVT_CONFIG_REG			(0x43)

/** 
 * BQ76PL536A CUV Config Configuration address 
 *	Default value: 0b00000000 0x00
 *	Type: R/W - GROUP 3
 */
#define CUV_CONFIG_REG			(0x44)

/** 
 * BQ76PL536A CUVT Config Configuration address 
 *	Default value: 0b00000000 0x00
 *	Type: R/W - GROUP 3
 */
#define CUVT_CONFIG_REG			(0x45)

/** 
 * BQ76PL536A OT Config Configuration address 
 *	Default value: 0b00000000 0x00
 *	Type: R/W - GROUP 3
 */
#define OT_CONFIG_REG			(0x46)

/** 
 * BQ76PL536A OTT Config Configuration address 
 *	Default value: 0b00000000 0x00
 *	Type: R/W - GROUP 3
 */
#define OTT_CONFIG_REG			(0x46)

/** 
 * BQ76PL536A USER1 Register address 
 *	Default value: 0b00000000 0x00
 *	Type: R/W - GROUP 3
 */
#define USER1_REG				(0x48)

/** 
 * BQ76PL536A USER2 Register address 
 *	Default value: 0b00000000 0x00
 *	Type: R/W - GROUP 3
 */
#define USER2_REG				(0x49)

/** 
 * BQ76PL536A USER3 Register address 
 *	Default value: 0b00000000 0x00
 *	Type: R/W - GROUP 3
 */
#define USER3_REG				(0x4A)

/** 
 * BQ76PL536A USER4 Register address 
 *	Default value: 0b00000000 0x00
 *	Type: R/W - GROUP 3
 */
#define USER4_REG				(0x4B)

/**
 * BQ76PL536A Write this value to SHDW_CTRL (0x3A) to enable writing Group 3
 * registers
 */
#define ENABLE_REG_3			(0x35)

/**
 * BQ76PL536A Write this value to RESET (0x3A) to reset the device 
 * registers
 */
#define RESET_DEVICE_VALUE		(0xA5)

/**
 * broadcast address
 */
#define BROADCAST_ADDRESS		(0x3F)

/**
 * COV predefined values
 */
#define MIN_COV_VALUE			2.0f
#define MAX_COV_VALUE			5.0f
#define COV_LSB_VALUE			0.05f

#define MIN_CUV_VALUE			0.7f
#define MAX_CUV_VALUE			3.3f
#define CUV_LSB_VALUE			0.1f

#define MAX_DELAY_VALUE			3100
#define DELAY_LSB_VALUE			100

#define MAX_CAL_TEMP			90.0f
#define MIN_CAL_TEMP			40.0f
#define CAL_TEMP_LSB			5.0f
#define OTT_LSB_VALUE			10
#define MAX_OTT_DELAY			2550

/**
 * @struct device_status
 * @brief struct that represents the device status register data
 */
struct device_status{
	uint8_t DRDY:1;		/**< Indicates that the data is ready to read*/
	uint8_t CBT:1;		/**< Indicates the cell balance timer is running */	
	uint8_t UVLO:1;		/**< Indicates that VBAT has fallen below UVT */
	uint8_t ECC_COR:1;	/**< Indicates that a bit error has been detected */
	uint8_t RESERVED:1; /**< Reserved */
	uint8_t ALERT:1;	/**< ALERT assert raised */
	uint8_t FAULT:1;	/**< FAULT asser raised */
	uint8_t ADDR_RQST:1;/**< Address writter to the correct address*/
};

/**
 * @struct alert_status
 * @brief struct that represents the alert status register data
 */
struct alert_status{
	uint8_t OT1:1;		/**< Indicates overtemperature on temp. sensor 1 */
	uint8_t OT2:1;		/**< Indicates overtemperature on temp. sensor 2 */	
	uint8_t SLEEP:1;	/**< Indicates that SLEEP mode has been activated */
	uint8_t TSD:1;		/**< Indicates thermal shutdown is active */
	uint8_t FORCE:1; 	/**< Asserts the ALERT signal */
	uint8_t ECC_ERR:1;	/**< Indicates validation of OTP regs */
	uint8_t PARITY:1;	/**< Indicates validation of group3 regs */
	uint8_t AR:1;		/**< Indicates that the address is valid */
};

/**
 * @struct fault_register
 * @brief struct that represents the fault status register data
 */
struct fault_status{
	uint8_t COV_BIT:1;		/**< Indicates COV detection */
	uint8_t CUV_BIT:1;		/**< Indicates CUV detection */	
	uint8_t CRC_BIT:1;		/**< Indicates garbled packet reception */
	uint8_t POR_BIT:1;		/**< Indicates power-on reset */
	uint8_t FORCE_BIT:1; 	/**< Indicates the FAULT signal */
	uint8_t I_FAULT_BIT:1;	/**< Indicates internal reg failure check*/
	uint8_t RESERVED:2;	/**< Reserved */
};

/**
 * @struct cov_fault_register
 * @brief struct that represents the cov fault status register data
 */
struct cov_fault{
	uint8_t OV1:1;		/**< Indicates cell 1 fault */
	uint8_t OV2:1;		/**< Indicates cell 2 fault */
	uint8_t OV3:1; 		/**< Indicates cell 3 fault */
	uint8_t OV4:1;		/**< Indicates cell 4 fault */
	uint8_t OV5:1;		/**< Indicates cell 5 fault */
	uint8_t OV6:1;		/**< Indicates cell 6 fault */	
	uint8_t RESERVED:2;	/**< Reserved */
};

/**
 * @struct cuv_fault_register
 * @brief struct that represents the cuv fault status register data
 */
struct cuv_fault{
	uint8_t UV1:1;		/**< Indicates cell 1 fault */
	uint8_t UV2:1;		/**< Indicates cell 2 fault */
	uint8_t UV3:1; 		/**< Indicates cell 3 fault */
	uint8_t UV4:1;		/**< Indicates cell 4 fault */
	uint8_t UV5:1;		/**< Indicates cell 5 fault */
	uint8_t UV6:1;		/**< Indicates cell 6 fault */	
	uint8_t RESERVED:2;	/**< Reserved */
};

/**
 * @struct presult_a
 * @brief struct that represents the PARITY_H (PRESULT_A (R/O)) register data
 */
struct presult_a{
	uint8_t FUNC:1;		/**< Function mode */
	uint8_t IO:1;		/**< I/O mode */
	uint8_t COVV:1;		/**< Overvoltage value */
	uint8_t COVT:1; 	/**< Overvoltage time */
	uint8_t CUVV:1;		/**< Undervoltage value */
	uint8_t CUVT:1;		/**< Undervoltage time */
	uint8_t OTV:1;		/**< Overtemp value */	
	uint8_t OTT:1;		/**< Overtemp time */
};

/**
 * @struct presult_b
 * @brief struct that represents the PARITY_H (PRESULT_B (R/O)) register data
 */
struct presult_b {
	uint8_t USER1:1;	/**< USER1 */
	uint8_t USER2:1;	/**< USER2 */
	uint8_t USER3:1;	/**< USER3 */
	uint8_t USER4:1;	/**< USER4 */
	uint8_t RESERVED:4;	/**< Reserved */
};

/**
 * @struct adc_control
 * @brief struct that represents adc control register data
 */
struct adc_control{
	uint8_t CELL_SEL:3;	/**< Selects the series cells for voltage measurement */
	uint8_t GPAI:1;		/**< Enables(1)/Disables(0) GPAI input to be measured */
	uint8_t TS:2;		/**< Selects temp sensor input to be meeasured */
	uint8_t ADC_ON:1;	/**< forces ADC subsystem ON */
	uint8_t RESERVED:1;	/**< Reserved */
};

/**
 * @enum temp_sensor_inputs
 * @brief Represents the possible values for sensor inputs, this is related to
 * 		  bit field TS on adc_control 
 * @see presult_b
 */
enum temp_sensor_inputs{
	NONE = 0b00,
	TS1 = 0b01,
	TS2 = 0b10,
	BOTH = 0b11,
};

/**
 * @enum bat_series_inputs
 * @brief Represents the possible values for cell series measurements, this is
 * related to bit fields CELL_SEL on adc_control
 * @see adc_control
 */
enum bat_series_inputs {
	CELL_1 = 0b000,
	CELL_1_2 = 0b001,
	CELL_1_3 = 0b010,
	CELL_1_4 = 0b011,
	CELL_1_5 = 0b100,
	CELL_1_6 = 0b101,
};

/**
 * @struct io_control
 * @brief struct that represents I/O control register data
 */
struct io_control {
	uint8_t TS1:1;		/**< Controls the connection to TS2 */
	uint8_t TS2:2;		/**< Controls the connection to TS2 */
	uint8_t SLEEP:1;	/**< Place the device in sleep mode */
	uint8_t RESERVED:2;	/**< Reserved */
	uint8_t GPIO_IN:1;	/**< Represents the input state of GPIO pin */
	uint8_t GPIO_OUT:1;	/**< Represents the output state of GPIO pin */
	uint8_t AUX:1;		/**< Controls the state of the AUX pin */
};

/**
 * @struct cb_ctrl
 * @brief struct that represents the cell balance output state control register
 */
struct cb_ctrl { 
	uint8_t CBAL:6;		/**< Determines if the CB(n) output is high or low */
	uint8_t RESERVED:2; /**< Reserved */
};

/**
 * @struct cb_time
 * @brief struct that represents the cell balance time register
 */
struct cb_time { 
	uint8_t CBT:6;		/**< Sets the time duration as scaled by CB[7] */
	uint8_t RESERVED:2; /**< Reserved */
	uint8_t CBCT:1;		/**< Controls minutes/seconds counting resolution */
};

/**
 * @struct address_ctrl
 * @brief struct that represents the address control register data
 */
struct address_control { 
	uint8_t ADDR_RQST:1;
	uint8_t RESERVED:1;
	uint8_t ADDR:6;		/**< sets the address for SPI comm */
};

/**
 * @struct function_config
 * @brief struct that represents the function config register data
 */
struct function_config { 
	uint8_t RESERVED_1:2;
	uint8_t CN:2;			/**< Configures the number of series cells used */
	uint8_t GPAI_SRC:1;		/**< Multiplex GPAI SRC to VBAT or output */
	uint8_t GPAI_REF:1;		/**< Sets GPAI ADC reference */
	uint8_t RESERVED_2:2;
};

/**
 * @enum series_cells
 * @brief possible values for function_config.CN
 */
enum series_cells {
	CELLS_6 = 0b00,
	CELLS_5 = 0b01,
	CELLS_4 = 0b10,
	CELLS_3 = 0b11,
};

/**
 * @struct io_config
 * @brief struct that represents the io config register data
 */
struct io_config { 
	uint8_t CRC_DIS:1;	/**< Enable/Disable the CRC generation */
	uint8_t RESERVED:6;	/**< RESERVED */		
	uint8_t CRCNOFLT:1;	/**< Enable/Disable detected CRC errors */
};

/**
 * @struct vth_config
 * @brief struct that represents the Overvoltage/Undervoltage config register 
 * 		  data
 */
struct vth_config { 
	uint8_t VTH:6;		/**< Overvoltage/Undervolage threshold */
	uint8_t RESERVED:1;	/**< RESERVED */		
	uint8_t DISABLE:1;	/**< Enable/Disable the COV/UVT function */
};

/**
 * @struct config_delay
 * @brief struct that represents the covt/cuvt config register data
 */
struct delay_config { 
	uint8_t DELAY:5;	/**< Delay time */
	uint8_t RESERVED:2;	/**< RESERVED */		
	uint8_t US_MS:1;	/**< Enable/Disable the COVT/CUVT function */
};

/**
 * @struct ot_config
 * @brief struct that represents the OT config register data
 */
struct ot_config {
	uint8_t OT1:4;
	uint8_t OT2:4;
};

#endif /* bq76pl536a_defs.h */
