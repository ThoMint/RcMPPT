/*
 * TMCL-Defines.h
 *
 *  Created on: 01.04.2020
 *      Author: OK / ED
 */

#ifndef TMCL_DEFINES_H
#define TMCL_DEFINES_H

//START HOST INTERFACE COMMON DEFINES

// operation codes
#define OPCODE_REPORT_LIVE_VALUE          10
#define OPCODE_REPORT_AVERAGE_VALUE       11
#define OPCODE_FINISHED_SAMPLE            12
#define OPCODE_SET_SETTING                13

// operation types
#define OPTYPE_OUTPUT_VOLTAGE          1
#define OPTYPE_INPUT_VOLTAGE           2
#define OPTYPE_TARGET_OUTPUT_VOLTAGE   3
#define OPTYPE_TARGET_OUTPUT_CURRENT   4
#define OPTYPE_TARGET_SAMPLE_RATE      5
#define OPTYPE_SAMPLE_NUM_OF_CHANNELS  6

// status codes
#define OPSTATUS_OK                                 100
#define OPSTATUS_CMD_LOADED                         101
#define OPSTATUS_CHKERR                             1
#define OPSTATUS_INVALID_CMD                        2
#define OPSTATUS_WRONG_TYPE                         3
#define OPSTATUS_INVALID_VALUE                      4
#define OPSTATUS_EEPROM_LOCKED                      5
#define OPSTATUS_CMD_NOT_AVAILABLE                  6
#define OPSTATUS_CMD_LOAD_ERROR                     7
#define OPSTATUS_WRITE_PROTECTED                    8
#define OPSTATUS_MAX_EXCEEDED                       9
#define OPSTATUS_DOWNLOAD_NOT_POSSIBLE              10
#define OPSTATUS_DELAYED                            128
#define OPSTATUS_ACTIVE_COMM                        129

#define DEVICE_CMD_QUEUE_SIZE 10

// TMCL host/device command
typedef struct
{
	uint8_t Status;
	uint8_t Opcode;
	uint8_t Type;
	union
	{
		uint32_t Int32;
		uint8_t Byte[4];
	} Value;
} HostCommand, DeviceCommand;

//END HOST INTERFACE COMMON DEFINES

#endif /* TMCL_DEFINES_H_ */
