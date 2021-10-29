/*
 * TMCL-Variables.h
 *
 *  Created on: 01.04.2020
 *      Author: OK / ED
 */

#ifndef TMCL_VARIABLES_H
#define TMCL_VARIABLES_H

#include <hostInterfaceDefines.h>

volatile HostCommand ActualHostCMD;
volatile DeviceCommand ActualDeviceCMD;

uint8_t USBHostCMD[APP_RX_DATA_SIZE];
uint8_t USBDeviceCMD[APP_TX_DATA_SIZE];

volatile DeviceCommand DeviceCMDQueue[DEVICE_CMD_QUEUE_SIZE];
volatile uint8_t deviceCMDsToProcess;
volatile int deviceCMDProcessPos, deviceCMDPopulatePos;

#endif /* TMCL_VARIABLES_H */
