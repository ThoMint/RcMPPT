/*
 * host_interface.h
 *
 *  Created on: Oct 6, 2021
 *      Author: Thomas
 */

#ifndef INC_HOSTINTERFACE_H_
#define INC_HOSTINTERFACE_H_

#include "main.h"
#include "usb_device.h"
#include "usbd_cdc.h"
#include "usbd_cdc_if.h"
#include "hostInterfaceDefines.h"
#include "hostInterfaceVariables.h"


void hostInterfaceProcessCommand();
void hostInterfaceQueueDeviceCMD(DeviceCommand cmd);
void hostInterfaceQueueDeviceCMDExpl(uint8_t Status, uint8_t Opcode, uint8_t Type, int Int32);

#endif /* INC_HOSTINTERFACE_H_ */
