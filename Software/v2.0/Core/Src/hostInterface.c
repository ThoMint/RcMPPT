/*
 * host_interface.c
 *
 *  Created on: Oct 6, 2021
 *      Author: Thomas
 */

#include "hostInterface.h"

void hostInterfaceExecuteActualCommand()
{
	switch (ActualHostCMD.Opcode)
	{
	case OPCODE_SET_SETTING:
		switch (ActualHostCMD.Type)
		{
		case OPTYPE_TARGET_OUTPUT_VOLTAGE:
			targetVout = ActualHostCMD.Value.Int32;
			break;
		case OPTYPE_TARGET_OUTPUT_CURRENT:
			targetIout = ActualHostCMD.Value.Int32;
			break;
		case OPTYPE_TARGET_SAMPLE_RATE:
			targetSampleDelay = 1000000 / ActualHostCMD.Value.Int32;
			break;
		}
		break;
	}
}

void hostInterfaceQueueDeviceCMDExpl(uint8_t Status, uint8_t Opcode, uint8_t Type, uint32_t Int32)
{
	DeviceCommand cmd;
	cmd.Status = Status;
	cmd.Opcode = Opcode;
	cmd.Type = Type;
	cmd.Value.Int32 = Int32;
	hostInterfaceQueueDeviceCMD(cmd);
}

void hostInterfaceQueueDeviceCMD(DeviceCommand cmd)
{
	deviceCMDsToProcess = 1;
	DeviceCMDQueue[deviceCMDPopulatePos] = cmd;
	deviceCMDPopulatePos++;
	if (deviceCMDPopulatePos >= DEVICE_CMD_QUEUE_SIZE)
	{
		deviceCMDPopulatePos = 0;
	}
}

void hostInterfaceProcessCommand()
{
	uint32_t CMDLen;

	if (deviceCMDsToProcess && !USB_getTxState())
	{
		int index = deviceCMDProcessPos;
		ActualDeviceCMD = DeviceCMDQueue[index];
		deviceCMDProcessPos++;
		if (deviceCMDProcessPos >= DEVICE_CMD_QUEUE_SIZE)
		{
			deviceCMDProcessPos = 0;
		}

		if (deviceCMDProcessPos == deviceCMDPopulatePos)
		{
			deviceCMDsToProcess = 0;
		}

		uint8_t Checksum = ActualDeviceCMD.Status + ActualDeviceCMD.Opcode + ActualDeviceCMD.Type + ActualDeviceCMD.Value.Byte[0] + ActualDeviceCMD.Value.Byte[1] + ActualDeviceCMD.Value.Byte[2] + ActualDeviceCMD.Value.Byte[3];

		USBDeviceCMD[0] = ActualDeviceCMD.Status;
		USBDeviceCMD[1] = ActualDeviceCMD.Opcode;
		USBDeviceCMD[2] = ActualDeviceCMD.Type;
		USBDeviceCMD[3] = ActualDeviceCMD.Value.Byte[0];
		USBDeviceCMD[4] = ActualDeviceCMD.Value.Byte[1];
		USBDeviceCMD[5] = ActualDeviceCMD.Value.Byte[2];
		USBDeviceCMD[6] = ActualDeviceCMD.Value.Byte[3];
		USBDeviceCMD[7] = Checksum;

		CDC_Transmit_FS(USBDeviceCMD, 8);
	}

	// last command was a reset?

	/* read next request */
	while (USB_retrieveCMD(USBHostCMD, &CMDLen))
	{
		if (CMDLen == 8)
		{
			uint8_t checksum = 0;
			for (uint8_t i = 0; i < 7; i++)
				checksum += USBHostCMD[i];

			if (checksum == USBHostCMD[7])  // check checksum
			{
				ActualHostCMD.Status = USBHostCMD[0];
				ActualHostCMD.Opcode = USBHostCMD[1];
				ActualHostCMD.Type = USBHostCMD[2];
				ActualHostCMD.Value.Byte[0] = USBHostCMD[3];
				ActualHostCMD.Value.Byte[1] = USBHostCMD[4];
				ActualHostCMD.Value.Byte[2] = USBHostCMD[5];
				ActualHostCMD.Value.Byte[3] = USBHostCMD[6];
				hostInterfaceExecuteActualCommand();
			}
		}
	}
}
