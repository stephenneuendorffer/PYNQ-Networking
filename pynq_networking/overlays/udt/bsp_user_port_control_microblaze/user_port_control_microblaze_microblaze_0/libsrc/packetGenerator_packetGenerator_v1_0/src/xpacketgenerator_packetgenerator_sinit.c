// ==============================================================
// File generated by Vivado(TM) HLS - High-Level Synthesis from C, C++ and SystemC
// Version: 2018.3.0
// Copyright (C) 1986-2018 Xilinx, Inc. All Rights Reserved.
// 
// ==============================================================

#ifndef __linux__

#include "xstatus.h"
#include "xparameters.h"
#include "xpacketgenerator_packetgenerator.h"

extern XPacketgenerator_packetgenerator_Config XPacketgenerator_packetgenerator_ConfigTable[];

XPacketgenerator_packetgenerator_Config *XPacketgenerator_packetgenerator_LookupConfig(u16 DeviceId) {
	XPacketgenerator_packetgenerator_Config *ConfigPtr = NULL;

	int Index;

	for (Index = 0; Index < XPAR_XPACKETGENERATOR_PACKETGENERATOR_NUM_INSTANCES; Index++) {
		if (XPacketgenerator_packetgenerator_ConfigTable[Index].DeviceId == DeviceId) {
			ConfigPtr = &XPacketgenerator_packetgenerator_ConfigTable[Index];
			break;
		}
	}

	return ConfigPtr;
}

int XPacketgenerator_packetgenerator_Initialize(XPacketgenerator_packetgenerator *InstancePtr, u16 DeviceId) {
	XPacketgenerator_packetgenerator_Config *ConfigPtr;

	Xil_AssertNonvoid(InstancePtr != NULL);

	ConfigPtr = XPacketgenerator_packetgenerator_LookupConfig(DeviceId);
	if (ConfigPtr == NULL) {
		InstancePtr->IsReady = 0;
		return (XST_DEVICE_NOT_FOUND);
	}

	return XPacketgenerator_packetgenerator_CfgInitialize(InstancePtr, ConfigPtr);
}

#endif
