// ==============================================================
// File generated by Vivado(TM) HLS - High-Level Synthesis from C, C++ and SystemC
// Version: 2018.3.0
// Copyright (C) 1986-2018 Xilinx, Inc. All Rights Reserved.
// 
// ==============================================================

/***************************** Include Files *********************************/
#include "xpacketgenerator_packetgenerator.h"

/************************** Function Implementation *************************/
#ifndef __linux__
int XPacketgenerator_packetgenerator_CfgInitialize(XPacketgenerator_packetgenerator *InstancePtr, XPacketgenerator_packetgenerator_Config *ConfigPtr) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(ConfigPtr != NULL);

    InstancePtr->Axilites_BaseAddress = ConfigPtr->Axilites_BaseAddress;
    InstancePtr->IsReady = XIL_COMPONENT_IS_READY;

    return XST_SUCCESS;
}
#endif

u32 XPacketgenerator_packetgenerator_Get_data_V_BaseAddress(XPacketgenerator_packetgenerator *InstancePtr) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    return (InstancePtr->Axilites_BaseAddress + XPACKETGENERATOR_PACKETGENERATOR_AXILITES_ADDR_DATA_V_BASE);
}

u32 XPacketgenerator_packetgenerator_Get_data_V_HighAddress(XPacketgenerator_packetgenerator *InstancePtr) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    return (InstancePtr->Axilites_BaseAddress + XPACKETGENERATOR_PACKETGENERATOR_AXILITES_ADDR_DATA_V_HIGH);
}

u32 XPacketgenerator_packetgenerator_Get_data_V_TotalBytes(XPacketgenerator_packetgenerator *InstancePtr) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    return (XPACKETGENERATOR_PACKETGENERATOR_AXILITES_ADDR_DATA_V_HIGH - XPACKETGENERATOR_PACKETGENERATOR_AXILITES_ADDR_DATA_V_BASE + 1);
}

u32 XPacketgenerator_packetgenerator_Get_data_V_BitWidth(XPacketgenerator_packetgenerator *InstancePtr) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    return XPACKETGENERATOR_PACKETGENERATOR_AXILITES_WIDTH_DATA_V;
}

u32 XPacketgenerator_packetgenerator_Get_data_V_Depth(XPacketgenerator_packetgenerator *InstancePtr) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    return XPACKETGENERATOR_PACKETGENERATOR_AXILITES_DEPTH_DATA_V;
}

u32 XPacketgenerator_packetgenerator_Write_data_V_Words(XPacketgenerator_packetgenerator *InstancePtr, int offset, int *data, int length) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr -> IsReady == XIL_COMPONENT_IS_READY);

    int i;

    if ((offset + length)*4 > (XPACKETGENERATOR_PACKETGENERATOR_AXILITES_ADDR_DATA_V_HIGH - XPACKETGENERATOR_PACKETGENERATOR_AXILITES_ADDR_DATA_V_BASE + 1))
        return 0;

    for (i = 0; i < length; i++) {
        *(int *)(InstancePtr->Axilites_BaseAddress + XPACKETGENERATOR_PACKETGENERATOR_AXILITES_ADDR_DATA_V_BASE + (offset + i)*4) = *(data + i);
    }
    return length;
}

u32 XPacketgenerator_packetgenerator_Read_data_V_Words(XPacketgenerator_packetgenerator *InstancePtr, int offset, int *data, int length) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr -> IsReady == XIL_COMPONENT_IS_READY);

    int i;

    if ((offset + length)*4 > (XPACKETGENERATOR_PACKETGENERATOR_AXILITES_ADDR_DATA_V_HIGH - XPACKETGENERATOR_PACKETGENERATOR_AXILITES_ADDR_DATA_V_BASE + 1))
        return 0;

    for (i = 0; i < length; i++) {
        *(data + i) = *(int *)(InstancePtr->Axilites_BaseAddress + XPACKETGENERATOR_PACKETGENERATOR_AXILITES_ADDR_DATA_V_BASE + (offset + i)*4);
    }
    return length;
}

u32 XPacketgenerator_packetgenerator_Write_data_V_Bytes(XPacketgenerator_packetgenerator *InstancePtr, int offset, char *data, int length) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr -> IsReady == XIL_COMPONENT_IS_READY);

    int i;

    if ((offset + length) > (XPACKETGENERATOR_PACKETGENERATOR_AXILITES_ADDR_DATA_V_HIGH - XPACKETGENERATOR_PACKETGENERATOR_AXILITES_ADDR_DATA_V_BASE + 1))
        return 0;

    for (i = 0; i < length; i++) {
        *(char *)(InstancePtr->Axilites_BaseAddress + XPACKETGENERATOR_PACKETGENERATOR_AXILITES_ADDR_DATA_V_BASE + offset + i) = *(data + i);
    }
    return length;
}

u32 XPacketgenerator_packetgenerator_Read_data_V_Bytes(XPacketgenerator_packetgenerator *InstancePtr, int offset, char *data, int length) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr -> IsReady == XIL_COMPONENT_IS_READY);

    int i;

    if ((offset + length) > (XPACKETGENERATOR_PACKETGENERATOR_AXILITES_ADDR_DATA_V_HIGH - XPACKETGENERATOR_PACKETGENERATOR_AXILITES_ADDR_DATA_V_BASE + 1))
        return 0;

    for (i = 0; i < length; i++) {
        *(data + i) = *(char *)(InstancePtr->Axilites_BaseAddress + XPACKETGENERATOR_PACKETGENERATOR_AXILITES_ADDR_DATA_V_BASE + offset + i);
    }
    return length;
}
