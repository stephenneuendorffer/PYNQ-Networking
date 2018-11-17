
 PARAMETER VERSION = 2.2.0


BEGIN OS
 PARAMETER OS_NAME = standalone
 PARAMETER OS_VER = 6.7
 PARAMETER PROC_INSTANCE = user_port_control_microblaze_microblaze_0
 PARAMETER stdin = user_port_control_microblaze_lmb_bram_if_cntlr_0
 PARAMETER stdout = user_port_control_microblaze_lmb_bram_if_cntlr_0
END


BEGIN PROCESSOR
 PARAMETER DRIVER_NAME = cpu
 PARAMETER DRIVER_VER = 2.7
 PARAMETER HW_INSTANCE = user_port_control_microblaze_microblaze_0
 PARAMETER compiler_flags =  -mlittle-endian -mxl-barrel-shift -mno-xl-soft-mul -mhard-float -mxl-float-convert -mxl-float-sqrt -mno-xl-soft-div -mcpu=v10.0
END


BEGIN DRIVER
 PARAMETER DRIVER_NAME = intrgpio
 PARAMETER DRIVER_VER = 4.1
 PARAMETER HW_INSTANCE = user_port_control_microblaze_axi_gpio_0
END

BEGIN DRIVER
 PARAMETER DRIVER_NAME = wdttb
 PARAMETER DRIVER_VER = 4.3
 PARAMETER HW_INSTANCE = user_port_control_microblaze_axi_timebase_wdt_0
END

BEGIN DRIVER
 PARAMETER DRIVER_NAME = mailbox_bram
 PARAMETER DRIVER_VER = 0.1
 PARAMETER HW_INSTANCE = user_port_control_microblaze_lmb_bram_if_cntlr_0
END

BEGIN DRIVER
 PARAMETER DRIVER_NAME = packetGenerator_packetGenerator
 PARAMETER DRIVER_VER = 1.0
 PARAMETER HW_INSTANCE = user_port_control_packetGenerator
END

BEGIN DRIVER
 PARAMETER DRIVER_NAME = mailbox_bram
 PARAMETER DRIVER_VER = 0.1
 PARAMETER HW_INSTANCE = user_port_egress_blocks_lmb_bram_if_cntlr_1
END

BEGIN DRIVER
 PARAMETER DRIVER_NAME = generic
 PARAMETER DRIVER_VER = 2.0
 PARAMETER HW_INSTANCE = user_port_egress_path_egress_buffer_reader_0
END

BEGIN DRIVER
 PARAMETER DRIVER_NAME = generic
 PARAMETER DRIVER_VER = 2.0
 PARAMETER HW_INSTANCE = user_port_egress_path_egress_buffer_writer_0
END

BEGIN DRIVER
 PARAMETER DRIVER_NAME = mailbox_bram
 PARAMETER DRIVER_VER = 0.1
 PARAMETER HW_INSTANCE = user_port_ingress_blocks_lmb_bram_if_cntlr_1
END

BEGIN DRIVER
 PARAMETER DRIVER_NAME = generic
 PARAMETER DRIVER_VER = 2.0
 PARAMETER HW_INSTANCE = user_port_ingress_path_ingress_buffer_reader_0
END

BEGIN DRIVER
 PARAMETER DRIVER_NAME = generic
 PARAMETER DRIVER_VER = 2.0
 PARAMETER HW_INSTANCE = user_port_ingress_path_ingress_buffer_writer_0
END


BEGIN LIBRARY
 PARAMETER LIBRARY_NAME = pynqmb
 PARAMETER LIBRARY_VER = 1.0
 PARAMETER PROC_INSTANCE = user_port_control_microblaze_microblaze_0
END


