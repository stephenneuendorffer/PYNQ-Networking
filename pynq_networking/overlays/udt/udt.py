#   Copyright (c) 2018, Xilinx, Inc.
#   All rights reserved.
#
#   Redistribution and use in source and binary forms, with or without
#   modification, are permitted provided that the following conditions are met:
#
#   1.  Redistributions of source code must retain the above copyright notice,
#       this list of conditions and the following disclaimer.
#
#   2.  Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#
#   3.  Neither the name of the copyright holder nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
#   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
#   AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
#   THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
#   PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
#   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
#   EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
#   PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
#   OR BUSINESS INTERRUPTION). HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
#   WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
#   OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
#   ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


import pynq
import pynq.lib
from pynq.lib import PynqMicroblaze
from pynq.lib.pynqmicroblaze import add_bsp
from pynq.overlay import DefaultIP
from pynq.mmio import MMIO
import logging
logging.getLogger("scapy.runtime").setLevel(logging.ERROR)
from scapy.all import *
from scapy.utils import inet_aton, inet_ntoa, mac2str, str2mac
import numpy as np

__author__ = "Stephen Neuendorffer"
__copyright__ = "Copyright 2018, Xilinx"
__email__ = "stephenn@xilinx.com"

class HLSIP(DefaultIP):
    def __init__(self, description, addressMap):
        super().__init__(description)
        self.addressMap = addressMap

class Register():
    """ An Object representing a register location within an MMIO region.
        By default a register is four bytes, but can be any length """
    def __init__(self, mmio, offset, len=4):
        self.mmio = mmio
        self.offset = offset
        self.len = len

    def readBytes(self):
        return self.mmio.read(self.offset)
    
    def writeBytes(self, x):
        assert(len(x) == self.len)
        self.mmio.write(self.offset,x)

    def read(self):
        return self.mmio.read(self.offset)

    def write(self, x):
        self.mmio.write(self.offset, x)
        #t = x
        #words = (self.len+3)//4
        #for i in range(words):
        #    self.mmio.write(self.offset + 4*i, t & 0xFFFFFFFF)
        #    t = t >> 32
            
class arp_ingress(HLSIP):
    """Driver for the arp_ingress IP

    Attributes
    ----------
    self.regs : MMIO
        Access to control registers.
    """
    
    def __init__(self, description):                                     
        """Return a new Audio object based on the hierarchy description.

        Parameters  
        ----------  
        description : dict
            The hierarchical description of the hierarchy
                                                         
        """                                              
        super().__init__(description, {'data':(0x20000, 16384, 64)})
        base_addr = description['phys_addr']

        self.regs = MMIO(base_addr, 1024)
        self.macAddress_reg = Register(self.regs, 0x10, len=8)
        self.ipAddress_reg = Register(self.regs, 0x1C)
        
    def init(self,macAddress_str,ipAddress_str):
        # Note that scapy uses these fields in network byte order, while
        # we assume they are in host byte order.
        macAddress = bytearray(mac2str(macAddress_str)[::-1])
        macAddress += b'\x00\x00'
        ipAddress = inet_aton(ipAddress_str)[::-1]
        #print(macAddress, ipAddress)
        
        self.macAddress_reg.writeBytes(bytes(macAddress))
        self.ipAddress_reg.writeBytes(bytes(ipAddress))
        
    bindto = ['xilinx.com:hls:arp_ingress_arp_ingress:1.0',
              'xilinx.com:hls:arp_egress_arp_egress:1.0']

class PacketRecorder(HLSIP):
    """Driver for the PacketRecorder IP

    Attributes
    ----------
    self.regs : MMIO
        Access to control registers.
    self.reset_reg : Register
    self.packetCount_reg : Register
    self.data : MMIO
        Access to data buffer.
    """
    
    def __init__(self, description):                                     
        """Return a new Audio object based on the hierarchy description.

        Parameters  
        ----------  
        description : dict
            The hierarchical description of the hierarchy
                                                         
        """                                              
        super().__init__(description, {'data':(0x20000, 16384, 64)})
        base_addr = description['phys_addr']

        self.regs = MMIO(base_addr, 1024)
        self.data = MMIO(base_addr+0x20000, 1024*16*8)
        self.reset_reg = Register(self.regs, 0x10)
        self.packetCount_reg = Register(self.regs, 0x18)
        
    def dumplist(self):
        ### Return the contents of the packet buffer as a scapy PacketList
        a=[]
        n = self.packetCount_reg.read()
        print("count = ", n)
        if(n > 1024):
            n = 1024 # we record at most 1024 packets
        if(n > 0):
          for j in range(n):
            to_read = 16*8 # from recorder parameters.  16 data beats * 8 bytes
            i = 0
            pkt = b""
            while to_read>4:
                data = self.data.read(j*16*8+i)
                pkt = pkt + data.to_bytes(4, byteorder='little')
                to_read = to_read - 4
                i = i + 4
            data = self.data.read(j*16*8+i)
            remainder = data.to_bytes(4, byteorder='little')
            remainder = remainder[0:to_read]
            pkt = pkt + remainder
            
            e = Ether(pkt)
            a.append(e)
            
        return PacketList(a)
                                                                               
    def reset(self):
        ### Reset the recorder. Empty the buffer and start recording again.
        self.reset_reg.write(1)
        self.reset_reg.write(0)
        
    bindto = ['xilinx.com:hls:packetRecorder_packetRecorder:1.0']

class PacketGenerator(HLSIP):
    def __init__(self, description):                                     
        super().__init__(description,{'data':(0x2000, 1024, 64)})
        base_addr = description['phys_addr']
        self.data = MMIO(base_addr+0x2000, 1024*64//8)
        self.tx_data = MMIO(base_addr+0x2000, 0x200*8)
        self.rx_data = MMIO(base_addr+0x2000+0x200*8, 0x200*8)

        self.LL = IP
        self.EN_OFFSET = 0x190*8
        self.LEN_OFFSET = 0x194*8
        # transmit offsets
        self.TX_DATA_OFFSET = 0x000
        self.TX_CTRL_OFFSET = 0x180
        self.TX_EN_OFFSET = 0x190
        self.TX_LEN_OFFSET = 0x194
        # receive offsets
        self.RX_DATA_OFFSET = 0x200
        self.RX_CTRL_OFFSET = 0x380
        self.RX_EN_OFFSET = 0x390
        self.RX_LEN_OFFSET = 0x394

    def read32(self, i):
        return self.data.read(i << 3)

    def write32(self, i, x):
        self.data.write(i >> 3, x)
    
    def has_packet(self):
        t = self.rx_data.read(self.EN_OFFSET)
        return t == 1

    def recv(self, format=Raw):
        if self.has_packet():
            to_read = self.rx_data.read(self.LEN_OFFSET)
            print("Received packet with ", to_read, "bytes");
            i = 0
            pkt = b""
            while to_read>4:
                data = self.rx_data.read(i*4)
                pkt = pkt + data.to_bytes(4, byteorder='little')
                to_read = to_read - 4
                i = i + 1
            data = self.rx_data.read(i*4)
            remainder = data.to_bytes(4, byteorder='little')
            remainder = remainder[0:to_read]
            pkt = pkt + remainder
            self.rx_data.write(self.EN_OFFSET, 0x00)
            return format(pkt)
        return None

    def recvmultiple(self):
        l = []
        while self.has_packet():
            to_read = self.rx_data.read(self.LEN_OFFSET)
            i = 0
            pkt = b""
            while to_read>4:
                data = self.rx_data.read(i*4)
                pkt = pkt + data.to_bytes(4, byteorder='little')
                to_read = to_read - 4
                i = i + 1
            data = self.rx_data.read(i*4)
            remainder = data.to_bytes(4, byteorder='little')
            remainder = remainder[0:to_read]
            pkt = pkt + remainder
            self.rx_data.write(self.EN_OFFSET, 0x00)
            l.append(self.LL(pkt))
        return PacketList(l)

    def sendnomin(self, packet):
        i = 0
        for b in bytes(packet):
            self.tx_data.mem[i] = b
            i = i+1
        #self.data.mem.write(packet);
        self.tx_data.write(self.LEN_OFFSET, len(packet))
        self.tx_data.write(self.EN_OFFSET, 0x1)
        
    def send(self, packet):
        # FIXME: remove this
        minLength = 50
        if len(packet)<minLength:
            zeros = b'\x00'*(minLength - len(packet))
            packet = packet/Padding(zeros)        
        assert(len(packet) >= 50) # Minimum size Ethernet frame.
        #self.tx_data.mem.seek(0x0)
        i = 0
        for b in bytes(packet):
            self.tx_data.mem[i] = b
            i = i+1
        #self.data.mem.write(packet);
        self.tx_data.write(self.LEN_OFFSET, len(packet))
        self.tx_data.write(self.EN_OFFSET, 0x1)
        
    def dump(self):
        for i in range(64):
            print(hex(self.rx_data.read(i*4)))            
        for i in range(64):
            print(hex(self.tx_data.read(i*4)))

    bindto = ['xilinx.com:hls:packetGenerator_packetGenerator:1.0']

class XXVEthernet(DefaultIP):
    def __init__(self, description):                                     
        super().__init__(description)

    def capture_stats(self):
        self.mmio.write(0x20,1) # Write PM_TICK

    def setBit(self, address, bit):
        self.mmio.write(address, self.mmio.read(address) | (1 << bit))

    def init(self):
        while(self.reinit()):
            None
            
    def reinit(self):
        # Initialize HW time stamping
        # if (is_tsn)
        # reset TXTS_RDFR
        # reset TXTS_SRR
        self.setBit(0x14, 1) # Enable FCS stripping on receive packets
        self.setBit(0xC, 1) # Turn on FCS insertion on transmit packets
        self.setBit(0xC, 0) # Enable TX
        self.setBit(0x14, 0) # Enable RX

        # MAC initialization taken from the linux xxv_ethernet driver
        block_lock = self.mmio.read(0x40C)
        if(block_lock == 0):
            print("XXVEthernet: Waiting for block lock...")
        else:
            RX_MTU = self.mmio.read(0x18) >> 16
            print("XXVEthernet: Locked! RX MTU is ", RX_MTU)
        
        return (block_lock == 0)

    def dump_stats(self):
        
        def dump_count(name, offset):
            ### Dump a 2-byte count with LSB at offset
            print(name, self.mmio.read(offset) + (self.mmio.read(offset+4) << 32))
              
        print("TX_STATUS_REG1", self.mmio.read(0x400))
        dump_count("TX_TOTAL_PACKETS", 0x700)
        dump_count("TX_TOTAL_GOOD_PACKETS",0x708)
        dump_count("TX_TOTAL_BYTES", 0x710)
        dump_count("TX_TOTAL_GOOD_BYTES", 0x718)
        dump_count("       Size <=  64 Bytes", 0x720)
        dump_count(" 65 <= Size <= 127 Bytes", 0x728)
        dump_count("128 <= Size <= 255 Bytes", 0x730)
        dump_count("256 <= Size <= 511 Bytes", 0x738)
        dump_count("512 < Size <= 1023 Bytes", 0x740)
        dump_count("1024 < Size <= 1518 Bytes", 0x748)
        dump_count("1519 < Size <= 1522 Bytes", 0x750)
        dump_count("1523 < Size <= 1548 Bytes", 0x758)
        dump_count("1549 < Size <= 2047 Bytes", 0x760)
        dump_count("2048 < Size <= 4095 Bytes", 0x768)
        dump_count("4096 < Size <= 8191 Bytes", 0x770)
        dump_count("8192 < Size <= 9215 Bytes", 0x778)

        print()
        print("RX_STATUS_REG1", self.mmio.read(0x404))
        dump_count("RX_TOTAL_PACKETS", 0x808)
        dump_count("RX_TOTAL_GOOD_PACKETS", 0x810)
        dump_count("RX_TOTAL_BYTES", 0x818)
        dump_count("RX_TOTAL_GOOD_BYTES", 0x820)
        dump_count("       Size <=  64 Bytes", 0x828)
        dump_count(" 65 <= Size <= 127 Bytes", 0x830)
        dump_count("128 <= Size <= 255 Bytes", 0x838)
        dump_count("256 <= Size <= 511 Bytes", 0x840)
        dump_count("512 < Size <= 1023 Bytes", 0x848)
        dump_count("1024 < Size <= 1518 Bytes", 0x850)
        dump_count("1519 < Size <= 1522 Bytes", 0x858)
        dump_count("1523 < Size <= 1548 Bytes", 0x860)
        dump_count("1549 < Size <= 2047 Bytes", 0x868)
        dump_count("2048 < Size <= 4095 Bytes", 0x870)
        dump_count("4096 < Size <= 8191 Bytes", 0x878)
        dump_count("8192 < Size <= 9215 Bytes", 0x880)
        
    bindto = ['xilinx.com:ip:xxv_ethernet:2.3',
              'xilinx.com:ip:xxv_ethernet:2.4']
    
class UDTOverlay(pynq.Overlay):
    """ The MQTTSN overlay.

    This overlay is designed to read the Pmod TMP2 sensor values and publish 
    them using MQTTSN protocol. PL acceleration is used on this overlay.

    Normally the constant PMODB can be extracted by the overlay directly. 
    In this notebook, however, we will manually define it to be compatible 
    with PYNQ image_v2.0 release. The definition of PMODB in this example also 
    omits the interrupt pins for simplicity.

    Attributes
    ----------
    leds : AxiGPIO
         4-bit output GPIO for interacting with the green LEDs LD0-3
    buttons : AxiGPIO
         4-bit input GPIO for interacting with the buttons BTN0-3
    switches : AxiGPIO
         2-bit input GPIO for interacting with the switches SW0 and SW1
    rgbleds : [pynq.board.RGBLED]
         Wrapper for GPIO for LD4 and LD5 multicolour LEDs

    """
    def __init__(self, bitfile, **kwargs):
        super().__init__(bitfile, **kwargs)

        if self.is_loaded():
            self.packetGenerator = self.user_port.udt_packetGenerator
            self.packetRecorder_user_ingress = self.user_port.packetRecorder_packetRecorder_1
            self.packetRecorder_user_egress = self.user_port.packetRecorder_packetRecorder_2
            self.packetRecorder_ingress = self.processor_dma_port.packetRecorder_packetRecorder_0
            self.packetRecorder_egress = self.processor_dma_port.packetRecorder_packetRecorder_1
            self.arp_ingress = self.user_port.arp_ingress_arp_ingress_0
            self.arp_egress = self.user_port.arp_egress_arp_egress_0

            #self.user_port.control_microblaze.mbtype = "Pmod"
            #self.iop_pmodb.mbtype = "Pmod"
            #self.iop_arduino.mbtype = "Arduino"

            #self.control_mb = self.user_port.control_microblaze.mb_info
            #print(self.control_mb)
            #self.PMODB = self.iop_pmodb.mb_info
            #self.ARDUINO = self.iop_arduino.mb_info

            #self.audio = self.audio_direct_0
            #self.leds = self.leds_gpio.channel1
            #self.switches = self.switches_gpio.channel1
            #self.buttons = self.btns_gpio.channel1
            #self.leds.setlength(4)
            #self.switches.setlength(2)
            #self.buttons.setlength(4)
            #self.leds.setdirection("out")
            #self.switches.setdirection("in")
            #self.buttons.setdirection("in")
            #self.rgbleds = ([None] * 4) + [pynq.lib.RGBLED(i)
            #                               for i in range(4, 6)]
            None


BIN_LOCATION = os.path.dirname(os.path.realpath(__file__)) + "/"
BSP_LOCATION = os.path.join(BIN_LOCATION, "bsp_user_port_control_microblaze")
add_bsp(BSP_LOCATION)
