#   Copyright (c) 2017, Xilinx, Inc.
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


import struct
import logging
logging.getLogger("scapy.runtime").setLevel(logging.ERROR)
from scapy.all import *


__author__ = "Stephen Neuendorffer"
__copyright__ = "Copyright 2018, Xilinx"
__email__ = "stephenn@xilinx.com"


""" Scapy dissector definitions for UDT packets.

"""


class UDT_DATA(Packet):
    fields_desc = [
        BitField("flags", 0, 2),
        BitField("inOrder", 1, 1),
        BitField("messageNo", 0, 29),

        IntField("timeStamp", 0),
        IntField("destSocket", 0),
    ]

udt_control_fields_desc = [
        #BitField("isControl", 1, 1),
        #BitField("type", 0, 15),
        #ShortField("reserved", 0),
        
        BitField("t", 0, 3),
        BitField("info", 0, 29),

        IntField("timeStamp", 0),
        IntField("destSocket", 0),
]

class UDT_HANDSHAKE(Packet):
    type = 0x0
    name = "HANDSHAKE"
    fields_desc = [
        BitField("t", 0, 3),
        BitField("info", 0, 29),
        IntField("timeStamp", 0),
        IntField("destSocket", 0),

        IntField("udtVersion", 4),
        IntField("udtType", 0), #ENUM
        IntField("initialSeqNo", 0),
        IntField("maximumPacketSize", 1522),
        IntField("maximumWindowSize", 64),
        IntField("connectionType", 0), # ENUM
        IntField("socket", 0),
        IntField("cookie", 0),
        IPField("ip", "127.0.0.1")
    ]

class UDT_KEEPALIVE(Packet):
    type = 0x01
    fields_desc = [
        BitField("t", 0, 3),
        BitField("info", 0, 29),
        IntField("timeStamp", 0),
        IntField("destSocket", 0)]
    
class UDT_ACK(Packet):
    type = 0x02
    name = "ACK"
    fields_desc = [
        IntField("ackseq", 0),
        IntField("timeStamp", 0),
        IntField("destSocket", 0),
        IntField("seqNo",0),
        IntField("RTT",0),
        IntField("RTTVariance",0),
        IntField("bufferAvailable",0),
        IntField("receiveRate",0),
        IntField("linkCapacity",0)]


class UDT_NAK(Packet):
    type = 0x03
    name = "NAK"
    fields_desc = [
        IntField("info", 0),
        IntField("timeStamp", 0),
        IntField("destSocket", 0)
    ]

class UDT_SHUTDOWN(Packet):
    type = 0x05
    name = "SHUTDOWN"
    fields_desc = [
        IntField("info", 0),
        IntField("timeStamp", 0),
        IntField("destSocket", 0)]

class UDT_ACK2(Packet):
    type = 0x06
    name = "ACK2"
    fields_desc = [
        IntField("ackseq", 0),
        IntField("timeStamp", 0),
        IntField("destSocket", 0)]

class UDT_DROPREQ(Packet):
    type = 0x07
    name = "DROPREQ"
    fields_desc = [
        IntField("messageID", 0),
        IntField("timeStamp", 0),
        IntField("destSocket", 0),
        IntField("firstSeqNo", 0),
        IntField("lastSeqNo", 0)]

class UDT_USER(Packet):
    type = 0x7FFF
    name = "SUBSCRIBE"
    fields_desc = [
        IntField("t", 0),
        IntField("timeStamp", 0),
        IntField("destSocket", 0)]

class UDT_UNKNOWN(Packet):
    name = "UNKNOWN"
    fields_desc = []
    
class UDT(Packet):
    fields_desc = [
        BitField("isControl", 0, 1),
        BitField("seqNo", 0, 31)
    ]
    def guess_payload_class(self, payload):
        #print("guessing ", self)
        type = self.seqNo >> 16 
        if self.isControl == 0:
            return UDT_DATA
        elif type == 0:
            return UDT_HANDSHAKE
        elif type == 1:
            return UDT_KEEPALIVE
        elif type == 2:
            return UDT_ACK
        elif type == 3:
            return UDT_NAK
        elif type == 5:
            return UDT_SHUTDOWN
        elif type == 6:
            return UDT_ACK2
        elif type == 7:
            return UDT_MESSAGEDROP
        elif type == 0x7FFF:
            return UDT_USER
        else:
            return UDT_UNKNOWN #Packed.guess_payload_class(self, payload)
    
    
#    def answers(self, other):
#        """Returns true if self is an answer from other. 
#        Assume that any MQTT packet is an answer for any other.
#        This is slightly inaccurate because only some packets
#        are actually responses, but it's good enough for now.
#        """
#        if other.__class__ == self.__class__:
#            return 1
#        return 0



