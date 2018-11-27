#pragma once
#include "ap_int.h"
#include "hls_stream.h"

//#include "eth_interface.h"
#include "ip.hpp"
//#include "cam.h"
#include "udt.h"

// struct arpentry {
//     struct ipaddress ip;
//     struct ethaddress eth;
// };

//#define RANDOM_LOSS

typedef ap_axiu<8*BYTESPERCYCLE,1,1,1> StreamType;

typedef ap_uint<13> MessageID;
typedef ap_uint<29> SequenceID;

//typedef hls::algorithmic_cam<256, 4, IPAddressT, MACAddressT> ArpCacheT;
//typedef hls::cam<4, IPAddressT, MACAddressT> ArpCacheT;

struct arpcache_insert_args {
    ap_uint<32> ip;
    ap_uint<48> mac;
};
typedef int arpcache_insert_return;
//struct arpcache_insert_return {
//    int dummy;
//};
void udt_ingress(ap_uint<32> ipAddress,
                 hls::stream<StreamType> &dataIn, hls::stream<StreamType> &dataOut);

void udt_egress(ap_uint<48> macAddress, ap_uint<32> ipAddress,
                hls::stream<StreamType> &dataIn, hls::stream<StreamType> &dataOut,
                hls::stream<arpcache_insert_args> &arpcache_insert_start, hls::stream<arpcache_insert_return> &arpcache_insert_done);
//,
//              ArpCacheT &arpcache);
void egress_buffer_writer(hls::stream<ap_uint<32> > &params,
                          hls::stream<StreamType> &in,    // input
                   //hls::stream<short> &input_length_stream, // input
                   //hls::stream<bufferIDT> &buffer_id_stream, // input
                   ap_uint<8*BYTESPERCYCLE> buffer_storage[2048/BYTESPERCYCLE][BUFFERCOUNT], // written
                   struct Block blocks[BUFFERCOUNT]
                          );
void egress_buffer_reader(hls::stream<StreamType> &in,    // input
                          hls::stream<StreamType> &out,    // output
                          hls::stream<ap_uint<32> > &params,
                          hls::stream<int> &control,
                          //ap_uint<32> ipAddress,
                          //ap_uint<32> ipDestAddress,
                          //int buffer_descriptor,
                          ap_uint<8*BYTESPERCYCLE> buffer_storage[2048/BYTESPERCYCLE][BUFFERCOUNT], // read
                          struct Block blocks[BUFFERCOUNT]
                          );
void ingress_buffer_writer(hls::stream<StreamType> &in,    // input
                           hls::stream<StreamType> &out,    // output
                           hls::stream<int> &params,
                           hls::stream<int> &buffer_id, // output
                           //int enable,
                           //int iRcvLastAck,
                           //hls::stream<short> &input_length_stream, // input
                           //hls::stream<bufferIDT> &buffer_id_stream, // input
                           ap_uint<8*BYTESPERCYCLE> buffer_storage[2048/BYTESPERCYCLE][BUFFERCOUNT], // written
                           struct Block blocks[BUFFERCOUNT]
                           );
void ingress_buffer_reader(hls::stream<ap_uint<32> > &params,
                           hls::stream<StreamType> &out,    // output
                           ap_uint<8*BYTESPERCYCLE> buffer_storage[2048/BYTESPERCYCLE][BUFFERCOUNT], // read
                           struct Block blocks[BUFFERCOUNT]
                           );


class CUDT_control {
    volatile IOPType *networkIOP;
    CUDT udt;
public:
    CUDT_control(volatile IOPType *_networkIOP):networkIOP(_networkIOP), udt(_networkIOP) {}
    void dumb_egress_control(hls::stream<int> &egress_reader_start,
                             struct Block blocks[BUFFERCOUNT]);
    void dumb_ingress_control(hls::stream<int> &ingress_writer_ctrl,
                              hls::stream<int> &ingress_buffer_id,
                              struct Block blocks[BUFFERCOUNT]);
    bool isConnected() {
        return udt.isConnected();
    }
};

namespace udt {
    // The UDT spec defines the first 16 bits of a control packet to be a '1' followed
    // by a 15 bit type field.  Here we combine the two into a single sixteen bit field
    // which always starts with a '1'.
    struct udt_ctrl_type {
        static const unsigned short HANDSHAKE = 0x8000;
        static const unsigned short KEEPALIVE = 0x8001;
        static const unsigned short ACK = 0x8002;
        static const unsigned short NAK = 0x8003;
        static const unsigned short SHUTDOWN = 0x8005;
        static const unsigned short ACK2 = 0x8006;
        static const unsigned short MESSAGEDROP = 0x8007;
        static const unsigned short USER = 0xFFFF;
    };

    typedef newfield<ap_uint<32>, boost::mpl::string<'seqn'> > sequenceNumber;
    typedef newfield<ap_uint<32>, boost::mpl::string<'mesn'> > messageNumber;
    typedef newfield<ap_uint<32>, boost::mpl::string<'time'> > timeStamp;
    typedef newfield<ap_uint<32>, boost::mpl::string<'sock'> > socketID;

    typedef newfield<ap_uint<16>, boost::mpl::string<'type'> > ctrl_type;
    typedef newfield<ap_uint<16>, boost::mpl::string<'resv'> > reserved;
    typedef newfield<ap_uint<32>, boost::mpl::string<'info'> > ctrl_info;

    typedef newfield<ap_uint<32>, boost::mpl::string<'vers'> > version; // 4
    typedef newfield<ap_uint<32>, boost::mpl::string<'sock','type'> > socketType; // STREAM or DGRAM
    typedef newfield<ap_uint<32>, boost::mpl::string<'maxp','size'> > maximumPacketSize; // including UDP/IP headers
    typedef newfield<ap_uint<32>, boost::mpl::string<'maxw','size'> > maximumFlowWindowSize;
    typedef newfield<ap_uint<32>, boost::mpl::string<'conn','type'> > connectionType; // regular or rendezvous
    typedef newfield<ap_uint<32>, boost::mpl::string<'cook'> > cookie;
    typedef newfield<ap_uint<128>, boost::mpl::string<'addr'> > IPaddress;

    typedef newfield<ap_uint<32>, boost::mpl::string<'RTT'> > RTT; // in microseconds
    typedef newfield<ap_uint<32>, boost::mpl::string<'RTTv'> > RTTvar;
    typedef newfield<ap_uint<32>, boost::mpl::string<'size'> > bufferSize;
    typedef newfield<ap_uint<32>, boost::mpl::string<'rate'> > packetRate; // in packets per second
    typedef newfield<ap_uint<32>, boost::mpl::string<'lcap'> > linkCapacity; // in packets per second

    using data_header   = fixed_header<boost::mpl::vector<sequenceNumber, messageNumber, timeStamp, socketID> >;
    using ctrl_header   = fixed_header<boost::mpl::vector<ctrl_type, reserved, ctrl_info, timeStamp, socketID> >;
    using ctrl_handshake_header = fixed_header<boost::mpl::vector<version, socketType, sequenceNumber, maximumPacketSize, maximumFlowWindowSize, connectionType, socketID, cookie, IPaddress> >;
    using ctrl_ack_header   = fixed_header<boost::mpl::vector<sequenceNumber, RTT, RTTvar, bufferSize, packetRate, linkCapacity> >;

    template <typename T> data_header::parsed_hdr      <T> parse_udt_data_hdr         (T &h) { return data_header::parsed_hdr      <T>(h); }
    template <typename T> ctrl_header::parsed_hdr      <T> parse_udt_ctrl_hdr         (T &h) { return ctrl_header::parsed_hdr      <T>(h); }
    template <typename T> ctrl_handshake_header::parsed_hdr  <T> parse_udt_ctrl_handshake_hdr     (T &h) { return ctrl_handshake_header::parsed_hdr      <T>(h); }
    template <typename T> ctrl_ack_header::parsed_hdr  <T> parse_udt_ctrl_ack_hdr     (T &h) { return ctrl_ack_header::parsed_hdr      <T>(h); }
}
