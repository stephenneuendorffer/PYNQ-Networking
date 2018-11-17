/*****************************************************************************
Copyright (c) 2001 - 2011, The Board of Trustees of the University of Illinois.
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are
met:

* Redistributions of source code must retain the above
  copyright notice, this list of conditions and the
  following disclaimer.

* Redistributions in binary form must reproduce the
  above copyright notice, this list of conditions
  and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the University of Illinois
  nor the names of its contributors may be used to
  endorse or promote products derived from this
  software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*****************************************************************************/

/*****************************************************************************
written by
   Yunhong Gu, last updated 01/18/2011
*****************************************************************************/
#ifndef WRAP
// Don't try to parse these includes for dependencies

#ifndef __UDT_H__
#define __UDT_H__

const static int mss=1500;
const static int BUFFERSIZE=2048; // power of 2 greater than mss.
const static int BYTESPERCYCLE=8;
const static int BUFFERCOUNT=32;

/*
#ifndef WIN32
   #include <sys/types.h>
   #include <sys/socket.h>
   #include <netinet/in.h>
#else
   #ifdef __MINGW__
      #include <stdint.h>
      #include <ws2tcpip.h>
   #endif
   #include <windows.h>
#endif
*/

//#include <fstream>
#include <set>
#include <string>
#include <vector>

#include <stdint.h>

struct Block
{
    int m_iLength;                    // length of the block
    int32_t m_iMsgNo;                 // message number
    uint64_t m_OriginTime;            // original request time
    int32_t m_iSeqNo;
    int m_iTTL;                       // time to live (milliseconds)
    int used;
    uint32_t padding;
};

typedef unsigned int BlockID;
//typedef ap_uint<UnsignedBitWidth<BUFFERCOUNT-1>::Value> BlockID;



////////////////////////////////////////////////////////////////////////////////

//if compiling on VC6.0 or pre-WindowsXP systems
//use -DLEGACY_WIN32

//if compiling with MinGW, it only works on XP or above
//use -D_WIN32_WINNT=0x0501


#ifdef WIN32
   #ifndef __MINGW__
      // Explicitly define 32-bit and 64-bit numbers
      typedef __int32 int32_t;
      typedef __int64 int64_t;
      typedef unsigned __int32 uint32_t;
      #ifndef LEGACY_WIN32
         typedef unsigned __int64 uint64_t;
      #else
         // VC 6.0 does not support unsigned __int64: may cause potential problems.
         typedef __int64 uint64_t;
      #endif

      #ifdef UDT_EXPORTS
         #define UDT_API __declspec(dllexport)
      #else
         #define UDT_API __declspec(dllimport)
      #endif
   #else
      #define UDT_API
   #endif
#else
   #define UDT_API __attribute__ ((visibility("default")))
#endif

#define NO_BUSY_WAITING

#ifdef WIN32
   #ifndef __MINGW__
      typedef SOCKET SYSSOCKET;
   #else
      typedef int SYSSOCKET;
   #endif
#else
   typedef int SYSSOCKET;
#endif

typedef SYSSOCKET UDPSOCKET;
typedef int UDTSOCKET;

////////////////////////////////////////////////////////////////////////////////

typedef std::set<UDTSOCKET> ud_set;
#define UD_CLR(u, uset) ((uset)->erase(u))
#define UD_ISSET(u, uset) ((uset)->find(u) != (uset)->end())
#define UD_SET(u, uset) ((uset)->insert(u))
#define UD_ZERO(uset) ((uset)->clear())

enum EPOLLOpt
{
   // this values are defined same as linux epoll.h
   // so that if system values are used by mistake, they should have the same effect
   UDT_EPOLL_IN = 0x1,
   UDT_EPOLL_OUT = 0x4,
   UDT_EPOLL_ERR = 0x8
};

enum UDTSTATUS {INIT = 1, OPENED, LISTENING, CONNECTING, CONNECTED, BROKEN, CLOSING, CLOSED, NONEXIST};

////////////////////////////////////////////////////////////////////////////////

enum UDTOpt
{
   UDT_MSS,             // the Maximum Transfer Unit
   UDT_SNDSYN,          // if sending is blocking
   UDT_RCVSYN,          // if receiving is blocking
   UDT_CC,              // custom congestion control algorithm
   UDT_FC,		// Flight flag size (window size)
   UDT_SNDBUF,          // maximum buffer in sending queue
   UDT_RCVBUF,          // UDT receiving buffer size
   UDT_LINGER,          // waiting for unsent data when closing
   UDP_SNDBUF,          // UDP sending buffer size
   UDP_RCVBUF,          // UDP receiving buffer size
   UDT_MAXMSG,          // maximum datagram message size
   UDT_MSGTTL,          // time-to-live of a datagram message
   UDT_RENDEZVOUS,      // rendezvous connection mode
   UDT_SNDTIMEO,        // send() timeout
   UDT_RCVTIMEO,        // recv() timeout
   UDT_REUSEADDR,	// reuse an existing port or create a new one
   UDT_MAXBW,		// maximum bandwidth (bytes per second) that the connection can use
   UDT_STATE,		// current socket state, see UDTSTATUS, read only
   UDT_EVENT,		// current avalable events associated with the socket
   UDT_SNDDATA,		// size of data in the sending buffer
   UDT_RCVDATA		// size of data available for recv
};

////////////////////////////////////////////////////////////////////////////////

struct CPerfMon
{
   // global measurements
   int64_t msTimeStamp;                 // time since the UDT entity is started, in milliseconds
   int64_t pktSentTotal;                // total number of sent data packets, including retransmissions
   int64_t pktRecvTotal;                // total number of received packets
   int pktSndLossTotal;                 // total number of lost packets (sender side)
   int pktRcvLossTotal;                 // total number of lost packets (receiver side)
   int pktRetransTotal;                 // total number of retransmitted packets
   int pktSentACKTotal;                 // total number of sent ACK packets
   int pktRecvACKTotal;                 // total number of received ACK packets
   int pktSentNAKTotal;                 // total number of sent NAK packets
   int pktRecvNAKTotal;                 // total number of received NAK packets
   int64_t usSndDurationTotal;		// total time duration when UDT is sending data (idle time exclusive)

   // local measurements
   int64_t pktSent;                     // number of sent data packets, including retransmissions
   int64_t pktRecv;                     // number of received packets
   int pktSndLoss;                      // number of lost packets (sender side)
   int pktRcvLoss;                      // number of lost packets (receiver side)
   int pktRetrans;                      // number of retransmitted packets
   int pktSentACK;                      // number of sent ACK packets
   int pktRecvACK;                      // number of received ACK packets
   int pktSentNAK;                      // number of sent NAK packets
   int pktRecvNAK;                      // number of received NAK packets
   double mbpsSendRate;                 // sending rate in Mb/s
   double mbpsRecvRate;                 // receiving rate in Mb/s
   int64_t usSndDuration;		// busy sending time (i.e., idle time exclusive)

   // instant measurements
   double usPktSndPeriod;               // packet sending period, in microseconds
   int pktFlowWindow;                   // flow window size, in number of packets
   int pktCongestionWindow;             // congestion window size, in number of packets
   int pktFlightSize;                   // number of packets on flight
   double msRTT;                        // RTT, in milliseconds
   double mbpsBandwidth;                // estimated bandwidth, in Mb/s
   int byteAvailSndBuf;                 // available UDT sender buffer size
   int byteAvailRcvBuf;                 // available UDT receiver buffer size
};

////////////////////////////////////////////////////////////////////////////////

class UDT_API CUDTException
{
public:
   CUDTException(int major = 0, int minor = 0, int err = -1);
   CUDTException(const CUDTException& e);
   virtual ~CUDTException();

      // Functionality:
      //    Get the description of the exception.
      // Parameters:
      //    None.
      // Returned value:
      //    Text message for the exception description.

   virtual const char* getErrorMessage();

      // Functionality:
      //    Get the system errno for the exception.
      // Parameters:
      //    None.
      // Returned value:
      //    errno.

   virtual int getErrorCode() const;

      // Functionality:
      //    Clear the error code.
      // Parameters:
      //    None.
      // Returned value:
      //    None.

   virtual void clear();

private:
   int m_iMajor;        // major exception categories

// 0: correct condition
// 1: network setup exception
// 2: network connection broken
// 3: memory exception
// 4: file exception
// 5: method not supported
// 6+: undefined error

   int m_iMinor;		// for specific error reasons
   int m_iErrno;		// errno returned by the system if there is any
   std::string m_strMsg;	// text error message

   std::string m_strAPI;	// the name of UDT function that returns the error
   std::string m_strDebug;	// debug information, set to the original place that causes the error

public: // Error Code
   static const int SUCCESS;
   static const int ECONNSETUP;
   static const int ENOSERVER;
   static const int ECONNREJ;
   static const int ESOCKFAIL;
   static const int ESECFAIL;
   static const int ECONNFAIL;
   static const int ECONNLOST;
   static const int ENOCONN;
   static const int ERESOURCE;
   static const int ETHREAD;
   static const int ENOBUF;
   static const int EFILE;
   static const int EINVRDOFF;
   static const int ERDPERM;
   static const int EINVWROFF;
   static const int EWRPERM;
   static const int EINVOP;
   static const int EBOUNDSOCK;
   static const int ECONNSOCK;
   static const int EINVPARAM;
   static const int EINVSOCK;
   static const int EUNBOUNDSOCK;
   static const int ENOLISTEN;
   static const int ERDVNOSERV;
   static const int ERDVUNBOUND;
   static const int ESTREAMILL;
   static const int EDGRAMILL;
   static const int EDUPLISTEN;
   static const int ELARGEMSG;
   static const int EINVPOLLID;
   static const int EASYNCFAIL;
   static const int EASYNCSND;
   static const int EASYNCRCV;
   static const int ETIMEOUT;
   static const int EPEERERR;
   static const int EUNKNOWN;
};

////////////////////////////////////////////////////////////////////////////////

// If you need to export these APIs to be used by a different language,
// declare extern "C" for them, and add a "udt_" prefix to each API.
// The following APIs: sendfile(), recvfile(), epoll_wait(), geterrormsg(),
// include C++ specific feature, please use the corresponding sendfile2(), etc.

namespace UDT
{

typedef CUDTException ERRORINFO;
typedef UDTOpt SOCKOPT;
typedef CPerfMon TRACEINFO;
typedef ud_set UDSET;

UDT_API extern const UDTSOCKET INVALID_SOCK;
#undef ERROR
UDT_API extern const int ERROR;

UDT_API int startup();
UDT_API int cleanup();
UDT_API UDTSOCKET socket(int af, int type, int protocol);
UDT_API int bind(UDTSOCKET u, const struct sockaddr* name, int namelen);
UDT_API int bind2(UDTSOCKET u, UDPSOCKET udpsock);
UDT_API int listen(UDTSOCKET u, int backlog);
UDT_API UDTSOCKET accept(UDTSOCKET u, struct sockaddr* addr, int* addrlen);
UDT_API int connect(UDTSOCKET u, const struct sockaddr* name, int namelen);
UDT_API int close(UDTSOCKET u);
UDT_API int getpeername(UDTSOCKET u, struct sockaddr* name, int* namelen);
UDT_API int getsockname(UDTSOCKET u, struct sockaddr* name, int* namelen);
UDT_API int getsockopt(UDTSOCKET u, int level, SOCKOPT optname, void* optval, int* optlen);
UDT_API int setsockopt(UDTSOCKET u, int level, SOCKOPT optname, const void* optval, int optlen);
UDT_API int send(UDTSOCKET u, const char* buf, int len, int flags);
UDT_API int recv(UDTSOCKET u, char* buf, int len, int flags);
UDT_API int sendmsg(UDTSOCKET u, const char* buf, int len, int ttl = -1, bool inorder = false);
UDT_API int recvmsg(UDTSOCKET u, char* buf, int len);
UDT_API int64_t sendfile(UDTSOCKET u, std::fstream& ifs, int64_t& offset, int64_t size, int block = 364000);
UDT_API int64_t recvfile(UDTSOCKET u, std::fstream& ofs, int64_t& offset, int64_t size, int block = 7280000);
UDT_API int64_t sendfile2(UDTSOCKET u, const char* path, int64_t* offset, int64_t size, int block = 364000);
UDT_API int64_t recvfile2(UDTSOCKET u, const char* path, int64_t* offset, int64_t size, int block = 7280000);

// select and selectEX are DEPRECATED; please use epoll. 
UDT_API int select(int nfds, UDSET* readfds, UDSET* writefds, UDSET* exceptfds, const struct timeval* timeout);
UDT_API int selectEx(const std::vector<UDTSOCKET>& fds, std::vector<UDTSOCKET>* readfds,
                     std::vector<UDTSOCKET>* writefds, std::vector<UDTSOCKET>* exceptfds, int64_t msTimeOut);

UDT_API int epoll_create();
UDT_API int epoll_add_usock(int eid, UDTSOCKET u, const int* events = NULL);
UDT_API int epoll_add_ssock(int eid, SYSSOCKET s, const int* events = NULL);
UDT_API int epoll_remove_usock(int eid, UDTSOCKET u);
UDT_API int epoll_remove_ssock(int eid, SYSSOCKET s);
UDT_API int epoll_wait(int eid, std::set<UDTSOCKET>* readfds, std::set<UDTSOCKET>* writefds, int64_t msTimeOut,
                       std::set<SYSSOCKET>* lrfds = NULL, std::set<SYSSOCKET>* wrfds = NULL);
UDT_API int epoll_wait2(int eid, UDTSOCKET* readfds, int* rnum, UDTSOCKET* writefds, int* wnum, int64_t msTimeOut,
                        SYSSOCKET* lrfds = NULL, int* lrnum = NULL, SYSSOCKET* lwfds = NULL, int* lwnum = NULL);
UDT_API int epoll_release(int eid);
UDT_API ERRORINFO& getlasterror();
UDT_API int getlasterror_code();
UDT_API const char* getlasterror_desc();
UDT_API int perfmon(UDTSOCKET u, TRACEINFO* perf, bool clear = true);
UDT_API UDTSTATUS getsockstate(UDTSOCKET u);

}  // namespace UDT

#include "packet.h"
#include "window.h"
#include "core.h"



struct IP {
    volatile unsigned short version_dscp;
    volatile unsigned short len;
    volatile unsigned short id;
    volatile unsigned short fragment;
    volatile unsigned char ttl;
    volatile unsigned char proto;
    volatile unsigned short chksum;
    volatile int src;
    volatile int dst;
};
struct UDP {
    unsigned short sport;
    unsigned short dport;
    unsigned short len;
    unsigned short chksum;
};
#include "mb_interface.h"
static unsigned char get_ttl(volatile unsigned char *packet) {
    volatile struct IP * ip_hdr = (struct IP *)packet;
    unsigned char val;
    val = ip_hdr->ttl;
    return val;
}
static unsigned char get_proto(volatile unsigned char *packet) {
    volatile struct IP * ip_hdr = (struct IP *)packet;
    unsigned char val;
    val = ip_hdr->proto;
    return val;
}
static unsigned int get_src(volatile unsigned char *packet) {
    volatile struct IP * ip_hdr = (struct IP *)packet;
    unsigned int val;
    val = lwr(&ip_hdr->src);
    return val;
}
static unsigned int get_dst(volatile unsigned char *packet) {
    volatile struct IP * ip_hdr = (struct IP *)packet;
    unsigned int val;
    val = lwr(&ip_hdr->dst);
    return val;
}
static unsigned short get_ip_len(volatile unsigned char *packet) {
    volatile struct IP * ip_hdr = (struct IP *)packet;
    unsigned short val;
    val = lhur(&ip_hdr->version_dscp); // Not a bug.  the byte swapped variants have wierd addressing.
    return val;
}
static void set_ip_len(volatile unsigned char *packet, unsigned short val) {
    volatile struct IP * ip_hdr = (struct IP *)packet;
    shr(&ip_hdr->version_dscp, val); // Not a bug.  the byte swapped variants have wierd addressing.
}
static unsigned short get_ip_chksum(volatile unsigned char *packet) {
    volatile struct IP * ip_hdr = (struct IP *)packet;
    unsigned short val;
    val = lhur(&ip_hdr->ttl); // Not a bug.  the byte swapped variants have wierd addressing.
    return val;
}
static void set_ip_chksum(volatile unsigned char *packet, unsigned short val) {
    volatile struct IP * ip_hdr = (struct IP *)packet;
    shr(&ip_hdr->ttl, val); // Not a bug.  the byte swapped variants have wierd addressing.
}
static void set_dst(volatile unsigned char *packet, uint32_t val) {
    volatile struct IP * ip_hdr = (struct IP *)packet;
    swr(&ip_hdr->dst, val);
}


static unsigned short get_sport(volatile unsigned char *packet) {
    volatile struct UDP * udp_hdr = (struct UDP *)&packet[20];
    unsigned short val;
    val = lhur(&udp_hdr->dport); // Not a bug.  the byte swapped variants have wierd addressing.
    return val;
}
static unsigned short get_dport(volatile unsigned char *packet) {
    volatile struct UDP * udp_hdr = (struct UDP *)&packet[20];
    unsigned short val;
    val = lhur(&udp_hdr->sport); // Not a bug.  the byte swapped variants have wierd addressing.
    return val;
}
static unsigned short get_len(volatile unsigned char *packet) {
    volatile struct UDP * udp_hdr = (struct UDP *)&packet[20];
    unsigned short val;
    val = lhur(&udp_hdr->chksum); // Not a bug.  the byte swapped variants have wierd addressing.
    return val;
}
static unsigned short get_chksum(volatile unsigned char *packet) {
    volatile struct UDP * udp_hdr = (struct UDP *)&packet[20];
    unsigned short val;
    val = lhur(&udp_hdr->len); // Not a bug.  the byte swapped variants have wierd addressing.
    return val;
}
static void set_sport(volatile unsigned char *packet, unsigned short val) {
    volatile struct UDP * udp_hdr = (struct UDP *)&packet[20];
    shr(&udp_hdr->dport, val); // Not a bug.  the byte swapped variants have wierd addressing.
}
static void set_dport(volatile unsigned char *packet, unsigned short val) {
    volatile struct UDP * udp_hdr = (struct UDP *)&packet[20];
    shr(&udp_hdr->sport, val); // Not a bug.  the byte swapped variants have wierd addressing.
}
static void set_len(volatile unsigned char *packet, unsigned short val) {
    volatile struct UDP * udp_hdr = (struct UDP *)&packet[20];
    shr(&udp_hdr->chksum, val); // Not a bug.  the byte swapped variants have wierd addressing.
}
static void set_chksum(volatile unsigned char *packet, unsigned short val) {
    volatile struct UDP * udp_hdr = (struct UDP *)&packet[20];
    shr(&udp_hdr->len, val); // Not a bug.  the byte swapped variants have wierd addressing.
}


static uint32_t ntohl(uint32_t in) {
    return mb_swapb(in);
}
static uint32_t htonl(uint32_t in) {
    return mb_swapb(in);
}

static int ingress_params(int enable, unsigned int iRcvLastAck) {
    putfsl(enable,0); 
    putfsl(iRcvLastAck,0);
    putfsl(1,3); // Reset ingress_buffer_reader    
    putfsl(1,4); // Reset egress_buffer_writer
    return 0;
}

static int egress_params(unsigned int ip, unsigned int ipDst, unsigned int dport, unsigned int peerID) {
    putfsl(ip,1);
    putfsl(ipDst,1);
    putfsl(dport,1);
    putfsl(peerID,1);
    return 0;
}
static int egress_send(unsigned int buffer) {
    nputfsl(buffer, 2);
    int invalid;
    fsl_isinvalid(invalid);
    return !invalid;
}
static int ingress_get(unsigned int *buffer) {
    unsigned int b;
    ngetfsl(b, 0);
    *buffer = b;
    int invalid;
    fsl_isinvalid(invalid);
    return !invalid;
}

#endif
#endif
