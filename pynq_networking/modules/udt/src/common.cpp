/*****************************************************************************
Copyright (c) 2001 - 2010, The Board of Trustees of the University of Illinois.
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
   Yunhong Gu, last updated 07/25/2010
*****************************************************************************/


#include <cmath>
//#include "md5.h"
#include "common.h"


//bool CTimer::m_bUseMicroSecond = false;
uint64_t CTimer::s_ullCPUFrequency = CTimer::readCPUFrequency();
/*
CTimer::CTimer():
m_ullSchedTime(),
m_TickCond(),
m_TickLock()
{ 
   #ifndef WIN32
      pthread_mutex_init(&m_TickLock, NULL);
      pthread_cond_init(&m_TickCond, NULL);
   #else
      m_TickLock = CreateMutex(NULL, false, NULL);
      m_TickCond = CreateEvent(NULL, false, false, NULL);
   #endif
}

CTimer::~CTimer()
{
   #ifndef WIN32
      pthread_mutex_destroy(&m_TickLock);
      pthread_cond_destroy(&m_TickCond);
   #else
      CloseHandle(m_TickLock);
      CloseHandle(m_TickCond);
   #endif
}
*/
void CTimer::rdtsc(uint64_t &x)
{  
  volatile uint32_t *timescale = (uint32_t *)0x41a00000;
  static uint32_t lasttime = 0;
  static uint32_t t = 0;
  uint32_t time = timescale[2];
  if( time < lasttime) {
    t++;
  }
  uint64_t t2 = t;
  t2 <<= 32;
  t2 |= time;
  lasttime = time;
  x = t2;
  return;
/*
   if (m_bUseMicroSecond)
   {
      x = getTime();
      return;
   }

   #ifdef IA32
      uint32_t lval, hval;
      //asm volatile ("push %eax; push %ebx; push %ecx; push %edx");
      //asm volatile ("xor %eax, %eax; cpuid");
      asm volatile ("rdtsc" : "=a" (lval), "=d" (hval));
      //asm volatile ("pop %edx; pop %ecx; pop %ebx; pop %eax");
      x = hval;
      x = (x << 32) | lval;
   #elif defined(IA64)
      asm ("mov %0=ar.itc" : "=r"(x) :: "memory");
   #elif defined(AMD64)
      uint32_t lval, hval;
      asm ("rdtsc" : "=a" (lval), "=d" (hval));
      x = hval;
      x = (x << 32) | lval;
   #elif defined(WIN32)
      //HANDLE hCurThread = ::GetCurrentThread(); 
      //DWORD_PTR dwOldMask = ::SetThreadAffinityMask(hCurThread, 1); 
      BOOL ret = QueryPerformanceCounter((LARGE_INTEGER *)&x);
      //SetThreadAffinityMask(hCurThread, dwOldMask);
      if (!ret)
         x = getTime() * s_ullCPUFrequency;
   #elif defined(OSX)
      x = mach_absolute_time();
   #else
      // use system call to read time clock for other archs
      x = getTime();
   #endif
*/
}

uint64_t CTimer::readCPUFrequency()
{
  // Approximate this to avoid a slow division
  uint64_t frequency = 128; // 156;  // 1 tick per microsecond.
   /*
   #if defined(IA32) || defined(IA64) || defined(AMD64)
      uint64_t t1, t2;

      rdtsc(t1);
      timespec ts;
      ts.tv_sec = 0;
      ts.tv_nsec = 100000000;
      nanosleep(&ts, NULL);
      rdtsc(t2);

      // CPU clocks per microsecond
      frequency = (t2 - t1) / 100000;
   #elif defined(WIN32)
      int64_t ccf;
      if (QueryPerformanceFrequency((LARGE_INTEGER *)&ccf))
         frequency = ccf / 1000000;
   #elif defined(OSX)
      mach_timebase_info_data_t info;
      mach_timebase_info(&info);
      frequency = info.denom * 1000ULL / info.numer;
   #endif

   // Fall back to microsecond if the resolution is not high enough.
   if (frequency < 10)
   {
      frequency = 1;
      m_bUseMicroSecond = true;
      }*/
   return frequency;
}

uint64_t CTimer::getCPUFrequency()
{
    return s_ullCPUFrequency;
}

// void CTimer::sleep(uint64_t interval)
// {
//    uint64_t t;
//    rdtsc(t);

//    // sleep next "interval" time
//    sleepto(t + interval);
// }

// void CTimer::sleepto(uint64_t nexttime)
// {
//    // Use class member such that the method can be interrupted by others
//    m_ullSchedTime = nexttime;

//    uint64_t t;
//    rdtsc(t);

//    while (t < m_ullSchedTime)
//    {
//       #ifndef NO_BUSY_WAITING
//          #ifdef IA32
//             __asm__ volatile ("pause; rep; nop; nop; nop; nop; nop;");
//          #elif IA64
//             __asm__ volatile ("nop 0; nop 0; nop 0; nop 0; nop 0;");
//          #elif AMD64
//             __asm__ volatile ("nop; nop; nop; nop; nop;");
//          #endif
//       #else
//          #ifndef WIN32
//             timeval now;
//             timespec timeout;
//             gettimeofday(&now, 0);
//             if (now.tv_usec < 990000)
//             {
//                timeout.tv_sec = now.tv_sec;
//                timeout.tv_nsec = (now.tv_usec + 10000) * 1000;
//             }
//             else
//             {
//                timeout.tv_sec = now.tv_sec + 1;
//                timeout.tv_nsec = (now.tv_usec + 10000 - 1000000) * 1000;
//             }
//             pthread_mutex_lock(&m_TickLock);
//             pthread_cond_timedwait(&m_TickCond, &m_TickLock, &timeout);
//             pthread_mutex_unlock(&m_TickLock);
//          #else
//             WaitForSingleObject(m_TickCond, 1);
//          #endif
//       #endif

//       rdtsc(t);
//    }
// }

// void CTimer::interrupt()
// {
//    // schedule the sleepto time to the current CCs, so that it will stop
//    rdtsc(m_ullSchedTime);
//    tick();
// }

// void CTimer::tick()
// {
//    #ifndef WIN32
//       pthread_cond_signal(&m_TickCond);
//    #else
//       SetEvent(m_TickCond);
//    #endif
// }

uint64_t CTimer::getTime()
{
  // return 0;
  //For Cygwin and other systems without microsecond level resolution, uncomment the following three lines
   uint64_t x;
   rdtsc(x);
   return x >> 7; // / s_ullCPUFrequency;
   //Specific fix may be necessary if rdtsc is not available either.

   // #ifndef WIN32
   //    timeval t;
   //    gettimeofday(&t, 0);
   //    return t.tv_sec * 1000000ULL + t.tv_usec;
   // #else
   //    LARGE_INTEGER ccf;
   //    HANDLE hCurThread = ::GetCurrentThread(); 
   //    DWORD_PTR dwOldMask = ::SetThreadAffinityMask(hCurThread, 1);
   //    if (QueryPerformanceFrequency(&ccf))
   //    {
   //       LARGE_INTEGER cc;
   //       if (QueryPerformanceCounter(&cc))
   //       {
   //          SetThreadAffinityMask(hCurThread, dwOldMask); 
   //          return (cc.QuadPart * 1000000ULL / ccf.QuadPart);
   //       }
   //    }

   //    SetThreadAffinityMask(hCurThread, dwOldMask); 
   //    return GetTickCount() * 1000ULL;
   // #endif
}
/*
void CTimer::triggerEvent()
{
   #ifndef WIN32
      pthread_cond_signal(&m_EventCond);
   #else
      SetEvent(m_EventCond);
   #endif
}

void CTimer::waitForEvent()
{
   #ifndef WIN32
      timeval now;
      timespec timeout;
      gettimeofday(&now, 0);
      if (now.tv_usec < 990000)
      {
         timeout.tv_sec = now.tv_sec;
         timeout.tv_nsec = (now.tv_usec + 10000) * 1000;
      }
      else
      {
         timeout.tv_sec = now.tv_sec + 1;
         timeout.tv_nsec = (now.tv_usec + 10000 - 1000000) * 1000;
      }
      pthread_mutex_lock(&m_EventLock);
      pthread_cond_timedwait(&m_EventCond, &m_EventLock, &timeout);
      pthread_mutex_unlock(&m_EventLock);
   #else
      WaitForSingleObject(m_EventCond, 1);
   #endif
}

void CTimer::sleep()
{
   #ifndef WIN32
      usleep(10);
   #else
      Sleep(1);
   #endif
}
*/

/*

//
bool CIPAddress::ipcmp(const sockaddr* addr1, const sockaddr* addr2, int ver)
{
   if (AF_INET == ver)
   {
      sockaddr_in* a1 = (sockaddr_in*)addr1;
      sockaddr_in* a2 = (sockaddr_in*)addr2;

      if ((a1->sin_port == a2->sin_port) && (a1->sin_addr.s_addr == a2->sin_addr.s_addr))
         return true;
   }
   else
   {
      sockaddr_in6* a1 = (sockaddr_in6*)addr1;
      sockaddr_in6* a2 = (sockaddr_in6*)addr2;

      if (a1->sin6_port == a2->sin6_port)
      {
         for (int i = 0; i < 16; ++ i)
            if (*((char*)&(a1->sin6_addr) + i) != *((char*)&(a2->sin6_addr) + i))
               return false;

         return true;
      }
   }

   return false;
}

void CIPAddress::ntop(const sockaddr* addr, uint32_t ip[4], int ver)
{
   if (AF_INET == ver)
   {
      sockaddr_in* a = (sockaddr_in*)addr;
      ip[0] = a->sin_addr.s_addr;
   }
   else
   {
      sockaddr_in6* a = (sockaddr_in6*)addr;
      ip[3] = (a->sin6_addr.s6_addr[15] << 24) + (a->sin6_addr.s6_addr[14] << 16) + (a->sin6_addr.s6_addr[13] << 8) + a->sin6_addr.s6_addr[12];
      ip[2] = (a->sin6_addr.s6_addr[11] << 24) + (a->sin6_addr.s6_addr[10] << 16) + (a->sin6_addr.s6_addr[9] << 8) + a->sin6_addr.s6_addr[8];
      ip[1] = (a->sin6_addr.s6_addr[7] << 24) + (a->sin6_addr.s6_addr[6] << 16) + (a->sin6_addr.s6_addr[5] << 8) + a->sin6_addr.s6_addr[4];
      ip[0] = (a->sin6_addr.s6_addr[3] << 24) + (a->sin6_addr.s6_addr[2] << 16) + (a->sin6_addr.s6_addr[1] << 8) + a->sin6_addr.s6_addr[0];
   }
}

void CIPAddress::pton(sockaddr* addr, const uint32_t ip[4], int ver)
{
   if (AF_INET == ver)
   {
      sockaddr_in* a = (sockaddr_in*)addr;
      a->sin_addr.s_addr = ip[0];
   }
   else
   {
      sockaddr_in6* a = (sockaddr_in6*)addr;
      for (int i = 0; i < 4; ++ i)
      {
         a->sin6_addr.s6_addr[i * 4] = ip[i] & 0xFF;
         a->sin6_addr.s6_addr[i * 4 + 1] = (unsigned char)((ip[i] & 0xFF00) >> 8);
         a->sin6_addr.s6_addr[i * 4 + 2] = (unsigned char)((ip[i] & 0xFF0000) >> 16);
         a->sin6_addr.s6_addr[i * 4 + 3] = (unsigned char)((ip[i] & 0xFF000000) >> 24);
      }
   }
}

//
void CMD5::compute(const char* input, unsigned char result[16])
{
   md5_state_t state;

   md5_init(&state);
   md5_append(&state, (const md5_byte_t *)input, strlen(input));
   md5_finish(&state, result);
}
*/
