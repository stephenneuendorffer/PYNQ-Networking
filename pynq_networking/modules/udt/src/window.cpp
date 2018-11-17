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
   Yunhong Gu, last updated 01/22/2011
*****************************************************************************/

#include <cmath>
#include "common.h"
#include "window.h"
#include <algorithm>

using namespace std;

template class CACKWindow<256>;
template class CPktTimeWindow<16, 64>;
template class CPktTimeWindow<16, 16>;

template<int size>
CACKWindow<size>::CACKWindow():
m_iHead(0),
m_iTail(0)
{
   m_piACKSeqNo[0] = -1;
}

template<int size>
void CACKWindow<size>::store(int32_t seq, int32_t ack)
{
   m_piACKSeqNo[m_iHead] = seq;
   m_piACK[m_iHead] = ack;
   m_pTimeStamp[m_iHead] = CTimer::getTime();

   if(m_iHead == m_iSize-1) m_iHead = 0; else m_iHead++;

   // overwrite the oldest ACK since it is not likely to be acknowledged
   if (m_iHead == m_iTail)
     if(m_iTail == m_iSize-1) m_iTail = 0; else m_iTail++;
}

template<int size>
int CACKWindow<size>::acknowledge(int32_t seq, int32_t& ack)
{
    IndexType i = m_iTail;
    for (int j = 0; j < m_iSize; j++) {
        if(i == m_iHead) break;
	//pyprintf("CACKWindow checking %d-%d %d %d %d\n", m_iTail, m_iHead, i, seq, m_piACKSeqNo[i]);
        // looking for indentical ACK Seq. No.
        if (seq == m_piACKSeqNo[i]) {
            // return the Data ACK it carried
            ack = m_piACK[i];

            // calculate RTT
            int rtt = int(CTimer::getTime() - m_pTimeStamp[i]);
	    //pyprintf("CACKWindow match %d %d\n", CTimer::getTime(),  m_pTimeStamp[i]);
	    m_iTail = i;
	    /*  if (i + 1 == m_iHead)
            {
               m_iTail = m_iHead = 0;
               m_piACKSeqNo[0] = -1;
            }
            else
	    */
	      if(m_iTail == m_iSize-1) m_iTail = 0; else m_iTail++;
	    //m_iTail = i + 1;

            return rtt;
        } else {
	  i++;
	}
    }

    // Bad input, the ACK node has been overwritten
    return -1;
}

////////////////////////////////////////////////////////////////////////////////
template<int asize, int psize>
CPktTimeWindow<asize,psize>::CPktTimeWindow():
m_iPktWindowPtr(0),
m_iProbeWindowPtr(0),
m_iLastSentTime(0),
m_iMinPktSndInt(1000000),
m_LastArrTime(),
m_CurrArrTime(),
m_ProbeTime()
{
   m_LastArrTime = CTimer::getTime();

   // for (int i = 0; i < m_iAWSize; ++ i)
   //    m_piPktWindow[i] = 1000000;

   // for (int k = 0; k < m_iPWSize; ++ k)
   //    m_piProbeWindow[k] = 1000;
}

template<int asize, int psize>
CPktTimeWindow<asize,psize>::~CPktTimeWindow()
{
}

template<int asize, int psize>
int CPktTimeWindow<asize,psize>::getMinPktSndInt() const
{
   return m_iMinPktSndInt;
}

template<int asize, int psize>
int CPktTimeWindow<asize,psize>::getPktRcvSpeed()
{
   // get median value, but cannot change the original value order in the window
   std::copy(m_piPktWindow, m_piPktWindow + m_iAWSize - 1, m_piPktReplica);
   std::nth_element(m_piPktReplica, m_piPktReplica + (m_iAWSize / 2), m_piPktReplica + m_iAWSize - 1);
   int median = m_piPktReplica[m_iAWSize / 2];

   int count = 0;
   int sum = 0;
   int upper = median << 3;
   int lower = median >> 3;

   // median filtering
   int * p = m_piPktWindow;
   for (int i = 0, n = m_iAWSize; i < n; ++ i)
   {
      if ((*p < upper) && (*p > lower))
      {
         ++ count;
         sum += *p;
      }
      ++ p;
   }

   // claculate speed, or return 0 if not enough valid value
   if (count > (m_iAWSize >> 1))
     return 0; //FIXME (int)ceil(1000000.0 / (sum / count));
   else
      return 0;
}

template<int asize, int psize>
int CPktTimeWindow<asize,psize>::getBandwidth()
{
   // get median value, but cannot change the original value order in the window
   std::copy(m_piProbeWindow, m_piProbeWindow + m_iPWSize - 1, m_piProbeReplica);
   std::nth_element(m_piProbeReplica, m_piProbeReplica + (m_iPWSize / 2), m_piProbeReplica + m_iPWSize - 1);
   int median = m_piProbeReplica[m_iPWSize / 2];

   int count = 1;
   int sum = median;
   int upper = median << 3;
   int lower = median >> 3;

   // median filtering
   int* p = &(m_piProbeWindow[0]);
   for (int i = 0, n = m_iPWSize; i < n; ++ i)
   {
      if ((*p < upper) && (*p > lower))
      {
         ++ count;
         sum += *p;
      }
      ++ p;
   }

   return 0; //FIXME (int)ceil(1000000.0 / (double(sum) / double(count)));
}

template<int asize, int psize>
void CPktTimeWindow<asize,psize>::onPktSent(int currtime)
{
   int interval = currtime - m_iLastSentTime;

   if ((interval < m_iMinPktSndInt) && (interval > 0))
      m_iMinPktSndInt = interval;

   m_iLastSentTime = currtime;
}

template<int asize, int psize>
void CPktTimeWindow<asize,psize>::onPktArrival()
{
   m_CurrArrTime = CTimer::getTime();

   // record the packet interval between the current and the last one
   *(m_piPktWindow + m_iPktWindowPtr) = int(m_CurrArrTime - m_LastArrTime);

   // the window is logically circular
   ++ m_iPktWindowPtr;
   if (m_iPktWindowPtr == m_iAWSize)
      m_iPktWindowPtr = 0;

   // remember last packet arrival time
   m_LastArrTime = m_CurrArrTime;
}

template<int asize, int psize>
void CPktTimeWindow<asize,psize>::probe1Arrival()
{
   m_ProbeTime = CTimer::getTime();
}

template<int asize, int psize>
void CPktTimeWindow<asize,psize>::probe2Arrival()
{
   m_CurrArrTime = CTimer::getTime();

   // record the probing packets interval
   *(m_piProbeWindow + m_iProbeWindowPtr) = int(m_CurrArrTime - m_ProbeTime);
   // the window is logically circular
   ++ m_iProbeWindowPtr;
   if (m_iProbeWindowPtr == m_iPWSize)
      m_iProbeWindowPtr = 0;
}


