/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2013, SRI International
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/** \Author: Sean Seungkook Yun */

#include "phaseSpace/phaseSpaceDriver.h"

namespace phaseSpace{
#include "phaseSpace/msg.h"
}

phaseSpace::phaseSpaceDriver::phaseSpaceDriver(
                              const std::string hostname,
                              const int port,
                              const int n_uid)
  : is_new_(false), n_uid_(n_uid)
{
  udp_server_=new udpServer(hostname, port);
  
  run();
}

phaseSpace::phaseSpaceDriver::~phaseSpaceDriver()
{
  if (udp_server_)
    delete udp_server_;
}

void phaseSpace::phaseSpaceDriver::run()
{
  boost::thread udp_thread;

  udp_thread=boost::thread(&phaseSpace::phaseSpaceDriver::threadRun, this);
  udp_thread.join();
}

int phaseSpace::phaseSpaceDriver::read_packet()
{
  mutex_.lock();

  phaseSpace::AtlasSimMsg pck[4];
  int nbyte=udp_server_->recv_udp(pck, sizeof(pck));

  //TODO: update poses_ here

  mutex_.unlock();

  return nbyte;
}
void phaseSpace::phaseSpaceDriver::threadRun()
{
  while(true) {
	read_packet();

	boost::this_thread::sleep(boost::posix_time::milliseconds(10));
  }
}

void phaseSpace::phaseSpaceDriver::read_phasespace(std::vector<tf::Pose>& poses)
{
  mutex_.lock();
  poses=poses_;  //copy the whole structure due to synchronous comm
  mutex_.unlock();
}


