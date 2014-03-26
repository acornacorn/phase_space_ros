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

#ifndef _PHASESPACE__DRIVER_H_
#define _PHASESPACE__DRIVER_H_

//#include "socket/udpServer.h"
//#include <boost/thread/mutex.hpp>
//#include <boost/thread/thread.hpp>
#include <ros/ros.h>
#include "tf/tf.h"

#include "owl.h"
#include "owl_math.h"

namespace phasespace {

static const int MAX_MARKER=32;
static const int MAX_UID=4;

struct marker{
  int id;
  float pos[3];
};

struct rigid_body {
  int id;
  std::vector<phasespace::marker> markers;
};

class phaseSpaceDriver {
public:
  phaseSpaceDriver(const std::string hostname,
                   const std::vector<std::string> rigid_body_names);
  ~phaseSpaceDriver();

  int init();

  //read the uid input.
  int read_phasespace(std::vector<tf::Pose>&);



private:
  //do not copy
  phaseSpaceDriver(const phaseSpaceDriver&);

  //read from the hardware
  void updateMarker(const int index, const float* pose);
  
  //read rigid body from the file
  int read_rigid_body_file(const char *rbfile, const int id);



  const int n_uid_;

  std::vector<tf::Pose> poses_;
  std::vector<std::string> filenames_;

  const std::string hostname_;
  bool is_new_;
  //markers are not used but still need to be read anyway
  std::vector<rigid_body> rb_markers_;
  int n_markers_;
  //the uids. 0=left

};

}

#endif
