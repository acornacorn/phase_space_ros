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

#include "phasespace/phaseSpaceDriver.h"

namespace phaseSpace{
#include "phasespace/msg.h"
}

#define SLAVE_CLIENT_MODE

phasespace::phaseSpaceDriver::phaseSpaceDriver(
                              const std::string hostname,
                              const int n_uid)
  : hostname_(hostname), is_new_(false), n_uid_(n_uid)
{
  // there's no default constructor of tf::Pose
  // with (0,0,0) position and (1,0,0,0) quaternion. it's annoying...
  for (int i=0; i<n_uid_; i++)
	  poses_.push_back(tf::Pose(tf::Quaternion(0,0,0,1)));
}

phasespace::phaseSpaceDriver::~phaseSpaceDriver()
{
  // cleanup
  owlDone();
}

int phasespace::phaseSpaceDriver::init()
{
  int flags = 0;

#ifdef SLAVE_CLIENT_MODE
  flags |= OWL_SLAVE;
#endif
  if(owlInit(hostname_.c_str(), flags) < 0)
    return -1;
  owlSetInteger(OWL_FRAME_BUFFER_SIZE, 0);

#ifndef SLAVE_CLIENT_MODE
  // create tracker 0
  int  tracker = 0;
  owlTrackeri(tracker, OWL_CREATE, OWL_POINT_TRACKER);

  // set markers
  for(int i = 0; i < n_uid_; i++)
    owlMarkeri(MARKER(tracker, i), OWL_SET_LED, i);

  // activate tracker
  owlTracker(tracker, OWL_ENABLE);
  // flush requests and check for errors
  if(!owlGetStatus())
  {
      owl_print_error("error in point tracker setup", owlGetError());
      return -1;
  }
#endif

  // set default frequency
  owlSetFloat(OWL_FREQUENCY, OWL_MAX_FREQUENCY);

  // start streaming
  owlSetInteger(OWL_STREAMING, OWL_ENABLE);

  return 0;
}

void phasespace::phaseSpaceDriver::updateMarker(const int index, const float* pose)
{
  if (index>n_uid_)
	  return;

  poses_[index].setOrigin(tf::Vector3(pose[0], pose[1], pose[2]));
  poses_[index].setRotation(tf::Quaternion(pose[3],pose[4],pose[5],pose[6]));
}

int phasespace::phaseSpaceDriver::read_phasespace(std::vector<tf::Pose>& poses)
{
  int rc = 0;
  int err;

  // get the rigid body
  int n = owlGetRigids(rigid_, n_uid_);

  // get the rigid body markers
  //  note: markers have to be read,
  //  even if they are not used
  int m = owlGetMarkers(markers_, MAX_MARKER);

  // check for error
  if((err = owlGetError()) != OWL_NO_ERROR)
  {
	owl_print_error("error", err);
	return -1;
  }

  // no data yet
  if(n == 0)
  {
	return 0;
  }

  is_new_=true;
  for(int j = 0; j < n; j++){
	if(rigid_[j].cond > 0) {
     // printf("r(%d) %3.0f %d id=%d:\n", j, rigid_[j].cond, rigid_[j].flag, rigid_[j].id);
    //printf("pose: %3.2f %3.2f %3.2f \n", rigid_[j].pose[0], rigid_[j].pose[1], rigid_[j].pose[2]);
  	  const int id = rigid_[j].id - 1;
	  if (id < 0 || id > n_uid_)
	  {
		printf("id error %d\n", id);
		return -1;
	  }

	  //build pose here
	  updateMarker(j, rigid_[j].pose);
	}
	else
	  is_new_=false;
  }
  poses=poses_;
  if (is_new_) return 1;
  return 0;
}


