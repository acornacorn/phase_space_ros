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
#include <stdio.h>
#include "phasespace/phaseSpaceDriver.h"
#include "ros/assert.h"

namespace phaseSpace{
#include "phasespace/msg.h"
}

phasespace::phaseSpaceDriver::phaseSpaceDriver(
                              const std::string hostname,
                              const std::vector<std::string> rigid_body_names)
  : hostname_(hostname), is_new_(false), n_uid_(rigid_body_names.size())
{
  // there's no default constructor of tf::Pose
  // with (0,0,0) position and (1,0,0,0) quaternion. it's annoying...
  for (int i=0; i<n_uid_; i++)
	  poses_.push_back(tf::Pose(tf::Quaternion(0,0,0,1)));
  rb_markers_.resize(n_uid_);

  for(int i=0; i<n_uid_; i++)
    read_rigid_body_file(rigid_body_names[i].c_str(), i);
}

phasespace::phaseSpaceDriver::~phaseSpaceDriver()
{
  // cleanup
  owlDone();
}

int phasespace::phaseSpaceDriver::read_rigid_body_file(const char *rbfile, const int id)
{
  FILE *fp;
  char whitespace;
  int m;

  fp = fopen(rbfile, "r");
  if (!fp) {
    perror(rbfile);
    return -1;
  }

  for(m = 0; m < phasespace::MAX_MARKER; m++) {
    phasespace::marker rb_marker;

    if (fscanf(fp, "%d%c %f %f %f",
	       &rb_marker.id, &whitespace,
	       &rb_marker.pos[0],
	       &rb_marker.pos[1],
	       &rb_marker.pos[2]) != 5) {
      break;
    }

    rb_markers_[id].markers.push_back(rb_marker);
  }
  fclose(fp);

  ROS_INFO("%s: %d markers loaded\n", rbfile, m);

  return 0;
}

int phasespace::phaseSpaceDriver::init()
{
  size_t flags = 0;

  if(owlInit(hostname_.c_str(), flags) < 0) {
    ROS_ERROR("CANNOT run owlInit");
	return -1;
  }
  owlSetInteger(OWL_FRAME_BUFFER_SIZE, 0);

  for (int tracker=0; tracker<rb_markers_.size(); tracker++) {
    // create tracker
    owlTrackeri(tracker, OWL_CREATE, OWL_RIGID_TRACKER);

    // set markers
    for(int i = 0; i < rb_markers_[tracker].markers.size(); i++) {
      owlMarkeri(MARKER(tracker, i), OWL_SET_LED, rb_markers_[tracker].markers[i].id);
      //ROS_INFO("id : %d ", rb_markers_[tracker].markers[i].id);
      // set marker positions
      owlMarkerfv(MARKER(tracker, i), OWL_SET_POSITION, rb_markers_[tracker].markers[i].pos);
      //ROS_INFO("pose : %f %f %f \n", rb_markers_[tracker].markers[i].pos[0], 
       //                    rb_markers_[tracker].markers[i].pos[1], 
       //                    rb_markers_[tracker].markers[i].pos[2]);
    }

    // activate tracker
    owlTracker(tracker, OWL_ENABLE);
    // flush requests and check for errors
    if(!owlGetStatus())
    {
      owl_print_error("error in point tracker setup", owlGetError());
      return -1;
    }
  }

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
  poses_[index].setRotation(tf::Quaternion(pose[4],pose[5],pose[6],pose[3]));
}

int phasespace::phaseSpaceDriver::read_phasespace(std::vector<tf::Pose>& poses)
{
  int return_bit=0;
  int err;

  OWLRigid rigid[n_uid_];
  OWLMarker markers[MAX_MARKER];

  // get the rigid body
  int n = owlGetRigids(rigid, n_uid_);

  // get the rigid body markers
  //  note: markers have to be read,
  //  even if they are not used
  int m = owlGetMarkers(markers, MAX_MARKER);

  // check for error
  if((err = owlGetError()) != OWL_NO_ERROR)
  {
	owl_print_error("error", err);
	return -1;
  }

  is_new_=false;
  // no data yet
  if(n == 0)
  {
    return 0;
  }
  
  for(int j = 0; j < n; j++){
    if(rigid[j].cond > 0) {
      return_bit|=(1<<j);

      //build pose here
      updateMarker(j, rigid[j].pose);
    }
  }
  poses=poses_;

  return return_bit;
}


