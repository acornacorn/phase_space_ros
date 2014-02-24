/*********************************************************************
*
* This is free and unencumbered software released into the public domain.
* 
* Anyone is free to copy, modify, publish, use, compile, sell, or
* distribute this software, either in source code form or as a compiled
* binary, for any purpose, commercial or non-commercial, and by any
* means.
* 
* In jurisdictions that recognize copyright laws, the author or authors
* of this software dedicate any and all copyright interest in the
* software to the public domain. We make this dedication for the benefit
* of the public at large and to the detriment of our heirs and
* successors. We intend this dedication to be an overt act of
* relinquishment in perpetuity of all present and future rights to this
* software under copyright law.
* 
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
* IN NO EVENT SHALL THE AUTHORS BE LIABLE FOR ANY CLAIM, DAMAGES OR
* OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
* ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
* OTHER DEALINGS IN THE SOFTWARE.
* 
* For more information, please refer to <http://unlicense.org/>
* 
**********************************************************************/

/** 
 * Author: Sean Seungkook Yun <seungkook.yun@sri.com> 
*/

#include <string>
#include <ros/ros.h>
#include "tf/tf.h"
#include "phasespace/PhaseSpaceMsg.h"
#include "phasespace/phaseSpaceDriver.h"
// Visualization
#include <tf/transform_broadcaster.h>

using namespace phasespace;
using std::string;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "phasespace_driver");
  ros::NodeHandle n, n_private("~");
  ros::Time::init();

  //initialize hardware
  ROS_INFO("Initializing PhaseSpace. Please wait....");
  std::string phasespace_hostname;
  n_private.param<std::string>("phasespace_pc", phasespace_hostname, "phasespace-pc");
  int port;
  n_private.param<int>("phasespace_port", port, 5064);
  int num_sen;
  n_private.param<int>("n_sensor", num_sen, 2);
  phaseSpaceDriver phasespace(phasespace_hostname, port, num_sen);
  ROS_INFO("Initialization Complete.");

  if (num_sen<2) {
	ROS_ERROR("at least 2 trackers required");
	return -1;
  }

  ROS_INFO("Output is set: position/quaternion");

  bool publish_tf;
  n_private.param<bool>("publish_tf", publish_tf, false);
  if(publish_tf)
	ROS_INFO("Publishing frame data to TF.");

  //orientation of how the sensor tip is attached
  double rx, ry, rz;
  n_private.param<double>("attach_roll", rx, 0);
  n_private.param<double>("attach_pitch", ry, 0);
  n_private.param<double>("attach_yaw", rz, 0);
  tf::Matrix3x3 phasespace_attach_ori;
  phasespace_attach_ori.setEulerYPR(rz, ry, rx);
  tf::Pose phasespace_attach(phasespace_attach_ori, tf::Vector3(0,0,0));

  // Initialize ROS stuff
  ros::Publisher phasespace_pub = n.advertise<phasespace::PhaseSpaceMsg>("phasespace_msg", 1);
  ros::Publisher phasespace_raw_pub = n.advertise<phasespace::PhaseSpaceMsg>("phasespace_raw_msg", 1);
  tf::TransformBroadcaster *broadcaster = 0;
  if(publish_tf)
	broadcaster = new tf::TransformBroadcaster();

  // mangle the reported pose into the ROS frame conventions
  const tf::Matrix3x3 ros_to_phasespace( -1,  0,  0,
							  0,  1,  0,
							  0,  0, -1 );



  while (n.ok())
  {
	std::vector<tf::Pose> poses;

	//publish data
    phasespace::PhaseSpaceMsg msg;
    msg.header.stamp = ros::Time::now();

	//publish raw data
    phasespace::PhaseSpaceMsg msg_raw;
	msg_raw.header.stamp = ros::Time::now();

    phasespace.read_phasespace(poses);

    std::vector<geometry_msgs::TransformStamped> transforms(poses.size());

    for( int i = 0; i <transforms.size()  ; ++i )
	{
      tf::transformTFToMsg(poses[i], transforms[i].transform);
      msg_raw.transform[i]=transforms[i].transform;

      tf::Pose pose_calibrated;
      pose_calibrated.setBasis(ros_to_phasespace*poses[i].getBasis());
      pose_calibrated.setOrigin(ros_to_phasespace*pose_calibrated.getOrigin());
      pose_calibrated*=phasespace_attach;
      tf::transformTFToMsg(pose_calibrated, transforms[i].transform);

		msg.transform[i]=transforms[i].transform;
	}
    phasespace_pub.publish(msg);
    phasespace_raw_pub.publish(msg_raw);


	if(broadcaster)
	{
	  std::string frames[4] = {"phasespace_left", "phasespace_right", "phasespace_left2", "phasespace_right2"};
      for(int kk = 0; kk < num_sen; kk++)
      {
        transforms[kk].header.stamp = msg.header.stamp;
        transforms[kk].header.frame_id = "phasespace_base";
        transforms[kk].child_frame_id = frames[kk];
      }

      broadcaster->sendTransform(transforms);
    }

	ros::spinOnce();
  }

}









