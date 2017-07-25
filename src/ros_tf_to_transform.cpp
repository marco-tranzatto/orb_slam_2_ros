/*
 * Copyright (c) 2016, Marco Tranzatto [marcot@ethz.ch],
 * Autonomous Systems Lab, ETH Zurich, Switzerland
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Autonomous Systems Lab, ETH Zurich nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 *PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 *LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
 *TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>

int main(int argc, char **argv) {

  ros::init(argc, argv, "ros_tf_to_transform");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  ros::Publisher transform_pub = nh.advertise<geometry_msgs::TransformStamped>(
      "T_world_viconcam0", 1);

  // User's settings.
  std::string frame_id;
  std::string child_frame_id;
  double publishing_frequency;

  nh_private.param<std::string>("frame_id",
                                frame_id, "world");
  nh_private.param<std::string>("child_frame_id",
                                child_frame_id, "vicon_cam0");
  nh_private.param("publishing_frequency",
                   publishing_frequency, 120.0);

  // Publish TransformStamped based on tf transform.
  tf::TransformListener listener;

  ros::Rate rate(publishing_frequency);

  while (nh.ok()) {

    tf::StampedTransform transform;
    bool error = false;
    try {
      listener.lookupTransform(frame_id, child_frame_id,
                               ros::Time(0), transform);
    }
    catch (tf::TransformException ex) {
      ROS_ERROR("%s", ex.what());
      error = true;
    }

    if (!error) {
      geometry_msgs::TransformStamped transform_msg;

      tf::transformStampedTFToMsg(transform, transform_msg);
      transform_pub.publish(transform_msg);
    }

    rate.sleep();
  }
  return 0;
};