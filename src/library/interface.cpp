#include "orb_slam_2_ros/interface.hpp"

#include <glog/logging.h>
#include <minkindr_conversions/kindr_msg.h>
#include <minkindr_conversions/kindr_tf.h>
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>

namespace orb_slam_2_interface {

OrbSlam2Interface::OrbSlam2Interface(const ros::NodeHandle& nh,
                                     const ros::NodeHandle& nh_private)
    : nh_(nh),
      nh_private_(nh_private),
      slam_system_(nullptr),
      verbose_(kDefaultVerbose),
      frame_id_(kDefaultFrameId),
      child_frame_id_(kDefaultChildFrameId),
      visualization_(kDefaultVisualization),
      save_map_file_path_(kDefaultSaveMapFilePath),
      load_map_file_path_(kDefaultLoadMapFilePath),
      load_existing_map_(kDefaultLoadExistingMap),
      localization_mode_(kDefaultLocalizationMode) {
  // Getting data and params
  advertiseTopics();
  advertiseServices();
  getParametersFromRos();
}

void OrbSlam2Interface::advertiseTopics() {
  // Advertising topics
  T_pub_ = nh_private_.advertise<geometry_msgs::TransformStamped>(
      "transform_cam", 1);
  // Creating a callback timer for TF publisher
  tf_timer_ = nh_.createTimer(ros::Duration(0.01),
                              &OrbSlam2Interface::publishCurrentPoseAsTF, this);
}

void OrbSlam2Interface::advertiseServices() {
  save_orb_map_srv_ = nh_private_.advertiseService(
      "save_orb_map", &OrbSlam2Interface::saveMap, this);
}

void OrbSlam2Interface::getParametersFromRos() {
  // Getting the paths to the files required by orb slam
  CHECK(nh_private_.getParam("vocabulary_file_path", vocabulary_file_path_))
      << "Please provide the vocabulary_file_path as a ros param.";
  CHECK(nh_private_.getParam("settings_file_path", settings_file_path_))
      << "Please provide the settings_file_path as a ros param.";
  // Optional params
  nh_private_.getParam("verbose", verbose_);
  nh_private_.getParam("frame_id", frame_id_);
  nh_private_.getParam("child_frame_id", child_frame_id_);
  nh_private_.getParam("visualization", visualization_);
  nh_private_.getParam("save_map_file_path", save_map_file_path_);
  nh_private_.getParam("load_map_file_path", load_map_file_path_);
  nh_private_.getParam("load_existing_map", load_existing_map_);
  nh_private_.getParam("localization_mode", localization_mode_);

  // Params check
  if (localization_mode_ && !load_existing_map_) {
    ROS_WARN("Localization mode only is enabled (no mapping) but no existing map is being loaded.");
  }
}

void OrbSlam2Interface::publishCurrentPose(const Transformation& T,
                                           const std_msgs::Header& header) {
  // Creating the message
  geometry_msgs::TransformStamped msg;
  // Filling out the header
  msg.header = header;
  // Setting the child and parent frames
  msg.child_frame_id = child_frame_id_;
  // Converting from a minkindr transform to a transform message
  tf::transformKindrToMsg(T, &msg.transform);
  // Publishing the current transformation.
  T_pub_.publish(msg);
}

void OrbSlam2Interface::publishCurrentPoseAsTF(const ros::TimerEvent& event) {
  tf::Transform tf_transform;
  tf::transformKindrToTF(T_W_C_, &tf_transform);
  tf_broadcaster_.sendTransform(tf::StampedTransform(
      tf_transform, ros::Time::now(), frame_id_, child_frame_id_));
}

void OrbSlam2Interface::convertOrbSlamPoseToKindr(const cv::Mat& T_cv,
                                                  Transformation* T_kindr) {
  // Argument checks
  CHECK_NOTNULL(T_kindr);
  CHECK_EQ(T_cv.cols, 4);
  CHECK_EQ(T_cv.rows, 4);
  // Open CV mat to Eigen matrix (float)
  Eigen::Matrix4f T_eigen_f;
  cv::cv2eigen(T_cv, T_eigen_f);
  // Eigen matrix (float) to Eigen matrix (double)
  Eigen::Matrix4d T_eigen_d = T_eigen_f.cast<double>();
  // Extracting and orthonormalizing the rotation matrix
  Eigen::Matrix3d R_unnormalized = T_eigen_d.block<3, 3>(0, 0);
  Eigen::AngleAxisd aa(R_unnormalized);
  Eigen::Matrix3d R = aa.toRotationMatrix();
  // Constructing the transformation
  Quaternion q_kindr(R);
  Eigen::Vector3d t_kindr(T_eigen_d.block<3, 1>(0, 3));
  *T_kindr = Transformation(q_kindr, t_kindr);
}

bool OrbSlam2Interface::saveMap(std_srvs::Trigger::Request& request,
                                std_srvs::Trigger::Response& response) {
  bool success;
  std::string message;

  // TODO delete me, I'm used to testing save/load map
  ROS_WARN("Sendig command 'Shutdown' to SLAM!");
  slam_system_->Shutdown();
  // END TODO delete me, I'm used to testing save/load map

  slam_system_->SaveMap(save_map_file_path_, &success, &message);
  response.success = success;
  response.message = message;

  return true;
}

}  // namespace orb_slam_2_interface
