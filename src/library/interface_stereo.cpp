#include <cv_bridge/cv_bridge.h>

#include "orb_slam_2_ros/interface_stereo.hpp"

namespace orb_slam_2_interface {

OrbSlam2InterfaceStereo::OrbSlam2InterfaceStereo(
    const ros::NodeHandle& nh, const ros::NodeHandle& nh_private)
    : OrbSlam2Interface(nh, nh_private),
      rectify_input_images_(kDefaultRectifyInputImages),
      valid_open_cv_rectify_param_(false) {
  // Getting data and params
  subscribeToTopics();
  getStereoParametersFromRos();
  slam_system_ = std::shared_ptr<ORB_SLAM2::System>(
      new ORB_SLAM2::System(vocabulary_file_path_, settings_file_path_,
                            ORB_SLAM2::System::STEREO, visualization_,
                            load_existing_map_, load_map_file_path_));
}

void OrbSlam2InterfaceStereo::subscribeToTopics() {
  // Subscribing to the stereo images
  left_sub_ = std::shared_ptr<message_filters::Subscriber<sensor_msgs::Image>>(
      new message_filters::Subscriber<sensor_msgs::Image>(
          nh_, "camera/left/image_raw", 1));
  right_sub_ = std::shared_ptr<message_filters::Subscriber<sensor_msgs::Image>>(
      new message_filters::Subscriber<sensor_msgs::Image>(
          nh_, "camera/right/image_raw", 1));
  // Creating a synchronizer
  sync_ = std::shared_ptr<message_filters::Synchronizer<sync_pol>>(
      new message_filters::Synchronizer<sync_pol>(sync_pol(10), *left_sub_,
                                                  *right_sub_));
  // Registering the synchronized image callback
  sync_->registerCallback(
      boost::bind(&OrbSlam2InterfaceStereo::stereoImageCallback, this, _1, _2));
}

void OrbSlam2InterfaceStereo::getStereoParametersFromRos() {
  // Optional params
  nh_private_.getParam("rectify_input_images", rectify_input_images_);

  if (rectify_input_images_) {
    getParametersStereoOpenCV();
  }
}

void OrbSlam2InterfaceStereo::getParametersStereoOpenCV() {
  // Adapted from russellaabuchanan 's code.
  cv::FileStorage fsSettings(settings_file_path_, cv::FileStorage::READ);

  cv::Mat K_l, K_r, P_l, P_r, R_l, R_r, D_l, D_r, T_right_left;
  double fx, fy, cx, cy, bf;
  int rows_l, cols_l, rows_r, cols_r;

  // Left camera
  fsSettings["LEFT.K"] >> K_l;
  fsSettings["LEFT.D"] >> D_l;
  rows_l = fsSettings["LEFT.height"];
  cols_l = fsSettings["LEFT.width"];
  fsSettings["LEFT.P"] >> P_l;
  fsSettings["LEFT.R"] >> R_l;
  fx = fsSettings["Camera.fx"];
  fy = fsSettings["Camera.fy"];
  cx = fsSettings["Camera.cx"];
  cy = fsSettings["Camera.cy"];

  // Right camera
  fsSettings["RIGHT.K"] >> K_r;
  fsSettings["RIGHT.D"] >> D_r;
  rows_r = fsSettings["RIGHT.height"];
  cols_r = fsSettings["RIGHT.width"];
  fsSettings["RIGHT.P"] >> P_r;
  fsSettings["RIGHT.R"] >> R_r;

  // Relative transformation.
  fsSettings["T_RIGHT_LEFT"] >> T_right_left;
  bf = fsSettings["Camera.bf"];

  if (K_l.empty() || K_r.empty() || D_l.empty() || D_r.empty() || rows_l == 0 ||
      rows_r == 0 || cols_l == 0 || cols_r == 0) {
    ROS_ERROR("Distortion parameters for stereo rectification are missing!");
    return;
  }

  if (R_l.empty() || R_r.empty()) {
    if (!T_right_left.empty()) {
      ROS_WARN(
          "Rectification matrices 'LEFT.R, RIGHT.R' are missing. Calculating "
          "them now.");

      cv::Mat tmp1, tmp2, tmp3;
      cv::stereoRectify(K_l, D_l, K_r, D_r, cv::Size(cols_l, rows_l),
                        T_right_left.rowRange(0, 3).colRange(0, 3),
                        T_right_left.col(3).rowRange(0, 3), R_l, R_r, tmp1,
                        tmp2, tmp3);
    } else {
      ROS_ERROR(
          "Rectification matrices cannot be calculated without providing "
          "'T_RIGHT_LEFT' parameter.");
      return;
    }
  }

  // Comparing output stereoRectify function  and R and P matrices from Euroc
  // dataset,
  // it looks like the P matrices must be filled in by using
  // 'camera calibration and distortion parameters' on top of the yaml file.
  // (marco-tranzatto)
  if (P_l.empty() || P_r.empty()) {
    ROS_WARN(
        "Projection matrices 'LEFT.P, RIGHT.P' are missing. Filling them now "
        "using Camera settings (fx,fy,cx,cy,bf).");

    P_l = (cv::Mat_<double>(3, 4) << fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0,
           0.0, 1.0, 0.0);
    P_r = (cv::Mat_<double>(3, 4) << fx, 0.0, cx, -bf, 0.0, fy, cy, 0.0, 0.0,
           0.0, 1.0, 0.0);
  }

  // TODO The following code is suposed to compute in one single step the
  // matrices
  // R_l, R_r, P_l, P_r. When checking the results with Euroc dataset,
  // unfortunately the output P_l and P_r
  // are quite different from the ones in Euroc.yaml file.
  // For now I just leave this here, but commented. (marco-tranzatto)
  /*
  if (P_l.empty() || P_r.empty() || R_l.empty() || R_r.empty()) {
    if (!T_right_left.empty()) {
      ROS_WARN("Rectification matrices 'LEFT.P, RIGHT.P, LEFT.R, RIGHT.R' are
  missing. Calculating them now.");
      cv::stereoRectify(K_l, D_l, K_r, D_r, cv::Size(cols_l, rows_l),
                        T_right_left.rowRange(0, 3).colRange(0, 3),
                        T_right_left.col(3).rowRange(0, 3), R_l, R_r, P_l, P_r,
  Q);
    } else {
      ROS_ERROR("Rectification matrices cannot be calculated without providing
  'T_RIGHT_LEFT' parameter.");
      return;
    }
  }
  */

  cv::initUndistortRectifyMap(K_l, D_l, R_l, P_l.rowRange(0, 3).colRange(0, 3),
                              cv::Size(cols_l, rows_l), CV_32F,
                              left_rectification_map1_,
                              left_rectification_map2_);
  cv::initUndistortRectifyMap(K_r, D_r, R_r, P_r.rowRange(0, 3).colRange(0, 3),
                              cv::Size(cols_r, rows_r), CV_32F,
                              right_rectification_map1_,
                              right_rectification_map2_);

  valid_open_cv_rectify_param_ = true;
}

void OrbSlam2InterfaceStereo::stereoImageCallback(
    const sensor_msgs::ImageConstPtr& msg_left,
    const sensor_msgs::ImageConstPtr& msg_right) {
  // Copy the ros image message to cv::Mat.
  cv_bridge::CvImageConstPtr cv_ptr_left;
  cv::Mat T_C_W_opencv;
  try {
    cv_ptr_left = cv_bridge::toCvShare(msg_left);
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  cv_bridge::CvImageConstPtr cv_ptr_right;
  try {
    cv_ptr_right = cv_bridge::toCvShare(msg_right);
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  if (rectify_input_images_) {
    if (valid_open_cv_rectify_param_) {
      ROS_INFO_ONCE("Input images are going to be rectified using OpenCV.");
      cv::Mat imLeft, imRight;
      cv::remap(cv_ptr_left->image, imLeft, left_rectification_map1_,
                left_rectification_map2_, cv::INTER_LINEAR);
      cv::remap(cv_ptr_right->image, imRight, right_rectification_map1_,
                right_rectification_map2_, cv::INTER_LINEAR);
      // Handing the image to ORB slam for tracking
      T_C_W_opencv = slam_system_->TrackStereo(
          imLeft, imRight, cv_ptr_left->header.stamp.toSec());
    } else {
      ROS_ERROR("Open CV parameters for stereo rectification are missing!");
    }
  } else {
    // Input images are assumed to be already rectified
    // Handing the image to ORB slam for tracking
    ROS_INFO_ONCE("Input images are assumed to be already rectified.");
    T_C_W_opencv =
        slam_system_->TrackStereo(cv_ptr_left->image, cv_ptr_right->image,
                                  cv_ptr_left->header.stamp.toSec());
  }

  // If tracking successfull
  if (!T_C_W_opencv.empty()) {
    // Converting to kindr transform and publishing
    Transformation T_C_W, T_W_C;
    convertOrbSlamPoseToKindr(T_C_W_opencv, &T_C_W);
    T_W_C = T_C_W.inverse();
    publishCurrentPose(T_W_C, msg_left->header);
    // Saving the transform to the member for publishing as a TF
    T_W_C_ = T_W_C;
  }
}

}  // namespace orb_slam_2_interface