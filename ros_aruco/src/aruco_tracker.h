#pragma once

#include <vector>
#include <map>
#include <ros/ros.h>
#include <opencv2/core/types.hpp>
#include <Eigen/Dense>

#include "utility/aruco_board.h"
#include "utility/cam_calib.h"


struct ArucoResult {
  ros::Time stamp;
  ros::Time stamp_done;
  std::vector<int> ids;
  std::vector<cv::Point3f> corners3d;
  Eigen::Isometry3d T_cam_marker;   // marker 在相机下的位姿
  std::vector<cv::Mat> detect_images;
  bool valid = false;
};

class ArucoTracker {
public:
  bool init(ros::NodeHandle& pnh, const std::string &config_file);
  ArucoResult process(const cv::Mat& left, const cv::Mat& right);

private:
  ArucoBoard board_;
  CamCalib cam_calib_;

  std::map<int, std::vector<cv::Point3f>> getAruco3d(const cv::Mat& img1, const cv::Mat& img2);
  std::map<int, std::vector<cv::Point3f>> getAruco3d(const std::map<int, std::vector<cv::Point2f>>& corners1, 
  std::map<int, std::vector<cv::Point2f>> corners2);
  bool estimateMarkerPose(const std::map<int, std::vector<cv::Point3f>>& P_cam, cv::Matx33f& R,cv::Vec3f& t);
  static bool estimateRigidTransform(const std::vector<cv::Point3f>& P,const std::vector<cv::Point3f>& Q, Eigen::Matrix3d& R, Eigen::Vector3d& t);
  static bool transCoors(const std::vector<cv::Point3f>& P, Eigen::Matrix3d& R, Eigen::Vector3d& t, std::vector<cv::Point3f>& Q);
  static cv::Point3f toPointf(const Eigen::Vector3d& p);
  static Eigen::Vector3d toEigen(const cv::Point3f& p);


};