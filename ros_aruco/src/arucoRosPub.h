#ifndef ARUCO_ROS_PUB_H
#define ARUCO_ROS_PUB_H

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/MarkerArray.h>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>

#include <Eigen/Geometry>
#include <opencv2/core.hpp>

#include <string>
#include <vector>

// 假设算法层返回的结果结构体定义在 aruco_tracker.h 中
#include "aruco_tracker.h"

class ArucoRosPub {
public:
  ArucoRosPub(ros::NodeHandle& nh, ros::NodeHandle& pnh, std::string &conf_file);
  ~ArucoRosPub() = default;

private:
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<
      sensor_msgs::Image, sensor_msgs::Image>;
  using Synchronizer = message_filters::Synchronizer<SyncPolicy>;

  void imageCallback(const sensor_msgs::ImageConstPtr& left_msg,
                     const sensor_msgs::ImageConstPtr& right_msg);

  void loadParameters();

  void publishDetectedImage(const ArucoResult& result);
  void publishCorners3D(const ArucoResult& result);
  void publishCameraPose(const ArucoResult& result);
  void publishBoardPose(const ArucoResult& result);
  void publishBoardTrajectory(const ArucoResult& result); // 发布轨迹的函数
  
  void publishVisualization(const ArucoResult& result);
  void broadcastTF(const ArucoResult& result);

  static bool toCvMat(const sensor_msgs::ImageConstPtr& msg, cv::Mat& out);

private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  std::string conf;

  // 图像订阅 + 时间同步
  image_transport::ImageTransport it_;
  image_transport::SubscriberFilter left_sub_;
  image_transport::SubscriberFilter right_sub_;
  std::shared_ptr<Synchronizer> sync_;

  // 发布者
  ros::Publisher left_vis_pub_;
  ros::Publisher right_vis_pub_;
  ros::Publisher corners_pub_;
  ros::Publisher pose_pub_;
  ros::Publisher marker_pub_;
  ros::Publisher board_pose_pub_;
  ros::Publisher trajectory_pub_;

  // TF
  tf2_ros::TransformBroadcaster tf_broadcaster_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // 算法
  ArucoTracker tracker_;

  // 参数
  std::string left_topic_;
  std::string right_topic_;

  std::string left_vis_topic_;
  std::string right_vis_topic_;

  std::string left_camera_frame_;
  std::string right_camera_frame_;
  std::string map_frame_;
  std::string marker_frame_prefix_;
  std::string corners_frame_;      // 3D 角点发布坐标系
  std::string corners_topic_;
  std::string pose_topic_;
  std::string board_topic_;
  std::string marker_topic_;
  std::string marker_trajectory_topic_;

  bool publish_tf_;
  bool publish_map_to_camera_;     // 是否广播 map -> camera
  bool publish_camera_to_marker_;  // 是否广播 camera -> marker
  bool publish_board_to_camer;
  bool publish_board_trajectory;
  double tf_publish_rate_;

  size_t max_trajectory_points_ = 1000; // 最大保留的轨迹点数量
  std::deque<geometry_msgs::Point> trajectory_points_;// 保存历史轨迹点，坐标均为左相机坐标系
};

#endif  // ARUCO_ROS_PUB_H