#include "arucoRosPub.h"

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <Eigen/Geometry>
//#include  <opencv2/highgui/highgui.hpp>

// ---------------- 构造 / 初始化 ----------------

ArucoRosPub::ArucoRosPub(ros::NodeHandle& nh, ros::NodeHandle& pnh, std::string &conf_file)
  : nh_(nh),
    pnh_(pnh),
    it_(nh_),
    left_sub_(it_, "", 1),   // topic 稍后 subscribe 时传
    right_sub_(it_, "", 1),
    tf_listener_(tf_buffer_),
    conf(conf_file)
{
 
  loadParameters();
  // 初始化算法
  if (!tracker_.init(pnh_, conf)) {
    ROS_FATAL("[ArucoRosPub] ArucoTracker init failed.");
    ros::shutdown();
    return;
  }

  // 订阅左右原始图像
  left_sub_.subscribe(it_, left_topic_, 1);
  right_sub_.subscribe(it_, right_topic_, 1);

  sync_.reset(new Synchronizer(SyncPolicy(10), left_sub_, right_sub_));
  sync_->registerCallback(
      boost::bind(&ArucoRosPub::imageCallback, this, _1, _2));

  // 发布者：公共结果用 nh_，方便外部订阅
  left_vis_pub_ = nh_.advertise<sensor_msgs::Image>(left_vis_topic_, 1);
  right_vis_pub_ = nh_.advertise<sensor_msgs::Image>(right_vis_topic_, 1);
  corners_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(corners_topic_, 1);
  pose_pub_    = nh_.advertise<geometry_msgs::PoseStamped>(pose_topic_, 1);
  board_pose_pub_    = nh_.advertise<geometry_msgs::PoseStamped>(board_topic_, 1);
  marker_pub_  = nh_.advertise<visualization_msgs::MarkerArray>(marker_topic_, 1);
  trajectory_pub_ =nh_.advertise<visualization_msgs::MarkerArray>(marker_trajectory_topic_, 1);

  ROS_INFO_STREAM("[ArucoRosPub] left  image topic: " << left_sub_.getTopic());
  ROS_INFO_STREAM("[ArucoRosPub] right image topic: " << right_sub_.getTopic());
  ROS_INFO_STREAM("[ArucoRosPub] left  vis topic: " << left_vis_pub_.getTopic());
  ROS_INFO_STREAM("[ArucoRosPub] right vis topic: " << right_vis_pub_.getTopic());
  ROS_INFO_STREAM("[ArucoRosPub] corners topic    : " << corners_pub_.getTopic());
  ROS_INFO_STREAM("[ArucoRosPub] pose topic       : " << pose_pub_.getTopic());
  ROS_INFO_STREAM("[ArucoRosPub] board pose topic : " << board_pose_pub_.getTopic());
  ROS_INFO_STREAM("[ArucoRosPub] marker trace topic : " << trajectory_pub_.getTopic());
  ROS_INFO_STREAM("[ArucoRosPub] publish_tf       : " << publish_tf_);
}

// ---------------- 参数读取 ----------------

void ArucoRosPub::loadParameters() {
  // 输入话题
  pnh_.param<std::string>("left_topic",  left_topic_,  "/image/raw/cam0");
  pnh_.param<std::string>("right_topic", right_topic_, "/image/raw/cam1");

  // 坐标系
  pnh_.param<std::string>("left_camera_frame",  left_camera_frame_,  "camera_left_optical");
  pnh_.param<std::string>("right_camera_frame", right_camera_frame_, "camera_right_optical");
  pnh_.param<std::string>("map_frame",          map_frame_,          "map");
  pnh_.param<std::string>("marker_frame_prefix", marker_frame_prefix_, "aruco_marker_");


  // 3D 角点发布坐标系：默认相机光学坐标系；也可以设为 map
  pnh_.param<std::string>("corners_frame", corners_frame_, left_camera_frame_);

  // 输出话题
  pnh_.param<std::string>("left_vis_topic", left_vis_topic_, "aruco/cam0");
  pnh_.param<std::string>("right_vis_topic",right_vis_topic_, "aruco/cam1");
  pnh_.param<std::string>("corners_topic", corners_topic_, "aruco/corners3d");
  pnh_.param<std::string>("pose_topic",    pose_topic_,    "aruco/camera_pose");
  pnh_.param<std::string>("board_topic",   board_topic_,   "aruco/board_pose");
  pnh_.param<std::string>("marker_topic",  marker_topic_,  "aruco/markers");
  pnh_.param<std::string>("marker_trajectory_topic_",  marker_trajectory_topic_,  "aruco/board_trajectory");
    

  // TF 相关
  pnh_.param<bool>("publish_tf", publish_tf_, true);
  pnh_.param<bool>("publish_map_to_camera", publish_map_to_camera_, false);
  pnh_.param<bool>("publish_camera_to_marker", publish_camera_to_marker_, true);
  pnh_.param<bool>("publish_board_trajectory", publish_board_trajectory, true);
  pnh_.param<double>("tf_publish_rate", tf_publish_rate_, 30.0);
}

// ---------------- 图像回调 ----------------

void ArucoRosPub::imageCallback(const sensor_msgs::ImageConstPtr& left_msg,
                                const sensor_msgs::ImageConstPtr& right_msg) {
  cv::Mat left_img, right_img;
  // ROS_INFO_STREAM("synced pair: L=" << left_msg->header.stamp.toSec()
  //                   << " R=" << right_msg->header.stamp.toSec() 
  //                   <<" tdiff = " << (left_msg->header.stamp - right_msg->header.stamp).toSec());

  if (!toCvMat(left_msg, left_img) || !toCvMat(right_msg, right_img)) {
    ROS_WARN_THROTTLE(2.0, "[ArucoRosPub] cv_bridge convert failed.");
    return;
  }

  ArucoResult result = tracker_.process(left_img,right_img);
  result.stamp = left_msg->header.stamp;

  if (!result.valid) {
    ROS_DEBUG_THROTTLE(2.0, "[ArucoRosPub] no valid aruco result.");
    return;
  }

  publishDetectedImage(result);
  publishCorners3D(result);
  publishBoardPose(result);
  publishCameraPose(result);
  publishBoardTrajectory(result);
  publishVisualization(result);

  if (publish_tf_) {
    broadcastTF(result);
  }
}


void ArucoRosPub::publishDetectedImage(const ArucoResult& result){
  if (result.detect_images.empty() || result.detect_images.size() < 2) return;

  std_msgs::Header header;
  header.stamp = ros::Time::now();
  header.frame_id = left_camera_frame_;

  try {
      // 发布左图像
      cv_bridge::CvImage left_cv_image(
          header,
          sensor_msgs::image_encodings::BGR8,
          result.detect_images[0]
      );
      left_vis_pub_.publish(left_cv_image.toImageMsg());

      // 发布右图像
      cv_bridge::CvImage right_cv_image(
          header,
          sensor_msgs::image_encodings::BGR8,
          result.detect_images[1]
      );
      right_vis_pub_.publish(right_cv_image.toImageMsg());
  }
  catch (const cv_bridge::Exception& e) {
      ROS_ERROR("Failed to convert image: %s", e.what());
  }
}


// ---------------- 3D 角点发布 ----------------
void ArucoRosPub::publishCorners3D(const ArucoResult& result) {
  if (result.corners3d.empty()) return;

  std::string out_frame = corners_frame_;

  // 如果希望发布到 map，这里用 tf2 把点从相机坐标系转换到 map
  std::vector<cv::Point3f> pts = result.corners3d;
  if (out_frame != left_camera_frame_) {
    try {
      geometry_msgs::TransformStamped tf =
          tf_buffer_.lookupTransform(out_frame, left_camera_frame_,
                                     result.stamp, ros::Duration(0.05));
      Eigen::Isometry3d T = tf2::transformToEigen(tf);
      for (auto& p : pts) {
        Eigen::Vector3d v(p.x, p.y, p.z);
        v = T * v;
        p = cv::Point3f(static_cast<float>(v.x()),
                        static_cast<float>(v.y()),
                        static_cast<float>(v.z()));
      }
    } catch (const tf2::TransformException& ex) {
      ROS_WARN_THROTTLE(2.0, "[ArucoRosPub] transform to %s failed: %s",
                        out_frame.c_str(), ex.what());
      // 转换失败就退回相机坐标系，保证数据仍可发布
      out_frame = left_camera_frame_;
      pts = result.corners3d;
    }

  }

  sensor_msgs::PointCloud2 cloud;
  cloud.header.stamp = result.stamp;
  cloud.header.frame_id = out_frame;
  cloud.height = 1;
  cloud.width = static_cast<uint32_t>(pts.size());
  cloud.is_dense = true;
  cloud.is_bigendian = false;

  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  modifier.resize(pts.size());

  sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");
  for (size_t i = 0; i < pts.size(); ++i, ++iter_x, ++iter_y, ++iter_z) {
    *iter_x = pts[i].x;
    *iter_y = pts[i].y;
    *iter_z = pts[i].z;
  }

  corners_pub_.publish(cloud);
}

// ---------------- 相机位姿发布 ----------------
void ArucoRosPub::publishBoardPose(const ArucoResult& result) {
  // result.T_cam_marker：marker 在相机下的位姿 
  // 发布标定板相对相机的位姿

  geometry_msgs::PoseStamped pose_msg;
  pose_msg.header.stamp = result.stamp;

  Eigen::Isometry3d T_cam_marker = result.T_cam_marker;
  pose_msg.header.frame_id = left_camera_frame_;

  const Eigen::Vector3d& t = T_cam_marker.translation();
  Eigen::Quaterniond q(T_cam_marker.rotation());

  pose_msg.pose.position.x = t.x();
  pose_msg.pose.position.y = t.y();
  pose_msg.pose.position.z = t.z();
  pose_msg.pose.orientation.x = q.x();
  pose_msg.pose.orientation.y = q.y();
  pose_msg.pose.orientation.z = q.z();
  pose_msg.pose.orientation.w = q.w();

  board_pose_pub_.publish(pose_msg);
}

void ArucoRosPub::publishCameraPose(const ArucoResult& result) {
  // result.T_cam_marker：marker 在相机下的位姿
  // 若要发布相机在 map 下的位姿，需要 T_map_marker
  // 若 map 已由外部 SLAM 定义，请不要重复发布 map -> camera。
  // 这里假设发的是 camera 在 marker 下的位姿的逆（pose of camera in marker frame）作为 PoseStamped 的备用输出。
  // 更常见的做法：如果你有 map -> marker 的固定 TF，可以查出来再合成。

  geometry_msgs::PoseStamped pose_msg;
  pose_msg.header.stamp = result.stamp;

  // 默认把相机位姿表达在 marker 坐标系下：T_marker_cam = T_cam_marker.inverse()
  Eigen::Isometry3d T_marker_cam = result.T_cam_marker.inverse();

  pose_msg.header.frame_id = marker_frame_prefix_ + std::to_string(
      result.ids.empty() ? 0 : result.ids.front());

  const Eigen::Vector3d& t = T_marker_cam.translation();
  Eigen::Quaterniond q(T_marker_cam.rotation());

  pose_msg.pose.position.x = t.x();
  pose_msg.pose.position.y = t.y();
  pose_msg.pose.position.z = t.z();
  pose_msg.pose.orientation.x = q.x();
  pose_msg.pose.orientation.y = q.y();
  pose_msg.pose.orientation.z = q.z();
  pose_msg.pose.orientation.w = q.w();

  pose_pub_.publish(pose_msg);
}

void ArucoRosPub::publishBoardTrajectory(const ArucoResult& result)
{
    // 1. 获取标定板中心在左相机坐标系下的位置
    const Eigen::Vector3d center = result.T_cam_marker.translation();
    geometry_msgs::Point current_point;
    current_point.x = center.x();
    current_point.y = center.y();
    current_point.z = center.z();

    // 平移部分 (translation)
    const auto& R = result.T_cam_marker.rotation();
    const auto& t = result.T_cam_marker.translation();
    ROS_INFO_STREAM("board R = \n" << R);
    ROS_INFO_STREAM("board t = \n" << t);
    ROS_INFO_STREAM("board loc = "
                << current_point.x << " "
                << current_point.y << " "
                << current_point.z);
    // 2. 保存当前中心点
    trajectory_points_.push_back(current_point);

    // 3. 限制轨迹点数量，避免内存无限增长
    while (trajectory_points_.size() > max_trajectory_points_) {
        trajectory_points_.pop_front();
    }

    // 4. 构造 MarkerArray
    visualization_msgs::MarkerArray marker_array;

    // --------------------------------------------------
    // Marker 0：标定板中心的实时位置
    // --------------------------------------------------
    visualization_msgs::Marker center_marker;
    center_marker.header.stamp = result.stamp;
    center_marker.header.frame_id = left_camera_frame_;
    center_marker.ns = "board_trajectory";
    center_marker.id = 0;
    center_marker.type = visualization_msgs::Marker::SPHERE;
    center_marker.action = visualization_msgs::Marker::ADD;

    center_marker.pose.position = current_point;
    center_marker.pose.orientation.w = 1.0;

    center_marker.scale.x = 0.02;
    center_marker.scale.y = 0.02;
    center_marker.scale.z = 0.02;

    center_marker.color.r = 1.0;
    center_marker.color.g = 0.0;
    center_marker.color.b = 0.0;
    center_marker.color.a = 1.0;

    center_marker.lifetime = ros::Duration(0);
    marker_array.markers.push_back(center_marker);

    // --------------------------------------------------
    // Marker 1：历史运动轨迹
    // --------------------------------------------------
    visualization_msgs::Marker trajectory_marker;

    trajectory_marker.header.stamp = result.stamp;
    trajectory_marker.header.frame_id = left_camera_frame_;

    trajectory_marker.ns = "board_trajectory";
    trajectory_marker.id = 1;
    trajectory_marker.type = visualization_msgs::Marker::LINE_STRIP;
    trajectory_marker.action = visualization_msgs::Marker::ADD;
    // 线宽，单位为米
    trajectory_marker.scale.x = 0.005;
    trajectory_marker.color.r = 0.0;
    trajectory_marker.color.g = 1.0;
    trajectory_marker.color.b = 0.0;
    trajectory_marker.color.a = 1.0;
    trajectory_marker.pose.orientation.w = 1.0;
    // 按时间顺序连接历史轨迹点
    trajectory_marker.points.assign(
        trajectory_points_.begin(),
        trajectory_points_.end());

    trajectory_marker.lifetime = ros::Duration(0);
    marker_array.markers.push_back(trajectory_marker);
    // 5. 发布中心点和轨迹
    trajectory_pub_.publish(marker_array);
}



// ---------------- Marker 可视化 ----------------

void ArucoRosPub::publishVisualization(const ArucoResult& result) {
  visualization_msgs::MarkerArray arr;
  int id = 0;

  for (size_t i = 0; i < result.corners3d.size(); ++i, ++id) {
    visualization_msgs::Marker m;
    m.header.stamp = result.stamp;
    m.header.frame_id = left_camera_frame_;
    m.ns = "aruco_corners";
    m.id = id;
    m.type = visualization_msgs::Marker::SPHERE;
    m.action = visualization_msgs::Marker::ADD;
    m.scale.x = m.scale.y = m.scale.z = 0.02;

    m.pose.position.x = result.corners3d[i].x;
    m.pose.position.y = result.corners3d[i].y;
    m.pose.position.z = result.corners3d[i].z;
    m.pose.orientation.w = 1.0;

    m.color.a = 1.0;
    m.color.r = 0.0;
    m.color.g = 1.0;
    m.color.b = 0.0;

    arr.markers.push_back(m);
  }

  if (!arr.markers.empty()) {
    marker_pub_.publish(arr);
  }
}

// ---------------- 动态 TF ----------------

void ArucoRosPub::broadcastTF(const ArucoResult& result) {
  // 场景 1：直接把 marker 在相机下的位姿广播为 camera -> marker
  if (publish_camera_to_marker_) {
    for (size_t i = 0; i < result.ids.size(); ++i) {
      int marker_id = result.ids[i];

      // 注意：这里假设 result.T_cam_marker 是“单个 marker 在相机下的位姿”。
      // 如果一次检测到多个 marker，需要 tracker 返回每个 marker 的位姿，
      // 这里示例只广播第一个作为占位。
      if (i != 0) break;

      geometry_msgs::TransformStamped tf_msg;
      tf_msg.header.stamp = result.stamp;
      tf_msg.header.frame_id = left_camera_frame_;
      tf_msg.child_frame_id = marker_frame_prefix_ + std::to_string(marker_id);

      const Eigen::Vector3d& t = result.T_cam_marker.translation();
      Eigen::Quaterniond q(result.T_cam_marker.rotation());

      tf_msg.transform.translation.x = t.x();
      tf_msg.transform.translation.y = t.y();
      tf_msg.transform.translation.z = t.z();
      tf_msg.transform.rotation.x = q.x();
      tf_msg.transform.rotation.y = q.y();
      tf_msg.transform.rotation.z = q.z();
      tf_msg.transform.rotation.w = q.w();

      tf_broadcaster_.sendTransform(tf_msg);
    }
  }

  // 场景 2：直接发布 map -> camera，仅在外部没有其他 map->camera 来源时打开
  if (publish_map_to_camera_) {
    try {
      // 需要已知 map -> marker 的固定 TF
      int marker_id = result.ids.empty() ? 0 : result.ids.front();
      std::string marker_frame = marker_frame_prefix_ + std::to_string(marker_id);

      geometry_msgs::TransformStamped tf_map_marker =
          tf_buffer_.lookupTransform(map_frame_, marker_frame,
                                     result.stamp, ros::Duration(0.05));

      Eigen::Isometry3d T_map_marker = tf2::transformToEigen(tf_map_marker);
      Eigen::Isometry3d T_cam_marker = result.T_cam_marker;
      Eigen::Isometry3d T_map_cam = T_map_marker * T_cam_marker.inverse();

      geometry_msgs::TransformStamped tf_msg;
      tf_msg.header.stamp = result.stamp;
      tf_msg.header.frame_id = map_frame_;
      tf_msg.child_frame_id = left_camera_frame_;

      const Eigen::Vector3d& t = T_map_cam.translation();
      Eigen::Quaterniond q(T_map_cam.rotation());

      tf_msg.transform.translation.x = t.x();
      tf_msg.transform.translation.y = t.y();
      tf_msg.transform.translation.z = t.z();
      tf_msg.transform.rotation.x = q.x();
      tf_msg.transform.rotation.y = q.y();
      tf_msg.transform.rotation.z = q.z();
      tf_msg.transform.rotation.w = q.w();

      tf_broadcaster_.sendTransform(tf_msg);
    } catch (const tf2::TransformException& ex) {
      ROS_WARN_THROTTLE(2.0, "[ArucoRosPub] lookup map->marker failed: %s", ex.what());
    }
  }
}

// ---------------- 工具函数 ----------------

bool ArucoRosPub::toCvMat(const sensor_msgs::ImageConstPtr& msg, cv::Mat& out) {
  try {
    cv_bridge::CvImageConstPtr cv_ptr;
    if (msg->encoding == sensor_msgs::image_encodings::MONO8 ||
        msg->encoding == sensor_msgs::image_encodings::BGR8 ||
        msg->encoding == sensor_msgs::image_encodings::RGB8) {
      cv_ptr = cv_bridge::toCvShare(msg);
    } else {
      cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
    }
    out = cv_ptr->image.clone();
    return true;
  } catch (const cv_bridge::Exception& ex) {
    ROS_ERROR("[ArucoRosPub] cv_bridge exception: %s", ex.what());
    return false;
  }
}