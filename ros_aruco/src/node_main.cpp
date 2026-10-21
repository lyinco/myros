#include <ros/ros.h>
#include "arucoRosPub.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "aruco_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  std::string config_file = "/home/lyin/catkin_ws/src/myros/ros_aruco/config/720/stereo_config.yaml";
  ArucoRosPub pub_node(nh, pnh, config_file);

  ros::spin();
  return 0;
}