## Real-Time ArUco Board Pose Estimation and Visualization in ROS
[▶ Watch the demo on YouTube](https://www.youtube.com/watch?v=Qe8zpcCgD2U)

This video demonstrates real-time ArUco board pose estimation using ROS and computer vision. By detecting ArUco markers from stereo camera images and reconstructing 3D points through stereo triangulation, the system estimates the board's 3D pose and visualizes its position in real time in RViz.

### Key Features
- **ArUco Marker Detection:** Detects board markers from camera images.
- **Stereo 3D Reconstruction:** Uses corresponding left and right image corners to reconstruct 3D points.
- **Pose Estimation:** Estimates the board's rotation and translation using 3D point correspondences.
- **Real-Time ROS Visualization:** Publishes pose and 3D visualization data for display in RViz.
- **Board Pose Tracking:** Visualizes the board's position and movement as it moves in front of the cameras.

### Technologies
- ROS (Robot Operating System)
- C++
- OpenCV
- Eigen
- Stereo Vision
- ArUco Markers
- RViz

This project demonstrates how stereo vision and fiducial markers can be integrated into a ROS-based perception pipeline for real-time 3D pose estimation and visualization.

#ROS #OpenCV #ArUco #StereoVision #RViz #ComputerVision #Robotics #PoseEstimation