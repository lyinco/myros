#include "aruco_tracker.h"
#include <opencv2/highgui.hpp>

inline Eigen::Isometry3d cvPoseToEigenIsometry(const cv::Matx33f& R_cv, const cv::Vec3f& t_cv)
{
    Eigen::Matrix3d R;
    R << R_cv(0,0), R_cv(0,1), R_cv(0,2),
         R_cv(1,0), R_cv(1,1), R_cv(1,2),
         R_cv(2,0), R_cv(2,1), R_cv(2,2);
    Eigen::Vector3d t(static_cast<double>(t_cv[0]),
                      static_cast<double>(t_cv[1]),
                      static_cast<double>(t_cv[2]));

     if (!R.allFinite() || !t.allFinite())
    {
        throw std::runtime_error(
            "cvPoseToEigenIsometry: NaN or Inf detected");
    }

    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    T.linear() = R;
    T.translation() = t;
    return T;
}

bool ArucoTracker::init(ros::NodeHandle& pnh, const std::string& config_file) {
  // 这里可以读取参数，初始化相机内参、Aruco 字典等
  // 例如：
  // pnh.param<std::string>("aruco_dictionary", aruco_dictionary_, "DICT_4X4_50");

  board_ = ArucoBoard();
  bool res = board_.init();
  if (!res) return false;

  cam_calib_ = CamCalib();
  cam_calib_.setConfigFile(config_file);
  return true;
}

ArucoResult ArucoTracker::process(const cv::Mat& left, const cv::Mat& right) {
  ArucoResult result;
  result.stamp = ros::Time::now();

  cv::Mat vis1 = left.clone();
  cv::Mat vis2 = right.clone();  

  std::map<int, std::vector<cv::Point2f>> corners1 = board_.detectMarkerCorners(left);
  std::map<int, std::vector<cv::Point2f>> corners2 = board_.detectMarkerCorners(right);

  cv::Scalar color_true=cv::Scalar(0, 255, 0);
  vis1 = board_.drawMarkers(vis1, corners1, color_true, 3, true);
  vis2 = board_.drawMarkers(vis2, corners2, color_true, 3, true);

  std::map<int, std::vector<cv::Point3f>>  corners3d = getAruco3d(corners1, corners2);
  for(const auto kv:corners3d)
  {
    result.ids.emplace_back(kv.first);
    cv::Mat mat(kv.second);  // Nx1 CV_32FC3
    cv::Scalar mean = cv::mean(mat);
    result.corners3d.emplace_back(cv::Point3f(mean[0], mean[1], mean[2]));
  }
  result.stamp_done = ros::Time::now();
  

  std::map<int, std::vector<cv::Point2f>> reproj1;
  std::map<int, std::vector<cv::Point2f>> reproj2;
  for(const auto&kv:corners3d){
    std::vector<cv::Point2f> lf, rt;
    std::vector<cv::Point3f> pts3ds = kv.second;
    for(int ii=0;ii<4;ii++)
    {
      cv::Point2f ptl_, ptr_;
      cam_calib_.projectPointToStereoImages(pts3ds[ii], ptl_, ptr_);
      lf.push_back(ptl_);
      rt.push_back(ptr_);
    }

    reproj1[kv.first] = lf;
    reproj2[kv.first] = rt;
  }

//   cv::Scalar color=cv::Scalar(0, 0, 255);
//   vis1 = board_.drawMarkers(vis1, reproj1, color, 1, false);
//   vis2 = board_.drawMarkers(vis2, reproj2, color, 1, false);
result.detect_images.push_back(vis1);
result.detect_images.push_back(vis2);

  // cv::imshow("left", vis1);
  // cv::imshow("right", vis2);
  // cv::waitKey(1);

  // 实现 Aruco 标记检测和位姿估计逻辑
  cv::Matx33f poser;
  cv::Vec3f poset;
  bool success = estimateMarkerPose(corners3d, poser, poset);
  if(success){
    result.T_cam_marker = cvPoseToEigenIsometry(poser, poset);
  }
  result.valid = true;
  return result;
}


std::map<int, std::vector<cv::Point3f>> ArucoTracker::getAruco3d(const std::map<int, std::vector<cv::Point2f>>& corners1, 
  std::map<int, std::vector<cv::Point2f>> corners2){

    std::map<int, std::vector<cv::Point3f>> result;
    if (corners1.empty() || corners2.empty()) return result;

    std::vector<cv::Point3f> pts_cam;
    std::vector<cv::Point3f> pts_obj;
    std::map<int, std::vector<cv::Point3f>> world_obj = board_.getBoardCorners();


    for (const auto& kv : corners1) {

      int id = kv.first;

      // 右目没有该 marker，跳过
      auto it = corners2.find(id);
      if (it == corners2.end()) continue;

      // 角点数必须都是 4
      if (kv.second.size() != 4) continue;
      if (it->second.size() != 4) continue;

      for (int i = 0; i < 4; ++i) {
          cv::Point2f pt1(kv.second[i].x, kv.second[i].y);
          cv::Point2f pt2(it->second[i].x, it->second[i].y);
          cv::Point3f pt3d;
          cam_calib_.triangulatePoint(pt1, pt2, pt3d);
          result[id].push_back(pt3d);
      }
    
      auto world_it = world_obj.find(id);
      if (world_it == world_obj.end()) continue;
      if (world_it->second.size() != 4) continue;
      pts_cam.push_back(result[id][0]);
      pts_obj.push_back(world_obj[id][0]);
    }

    if (pts_cam.empty() || pts_cam.size()!=pts_obj.size()) return result;

    Eigen::Matrix3d matr;
    Eigen::Vector3d matt;
    estimateRigidTransform(pts_obj, pts_cam,  matr, matt);

    // 所有点转换到
    for (const auto& kv : world_obj){
        int id = kv.first;
        auto it = result.find(id);
        if (it == result.end()) {
            std::vector<cv::Point3f> mm;
            transCoors(kv.second,  matr, matt, mm);
            result[id] = mm;
        }
    }
    
    return result;
}


std::map<int, std::vector<cv::Point3f>> ArucoTracker::getAruco3d(const cv::Mat& img1, const cv::Mat& img2){

  std::map<int, std::vector<cv::Point3f>> result;
  if (img1.empty() || img2.empty()) return result;

  std::map<int, std::vector<cv::Point2f>> corners1 = board_.detectMarkerCorners(img1);
  std::map<int, std::vector<cv::Point2f>> corners2 = board_.detectMarkerCorners(img2);
  return getAruco3d(corners1, corners2);
}

bool ArucoTracker::estimateMarkerPose(
    const std::map<int, std::vector<cv::Point3f>>& P_cam, cv::Matx33f& R, cv::Vec3f& t)
{
    if (P_cam.size()<1 )
        return false;
    
    cv::Vec3f centroidCam(0, 0, 0);
    cv::Vec3f centroidWorld(0, 0, 0);
    int shared_nums = 0;
    std::vector<cv::Point3f> pts_cams;
    std::vector<cv::Point3f> pts_world;
    std::map<int, std::vector<cv::Point3f>> board_corners = board_.getBoardCorners();
    for(const auto &kv: P_cam){
      int mid = kv.first;
      auto it = board_corners.find(mid);
      if (it == board_corners.end()) continue;

      const auto& corners_cam = kv.second;
      const auto& corners_world = it->second;
      if (corners_cam.empty() || corners_world.empty())
            continue;
      
      // 相机坐标系下的 marker 中心
      const cv::Scalar mean1 = cv::mean(cv::Mat(corners_cam));
      // 世界坐标系下的 marker 中心
      const cv::Scalar mean2 = cv::mean(cv::Mat(corners_world));

      const cv::Point3f center_cam(
          static_cast<float>(mean1[0]),
          static_cast<float>(mean1[1]),
          static_cast<float>(mean1[2]));

      const cv::Point3f center_world(
          static_cast<float>(mean2[0]),
          static_cast<float>(mean2[1]),
          static_cast<float>(mean2[2]));

      pts_cams.emplace_back(center_cam);
      pts_world.emplace_back(center_world);

      centroidCam += cv::Vec3f(
          center_cam.x, center_cam.y, center_cam.z);

      centroidWorld += cv::Vec3f(
          center_world.x, center_world.y, center_world.z);

      ++shared_nums;
    }

    if (shared_nums<1) return false;

    centroidCam *= 1.0f / shared_nums;
    centroidWorld *= 1.0f / shared_nums;
    // 3. 构造协方差矩阵 H
    cv::Matx33d H = cv::Matx33d::zeros();
    for (int i = 0; i < shared_nums; ++i)
    {
        cv::Vec3f p(pts_cams[i].x, pts_cams[i].y, pts_cams[i].z);
        cv::Vec3f q(pts_world[i].x, pts_world[i].y, pts_world[i].z);

        p -= centroidCam;
        q -= centroidWorld;

        H += cv::Matx33f(
            q[0] * p[0], q[0] * p[1], q[0] * p[2],
            q[1] * p[0], q[1] * p[1], q[1] * p[2],
            q[2] * p[0], q[2] * p[1], q[2] * p[2]);
    }

    // 4. SVD 分解
    cv::Mat w, U, Vt;
    cv::SVD::compute(cv::Mat(H), w, U, Vt);

    // 调试：确认 SVD 输出
    ROS_INFO("U: rows=%d cols=%d type=%d",
            U.rows, U.cols, U.type());
    ROS_INFO("Vt: rows=%d cols=%d type=%d",
            Vt.rows, Vt.cols, Vt.type());

    ROS_INFO("U = \n%f %f %f\n%f %f %f\n%f %f %f",
        U.at<double>(0,0), U.at<double>(0,1), U.at<double>(0,2),
        U.at<double>(1,0), U.at<double>(1,1), U.at<double>(1,2),
        U.at<double>(2,0), U.at<double>(2,1), U.at<double>(2,2));

    // 5. 使用 double 计算旋转矩阵
    cv::Matx33d Um, Vtm;

    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            Um(i, j) = U.at<double>(i, j);
            Vtm(i, j) = Vt.at<double>(i, j);
        }
    }

    cv::Matx33d V = Vtm.t();
    cv::Matx33d Rd = V * Um.t();

    // 防止反射矩阵
    if (cv::determinant(cv::Mat(Rd)) < 0.0)
    {
        V(0, 2) *= -1.0;
        V(1, 2) *= -1.0;
        V(2, 2) *= -1.0;

        Rd = V * Um.t();
    }

    // 转换为 float
    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            R(i, j) = static_cast<float>(Rd(i, j));
        }
    }

    // 6. 计算平移向量
    t = centroidCam - R * centroidWorld;
    ROS_INFO("cam pose R = %f %f %f %f %f %f %f %f %f ", 
      R(0,0),R(0,1), R(0,2),
      R(1,0),R(1,1), R(1,2), 
      R(2,0),R(2,1), R(2,2));
    ROS_INFO("cam pose T = %f %f %f", t(0), t(1), t(2));
    return true;
}

// Q=RP+t
bool ArucoTracker::estimateRigidTransform(const std::vector<cv::Point3f>& P, const std::vector<cv::Point3f>& Q, Eigen::Matrix3d& R, Eigen::Vector3d& t)
{
    if (P.size() != Q.size() || P.size() < 3) {
        std::cerr << "Error: Need at least 3 corresponding points."
                  << std::endl;
        return false;
    }

    const int n = static_cast<int>(P.size());

    // 1. 计算质心
    Eigen::Vector3d centroid_P = Eigen::Vector3d::Zero();
    Eigen::Vector3d centroid_Q = Eigen::Vector3d::Zero();

    for (int i = 0; i < n; ++i) {
        centroid_P += Eigen::Vector3d(P[i].x, P[i].y, P[i].z);
        centroid_Q += Eigen::Vector3d(Q[i].x, Q[i].y, Q[i].z);
    }

    centroid_P /= n;
    centroid_Q /= n;

    // 2. 构造协方差矩阵
    Eigen::Matrix3d H = Eigen::Matrix3d::Zero();

    for (int i = 0; i < n; ++i) {
        Eigen::Vector3d p(P[i].x, P[i].y, P[i].z);
        Eigen::Vector3d q(Q[i].x, Q[i].y, Q[i].z);

        p -= centroid_P;
        q -= centroid_Q;

        H += p * q.transpose();
    }

    // 3. SVD 分解
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(
        H, Eigen::ComputeFullU | Eigen::ComputeFullV);

    Eigen::Matrix3d U = svd.matrixU();
    Eigen::Matrix3d V = svd.matrixV();

    // 4. 求旋转矩阵，避免反射
    Eigen::Matrix3d D = Eigen::Matrix3d::Identity();

    if ((V * U.transpose()).determinant() < 0.0) {
        D(2, 2) = -1.0;
    }

    R = V * D * U.transpose();
    // 5. 求平移向量
    t = centroid_Q - R * centroid_P;
    return true;
}

// 将 cv::Point3f 转换为 Eigen::Vector3d
Eigen::Vector3d ArucoTracker::toEigen(const cv::Point3f& p)
{
    return Eigen::Vector3d(p.x, p.y, p.z);
}

cv::Point3f ArucoTracker::toPointf(const Eigen::Vector3d& p)
{
    return cv::Point3f(
        static_cast<float>(p(0)),
        static_cast<float>(p(1)),
        static_cast<float>(p(2))
    );
}


bool ArucoTracker::transCoors(
    const std::vector<cv::Point3f>& P,
    Eigen::Matrix3d& R,
    Eigen::Vector3d& t,
    std::vector<cv::Point3f>& Q)
{
    Q.clear();
    Q.reserve(P.size());

    for (size_t i = 0; i < P.size(); ++i) {
        Eigen::Vector3d point_P(
            P[i].x,
            P[i].y,
            P[i].z
        );

        Eigen::Vector3d point_Q = R * point_P + t;

        Q.push_back(toPointf(point_Q));
    }

    return true;
}
