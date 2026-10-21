#include <ros/ros.h>
#include "cam_calib.h"
#include "parameters.h"
#include "camodocal/camera_models/CameraFactory.h"
#include "camodocal/camera_models/PinholeCamera.h"
// #include <opencv2/calib3d.hpp>
// #include <opencv2/opencv.hpp>


void CamCalib::setConfigFile(const std::string &conf_file){
    conf_file_ = conf_file;
    ROS_INFO("config_file = %s", conf_file_);
    readParameters(conf_file_);
    readIntrinsicParameter(CAM_NAMES, RIC, TIC);
}

bool CamCalib::triangulatePoint(const cv::Point2f& uvl, const cv::Point2f& uvr, cv::Point3f& point3D)
{
    if (m1.empty() || m2.empty() ||
        d1.empty() || d2.empty() ||
        matr.empty() || matt.empty())
    {
        return false;
    }

    // 统一使用 double 精度
    cv::Mat K1, K2, D1, D2, R64, T64;

    m1.convertTo(K1, CV_32F);
    m2.convertTo(K2, CV_32F);
    d1.convertTo(D1, CV_32F);
    d2.convertTo(D2, CV_32F);
    matr.convertTo(R64, CV_32F);
    matt.convertTo(T64, CV_32F);

    if (K1.rows != 3 || K1.cols != 3 ||
        K2.rows != 3 || K2.cols != 3 ||
        R64.rows != 3 || R64.cols != 3 ||
        T64.total() != 3)
    {
        return false;
    }

    T64 = T64.reshape(1, 3);

    // 1. 对左右图像中的匹配点分别去畸变
    // 不指定 P，因此输出是归一化相机坐标 (x, y)，不是像素坐标
    std::vector<cv::Point2f> ptsL = {uvl};
    std::vector<cv::Point2f> ptsR = {uvr};
    // std::vector<cv::Point2f> ptsL = {cv::Point2f(885.4684, 507.1514)};
    // std::vector<cv::Point2f> ptsR = {cv::Point2f(1037.6409, 437.26007)};

    std::vector<cv::Point2f> normL, normR;

    cv::undistortPoints(ptsL, normL, K1, D1);
    cv::undistortPoints(ptsR, normR, K2, D2);

    // 2. 构建归一化坐标系下的投影矩阵
    // 左相机：P1 = [I | 0]
    cv::Mat P1 = cv::Mat::zeros(3, 4, CV_32F);
    cv::Mat I = cv::Mat::eye(3, 3, CV_32F);
    I.copyTo(P1(cv::Rect(0, 0, 3, 3)));

    // 右相机：P2 = [R | T]
    cv::Mat P2;
    cv::hconcat(R64, T64, P2);

    // 3. 三角测量
    cv::Mat points4D;
    cv::triangulatePoints(P1, P2, normL, normR, points4D);

    // 4. 齐次坐标转为普通三维坐标
    float w = points4D.at<float>(3, 0);

    if (!std::isfinite(w) || std::abs(w) < 1e-12)
        return false;

    float X = points4D.at<float>(0, 0) / w;
    float Y = points4D.at<float>(1, 0) / w;
    float Z = points4D.at<float>(2, 0) / w;

    if (!std::isfinite(X) ||
        !std::isfinite(Y) ||
        !std::isfinite(Z))
    {
        return false;
    }

    // 5. 检查点是否位于左右相机的前方
    cv::Mat Xl = (cv::Mat_<float>(3, 1) << X, Y, Z);
    cv::Mat Xr = R64 * Xl + T64;

    if (Z <= 0.0 || Xr.at<float>(2, 0) <= 0.0)
        return false;

    point3D = cv::Point3f(X, Y, Z);
    return true;
}



bool CamCalib::projectPointToStereoImages(const cv::Point3f& pt3d, cv::Point2f& uvl, cv::Point2f& uvr)
{
    if (m1.empty() || m2.empty() ||
        d1.empty() || d2.empty() ||
        matr.empty() || matt.empty())
    {
        return false;
    }

    cv::Mat K1, K2, D1, D2, R64, T64;

    m1.convertTo(K1, CV_32F);
    m2.convertTo(K2, CV_32F);
    d1.convertTo(D1, CV_32F);
    d2.convertTo(D2, CV_32F);
    matr.convertTo(R64, CV_32F);
    matt.convertTo(T64, CV_32F);

    if (K1.rows != 3 || K1.cols != 3 ||
        K2.rows != 3 || K2.cols != 3 ||
        R64.rows != 3 || R64.cols != 3 ||
        T64.total() != 3)
    {
        return false;
    }

    T64 = T64.reshape(1, 3);

    // 检查输入点是否为有限值
    if (!std::isfinite(pt3d.x) ||
        !std::isfinite(pt3d.y) ||
        !std::isfinite(pt3d.z))
    {
        return false;
    }

    // 左相机坐标系下的点
    if (pt3d.z <= 0.0)
        return false;

    // 转换到右相机坐标系
    cv::Mat pointL = (cv::Mat_<float>(3, 1)
        << pt3d.x, pt3d.y, pt3d.z);

    cv::Mat pointR = R64 * pointL + T64;

    float Xr = pointR.at<float>(0, 0);
    float Yr = pointR.at<float>(1, 0);
    float Zr = pointR.at<float>(2, 0);

    if (!std::isfinite(Zr) || Zr <= 0.0)
        return false;

    // 左相机投影：
    // pt3d 已在左相机坐标系，外参使用单位旋转和零平移
    std::vector<cv::Point3f> pointsL = {pt3d};
    std::vector<cv::Point2f> imagePointsL;

    cv::Mat rvecZero = cv::Mat::zeros(3, 1, CV_32F);
    cv::Mat tvecZero = cv::Mat::zeros(3, 1, CV_32F);

    cv::projectPoints(
        pointsL, rvecZero, tvecZero,
        K1, D1, imagePointsL);

    // 右相机投影：
    // 使用 R、T 将左相机坐标系的点变换到右相机
    cv::Mat rvec;
    cv::Rodrigues(R64, rvec);

    std::vector<cv::Point3f> pointsR = {pt3d};
    std::vector<cv::Point2f> imagePointsR;

    cv::projectPoints(
        pointsR, rvec, T64,
        K2, D2, imagePointsR);

    uvl = imagePointsL[0];
    uvr = imagePointsR[0];

    return std::isfinite(uvl.x) && std::isfinite(uvl.y) && std::isfinite(uvr.x) && std::isfinite(uvr.y);
}



// cv::Point3f CamCalib::triangulatePoint(const cv::Point2f &point0, const cv::Point2f &point1)
// {
//     Eigen::Vector3d unpts0, unpts1;
//     dynamic_cast<camodocal::PinholeCamera*>(m_camera[0].get())->liftProjective(Eigen::Vector2d(point0.x, point0.y), unpts0);
//     dynamic_cast<camodocal::PinholeCamera*>(m_camera[1].get())->liftProjective(Eigen::Vector2d(point1.x, point1.y), unpts1);

//     // Eigen::Matrix<double, 4, 4> pp1;
//     // pp1 << Poses[1](0,0), Poses[1](0,1), Poses[1](0,2), Poses[1](0,3),
//     //        Poses[1](1,0), Poses[1](1,1), Poses[1](1,2), Poses[1](1,3),
//     //        Poses[1](2,0), Poses[1](2,1), Poses[1](2,2), Poses[1](2,3),
//     //        0, 0, 0, 1;
//     // Eigen::Matrix4d pp1_inv = pp1.inverse();

//     Eigen::Matrix4d design_matrix = Eigen::Matrix4d::Zero();
//     design_matrix.row(0) = unpts0[0] * Poses[0].row(2) - Poses[0].row(0);
//     design_matrix.row(1) = unpts0[1] * Poses[0].row(2) - Poses[0].row(1);
//     design_matrix.row(2) = unpts1[0] * Poses[1].row(2) - Poses[1].row(0);
//     design_matrix.row(3) = unpts1[1] * Poses[1].row(2) - Poses[1].row(1);
//     Eigen::Vector4d triangulated_point;
//     triangulated_point =
//               design_matrix.jacobiSvd(Eigen::ComputeFullV).matrixV().rightCols<1>();
//     cv::Point3f point_3d = cv::Point3f(triangulated_point(0) / triangulated_point(3),
//                                  triangulated_point(1) / triangulated_point(3),
//                                  triangulated_point(2) / triangulated_point(3));
//     return point_3d;
// }

// std::vector<cv::Point2f> CamCalib::reproject3d(const cv::Point3f& pt)
// {
//     Eigen::Vector4d pt_world(pt.x, pt.y, pt.z, 1.0);
//     std::vector<cv::Point2f> result;
//     for (int i = 0; i < 2; ++i) {
//         Eigen::Vector3d pt_cam = Poses[i] * pt_world;
//         Eigen::Vector3d pt_cam_3d(pt_cam(0), pt_cam(1), pt_cam(2));
        
//         Eigen::Vector2d pt_2d;
//         auto cam = dynamic_cast<camodocal::PinholeCamera*>(m_camera[i].get());
//         cam->spaceToPlane(pt_cam_3d, pt_2d);
        
//         result.emplace_back(pt_2d.x(), pt_2d.y());
//     }
//     return result;
// }

void CamCalib::readIntrinsicParameter(const std::vector<std::string> &calib_file, const std::vector<Eigen::Matrix3d> &RIC, const std::vector<Eigen::Vector3d> &TIC)
{
    m_camera.clear();
    ric.clear();
    tic.clear();
    Ks.clear();
    Ds.clear();
    Poses.clear();
    for (size_t i = 0; i < calib_file.size(); i++)
    {
        ROS_INFO("calilb_file = %s", calib_file[i].c_str());
        camodocal::CameraPtr camera = camodocal::CameraFactory::instance()->generateCameraFromYamlFile(calib_file[i]);
        camodocal::PinholeCamera* pinhole = 
        dynamic_cast<camodocal::PinholeCamera*>(camera.get());
        const camodocal::PinholeCamera::Parameters& params1 = pinhole->getParameters();
        // ROS_INFO("cam[%d] fx=%f, fy=%f, cx=%f, cy=%f", i, params1.fx(), params1.fy(), params1.cx(), params1.cy());
        m_camera.push_back(camera);
        ric.push_back(RIC[i]);
        tic.push_back(TIC[i]);

        Eigen::Matrix3d matK;
        matK << params1.fx(), 0, params1.cx(),
                       0, params1.fy(), params1.cy(),
                       0, 0, 1;
        Ks.push_back(matK);
        Eigen::Vector4d matD;
        matD << params1.k1(), params1.k2(), params1.p1(), params1.p2();    
        Ds.push_back(matD);
        // Eigen::Matrix<double, 3, 4> pp;
        // pp << ric[icam](0,0), ric[icam](0,1), ric[icam](0,2), tic[icam](0),
        //             ric[icam](1,0), ric[icam](1,1), ric[icam](1,2), tic[icam](1),
        //             ric[icam](2,0), ric[icam](2,1), ric[icam](2,2), tic[icam](2);
        // Poses.push_back(pp);


        Eigen::Matrix4d T_imu_cam = Eigen::Matrix4d::Identity();
        T_imu_cam << ric[i](0,0), ric[i](0,1), ric[i](0,2), tic[i](0),
                    ric[i](1,0), ric[i](1,1), ric[i](1,2), tic[i](1),
                    ric[i](2,0), ric[i](2,1), ric[i](2,2), tic[i](2),
                    0, 0, 0, 1;
        // 存 3x4 的 [R|t]
        Poses.push_back(T_imu_cam.block<3, 4>(0, 0));
        // ROS_INFO("pose[-1] RT = %f %f %f %f %f %f %f %f %f %f %f %f", icam, 
        //     T_imu_cam(0,0), T_imu_cam(0,1), T_imu_cam(0,2), T_imu_cam(0,3), 
        //     T_imu_cam(1,0), T_imu_cam(1,1), T_imu_cam(1,2), T_imu_cam(1,3), 
        //     T_imu_cam(2,0), T_imu_cam(2,1), T_imu_cam(2,2), T_imu_cam(2,3) );
    }

    if (m_camera.size()!=2) {
        ROS_WARN("camera number is not 2.");
        return;
    }


    m1 = (cv::Mat_<float>(3, 3) <<
        991.7910260687049, 0.0, 613.9130839838093,
        0.0, 991.9419004860887, 435.39232502909186,
        0.0, 0.0, 1.0);

    // camera1 distortion d1 (5x1)
    d1 = (cv::Mat_<float>(5, 1) <<
        -0.07799370639100496,
        0.25173465243992804,
        -9.184759790271654e-05,
        0.00025258481983479324,
        -0.2726631483799577);

    // camera2 intrinsics m2 (3x3)
    m2 = (cv::Mat_<float>(3, 3) <<
        988.4717953032936, 0.0, 590.9793618083769,
        0.0, 987.8477706696484, 369.7344647619148,
        0.0, 0.0, 1.0);

    // camera2 distortion d2 (5x1)
    d2 = (cv::Mat_<float>(5, 1) <<
        -0.0722834146582847,
        0.19512061402752384,
        -0.00039152426404995554,
        -0.0009264456685067678,
        -0.17405238973806547);

    // rotation matrix R (3x3)
    matr = (cv::Mat_<float>(3, 3) <<
        0.9999992586338078, -9.144070994172981e-05, 0.0012142365631524236,
        9.568336789973387e-05, 0.9999938896498164, -0.0034944967769385423,
        -0.001213909604475647, 0.003494610368480562, 0.9999931570375094);

    // translation vector T (3x1)
    matt = (cv::Mat_<float>(3, 1) <<
        0.059965455297203124,
        -0.00019674478221781783,
        -0.0010502321873644886);

}


