#pragma once

#include <camodocal/camera_models/CameraFactory.h>
#include <camodocal/camera_models/Camera.h>
#include <Eigen/Core>
#include <opencv2/core.hpp>
#include <vector>


class CamCalib{
public:
    CamCalib() = default;
    ~CamCalib() = default;

    void setConfigFile(const std::string &conf_file);
    // cv::Point3f triangulatePoint(const cv::Point2f& pt1, const cv::Point2f& pt2);
    bool triangulatePoint(const cv::Point2f& uvl, const cv::Point2f& uvr, cv::Point3f& point3D);
    // std::vector<cv::Point2f> reproject3d(const cv::Point3f& pt);
    bool projectPointToStereoImages( const cv::Point3f& pt3d, cv::Point2f& uvl, cv::Point2f& uvr);


private:
    std::string conf_file_;
    
    std::vector<camodocal::CameraPtr> m_camera;
    std::vector<Eigen::Matrix3d> ric;
    std::vector<Eigen::Vector3d> tic;

    std::vector<Eigen::Matrix3d> Ks;
    std::vector<Eigen::Vector4d> Ds;
    std::vector<Eigen::Matrix<double, 3, 4>> Poses;

    cv::Mat_<float> m1, m2, d1, d2, matr, matt;

    void readIntrinsicParameter(const std::vector<std::string>& calib_file, 
        const std::vector<Eigen::Matrix3d> &RIC, const std::vector<Eigen::Vector3d> &TIC);

};