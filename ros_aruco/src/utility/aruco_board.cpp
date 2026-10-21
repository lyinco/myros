
#include <ros/ros.h>

#include <opencv2/core/types.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "aruco_board.h"

bool ArucoBoard::init() {

    board_ = cv::aruco::CharucoBoard::create(board_cols_, board_rows_, board_square_length_, board_marker_length_, cv::aruco::getPredefinedDictionary(dict_id_));
    if (board_ == nullptr) {
        ROS_ERROR("[ArucoBoard] Failed to create CharucoBoard.");   
        return false;
    }

    dict_ = cv::aruco::getPredefinedDictionary(dict_id_);
    if (dict_ == nullptr) {
        ROS_ERROR("[ArucoBoard] Failed to create aruco dictionary.");   
        return false;
    }

    const std::vector<std::vector<cv::Point3f>>& objPoints = board_->objPoints;
    const std::vector<int>& ids = board_->ids;
    board_corners3d_.clear();
    for (size_t i = 0; i < ids.size(); ++i)
    {
        board_corners3d_[ids[i]] = objPoints[i];
    }
    return true;
}

std::map<int, std::vector<cv::Point3f>> ArucoBoard::getBoardCorners(){
    return board_corners3d_;
}

std::map<int, std::vector<cv::Point2f>> ArucoBoard::detectMarkerCorners(
        const cv::Mat& img)
{
    std::map<int, std::vector<cv::Point2f>> arucoCorners;

    if (img.empty() || !dict_) {
        return arucoCorners;
    }

    // 4.2 里 DetectorParameters 用 create() 创建
    cv::Ptr<cv::aruco::DetectorParameters> params =
        cv::aruco::DetectorParameters::create();

    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
    std::vector<std::vector<cv::Point2f>> rejected;

    // 4.2 的函数签名：detectMarkers(image, dictionary, corners, ids, params, rejected)
    cv::aruco::detectMarkers(img, dict_, corners, ids, params, rejected);
    for (size_t i = 0; i < ids.size(); ++i) {
        arucoCorners[ids[i]] = corners[i];
    }
    return arucoCorners;
}

cv::Mat ArucoBoard::drawMarkers(const cv::Mat& img, const std::map<int, std::vector<cv::Point2f>>& corners, const cv::Scalar& color, int radius, bool with_id)
{
    cv::Mat img_copy;
    if (img.channels() == 1) {
        // 灰度 → BGR
        cv::cvtColor(img, img_copy, cv::COLOR_GRAY2BGR);
    } else {
        img_copy = img.clone();
    }
    for (const auto& kv : corners) {
        const std::vector<cv::Point2f>& pts = kv.second;
        for (const cv::Point2f& pt : pts) {
            cv::circle(img_copy, pt, radius, color,  -1);
        }
        cv::line(img_copy, pts[0], pts[1], color);
        cv::line(img_copy, pts[1], pts[2], color);
        cv::line(img_copy, pts[2], pts[3], color);
        cv::line(img_copy, pts[3], pts[0], color);
        
        if (with_id){
            cv::putText(img_copy, std::to_string(kv.first), pts[0] - cv::Point2f(0, 10),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, color, 2);
        }
        
    }
    return img_copy;
}
