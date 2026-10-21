
#pragma once

#include <opencv2/core.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <vector>
#include <map>


class ArucoBoard {
public:
    ArucoBoard() = default;
    ~ArucoBoard() = default;

    // bool init(int dict_id = cv::aruco::DICT_5X5_1000,
    //             int board_rows = 9, int board_cols = 12,
    //             float board_square_length = 0.015f, float board_marker_length = 0.01125f);
    bool init();
    
    std::map<int, std::vector<cv::Point2f>> detectMarkerCorners(const cv::Mat& img);
    cv::Mat drawMarkers(const cv::Mat& img, const std::map<int, std::vector<cv::Point2f>>& corners, const cv::Scalar& color=cv::Scalar(0, 255, 0), int radius = 5, bool with_id = true);
    std::map<int, std::vector<cv::Point3f>> getBoardCorners();

private:

    int dict_id_ = cv::aruco::DICT_5X5_1000;  ///< ArUco 字典 ID
    int board_rows_ = 9;                            ///< ArUco 棋盘行数
    int board_cols_ = 12;                            ///< ArUco 棋盘列数
    float board_square_length_ = 0.015f;             ///< ArUco �棋盘方格边长（单位：米）
    float board_marker_length_ = 0.01125f;             ///< ArUco 棋盘 marker 边长（单位：米
    cv::Ptr<cv::aruco::Dictionary> dict_;       ///< ArUco 字典
    cv::Ptr<cv::aruco::CharucoBoard> board_; ///< ArUco 棋盘对象
    std::map<int, std::vector<cv::Point3f>> board_corners3d_; ///< ArUco 棋盘角点在世界坐标系下的 3D 坐标

};
