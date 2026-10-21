
#include <opencv2/opencv.hpp>
#include <vector>
#include <cmath>

bool triangulatePoint(
    const cv::Mat& m1,
    const cv::Mat& d1,
    const cv::Mat& m2,
    const cv::Mat& d2,
    const cv::Mat& R,
    const cv::Mat& T,
    const cv::Point2d& uvl,
    const cv::Point2d& uvr,
    cv::Point3d& point3D)
{
    if (m1.empty() || m2.empty() ||
        d1.empty() || d2.empty() ||
        R.empty() || T.empty())
    {
        return false;
    }

    // 统一使用 double 精度
    cv::Mat K1, K2, D1, D2, R64, T64;

    m1.convertTo(K1, CV_64F);
    m2.convertTo(K2, CV_64F);
    d1.convertTo(D1, CV_64F);
    d2.convertTo(D2, CV_64F);
    R.convertTo(R64, CV_64F);
    T.convertTo(T64, CV_64F);

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
    std::vector<cv::Point2d> ptsL = {uvl};
    std::vector<cv::Point2d> ptsR = {uvr};

    std::vector<cv::Point2d> normL, normR;

    cv::undistortPoints(ptsL, normL, K1, D1);
    cv::undistortPoints(ptsR, normR, K2, D2);

    // 2. 构建归一化坐标系下的投影矩阵
    // 左相机：P1 = [I | 0]
    cv::Mat P1 = cv::Mat::zeros(3, 4, CV_64F);
    cv::Mat I = cv::Mat::eye(3, 3, CV_64F);
    I.copyTo(P1(cv::Rect(0, 0, 3, 3)));

    // 右相机：P2 = [R | T]
    cv::Mat P2;
    cv::hconcat(R64, T64, P2);

    // 3. 三角测量
    cv::Mat points4D;
    cv::triangulatePoints(P1, P2, normL, normR, points4D);

    // 4. 齐次坐标转为普通三维坐标
    double w = points4D.at<double>(3, 0);

    if (!std::isfinite(w) || std::abs(w) < 1e-12)
        return false;

    double X = points4D.at<double>(0, 0) / w;
    double Y = points4D.at<double>(1, 0) / w;
    double Z = points4D.at<double>(2, 0) / w;

    if (!std::isfinite(X) ||
        !std::isfinite(Y) ||
        !std::isfinite(Z))
    {
        return false;
    }

    // 5. 检查点是否位于左右相机的前方
    cv::Mat Xl = (cv::Mat_<double>(3, 1) << X, Y, Z);
    cv::Mat Xr = R64 * Xl + T64;

    if (Z <= 0.0 || Xr.at<double>(2, 0) <= 0.0)
        return false;

    point3D = cv::Point3d(X, Y, Z);
    return true;
}


#include <opencv2/opencv.hpp>
#include <vector>
#include <cmath>

// 将左相机坐标系下的 3D 点投影到左右图像
//
// 参数：
//   pt3d : 相对于左相机坐标系的 3D 点
//   m1, d1 : 左相机内参、畸变参数
//   m2, d2 : 右相机内参、畸变参数
//   R, T : 左相机坐标系到右相机坐标系的变换
//
// 输出：
//   uvl : 左图像中的像素坐标
//   uvr : 右图像中的像素坐标
//
// 返回：
//   true  : 两个相机中该点均位于相机前方，投影有效
//   false : 输入无效或点不在任一相机前方

bool projectPointToStereoImages(
    const cv::Point3d& pt3d,
    const cv::Mat& m1,
    const cv::Mat& d1,
    const cv::Mat& m2,
    const cv::Mat& d2,
    const cv::Mat& R,
    const cv::Mat& T,
    cv::Point2d& uvl,
    cv::Point2d& uvr)
{
    if (m1.empty() || m2.empty() ||
        d1.empty() || d2.empty() ||
        R.empty() || T.empty())
    {
        return false;
    }

    // 统一使用 double 精度
    cv::Mat K1, K2, D1, D2, R64, T64;

    m1.convertTo(K1, CV_64F);
    m2.convertTo(K2, CV_64F);
    d1.convertTo(D1, CV_64F);
    d2.convertTo(D2, CV_64F);
    R.convertTo(R64, CV_64F);
    T.convertTo(T64, CV_64F);

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
    cv::Mat pointL = (cv::Mat_<double>(3, 1)
        << pt3d.x, pt3d.y, pt3d.z);

    cv::Mat pointR = R64 * pointL + T64;

    double Xr = pointR.at<double>(0, 0);
    double Yr = pointR.at<double>(1, 0);
    double Zr = pointR.at<double>(2, 0);

    if (!std::isfinite(Zr) || Zr <= 0.0)
        return false;

    // 左相机投影：
    // pt3d 已在左相机坐标系，外参使用单位旋转和零平移
    std::vector<cv::Point3d> pointsL = {pt3d};
    std::vector<cv::Point2d> imagePointsL;

    cv::Mat rvecZero = cv::Mat::zeros(3, 1, CV_64F);
    cv::Mat tvecZero = cv::Mat::zeros(3, 1, CV_64F);

    cv::projectPoints(
        pointsL, rvecZero, tvecZero,
        K1, D1, imagePointsL);

    // 右相机投影：
    // 使用 R、T 将左相机坐标系的点变换到右相机
    cv::Mat rvec;
    cv::Rodrigues(R64, rvec);

    std::vector<cv::Point3d> pointsR = {pt3d};
    std::vector<cv::Point2d> imagePointsR;

    cv::projectPoints(
        pointsR, rvec, T64,
        K2, D2, imagePointsR);

    uvl = imagePointsL[0];
    uvr = imagePointsR[0];

    return std::isfinite(uvl.x) &&
           std::isfinite(uvl.y) &&
           std::isfinite(uvr.x) &&
           std::isfinite(uvr.y);
}




int main()
{
    cv::Point2d uvl(885.4684, 507.1514); // 3d=[0.09241427 0.02445695 0.33596544]
    cv::Point2d uvr(1037.6409, 437.26007);
    cv::Point3f exp_pt3d(0.09241427, 0.02445695, 0.33596544);
    cv::Mat m1 = (cv::Mat_<double>(3, 3) <<
        991.7910260687049, 0.0, 613.9130839838093,
        0.0, 991.9419004860887, 435.39232502909186,
        0.0, 0.0, 1.0);

    // camera1 distortion d1 (5x1)
    cv::Mat d1 = (cv::Mat_<double>(5, 1) <<
        -0.07799370639100496,
        0.25173465243992804,
        -9.184759790271654e-05,
        0.00025258481983479324,
        -0.2726631483799577);

    // camera2 intrinsics m2 (3x3)
    cv::Mat m2 = (cv::Mat_<double>(3, 3) <<
        988.4717953032936, 0.0, 590.9793618083769,
        0.0, 987.8477706696484, 369.7344647619148,
        0.0, 0.0, 1.0);

    // camera2 distortion d2 (5x1)
    cv::Mat d2 = (cv::Mat_<double>(5, 1) <<
        -0.0722834146582847,
        0.19512061402752384,
        -0.00039152426404995554,
        -0.0009264456685067678,
        -0.17405238973806547);

    // rotation matrix R (3x3)
    cv::Mat R = (cv::Mat_<double>(3, 3) <<
        0.9999992586338078, -9.144070994172981e-05, 0.0012142365631524236,
        9.568336789973387e-05, 0.9999938896498164, -0.0034944967769385423,
        -0.001213909604475647, 0.003494610368480562, 0.9999931570375094);

    // translation vector T (3x1)
    cv::Mat T = (cv::Mat_<double>(3, 1) <<
        0.059965455297203124,
        -0.00019674478221781783,
        -0.0010502321873644886);

    cv::Point3d point3D;

    if (triangulatePoint(m1, d1, m2, d2,
                        R, T, uvl, uvr, point3D))
    {
        std::cout << "X = " << point3D.x << std::endl;
        std::cout << "Y = " << point3D.y << std::endl;
        std::cout << "Z = " << point3D.z << std::endl;
    }
    else
    {
        std::cout << "Triangulation failed." << std::endl;
    }


    // 重投影
    cv::Point2d uvlp, uvrp;
    projectPointToStereoImages(point3D, m1, d1, m2, d2, R, T, uvlp, uvrp);
    std::cout << "reproject left uvx = " << uvlp.x << std::endl;
    std::cout << "reproject left uvy = " << uvlp.y << std::endl;
    std::cout << "reproject right uvx = " << uvrp.x << std::endl;
    std::cout << "reproject right uvy = " << uvrp.y << std::endl;

    return 0;
}
