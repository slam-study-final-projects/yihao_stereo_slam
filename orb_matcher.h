#include <Eigen/Core>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <vector>

using namespace std;
using namespace Eigen;

typedef vector<Vector3d, Eigen::aligned_allocator<Vector3d>> VecVector3d;
typedef vector<Vector2d, Eigen::aligned_allocator<Vector3d>> VecVector2d;
typedef Matrix<double, 6, 1> Vector6d;
typedef vector<bool> DescType;

const double pi = 3.1415926;

class OrbMatcher {
public:
  OrbMatcher();
  ~OrbMatcher() = default;
  std::vector<cv::DMatch> Run();
  void SetParams(double fx, double fy, double cx, double cy, double d);
  void SetGrayImages(const cv::Mat &first, const cv::Mat &second);
  void SetDepthImages(const cv::Mat &first, const cv::Mat &second);
  std::vector<cv::Point3f> depth_points1_, depth_points2_;

private:
  float GetPointDepth(const cv::Mat &depth_image, const cv::Point2f &point);
  void ComputeAngle(const cv::Mat &image, std::vector<cv::KeyPoint> &keypoints);
  void ComputeORBDesc(const cv::Mat &image, vector<cv::KeyPoint> &keypoints,
                      vector<DescType> &desc);
  void BfMatch(const std::vector<DescType> &desc1,
               const std::vector<DescType> &desc2,
               std::vector<cv::DMatch> &matches);

  cv::Mat first_gray_img_, second_gray_img_, first_depth_img_,
      second_depth_img_;
  double fx_, fy_, cx_, cy_, d_;
};
