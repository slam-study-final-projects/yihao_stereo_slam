#include "sophus/se3.h"
#include <Eigen/Core>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <vector>

using namespace std;
using namespace Eigen;

class IcpTranslator {
public:
  void SetPointsAndMatches(const vector<cv::Point3f> &depth_points1,
                           const vector<cv::Point3f> &depth_points2,
                           const vector<cv::DMatch> &matches);
  void Run();
  Eigen::MatrixXd R_;
  Eigen::Vector3d t_;

private:
  std::vector<cv::Point3f> depth_points1_, depth_points2_;
  std::vector<cv::DMatch> matches_;
};
