#include <opencv2/opencv.hpp>
#include <vector>

class DepthGenerator {
public:
  DepthGenerator();
  ~DepthGenerator() = default;
  cv::Mat Run();
  void SetParams(double fx, double fy, double cx, double cy, double d);
  void SetStereoImages(const cv::Mat &left, const cv::Mat &right);

private:
  double fx_, fy_, cx_, cy_, d_;
  cv::Mat left_img_, right_img_, disp_img_, depth_img_;
  cv::Ptr<cv::StereoSGBM> sgbm_;
};
