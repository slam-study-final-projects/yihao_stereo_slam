#include "depth_generator.h"

DepthGenerator::DepthGenerator() : fx_(0), fy_(0), cx_(0), cy_(0), d_(0) {}

void DepthGenerator::SetParams(double fx, double fy, double cx, double cy,
                               double d) {
  fx_ = fx, fy_ = fy, cx_ = cx, cy_ = cy, d_ = d;
  return;
}

void DepthGenerator::SetStereoImages(const cv::Mat &left,
                                     const cv::Mat &right) {
  left_img_ = left.clone();
  right_img_ = right.clone();
}

cv::Mat DepthGenerator::Run() {

  int mindisparity = 0;
  int ndisparities = 96;  // 64;
  int SADWindowSize = 11; // 11;

  // SGBM
  cv::Ptr<cv::StereoSGBM> sgbm =
      cv::StereoSGBM::create(mindisparity, ndisparities, SADWindowSize);
  int P1 = 8 * left_img_.channels() * SADWindowSize * SADWindowSize;
  int P2 = 32 * left_img_.channels() * SADWindowSize * SADWindowSize;
  sgbm->setP1(P1);
  sgbm->setP2(P2);
  sgbm->setPreFilterCap(15);
  sgbm->setUniquenessRatio(10);
  sgbm->setSpeckleRange(2);
  sgbm->setSpeckleWindowSize(100);
  sgbm->setDisp12MaxDiff(1);
  // sgbm->setMode(cv::StereoSGBM::MODE_HH);
  sgbm->compute(left_img_, right_img_, disp_img_);
  disp_img_.convertTo(disp_img_, CV_32F, 1.0 / 16);

  depth_img_ = cv::Mat(disp_img_.rows, disp_img_.cols, CV_32FC1);

  for (int v = 0; v < disp_img_.rows; v++)
    for (int u = 0; u < disp_img_.cols; u++) {

      double z = d_ / (disp_img_.at<float>(v, u) /
                       fx_); // 先计算z的值 // switch uchar to float
      if (z > 0 &&
          z < 15) { // 判断z是否有效，可提升计算效率 and accuracy 0 ~ 15 meters.
        depth_img_.at<float>(v, u) = z; //
      } else {
        depth_img_.at<float>(v, u) = 0;
      }
    }
  return depth_img_;
}
