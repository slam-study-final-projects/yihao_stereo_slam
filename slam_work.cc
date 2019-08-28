#include "slam_work.h"

using namespace std;

SlamWork::SlamWork()
    : pose_number_(0), input_path_(""), depth_generator_(), orb_matcher_(),
      icp_translator_() {}

void SlamWork::SetParams(double fx, double fy, double cx, double cy, double d) {
  depth_generator_.SetParams(fx, fy, cx, cy, d);
  orb_matcher_.SetParams(fx, fy, cx, cy, d);
  return;
}

void SlamWork::ComputePose() {

  for (int i = 0; i < pose_number_; ++i) {
    string left_path = input_path_ + "//image_0//00000" + to_string(i) + ".png";
    string right_path =
        input_path_ + "//image_1//00000" + to_string(i) + ".png";

    string left_path2 =
        input_path_ + "//image_0//00000" + to_string(i + 1) + ".png";
    string right_path2 =
        input_path_ + "//image_1//00000" + to_string(i + 1) + ".png";

    cv::Mat left_first = cv::imread(left_path, 0);
    cv::Mat right_first = cv::imread(right_path, 0);
    cv::Mat left_second = cv::imread(left_path2, 0);
    cv::Mat right_second = cv::imread(right_path2, 0);

    depth_generator_.SetStereoImages(left_first, right_first);
    cv::Mat depth_first = depth_generator_.Run();
    depth_generator_.SetStereoImages(left_second, right_second);
    cv::Mat depth_second = depth_generator_.Run();

    orb_matcher_.SetGrayImages(left_first, left_second);
    orb_matcher_.SetDepthImages(depth_first, depth_second);

    std::vector<cv::DMatch> matches = orb_matcher_.Run();
    icp_translator_.SetPointsAndMatches(orb_matcher_.depth_points1_,
                                        orb_matcher_.depth_points2_, matches);
    icp_translator_.Run();

    std::cout << "R = " << icp_translator_.R_ << std::endl;
    std::cout << "t = " << icp_translator_.t_ << std::endl;
    R_world_ = R_world_ * icp_translator_.R_;
    t_world_ = icp_translator_.R_ * t_world_ + icp_translator_.t_;
  }
  std::cout << "R_world = " << R_world_ << std::endl;
  std::cout << "t_world = " << t_world_ << std::endl;

  return;
}
