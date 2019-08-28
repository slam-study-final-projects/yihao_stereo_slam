#include "sophus/se3.h"
#include <Eigen/Core>
#include <Eigen/Dense>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <pangolin/pangolin.h>
#include <string>
#include <unistd.h>
#include <vector>

#include "depth_generator.h"
#include "icp_translator.h"
#include "orb_matcher.h"

using namespace std;

class SlamWork {
public:
  SlamWork();
  ~SlamWork() = default;
  void SetParams(double fx, double fy, double cx, double cy, double d);
  void ComputePose();

  string input_path_;
  int pose_number_;

  DepthGenerator depth_generator_;
  OrbMatcher orb_matcher_;
  IcpTranslator icp_translator_;

  Eigen::Matrix3d R_world_;
  Eigen::Vector3d t_world_;
};
