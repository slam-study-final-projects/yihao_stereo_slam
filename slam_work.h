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

class SlamWork {
	public:
	 SlamWork();
	 ~SlamWork() = default;
	 // bool LoadCalibParams(const std::string& path);
	 bool ComputePose();

	 cv::Mat K_;
   double fx, fy, cx, cy, d;
};
