#include "icp_translator.h"

void IcpTranslator::SetPointsAndMatches(
    const vector<cv::Point3f> &depth_points1,
    const vector<cv::Point3f> &depth_points2,
    const vector<cv::DMatch> &matches) {
  depth_points1_ = depth_points1;
  depth_points2_ = depth_points2;
  matches_ = matches;
}

void IcpTranslator::Run() {

  vector<Eigen::Vector3d> pi_g, pi_e;

  for (int k = 0; k < matches_.size(); ++k) {
    pi_g.emplace_back(depth_points1_[matches_[k].queryIdx].x,
                      depth_points1_[matches_[k].queryIdx].y,
                      depth_points1_[matches_[k].queryIdx].z);
    pi_e.emplace_back(depth_points2_[matches_[k].trainIdx].x,
                      depth_points2_[matches_[k].trainIdx].y,
                      depth_points2_[matches_[k].trainIdx].z);
  }

  // Compute the mass center of it.
  Eigen::Vector3d p_gc(0, 0, 0), p_ec(0, 0, 0);
  for (auto p : pi_g) {
    p_gc += p / static_cast<double>(pi_g.size());
  }

  for (auto p : pi_e) {
    p_ec += p / static_cast<double>(pi_e.size());
  }

  vector<Eigen::Vector3d> qi_g, qi_e;
  for (auto p : pi_g) {
    qi_g.emplace_back(p - p_gc);
  }

  for (auto p : pi_e) {
    qi_e.emplace_back(p - p_ec);
  }

  Eigen::Matrix3d W;
  W = Eigen::Matrix3d::Zero(3, 3);
  for (int i = 0; i < qi_g.size(); ++i) {
    W += qi_g[i] * qi_e[i].transpose();
  }

  // SVD
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(
      W,
      Eigen::ComputeFullV | Eigen::ComputeFullU); // ComputeFull vs ComputeThin
  Eigen::MatrixXd U = svd.matrixU();
  Eigen::MatrixXd V = svd.matrixV();

  R_ = U * V.transpose(); // 计算得到R
  t_ = p_gc - R_ * p_ec;
}
