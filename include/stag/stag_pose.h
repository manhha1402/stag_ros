#ifndef STAGPOSE_H
#define STAGPOSE_H
#include <Eigen/Eigen>
#include "stag/Stag.h"
#include "stag_ros/structures.hpp"
namespace stag {

class StagPose {

public:
  StagPose(double marker_size, int marker_id,
           int library_hd = 15,
           int error_correction = 7, bool in_keep_logs = false );
  virtual ~StagPose();
  void setCameraMatrix(const Eigen::Matrix3d& intrinsic_matrix, const cv::Mat& dist_coeffs = cv::Mat::zeros(1, 5, CV_64F));
  bool getMarkerPose(const cv::Mat& input_image,Eigen::Matrix4d& marker2cam);

private:
  std::shared_ptr<stag::Stag> stag_;
  int marker_id_;
  double marker_size_;
  cv::Mat camera_matrix_;
  cv::Mat dist_coeffs_;
  bool set_camera_matrix_;

  // Tag and bundle info
  std::vector<stag_ros::Bundle> bundles;
  std::vector<stag_ros::Tag> tags_;
};
} // namespace stag
#endif //STAGPOSE_H
