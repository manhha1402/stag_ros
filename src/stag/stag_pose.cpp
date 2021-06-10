#include "stag/stag_pose.h"

namespace stag {
StagPose::StagPose(double marker_size, int marker_id,
                   int library_hd,
                   int error_correction ,bool in_keep_logs ):
  marker_size_(marker_size),marker_id_(marker_id)
{
  stag_ = std::make_shared<stag::Stag>(library_hd,error_correction,in_keep_logs);

}
StagPose::~StagPose()
{}
void StagPose::setCameraMatrix(const Eigen::Matrix3d& intrinsic_matrix, const cv::Mat& dist_coeffs)
{
  camera_matrix_ = (cv::Mat_<double>(3, 3) << intrinsic_matrix(0, 0), 0.0, intrinsic_matrix(0, 2),
                    0.0, intrinsic_matrix(1, 1), intrinsic_matrix(1, 2), 0.0, 0.0, 1.0);
  dist_coeffs_ = dist_coeffs;
  set_camera_matrix_ = true;
}

bool StagPose::getMarkerPose(const cv::Mat& input_image,Eigen::Matrix4d& marker2cam)
{
  stag_->markers.clear();
  stag_->detectMarkers(input_image);
  auto markers = stag_->getMarkerList();
  if(markers.size() == 0)
    return false;
  std::sort(
        markers.begin(),
        markers.end(),
        [](const stag::Marker& a, const stag::Marker& b){
    return a.id < b.id;
  }
  );
  std::map<int, std::vector<cv::Point2d>> unrefined_corners;

  for (auto& marker : markers) {
    unrefined_corners[marker.id] = marker.corners;
  }

}

} // namespace stag
