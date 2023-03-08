#include "sg_filter.h"

namespace smartlabel {

bool SGFilter::setInputCloud(const LaneCloudPtr &source) {
  if (source == nullptr || source->size() < 2) {
    return false;
  }
  src_ = source;
  return true;
}

void SGFilter::filter(const LaneCloudPtr &result) {
  result->clear();
  pcl::KdTreeFLANN<Point_Lane> kdtree;
  kdtree.setInputCloud(src_);
  std::vector<int> indices;
  std::vector<float> distance_sq;
  LaneCloudPtr tmp_cloud(new LaneCloud);
  for (size_t i = 0; i < src_->size(); ++i) {
    const auto &p = src_->at(i);
    kdtree.radiusSearch(p, radius_, indices, distance_sq);
    tmp_cloud->clear();
    for (size_t j = 0; j < indices.size(); ++j) {
      const auto &p2 = src_->at(indices[j]);
      tmp_cloud->push_back(p2);
    }
    pcl::ModelCoefficients::Ptr coeff(new pcl::ModelCoefficients);
    if (fit_line_3d(tmp_cloud, coeff)) {
      Point_Lane proj_point;
      project_point(p, coeff, proj_point);
      result->push_back(proj_point);
    }
  }
}

}  // namespace smartlabel