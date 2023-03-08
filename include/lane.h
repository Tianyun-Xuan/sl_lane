#pragma once
// clang-format off
#include "point_type.h"
// clang-format on
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>

// #include <boost/geometry.hpp>
// #include <boost/geometry/geometries/geometries.hpp>
#include <experimental/filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <opencv/cv.hpp>
#include <ostream>
#include <string>
#include <unordered_map>
#include <vector>

#include "geometry_util.h"
#include "sg_filter.h"

// namespace bg = boost::geometry;
namespace fs = std::experimental::filesystem;

namespace smartlabel {

enum LaneType {
  UNKNOWN = 0,
  STRAIGHT = 1,
  CURVE = 2,
};

struct LaneSegment {
  Point_Lane start_point_;
  Point_Lane end_point_;
  Eigen::Vector4f& centroid_;
  Polynomia parameters_;
};

struct LaneCluster {
  int lane_id;
  LaneCloudPtr cloud;
  LaneCloudPtr cloud_smoothed;
  Point_Lane start_point;
  Point_Lane end_point;
  Eigen::Vector4f center_point;
  Hoff hoff_params;
  LaneType lane_type;
  float tmin;
  float tmax;
  float deplacement;
  float ratio;
};

struct FitParameters {
  DBscan_Parameters dbscan_parameters;

  // semantic filter parameters
  uint32_t value_motion_type = 2;
  uint32_t value_label_type = 10;
  float value_original_distance = 100.f;
};

class LaneFitting {
 private:
  FitParameters params_;

 public:
  LaneFitting(const FitParameters& params) : params_(params){};
  ~LaneFitting() = default;

  bool is_lane(const Point_Auto_Label& point);
  LaneCloudPtr raw_filter(const std::vector<SLCloudPtr>& source);
  void initialize(const std::vector<LaneCloudPtr>& source,
                  std::vector<LaneCluster>& result);
  LaneCluster initialize(const LaneCloudPtr& source);

  bool find_referrence_line(const std::vector<LaneCluster>& clusters,
                            Hoff& ref_params, Eigen::Vector4f& ref_center);
  void classify(std::vector<LaneCluster>& lane_clusters, const Hoff& ref_params,
                const Eigen::Vector4f& ref_center);

  float distance(const Point_Lane& point) const;

  void fit(const std::vector<SLCloudPtr>& source,
           std::vector<LaneSegment>& result);
};

}  // namespace smartlabel