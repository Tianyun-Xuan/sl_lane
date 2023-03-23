#ifndef LANE_H
#define LANE_H
// clang-format off
#include "point_type.h"
// clang-format on
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>

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
#include "loss.h"
#include "sg_filter.h"

// namespace bg = boost::geometry;
namespace fs = std::experimental::filesystem;

namespace smartlabel {

enum LaneType {
  UNKNOWN = 0,
  STRAIGHT = 1,
  CURVE = 2,
};

class LaneSegment {
 public:
  LaneSegment(const LaneCloudPtr& cloud);
  ~LaneSegment() = default;

  LaneCloudPtr cloud;
  LaneCloudPtr cloud_smoothed;
  Point_Lane start_point;
  Point_Lane end_point;
  Polynomia parameters;

  LaneCloudPtr print(const double step);

  bool in_interval(const double x) const {
    return (x >= start_point.x && x <= end_point.x);
  }

  double y(const double x) const { return parameters.y(x); }

  friend LaneSegment operator+=(LaneSegment& lhs, const LaneSegment& rhs);
};

struct LaneCluster {
  int lane_id;
  LaneCloudPtr cloud;
  LaneCloudPtr cloud_smoothed;
  Point_Lane start_point;
  Point_Lane end_point;
  Eigen::Vector4f start_point_tangent;
  Eigen::Vector4f end_point_tangent;
  Eigen::Vector4f center_point;
  Hoff hoff_params;
  LaneType lane_type;
  float tmin;
  float tmax;
  float deplacement;
  float ratio;
  double timestamp;
  LaneCloudPtr print(const double step);
};

struct FitParameters {
  DBscan_Parameters dbscan_parameters;

  // semantic filter parameters
  uint32_t value_motion_type = 2;
  uint32_t value_lane_type = 10;
  uint32_t value_is_laneline = 1;
  uint32_t value_ground_flag = 1;
  uint32_t value_inside_curb = 1;
  float value_original_distance = 100.f;
  double x_step = 2;
  double y_step = 1;
};

class LaneFitting {
 private:
  FitParameters params_;
  double m_x_min_;
  double m_y_min_;
  double m_x_max_;
  double m_y_max_;
  std::vector<std::vector<double>> m_height_map_;
  std::vector<std::vector<double>> m_intensity_map_;

 public:
  LaneFitting(const FitParameters& params)
      : params_(params),
        m_x_min_(std::numeric_limits<double>::max()),
        m_y_min_(std::numeric_limits<double>::max()),
        m_x_max_(-std::numeric_limits<double>::max()),
        m_y_max_(-std::numeric_limits<double>::max()){};
  ~LaneFitting() = default;

  bool is_lane(const Point_Auto_Label& point);
  bool is_ground(const Point_Auto_Label& point);
  bool inside_curb(const Point_Auto_Label& point);



  void label_filter(const std::vector<SLCloudPtr>& source,
                    std::vector<SLCloudPtr>& inside_curb_clouds,
                    const LaneCloudPtr& lane_cloud,
                    const LaneCloudPtr& ground_cloud);
  void initialize(const std::vector<LaneCloudPtr>& source,
                  std::vector<LaneCluster>& result);
  LaneCluster initialize(const LaneCloudPtr& source);

  // void endpoint_match(const std::vector<LaneCluster>& clusters,
  // std::vector<size_t>& cluster_id);

  bool find_referrence_line(const std::vector<LaneCluster>& clusters,
                            Hoff& ref_params, Eigen::Vector4f& ref_center);
  void classify(std::vector<LaneCluster>& lane_clusters, const Hoff& ref_params,
                const Eigen::Vector4f& ref_center);

  float distance(const Point_Lane& point) const;

  void extract_candidate(const std::vector<SLCloudPtr>& source,
                         const std::vector<LaneCluster>& clusters,
                         std::vector<SLCloudPtr>& candidates);

  void extract_candidate(const std::vector<SLCloudPtr>& source,
                         const std::vector<LaneSegment>& segments,
                         std::vector<SLCloudPtr>& candidates);

  void ground_map(const LaneCloudPtr& ground_cloud);

  // SLCloudPtr select_candidate(const SLCloudPtr& candidates);

  SLCloudPtr adapte_intesnity_filter(const SLCloudPtr& candidates,
                                     const double radius,
                                     const double threshold);
  SLCloudPtr neiborhood_filter(const SLCloudPtr& candidates,
                               const double radius, const size_t threshold);

  void fit(const std::vector<SLCloudPtr>& source,
           std::vector<SLCloudPtr>& result);
};

}  // namespace smartlabel

#endif  // SMARTLABEL_LANE_FITTING_H_