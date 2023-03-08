#include "lane.h"
#define DEBUG
namespace smartlabel {

bool LaneFitting::is_lane(const Point_Auto_Label& point) {
  return point.motion_type == this->params_.value_motion_type &&
         point.semantic_3d_label == this->params_.value_label_type &&
         original_dst(point) < this->params_.value_original_distance;
}

LaneCloudPtr LaneFitting::raw_filter(const std::vector<SLCloudPtr>& source) {
  LaneCloudPtr result(new LaneCloud);
  for (const auto& cloud : source) {
    for (const auto& point : cloud->points) {
      if (is_lane(point)) {
        Point_Lane lane_point;
        lane_point.x = point.x;
        lane_point.y = point.y;
        lane_point.z = point.z;
        lane_point.intensity = point.intensity;
        lane_point.t = 0;
        result->push_back(lane_point);
      }
    }
  }
  return result;
}

LaneCluster LaneFitting::initialize(const LaneCloudPtr& source) {
  Eigen::Vector4f center_point;
  Hoff hoff_coeff;
  FitLine1DByRegression(source, hoff_coeff, center_point);
  float t_min = std::numeric_limits<float>::max();
  float t_max = -(std::numeric_limits<float>::max)();
  for (auto& point : source->points) {
    point.t = (point.x - center_point[0]) * hoff_coeff.tx +
              (point.y - center_point[1]) * hoff_coeff.ty;
    t_max = std::max(point.t, t_max);
    t_min = std::min(point.t, t_min);
  }
  Point_Lane start_point;
  Point_Lane end_point;
  start_point.x = center_point[0] + t_min * hoff_coeff.tx;
  start_point.y = center_point[1] + t_min * hoff_coeff.ty;
  start_point.z = 0.f;
  end_point.x = center_point[0] + t_max * hoff_coeff.tx;
  end_point.y = center_point[1] + t_max * hoff_coeff.ty;
  start_point.z = 0.f;
  LaneCloudPtr lane_cloud_discretized(new LaneCloud);
  cloud_discretized(source, 0.2, t_min, t_max, lane_cloud_discretized);
  LaneCloudPtr lane_cloud_smoothed(new LaneCloud);

  SGFilter sg_filter;
  sg_filter.setRadius(2.f);
  sg_filter.setInputCloud(lane_cloud_discretized);
  sg_filter.filter(lane_cloud_smoothed);

  LaneCluster cluster;
  cluster.lane_id = -1;
  cluster.cloud = source;
  cluster.cloud_smoothed = lane_cloud_smoothed;
  cluster.start_point = start_point;
  cluster.end_point = end_point;
  cluster.center_point = center_point;
  cluster.hoff_params = hoff_coeff;
  cluster.tmin = t_min;
  cluster.tmax = t_max;
  cluster.deplacement = t_max - t_min;
  cluster.ratio =
      accumulate_distance(lane_cloud_smoothed) / cluster.deplacement;

  // ratio value is not accurate
  if (cluster.ratio > 1.2) {
    cluster.lane_type = LaneType::CURVE;
  } else {
    cluster.lane_type = LaneType::STRAIGHT;
  }
  return cluster;
}

void LaneFitting::initialize(const std::vector<LaneCloudPtr>& source,
                             std::vector<LaneCluster>& result) {
  result.clear();
  for (auto& cluster : source) {
    result.push_back(initialize(cluster));
  }
}

bool LaneFitting::find_referrence_line(const std::vector<LaneCluster>& clusters,
                                       Hoff& ref_params,
                                       Eigen::Vector4f& ref_center) {
  double min_r = std::numeric_limits<double>::max();
  double max_r = std::numeric_limits<double>::min();
  double min_d = std::numeric_limits<double>::max();
  double max_d = std::numeric_limits<double>::min();
  for (size_t i = 0; i < clusters.size(); ++i) {
    if (clusters[i].lane_type == LaneType::STRAIGHT &&
        clusters[i].deplacement > 5.f) {
      min_r = std::min(clusters[i].hoff_params.r, min_r);
      max_r = std::max(clusters[i].hoff_params.r, max_r);

      min_d = std::min(clusters[i].hoff_params.d, min_d);
      max_d = std::max(clusters[i].hoff_params.d, max_d);
    }
  }
  const double rstep = M_PI / 180;
  const double dstep = 1.f;
  // const size_t rsize = (max_r - min_r) / rstep + 1;
  const size_t dsize = (max_d - min_d) / dstep + 1;

  std::unordered_map<size_t, LaneCloud> hoff_cluster;
  for (size_t i = 0; i < clusters.size(); ++i) {
    if (clusters[i].lane_type == LaneType::STRAIGHT &&
        clusters[i].deplacement > 5.f) {
      size_t ridx = (clusters[i].hoff_params.r - min_r) / rstep;
      size_t didx = (clusters[i].hoff_params.d - min_d) / dstep;
      const size_t hoff_id = ridx * dsize + didx;
      if (hoff_cluster.find(hoff_id) != hoff_cluster.end()) {
        // for (auto& point : clusters[i].cloud->points) {
        //   hoff_cluster[hoff_id]->push_back(point);
        // }
        hoff_cluster[hoff_id] += *clusters[i].cloud;
      } else {
        LaneCloud tcloud = *clusters[i].cloud;
        hoff_cluster.insert({hoff_id, tcloud});
      }
    }
  }
  //
  size_t point_size = 0;
  for (auto& cluster : clusters) {
    point_size += cluster.cloud->size();
  }
  std::cout << "in ref cluster point size: " << point_size << std::endl;
  //
  size_t ref_id = 0;
  size_t max_size = 0;
  for (auto& pair : hoff_cluster) {
    if (pair.second.size() > max_size) {
      max_size = pair.second.size();
      ref_id = pair.first;
    }
  }
  if (max_size < 100) {
    return false;
  } else {
    LaneCloudPtr ref_cluster (new LaneCloud(hoff_cluster[ref_id]));
    FitLine1DByRegression(ref_cluster, ref_params, ref_center);
    return true;
  }
}

void LaneFitting::classify(std::vector<LaneCluster>& lane_clusters,
                           const Hoff& ref_params,
                           const Eigen::Vector4f& ref_center) {
  std::vector<float> dists;
  float min_dist = std::numeric_limits<float>::max();
  float max_dist = std::numeric_limits<float>::min();

  // calculate distance to reference line
  for (size_t i = 0; i < lane_clusters.size(); ++i) {
    const auto& center = lane_clusters[i].center_point;
    float dist_y = ref_params.s * center[0] + ref_params.d - center[1];
    dists.emplace_back(dist_y);
    min_dist = std::min(min_dist, dist_y);
    max_dist = std::max(max_dist, dist_y);
  }

  // Lane width
  const float dist_step = 3.75;
  // const size_t dis_num = (max_dist - min_dist) / dist_step + 1;

  for (size_t i = 0; i < lane_clusters.size(); ++i) {
    if (lane_clusters[i].lane_type != LaneType::STRAIGHT) {
      lane_clusters[i].lane_id = -1;
      continue;
    }
    size_t dist_id = (dists[i] - min_dist) / dist_step;
    double deplace =
        std::min(dists[i] - (dist_id * dist_step + min_dist),
                 ((dist_id + 1) * dist_step + min_dist - dists[i]));
    // threshold
    if (deplace < 0.25) {
      lane_clusters[i].lane_id = dist_id;
    } else {
      lane_clusters[i].lane_id = -1;
    }
  }
}

void LaneFitting::fit(const std::vector<SLCloudPtr>& source,
                      std::vector<LaneSegment>& result) {
  /*
  0. load pcd
  1. label -> raw_points + start&end
  2. dbscan -> raw_clusters (may have sidewalkers, charaters, etc.)
  3. for each cluster find start&end&center, fit line by start&end, calculate
  t for each points
  4. cloud discretized -> StraightLine,CubicLane,Others
  if StraightLine
    5. In HoffSpace -> params + centers
    6. find_referrence_line -> ref_params, ref_center
    7. distance to ref_line + lane_size -> different lanes
  others
    hermite spline
  */
  LaneCloudPtr filtered = raw_filter(source);
  std::vector<LaneCloudPtr> pc_clusters;
  dbscan(this->params_.dbscan_parameters, filtered, pc_clusters);
  std::vector<LaneCluster> clusters;
  initialize(pc_clusters, clusters);
  Hoff ref_params;
  Eigen::Vector4f ref_center;
  if (find_referrence_line(clusters, ref_params, ref_center)) {
    classify(clusters, ref_params, ref_center);
  } else {
    std::cout << "no reference line found" << std::endl;
  }

#ifdef DEBUG
  std::unordered_map<int, LaneCloudPtr> lane_clouds;
  for (auto& cluster : clusters) {
    if (lane_clouds.find(cluster.lane_id) != lane_clouds.end()) {
      for (auto& point : cluster.cloud->points) {
        lane_clouds[cluster.lane_id]->push_back(point);
      }
    } else {
      lane_clouds.insert({cluster.lane_id, cluster.cloud});
    }
  }
  size_t sum_points = 0;
  for (auto& pair : lane_clouds) {
    sum_points += pair.second->size();
    pcl::io::savePCDFileASCII("/home/demo/repos/sl_lane/data/hoff/" +
                                  std::to_string(pair.first) + ".pcd",
                              *pair.second);
  }
  std::cout << "Saved points: " << sum_points << std::endl;
#endif
}
}  // namespace smartlabel