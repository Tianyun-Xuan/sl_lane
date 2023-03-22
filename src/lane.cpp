#include "lane.h"

#include <unordered_set>
#define DEBUG
namespace smartlabel {

LaneSegment::LaneSegment(const LaneCloudPtr& cloud) {
  this->cloud = LaneCloudPtr(new LaneCloud(*cloud));
  FitLine3DByRegression(this->cloud, this->parameters);
  // Attention: the start point and end point are decided by the order of x
  find_end_point(this->cloud, this->start_point, this->end_point);
  this->cloud_smoothed = this->print(0.1);
}

LaneSegment operator+=(LaneSegment& lhs, const LaneSegment& rhs) {
  LaneCloudPtr cloud(new LaneCloud);
  *cloud += *lhs.cloud;
  *cloud += *rhs.cloud;
  lhs = LaneSegment(cloud);
  return lhs;
}

LaneCloudPtr LaneCluster::print(const double step) {
  LaneCloudPtr cloud(new LaneCloud);
  for (double x = start_point.x; x < end_point.x; x += step) {
    Point_Lane point;
    point.x = x;
    point.y = hoff_params.s * x + hoff_params.d;
    // trick here for hausdorff distance to calculate only on plane x-y
    point.z = 0.f;
    cloud->push_back(point);
  }
  return cloud;
}

LaneCloudPtr LaneSegment::print(const double step) {
  LaneCloudPtr cloud(new LaneCloud);
  for (double x = start_point.x; x < end_point.x; x += step) {
    Point_Lane point = parameters.projection(x);
    cloud->push_back(point);
  }
  return cloud;
}

bool LaneFitting::is_lane(const Point_Auto_Label& point) {
  return point.motion_type == this->params_.value_motion_type &&
         point.semantic_3d_label == this->params_.value_lane_type &&
         original_dst(point) < this->params_.value_original_distance;
}

bool LaneFitting::is_ground(const Point_Auto_Label& point) {
  return point.motion_type == this->params_.value_motion_type &&
         point.semantic_3d_label == this->params_.value_ground_type &&
         original_dst(point) < this->params_.value_original_distance;
}

void LaneFitting::label_filter(const std::vector<SLCloudPtr>& source,
                               const LaneCloudPtr& lane_cloud,
                               const LaneCloudPtr& ground_cloud) {
  lane_cloud->clear();
  ground_cloud->clear();
  for (const auto& cloud : source) {
    for (const auto& point : cloud->points) {
      m_x_max_ = std::fmax(m_x_max_, point.x);
      m_x_min_ = std::fmin(m_x_min_, point.x);
      m_y_max_ = std::fmax(m_y_max_, point.y);
      m_y_min_ = std::fmin(m_y_min_, point.y);
      if (is_lane(point)) {
        Point_Lane lane_point;
        lane_point.x = point.x;
        lane_point.y = point.y;
        lane_point.z = point.z;
        lane_point.intensity = point.intensity;
        lane_point.timestamp = point.timestamp;
        lane_point.t = 0;
        lane_cloud->push_back(lane_point);
        ground_cloud->push_back(lane_point);
      } else if (is_ground(point)) {
        Point_Lane ground_point;
        ground_point.x = point.x;
        ground_point.y = point.y;
        ground_point.z = point.z;
        ground_point.intensity = point.intensity;
        ground_point.timestamp = point.timestamp;
        ground_point.t = 0;
        ground_cloud->push_back(ground_point);
        continue;
      }
    }
  }
}

// (TODO: design a class for this)
LaneCluster LaneFitting::initialize(const LaneCloudPtr& source) {
  // Initialize the cluster
  LaneCluster cluster;
  cluster.lane_id = -1;
  cluster.cloud = source;
  cluster.tmin = std::numeric_limits<float>::max();
  cluster.tmax = -(std::numeric_limits<float>::max)();

  // Fit the lane cloud
  FitLine1DByRegression(source, cluster.hoff_params, cluster.center_point);

  double ts_sum = 0.f;
  for (auto& point : source->points) {
    point.t = (point.x - cluster.center_point[0]) * cluster.hoff_params.tx +
              (point.y - cluster.center_point[1]) * cluster.hoff_params.ty;
    cluster.tmax = std::fmax(point.t, cluster.tmax);
    cluster.tmin = std::fmin(point.t, cluster.tmin);
    ts_sum += point.timestamp;
  }
  cluster.deplacement = cluster.tmax - cluster.tmin;
  // (Deprecated: replace with start_point and end_point of cloud discretized)
  // Point_Lane start_point;
  // Point_Lane end_point;
  // start_point.x = center_point[0] + t_min * hoff_coeff.tx;
  // start_point.y = center_point[1] + t_min * hoff_coeff.ty;
  // start_point.z = 0.f;
  // end_point.x = center_point[0] + t_max * hoff_coeff.tx;
  // end_point.y = center_point[1] + t_max * hoff_coeff.ty;
  // start_point.z = 0.f;
  LaneCloudPtr lane_cloud_discretized(new LaneCloud);
  // (TODO: parameters)
  cloud_discretized(source, 0.2, cluster.tmin, cluster.tmax,
                    lane_cloud_discretized);
  LaneCloudPtr temp_cloud_smoothed(new LaneCloud);
  cluster.cloud_smoothed.swap(temp_cloud_smoothed);
  // Smooth the lane cloud
  SGFilter sg_filter;
  // (TODO: parameters)
  sg_filter.setRadius(2.f);
  sg_filter.setInputCloud(lane_cloud_discretized);
  sg_filter.filter(cluster.cloud_smoothed);

  cluster.start_point = cluster.cloud_smoothed->points.front();
  cluster.end_point = cluster.cloud_smoothed->points.back();

  std::vector<pcl::ModelCoefficients::Ptr> coefficients;
  // (TODO: parameters)
  TangentLine(cluster.cloud_smoothed, 5.f, coefficients);
  assert(coefficients.size() == 2);
  Eigen::Vector4f start_point_tangent(coefficients[0]->values[3],
                                      coefficients[0]->values[4],
                                      coefficients[0]->values[5], 0.f);
  Eigen::Vector4f end_point_tangent(coefficients[1]->values[3],
                                    coefficients[1]->values[4],
                                    coefficients[1]->values[5], 0.f);
  //(TODO) I am not sure the 4th element of the vector is 0 or 1
  Eigen::Vector4f center_start =
      Eigen::Vector4f(cluster.start_point.x, cluster.start_point.y,
                      cluster.start_point.z, cluster.center_point[3]) -
      cluster.center_point;
  Eigen::Vector4f center_end =
      Eigen::Vector4f(cluster.end_point.x, cluster.end_point.y,
                      cluster.end_point.z, cluster.center_point[3]) -
      cluster.center_point;

  cluster.start_point_tangent = start_point_tangent.dot(center_start) > 0
                                    ? start_point_tangent
                                    : -start_point_tangent;
  cluster.end_point_tangent = end_point_tangent.dot(center_end) > 0
                                  ? end_point_tangent
                                  : -end_point_tangent;
  cluster.ratio =
      accumulate_distance(cluster.cloud_smoothed) / cluster.deplacement;
  cluster.timestamp = ts_sum / source->size();

  // ratio value is not accurate
  // (TODO: parameters)
  if (cluster.ratio > 1.15) {
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
#ifdef DEBUG
  size_t point_size = 0;
  for (auto& cluster : clusters) {
    point_size += cluster.cloud->size();
  }

  std::cout << "in ref cluster point size: " << point_size << std::endl;
#endif
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
    LaneCloudPtr ref_cluster(new LaneCloud(hoff_cluster[ref_id]));
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

  // (TODO: Parameters) Lane width
  const float dist_step = 3.75;

  for (size_t i = 0; i < lane_clusters.size(); ++i) {
    if (lane_clusters[i].lane_type != LaneType::STRAIGHT) {
      lane_clusters[i].lane_id = -1;
      continue;
    }
    size_t dist_id = (dists[i] - min_dist) / dist_step;
    double deplace =
        std::min(dists[i] - (dist_id * dist_step + min_dist),
                 ((dist_id + 1) * dist_step + min_dist - dists[i]));
    // (TODO: Parameters) distance threshold
    if (deplace < 0.25) {
      lane_clusters[i].lane_id = dist_id;
    } else {
      lane_clusters[i].lane_id = -1;
    }
  }
}
void LaneFitting::ground_map(const LaneCloudPtr& ground_cloud) {
  const size_t x_size = (m_x_max_ - m_x_min_) / this->params_.x_step + 1;
  const size_t y_size = (m_y_max_ - m_y_min_) / this->params_.y_step + 1;
  m_height_map_.resize(x_size, std::vector<double>(y_size, 0));
  m_intensity_map_.resize(x_size, std::vector<double>(y_size, 0));
  std::vector<std::vector<size_t>> count_map(x_size,
                                             std::vector<size_t>(y_size, 0));
  for (auto& point : ground_cloud->points) {
    const size_t x_idx = (point.x - m_x_min_) / params_.x_step;
    const size_t y_idx = (point.y - m_y_min_) / params_.y_step;
    m_height_map_[x_idx][y_idx] += point.z;
    m_intensity_map_[x_idx][y_idx] += point.intensity;
    count_map[x_idx][y_idx] += 1;
  }
  for (size_t i = 0; i < x_size; ++i) {
    for (size_t j = 0; j < y_size; ++j) {
      if (count_map[i][j] > 0) {
        m_height_map_[i][j] /= count_map[i][j];
        m_intensity_map_[i][j] /= count_map[i][j];
      }
    }
  }
}

void LaneFitting::extract_candidate(const std::vector<SLCloudPtr>& source,
                                    const std::vector<LaneCluster>& clusters,
                                    std::vector<SLCloudPtr>& candidates) {
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
#ifdef DEBUG
  for (auto& pair : lane_clouds) {
    pcl::io::savePCDFileASCII("/home/demo/repos/sl_lane/data/hoff/" +
                                  std::to_string(pair.first) + ".pcd",
                              *pair.second);
  }
#endif

  std::vector<LaneCluster> straight_lane_cluster;
  for (auto& pair : lane_clouds) {
    if (pair.first == -1) {
      continue;
    }
    auto temp_cluster = initialize(pair.second);
    temp_cluster.lane_id = pair.first;
    straight_lane_cluster.push_back(temp_cluster);
  }
#ifdef DEBUG
  for (auto& cluster : straight_lane_cluster) {
    pcl::io::savePCDFileBinary("/home/demo/repos/sl_lane/data/straight/" +
                                   std::to_string(cluster.lane_id) + ".pcd",
                               *cluster.print(0.1));
  }
#endif
  candidates.clear();
  for (size_t i = 0; i < straight_lane_cluster.size(); ++i) {
    SLCloudPtr temp(new SLCloud);
    candidates.push_back(temp);
  }
  for (const auto& cloud : source) {
    for (const auto& point : cloud->points) {
      const size_t x_idx = (point.x - m_x_min_) / params_.x_step;
      const size_t y_idx = (point.y - m_y_min_) / params_.y_step;
      const double z_dis = fabs(m_height_map_[x_idx][y_idx] - point.z);
      if (z_dis < 0.09) {  // 3Σ = 0.09
        for (size_t i = 0; i < straight_lane_cluster.size(); ++i) {
          const auto& cluster = straight_lane_cluster[i];
          const double y_dis = fabs(cluster.hoff_params.s * point.x +
                                    cluster.hoff_params.d - point.y);
          if (y_dis < 0.3) {  // width 15cm,
            candidates[i]->push_back(point);
            continue;
          }
        }
      }
    }
  }
}

void LaneFitting::extract_candidate(const std::vector<SLCloudPtr>& source,
                                    const std::vector<LaneSegment>& segments,
                                    std::vector<SLCloudPtr>& candidates) {
  candidates.clear();
  for (size_t i = 0; i < segments.size(); ++i) {
    SLCloudPtr temp(new SLCloud);
    candidates.push_back(temp);
  }

  for (const auto& cloud : source) {
    for (const auto& point : cloud->points) {
      const size_t x_idx = (point.x - m_x_min_) / params_.x_step;
      const size_t y_idx = (point.y - m_y_min_) / params_.y_step;
      const double z_dis = fabs(m_height_map_[x_idx][y_idx] - point.z);

      if (z_dis < 0.09) {  // 3Σ = 0.09
        for (size_t i = 0; i < segments.size(); ++i) {
          const auto& segment = segments[i];
          if (segment.in_interval(point.x)) {
            const double y_dis = fabs(segment.y(point.x) - point.y);
            if (y_dis < 0.3) {  // width 15cm,
              candidates[i]->push_back(point);
            }
          }
        }
      }
    }
  }
}

// SLCloudPtr LaneFitting::select_candidate(const SLCloudPtr& candidates) {
//   double sum = 0;
//   for (auto& point : candidates->points) {
//     sum += point.intensity;
//   }
//   const double mean = sum / candidates->points.size();
//   // calculate RootMeanSquareError
//   double rmse = 0;
//   for (auto& point : candidates->points) {
//     rmse += (point.intensity - mean) * (point.intensity - mean);
//   }
//   rmse = sqrt(rmse / candidates->points.size());
//   std::cout << "mean: " << mean << ", rmse: " << rmse << std::endl;
//   // select points
//   SLCloudPtr selected(new SLCloud);
//   for (auto& point : candidates->points) {
//     if (point.intensity - mean > 2 * rmse) {
//       selected->push_back(point);
//     }
//   }
// }

SLCloudPtr LaneFitting::adapte_intesnity_filter(const SLCloudPtr& candidates,
                                                const double radius,
                                                const double threshold) {
  pcl::KdTreeFLANN<Point_Auto_Label> kdtree;
  kdtree.setInputCloud(candidates);
  SLCloudPtr selected(new SLCloud);
  // search neighbors points
  for (auto& point : candidates->points) {
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;
    if (kdtree.radiusSearch(point, radius, pointIdxRadiusSearch,
                            pointRadiusSquaredDistance) > 0) {
      double sum = 0;
      for (auto& idx : pointIdxRadiusSearch) {
        sum += candidates->points[idx].intensity;
      }
      // calculate mean
      double mean = sum / pointIdxRadiusSearch.size();
      if (point.intensity - mean > threshold) {
        selected->push_back(point);
      }
    }
  }
  return selected;
}

SLCloudPtr LaneFitting::neiborhood_filter(const SLCloudPtr& candidates,
                                          const double radius,
                                          const size_t threshold) {
  pcl::KdTreeFLANN<Point_Auto_Label> kdtree;
  kdtree.setInputCloud(candidates);
  SLCloudPtr selected(new SLCloud);
  // search neighbors points
  for (auto& point : candidates->points) {
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;
    if (kdtree.radiusSearch(point, radius, pointIdxRadiusSearch,
                            pointRadiusSquaredDistance) > 0) {
      if (pointIdxRadiusSearch.size() > threshold) {
        selected->push_back(point);
      }
    }
  }
  return selected;
}

void check_size(const std::vector<SLCloudPtr>& source,
                const std::string& flag) {
  size_t sum = 0;
  for (const auto& cloud : source) {
    sum += cloud->points.size();
  }
  std::cout << flag << " total size: " << sum << std::endl;
}

void LaneFitting::fit(const std::vector<SLCloudPtr>& source,
                      std::vector<SLCloudPtr>& result) {
  // Collect Lane Points and Lane + Ground Points
  LaneCloudPtr filtered(new LaneCloud);
  LaneCloudPtr ground_cloud(new LaneCloud);
  label_filter(source, filtered, ground_cloud);

  // generate height map and intensity map
  ground_map(ground_cloud);

  // linearity filter
  LaneCloudPtr linearity_filtered(new LaneCloud);
  linearity_filter(filtered, linearity_filtered, 0.9, 3);

  // DBSCAN
  std::vector<LaneCloudPtr> pc_clusters;
  dbscan(this->params_.dbscan_parameters, linearity_filtered, pc_clusters);

  // Calculate Lane Parameters and sort by timestamp
  std::vector<LaneCluster> raw_clusters;
  initialize(pc_clusters, raw_clusters);
  std::sort(raw_clusters.begin(), raw_clusters.end(),
            [](const LaneCluster& a, const LaneCluster& b) {
              return a.timestamp < b.timestamp;
            });

  std::vector<LaneCluster> clusters;
  for (auto& raw_cluster : raw_clusters) {
    if (raw_cluster.deplacement * raw_cluster.ratio < 2.f) {
      continue;
    }
    clusters.push_back(raw_cluster);
  }

  std::vector<EndPoint> end_points;
  for (size_t i = 0; i < clusters.size(); ++i) {
#ifdef DEBUG
    pcl::io::savePCDFileBinary(
        "/home/demo/repos/sl_lane/data/clusters/" + std::to_string(i) + ".pcd",
        *clusters[i].cloud);
    pcl::io::savePCDFileBinary(
        "/home/demo/repos/sl_lane/data/debug/" + std::to_string(i) + ".pcd",
        *clusters[i].cloud_smoothed);
#endif
    {
      EndPoint start_point;
      start_point.point = clusters[i].start_point;
      start_point.tangent = clusters[i].start_point_tangent.normalized();
      start_point.cluster_id = i;
      start_point.is_start = 1;
      end_points.push_back(start_point);
    }
    {
      EndPoint end_point;
      end_point.point = clusters[i].end_point;
      end_point.tangent = clusters[i].end_point_tangent.normalized();
      end_point.cluster_id = i;
      end_point.is_start = 0;
      end_points.push_back(end_point);
    }
  }
  std::vector<std::set<int>> connect_flags;
  std::vector<LaneCloudPtr> connect_cloud;
  solve_km(end_points, connect_cloud, connect_flags);

  std::vector<LaneSegment> segments;
  for (size_t i = 0; i < connect_flags.size(); ++i) {
    LaneCloudPtr cloud(new pcl::PointCloud<Point_Lane>);
    for (auto& j : connect_flags[i]) {
      *cloud += *clusters[j].cloud_smoothed;
    }
    *cloud += *connect_cloud[i];
    LaneSegment segment(cloud);
    segments.push_back(segment);
#ifdef DEBUG
    pcl::io::savePCDFileBinary(
        "/home/demo/repos/sl_lane/data/straight/" + std::to_string(i) + ".pcd",
        *cloud);
#endif
  }

  // use hausdorff distance to merge line segments
  // hausdorff distance is directional that means hausdorff(A,B) !=
  // hausdorff(B,A). while lane merge action is indirectional which means
  // merge(A,B) == merge(B,A).
  // It may exist several merge actions for one lane segment.
  // For example, lane segment A and B are merged to C, and lane segment C and D
  // are merged to E. In this case, lane segment A and D are merged to E.
  // So we need to merge lane segments in a loop until no lane segment can be
  // merged.

  for (size_t i = 0; i < segments.size(); ++i) {
    for (size_t j = i + 1; j < segments.size(); ++j) {
      if (hausdorff_distance<Point_Lane>(segments[i].cloud, segments[j].cloud) <
              0.5 ||
          hausdorff_distance<Point_Lane>(segments[j].cloud, segments[i].cloud) <
              0.5) {
        segments[i] += segments[j];
        segments.erase(segments.begin() + j);
      }
    }
  }
#ifdef DEBUG
  for (size_t i = 0; i < segments.size(); ++i) {
    pcl::io::savePCDFileBinary(
        "/home/demo/repos/sl_lane/data/smooth/" + std::to_string(i) + ".pcd",
        *segments[i].cloud_smoothed);
  }
#endif

  std::vector<SLCloudPtr> candidates;
  extract_candidate(source, segments, candidates);

  std::vector<std::unordered_set<size_t>> points_to_label(
      source.size(), std::unordered_set<size_t>{});

  for (size_t i = 0; i < candidates.size(); ++i) {
    auto intensity_filtered_cloud =
        adapte_intesnity_filter(candidates[i], 1, 5);
    for (const auto& point : intensity_filtered_cloud->points) {
      // mamamiya
      points_to_label[point.frame_index].insert(point.point_index);
    }
  }

  result.clear();
  for (size_t i = 0; i < source.size(); ++i) {
    // Deep copy
    SLCloudPtr res_cloud(new SLCloud(*source[i]));
    for (auto& point : res_cloud->points) {
      // mamamiya
      if (points_to_label[point.frame_index].count(point.point_index)) {
        point.relabel = this->params_.value_lane_type;
        point.motion_type = this->params_.value_motion_type;
      } else {
        if (point.relabel == this->params_.value_lane_type) {
          point.relabel = this->params_.value_ground_type;
        }
      }
    }
    result.push_back(res_cloud);
  }
}

}  // namespace smartlabel