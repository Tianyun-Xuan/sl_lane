#include "geometry_util.h"

namespace smartlabel {

float original_dst(const Point_Auto_Label &point) {
  return sqrt(pow(point.original_x, 2) + pow(point.original_y, 2) +
              pow(point.original_z, 2));
}

void project_point(const Point_Lane &src_point,
                   const pcl::ModelCoefficients::Ptr &coeff,
                   Point_Lane &dst_point) {
  float x0 = coeff->values[0];
  float y0 = coeff->values[1];
  float z0 = coeff->values[2];
  float a = coeff->values[3];
  float b = coeff->values[4];
  float c = coeff->values[5];
  float x2 = src_point.x;
  float y2 = src_point.y;
  float z2 = src_point.z;
  float t = a * (x2 - x0) + b * (y2 - y0) + c * (z2 - z0);
  dst_point.x = a * t + x0;
  dst_point.y = b * t + y0;
  dst_point.z = c * t + z0;
}

bool fit_line_3d(const LaneCloudPtr &cloud,
                 pcl::ModelCoefficients::Ptr &coeff) {
  if (cloud == nullptr || cloud->size() < 2) {
    return false;
  }

  double x2{0}, xy{0}, xz{0}, x{0}, y2{0}, yz{0}, y{0}, z2{0}, z{0};
  for (size_t i = 0; i < cloud->size(); ++i) {
    const auto &p = cloud->at(i);
    x += p.x;
    y += p.y;
    z += p.z;
  }
  double inv = 1.0 / static_cast<double>(cloud->size());
  x *= inv;
  y *= inv;
  z *= inv;
  for (size_t i = 0; i < cloud->size(); ++i) {
    const auto &p = cloud->at(i);
    double dx = p.x - x;
    double dy = p.y - y;
    double dz = p.z - z;
    x2 += dx * dx;
    xy += dx * dy;
    xz += dx * dz;
    y2 += dy * dy;
    yz += dy * dz;
    z2 += dz * dz;
  }
  Eigen::Matrix3d J;
  J << x2, xy, xz, xy, y2, yz, xz, yz, z2;
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(J, Eigen::ComputeFullU);
  auto U = svd.matrixU();
  double a = U(0, 0);
  double b = U(1, 0);
  double c = U(2, 0);

  coeff->values.clear();
  coeff->values.emplace_back(x);
  coeff->values.emplace_back(y);
  coeff->values.emplace_back(z);
  coeff->values.emplace_back(a);
  coeff->values.emplace_back(b);
  coeff->values.emplace_back(c);
  return true;
}

float accumulate_distance(const LaneCloudPtr &cloud) {
  float sum = 0.f;
  for (size_t i = 1; i < cloud->size(); ++i) {
    const auto &p1 = cloud->at(i - 1);
    const auto &p2 = cloud->at(i);
    float dx = p1.x - p2.x;
    float dy = p1.y - p2.y;
    float dz = p1.z - p2.z;
    sum += std::sqrt(dx * dx + dy * dy + dz * dz);
  }
  return sum;
}

bool find_end_point(const LaneCloudPtr &source, Point_Lane &start_point,
                    Point_Lane &end_point) {
  if (source == nullptr || source->size() < 2) {
    return false;
  }
  auto &seed_point = source->at(0);
  float max_dist = -(std::numeric_limits<float>::max)();
  int max_idx = 0;
  for (size_t i = 0; i < source->size(); ++i) {
    const auto &tmp_p = source->at(i);
    float dx = tmp_p.x - seed_point.x;
    float dy = tmp_p.y - seed_point.y;
    float dz = tmp_p.z - seed_point.z;
    float distance_sq = dx * dx + dy * dy + dz * dz;
    if (distance_sq > max_dist) {
      max_dist = distance_sq;
      max_idx = i;
    }
  }
  start_point = source->at(max_idx);
  seed_point = source->at(max_idx);
  max_dist = -(std::numeric_limits<float>::max)();
  max_idx = 0;
  for (size_t i = 0; i < source->size(); ++i) {
    const auto &tmp_p = source->at(i);
    float dx = tmp_p.x - seed_point.x;
    float dy = tmp_p.y - seed_point.y;
    float dz = tmp_p.z - seed_point.z;
    float distance_sq = dx * dx + dy * dy + dz * dz;
    if (distance_sq > max_dist) {
      max_dist = distance_sq;
      max_idx = i;
    }
  }
  end_point = source->at(max_idx);
  if (start_point.x > end_point.x) {
    std::swap(start_point, end_point);
  }
  return true;
}

void dbscan(const DBscan_Parameters &parameters, const LaneCloudPtr &source,
            std::vector<LaneCloudPtr> &clusters) {
  clusters.clear();
#define UNPROCESSED 1
#define PROCESSING 2
#define PROCESSED 0
#define NOISE -1
  std::vector<int> nn_indices;
  std::vector<float> nn_distances;
  std::vector<int> types(source->points.size(), UNPROCESSED);

  pcl::KdTreeFLANN<Point_Lane> kdtree_;
  kdtree_.setInputCloud(source);

  double eps = parameters.neighbour_radius;
  size_t minPts = parameters.minPts_in_neighbour;
  size_t min_pts_per_cluster = parameters.min_pts_per_cluster;
  size_t max_pts_per_cluster = source->size();

  size_t index = 0;

  for (size_t i = 0; i < source->points.size(); ++i) {  // global index
    if (types[i] == PROCESSED || types[i] == NOISE) {
      continue;
    }

    size_t nn_size = kdtree_.radiusSearch(i, eps, nn_indices, nn_distances);
    if (nn_size < minPts) {
      types[i] == NOISE;
      continue;
    }

    // cluster_id
    std::vector<int> seed_queue;
    seed_queue.push_back(i);
    types[i] = PROCESSED;

    for (size_t j = 0; j < nn_size; ++j) {
      if (nn_indices[j] != int(i)) {  // eliminate the seed point
        seed_queue.push_back(nn_indices[j]);
        types[nn_indices[j]] = PROCESSING;
      }
    }  // for every point near the chosen core point.

    size_t sq_idx = 1;
    while (sq_idx < seed_queue.size()) {
      int cloud_index = seed_queue[sq_idx];
      if (types[cloud_index] == NOISE || types[cloud_index] == PROCESSED) {
        types[cloud_index] = PROCESSED;
        ++sq_idx;
        continue;  // no need to check neighbors.
      }
      nn_size =
          kdtree_.radiusSearch(cloud_index, eps, nn_indices, nn_distances);
      if (nn_size >= minPts) {  // seems useless
        for (size_t j = 0; j < nn_size; j++) {
          if (types[nn_indices[j]] == UNPROCESSED) {
            seed_queue.push_back(nn_indices[j]);
            types[nn_indices[j]] = PROCESSING;
          }
        }
      }
      types[cloud_index] = PROCESSED;
      ++sq_idx;
    }
    // all the point in the same cluster are stocked in the queue

    if (seed_queue.size() >= min_pts_per_cluster &&
        seed_queue.size() <= max_pts_per_cluster) {
      ++index;
      pcl::PointCloud<Point_Lane>::Ptr tune(new pcl::PointCloud<Point_Lane>);

      for (size_t j = 0; j < seed_queue.size(); ++j) {
        tune->push_back(source->points[seed_queue[j]]);
      }
      clusters.push_back(tune);
    }
  }
}

void centerlize(const LaneCloudPtr &source, LaneCloudPtr &result,
                Eigen::Vector4f &centroid) {
  pcl::compute3DCentroid(*source, centroid);
  result->resize(source->size());
  for (size_t i = 0; i < source->size(); ++i) {
    result->points[i].x = source->points[i].x - centroid[0];
    result->points[i].y = source->points[i].y - centroid[1];
    result->points[i].z = source->points[i].z - centroid[2];
  }
}

void FitLine1DByRegression(const LaneCloudPtr &source, Hoff &parameters,
                           Eigen::Vector4f &centroid) {
  LaneCloudPtr descenter(new LaneCloud);
  centerlize(source, descenter, centroid);
  // AX = B
  const int N = 2;
  // Construct A mat
  cv::Mat A = cv::Mat::zeros(N, N, CV_64FC1);
  for (int row = 0; row < A.rows; ++row) {
    for (int col = 0; col < A.cols; ++col) {
      for (size_t k = 0; k < descenter->size(); ++k) {
        A.at<double>(row, col) =
            A.at<double>(row, col) + pow(descenter->points[k].x, row + col);
      }
    }
  }
  // // Normalization
  // const double lambda = 0.f;
  // for (int row = 0; row < A.rows; ++row) {
  //   for (int col = 0; col < A.cols; ++col) {
  //     A.at<double>(row, col) = A.at<double>(row, col) - lambda;
  //   }
  // }
  // Construct B mat
  cv::Mat B = cv::Mat::zeros(N, 1, CV_64FC1);
  for (int row = 0; row < B.rows; row++) {
    for (size_t k = 0; k < descenter->size(); ++k) {
      B.at<double>(row, 0) =
          B.at<double>(row, 0) +
          pow(descenter->points[k].x, row) * descenter->points[k].y;
    }
  }
  // Solve the A*X = B
  cv::Mat X;
  cv::solve(A, B, X, cv::DECOMP_LU);
  // [y = b + a * x] in local coordinate
  // [(y - centroid[1]) = b + a * (x - centroid[0]) + ] in global coordinate
  auto ga = X.at<double>(1, 0);
  auto gb = ga * (-centroid[0]) + X.at<double>(0, 0) + centroid[1];
  parameters = Hoff(ga, gb);
}  // FitLine1DByRegression

void FitLine3DByRegression(const LaneCloudPtr &source, Polynomia &parameters) {
  LaneCloudPtr descenter(new LaneCloud);
  centerlize(source, descenter, parameters.centroid);
  // AX = B
  const int N = 4;
  // Construct A mat
  cv::Mat A = cv::Mat::zeros(N, N, CV_64FC1);
  for (int row = 0; row < A.rows; ++row) {
    for (int col = 0; col < A.cols; ++col) {
      for (size_t k = 0; k < descenter->size(); ++k) {
        A.at<double>(row, col) += pow(descenter->points[k].x, row + col);
      }
    }
  }
  // Construct B mat
  cv::Mat B = cv::Mat::zeros(N, 1, CV_64FC1);
  for (int row = 0; row < B.rows; ++row) {
    for (size_t k = 0; k < descenter->size(); ++k) {
      B.at<double>(row, 0) +=
          pow(descenter->points[k].x, row) * descenter->points[k].y;
    }
  }
  cv::Mat X;
  cv::solve(A, B, X, cv::DECOMP_LU);
  // y = d + a * x + b * x^2 + c * x^3
  parameters.d = X.at<double>(0, 0);
  parameters.a = X.at<double>(1, 0);
  parameters.b = X.at<double>(2, 0);
  parameters.c = X.at<double>(3, 0);
}  // FitLine3DByRegression

bool cloud_discretized(const LaneCloudPtr &source, float invterval, float tmin,
                       float tmax, LaneCloudPtr &result) {
  result->clear();
  // get t grids
  float inv_step = 1.f / invterval;
  size_t t_size = (tmax - tmin) * inv_step + 1;
  std::vector<double> xlist(t_size, 0.f);
  std::vector<double> ylist(t_size, 0.f);
  std::vector<double> zlist(t_size, 0.f);
  std::vector<int> count_list(t_size, 0);

  // discrete
  for (const auto &point : source->points) {
    int grid_idx = (point.t - tmin) * inv_step + 0.5f;
    if (grid_idx < 0) {
      grid_idx = 0;
    } else if (grid_idx >= t_size) {
      grid_idx = t_size - 1;
    }
    xlist[grid_idx] += point.x;
    ylist[grid_idx] += point.y;
    zlist[grid_idx] += point.z;
    ++count_list[grid_idx];
  }
  for (size_t i = 0; i < t_size; ++i) {
    if (count_list[i] > 0) {
      float inv_count = 1.f / static_cast<float>(count_list[i]);
      Point_Lane tmp_point;
      tmp_point.x = xlist[i] * inv_count;
      tmp_point.y = ylist[i] * inv_count;
      tmp_point.z = zlist[i] * inv_count;
      tmp_point.intensity = 254.f;
      result->push_back(tmp_point);
    }
  }
  return true;
}
}  // namespace smartlabel