#include "loss.h"

#include <chrono>
namespace smartlabel {
double distance_value(const EndPoint& a, const EndPoint& b) {
  return (a.point.getVector3fMap() - b.point.getVector3fMap()).norm();
}
double distance_loss(const EndPoint& a, const EndPoint& b) {
  // threshold = 100m
  double distance = distance_value(a, b);
  return distance > 100.f ? 1.f : distance / 100.f;
}

double angle_value(const EndPoint& a, const EndPoint& b) {
  auto va = a.tangent.normalized();
  auto vb = b.tangent.normalized();
  // [0, pi]
  double angle = std::acos(va.dot(vb));
  return angle;
}
double angle_loss(const EndPoint& a, const EndPoint& b) {
  auto angle = angle_value(a, b);
  return angle < M_PI / 2 ? 1.f : (M_PI - angle) / (M_PI / 2);
}
double time_loss(const EndPoint& a, const EndPoint& b) {
  // threshold = 10s
  double time = std::abs(a.point.timestamp - b.point.timestamp);
  return time > 10 ? 1.f : time / 10.f;
}

double connect_value(const EndPoint& a, const EndPoint& b) {
  Eigen::Vector4f vconnect =
      b.point.getVector4fMap() - a.point.getVector4fMap();
  vconnect = vconnect.normalized();
  // [-1, 1]
  return vconnect.dot(a.tangent.normalized());
}

double connect_loss(const EndPoint& a, const EndPoint& b) {
  //
  double connect_threshold = 0.95;  // 20 degree
  auto value = connect_value(a, b);
  return value < connect_threshold ? 1.f
                                   : (1 - value) / (1 - connect_threshold);
}

double connect_loss(const double value) {
  //
  double connect_threshold = 0.95;  // 20 degree
  return value < connect_threshold ? 1.f
                                   : (1 - value) / (1 - connect_threshold);
}

double distance_tangential(const EndPoint& a, const EndPoint& b) {
  Eigen::Vector4f vconnect =
      b.point.getVector4fMap() - a.point.getVector4fMap();
  return vconnect.dot(a.tangent);
}

double distance_lateral(const EndPoint& a, const EndPoint& b) {
  auto distance = distance_value(a, b);
  auto tangential = distance_tangential(a, b);
  return std::sqrt(distance * distance - tangential * tangential);
}

// KM loss [0, 1] 0 is best
double loss_function(const EndPoint& a, const EndPoint& b) {
  // (TODO paramters)
  // const double lambda_distance = 0.3;
  // const double lambda_angle = 0.2;
  // const double lambda_time = 0.1;
  const double lambda_connect = 0.25;
  const double lambda_hermite = 0.5;
  const double hermite_distance = hermite_interpolate_distance(a, b);
  const double angle = angle_value(a, b);
  const double connected_a_b = connect_value(a, b);
  const double connected_b_a = connect_value(b, a);
  if (angle < M_PI / 2 || hermite_distance > 50.f ||
      a.cluster_id == b.cluster_id || connected_a_b < 0.f ||
      connected_b_a < 0.f || connected_a_b * connected_b_a < 0.75) {
    return 1.f;
  } else {
    // please make sure loss(a,b) == loss(b,a)
    return lambda_connect * connect_loss(connected_a_b) +
           lambda_connect * connect_loss(connected_b_a) +
           lambda_hermite * hermite_interpolate_loss(a, b, hermite_distance);
  }
}

void solve_km(const std::vector<EndPoint>& inputs,
              std::vector<LaneCloudPtr>& connect_cloud,
              std::vector<std::set<int>>& result) {
  std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
  connect_cloud.clear();
  result.clear();
  size_t n = inputs.size();
  Matrix<double> cost(n, n);
  // must be a symmetric matrix
  // so only calculate right-top triangle
  for (size_t i = 0; i < n; ++i) {
    for (size_t j = 0; j < n; ++j) {
      if (i == j) {
        cost(i, j) = 100.f;
      } else if (i < j) {
        cost(i, j) = 100.f * loss_function(inputs[i], inputs[j]);
      } else {
        cost(i, j) = cost(j, i);
      }
    }
  }

  Munkres<double> km;
  Matrix<double> copy_cost = cost;

  // for (size_t i = 0; i < n; ++i) {
  //   for (size_t j = 0; j < n; ++j) {
  //     if (cost(i, j) != cost(j, i)) {
  //       std::cout << "Not symmetric " << i << " " << j << " " << cost(i, j)
  //                 << " " << cost(j, i) << std::endl;
  //     }
  //   }
  // }
  std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
  auto duration =
      std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count();
  std::cout << "Generating cost matrix time " << duration << "ms" << std::endl;

  km.solve(cost);

  ConnectVector connect_vector(n);
  for (size_t i = 0; i < n; ++i) {
    for (size_t j = 0; j < n; ++j) {
      if (cost(i, j) == 0) {
        auto angle = angle_value(inputs[i], inputs[j]);
        auto distance = distance_value(inputs[i], inputs[j]);
        if (angle > M_PI / 2 &&  // direction is opposite
            distance < 80.f &&   // distance is less than 80m
            inputs[i].cluster_id !=
                inputs[j].cluster_id &&  // not in same cluster
            copy_cost(i, j) < 100.f) {   // not forbidden

          if (!connect_vector.connect(i, j)) {
            std::cout << "Not connected " << i << " " << j << std::endl;
            std::cout << connect_vector.map[i] << " " << connect_vector.map[j]
                      << std::endl;
          }
        }
      }
    }
  }

  std::vector<bool> visited(n, false);
  for (size_t i = 0; i < n; ++i) {
    if (!visited[i] && connect_vector.map[i] == -1) {  // root point
      size_t cluster_id = i / 2;
      int left = i % 2 ? i - 1 : i + 1;
      visited[i] = true;
      visited[left] = true;

      std::set<int> cluster;
      cluster.insert(cluster_id);
      LaneCloudPtr cloud(new LaneCloud);
      int right = connect_vector.map[left];

      while (right != -1) {
        visited[right] = true;
        cluster_id = right / 2;
        cluster.insert(cluster_id);
        LaneCloudPtr interpolated_cloud =
            hermite_interpolate_cloud(inputs[left], inputs[right]);
        *cloud += *interpolated_cloud;
        left = right % 2 ? right - 1 : right + 1;
        visited[left] = true;
        right = connect_vector.map[left];
      }
      result.push_back(cluster);
      connect_cloud.push_back(cloud);
    }
  }
}

double hermite_interpolate_distance(const EndPoint& a, const EndPoint& b) {
  return accumulate_distance(hermite_interpolate_cloud(a, b));
}

LaneCloudPtr hermite_interpolate_cloud(const EndPoint& a, const EndPoint& b) {
  //(TODO parameters)
  const double interval = 0.1;
  // check slope
  const double k1 = a.tangent[0] != 0.f ? a.tangent[1] / a.tangent[0] : 1000.f;
  const double k2 = b.tangent[0] != 0.f ? b.tangent[1] / b.tangent[0] : 1000.f;
  LaneCloudPtr interpolated_cloud(new LaneCloud);
  hermite_interpolate_2points(a.point, b.point, k1, k2, interval,
                              interpolated_cloud);
  return interpolated_cloud;
}

double hermite_interpolate_loss(const EndPoint& a, const EndPoint& b) {
  //(TODO parameters)

  const double distance_threshold = 50.f;
  const double distance_threshold_square =
      distance_threshold * distance_threshold;

  // To avoid the tangent of two point are perpendicular
  if (std::abs(a.tangent.dot(b.tangent)) < 0.1) {
    return 1.f;
  }

  auto distance = hermite_interpolate_distance(a, b);
  // (-inf, 1] -> [0, +inf)
  float loss = 1.f - (distance_threshold_square - distance * distance) /
                         distance_threshold_square;
  // [0,1] -> [best, worst]
  return loss > 1.f ? 1.f : loss;
}

double hermite_interpolate_loss(const EndPoint& a, const EndPoint& b,
                                const double value) {
  //(TODO parameters)

  const double distance_threshold = 50.f;
  const double distance_threshold_square =
      distance_threshold * distance_threshold;

  // To avoid the tangent of two point are perpendicular
  if (std::abs(a.tangent.dot(b.tangent)) < 0.1) {
    return 1.f;
  }
  // (-inf, 1] -> [0, +inf)
  float loss = 1.f - (distance_threshold_square - value * value) /
                         distance_threshold_square;
  // [0,1] -> [best, worst]
  return loss > 1.f ? 1.f : loss;
}
}  // namespace smartlabel
