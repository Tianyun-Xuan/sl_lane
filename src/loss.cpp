#include "loss.h"
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

double connect_loss(const EndPoint& a, const EndPoint& b) {
  Eigen::Vector4f vconnect =
      b.point.getVector4fMap() - a.point.getVector4fMap();
  vconnect = vconnect.normalized();
  // [-1, 1] -> [worst, best]
  return 1.f - (vconnect.dot(a.tangent.normalized()) + 1.f) / 2.f;
}

// KM loss [0, 1] 0 is best
double loss_function(const EndPoint& a, const EndPoint& b) {
  // const double lambda_distance = 0.3;
  // const double lambda_angle = 0.2;
  // const double lambda_time = 0.1;
  const double lambda_connect = 0.5;
  if (angle_value(a, b) < M_PI / 2 && distance_value(a, b) < 100.f) {
    return 1.f;
  } else {
    return lambda_connect * connect_loss(a, b) +
           lambda_connect * connect_loss(a, b);
  }
}

void solve_km(const std::vector<EndPoint>& inputs, std::vector<int>& result) {
  size_t n = inputs.size();
  Matrix<double> cost(n, n);
  for (size_t i = 0; i < n; ++i) {
    for (size_t j = 0; j < n; ++j) {
      if (i == j) {
        cost(i, j) = 1;
        continue;
      } else {
        cost(i, j) = 100.f * loss_function(inputs[i], inputs[j]);
      }
    }
  }
  Munkres<double> km;
  km.solve(cost);
  // note the target cluster
  // if the i-th element of result is j, then the i-th cluster's end point
  // should be connected to the j-th cluster's start point, else note -1
  result.resize(n, -1);
  for (size_t i = 0; i < n; ++i) {
    for (size_t j = 0; j < n; ++j) {
      if (cost(i, j) == 0) {
        auto angle = angle_value(inputs[i], inputs[j]);
        auto distance = distance_value(inputs[i], inputs[j]);
        if (angle > M_PI / 2 && distance < 100.f) {
          // if inputs[i] is start point, then connect to inputs[j]'s end point
          if (inputs[i].is_start) {
            result[i] = j;
          } else {
            result[j] = i;
          }
          continue;
        }
      }
    }
  }
}
}  // namespace smartlabel