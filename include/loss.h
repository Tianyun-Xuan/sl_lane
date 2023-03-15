#ifndef LOSS_H
#define LOSS_H
// clang-format off
#include "point_type.h"
// clang-format on
#include <Eigen/Core>
#include <munker.hpp>

#include "geometry_util.h"
namespace smartlabel {

struct EndPoint {
  Point_Lane point;
  Eigen::Vector4f tangent;
  size_t cluster_id;
  bool is_start;  // True if start point, false if end point
};

double distance_value(const EndPoint& a, const EndPoint& b);
double distance_loss(const EndPoint& a, const EndPoint& b);
double angle_value(const EndPoint& a, const EndPoint& b);
double angle_loss(const EndPoint& a, const EndPoint& b);
double time_loss(const EndPoint& a, const EndPoint& b);
double connect_loss(const EndPoint& a, const EndPoint& b);
double loss_function(const EndPoint& a, const EndPoint& b);
double hermite_interpolate_loss(const EndPoint& a, const EndPoint& b);

void solve_km(const std::vector<EndPoint>& inputs, std::vector<int>& result);
}  // namespace smartlabel

#endif /* !defined(LOSS_H) */
