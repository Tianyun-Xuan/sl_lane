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
  bool is_start;  // True if start point (timestamp), false if end point

  friend std::ostream& operator<<(std::ostream& os, const EndPoint& node) {
    os << "point: " << node.point.x << ", " << node.point.y << ", "
       << node.point.z << std::endl
       << "tangent: " << node.tangent << std::endl
       << "cluster_id: " << node.cluster_id << std::endl
       << "is_start: " << node.is_start << std::endl;
    return os;
  }
};

class ConnectVector {
 public:
  std::vector<int> map;
  ConnectVector(size_t n) : map(n, -1) {}

  bool connect(const int i, const int j) {
    if (i == j) return false;
    // once connected, connect both side
    if (map[i] == -1 && map[j] == -1) {
      map[i] = j;
      map[j] = i;
    } else if (map[i] != j || map[j] != i) {
      return false;
    }
    return true;
  }
};

// struct PairPoint {
//   int cluster_id = -1;
//   bool point_id = false;

//   PairPoint() : cluster_id(-1), point_id(false) {}
//   PairPoint(int cluster_id, bool point_id)
//       : cluster_id(cluster_id), point_id(point_id) {}

//   bool operator==(const PairPoint& other) const {
//     return cluster_id == other.cluster_id && point_id == other.point_id;
//   }

//   bool operator!=(const PairPoint& other) const {
//     return cluster_id != other.cluster_id || point_id != other.point_id;
//   }

//   bool inited() const { return cluster_id != -1; }

//   // int cluster() const { return cluster_id; }
//   // bool point() const { return point_id; }
// };

// class NodePoint {
//  public:
//   int cluster_id = -1;
//   PairPoint left;
//   PairPoint right;

//   NodePoint() = default;
//   NodePoint(size_t i) : cluster_id(i), left(PairPoint()),
//   right(PairPoint())
//   {} ~NodePoint() = default;

//   bool set_friend(const bool is_start, const PairPoint& target) {
//     if (target.cluster_id == cluster_id || !target.inited())
//       return false;  // cannot set my self
//     if (is_start) {
//       if (left.inited()) {
//         if (left == target) return true;  // already set and is same
//         return false;                     // already set
//       }
//       left = target;
//     } else {
//       if (right.inited()) {
//         if (right == target) return true;  // already set and is same
//         return false;                      // already set
//       }
//       right = target;
//     }

//     return true;
//   }

//   PairPoint next(const PairPoint& target) const {
//     if (target == left) return right;
//     if (target == right) return left;
//     return PairPoint();
//   }

//   bool is_single() const { return !left.inited() && !right.inited(); }
//   bool is_root() const { return !left.inited() || !right.inited(); }

//   PairPoint get_root() const {
//     if (left.inited()) return left;
//     if (right.inited()) return right;
//     return PairPoint();
//   }

//   friend std::ostream& operator<<(std::ostream& os, const NodePoint& node)
//   {
//     os << "cluster_id: " << node.cluster_id << std::endl
//        << "left: " << node.left.cluster_id << ", " << node.left.point_id
//        << std::endl
//        << "right: " << node.right.cluster_id << ", " << node.right.point_id
//        << std::endl;
//     return os;
//   }
// };

double distance_value(const EndPoint& a, const EndPoint& b);
double distance_loss(const EndPoint& a, const EndPoint& b);
double angle_value(const EndPoint& a, const EndPoint& b);
double angle_loss(const EndPoint& a, const EndPoint& b);
double time_loss(const EndPoint& a, const EndPoint& b);
double connect_value(const EndPoint& a, const EndPoint& b);
double connect_loss(const EndPoint& a, const EndPoint& b);
double connect_loss(const double value);
double loss_function(const EndPoint& a, const EndPoint& b);
double hermite_interpolate_distance(const EndPoint& a, const EndPoint& b);
double hermite_interpolate_loss(const EndPoint& a, const EndPoint& b);
double hermite_interpolate_loss(const EndPoint& a, const EndPoint& b, const double value);
double distance_tangential(const EndPoint& a, const EndPoint& b);
double distance_lateral(const EndPoint& a, const EndPoint& b);
LaneCloudPtr hermite_interpolate_cloud(const EndPoint& a, const EndPoint& b);
void solve_km(const std::vector<EndPoint>& inputs,
              std::vector<LaneCloudPtr>& connect_cloud,
              std::vector<std::set<int>>& result);
}  // namespace smartlabel

#endif /* !defined(LOSS_H) */
