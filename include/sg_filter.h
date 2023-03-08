#pragma once
// clang-format off
#include "point_type.h"
// clang-format on
#include "geometry_util.h"

namespace smartlabel {

class SGFilter {
 public:
  bool setInputCloud(const LaneCloudPtr& source);
  inline void setRadius(float radius) { radius_ = radius; };
  void filter(const LaneCloudPtr& result);

 private:
  LaneCloudPtr src_;
  float radius_;
};

}  // namespace smartlabel