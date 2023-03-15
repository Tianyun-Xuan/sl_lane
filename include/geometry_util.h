#pragma once
// clang-format off
#include "point_type.h"
// clang-format on
#include <pcl/ModelCoefficients.h>
#include <pcl/common/centroid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/search/kdtree.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <opencv/cv.hpp>

typedef pcl::PointCloud<Point_Auto_Label> SLCloud;
typedef pcl::PointCloud<Point_Auto_Label>::Ptr SLCloudPtr;
typedef pcl::PointCloud<Point_Lane> LaneCloud;
typedef pcl::PointCloud<Point_Lane>::Ptr LaneCloudPtr;

namespace smartlabel {

struct Polynomia {
  double a;
  double b;
  double c;
  double d;
  Eigen::Vector4f centroid;
  // y = a * x + b * x^2 + c * x^3 + d
  double y(double x) { return d + a * x + b * x * x + c * x * x * x; }
};

struct Hoff {
  double s;
  double d;  // global coordinate
  double r;
  double tx;
  double ty;
  Hoff() = default;
  // y = slope * x + intercept
  // t = tx *x + ty *y
  Hoff(double slope, double intercept)
      : s(slope), d(intercept), r(atan(slope)) {
    tx = 1.f / sqrt(1 + slope * slope);
    ty = slope / sqrt(1 + slope * slope);
  }
  Hoff(const Hoff &other)
      : s(other.s), d(other.d), r(other.r), tx(other.tx), ty(other.ty) {}
};

float original_dst(const Point_Auto_Label &point);
void project_point(const Point_Lane &src_point,
                   const pcl::ModelCoefficients::Ptr &coeff,
                   Point_Lane &dst_point);
bool fit_line_3d(const LaneCloudPtr &cloud, pcl::ModelCoefficients::Ptr &coeff);
float accumulate_distance(const LaneCloudPtr &cloud);
bool find_end_point(const LaneCloudPtr &source, Point_Lane &start_point,
                    Point_Lane &end_point);

struct DBscan_Parameters {
  // dbscan parameters
  double neighbour_radius = 0.2;
  size_t minPts_in_neighbour = 10;
  size_t min_pts_per_cluster = 40;
};

void dbscan(const DBscan_Parameters &parameters, const LaneCloudPtr &source,
            std::vector<LaneCloudPtr> &clusters);

void centerlize(const LaneCloudPtr &source, LaneCloudPtr &result,
                Eigen::Vector4f &centroid);
bool cloud_discretized(const LaneCloudPtr &source, float invterval, float tmin,
                       float tmax, LaneCloudPtr &result);
void FitLine1DByRegression(const LaneCloudPtr &source, Hoff &parameters,
                           Eigen::Vector4f &centroid);
void FitLine3DByRegression(const LaneCloudPtr &source, Polynomia &parameters);

void NormalLine(const LaneCloudPtr &source, const double radius,
                pcl::PointCloud<pcl::Normal>::Ptr &cloud_normals);

void TangentLine(const LaneCloudPtr &source, const double radius,
                 std::vector<pcl::ModelCoefficients::Ptr> &coefficients);

void hermite_interpolate_2points(const Point_Lane &p0, const Point_Lane &p1,
                                 float k0, float k1, float interval_x,
                                 LaneCloudPtr &dst);

}  // namespace smartlabel