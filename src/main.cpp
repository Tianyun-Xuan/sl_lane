// clang-format off
#include "point_type.h"
// clang-format on
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/search/kdtree.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/geometries.hpp>
#include <experimental/filesystem>
#include <fstream>
#include <memory>
#include <opencv/cv.hpp>
#include <ostream>
#include <string>
#include <unordered_map>
#include <vector>

namespace bg = boost::geometry;
typedef bg::model::point<double, 2, bg::cs::cartesian> point_t;
typedef bg::model::linestring<point_t> linestring_t;
namespace fs = std::experimental::filesystem;

typedef pcl::PointCloud<Point_Auto_Label>::Ptr CloudPtr;

struct Polynomia {
  double a;
  double b;
  double c;
  double d;
  // y^2 = a * x + b * x^2 + c * x^3 + d

  double y(double x) { return d + a * x + b * x * x + c * x * x * x; }
};

struct Hoff {
  double d;
  double r;
  Hoff() = default;
  Hoff(double slope, double deplacement) : d(deplacement), r(atan(slope)) {}
  Hoff(const Hoff& other) : d(other.d), r(other.r) {}
};

double dst(const Point_Auto_Label& point) {
  return sqrt(pow(point.original_x, 2) + pow(point.original_y, 2) +
              pow(point.original_z, 2));
}

void dbscan(const CloudPtr& source, std::vector<CloudPtr>& clusters) {
  clusters.clear();
#define UNPROCESSED 1
#define PROCESSING 2
#define PROCESSED 0
#define NOISE -1
  std::vector<int> nn_indices;
  std::vector<float> nn_distances;
  std::vector<int> types(source->points.size(), UNPROCESSED);

  pcl::KdTreeFLANN<Point_Auto_Label> kdtree_;
  kdtree_.setInputCloud(source);

  double eps = 0.2;
  size_t minPts = 10;
  size_t min_pts_per_cluster = 40;
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
      pcl::PointCloud<Point_Auto_Label>::Ptr tune(
          new pcl::PointCloud<Point_Auto_Label>);

      for (size_t j = 0; j < seed_queue.size(); ++j) {
        tune->push_back(source->points[seed_queue[j]]);
      }

      // pcl::io::savePCDFileBinary<Point_Auto_Label>(
      //     "/home/demo/repos/diff/cluster/" + std::to_string(index) + ".pcd",
      //     *tune);
      clusters.push_back(tune);
    }
  }
}

// double find_sommet(const std::vector<double>& inputs) {
//   double max_d = std::numeric_limits<double>::min();
//   double min_d = std::numeric_limits<double>::max();
//   const double lstep = 1;
//   for (const auto& value : inputs) {
//     max_d = std::max(value, max_d);
//     min_d = std::min(value, min_d);
//   }
//   const size_t length = (max_d - min_d) / lstep + 1;

//   std::vector<size_t> votes(length, 0);

//   for (const auto& value : inputs) {
//     const size_t vstep = (value - min_d) / lstep;
//     ++votes[vstep];
//   }

//   size_t largest_value = 0;
//   size_t largest_index = 0;
//   for (size_t i = 0; i < length; ++i) {
//     if (votes[i] > largest_value) {
//       largest_value = votes[i];
//       largest_index = i;
//     }
//   }

//   std::ofstream fdps;
//   fdps.open("/home/demo/repos/diff/fine.txt");
//   for (auto& value : votes) {
//     fdps << value << std::endl;
//   }
//   fdps.close();

//   return min_d + largest_index * lstep;
// }

// void find_finest_lane(const std::vector<CloudPtr>& clusters,
//                       std::vector<CloudPtr>& finest) {
//   finest.clear();
//   std::vector<double> mean_value;

//   for (const auto& cloud : clusters) {
//     double mean_y = 0.f;
//     const size_t psum = cloud->size();

//     for (const auto& point : cloud->points) {
//       mean_y += point.y / psum;
//     }
//     mean_value.emplace_back(mean_y);
//   }

//   double finest_y = find_sommet(mean_value);

//   const double threshold = 0.5;

//   for (size_t i = 0; i < clusters.size(); ++i) {
//     if (abs(mean_value[i] - finest_y) <= 0.1) {
//       finest.emplace_back(clusters[i]);
//     }
//   }
// }

std::vector<CloudPtr> lane() {
  const std::string seg_dir = "/home/demo/repos/sl_lane/Seg_binary";
  const std::string res_dir = "/home/demo/repos/sl_lane/pcd";
  const std::string diff_dir = "/home/demo/repos/sl_lane/diff";
  const std::string map_dir = "/home/demo/repos/sl_lane/frames";
  // clear and reserve
  std::vector<std::string> paths{};
  for (auto& item : fs::directory_iterator(map_dir)) {
    fs::path addr = item;
    if (addr.extension() == ".pcd") {
      fs::path filename = addr.stem();
      paths.emplace_back(filename);
    }
  }
  std::sort(paths.begin(), paths.end());

  // for (const auto& filename : paths) {
  //   std::string res_file = res_dir + "/" + filename + ".pcd";
  //   std::string seg_file = seg_dir + "/" + filename + ".pcd";
  //   pcl::PointCloud<Point_Auto_Label>::Ptr cloud(
  //       new pcl::PointCloud<Point_Auto_Label>);
  //   pcl::PointCloud<Point_Seg>::Ptr dtcloud(new pcl::PointCloud<Point_Seg>);

  //   pcl::io::loadPCDFile(res_file, *cloud);
  //   pcl::io::loadPCDFile(seg_file, *dtcloud);

  //   if (cloud->size() != dtcloud->size()) {
  //     continue;
  //   }

  //   for (size_t iter = 0; iter < cloud->size(); ++iter) {
  //     auto& res_point = cloud->points[iter];
  //     auto& seg_point = dtcloud->points[iter];
  //     if (res_point.relabel != seg_point.dt_label) {
  //       seg_point.Line_ID = res_point.relabel * 100 + seg_point.dt_label;
  //     } else {
  //       seg_point.Line_ID = 0;
  //     }
  //   }
  //   std::string diff_file = diff_dir + "/" + filename + ".pcd";
  //   pcl::io::savePCDFileBinary<Point_Seg>(diff_file, *dtcloud);
  // }

  pcl::PointCloud<Point_Auto_Label>::Ptr line(
      new pcl::PointCloud<Point_Auto_Label>);

  for (const auto& filename : paths) {
    std::string frames_file = map_dir + "/" + filename + ".pcd";
    pcl::PointCloud<Point_Auto_Label>::Ptr cloud(
        new pcl::PointCloud<Point_Auto_Label>);
    pcl::io::loadPCDFile(frames_file, *cloud);

    for (const auto& point : cloud->points) {
      if (point.motion_type == 2 && point.semantic_3d_label == 10 &&
          dst(point) < 100.f) {
        line->push_back(point);
      }
    }
  }
  std::vector<CloudPtr> clusters;
  dbscan(line, clusters);
  return clusters;

  // size_t largest_size = 0;
  // size_t largest_id = 0;
  // for (size_t i = 0; i < clusters.size(); ++i) {
  //   if (clusters[i]->size() > largest_size) {
  //     largest_size = clusters[i]->size();
  //     largest_id = i;
  //   }
  // }

  // pcl::PointCloud<Point_Auto_Label>::Ptr fine_line(
  //     new pcl::PointCloud<Point_Auto_Label>);
  // fine_line = clusters[largest_id];
  // pcl::io::savePCDFileBinary<Point_Auto_Label>("/home/demo/repos/diff/fine.pcd",
  //                                              *fine_line);
}

void downsample(const CloudPtr& src, const CloudPtr& res) {
  pcl::VoxelGrid<Point_Auto_Label> sor;
  sor.setInputCloud(src);
  sor.setLeafSize(0.1f, 1.f, 1.f);
  sor.filter(*res);
}

void FitLine3DByRegression(const CloudPtr& src, Polynomia& parameters) {
  // AX = B
  const int N = 4;
  // Construct A mat
  cv::Mat A = cv::Mat::zeros(N, N, CV_64FC1);

  for (int row = 0; row < A.rows; ++row) {
    for (int col = 0; col < A.cols; ++col) {
      for (size_t k = 0; k < src->size(); ++k) {
        A.at<double>(row, col) =
            A.at<double>(row, col) + pow(src->points[k].x, row + col);
      }
    }
  }

  //
  const double lambda = 1;
  for (int row = 0; row < A.rows; ++row) {
    for (int col = 0; col < A.cols; ++col) {
      A.at<double>(row, col) = A.at<double>(row, col) - lambda;
    }
  }

  // Construct B mat
  cv::Mat B = cv::Mat::zeros(N, 1, CV_64FC1);
  for (int row = 0; row < B.rows; row++) {
    for (size_t k = 0; k < src->size(); ++k) {
      B.at<double>(row, 0) =
          B.at<double>(row, 0) + pow(src->points[k].x, row) * src->points[k].y;
    }
  }
  // Solve the A*X = B
  cv::Mat X;
  cv::solve(A, B, X, cv::DECOMP_LU);
  // y = d + a * x + b * x^2 + c * x^3
  parameters.d = X.at<double>(0, 0);
  parameters.a = X.at<double>(1, 0);
  parameters.b = X.at<double>(2, 0);
  parameters.c = X.at<double>(3, 0);
}  // FitLine3DByRegression

void FitLine1DByRegression(const CloudPtr& src, Hoff& parameters) {
  // AX = B
  const int N = 2;
  // Construct A mat
  cv::Mat A = cv::Mat::zeros(N, N, CV_64FC1);
  for (int row = 0; row < A.rows; ++row) {
    for (int col = 0; col < A.cols; ++col) {
      for (size_t k = 0; k < src->size(); ++k) {
        A.at<double>(row, col) =
            A.at<double>(row, col) + pow(src->points[k].x, row + col);
      }
    }
  }
  // Normalization
  const double lambda = 0.f;
  for (int row = 0; row < A.rows; ++row) {
    for (int col = 0; col < A.cols; ++col) {
      A.at<double>(row, col) = A.at<double>(row, col) - lambda;
    }
  }
  // Construct B mat
  cv::Mat B = cv::Mat::zeros(N, 1, CV_64FC1);
  for (int row = 0; row < B.rows; row++) {
    for (size_t k = 0; k < src->size(); ++k) {
      B.at<double>(row, 0) =
          B.at<double>(row, 0) + pow(src->points[k].x, row) * src->points[k].y;
    }
  }
  // Solve the A*X = B
  cv::Mat X;
  cv::solve(A, B, X, cv::DECOMP_LU);
  // y = b + a * x
  parameters = Hoff(X.at<double>(1, 0), X.at<double>(0, 0));

}  // FitLine1DByRegression

void fitline() {
  pcl::PointCloud<Point_Auto_Label>::Ptr fine_line(
      new pcl::PointCloud<Point_Auto_Label>);
  pcl::io::loadPCDFile("/home/demo/repos/sl_lane/fine.pcd", *fine_line);
  pcl::PointCloud<Point_Auto_Label>::Ptr down(
      new pcl::PointCloud<Point_Auto_Label>);
  downsample(fine_line, down);
  pcl::io::savePCDFileBinary<Point_Auto_Label>("/home/demo/repos/sl_lane/down.pcd",
                                               *down);
  Polynomia result;
  FitLine3DByRegression(down, result);
  // std::cout << result.a << ", " << result.b << ", " << result.c << ", "
  //           << result.d << std::endl;

  pcl::PointCloud<pcl::PointXYZI>::Ptr fit(new pcl::PointCloud<pcl::PointXYZI>);
  for (double x = -1000.f; x < 1000.f; x = x + 0.5) {
    pcl::PointXYZI temp;
    temp.x = x;
    temp.y = result.y(x);
    fit->push_back(temp);
  }
  pcl::io::savePCDFileBinary<pcl::PointXYZI>("/home/demo/repos/sl_lane/fit.pcd",
                                             *fit);
}

void filter_cluster(const std::vector<CloudPtr>& clusters,
                    std::vector<CloudPtr>& fine) {
  fine.clear();
  std::vector<cv::Point2f> pointcloud_in_cv;
  pointcloud_in_cv.reserve(10000);
  for (auto& cluster : clusters) {
    for (auto& point : cluster->points) {
      cv::Point p;
      p.x = point.x * 10000;
      p.y = point.y * 10000;
      pointcloud_in_cv.emplace_back(p);
    }
    cv::RotatedRect rect = cv::minAreaRect(pointcloud_in_cv);
    float length = (rect.size.height > rect.size.width ? rect.size.height
                                                       : rect.size.width) /
                   10000.0;
    float width = (rect.size.height < rect.size.width ? rect.size.height
                                                      : rect.size.width) /
                  10000.0;
    if (length / width > 10.f) {
      fine.push_back(cluster);
    }
  }
}

int main(int argc, char* argv[]) {
  std::vector<CloudPtr> clusters = lane();
  std::vector<CloudPtr> fine_clusters;
  filter_cluster(clusters, fine_clusters);

  std::vector<Hoff> parameters;

  // hoff
  double min_r = std::numeric_limits<double>::max();
  double max_r = std::numeric_limits<double>::min();
  double min_d = std::numeric_limits<double>::max();
  double max_d = std::numeric_limits<double>::min();
  for (const auto& cluster : fine_clusters) {
    Hoff temp;
    FitLine1DByRegression(cluster, temp);
    parameters.emplace_back(temp);

    min_r = std::min(temp.r, min_r);
    max_r = std::max(temp.r, max_r);

    min_d = std::min(temp.d, min_d);
    max_d = std::max(temp.d, max_d);
  }

  std::cout << min_r << ", " << max_r << ", " << min_d << ", " << max_d
            << std::endl;

  const double rstep = M_PI / 180;
  const double dstep = 0.02;
  size_t rsize = (max_r - min_r) / rstep + 1;
  size_t dsize = (max_d - min_d) / dstep + 1;

  // std::vector<size_t> hoff_space(rsize * dsize, 0);
  std::unordered_map<size_t, CloudPtr> hoff_cluster;
  cv::Mat paint = cv::Mat::zeros(rsize + 1, dstep + 1, CV_8UC1);
  for (size_t hi = 0; hi < parameters.size(); ++hi) {
    size_t ridx = (parameters[hi].r - min_r) / rstep;
    size_t didx = (parameters[hi].d - min_d) / dstep;
    // hoff_space[ridx * dsize + didx] = hi;
    size_t hoff_id = ridx * dsize + didx;
    if (hoff_cluster.find(hoff_id) != hoff_cluster.end()) {
      for (auto& point : fine_clusters[hi]->points) {
        hoff_cluster[hoff_id]->push_back(point);
      }
    } else {
      hoff_cluster.insert({hoff_id, fine_clusters[hi]});
    }
    paint.at<int>(ridx, didx) = paint.at<int>(ridx, didx) + 1;
  }
  cv::Mat seeMat;
  cv::normalize(paint, seeMat, 0, 255, cv::NORM_MINMAX);
  seeMat.convertTo(seeMat, CV_8UC1);
  cv::imwrite("/home/demo/repos/sl_lane/hoff.tiff", seeMat);

  for (const auto& pair : hoff_cluster) {
    pcl::io::savePCDFileBinary<Point_Auto_Label>(
        "/home/demo/repos/sl_lane/hoff/" + std::to_string(pair.first) + ".pcd",
        *(pair.second));
  }
}
