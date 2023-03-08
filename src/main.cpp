#include <experimental/filesystem>
#include <fstream>
#include <memory>
#include <ostream>
#include <string>
#include <unordered_map>
#include <vector>

#include "lane.h"

// namespace bg = boost::geometry;
// typedef bg::model::point<double, 2, bg::cs::cartesian> point_t;
// typedef bg::model::linestring<point_t> linestring_t;
namespace fs = std::experimental::filesystem;

using namespace smartlabel;

void test_FitLine1DByRegression() {
  LaneCloudPtr src(new LaneCloud);
  pcl::io::loadPCDFile(
      "/home/demo/repos/sl_lane/data/clusters/pc_cluster_0.pcd", *src);
  LaneCloudPtr dst(new LaneCloud);
  Hoff hoff_params;
  Eigen::Vector4f centroid;
  FitLine1DByRegression(src, hoff_params, centroid);
  Point_Lane start_point;
  Point_Lane end_point;
  find_end_point(src, start_point, end_point);

  for (double x = start_point.x; x < end_point.x; x += 0.1) {
    Point_Lane point;
    point.x = x;
    point.y = hoff_params.s * x + hoff_params.d;
    point.z = centroid[2];
    point.intensity = 254;
    dst->push_back(point);
  }
  *src += *dst;
  pcl::io::savePCDFileASCII(
      "/home/demo/repos/sl_lane/data/debug/pc_cluster_0_fit.pcd", *src);
}

void test_cloud_discretized() {
  LaneCloudPtr src(new LaneCloud);
  pcl::io::loadPCDFile(
      "/home/demo/repos/sl_lane/data/clusters/pc_cluster_0.pcd", *src);
  Hoff hoff_params;
  Eigen::Vector4f centroid;
  FitLine1DByRegression(src, hoff_params, centroid);
  float t_min = std::numeric_limits<float>::max();
  float t_max = -(std::numeric_limits<float>::max)();
  for (auto& point : src->points) {
    point.t = (point.x - centroid[0]) * hoff_params.tx +
              (point.y - centroid[1]) * hoff_params.ty;
    t_max = std::max(point.t, t_max);
    t_min = std::min(point.t, t_min);
  }
  LaneCloudPtr dst(new LaneCloud);
  cloud_discretized(src, 0.2, t_min, t_max, dst);
  *src += *dst;
  pcl::io::savePCDFileASCII(
      "/home/demo/repos/sl_lane/data/debug/pc_discretized.pcd", *src);
}

void test_initialize() {
  FitParameters params;
  LaneFitting lane_fitting(params);
  std::vector<std::string> paths{};
  const std::string cluster_dir = "/home/demo/repos/sl_lane/data/clusters";
  for (auto& item : fs::directory_iterator(cluster_dir)) {
    fs::path addr = item;
    if (addr.extension() == ".pcd") {
      fs::path filename = addr.stem();
      paths.emplace_back(filename);
    }
    LaneCloudPtr tcloud(new LaneCloud);
    pcl::io::loadPCDFile(addr, *tcloud);
    auto lane_cluster = lane_fitting.initialize(tcloud);
    if (lane_cluster.cloud->size() != tcloud->size()) {
      std::cout << "Error" << item << std::endl;
    }
  }
}
void func() {
  const std::string map_dir = "/home/demo/repos/sl_lane/data/frames";

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

  std::vector<SLCloudPtr> clouds{};

  for (const auto& filename : paths) {
    std::string frames_file = map_dir + "/" + filename + ".pcd";
    pcl::PointCloud<Point_Auto_Label>::Ptr cloud(
        new pcl::PointCloud<Point_Auto_Label>);
    pcl::io::loadPCDFile(frames_file, *cloud);
    clouds.push_back(cloud);
  }

  FitParameters params;
  std::vector<LaneSegment> result;
  LaneFitting lane_fitting(params);
  lane_fitting.fit(clouds, result);
}

int main(int argc, char* argv[]) {
  // test_FitLine1DByRegression();
  // test_cloud_discretized();
  // test_initialize();
  func();
}
