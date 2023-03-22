#include <experimental/filesystem>
#include <fstream>
#include <memory>
#include <ostream>
#include <string>
#include <unordered_map>
#include <vector>

#include "benchmark.h"
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
  const std::string map_dir = "/home/demo/repos/sl_lane/data/frames/0001/";
  // clear and reserve
  std::vector<std::string> paths{};
  for (auto& item : fs::directory_iterator(map_dir)) {
    fs::path addr = item;
    if (addr.extension() == ".pcd") {
      fs::path filename = addr.stem();
      paths.emplace_back(filename);
    }
  }
  std::sort(paths.begin(), paths.end(),
            [](const std::string& a, const std::string& b) -> bool {
              return std::stoi(a) < std::stoi(b);
            });

  std::vector<SLCloudPtr> clouds{};

  for (const auto& filename : paths) {
    std::string frames_file = map_dir + "/" + filename + ".pcd";
    pcl::PointCloud<Point_Auto_Label>::Ptr cloud(
        new pcl::PointCloud<Point_Auto_Label>);
    pcl::io::loadPCDFile(frames_file, *cloud);
    clouds.push_back(cloud);
  }

  FitParameters params;
  std::vector<SLCloudPtr> result;
  LaneFitting lane_fitting(params);
  lane_fitting.fit(clouds, result);

  for (size_t i = 0; i < result.size(); ++i) {
    pcl::io::savePCDFileBinary(
        "/home/demo/repos/sl_lane/data/res/0001/" + std::to_string(i) + ".pcd",
        *result[i]);
  }
}

void test_km() {
  const std::string cluster_dir =
      "/home/demo/repos/sl_lane/data/unit_test/test-1/";

  // clear and reserve
  std::vector<std::string> paths{};
  for (auto& item : fs::directory_iterator(cluster_dir)) {
    fs::path addr = item;
    if (addr.extension() == ".pcd") {
      fs::path filename = addr.stem();
      paths.emplace_back(filename);
    }
  }
  std::sort(paths.begin(), paths.end(),
            [](const std::string& a, const std::string& b) -> bool {
              return std::stoi(a) < std::stoi(b);
            });

  std::vector<LaneCloudPtr> clouds{};

  for (const auto& filename : paths) {
    std::string frames_file = cluster_dir + "/" + filename + ".pcd";
    pcl::PointCloud<Point_Lane>::Ptr cloud(new pcl::PointCloud<Point_Lane>);
    pcl::io::loadPCDFile(frames_file, *cloud);
    clouds.push_back(cloud);
  }

  FitParameters params;
  LaneFitting lane_fitting(params);
  std::vector<LaneCluster> clusters;
  lane_fitting.initialize(clouds, clusters);

  std::vector<EndPoint> end_points;
  for (size_t i = 0; i < clusters.size(); ++i) {
    if (clusters[i].deplacement * clusters[i].ratio < 2.f) {
      continue;
    }
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

  for (size_t i = 0; i < connect_flags.size(); ++i) {
    std::cout << "Lane " << i << " : ";
    for (auto& item : connect_flags[i]) {
      std::cout << item << " ";
    }
    std::cout << std::endl;
  }

  for (size_t i = 0; i < connect_flags.size(); ++i) {
    pcl::PointCloud<Point_Lane>::Ptr cloud(new pcl::PointCloud<Point_Lane>);
    for (auto& item : connect_flags[i]) {
      *cloud += *clusters[item].cloud_smoothed;
    }
    *cloud += *connect_cloud[i];
    pcl::io::savePCDFileBinary(
        "/home/demo/repos/sl_lane/data/debug/" + std::to_string(i) + ".pcd",
        *cloud);
  }
}

void test_loss() {
  const std::string cloud_003_path =
      "/home/demo/repos/sl_lane/data/unit_test/003.pcd";
  const std::string cloud_005_path =
      "/home/demo/repos/sl_lane/data/unit_test/005.pcd";

  LaneCloudPtr cloud_003(new LaneCloud);
  LaneCloudPtr cloud_005(new LaneCloud);
  pcl::io::loadPCDFile(cloud_003_path, *cloud_003);
  pcl::io::loadPCDFile(cloud_005_path, *cloud_005);

  FitParameters params;
  LaneFitting lane_fitting(params);

  auto cluster_003 = lane_fitting.initialize(cloud_003);
  auto cluster_005 = lane_fitting.initialize(cloud_005);

  EndPoint start_point_003;
  start_point_003.point = cluster_003.start_point;
  start_point_003.tangent = cluster_003.start_point_tangent.normalized();
  start_point_003.cluster_id = 3;
  start_point_003.is_start = 1;

  EndPoint end_point_003;
  end_point_003.point = cluster_003.end_point;
  end_point_003.tangent = cluster_003.end_point_tangent.normalized();
  end_point_003.cluster_id = 3;
  end_point_003.is_start = 0;

  EndPoint start_point_005;
  start_point_005.point = cluster_005.start_point;
  start_point_005.tangent = cluster_005.start_point_tangent.normalized();
  start_point_005.cluster_id = 5;
  start_point_005.is_start = 1;

  EndPoint end_point_005;
  end_point_005.point = cluster_005.end_point;
  end_point_005.tangent = cluster_005.end_point_tangent.normalized();
  end_point_005.cluster_id = 5;
  end_point_005.is_start = 0;

  std::cout << angle_value(end_point_003, start_point_005) << std::endl;

  std::cout << "end_point_003" << end_point_003 << std::endl;
  std::cout << "start_point_005" << start_point_005 << std::endl;

  std::cout << "connect_loss a b "
            << connect_loss(end_point_003, start_point_005) << std::endl;
  std::cout << "connect_loss b a "
            << connect_loss(start_point_005, end_point_003) << std::endl;

  std::cout << hermite_interpolate_loss(end_point_003, start_point_005)
            << std::endl;
}

void test_mismatch() {
  const std::string cloud_002_path =
      "/home/demo/repos/sl_lane/data/unit_test/test-1/002.pcd";
  const std::string cloud_004_path =
      "/home/demo/repos/sl_lane/data/unit_test/test-1/004.pcd";
  const std::string cloud_006_path =
      "/home/demo/repos/sl_lane/data/unit_test/test-1/006.pcd";

  LaneCloudPtr cloud_002(new LaneCloud);
  LaneCloudPtr cloud_004(new LaneCloud);
  LaneCloudPtr cloud_006(new LaneCloud);
  pcl::io::loadPCDFile(cloud_002_path, *cloud_002);
  pcl::io::loadPCDFile(cloud_004_path, *cloud_004);
  pcl::io::loadPCDFile(cloud_006_path, *cloud_006);

  FitParameters params;
  LaneFitting lane_fitting(params);

  auto cluster_002 = lane_fitting.initialize(cloud_002);
  auto cluster_004 = lane_fitting.initialize(cloud_004);
  auto cluster_006 = lane_fitting.initialize(cloud_006);

  EndPoint end_point_002;
  end_point_002.point = cluster_002.end_point;
  end_point_002.tangent = cluster_002.end_point_tangent.normalized();
  end_point_002.cluster_id = 2;
  end_point_002.is_start = 0;

  EndPoint start_point_004;
  start_point_004.point = cluster_004.start_point;
  start_point_004.tangent = cluster_004.start_point_tangent.normalized();
  start_point_004.cluster_id = 4;
  start_point_004.is_start = 1;

  EndPoint start_point_006;
  start_point_006.point = cluster_006.start_point;
  start_point_006.tangent = cluster_006.start_point_tangent.normalized();
  start_point_006.cluster_id = 6;
  start_point_006.is_start = 1;

  std::cout << "002 - 004 loss" << std::endl
            << "C 2 - 4: " << connect_loss(end_point_002, start_point_004)
            << std::endl
            << "C 4 - 2: " << connect_loss(start_point_004, end_point_002)
            << std::endl
            << "Hermite: "
            << hermite_interpolate_loss(end_point_002, start_point_004)
            << std::endl
            << "loss fuction: " << loss_function(end_point_002, start_point_004)
            << std::endl;

  std::cout << "002 - 006 loss" << std::endl
            << "C 2 - 6: " << connect_loss(end_point_002, start_point_006)
            << std::endl
            << "C 6 - 2: " << connect_loss(start_point_006, end_point_002)
            << std::endl
            << "Hermite: "
            << hermite_interpolate_loss(end_point_002, start_point_006)
            << std::endl
            << "loss fuction: " << loss_function(end_point_002, start_point_006)
            << std::endl;
}

void test_end_point(const EndPoint& a, const EndPoint& b) {
  std::cout << "Loss: " << loss_function(a, b) << std::endl;
  std::cout << "HermiteLoss: " << hermite_interpolate_loss(a, b) << std::endl;
  std::cout << "ConnectLoss: " << connect_loss(a, b) << std::endl;
  std::cout << "ReverseConnectLoss: " << connect_loss(b, a) << std::endl;
  std::cout << "HermiteDistance: " << hermite_interpolate_distance(a, b)
            << std::endl;
  std::cout << "ConnectValue: " << connect_value(a, b) << std::endl;
  std::cout << "ReverseConnectValue: " << connect_value(b, a) << std::endl;
  std::cout << "Angle: " << angle_value(a, b) << std::endl;
  std::cout << "Distance: " << distance_value(a, b) << std::endl;

  std::cout << "Distance tangential: " << distance_tangential(a, b)
            << std::endl;
  std::cout << "Distance tangential: " << distance_tangential(b, a)
            << std::endl;
  std::cout << "Distance lateral: " << distance_lateral(a, b) << std::endl;
  std::cout << "Distance lateral: " << distance_lateral(b, a) << std::endl;
  std::cout << "-------------------------------------------------" << std::endl;
}

void test_connected_match() {
  const std::string cluster_dir =
      "/home/demo/repos/sl_lane/data/unit_test/test-3/";

  // clear and reserve
  std::vector<std::string> paths{};
  for (auto& item : fs::directory_iterator(cluster_dir)) {
    fs::path addr = item;
    if (addr.extension() == ".pcd") {
      fs::path filename = addr.stem();
      paths.emplace_back(filename);
    }
  }
  std::sort(paths.begin(), paths.end(),
            [](const std::string& a, const std::string& b) -> bool {
              return std::stoi(a) < std::stoi(b);
            });

  std::vector<LaneCloudPtr> clouds{};
  for (const auto& filename : paths) {
    std::string frames_file = cluster_dir + "/" + filename + ".pcd";
    pcl::PointCloud<Point_Lane>::Ptr cloud(new pcl::PointCloud<Point_Lane>);
    pcl::io::loadPCDFile(frames_file, *cloud);
    clouds.push_back(cloud);
  }

  FitParameters params;
  LaneFitting lane_fitting(params);

  std::vector<LaneCluster> clusters;
  for (const auto& cloud : clouds) {
    clusters.push_back(lane_fitting.initialize(cloud));
    pcl::io::savePCDFileBinary("/home/demo/repos/sl_lane/data/straight/" +
                                   std::to_string(clusters.size()) + ".pcd",
                               *clusters.back().cloud_smoothed);
  }

  std::vector<EndPoint> end_points;

  for (size_t i = 0; i < clusters.size(); ++i) {
    const auto& cluster = clusters[i];
    EndPoint start_point;
    start_point.point = cluster.start_point;
    start_point.tangent = cluster.start_point_tangent.normalized();
    start_point.cluster_id = i;
    end_points.push_back(start_point);

    EndPoint end_point;
    end_point.point = cluster.end_point;
    end_point.tangent = cluster.end_point_tangent.normalized();
    end_point.cluster_id = i;
    end_points.push_back(end_point);
  }

  for (const auto& end_point : end_points) {
    std::cout << end_point << std::endl;
  }
  test_end_point(end_points[1], end_points[2]);
  test_end_point(end_points[1], end_points[4]);
}

void test_fit_curve() {
  const std::string cluster_dir = "/home/demo/repos/sl_lane/data/straight/";

  // clear and reserve
  std::vector<std::string> paths{};
  for (auto& item : fs::directory_iterator(cluster_dir)) {
    fs::path addr = item;
    if (addr.extension() == ".pcd") {
      fs::path filename = addr.stem();
      paths.emplace_back(filename);
    }
  }
  std::sort(paths.begin(), paths.end(),
            [](const std::string& a, const std::string& b) -> bool {
              return std::stoi(a) < std::stoi(b);
            });

  for (size_t i = 0; i < paths.size(); ++i) {
    std::string frames_file = cluster_dir + "/" + paths[i] + ".pcd";
    LaneCloudPtr cloud(new LaneCloud);
    pcl::io::loadPCDFile(frames_file, *cloud);
    LaneSegment segment(cloud);
    pcl::io::savePCDFileBinary(
        "/home/demo/repos/sl_lane/data/smooth/" + std::to_string(i) + ".pcd",
        *segment.cloud_smoothed);
  }
}

void test_hausdorff_distance() {
  const std::string cluster_dir = "/home/demo/repos/sl_lane/data/smooth/";
  // clear and reserve
  std::vector<std::string> paths{};
  for (auto& item : fs::directory_iterator(cluster_dir)) {
    fs::path addr = item;
    if (addr.extension() == ".pcd") {
      fs::path filename = addr.stem();
      paths.emplace_back(filename);
    }
  }
  std::sort(paths.begin(), paths.end(),
            [](const std::string& a, const std::string& b) -> bool {
              return std::stoi(a) < std::stoi(b);
            });

  std::vector<LaneCloudPtr> clouds{};
  for (const auto& filename : paths) {
    std::string frames_file = cluster_dir + "/" + filename + ".pcd";
    pcl::PointCloud<Point_Lane>::Ptr cloud(new pcl::PointCloud<Point_Lane>);
    pcl::io::loadPCDFile(frames_file, *cloud);
    clouds.push_back(cloud);
  }

  const size_t n = clouds.size();
  Matrix<double> cost(n, n);
  for (size_t i = 0; i < n; ++i) {
    for (size_t j = 0; j < n; ++j) {
      cost(i, j) = hausdorff_distance<Point_Lane>(clouds[i], clouds[j]);
    }
  }

  std::cout << cost << std::endl;
}

template <typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> load_pcd(
    const std::string& dir) {
  // clear and reserve
  std::vector<std::string> paths{};
  for (auto& item : fs::directory_iterator(dir)) {
    fs::path addr = item;
    if (addr.extension() == ".pcd") {
      fs::path filename = addr.stem();
      paths.emplace_back(filename);
    }
  }
  std::sort(paths.begin(), paths.end(),
            [](const std::string& a, const std::string& b) -> bool {
              return std::stoi(a) < std::stoi(b);
            });

  std::vector<SLCloudPtr> clouds{};
  for (const auto& filename : paths) {
    std::string frames_file = dir + "/" + filename + ".pcd";
    typename pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
    pcl::io::loadPCDFile(frames_file, *cloud);
    clouds.push_back(cloud);
  }

  return clouds;
}

void benchmark() {
  const std::string res_dir = "/home/demo/repos/sl_lane/data/res/0002";
  const std::string label_dir = "/home/demo/repos/sl_lane/data/Seg_binary";

  std::fstream fdescriptor;
  fdescriptor.open("/home/demo/repos/sl_lane/D0002.txt", std::ios::out);

  std::vector<std::string> paths;
  std::vector<std::string> labels;

  // get all pcd files
  for (auto& item : fs::directory_iterator(res_dir)) {
    fs::path addr = item;
    if (addr.extension() == ".pcd") {
      fs::path filename = addr.stem();
      paths.emplace_back(filename);
    }
  }
  std::sort(paths.begin(), paths.end(),
            [](const std::string& a, const std::string& b) -> bool {
              return std::stoi(a) < std::stoi(b);
            });

  // verification
  for (auto& item : fs::directory_iterator(label_dir)) {
    fs::path addr = item;
    if (addr.extension() == ".pcd") {
      fs::path filename = addr.stem();
      labels.emplace_back(filename);
    }
  }
  std::sort(labels.begin(), labels.end());

  std::cout << "paths size: " << paths.size() << std::endl;
  std::cout << "labels size: " << labels.size() << std::endl;

  // compare
  // 4 steps
  // 0-50, 50-100, 100-150, > 150
  PointCountMap counts(4);

  for (size_t i = 0; i < paths.size(); ++i) {
    std::string path = res_dir + "/" + paths[i] + ".pcd";
    std::string label = label_dir + "/" + labels[i] + ".pcd";

    // load pcd
    pcl::PointCloud<Point_Auto_Label>::Ptr cloud(
        new pcl::PointCloud<Point_Auto_Label>);
    pcl::io::loadPCDFile(path, *cloud);

    pcl::PointCloud<Point_Seg_binary>::Ptr label_cloud(
        new pcl::PointCloud<Point_Seg_binary>);
    pcl::io::loadPCDFile(label, *label_cloud);

    assert(cloud->size() == label_cloud->size());

    PointCountMap vs_params_temp(4);

    // compare
    for (size_t j = 0; j < cloud->size(); ++j) {
      double dis =
          sqrt(cloud->points[j].original_x * cloud->points[j].original_x +
               cloud->points[j].original_y * cloud->points[j].original_y +
               cloud->points[j].original_z * cloud->points[j].original_z);
      size_t index = dis / 50.f;
      index = index > 3 ? 3 : index;
      // TP
      if (cloud->points[j].relabel == 10 &&
          label_cloud->points[j].dt_label == 10) {
        vs_params_temp[index].tp += 1;
      }
      // FP
      else if (cloud->points[j].relabel == 10 &&
               label_cloud->points[j].dt_label != 10) {
        vs_params_temp[index].fp += 1;
      }
      // FN
      else if (cloud->points[j].relabel != 10 &&
               label_cloud->points[j].dt_label == 10) {
        vs_params_temp[index].fn += 1;
      }
    }

    // print each frame
    fdescriptor << paths[i] << std::endl;
    fdescriptor << vs_params_temp << std::endl;
    fdescriptor << "-------------------------------------------" << std::endl;
    counts += vs_params_temp;
  }

  // print total statistics for each interval
  for (size_t i = 0; i < 4; ++i) {
    fdescriptor << "Interval: " << i << std::endl;
    fdescriptor << "TP: " << counts[i].tp << std::endl;
    fdescriptor << "FP: " << counts[i].fp << std::endl;
    fdescriptor << "FN: " << counts[i].fn << std::endl;
    fdescriptor << "Precision: " << counts[i].precision() << std::endl;
    fdescriptor << "Recall: " << counts[i].recall() << std::endl;
    fdescriptor << "F1 score: " << counts[i].f1_score() << std::endl;
    fdescriptor << "Iou: " << counts[i].iou() << std::endl;
    fdescriptor << "-------------------------------------------" << std::endl;
  }

  // print total statistics
  auto total = counts.total();
  fdescriptor << "Total" << std::endl;
  fdescriptor << "TP: " << total.tp << std::endl;
  fdescriptor << "FP: " << total.fp << std::endl;
  fdescriptor << "FN: " << total.fn << std::endl;
  fdescriptor << "Precision: " << total.precision() << std::endl;
  fdescriptor << "Recall: " << total.recall() << std::endl;
  fdescriptor << "F1 score: " << total.f1_score() << std::endl;
  fdescriptor << "Iou: " << total.iou() << std::endl;
  fdescriptor << "-------------------------------------------" << std::endl;

  fdescriptor.close();
}

void temp_relabel() {
  auto clouds =
      load_pcd<Point_Auto_Label>("/home/demo/repos/sl_lane/data/res/0002");

  auto difei =
      load_pcd<Point_Auto_Label>("/home/demo/repos/sl_lane/data/free_space");

  for (size_t i = 0; i < clouds.size(); ++i) {
    for (size_t j = 0; j < clouds[i]->size(); ++j) {
      if (clouds[i]->points[j].relabel == 10) {
        difei[i]->points[j].is_laneline = true;
      } else {
        difei[i]->points[j].is_laneline = false;
      }
    }
  }

  for (size_t i = 0; i < clouds.size(); ++i) {
    std::string path =
        "/home/demo/repos/sl_lane/data/debug/" + std::to_string(i) + ".pcd";
    pcl::io::savePCDFileBinary(path, *difei[i]);
  }
}

int main(int argc, char* argv[]) {
  // test_FitLine1DByRegression();
  // test_cloud_discretized();
  // test_initialize();
  // test_km();
  // test_loss();
  // test_mismatch();
  // test_connected_match();
  func();
  // benchmark();
  // test_fit_curve();
  // test_hausdorff_distance();
  // temp_relabel();
}
