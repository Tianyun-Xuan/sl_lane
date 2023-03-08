#pragma once
#define BOOST_MPL_CFG_NO_PREPROCESSED_HEADERS
#define BOOST_MPL_LIMIT_VECTOR_SIZE 30  // or whatever you need
#define BOOST_MPL_LIMIT_MAP_SIZE 30     // or whatever you need
#define PCL_NO_PRECOMPILE
#include <pcl/io/pcd_io.h>
#include <pcl/pcl_macros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "boost/mpl/vector/vector30.hpp"

struct EIGEN_ALIGN16 Point_Auto_Label {
  PCL_ADD_POINT4D;
  // Original Position
  float original_x;
  float original_y;
  float original_z;
  // Index
  std::uint32_t frame_index;
  std::uint32_t point_index;
  double timestamp;
  float intensity;
  float elongation;
  std::uint32_t flags;
  std::uint32_t scan_id;
  std::uint32_t scan_idx;
  // Foreground or Background
  std::uint32_t motion_type;
  float motion_type_confidence;
  std::uint32_t ground_flag;
  // label and confidence
  std::uint32_t semantic_3d_label;
  float semantic_3d_confidence;
  std::uint32_t semantic_2d_label;
  float semantic_2d_confidence;
  std::uint32_t detection_3d_label;
  float detection_3d_confidence;
  std::uint32_t detection_2d_label;
  float detection_2d_confidence;
  // relabel and confidence
  std::uint32_t relabel;
  float relabel_confidence;
  std::uint32_t relabel_2nd;
  float relabel_2nd_confidence;
  // Reserved
  std::int32_t reserved;
  // Aligned
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // PCL_MAKE_ALIGNED_OPERATOR_NEW  // pcl 1.12
};

struct EIGEN_ALIGN16 Point_Lane {
  PCL_ADD_POINT4D;
  float intensity;
  float t;
  // Aligned
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // PCL_MAKE_ALIGNED_OPERATOR_NEW  // pcl 1.12
};

namespace pcl {
struct EIGEN_ALIGN16 Point_Mapping {
  PCL_ADD_POINT4D;
  // Original Position

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
}  // namespace pcl

namespace pcl {

struct EIGEN_ALIGN16 PointXYZTIFES {
  PCL_ADD_POINT4D;
  // PCL_ADD_UNION_NORMAL4D // 增加一个normal部分，和后续的算法适配
  float original_x;
  float original_y;
  float original_z;
  double timestamp;
  std::uint16_t intensity;
  std::uint16_t flags;
  std::uint16_t elongation;
  std::uint16_t scan_id;
  std::uint16_t scan_idx;
  std::uint16_t point_type;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
}  // namespace pcl

struct EIGEN_ALIGN16 Point_Raw {
  PCL_ADD_POINT4D;
  double timestamp;
  std::uint16_t intensity;
  std::uint8_t flags;
  std::uint8_t elongation;
  std::uint16_t scan_id;
  std::uint16_t scan_idx;
  std::uint8_t is_2nd_return;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // PCL_MAKE_ALIGNED_OPERATOR_NEW  // pcl 1.12
};

struct EIGEN_ALIGN16 Point_Output_V01 {
  PCL_ADD_POINT4D;
  std::uint32_t dt_label;
  float dtlabel_confidence;
  double timestamp;
  std::uint16_t intensity;
  std::uint8_t flags;
  std::uint8_t elongation;
  std::uint16_t scan_id;
  std::uint16_t scan_idx;
  std::uint8_t is_2nd_return;
  std::uint32_t line_group_ID;
  std::uint32_t Line_ID;
  std::uint32_t line_type;
  std::uint32_t line_status;
  float x_map;
  float y_map;
  float z_map;
  // Aligned
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // PCL_MAKE_ALIGNED_OPERATOR_NEW  // pcl 1.12
};

struct EIGEN_ALIGN16 Point_Seg {
  PCL_ADD_POINT4D;
  double timestamp;
  std::uint16_t intensity;
  std::uint8_t line_flag;
  std::uint8_t flags;
  std::uint16_t elongation;
  std::uint16_t scan_id;
  std::uint8_t scan_idx;
  std::uint32_t is_2nd_return;
  std::uint32_t dt_label;
  std::int32_t line_group_ID;
  std::uint32_t Line_ID;
  std::uint32_t line_type;
  std::uint32_t line_status;
  // Aligned
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // PCL_MAKE_ALIGNED_OPERATOR_NEW  // pcl 1.12
};

struct EIGEN_ALIGN16 Point_label_verification {
  PCL_ADD_POINT4D;
  std::uint32_t dt_label;
  float dtlabel_confidence;
  // Aligned
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // PCL_MAKE_ALIGNED_OPERATOR_NEW  // pcl 1.12
};

// clang-format off
POINT_CLOUD_REGISTER_POINT_STRUCT(
    Point_Auto_Label,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, original_x, original_x)
    (float, original_y, original_y)
    (float, original_z, original_z)
    (std::uint32_t, frame_index, frame_index)
    (std::uint32_t, point_index, point_index)
    (double, timestamp, timestamp)
    (float, intensity, intensity)
    (float, elongation, elongation)
    (std::uint32_t, flags, flags)
    (std::uint32_t, scan_id, scan_id)
    (std::uint32_t, scan_idx, scan_idx)
    (std::uint32_t, motion_type, motion_type)
    (float, motion_type_confidence, motion_type_confidence)
    (std::uint32_t, ground_flag, ground_flag)
    (std::uint32_t, semantic_3d_label, semantic_3d_label)
    (float, semantic_3d_confidence, semantic_3d_confidence)
    (std::uint32_t, semantic_2d_label, semantic_2d_label)
    (float, semantic_2d_confidence, semantic_2d_confidence)
    (std::uint32_t, detection_3d_label, detection_3d_label)
    (float, detection_3d_confidence, detection_3d_confidence)
    (std::uint32_t, detection_2d_label, detection_2d_label)
    (float, detection_2d_confidence, detection_2d_confidence)
    (std::uint32_t, relabel, relabel)
    (float, relabel_confidence,relabel_confidence)
    (std::uint32_t, relabel_2nd, relabel_2nd)
    (float, relabel_2nd_confidence,relabel_2nd_confidence)
    (std::int32_t, reserved, reserved)
)

POINT_CLOUD_REGISTER_POINT_STRUCT(
    Point_Mapping,
    (float, x, x)
    (float, y, y)
    (float, z, z)    
)

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZTIFES,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (float, original_x, original_x) 
                                  (float, original_y, original_y)
                                  (float, original_z, original_z)
                                  (double, timestamp, timestamp)
                                  (std::uint16_t, intensity, intensity)
                                  (std::uint16_t, flags, flags)
                                  (std::uint16_t, elongation, elongation)
                                  (std::uint16_t, scan_id, scan_id)
                                  (std::uint16_t, scan_idx, scan_idx)
                                  (std::uint16_t, point_type, point_type))

POINT_CLOUD_REGISTER_POINT_STRUCT(
  Point_Raw,
  (float, x, x)
  (float, y, y)
  (float, z, z)
  (double, timestamp, timestamp)
  (std::uint16_t, intensity, intensity)
  (std::uint8_t, flags, flags)
  (std::uint8_t, elongation, elongation)
  (std::uint16_t, scan_id, scan_id)
  (std::uint16_t, scan_idx, scan_idx)
  (std::uint8_t, is_2nd_return, is_2nd_return)
)

POINT_CLOUD_REGISTER_POINT_STRUCT(
  Point_Output_V01,
  (float, x, x)
  (float, y, y)
  (float, z, z)
  (std::uint32_t, dt_label, dt_label)
  (float, dtlabel_confidence, dtlabel_confidence)
  (double, timestamp, timestamp)
  (std::uint16_t, intensity, intensity)
  (std::uint8_t, flags, flags)
  (std::uint8_t, elongation, elongation)
  (std::uint16_t, scan_id, scan_id)
  (std::uint16_t, scan_idx, scan_idx)
  (std::uint8_t, is_2nd_return, is_2nd_return)
  (std::uint32_t, line_group_ID, line_group_ID)
  (std::uint32_t, Line_ID, Line_ID)
  (std::uint32_t, line_type, line_type)
  (std::uint32_t, line_status, line_status)
  (float, x_map, x_map)
  (float, y_map, y_map)
  (float, z_map, z_map)
)

POINT_CLOUD_REGISTER_POINT_STRUCT(
  Point_label_verification,
  (float, x, x)
  (float, y, y)
  (float, z, z)
  (std::uint32_t, dt_label, dt_label)
  (float, dtlabel_confidence, dtlabel_confidence)
)

POINT_CLOUD_REGISTER_POINT_STRUCT (
  Point_Seg,
  (float, x, x)
  (float, y, y)
  (float, z, z)
  (double,timestamp, timestamp)
  (std::uint16_t, intensity,intensity)
  (std::uint8_t, line_flag,line_flag)
  (std::uint8_t, flags,flags)
  (std::uint16_t, elongation,elongation)
  (std::uint16_t, scan_id,scan_id)
  (std::uint8_t, scan_idx,scan_idx)
  (std::uint32_t, is_2nd_return,is_2nd_return)
  (std::uint32_t, dt_label,dt_label)
  (std::int32_t, line_group_ID,line_group_ID)
  (std::uint32_t, Line_ID,Line_ID)
  (std::uint32_t, line_type,line_type)
  (std::uint32_t, line_status,line_status)
)

POINT_CLOUD_REGISTER_POINT_STRUCT (
  Point_Lane,
  (float, x, x)
  (float, y, y)
  (float, z, z)
  (float, intensity,intensity)
  (float, t, t)
)