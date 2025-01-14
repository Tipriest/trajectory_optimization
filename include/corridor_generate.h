#pragma once
#include "grid_map.h"
#include "traj_search3d.h"
#include <Eigen/Dense>
#include <backward.hpp>
#include <decomp_ros_utils/data_ros_utils.h>
#include <decomp_util/ellipsoid_decomp.h>
#include <grid_map_core/grid_map_core.hpp>
#include <ros/ros.h>
#include <string>
#include <visualization_msgs/Marker.h>
class CorridorGenerator {
public:
  // 应该有一个search的ptr的进来
  CorridorGenerator(const ros::NodeHandle &nh,
                    std::shared_ptr<GridMapGenerator> gridmap_generator,
                    std::shared_ptr<AstarSearcher> gird_path_finder);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  double m_length;
  double m_width;
  double m_resolution;
  const std::shared_ptr<GridMapGenerator> m_grid_map_genertaor_ptr;
  const std::shared_ptr<AstarSearcher> m_grid_path_finder_ptr;

  // ros related
  ros::NodeHandle m_nh;
  ros::Publisher m_ellipsoid_array_pub;
  ros::Publisher m_polyhedron_array_pub;
  ros::Publisher m_discrete_path_pub;
  ros::Timer corridor_generate_timer;

  vec_E<Eigen::Vector2d> m_discrete_path;
  vec_E<Eigen::Vector2d> m_obs2d;
  EllipsoidDecomp2D m_decomp_util;
  vec_E<Ellipsoid2D> m_elliposid2d;
  vec_E<Polyhedron2D> m_polyhedron2d;

  void getDiscretePath(double interval = 0.5);
  void getObs();
  void generateCorridor();
  void displayCorridor();
  void displayDiscretePathPoint();
  void corridor_generate_timer_cb(const ros::TimerEvent &e);
};
