#pragma once
#include <eigen3/Eigen/Dense>
#include <fstream>
#include <iostream>
#include <ros/console.h>
#include <ros/ros.h>
#include <stdio.h>
#include <string>
#include <vector>

#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Odometry.h"
#include "tf/tf.h"
#include "tf/transform_datatypes.h"
#include "bezier_generator3d.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"

#include "bezier_base.h"
#include "data_type.h"
#include "geometry_msgs/PoseArray.h"
#include "quadrotor_msgs/PolynomialTrajectory.h"
// class TestClass2D {
// public:
//   TestClass(ros::NodeHandle &h) {}
//   AddWayPoint();
//   ExpandCorridor();
//   SolveTrajectory();
//   Vis(bool wp, bool corridor; bool traj);

// }
// using namespace Eigen;
class TrajOptimize3D {
public:
  TrajOptimize3D(ros::NodeHandle &h);
  void addWayPoint(const Eigen::Vector3d &waypoint);
  void addWayPoints(const vector<Eigen::Vector3d> &waypoints);
  void expandCubeFromWp();
  void timeAllocation();
  void solveTrajectory();
  void getBezierTraj();
  Eigen::Vector3d getPosFromBezier(double t_now, int seg_now);
  void vis(bool wp, bool corridor, bool traj);
  void visWayPoints();
  void visCorridor();
  void visTraj();
  void visBezierTrajectory();
  // corridor & corrdior vis
  std::vector<Cube> m_cubes;
  visualization_msgs::MarkerArray m_cube_vis;
  ros::Publisher m_cubes_vis_pub;

  //
  std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>
      m_waypoints;
  visualization_msgs::MarkerArray m_waypoints_vis;
  ros::Publisher m_waypoints_vis_pub;

  // 描述优化后的轨迹
  int m_traj_order = 5; //多项式阶数
  int m_seg_num;        //段数
  double m_minimize_order = 2.5;
  double m_max_vel = 1.5;
  double m_max_acc = 0.5;
  Bernstein m_bernstein;
  MatrixXd m_MQM, m_FM;
  VectorXd m_C, m_Cv, m_Ca, m_Cj;

  ros::Publisher m_poly_traj_vis_pub;

  Eigen::MatrixXd m_bezier_coeff;
  VectorXd m_seg_time;
  quadrotor_msgs::PolynomialTrajectory m_bezier_traj;
  visualization_msgs::Marker m_bezier_traj_vis;
  ros::Publisher m_bezier_traj_vis_pub;
};