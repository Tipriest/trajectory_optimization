#pragma once

#include "backward.hpp"
#include "data_type.h"
#include "grid_map.h"

#include "geometry_msgs/PoseStamped.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"

#include <Eigen/Dense>
#include <Eigen/Eigen>

#include <iostream>
#include <ros/console.h>
#include <ros/ros.h>

class PathSearcher {
public:
  int m_GLX_SIZE, m_GLY_SIZE, m_GLZ_SIZE; //全局长度
  int m_LX_SIZE, m_LY_SIZE, m_LZ_SIZE;    //局部长度
  double m_resolution, m_inv_resolution;  //分辨率
  double m_gl_xl, m_gl_yl, m_gl_zl;       //地图左下角位置偏置

  ros::NodeHandle m_nh;
  // TODO: 想要在这先Initialize一下这两个Eigen库的变量
  ros::Subscriber start_end_point_subscriber; //接收开始点，结束点
  ros::Publisher start_end_point_vis_publisher;

  ros::Timer path_search_timer;

  // 析构函数
  virtual ~PathSearcher() { removeDisplayPath(); }

  // 纯虚函数：搜索路径
  virtual void searchPath(Eigen::Vector3d start_pt, Eigen::Vector3d end_pt) = 0;

  // 函数：显示路径
  visualization_msgs::Marker m_path_vis;
  ros::Publisher m_search_path_publisher;

  virtual std::vector<Eigen::Vector3d> getPath() = 0;
  void displayPath();
  void removeDisplayPath();
};
class AstarSearcher : public PathSearcher {
private:
  Eigen::Vector3d gridIndex2coord(Eigen::Vector3i index);
  Eigen::Vector3i coord2gridIndex(Eigen::Vector3d pt);
  GridNodePtr pos2gridNodePtr(Eigen::Vector3d pos);

  double getDiagHeu(GridNodePtr node1, GridNodePtr node2);
  double getManhHeu(GridNodePtr node1, GridNodePtr node2);
  double getEuclHeu(GridNodePtr node1, GridNodePtr node2);
  double getHeu(GridNodePtr node1, GridNodePtr node2);

  std::vector<GridNodePtr> retrievePath(GridNodePtr current);

  std::vector<GridNodePtr> expandedNodes;
  std::vector<GridNodePtr> gridPath;
  GridNodePtr ***GridNodeMap;
  std::multimap<double, GridNodePtr> openSet;

  const std::shared_ptr<GridMapGenerator> m_grid_map_genertaor_ptr;
  grid_map::GridMap m_grid_map;

  double m_tie_breaker =
      1.0 + 1.0 / 10000; //设置算法倾向性，避免启发式函数效果接近的问题

public:
  Eigen::Vector3d start_point; //搜索开始点
  Eigen::Vector3d end_point;   //搜索结束点

  AstarSearcher(ros::NodeHandle nh,
                std::shared_ptr<GridMapGenerator> gridmap_generator);
  ~AstarSearcher() { removeDisplayPath(); };

  void initGridNodeMap();
  // void linkLocalMap(sdf_tools::CollisionMapGrid *local_map,
  // Eigen::Vector3d xyz_l);
  void searchPath(Eigen::Vector3d start_pt, Eigen::Vector3d end_pt);

  void resetLocalMap();
  void resetGlobalMap();
  void resetPath();
  void rcvPosCmdCallBack(const geometry_msgs::PoseStamped &cmd);
  void searchAndVisPathCB(const ros::TimerEvent &e);
  std::vector<Eigen::Vector3d> getPath();
  std::vector<GridNodePtr> getVisitedNodes();
};