#include "backward.hpp"
#include "data_type.h"
#include "grid_map.h"

#include <Eigen/Eigen>
#include <iostream>
#include <ros/console.h>
#include <ros/ros.h>

class gridPathFinder {
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

  int m_GLX_SIZE, m_GLY_SIZE, m_GLZ_SIZE; //全局长度
  int LX_SIZE, LY_SIZE, LZ_SIZE;          //局部长度
  double m_resolution, m_inv_resolution;  //分辨率
  double m_gl_xl, m_gl_yl, m_gl_zl;       //地图左下角位置偏置
  double m_tie_breaker =
      1.0 + 1.0 / 10000; //设置算法倾向性，避免启发式函数效果接近的问题

public:
  gridPathFinder(std::shared_ptr<GridMapGenerator> gridmap_generator);
  ~gridPathFinder(){};

  void initGridNodeMap();
  // void linkLocalMap(sdf_tools::CollisionMapGrid *local_map,
  // Eigen::Vector3d xyz_l);
  void AstarSearch(Eigen::Vector3d start_pt, Eigen::Vector3d end_pt);

  void resetLocalMap();
  void resetGlobalMap();
  void resetPath();

  std::vector<Eigen::Vector3d> getPath();
  std::vector<GridNodePtr> getVisitedNodes();
};