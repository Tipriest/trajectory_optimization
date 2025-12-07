#pragma once

#include "base_path_searcher.h"


class AstarSearcher : public BasePathSearcher {
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