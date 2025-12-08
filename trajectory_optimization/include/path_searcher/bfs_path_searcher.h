#pragma once

#include "base_path_searcher.h"
#include <queue>

class BFSSearcher : public BasePathSearcher {
public:
  BFSSearcher(const ros::NodeHandle &nh,
              const std::string &grid_map_topic = "grid_map")
      : BasePathSearcher(nh, grid_map_topic) {}

  void updateMapDataStructure() override;
  void searchPath() override;

private:
  // 简单 2D 网格占用图：true = 占用
  std::vector<std::vector<bool>> occupied_;
  double resolution_{0.0};
  double origin_x_{0.0}, origin_y_{0.0};
  int size_x_{0}, size_y_{0};

  bool worldToGrid(const Eigen::Vector2d &pt, int &ix, int &iy) const;
  Eigen::Vector3d gridToWorld(int ix, int iy) const;
};
