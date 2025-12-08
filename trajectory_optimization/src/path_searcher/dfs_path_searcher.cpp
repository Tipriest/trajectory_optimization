#include "../../include/path_searcher/dfs_path_searcher.h"

using namespace Eigen;

void DFSSearcher::updateMapDataStructure() {
  if (!search_map.exists("elevation") &&
      !search_map.exists("dynamic_obstacle")) {
    return;
  }

  resolution_ = search_map.getResolution();
  const auto length = search_map.getLength();
  const auto pos = search_map.getPosition();
  size_x_ = static_cast<int>(length.x() / resolution_);
  size_y_ = static_cast<int>(length.y() / resolution_);
  origin_x_ = pos.x() - length.x() / 2.0;
  origin_y_ = pos.y() - length.y() / 2.0;

  occupied_.assign(size_x_, std::vector<bool>(size_y_, false));

  const std::string layer = search_map.exists("dynamic_obstacle")
                                ? "dynamic_obstacle"
                                : search_map.getLayers().front();
  for (grid_map::GridMapIterator it(search_map); !it.isPastEnd(); ++it) {
    const grid_map::Index index(*it);
    grid_map::Position p;
    if (!search_map.getPosition(index, p))
      continue;
    int ix = static_cast<int>((p.x() - origin_x_) / resolution_);
    int iy = static_cast<int>((p.y() - origin_y_) / resolution_);
    if (ix < 0 || ix >= size_x_ || iy < 0 || iy >= size_y_)
      continue;

    float v1 = search_map.at("elevation", index);
    float v2 = search_map.at("dynamic_obstacle", index);
    // 阈值: > 0 认为是障碍
    if ((std::isfinite(v1) && v1 > 0.0f) || (std::isfinite(v2) && v2 > 0.0f))
      occupied_[ix][iy] = true;
  }
}

bool DFSSearcher::worldToGrid(const Eigen::Vector2d &pt, int &ix,
                              int &iy) const {
  if (resolution_ <= 0.0 || size_x_ <= 0 || size_y_ <= 0)
    return false;
  ix = static_cast<int>((pt.x() - origin_x_) / resolution_);
  iy = static_cast<int>((pt.y() - origin_y_) / resolution_);
  if (ix < 0 || ix >= size_x_ || iy < 0 || iy >= size_y_)
    return false;
  return true;
}

Eigen::Vector3d DFSSearcher::gridToWorld(int ix, int iy) const {
  double x = origin_x_ + (ix + 0.5) * resolution_;
  double y = origin_y_ + (iy + 0.5) * resolution_;
  return Eigen::Vector3d(x, y, 0.0);
}

bool DFSSearcher::dfs(int x, int y, int gx, int gy,
                      std::vector<std::vector<bool>> &visited,
                      std::vector<std::pair<int, int>> &path_cells) {
  if (x == gx && y == gy) {
    path_cells.emplace_back(x, y);
    return true;
  }
  visited[x][y] = true;
  path_cells.emplace_back(x, y);

  // 8 邻域：上下左右 + 对角
  const int dx[8] = {1, -1, 0, 0, 1, 1, -1, -1};
  const int dy[8] = {0, 0, 1, -1, 1, -1, 1, -1};

  for (int k = 0; k < 8; ++k) {
    int nx = x + dx[k];
    int ny = y + dy[k];
    if (nx < 0 || nx >= size_x_ || ny < 0 || ny >= size_y_)
      continue;
    if (visited[nx][ny] || occupied_[nx][ny])
      continue;
    if (dfs(nx, ny, gx, gy, visited, path_cells))
      return true;
  }
  // 回溯失败，弹出当前节点
  path_cells.pop_back();
  return false;
}

void DFSSearcher::searchPath() {
  m_path.clear();
  if (size_x_ <= 0 || size_y_ <= 0)
    return;

  int sx, sy, gx, gy;
  if (!worldToGrid(Eigen::Vector2d(m_start_point.x(), m_start_point.y()), sx,
                   sy))
    return;
  if (!worldToGrid(Eigen::Vector2d(m_end_point.x(), m_end_point.y()), gx, gy))
    return;
  if (occupied_[sx][sy] || occupied_[gx][gy])
    return;

  std::vector<std::vector<bool>> visited(size_x_,
                                         std::vector<bool>(size_y_, false));
  std::vector<std::pair<int, int>> cells;

  if (!dfs(sx, sy, gx, gy, visited, cells))
    return;

  std::vector<Eigen::Vector3d> pts;
  for (auto &c : cells) {
    pts.push_back(gridToWorld(c.first, c.second));
  }
  m_path = std::move(pts);
}
