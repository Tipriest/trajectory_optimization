#include "grid_map_interface.h"

using namespace grid_map;

GridMapInterface::GridMapInterface(const ros::NodeHandle &nh,
                                   const std::string &grid_map_topic)
    : nh_(nh) {
  // 订阅原始 grid_map
  map_subscriber_ = nh_.subscribe<grid_map_msgs::GridMap>(
      grid_map_topic, 1, &GridMapInterface::gridMapCallback, this);

  // 发布处理后的 grid_map
  std::string output_topic = grid_map_topic + "_processed";
  map_publisher_ = nh_.advertise<grid_map_msgs::GridMap>(output_topic, 1, true);

  ROS_INFO_STREAM("GridMapInterface subscribed to "
                  << grid_map_topic << " and publishing processed map to "
                  << output_topic);
}

void GridMapInterface::gridMapCallback(
    const grid_map_msgs::GridMapConstPtr &msg) {
  std::lock_guard<std::mutex> lock(map_mutex_);

  // ROS msg -> grid_map::GridMap
  GridMapRosConverter::fromMessage(*msg, grid_map_);

  // 记录基本参数（以完整 map 尺寸为例）
  length_ = grid_map_.getLength().x();     // x 方向长度
  width_ = grid_map_.getLength().y();      // y 方向长度
  resolution_ = grid_map_.getResolution(); // 栅格分辨率
  map_start_pos_x_ = grid_map_.getPosition().x();
  map_start_pos_y_ = grid_map_.getPosition().y();

  // 左下角偏移（相对于 map 中心）
  leftdown_offset_x_ = map_start_pos_x_ - length_ / 2.0;
  leftdown_offset_y_ = map_start_pos_y_ - width_ / 2.0;

  // 记录所有图层名称
  layers_ = grid_map_.getLayers();

  has_received_gridmap_ = true;

  // 示例：在回调里直接做一次简单处理并发布
  // 你也可以选择外部周期性调用 processAndPublish()
  // publish();
}

grid_map::GridMap GridMapInterface::getGridMapCopy() const {
  std::lock_guard<std::mutex> lock(map_mutex_);
  return grid_map_;
}

// 将一个 layer 中的 NaN 填成给定值，避免运算时出问题
void GridMapInterface::fillNaNWithValue(const std::string &layer, float value) {
  if (!grid_map_.exists(layer)) {
    ROS_WARN_STREAM("Layer " << layer << " does not exist, skip fillNaN.");
    return;
  }

  auto &data = grid_map_[layer]; // Eigen::MatrixXf
  for (int i = 0; i < data.rows(); ++i) {
    for (int j = 0; j < data.cols(); ++j) {
      if (!std::isfinite(data(i, j))) {
        data(i, j) = value;
      }
    }
  }
}

// 简单示例：对某个 occupancy 图层做“膨胀”，阈值以上认为是占用
void GridMapInterface::dilateOccupied(const std::string &layer, float threshold,
                                      int radius) {
  if (!grid_map_.exists(layer)) {
    ROS_WARN_STREAM("Layer " << layer << " does not exist, skip dilate.");
    return;
  }
  if (radius <= 0)
    return;

  const auto &data = grid_map_[layer];
  GridMap tmpMap = grid_map_;
  auto &out = tmpMap[layer];

  out = data; // 先拷贝

  for (GridMapIterator it(grid_map_); !it.isPastEnd(); ++it) {
    const Index index(*it);
    float v = data(index(0), index(1));
    if (!std::isfinite(v) || v < threshold) {
      continue;
    }

    // 对周围半径内的格子做膨胀
    for (int dx = -radius; dx <= radius; ++dx) {
      for (int dy = -radius; dy <= radius; ++dy) {
        Index n(index(0) + dx, index(1) + dy);
        if (!grid_map_.isValid(n))
          continue;
        out(n(0), n(1)) = std::max(out(n(0), n(1)), v);
      }
    }
  }

  // 用膨胀后的图层替换
  grid_map_[layer] = out;
}

// 裁剪到给定 polygon 区域之外置为 NaN(示例)
void GridMapInterface::cropToPolygon(const geometry_msgs::Polygon &polygon) {
  if (polygon.points.empty())
    return;

  // 将 polygon 转为 grid_map::Polygon
  grid_map::Polygon gmPoly;
  for (const auto &p : polygon.points) {
    gmPoly.addVertex(Position(p.x, p.y));
  }

  // 遍历所有 cell，若不在 polygon 内则置 NaN
  for (auto layer : layers_) {
    auto &data = grid_map_[layer];
    for (GridMapIterator it(grid_map_); !it.isPastEnd(); ++it) {
      const Index index(*it);
      Position pos;
      if (!grid_map_.getPosition(index, pos)) {
        data(index(0), index(1)) = NAN;
        continue;
      }
      if (!gmPoly.isInside(pos)) {
        data(index(0), index(1)) = NAN;
      }
    }
  }
}

void GridMapInterface::processAndPublish() {
  std::lock_guard<std::mutex> lock(map_mutex_);
  if (!has_received_gridmap_)
    return;

  // 这里写你希望对 map 做的处理流程
  // 示例：
  // 1. 对名为 "elevation" 的图层填充 NaN
  if (grid_map_.exists("elevation")) {
    fillNaNWithValue("elevation", 0.0f);
  }

  // 2. 对 "occupancy" 图层进行简单膨胀
  if (grid_map_.exists("occupancy")) {
    dilateOccupied("occupancy", 0.5f, 1); // threshold=0.5, radius=1
  }

  // 3. 若需要，只裁剪到一定区域（这里只是示例，不实际调用）
  // geometry_msgs::Polygon poly;
  // ... 填充 poly ...
  // cropToPolygon(poly);

  // 发布处理后的 grid map
  grid_map_msgs::GridMap msg;
  GridMapRosConverter::toMessage(grid_map_, msg);
  map_publisher_.publish(msg);
}

void GridMapInterface::publish() {
  std::lock_guard<std::mutex> lock(map_mutex_);
  if (!has_received_gridmap_)
    return;

  // 发布处理后的 grid map
  grid_map_msgs::GridMap msg;
  GridMapRosConverter::toMessage(grid_map_, msg);
  map_publisher_.publish(msg);
}