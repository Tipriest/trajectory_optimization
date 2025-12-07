#pragma once

#include <algorithm>
#include <mutex>
#include <random>
#include <string>
#include <vector>

#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Polygon.h>
#include <ros/ros.h>

#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_msgs/GridMap.h>
#include <grid_map_ros/grid_map_ros.hpp>

class GridMapInterface {
public:
  GridMapInterface(const ros::NodeHandle &nh,
                   const std::string &grid_map_topic);

  /// 是否已经接收到至少一帧 grid map
  bool hasReceivedGridMap() const { return has_received_gridmap_; }

  /// 获取当前内部保存的 grid map（拷贝）
  grid_map::GridMap getGridMapCopy() const;

  /// 执行一次处理并发布（也可以在回调中自动调用）
  void processAndPublish();

private:
  /// 订阅回调
  void gridMapCallback(const grid_map_msgs::GridMapConstPtr &msg);

  /// 一些简单处理函数示例
  void fillNaNWithValue(const std::string &layer, float value);
  void dilateOccupied(const std::string &layer, float threshold, int radius);
  void cropToPolygon(const geometry_msgs::Polygon &polygon);

  ros::NodeHandle nh_;
  ros::Subscriber map_subscriber_;
  ros::Publisher map_publisher_;

  // 内部 grid map
  grid_map::GridMap grid_map_;
  mutable std::mutex map_mutex_;

  // 状态标志
  bool has_received_gridmap_{false};

  // 基本参数
  double length_{-1.0};
  double width_{-1.0};
  double resolution_{-1.0};
  double map_start_pos_x_{-1.0};
  double map_start_pos_y_{-1.0};
  double leftdown_offset_x_{-1.0};
  double leftdown_offset_y_{-1.0};
  std::vector<std::string> layers_;
};