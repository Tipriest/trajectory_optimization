#pragma once

#include "../grid_map_interface.h"
#include "geometry_msgs/PoseStamped.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include <Eigen/Dense>
#include <ros/ros.h>

using namespace std;

class BasePathSearcher {
public:
  // 构造函数&析构函数
  BasePathSearcher(const ros::NodeHandle &nh,
                   const std::string &grid_map_topic = "grid_map");
  virtual ~BasePathSearcher() {}

  // 搜索路径: 使用类内的start_point和end_point（强制派生类实现）
  virtual void searchPath() = 0;

  // 如果派生类需要也可以实现这个重载
  virtual void searchPath(Eigen::Vector3d /*start_pt*/,
                          Eigen::Vector3d /*end_pt*/) {}

  // 从 GridMap 构建内部数据结构（可选）
  virtual void updateMapDataStructure() {}

  virtual std::vector<Eigen::Vector3d> getPath() { return m_path; }

  virtual void searchAndVisPathCB(const ros::TimerEvent &);

  // 函数：发布路径
  void displayPath();
  void removeDisplayPath();

  // 起点终点只读访问接口，便于派生类使用
  const Eigen::Vector3d &getStart() const { return m_start_point; }
  const Eigen::Vector3d &getEnd() const { return m_end_point; }

protected:
  // 由基类回调填充起终点，派生类可读
  Eigen::Vector3d m_start_point, m_end_point;
  // 由派生类搜索得到路径，基类负责可视化
  std::vector<Eigen::Vector3d> m_path;
  // 当前用于搜索的 GridMap 副本
  grid_map::GridMap search_map;

  // 地图处理相关
  GridMapInterface m_gridmap_interface;

  // ROS相关
  ros::NodeHandle m_nh;
  ros::Subscriber start_end_point_subscriber; // 接收开始点，结束点
  ros::Publisher start_end_point_vis_publisher;
  ros::Timer path_search_timer;
  ros::Publisher m_search_path_publisher;

  void rcvPosCmdCallBack(const geometry_msgs::PoseStamped &cmd);
};
