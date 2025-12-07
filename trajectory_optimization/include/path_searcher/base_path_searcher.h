#pragma once

#include "../grid_map_interface.h"
#include "geometry_msgs/PoseStamped.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include <ros/ros.h>

class BasePathSearcher {
public:
  // 地图处理相关
  GridMapInterface m_gridmap_interface;

  // ROS相关
  ros::NodeHandle m_nh;
  ros::Subscriber start_end_point_subscriber; //接收开始点，结束点
  ros::Publisher start_end_point_vis_publisher;
  ros::Timer path_search_timer;
  void rcvPosCmdCallBack(const geometry_msgs::PoseStamped &cmd);

  ros::Publisher m_search_path_publisher;

  // 构造函数&析构函数
  BasePathSearcher(const ros::NodeHandle &nh,
                   const std::string &grid_map_topic);
  virtual ~BasePathSearcher() { removeDisplayPath(); }

  // 纯虚函数：搜索路径
  virtual void searchPath(Eigen::Vector3d start_pt, Eigen::Vector3d end_pt) = 0;
  virtual std::vector<Eigen::Vector3d> getPath() = 0;
  virtual std::vector<Eigen::Vector3d> searchAndVisPathCB() = 0;

  // 函数：发布路径
  void displayPath();
  void removeDisplayPath();

private:
  Eigen::Vector3d start_point, end_point;
};
