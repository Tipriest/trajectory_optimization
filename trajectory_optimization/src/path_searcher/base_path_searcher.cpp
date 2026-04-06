#include "../../include/path_searcher/base_path_searcher.h"

using namespace std;
using namespace Eigen;

BasePathSearcher::BasePathSearcher(const ros::NodeHandle &nh,
                                   const std::string &grid_map_topic)
    : m_gridmap_interface(nh, grid_map_topic), m_nh(nh) {
  m_nh.param("start_mode", start_mode_, start_mode_);
  m_nh.param("goal_topic", goal_topic_, goal_topic_);
  m_nh.param("odom_topic", odom_topic_, odom_topic_);
  m_nh.param("path_topic", path_topic_, path_topic_);
  m_nh.param("marker_frame_id", marker_frame_id_, marker_frame_id_);
  m_nh.param("start_marker_r", start_marker_r_, start_marker_r_);
  m_nh.param("start_marker_g", start_marker_g_, start_marker_g_);
  m_nh.param("start_marker_b", start_marker_b_, start_marker_b_);
  m_nh.param("start_marker_a", start_marker_a_, start_marker_a_);
  m_nh.param("goal_marker_r", goal_marker_r_, goal_marker_r_);
  m_nh.param("goal_marker_g", goal_marker_g_, goal_marker_g_);
  m_nh.param("goal_marker_b", goal_marker_b_, goal_marker_b_);
  m_nh.param("goal_marker_a", goal_marker_a_, goal_marker_a_);
  m_nh.param("start_marker_scale", start_marker_scale_, start_marker_scale_);
  m_nh.param("goal_marker_scale", goal_marker_scale_, goal_marker_scale_);

  start_end_point_vis_publisher =
      m_nh.advertise<visualization_msgs::MarkerArray>("start_end_point", 1);
  start_end_point_subscriber = m_nh.subscribe(
      goal_topic_, 1, &BasePathSearcher::rcvPosCmdCallBack, this);
  odom_subscriber =
      m_nh.subscribe(odom_topic_, 1, &BasePathSearcher::rcvOdomCallBack, this);
  m_search_path_vis_publisher =
      m_nh.advertise<visualization_msgs::Marker>("search_path_vis", 1);
  m_search_path_publisher =
      m_nh.advertise<nav_msgs::Path>(path_topic_, 1, true);

  path_search_timer = m_nh.createTimer(
      ros::Duration(0.05),
      boost::bind(&BasePathSearcher::searchAndVisPathCB, this, _1));
  path_search_timer.start();
}

void BasePathSearcher::displayPath() {
  visualization_msgs::Marker _path_vis;

  removeDisplayPath();
  _path_vis.points.clear();

  _path_vis.header.frame_id = "world";
  _path_vis.header.stamp = ros::Time();
  _path_vis.ns = "trajectory_search";
  _path_vis.type = visualization_msgs::Marker::LINE_STRIP;
  _path_vis.action = visualization_msgs::Marker::ADD;

  _path_vis.pose.orientation.x = 0.0;
  _path_vis.pose.orientation.y = 0.0;
  _path_vis.pose.orientation.z = 0.0;
  _path_vis.pose.orientation.w = 1.0;

  _path_vis.color.a = 0.8;
  _path_vis.color.r = 0.6;
  _path_vis.color.g = 0.8;
  _path_vis.color.b = 0.2;

  _path_vis.scale.x = 0.2; // LINE_STRIP的形状只取决于scale.x

  _path_vis.id = 0;
  std::vector<Eigen::Vector3d> path = getPath();
  std::cout << "path.size()=" << path.size() << std::endl;
  nav_msgs::Path path_msg;
  path_msg.header.stamp = ros::Time::now();
  path_msg.header.frame_id = marker_frame_id_;
  for (size_t i = 0; i < path.size(); i++) {
    geometry_msgs::Point point;
    point.x = path[i](0);
    point.y = path[i](1);
    point.z = path[i](2);
    _path_vis.points.push_back(point);

    geometry_msgs::PoseStamped pose;
    pose.header = path_msg.header;
    pose.pose.position = point;
    pose.pose.orientation.w = 1.0;
    path_msg.poses.push_back(pose);
  }

  m_search_path_vis_publisher.publish(_path_vis);
  m_search_path_publisher.publish(path_msg);
}

void BasePathSearcher::removeDisplayPath() {
  visualization_msgs::Marker _path_vis;
  _path_vis.points.clear();
  _path_vis.header.frame_id = "world";
  _path_vis.header.stamp = ros::Time();
  _path_vis.ns = "trajectory_search";
  _path_vis.type = visualization_msgs::Marker::LINE_STRIP;
  _path_vis.action = visualization_msgs::Marker::DELETE;

  m_search_path_vis_publisher.publish(_path_vis);
}

void BasePathSearcher::rcvPosCmdCallBack(
    const geometry_msgs::PoseStamped &cmd) {
  if (start_mode_ == "odom_plus_click") {
    if (!has_odom_) {
      ROS_WARN_THROTTLE(1.0, "Waiting for odom to set start point.");
      return;
    }
    m_start_point = odom_pos_;
    m_end_point(0) = cmd.pose.position.x;
    m_end_point(1) = cmd.pose.position.y;
    m_end_point(2) = cmd.pose.position.z;
    has_goal_ = true;
    has_start_ = true;
  } else {
    if (update_time_ == 0) {
      start_temp_(0) = cmd.pose.position.x;
      start_temp_(1) = cmd.pose.position.y;
      start_temp_(2) = cmd.pose.position.z;
      update_time_ = 1;
      return;
    }
    m_start_point = start_temp_;
    m_end_point(0) = cmd.pose.position.x;
    m_end_point(1) = cmd.pose.position.y;
    m_end_point(2) = cmd.pose.position.z;
    update_time_ = 0;
    has_start_ = true;
    has_goal_ = true;
  }

  publishStartEndMarkers(true);
}

void BasePathSearcher::rcvOdomCallBack(const nav_msgs::Odometry &odom) {
  odom_pos_(0) = odom.pose.pose.position.x;
  odom_pos_(1) = odom.pose.pose.position.y;
  odom_pos_(2) = odom.pose.pose.position.z;
  has_odom_ = true;
  if (start_mode_ == "odom_plus_click") {
    m_start_point = odom_pos_;
    has_start_ = true;
    publishStartEndMarkers(has_goal_);
  }
}

void BasePathSearcher::publishStartEndMarkers(bool include_goal) {
  visualization_msgs::MarkerArray markerArray_vis;
  for (auto &marker_vis : markerArray_vis.markers) {
    marker_vis.action = visualization_msgs::Marker::DELETE;
  }
  start_end_point_vis_publisher.publish(markerArray_vis);

  markerArray_vis.markers.clear();
  visualization_msgs::Marker marker_vis;
  marker_vis.header.frame_id = marker_frame_id_;
  marker_vis.header.stamp = ros::Time::now();
  marker_vis.ns = "trajectory_search";
  marker_vis.type = visualization_msgs::Marker::SPHERE;
  marker_vis.action = visualization_msgs::Marker::ADD;
  marker_vis.pose.orientation.x = 0.0;
  marker_vis.pose.orientation.y = 0.0;
  marker_vis.pose.orientation.z = 0.0;
  marker_vis.pose.orientation.w = 1.0;
  marker_vis.color.a = start_marker_a_;
  marker_vis.color.r = start_marker_r_;
  marker_vis.color.g = start_marker_g_;
  marker_vis.color.b = start_marker_b_;
  marker_vis.scale.x = start_marker_scale_;
  marker_vis.scale.y = start_marker_scale_;
  marker_vis.scale.z = start_marker_scale_;
  marker_vis.id = 0;
  marker_vis.pose.position.x = m_start_point(0);
  marker_vis.pose.position.y = m_start_point(1);
  marker_vis.pose.position.z = m_start_point(2);
  markerArray_vis.markers.push_back(marker_vis);

  if (include_goal && has_goal_) {
    marker_vis.id = 1;
    marker_vis.color.a = goal_marker_a_;
    marker_vis.color.r = goal_marker_r_;
    marker_vis.color.g = goal_marker_g_;
    marker_vis.color.b = goal_marker_b_;
    marker_vis.scale.x = goal_marker_scale_;
    marker_vis.scale.y = goal_marker_scale_;
    marker_vis.scale.z = goal_marker_scale_;
    marker_vis.pose.position.x = m_end_point(0);
    marker_vis.pose.position.y = m_end_point(1);
    marker_vis.pose.position.z = m_end_point(2);
    markerArray_vis.markers.push_back(marker_vis);
  }

  start_end_point_vis_publisher.publish(markerArray_vis);
}

void BasePathSearcher::searchAndVisPathCB(const ros::TimerEvent &) {
  if (!m_gridmap_interface.hasReceivedGridMap()) {
    return;
  }
  if (start_mode_ == "odom_plus_click") {
    if (!has_odom_ || !has_goal_) {
      return;
    }
    m_start_point = odom_pos_;
    has_start_ = true;
  } else if (!has_start_ || !has_goal_) {
    return;
  }
  // 更新用于搜索的地图副本
  search_map = m_gridmap_interface.getGridMapCopy();
  // 让派生类把 search_map 转成它自己的数据结构
  updateMapDataStructure();
  // 让派生类根据 m_start_point / m_end_point 填充 m_path
  searchPath();
  // 基类统一可视化
  displayPath();
}