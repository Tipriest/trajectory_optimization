#include "../../include/path_searcher/base_path_searcher.h"

using namespace std;
using namespace Eigen;

BasePathSearcher::BasePathSearcher(const ros::NodeHandle &nh,
                                   const std::string &grid_map_topic)
    : m_gridmap_interface(nh, grid_map_topic) {
  start_end_point_vis_publisher =
      m_nh.advertise<visualization_msgs::MarkerArray>("start_end_point", 1);
  start_end_point_subscriber = m_nh.subscribe(
      "/move_base_simple/goal", 1, &BasePathSearcher::rcvPosCmdCallBack, this);
  m_search_path_publisher =
      m_nh.advertise<visualization_msgs::Marker>("search_path", 1);

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
  for (size_t i = 0; i < path.size(); i++) {
    geometry_msgs::Point point;
    point.x = path[i](0);
    point.y = path[i](1);
    point.z = path[i](2);
    _path_vis.points.push_back(point);
  }

  m_search_path_publisher.publish(_path_vis);
}

void BasePathSearcher::removeDisplayPath() {
  visualization_msgs::Marker _path_vis;
  _path_vis.points.clear();
  _path_vis.header.frame_id = "world";
  _path_vis.header.stamp = ros::Time();
  _path_vis.ns = "trajectory_search";
  _path_vis.type = visualization_msgs::Marker::LINE_STRIP;
  _path_vis.action = visualization_msgs::Marker::DELETE;

  m_search_path_publisher.publish(_path_vis);
}

void BasePathSearcher::rcvPosCmdCallBack(
    const geometry_msgs::PoseStamped &cmd) {
  static int update_time = 0;
  static Eigen::Vector3d start_temp;
  if (update_time == 0) {
    start_temp(0) = cmd.pose.position.x;
    start_temp(1) = cmd.pose.position.y;
    start_temp(2) = cmd.pose.position.z;
    update_time++;
  } else if (update_time == 1) {
    start_point(0) = start_temp(0);
    start_point(1) = start_temp(1);
    start_point(2) = start_temp(2);
    end_point(0) = cmd.pose.position.x;
    end_point(1) = cmd.pose.position.y;
    end_point(2) = cmd.pose.position.z;
    update_time = 0;

    visualization_msgs::MarkerArray markerArray_vis;
    for (auto &marker_vis : markerArray_vis.markers)
      marker_vis.action = visualization_msgs::Marker::DELETE;

    start_end_point_vis_publisher.publish(markerArray_vis);

    markerArray_vis.markers.clear();
    visualization_msgs::Marker marker_vis;
    marker_vis.header.frame_id = "world";
    marker_vis.header.stamp = ros::Time::now();
    marker_vis.ns = "trajectory_search";
    marker_vis.type = visualization_msgs::Marker::SPHERE;
    marker_vis.action = visualization_msgs::Marker::ADD;
    marker_vis.pose.orientation.x = 0.0;
    marker_vis.pose.orientation.y = 0.0;
    marker_vis.pose.orientation.z = 0.0;
    marker_vis.pose.orientation.w = 1.0;
    marker_vis.color.a = 1.0;
    marker_vis.color.r = 0.0;
    marker_vis.color.g = 1.0;
    marker_vis.color.b = 0.0;
    marker_vis.scale.x = 0.5;
    marker_vis.scale.y = 0.5;
    marker_vis.scale.z = 0.5;
    marker_vis.id = 0;
    marker_vis.pose.position.x = start_point(0);
    marker_vis.pose.position.y = start_point(1);
    marker_vis.pose.position.z = start_point(2);
    markerArray_vis.markers.push_back(marker_vis);

    marker_vis.id = 1;
    marker_vis.color.a = 1.0;
    marker_vis.color.r = 1.0;
    marker_vis.color.g = 0.0;
    marker_vis.color.b = 0.8;
    marker_vis.pose.position.x = end_point(0);
    marker_vis.pose.position.y = end_point(1);
    marker_vis.pose.position.z = end_point(2);
    markerArray_vis.markers.push_back(marker_vis);
    start_end_point_vis_publisher.publish(markerArray_vis);
  }
}
