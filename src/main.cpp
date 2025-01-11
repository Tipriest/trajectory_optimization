#include "backward.hpp"
#include "grid_map.h"
#include "traj_optimize3d.h"
#include "traj_search3d.h"

namespace backward {
backward::SignalHandling sh;
}

Eigen::Vector3d startPoint(0.0, 0.0, 0.0), endPoint(0.0, 0.0, 0.0);
ros::Publisher start_end_point_vis_publisher;
ros::Publisher search_path_publisher;
std::shared_ptr<GridMapGenerator> grid_map_generator_ptr;
std::shared_ptr<GridPathFinder> grid_path_finder_ptr;

void rcvPosCmdCallBack(const geometry_msgs::PoseStamped cmd);
void publish_search_path(const std::vector<Eigen::Vector3d> &path);
void publishGridmap(const ros::TimerEvent &e);
void searchPath(const ros::TimerEvent &e);
int main(int argc, char **argv) {
  ros::init(argc, argv, "trajectory_optimization");
  ros::NodeHandle nh("~");
  ros::Subscriber start_end_point_subscriber =
      nh.subscribe("/move_base_simple/goal", 1, rcvPosCmdCallBack);
  start_end_point_vis_publisher =
      nh.advertise<visualization_msgs::MarkerArray>("start_end_point", 1);
  search_path_publisher =
      nh.advertise<visualization_msgs::MarkerArray>("search_path", 1);
  // 创建 GridMapGenerator 对象
  std::vector<std::string> global_map_layers = {"elevation",
                                                "dynamic_obstacle"};
  std::shared_ptr<grid_map::GridMap> global_map_ptr =
      std::make_shared<grid_map::GridMap>(global_map_layers);
  GridMapGenerator grid_map_generator(nh, global_map_ptr);
  grid_map_generator_ptr =
      std::make_shared<GridMapGenerator>(grid_map_generator);

  GridPathFinder grid_path_finder(grid_map_generator_ptr);
  grid_path_finder_ptr = std::make_shared<GridPathFinder>(grid_path_finder);
  // Timer
  ros::Timer m_map_publish_timer =
      nh.createTimer(ros::Duration(0.01), publishGridmap);
  m_map_publish_timer.start();

  // ros::Timer m_path_search_timer =
  //     nh.createTimer(ros::Duration(0.02), searchPath);
  // m_path_search_timer.start();
  ros::Rate rate(100);
  while (true) {

    // TrajOptimize3D testcase(nh);
    // Eigen::Vector3d point1, point2, point3, point4, point5;
    // std::vector<Eigen::Vector3d> waypoints = {Eigen::Vector3d(50, 50, 0), //
    //                                           Eigen::Vector3d(100, 120, 0),
    //                                           // Eigen::Vector3d(180, 150,
    //                                           0), // Eigen::Vector3d(250, 80,
    //                                           0),  // Eigen::Vector3d(280, 0,
    //                                           0)};
    // testcase.addWayPoints(waypoints);
    // testcase.expandCubeFromWp();
    // testcase.timeAllocation();
    // Cube cube_print = testcase.m_cubes[0];

    // testcase.solveTrajectory();
    // testcase.vis(true, true, true);
    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}
void rcvPosCmdCallBack(const geometry_msgs::PoseStamped cmd) {
  static int update_time = 0;
  static Eigen::Vector3d start_temp;
  if (update_time == 0) {
    start_temp(0) = cmd.pose.position.x;
    start_temp(1) = cmd.pose.position.y;
    start_temp(2) = cmd.pose.position.z;
    update_time++;
  } else if (update_time == 1) {
    startPoint(0) = start_temp(0);
    startPoint(1) = start_temp(1);
    startPoint(2) = start_temp(2);
    endPoint(0) = cmd.pose.position.x;
    endPoint(1) = cmd.pose.position.y;
    endPoint(2) = cmd.pose.position.z;
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
    marker_vis.pose.position.x = startPoint(0);
    marker_vis.pose.position.y = startPoint(1);
    marker_vis.pose.position.z = startPoint(2);
    markerArray_vis.markers.push_back(marker_vis);

    marker_vis.id = 1;
    marker_vis.color.a = 1.0;
    marker_vis.color.r = 1.0;
    marker_vis.color.g = 0.0;
    marker_vis.color.b = 0.8;
    marker_vis.pose.position.x = endPoint(0);
    marker_vis.pose.position.y = endPoint(1);
    marker_vis.pose.position.z = endPoint(2);
    markerArray_vis.markers.push_back(marker_vis);
    start_end_point_vis_publisher.publish(markerArray_vis);
  }
}

void publish_search_path(const std::vector<Eigen::Vector3d> &path) {
  visualization_msgs::MarkerArray mk_array;
  visualization_msgs::Marker mk;
  mk.header.frame_id = "world";
  mk.header.stamp = ros::Time::now();
  mk.ns = "trajectory_search";
  mk.type = visualization_msgs::Marker::SPHERE;
  mk.action = visualization_msgs::Marker::ADD;

  mk.pose.orientation.x = 0.0;
  mk.pose.orientation.y = 0.0;
  mk.pose.orientation.z = 0.0;
  mk.pose.orientation.w = 1.0;

  mk.color.a = 0.8;
  mk.color.r = 0.6;
  mk.color.g = 0.0;
  mk.color.b = 1.0;

  mk.scale.x = 0.5;
  mk.scale.y = 0.5;
  mk.scale.z = 0.5;

  int idx = 0;
  for (size_t i = 0; i < path.size(); i++) {
    mk.id = idx;

    mk.pose.position.x = path[i](0);
    mk.pose.position.y = path[i](1);
    mk.pose.position.z = path[i](2);

    idx++;
    mk_array.markers.push_back(mk);
  }

  search_path_publisher.publish(mk_array);
}

void publishGridmap(const ros::TimerEvent &e) {
  grid_map_generator_ptr->generate_dynamic_object();
  grid_map_generator_ptr->publishGridMap();
}

void searchPath(const ros::TimerEvent &e) {
  // ros::Time time1;
  grid_path_finder_ptr->resetGlobalMap();
  // ros::Time time2;
  // ROS_WARN("Time consume in resetGlobalMap is %f", (time2 -
  // time1).toSec());
  grid_path_finder_ptr->AstarSearch(startPoint, endPoint);
  std::vector<Eigen::Vector3d> path = grid_path_finder_ptr->getPath();
  publish_search_path(path);
}