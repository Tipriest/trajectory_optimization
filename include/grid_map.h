#pragma once

#include <algorithm>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Polygon.h>
#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_msgs/GridMap.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <random>
#include <ros/ros.h>

class GridMapGenerator {
public:
  GridMapGenerator(const ros::NodeHandle &nh);
  void generateGridMap();
  void publishGridMap();
  void generateAndPublishMap();
  void mapPubTimerCB(const ros::TimerEvent &e);
  void createPolygons(
      std::vector<std::vector<geometry_msgs::Point32>> polygon_points);
  void random_generate_obs(int circle_num, double radius_min,
                           double radius_max);

private:
  ros::NodeHandle nh_;

  double m_length = 40.0;
  double m_width = 40.0;
  double m_resolution = 0.1;
  double start_pos_x = 0.0;
  double start_pos_y = 0.0;

  ros::Publisher m_map_publisher;
  std::vector<geometry_msgs::Polygon> m_polygons;
  grid_map::GridMap m_grid_map;
  ros::Timer m_map_publish_timer;

  bool isPointInPolygon(double x, double y);
  void createPolygonExample();
};
