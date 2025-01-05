#include "grid_map.h"

using namespace grid_map;

GridMapGenerator::GridMapGenerator(const ros::NodeHandle &nh)
    : nh_(nh), m_grid_map({"elevation"}) {
  // 初始化 GridMap
  m_grid_map.setFrameId("world");
  m_grid_map.setGeometry(grid_map::Length(m_length, m_width), m_resolution,
                         grid_map::Position(start_pos_x, start_pos_y));
  generateGridMap();
  random_generate_obs(30, std::min(m_length, m_width) / 25,
                      std::max(m_length, m_width) / 10);
  // 发布 GridMap
  m_map_publisher = nh_.advertise<grid_map_msgs::GridMap>("grid_map", 1);
  m_map_publish_timer =
      nh_.createTimer(ros::Duration(1.0),
                      boost::bind(&GridMapGenerator::mapPubTimerCB, this, _1));
  m_map_publish_timer.start();
}

bool GridMapGenerator::isPointInPolygon(double x, double y) {
  for (auto polygon : m_polygons) {
    int n = polygon.points.size();
    bool inside = false;
    double p1x = polygon.points[0].x, p1y = polygon.points[0].y;
    for (int i = 1; i <= n; ++i) {
      double p2x = polygon.points[i % n].x, p2y = polygon.points[i % n].y;
      if (y > std::min(p1y, p2y)) {
        if (y <= std::max(p1y, p2y)) {
          if (x <= std::max(p1x, p2x)) {
            if (p1y != p2y) {
              double xinters = (y - p1y) * (p2x - p1x) / (p2y - p1y) + p1x;
              if (p1x == p2x || x <= xinters)
                inside = !inside;
            }
          }
        }
      }
      p1x = p2x;
      p1y = p2y;
    }
    if (inside) {
      return true;
    }
  }
  return false;
}

void GridMapGenerator::random_generate_obs(int circle_num, double radius_min,
                                           double radius_max) {
  // radius_min = std::min(m_length, m_width) / 25;
  // radius_max = std::max(m_length, m_width) / 10;
  // circle_num = 10;

  std::random_device rd; // 获取硬件随机数生成器的种子
  std::mt19937 gen(rd()); // 使用梅森旋转算法（Mersenne Twister）生成随机数
  std::uniform_int_distribution<> dis(1, 1000); // 定义随机数范围为1到1000
  std::vector<grid_map::Position> centers;
  auto tooclose = [centers, radius_max](grid_map::Position cur_center) -> bool {
    for (auto center : centers) {
      double dis = sqrt(pow(center.x() - cur_center.x(), 2) +
                        pow(center.y() - cur_center.y(), 2));
      if (dis < radius_max) {
        return true;
      }
    }
    return false;
  };
  for (int i = 0; i < circle_num; i++) {
    // 首先决定半径
    double radius = radius_min + dis(gen) / 1000.0 * (radius_max - radius_min);
    // 其次决定圆心位置
    grid_map::Position cur_center;
    cur_center.x() = dis(gen) / 1000.0 * m_length - m_length / 2;
    cur_center.y() = dis(gen) / 1000.0 * m_width - m_width / 2;
    if (tooclose(cur_center)) {
      i--;
      continue;
    }
    // 决定是什么形状，可选项有circle
    // int seed = dis(gen);
    int seed = 615;
    if (seed < 300) {
      for (grid_map::CircleIterator iterator(m_grid_map, cur_center, radius);
           !iterator.isPastEnd(); ++iterator) {
        m_grid_map.at("elevation", *iterator) = 1.5;
      }
    } else if (300 <= seed && seed < 400) {
      grid_map::Length length(2 * radius * dis(gen) / 1000.0,
                              2 * radius * dis(gen) / 1000.0);
      for (grid_map::EllipseIterator iterator(m_grid_map, cur_center, length,
                                              dis(gen) / 1000.0 * M_PI * 2);
           !iterator.isPastEnd(); ++iterator) {
        m_grid_map.at("elevation", *iterator) = 1.5;
      }
    } else if (400 <= seed && seed < 600) {
      for (grid_map::SpiralIterator iterator(m_grid_map, cur_center, radius);
           !iterator.isPastEnd(); ++iterator) {
        m_grid_map.at("elevation", *iterator) = 1.5;
      }
    } else if (600 <= seed && seed < 700) {
      double theta = 2 * M_PI * dis(gen) / 1000.0;
      grid_map::Position start(cur_center.x() + radius * cos(theta),
                               cur_center.y() + radius * sin(theta));
      grid_map::Position end(cur_center.x() - radius * cos(theta),
                             cur_center.y() - radius * sin(theta));
      for (grid_map::LineIterator iterator(m_grid_map, start, end);
           !iterator.isPastEnd(); ++iterator) {
        m_grid_map.at("elevation", *iterator) = 1.5;
      }
    } else if (700 <= seed && seed <= 1000) {
      int polygon_num = dis(gen) % 4 + 3;
      double theta_per = 2 * M_PI / polygon_num;
      grid_map::Polygon polygon;
      for (int i = 0; i <= polygon_num; i++) {
        double theta = i * theta_per;
        polygon.addVertex(
            grid_map::Position(cur_center.x() + radius * cos(theta),
                               cur_center.y() + radius * sin(theta)));
      }
      for (grid_map::PolygonIterator iterator(m_grid_map, polygon);
           !iterator.isPastEnd(); ++iterator) {
        m_grid_map.at("elevation", *iterator) = 1.5;
      }
    }
  }
}

void GridMapGenerator::createPolygonExample() {
  geometry_msgs::Polygon polygon;
  geometry_msgs::Point32 p1, p2, p3, p4;
  p1.x = 2.0;
  p1.y = 2.0;
  p1.z = 0.0;

  p2.x = 6.0;
  p2.y = 2.0;
  p2.z = 0.0;

  p3.x = 6.0;
  p3.y = 6.0;
  p3.z = 0.0;

  p4.x = 2.0;
  p4.y = 6.0;
  p4.z = 0.0;

  polygon.points.push_back(p1);
  polygon.points.push_back(p2);
  polygon.points.push_back(p3);
  polygon.points.push_back(p4);
  m_polygons.push_back(polygon);
  return;
}
void GridMapGenerator::createPolygons(
    std::vector<std::vector<geometry_msgs::Point32>> polygon_points) {
  for (int i = 0; i < polygon_points.size(); i++) {
    geometry_msgs::Polygon polygon;
    for (int j = 0; j < polygon_points[i].size(); j++) {
      polygon.points.push_back(polygon_points[i][j]);
    }
    m_polygons.push_back(polygon);
  }
  return;
}
void GridMapGenerator::generateGridMap() {

  // 设置地图上的高度值
  for (GridMapIterator it(m_grid_map); !it.isPastEnd(); ++it) {
    // Position position;
    // m_grid_map.getPosition(*it, position);

    m_grid_map.at("elevation", *it) = 0.0; // 区域外高度为 0

    // if (isPointInPolygon(x, y)) {
    //   m_grid_map.at("elevation", *it) = 1.5; // 区域内高度为 5
    // } else {
    //   m_grid_map.at("elevation", *it) = 0.0; // 区域外高度为 0
    // }
  }
}

void GridMapGenerator::publishGridMap() {
  grid_map_msgs::GridMap grid_map_msg;
  grid_map::GridMapRosConverter::toMessage(m_grid_map, grid_map_msg);
  m_map_publisher.publish(grid_map_msg);
}

void GridMapGenerator::generateAndPublishMap() {
  generateGridMap();
  publishGridMap();
}

void GridMapGenerator::mapPubTimerCB(const ros::TimerEvent &e) {
  publishGridMap();
}
