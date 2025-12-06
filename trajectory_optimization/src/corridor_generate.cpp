#include "corridor_generate.h"
CorridorGenerator::CorridorGenerator(
    const ros::NodeHandle &nh,
    std::shared_ptr<GridMapGenerator> gridmap_generator,
    std::shared_ptr<AstarSearcher> gird_path_finder)
    : m_grid_map_genertaor_ptr(gridmap_generator), m_nh(nh),
      m_grid_path_finder_ptr(gird_path_finder) {
  m_resolution = m_grid_map_genertaor_ptr->m_resolution;
  m_length = m_grid_map_genertaor_ptr->m_length;
  m_width = m_grid_map_genertaor_ptr->m_width;
  m_ellipsoid_array_pub = m_nh.advertise<decomp_ros_msgs::EllipsoidArray>(
      "ellipsoid_array", 1, true);
  m_polyhedron_array_pub = m_nh.advertise<decomp_ros_msgs::PolyhedronArray>(
      "polyhedron_array", 1, true);
  m_discrete_path_pub =
      m_nh.advertise<visualization_msgs::Marker>("discrete_path", 1, true);
  corridor_generate_timer = m_nh.createTimer(
      ros::Duration(0.05),
      boost::bind(&CorridorGenerator::corridor_generate_timer_cb, this, _1));
  corridor_generate_timer.start();
}

void CorridorGenerator::getDiscretePath(double interval) {
  m_discrete_path.clear();
  std::vector<Eigen::Vector3d> astar_path_points =
      m_grid_path_finder_ptr->getPath();
  if (astar_path_points.size() == 0) {
    return;
  }
  Eigen::Vector2d astar_path_start_point(astar_path_points[0](0),
                                         astar_path_points[0](1));
  m_discrete_path.push_back(astar_path_start_point);
  // int step = max(1, (int)(interval / m_resolution) - 1);
  auto calDis = [](Eigen::Vector3d a, Eigen::Vector2d b) {
    return sqrt(pow(a(0) - b(0), 2) + pow(a(1) - b(1), 2));
  };
  for (size_t i = 1; i < astar_path_points.size() - 1; i++) {
    if (calDis(astar_path_points[i],
               m_discrete_path[m_discrete_path.size() - 1]) >= interval) {
      Eigen::Vector2d astar_path_point(astar_path_points[i](0),
                                       astar_path_points[i](1));
      m_discrete_path.push_back(astar_path_point);
    }
  }
  Eigen::Vector2d astar_path_last_point(
      astar_path_points[astar_path_points.size() - 1](0),
      astar_path_points[astar_path_points.size() - 1](1));
  m_discrete_path.push_back(astar_path_last_point);
}
void CorridorGenerator::getObs() {
  m_obs2d.clear();
  for (grid_map::GridMapIterator it(m_grid_map_genertaor_ptr->m_grid_map);
       !it.isPastEnd(); ++(++(++(++it)))) {
    std::vector<std::string> map_layers = m_grid_map_genertaor_ptr->m_layers;
    for (auto map_layer : map_layers) {
      if (m_grid_map_genertaor_ptr->m_grid_map.at(map_layer, *it) > 1.0) {
        grid_map::Position position;
        m_grid_map_genertaor_ptr->m_grid_map.getPosition(*it, position);
        m_obs2d.push_back(position);
      }
    }
  }
  for (double x = -m_length / 2; x <= m_length / 2; x += 0.1) {
    m_obs2d.push_back(Eigen::Vector2d(x, -m_width / 2));
    m_obs2d.push_back(Eigen::Vector2d(x, m_width / 2));
  }
  for (double y = -m_width / 2; y <= m_width / 2; y += 0.1) {
    m_obs2d.push_back(Eigen::Vector2d(-m_length, y));
    m_obs2d.push_back(Eigen::Vector2d(m_length, y));
  }
}
void CorridorGenerator::generateCorridor() {
  // getDiscretePath();
  if (m_discrete_path.size() == 0) {
    return;
  }
  getObs();
  m_decomp_util.set_obs(m_obs2d);
  m_decomp_util.set_local_bbox(Vec2f(1, 2));
  m_decomp_util.dilate(m_discrete_path);
  m_elliposid2d = m_decomp_util.get_ellipsoids();
  m_polyhedron2d = m_decomp_util.get_polyhedrons();
}

void CorridorGenerator::displayCorridor() {
  if (m_elliposid2d.size() == 0 || m_polyhedron2d.size() == 0) {
    return;
  }
  // Publish visualization msgs
  decomp_ros_msgs::EllipsoidArray es_msg =
      DecompROS::ellipsoid_array_to_ros(m_elliposid2d);
  es_msg.header.frame_id = "world";
  m_ellipsoid_array_pub.publish(es_msg);

  decomp_ros_msgs::PolyhedronArray poly_msg =
      DecompROS::polyhedron_array_to_ros(m_polyhedron2d);
  poly_msg.header.frame_id = "world";
  m_polyhedron_array_pub.publish(poly_msg);
}

void CorridorGenerator::displayDiscretePathPoint() {
  if (m_discrete_path.size() == 0) {
    return;
  }
  visualization_msgs::Marker m_discrete_path_vis;
  m_discrete_path_vis.points.clear();

  m_discrete_path_vis.header.frame_id = "world";
  m_discrete_path_vis.header.stamp = ros::Time();
  m_discrete_path_vis.ns = "corridor_generate";
  m_discrete_path_vis.type = visualization_msgs::Marker::POINTS;
  m_discrete_path_vis.action = visualization_msgs::Marker::ADD;

  m_discrete_path_vis.color.a = 0.8;
  m_discrete_path_vis.color.r = 1.0;
  m_discrete_path_vis.color.g = 0.0;
  m_discrete_path_vis.color.b = 0.0;

  m_discrete_path_vis.scale.x = 0.3; // LINE_STRIP的形状只取决于scale.x
  m_discrete_path_vis.scale.y = 0.3; // LINE_STRIP的形状只取决于scale.x

  m_discrete_path_vis.id = 0;

  for (size_t i = 0; i < m_discrete_path.size(); i++) {
    geometry_msgs::Point point;
    point.x = m_discrete_path[i](0);
    point.y = m_discrete_path[i](1);
    point.z = 0.0;
    m_discrete_path_vis.points.push_back(point);
  }

  m_discrete_path_pub.publish(m_discrete_path_vis);
}

void CorridorGenerator::corridor_generate_timer_cb(const ros::TimerEvent &e) {
  getDiscretePath(2.0);
  // if(m_discrete_path.size()==2){
  //   displayDiscretePathPoint();
  //   return;
  // }
  getObs();
  generateCorridor();
  displayDiscretePathPoint();
  displayCorridor();
}