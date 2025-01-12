#include "traj_search3d.h"

using namespace std;
using namespace Eigen;

void PathSearcher::displayPath() {

  removeDisplayPath();
  m_path_vis.points.clear();

  m_path_vis.header.frame_id = "world";
  m_path_vis.header.stamp = ros::Time();
  m_path_vis.ns = "trajectory_search";
  m_path_vis.type = visualization_msgs::Marker::LINE_STRIP;
  m_path_vis.action = visualization_msgs::Marker::ADD;

  m_path_vis.pose.orientation.x = 0.0;
  m_path_vis.pose.orientation.y = 0.0;
  m_path_vis.pose.orientation.z = 0.0;
  m_path_vis.pose.orientation.w = 1.0;

  m_path_vis.color.a = 0.8;
  m_path_vis.color.r = 0.6;
  m_path_vis.color.g = 0.8;
  m_path_vis.color.b = 0.2;

  m_path_vis.scale.x = 0.2; // LINE_STRIP的形状只取决于scale.x

  m_path_vis.id = 0;
  std::vector<Eigen::Vector3d> path = getPath();
  for (size_t i = 0; i < path.size(); i++) {
    geometry_msgs::Point point;
    point.x = path[i](0);
    point.y = path[i](1);
    point.z = path[i](2);
    m_path_vis.points.push_back(point);
  }

  m_search_path_publisher.publish(m_path_vis);
}

void PathSearcher::removeDisplayPath() {

  m_path_vis.points.clear();
  m_path_vis.header.frame_id = "world";
  m_path_vis.header.stamp = ros::Time();
  m_path_vis.ns = "trajectory_search";
  m_path_vis.type = visualization_msgs::Marker::LINE_STRIP;
  m_path_vis.action = visualization_msgs::Marker::DELETE;

  m_search_path_publisher.publish(m_path_vis);
}

void AstarSearcher::rcvPosCmdCallBack(const geometry_msgs::PoseStamped &cmd) {
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
// 理论上的map的更新逻辑是一开始init的时候初始化一个全局的地图
// 随后在局部地图进行初始化

// 但是我现在在仿真中进行应用的时候,因为有动态障碍物,而且没有一个真实的感知范围,
// 因此选择将全局的地图都进行更新,但是在实车上使用的时候应该使用的是linkLocalMap

// 然后这个里面保留一个grid_map的指针用来执行更新
AstarSearcher::AstarSearcher(
    ros::NodeHandle nh, std::shared_ptr<GridMapGenerator> gridmap_generator)
    : m_grid_map_genertaor_ptr(gridmap_generator) {
  m_nh = nh;

  m_grid_map = m_grid_map_genertaor_ptr->m_grid_map;
  m_resolution = m_grid_map_genertaor_ptr->m_resolution;
  m_inv_resolution = 1.0 / m_resolution;
  m_gl_xl = m_grid_map_genertaor_ptr->m_leftdown_offset_x;
  m_gl_yl = m_grid_map_genertaor_ptr->m_leftdown_offset_y;
  m_gl_zl = 0.0;
  m_GLX_SIZE = m_grid_map_genertaor_ptr->m_length / m_resolution;
  m_GLY_SIZE = m_grid_map_genertaor_ptr->m_width / m_resolution;
  m_GLZ_SIZE = 1;
  initGridNodeMap();

  start_end_point_vis_publisher =
      m_nh.advertise<visualization_msgs::MarkerArray>("start_end_point", 1);
  start_end_point_subscriber = m_nh.subscribe(
      "/move_base_simple/goal", 1, &AstarSearcher::rcvPosCmdCallBack, this);
  m_search_path_publisher =
      nh.advertise<visualization_msgs::Marker>("search_path", 1);

  path_search_timer = m_nh.createTimer(
      ros::Duration(0.05),
      boost::bind(&AstarSearcher::searchAndVisPathCB, this, _1));
  path_search_timer.start();
}

void AstarSearcher::initGridNodeMap() {
  GridNodeMap = new GridNodePtr **[m_GLX_SIZE];
  for (int i = 0; i < m_GLX_SIZE; i++) {
    GridNodeMap[i] = new GridNodePtr *[m_GLY_SIZE];
    for (int j = 0; j < m_GLY_SIZE; j++) {
      GridNodeMap[i][j] = new GridNodePtr[m_GLZ_SIZE];
      for (int k = 0; k < m_GLZ_SIZE; k++) {
        Vector3i tmpIdx(i, j, k);
        Vector3d pos = gridIndex2coord(tmpIdx);
        GridNodeMap[i][j][k] = new GridNode(tmpIdx, pos);
        GridNodeMap[i][j][k]->occupancy =
            m_grid_map_genertaor_ptr->getOccupancy(
                pos(0), pos(1), m_grid_map_genertaor_ptr->m_layers);
      }
    }
  }
}

// void AstarSearcher::linkLocalMap(std::shared_ptr<grid_map::GridMap>
// local_map,
//                                   Vector3d xyz_l) {
//   Vector3d coord;
//   for (int64_t i = 0; i < m_LX_SIZE; i++) {
//     for (int64_t j = 0; j < m_LY_SIZE; j++) {
//       for (int64_t k = 0; k < m_LZ_SIZE; k++) {
//         coord(0) = xyz_l(0) + (double)(i + 0.5) * resolution;
//         coord(1) = xyz_l(1) + (double)(j + 0.5) * resolution;
//         coord(2) = xyz_l(2) + (double)(k + 0.5) * resolution;

//         Vector3i index = coord2gridIndex(coord);

//         if (index(0) >= m_GLX_SIZE || index(1) >= m_GLY_SIZE ||
//             index(2) >= m_GLZ_SIZE || index(0) < 0 || index(1) < 0 ||
//             index(2) < 0)
//           continue;

//         GridNodePtr ptr = GridNodeMap[index(0)][index(1)][index(2)];
//         ptr->id = 0;
//         ptr->occupancy = local_map->Get(i, j, k).first.occupancy;
//       }
//     }
//   }
// }

void AstarSearcher::resetLocalMap() {
  // ROS_WARN("expandedNodes size : %d", expandedNodes.size());
  for (auto tmpPtr : expandedNodes) {
    tmpPtr->occupancy = 0; // forget the occupancy
    tmpPtr->id = 0;
    tmpPtr->cameFrom = NULL;
    tmpPtr->gScore = inf;
    tmpPtr->fScore = inf;
  }

  for (auto ptr : openSet) {
    GridNodePtr tmpPtr = ptr.second;
    tmpPtr->occupancy = 0; // forget the occupancy
    tmpPtr->id = 0;
    tmpPtr->cameFrom = NULL;
    tmpPtr->gScore = inf;
    tmpPtr->fScore = inf;
  }

  expandedNodes.clear();
  // ROS_WARN("local map reset finish");
}

void AstarSearcher::resetGlobalMap() {
  // ROS_WARN("expandedNodes size : %d", expandedNodes.size());
  for (auto tmpPtr : expandedNodes) {
    tmpPtr->occupancy = 0; // forget the occupancy
    tmpPtr->id = 0;
    tmpPtr->cameFrom = NULL;
    tmpPtr->gScore = inf;
    tmpPtr->fScore = inf;
  }

  for (auto ptr : openSet) {
    GridNodePtr tmpPtr = ptr.second;
    tmpPtr->occupancy = 0; // forget the occupancy
    tmpPtr->id = 0;
    tmpPtr->cameFrom = NULL;
    tmpPtr->gScore = inf;
    tmpPtr->fScore = inf;
  }

  expandedNodes.clear();

  for (int i = 0; i < m_GLX_SIZE; i++) {
    for (int j = 0; j < m_GLY_SIZE; j++) {
      for (int k = 0; k < m_GLZ_SIZE; k++) {
        Vector3i tmpIdx(i, j, k);
        Vector3d pos = gridIndex2coord(tmpIdx);
        GridNodeMap[i][j][k]->occupancy =
            m_grid_map_genertaor_ptr->getOccupancy(
                pos(0), pos(1), m_grid_map_genertaor_ptr->m_layers);
      }
    }
  }

  // ROS_WARN("local map reset finish");
}
GridNodePtr AstarSearcher::pos2gridNodePtr(Vector3d pos) {
  Vector3i idx = coord2gridIndex(pos);
  GridNodePtr grid_ptr = new GridNode(idx, pos);

  return grid_ptr;
}

Vector3d AstarSearcher::gridIndex2coord(Vector3i index) {
  Vector3d pt;
  // cell_x_size_ * ((double)x_index + 0.5), cell_y_size_ * ((double)y_index +
  // 0.5), cell_z_size_ * ((double)z_index + 0.5)

  pt(0) = ((double)index(0) + 0.5) * m_resolution + m_gl_xl;
  pt(1) = ((double)index(1) + 0.5) * m_resolution + m_gl_yl;
  pt(2) = ((double)index(2) + 0.5) * m_resolution + m_gl_zl;

  /*pt(0) = (double)index(0) * resolution + m_gl_xl + 0.5 * resolution;
  pt(1) = (double)index(1) * resolution + m_gl_yl + 0.5 * resolution;
  pt(2) = (double)index(2) * resolution + m_gl_zl + 0.5 * resolution;*/
  return pt;
}

Vector3i AstarSearcher::coord2gridIndex(Vector3d pt) {
  Vector3i idx;
  idx << min(max(int((pt(0) - m_gl_xl) * m_inv_resolution), 0), m_GLX_SIZE - 1),
      min(max(int((pt(1) - m_gl_yl) * m_inv_resolution), 0), m_GLY_SIZE - 1),
      min(max(int((pt(2) - m_gl_zl) * m_inv_resolution), 0), m_GLZ_SIZE - 1);

  return idx;
}

double AstarSearcher::getDiagHeu(GridNodePtr node1, GridNodePtr node2) {
  double dx = abs(node1->index(0) - node2->index(0));
  double dy = abs(node1->index(1) - node2->index(1));
  double dz = abs(node1->index(2) - node2->index(2));

  double h = 0.0;
  int diag = min(min(dx, dy), dz);
  dx -= diag;
  dy -= diag;
  dz -= diag;

  if (dx == 0) {
    h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * min(dy, dz) + 1.0 * abs(dy - dz);
  }
  if (dy == 0) {
    h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * min(dx, dz) + 1.0 * abs(dx - dz);
  }
  if (dz == 0) {
    h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * min(dx, dy) + 1.0 * abs(dx - dy);
  }
  return h;
}

double AstarSearcher::getManhHeu(GridNodePtr node1, GridNodePtr node2) {
  double dx = abs(node1->index(0) - node2->index(0));
  double dy = abs(node1->index(1) - node2->index(1));
  double dz = abs(node1->index(2) - node2->index(2));

  return dx + dy + dz;
}

double AstarSearcher::getEuclHeu(GridNodePtr node1, GridNodePtr node2) {
  return (node2->index - node1->index).norm();
}

double AstarSearcher::getHeu(GridNodePtr node1, GridNodePtr node2) {
  return m_tie_breaker * getDiagHeu(node1, node2);
  // return m_tie_breaker * getEuclHeu(node1, node2);
}

vector<GridNodePtr> AstarSearcher::retrievePath(GridNodePtr current) {
  vector<GridNodePtr> path;
  path.push_back(current);

  while (current->cameFrom != NULL) {
    current = current->cameFrom;
    path.push_back(current);
  }

  return path;
}

vector<GridNodePtr> AstarSearcher::getVisitedNodes() {
  vector<GridNodePtr> visited_nodes;
  for (int i = 0; i < m_GLX_SIZE; i++)
    for (int j = 0; j < m_GLY_SIZE; j++)
      for (int k = 0; k < m_GLZ_SIZE; k++) {
        if (GridNodeMap[i][j][k]->id != 0)
          // if(GridNodeMap[i][j][k]->id == -1)
          visited_nodes.push_back(GridNodeMap[i][j][k]);
      }

  ROS_WARN("visited_nodes size : %ld", visited_nodes.size());
  return visited_nodes;
}

/*bool AstarSearcher::minClearance()
{
    neighborPtr->occupancy > 0.5
}
*/
void AstarSearcher::searchPath(Eigen::Vector3d start_pt,
                               Eigen::Vector3d end_pt) {
  // ros::Time   = ros::Time::now();
  GridNodePtr startPtr = pos2gridNodePtr(start_pt);
  GridNodePtr endPtr = pos2gridNodePtr(end_pt);

  openSet.clear();

  GridNodePtr neighborPtr = NULL;
  GridNodePtr current = NULL;

  startPtr->gScore = 0;
  startPtr->fScore = getHeu(startPtr, endPtr);
  startPtr->id = 1; // put start node in open set
  startPtr->coord = start_pt;
  openSet.insert(make_pair(startPtr->fScore, startPtr)); // put start in open
                                                         // set

  double tentative_gScore;

  int num_iter = 0;
  while (!openSet.empty()) {
    num_iter++;
    current = openSet.begin()->second;

    if (current->index(0) == endPtr->index(0) &&
        current->index(1) == endPtr->index(1) &&
        current->index(2) == endPtr->index(2)) {
      // ROS_WARN("[Astar]Reach goal..");
      // cout << "goal coord: " << endl << current->real_coord << endl;
      // cout << "total number of iteration used in Astar: " << num_iter <<
      // endl; ros::Time time_2 = ros::Time::now(); ROS_WARN("Time consume in A
      // star path finding is %f",
      //          (time_2 - time_1).toSec());
      gridPath = retrievePath(current);
      return;
    }
    openSet.erase(openSet.begin());
    current->id = -1; // move current node from open set to closed set.
    expandedNodes.push_back(current);

    for (int dx = -1; dx < 2; dx++)
      for (int dy = -1; dy < 2; dy++)
        for (int dz = -1; dz < 2; dz++) {
          if (dx == 0 && dy == 0 && dz == 0)
            continue;

          Vector3i neighborIdx;
          neighborIdx(0) = (current->index)(0) + dx;
          neighborIdx(1) = (current->index)(1) + dy;
          neighborIdx(2) = (current->index)(2) + dz;

          if (neighborIdx(0) < 0 || neighborIdx(0) >= m_GLX_SIZE ||
              neighborIdx(1) < 0 || neighborIdx(1) >= m_GLY_SIZE ||
              neighborIdx(2) < 0 || neighborIdx(2) >= m_GLZ_SIZE) {
            continue;
          }

          neighborPtr =
              GridNodeMap[neighborIdx(0)][neighborIdx(1)][neighborIdx(2)];

          /*                    if(minClearance() == false){
                                  continue;
                              }*/

          if (neighborPtr->occupancy > 0.5) {
            continue;
          }

          if (neighborPtr->id == -1) {
            continue; // in closed set.
          }

          double static_cost = sqrt(dx * dx + dy * dy + dz * dz);

          tentative_gScore = current->gScore + static_cost;

          if (neighborPtr->id != 1) {
            // discover a new node
            neighborPtr->id = 1;
            neighborPtr->cameFrom = current;
            neighborPtr->gScore = tentative_gScore;
            neighborPtr->fScore =
                neighborPtr->gScore + getHeu(neighborPtr, endPtr);
            neighborPtr->nodeMapIt = openSet.insert(make_pair(
                neighborPtr->fScore,
                neighborPtr)); // put neighbor in open set and record it.
            continue;
          } else if (tentative_gScore <=
                     neighborPtr->gScore) { // in open set and need update
            neighborPtr->cameFrom = current;
            neighborPtr->gScore = tentative_gScore;
            neighborPtr->fScore =
                tentative_gScore + getHeu(neighborPtr, endPtr);
            openSet.erase(neighborPtr->nodeMapIt);
            neighborPtr->nodeMapIt = openSet.insert(make_pair(
                neighborPtr->fScore,
                neighborPtr)); // put neighbor in open set and record it.
          }
        }
  }

  // ros::Time time_2 = ros::Time::now();
  // ROS_WARN("Time consume in A star path finding is %f",
  //          (time_2 - time_1).toSec());
}

vector<Vector3d> AstarSearcher::getPath() {
  vector<Vector3d> path;

  for (auto ptr : gridPath)
    path.push_back(ptr->coord);

  reverse(path.begin(), path.end());
  return path;
}

void AstarSearcher::resetPath() { gridPath.clear(); }
void AstarSearcher::searchAndVisPathCB(const ros::TimerEvent &e) {
  resetGlobalMap();
  searchPath(start_point, end_point);
  displayPath();
}