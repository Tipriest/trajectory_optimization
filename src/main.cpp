#include "backward.hpp"
#include "corridor_generate.h"
#include "grid_map.h"
#include "traj_optimize3d.h"
#include "traj_search3d.h"
namespace backward {
backward::SignalHandling sh;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "trajectory_optimization");
  ros::NodeHandle nh("~");

  // 创建 GridMapGenerator 对象
  std::vector<std::string> global_map_layers = {"elevation",
                                                "dynamic_obstacle"};
  std::shared_ptr<grid_map::GridMap> global_map_ptr =
      std::make_shared<grid_map::GridMap>(global_map_layers);
  std::shared_ptr<GridMapGenerator> grid_map_generator_ptr =
      std::make_shared<GridMapGenerator>(nh, global_map_ptr, global_map_layers);

  // AstarSearcher grid_path_finder();
  std::shared_ptr<AstarSearcher> grid_path_finder_ptr =
      std::make_shared<AstarSearcher>(nh, grid_map_generator_ptr);

  CorridorGenerator corridor_generator(nh, grid_map_generator_ptr,
                                       grid_path_finder_ptr);
  std::shared_ptr<CorridorGenerator> corridor_generator_ptr =
      std::make_shared<CorridorGenerator>(corridor_generator);
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
