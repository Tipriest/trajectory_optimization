#include "../include/path_searcher/base_path_searcher.h"
#include "../include/path_searcher/bfs_path_searcher.h"
#include "../include/path_searcher/dfs_path_searcher.h"
#include "backward.hpp"

namespace backward {
backward::SignalHandling sh;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "trajectory_optimization");
  ros::NodeHandle nh("~");

  std::string grid_map_topic = "/map_generator_node/grid_map";
  std::string search_type = "bfs";
  nh.param("search_type", search_type, search_type);

  std::shared_ptr<BasePathSearcher> path_searcher;
  if (search_type == "dfs") {
    path_searcher = std::make_shared<DFSSearcher>(nh, grid_map_topic);
  } else {
    path_searcher = std::make_shared<BFSSearcher>(nh, grid_map_topic);
  }

  ros::Rate rate(100);
  while (ros::ok()) {
    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}
