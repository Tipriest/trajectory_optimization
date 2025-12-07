#include "backward.hpp"
#include "../include/path_searcher/base_path_searcher.h"
namespace backward {
backward::SignalHandling sh;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "trajectory_optimization");
  ros::NodeHandle nh("~");

  // 创建 GridMapGenerator 对象
  std::vector<std::string> global_map_layers = {"elevation",
                                                "dynamic_obstacle"};
  // AstarSearcher grid_path_finder();
  // std::shared_ptr<BasePathSearcher> base_path_searcher_ptr =
  //     std::make_shared<BasePathSearcher>(nh, global_map_layers);

  ros::Rate rate(100);
  // 使用单线程
  while (true) {
    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}
