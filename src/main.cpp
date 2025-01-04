#include "backward.hpp"
#include "test_class.h"
namespace backward {

backward::SignalHandling sh;

}
int main(int argc, char **argv) {
  ros::init(argc, argv, "test");
  ros::NodeHandle nh("~");
  TestClass3D testcase(nh);
  Eigen::Vector3d point1, point2, point3, point4, point5;
  std::vector<Eigen::Vector3d> waypoints = {Eigen::Vector3d(50, 50, 0),   //
                                            Eigen::Vector3d(100, 120, 0), //
                                            Eigen::Vector3d(180, 150, 0), //
                                            Eigen::Vector3d(250, 80, 0),  //
                                            Eigen::Vector3d(280, 0, 0)};
  testcase.addWayPoints(waypoints);
  testcase.expandCubeFromWp();
  testcase.timeAllocation();
  Cube cube_print = testcase.m_cubes[0];
  for (int i = 0; i < 8; i++) {
    std::cout << "vertex "
              << "i: " << i << ": ("
              << "x = " << cube_print.m_vertex(i, 0) << ", "
              << "y = " << cube_print.m_vertex(i, 1) << ", "
              << "z = " << cube_print.m_vertex(i, 2) << ")" << std::endl;
  }
  testcase.solveTrajectory();

  ros::Rate rate(20);
  while (true) {
    testcase.vis(true, true, true);
    rate.sleep();
    ros::spinOnce();
  }
  return 0;
}