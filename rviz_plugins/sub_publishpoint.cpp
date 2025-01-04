#include "ros/console.h"
#include "ros/ros.h"
#include "tf/tf.h"
#include "tf/transform_datatypes.h"
#include "geometry_msgs/PointStamped.h"
void clicked_point_callback(const geometry_msgs::PointStamped& clicked_point){
  double x = clicked_point.point.x;
  double y = clicked_point.point.y;
  double z = clicked_point.point.z;
  std::cout<<"clicked: x: "<<x<<" y: "<<y<<" z: "<<z<<std::endl;
}
int main(int argc, char **argv) {
  ros::init(argc, argv, "receive_published_point_and_pub_its_coordinate");
  ros::NodeHandle handle("~");
  ros::Subscriber clicked_point_sub = handle.subscribe("/clicked_point", 10, clicked_point_callback);
  ros::Rate(50);
  ros::spin();

  return 0;
}