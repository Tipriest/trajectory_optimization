#include "test_class.h"
TestClass3D::TestClass3D(ros::NodeHandle &h) {
  m_cubes_vis_pub =
      h.advertise<visualization_msgs::MarkerArray>("corridor_vis", 1);
  m_waypoints_vis_pub =
      h.advertise<visualization_msgs::MarkerArray>("waypoint_vis", 1);
  m_poly_traj_vis_pub =
      h.advertise<visualization_msgs::Marker>("trajectory_vis", 1);
  m_bezier_traj_vis_pub =
      h.advertise<geometry_msgs::Pose>("bezier_traj_vis_pub", 1);

  m_bernstein.setParam(3, 12, 3);
  m_MQM = m_bernstein.getMQM()[m_traj_order];
  m_FM = m_bernstein.getFM()[m_traj_order];
  m_C = m_bernstein.getC()[m_traj_order];
  m_Cv = m_bernstein.getC_v()[m_traj_order];
  m_Ca = m_bernstein.getC_a()[m_traj_order];
  m_Cj = m_bernstein.getC_j()[m_traj_order];
}
void TestClass3D::addWayPoint(const Eigen::Vector3d &waypoint) {
  m_waypoints.push_back(waypoint);
}
void TestClass3D::addWayPoints(const vector<Eigen::Vector3d> &waypoints) {
  for (Eigen::Vector3d waypoint : waypoints) {
    m_waypoints.push_back(waypoint);
  }
}

void TestClass3D::expandCubeFromWp() {
  m_cubes.clear();
  for (size_t i = 0; i < m_waypoints.size(); i++) {
    Eigen::Vector3d center = m_waypoints[i];
    m_cubes.push_back(Cube(center, 50));
  }
}

void TestClass3D::timeAllocation() {
  for (size_t i = 0; i < m_cubes.size(); i++) {
    m_cubes[i].m_t = 100;
  }
}

void TestClass3D::solveTrajectory() {
  Eigen::MatrixXd pos = MatrixXd::Zero(2, 3);
  Eigen::MatrixXd vel = MatrixXd::Zero(2, 3);
  Eigen::MatrixXd acc = MatrixXd::Zero(2, 3);
  pos.row(0) = m_cubes[0].m_center;
  pos.row(1) = m_cubes[m_cubes.size() - 1].m_center;
  double obj;
  ros::Time time_bef_opt = ros::Time::now();

  int generateRes = BezierPloyCoeffGeneration3D(
      m_cubes, m_MQM, pos, vel, acc, m_max_vel, m_max_acc, m_traj_order,
      m_minimize_order, 0, true, true, obj, m_bezier_coeff);
  if (generateRes == -1) {
    ROS_WARN("Cannot find a feasible and optimal solution, somthing wrong with "
             "the mosek solver");
    return;
  } else {
    ROS_WARN("successfully generate the optimal path");
  }

  m_seg_num = m_cubes.size();
  m_seg_time.resize(m_seg_num);
  for (int i = 0; i < m_seg_num; i++) {
    m_seg_time(i) = m_cubes[i].m_t;
  }
  getBezierTraj();
  // visBezierTrajectory();
  // visTraj();
  ros::Time time_aft_opt = ros::Time::now();
  ROS_WARN("The objective of the program is %f", obj);
  ROS_WARN("The time consumation of the program is %f",
           (time_aft_opt - time_bef_opt).toSec());
}

void TestClass3D::getBezierTraj() {
  m_bezier_traj.num_segment = m_seg_num;

  int poly_num1d = m_traj_order + 1;
  int polyTotalNum1d = m_seg_num * poly_num1d;

  m_bezier_traj.coef_x.resize(polyTotalNum1d);
  m_bezier_traj.coef_y.resize(polyTotalNum1d);
  m_bezier_traj.coef_z.resize(polyTotalNum1d);

  int idx = 0;
  for (int i = 0; i < m_seg_num; i++) {
    for (int j = 0; j < poly_num1d; j++) {
      m_bezier_traj.coef_x[idx] = m_bezier_coeff(i, j);
      m_bezier_traj.coef_y[idx] = m_bezier_coeff(i, poly_num1d + j);
      m_bezier_traj.coef_z[idx] = m_bezier_coeff(i, 2 * poly_num1d + j);
      idx++;
    }
  }

  m_bezier_traj.header.frame_id = "/trajectory_optj";
  m_bezier_traj.header.stamp = ros::Time::now();

  m_bezier_traj.time.resize(m_seg_num);
  m_bezier_traj.order.resize(m_seg_num);

  m_bezier_traj.mag_coeff = 1.0;
  for (int idx = 0; idx < m_seg_num; ++idx) {
    m_bezier_traj.time[idx] = m_seg_time(idx);
    m_bezier_traj.order[idx] = m_traj_order;
  }

  m_bezier_traj.start_yaw = 0.0;
  m_bezier_traj.final_yaw = 0.0;
}
Eigen::Vector3d TestClass3D::getPosFromBezier(double t_now, int seg_now) {
  Eigen::Vector3d ret = VectorXd::Zero(3);
  VectorXd ctrl_now = m_bezier_coeff.row(seg_now);
  int ctrl_num1D = m_bezier_coeff.cols() / 3;

  for (int i = 0; i < 3; i++)
    for (int j = 0; j < ctrl_num1D; j++)
      ret(i) += m_C(j) * ctrl_now(i * ctrl_num1D + j) * pow(t_now, j) *
                pow((1 - t_now), (m_traj_order - j));

  return ret;
}
void TestClass3D::vis(bool wp, bool corridor, bool traj) {
  if (wp) {
    visWayPoints();
  }
  if (corridor) {
    // std::cout << "vis!" << std::endl;
    visCorridor();
  }
  if (traj) {
    visBezierTrajectory();
  }
}
void TestClass3D::visWayPoints() {
  // 删掉原来残留的waypoints
  for (auto &mk : m_waypoints_vis.markers)
    mk.action = visualization_msgs::Marker::DELETE;

  m_waypoints_vis_pub.publish(m_waypoints_vis);

  m_waypoints_vis.markers.clear();

  // 加入新的cube
  visualization_msgs::Marker mk;
  mk.header.frame_id = "world";
  mk.header.stamp = ros::Time::now();
  mk.ns = "trajectory_optj";
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

  mk.scale.x = 2.5;
  mk.scale.y = 2.5;
  mk.scale.z = 2.5;

  int idx = 0;
  for (int i = 0; i < int(m_waypoints.size()); i++) {
    mk.id = idx;

    mk.pose.position.x = m_waypoints[i](0);
    mk.pose.position.y = m_waypoints[i](1);
    mk.pose.position.z = m_waypoints[i](2);

    idx++;
    m_waypoints_vis.markers.push_back(mk);
  }

  m_waypoints_vis_pub.publish(m_waypoints_vis);
}
void TestClass3D::visCorridor() {
  // 删掉原来残留的cubes
  for (auto &mk : m_cube_vis.markers)
    mk.action = visualization_msgs::Marker::DELETE;

  m_cubes_vis_pub.publish(m_cube_vis);

  m_cube_vis.markers.clear();

  // 加入新的cube
  visualization_msgs::Marker mk;
  mk.header.frame_id = "world";
  mk.header.stamp = ros::Time::now();
  mk.ns = "trajectory_optj";
  mk.type = visualization_msgs::Marker::CUBE;
  mk.action = visualization_msgs::Marker::ADD;

  mk.pose.orientation.x = 0.0;
  mk.pose.orientation.y = 0.0;
  mk.pose.orientation.z = 0.0;
  mk.pose.orientation.w = 1.0;

  mk.color.a = 0.4;
  mk.color.r = 0.6;
  mk.color.g = 1.0;
  mk.color.b = 0.8;

  int idx = 0;
  for (int i = 0; i < int(m_cubes.size()); i++) {
    mk.id = idx;

    mk.pose.position.x =
        (m_cubes[i].m_vertex(0, 0) + m_cubes[i].m_vertex(3, 0)) / 2.0;
    mk.pose.position.y =
        (m_cubes[i].m_vertex(0, 1) + m_cubes[i].m_vertex(1, 1)) / 2.0;
    mk.pose.position.z = 0.0;

    mk.scale.x = (m_cubes[i].m_vertex(0, 0) - m_cubes[i].m_vertex(3, 0));
    mk.scale.y = (m_cubes[i].m_vertex(1, 1) - m_cubes[i].m_vertex(0, 1));
    mk.scale.z = 0.05;

    idx++;
    m_cube_vis.markers.push_back(mk);
  }

  m_cubes_vis_pub.publish(m_cube_vis);
}
void TestClass3D::visTraj() {
  visualization_msgs::Marker traj_vis;
  traj_vis.header.stamp = ros::Time::now();
  traj_vis.header.frame_id = "world";

  traj_vis.ns = "trajectory_optj";
  traj_vis.id = 0; // 标记的 ID，确保唯一
  traj_vis.type =
      visualization_msgs::Marker::SPHERE_LIST; // 使用 SPHERE_LIST 显示轨迹点

  traj_vis.action = visualization_msgs::Marker::ADD; // 添加轨迹
  traj_vis.scale.x = 1.0;                            // 点的大小
  traj_vis.scale.y = 1.0;
  traj_vis.scale.z = 1.0;

  traj_vis.pose.orientation.w = 1.0; // 默认不旋转
  traj_vis.color.r = 1.0;            // 轨迹颜色
  traj_vis.color.g = 0.0;
  traj_vis.color.b = 0.0;
  traj_vis.color.a = 1.0; // 不透明

  // 定义轨迹点
  std::vector<geometry_msgs::Point> points;

  // 这里生成一个简单的直线路径，您可以根据实际轨迹生成方式来修改
  for (double t = 0.0; t < 10.0; t += 0.1) {
    geometry_msgs::Point pt;
    pt.x = 10 * t;      // X 轴位置
    pt.y = 10 * sin(t); // Y 轴位置，使用正弦函数来模拟轨迹
    pt.z = 10 * cos(t); // Z 轴位置，使用余弦函数来模拟轨迹
    points.push_back(pt);
  }

  // 将点添加到 Marker 中
  traj_vis.points = points;

  // 发布轨迹
  m_poly_traj_vis_pub.publish(traj_vis);
}

/**
 * @brief
 * @param[in] time          My Param doc
 * @author Tipriest (a1503741059@163.com)
 */
void TestClass3D::visBezierTrajectory() {

  double traj_len = 0.0;
  int count = 0;
  Eigen::Vector3d cur, pre;
  cur.setZero();
  pre.setZero();

  Eigen::Vector3d state;

  // 定义轨迹点
  geometry_msgs::Pose traj_point;
  int idx = 0;
  int segment_num = m_bezier_coeff.rows();
  for (int i = 0; i < segment_num; i++) {
    for (double t = 0.0; t < 1.0; t += 10 / m_seg_time(i), count += 1) {
      state = getPosFromBezier(t, i);

      traj_point.orientation.w = 1.0;
      cur(0) = traj_point.position.x = m_seg_time(i) * state(0);
      cur(1) = traj_point.position.y = m_seg_time(i) * state(1);
      cur(2) = traj_point.position.z = m_seg_time(i) * state(2);
      m_bezier_traj_vis.poses.push_back(traj_point);

      if (count)
        traj_len += (pre - cur).norm();
      pre = cur;
    }
  }
  // ROS_INFO("[GENERATOR] The length of the trajectory; %.3lfm.", traj_len);
  // std::cout << "begin!" << std::endl;
  // ros::Duration(5.0).sleep();
  m_bezier_traj_vis_pub.publish(m_bezier_traj_vis);
  // m_bezier_traj_vis_pub.publish(m_bezier_traj_vis);
  // m_bezier_traj_vis_pub.publish(m_bezier_traj_vis);
  // std::cout << "finish!" << std::endl;
  // ros::Duration(5.0).sleep();
}