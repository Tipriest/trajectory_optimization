#ifndef _DATA_TYPE_
#define _DATA_TYPE_

#include <ros/console.h>
#include <ros/ros.h>
#include <stdio.h>

#include <eigen3/Eigen/Dense>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#define inf 1 >> 30

struct Cube;
struct GridNode;
typedef GridNode *GridNodePtr;

struct Cube {
  // Eigen::Vector3d p1, p2, p3, p4, p5, p6, p7, p8;   // the 8 m_vertex of a
  // cube
  Eigen::MatrixXd m_vertex; // size: 8*3
  Eigen::Vector3d m_center; // the m_center of the cube
  bool m_valid;             // indicates whether this cube should be deleted
  double m_radius; // could use m_center and m_radius generate a cube for test
  double m_t;      // time allocated to this cube
  std::vector<std::pair<double, double>> box;
  /*
             P3------------P2
             /|           /|              ^
            / |          / |              | z
          P0--|---------P1 |              |
           |  P7--------|--p6             |
           | /          | /               /--------> y
           |/           |/               /
          P4------------P5              / x
  */

  // create a cube using 8 m_vertex and the m_center point
  Cube(Eigen::MatrixXd vertex_, Eigen::Vector3d center_) {
    m_vertex = vertex_;
    m_center = center_;
    m_valid = true;
    m_t = 0.0;
    box.resize(3);
  }
  Cube(Eigen::Vector3d center_, double radius) {
    m_center = center_;
    setRadius(radius);
    m_vertex = Eigen::MatrixXd::Zero(8, 3);
    generateVertexFromRadius();
    m_valid = true;
    m_t = 0.0;
    box.resize(3);
  }
  Cube() {
    m_center = Eigen::VectorXd::Zero(3);
    m_vertex = Eigen::MatrixXd::Zero(8, 3);
    m_valid = true;
    m_t = 0.0;
    box.resize(3);
  }

  ~Cube() {}

  // create a inscribe cube of a ball using the m_center point and the m_radius
  // of the ball
  void setVertex(Eigen::MatrixXd vertex_, double resolution_) {
    m_vertex = vertex_;
    m_vertex(0, 1) -= resolution_ / 2.0;
    m_vertex(3, 1) -= resolution_ / 2.0;
    m_vertex(4, 1) -= resolution_ / 2.0;
    m_vertex(7, 1) -= resolution_ / 2.0;

    m_vertex(1, 1) += resolution_ / 2.0;
    m_vertex(2, 1) += resolution_ / 2.0;
    m_vertex(5, 1) += resolution_ / 2.0;
    m_vertex(6, 1) += resolution_ / 2.0;

    m_vertex(0, 0) += resolution_ / 2.0;
    m_vertex(1, 0) += resolution_ / 2.0;
    m_vertex(4, 0) += resolution_ / 2.0;
    m_vertex(5, 0) += resolution_ / 2.0;

    m_vertex(2, 0) -= resolution_ / 2.0;
    m_vertex(3, 0) -= resolution_ / 2.0;
    m_vertex(6, 0) -= resolution_ / 2.0;
    m_vertex(7, 0) -= resolution_ / 2.0;

    m_vertex(0, 2) += resolution_ / 2.0;
    m_vertex(1, 2) += resolution_ / 2.0;
    m_vertex(2, 2) += resolution_ / 2.0;
    m_vertex(3, 2) += resolution_ / 2.0;

    m_vertex(4, 2) -= resolution_ / 2.0;
    m_vertex(5, 2) -= resolution_ / 2.0;
    m_vertex(6, 2) -= resolution_ / 2.0;
    m_vertex(7, 2) -= resolution_ / 2.0;

    setBox();
  }
  void setRadius(double radius) { m_radius = radius; }
  void generateVertexFromRadius() {
    m_vertex(0, 0) = m_center(0) + m_radius;
    m_vertex(1, 0) = m_center(0) + m_radius;
    m_vertex(4, 0) = m_center(0) + m_radius;
    m_vertex(5, 0) = m_center(0) + m_radius;

    m_vertex(2, 0) = m_center(0) - m_radius;
    m_vertex(3, 0) = m_center(0) - m_radius;
    m_vertex(6, 0) = m_center(0) - m_radius;
    m_vertex(7, 0) = m_center(0) - m_radius;

    m_vertex(0, 1) = m_center(1) - m_radius;
    m_vertex(3, 1) = m_center(1) - m_radius;
    m_vertex(4, 1) = m_center(1) - m_radius;
    m_vertex(7, 1) = m_center(1) - m_radius;

    m_vertex(1, 1) = m_center(1) + m_radius;
    m_vertex(2, 1) = m_center(1) + m_radius;
    m_vertex(5, 1) = m_center(1) + m_radius;
    m_vertex(6, 1) = m_center(1) + m_radius;

    m_vertex(0, 2) = m_center(2) + m_radius;
    m_vertex(1, 2) = m_center(2) + m_radius;
    m_vertex(2, 2) = m_center(2) + m_radius;
    m_vertex(3, 2) = m_center(2) + m_radius;

    m_vertex(4, 2) = m_center(2) - m_radius;
    m_vertex(5, 2) = m_center(2) - m_radius;
    m_vertex(6, 2) = m_center(2) - m_radius;
    m_vertex(7, 2) = m_center(2) - m_radius;
    setBox();
  }

  // 设置Box的长宽高
  void setBox() {
    box.clear();
    box.resize(3);
    box[0] = std::make_pair(m_vertex(3, 0), m_vertex(0, 0));
    box[1] = std::make_pair(m_vertex(0, 1), m_vertex(1, 1));
    box[2] = std::make_pair(m_vertex(4, 2), m_vertex(1, 2));
  }

  void printBox() {
    std::cout << "m_center of the cube: \n" << m_center << std::endl;
    std::cout << "m_vertex of the cube: \n" << m_vertex << std::endl;
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct GridNode {
  int id; // 1--> open set, -1 --> closed set
  Eigen::Vector3d coord;
  Eigen::Vector3i index;

  double gScore, fScore;
  GridNodePtr cameFrom;
  std::multimap<double, GridNodePtr>::iterator nodeMapIt;
  double occupancy;

  std::vector<GridNodePtr>
      hisNodeList; // use a list to record nodes in its history

  GridNode(Eigen::Vector3i _index) {
    id = 0;
    index = _index;

    gScore = inf;
    fScore = inf;
    cameFrom = NULL;
  }

  GridNode(Eigen::Vector3i _index, Eigen::Vector3d _coord) {
    id = 0;
    index = _index;
    coord = _coord;

    gScore = inf;
    fScore = inf;
    cameFrom = NULL;
  }

  GridNode(){};

  ~GridNode(){};
};

#endif