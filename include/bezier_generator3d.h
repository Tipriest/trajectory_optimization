#pragma once

#include "bezier_base.h"
#include "data_type.h"
#include "mosek.h"
#include <fstream>
#include <iostream>
#include <ros/console.h>
#include <ros/ros.h>
#include <stdio.h>
#include <string>

using namespace std;
using namespace Eigen;

/* Use Bezier curve for the trajectory */
int BezierPloyCoeffGeneration2D(
    const vector<Cube> &corridor, const MatrixXd &MQM, const MatrixXd &pos,
    const MatrixXd &vel, const MatrixXd &acc, const double maxVel,
    const double maxAcc, const int traj_order, const double minimize_order,
    const double margin, const bool &isLimitVel, const bool &isLimitAcc,
    double &obj, MatrixXd &PolyCoeff);
int BezierPloyCoeffGeneration3D(
    const vector<Cube> &corridor, const MatrixXd &MQM, const MatrixXd &pos,
    const MatrixXd &vel, const MatrixXd &acc, const double maxVel,
    const double maxAcc, const int traj_order, const double minimize_order,
    const double margin, const bool &isLimitVel, const bool &isLimitAcc,
    double &obj, MatrixXd &PolyCoeff);
