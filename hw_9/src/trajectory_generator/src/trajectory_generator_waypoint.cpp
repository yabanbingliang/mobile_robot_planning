#include "trajectory_generator_waypoint.h"
#include <fstream>
#include <iostream>
#include <ros/console.h>
#include <ros/ros.h>
#include <stdio.h>
#include <string>

using namespace std;
using namespace Eigen;

#define inf 1 >> 30

TrajectoryGeneratorWaypoint::TrajectoryGeneratorWaypoint() {}

TrajectoryGeneratorWaypoint::~TrajectoryGeneratorWaypoint() {}

Eigen::MatrixXd TrajectoryGeneratorWaypoint::PolyQPGeneration(
  const int d_order,           // the order of derivative
  const Eigen::MatrixXd& Path, // waypoints coordinates (3d)
  const Eigen::MatrixXd& Vel,  // boundary velocity
  const Eigen::MatrixXd& Acc,  // boundary acceleration
  const Eigen::VectorXd& Time) // time allocation in each segment
{
  // enforce initial and final velocity and accleration, for higher order
  // derivatives, just assume them be 0;
  int p_order = 2 * d_order - 1; // the order of polynomial
  int p_num1d = p_order + 1;     // the number of variables in each segment

  int m = Time.size();
  MatrixXd PolyCoeff(m, 3 * p_num1d);

  // std::cout << "PolyCoeff shape: " << PolyCoeff.rows() << " x " << PolyCoeff.cols() << std::endl;

  /**
   *
   * STEP 3.2:  generate a minimum-snap piecewise monomial polynomial-based
   * trajectory
   *
   * **/

  int pieceNum = Path.rows() - 1;


  // Get the first point from Path
  Eigen::Vector3d initialPos = Path.row(0);

  // Get the last point from Path
  Eigen::Vector3d terminalPos = Path.row(Path.rows() - 1);



  /* way1 闭式解 */

  Eigen::Vector3d initialVel = Vel.row(0);
  Eigen::Vector3d initialAcc = Acc.row(0);
  Eigen::Vector3d terminalVel = Vel.row(1);
  Eigen::Vector3d terminalAcc = Acc.row(1);

  Eigen::MatrixXd M = Eigen::MatrixXd::Zero(8 * pieceNum, 8 * pieceNum);
  Eigen::MatrixXd b = Eigen::MatrixXd::Zero(8 * pieceNum, 3);

  // START
  Eigen::MatrixXd F_0(4, 8);
  F_0 << 1, 0, 0, 0, 0, 0, 0, 0,
    0, 1, 0, 0, 0, 0, 0, 0,
    0, 0, 2, 0, 0, 0, 0, 0,
    0, 0, 0, 6, 0, 0, 0, 0;
  M.block(0, 0, 4, 8) = F_0;
  b.block(0, 0, 4, 3) << initialPos(0), initialPos(1), initialPos(2),
    initialVel(0), initialVel(1), initialVel(2),
    initialAcc(0), initialAcc(1), initialAcc(2),
    0, 0, 0; //最后一行为jerk

  // std::cout << "F_0" << std::endl;
  // std::cout << M.block(0, 0, 4, 8) << std::endl;

  // std::cout << "b_0" << std::endl;
  // std::cout << b.block(0, 0, 4, 3) << std::endl;

  // cout << "pieceNum = " << pieceNum << endl;
  // for (int i = 0; i < m; i++) {
  //   cout << "Time[" << i << "] = " << Time(i) << endl;
  // }

  // cout << "Time(pieceNum - 1) = " << Time(pieceNum - 1) << endl;

  // END
  double t(Time(pieceNum - 1)); // 最后一段时间
  Eigen::MatrixXd E_M(4, 8);
  E_M << 1, t, pow(t, 2), pow(t, 3), pow(t, 4), pow(t, 5), pow(t, 6), pow(t, 7),
    0, 1, 2 * t, 3 * pow(t, 2), 4 * pow(t, 3), 5 * pow(t, 4), 6 * pow(t, 5), 7 * pow(t, 6),
    0, 0, 2, 6 * t, 12 * pow(t, 2), 20 * pow(t, 3), 30 * pow(t, 4), 42 * pow(t, 5),
    0, 0, 0, 6, 24 * t, 60 * pow(t, 2), 120 * pow(t, 3), 210 * pow(t, 4);
  M.block(8 * pieceNum - 4, 8 * (pieceNum - 1), 4, 8) = E_M;
  b.block(8 * pieceNum - 4, 0, 4, 3) << terminalPos(0), terminalPos(1), terminalPos(2),
    terminalVel(0), terminalVel(1), terminalVel(2),
    terminalAcc(0), terminalAcc(1), terminalAcc(2),
    0, 0, 0; // 最后一行为jerk

  // std::cout << "E_M" << std::endl;
  // std::cout << M.block(8 * pieceNum - 4, 8 * (pieceNum - 1), 4, 8) << std::endl;

  // std::cout << "b_M" << std::endl;
  // std::cout << b.block(8 * pieceNum - 4, 0, 4, 3) << std::endl;

  // cout << "start & end is ok!" << endl;

  for (int i = 1; i < pieceNum; i++) { //第二个到倒数第二个
    double t(Time(i - 1));
    Eigen::MatrixXd F_i(8, 8), E_i(8, 8);

    // Eigen::Vector3d D_i(intermediatePositions.transpose().row(i - 1));
    Eigen::Vector3d D_i(Path.row(i).transpose());

    E_i << 1, t, pow(t, 2), pow(t, 3), pow(t, 4), pow(t, 5), pow(t, 6), pow(t, 7),
      1, t, pow(t, 2), pow(t, 3), pow(t, 4), pow(t, 5), pow(t, 6), pow(t, 7),
      0, 1, 2 * t, 3 * pow(t, 2), 4 * pow(t, 3), 5 * pow(t, 4), 6 * pow(t, 5), 7 * pow(t, 6),
      0, 0, 2, 6 * t, 12 * pow(t, 2), 20 * pow(t, 3), 30 * pow(t, 4), 42 * pow(t, 5),
      0, 0, 0, 6, 24 * t, 60 * pow(t, 2), 120 * pow(t, 3), 210 * pow(t, 4),
      0, 0, 0, 0, 24, 120 * t, 360 * pow(t, 2), 840 * pow(t, 3),
      0, 0, 0, 0, 0, 120, 720 * t, 2520 * pow(t, 2),
      0, 0, 0, 0, 0, 0, 720, 5040 * t;
    F_i << 0, 0, 0, 0, 0, 0, 0, 0,
      -1, 0, 0, 0, 0, 0, 0, 0,
      0, -1, 0, 0, 0, 0, 0, 0,
      0, 0, -2, 0, 0, 0, 0, 0,
      0, 0, 0, -6, 0, 0, 0, 0,
      0, 0, 0, 0, -24, 0, 0, 0,
      0, 0, 0, 0, 0, -120, 0, 0,
      0, 0, 0, 0, 0, 0, -720, 0;
    int j = 8 * (i - 1);
    M.block(4 + 8 * (i - 1), j + 8, 8, 8) = F_i;
    M.block(4 + 8 * (i - 1), j, 8, 8) = E_i;
    b.block(4 + 8 * (i - 1), 0, 8, 3) << D_i(0), D_i(1), D_i(2),
      0, 0, 0,
      0, 0, 0,
      0, 0, 0,
      0, 0, 0,
      0, 0, 0,
      0, 0, 0,
      0, 0, 0;
  }
  Eigen::MatrixX3d coefficientMatrix;
  coefficientMatrix = M.inverse() * b;


  // std::cout << "Shape and size of M: " << M.rows() << "x" << M.cols() << std::endl;
  // std::cout << "Shape and size of b: " << b.rows() << "x" << b.cols() << std::endl;
  // std::cout << "Shape and size of coefficientMatrix: " << coefficientMatrix.rows() << "x" << coefficientMatrix.cols() << std::endl;
  // std::cout << "coefficientMatrix = " << std::endl;
  // std::cout << coefficientMatrix << std::endl;

  // coefficientMatrix 2 PolyCoeff
  // coefficientMatrix (pieceNum * 8) * 3， 每一行表示的是x y z的一个系数 每8行3列表示的是一段轨迹的x y z的多项式系数 从低到高
  // PolyCoeff pieceNum * (8 * 3) = pieceNum * 24，其中24表示的是一段轨迹的x y z的7阶多项式系数的个数，按照xyz和从低到高的顺序排列

  MatrixXd tmpPolyCoeff(m, 3 * p_num1d);
  // MatrixXd tmpPolyCoeff;

  for (int i = 0; i < m; ++i) {
    // 对于每个路径段
    for (int j = 0; j < p_num1d; ++j) {
      // j 表示当前多项式系数的索引
      // 这里我们需要将coefficientMatrix对应路径段的系数按顺序赋值到PolyCoeff中
      tmpPolyCoeff(i, j) = coefficientMatrix(i * 8 + j, 0); // X方向
      tmpPolyCoeff(i, j + p_num1d) = coefficientMatrix(i * 8 + j, 1); // Y方向
      tmpPolyCoeff(i, j + 2 * p_num1d) = coefficientMatrix(i * 8 + j, 2); // Z方向
    }
  }

  // 逆序赋值 不对
  // for (int i = 0; i < m; ++i) {
  //   // 对于每个路径段
  //   for (int j = 0; j < p_num1d; ++j) {
  //     // j 表示当前多项式系数的索引
  //     // 这里我们需要将coefficientMatrix对应路径段的系数按顺序赋值到PolyCoeff中
  //     tmpPolyCoeff(i, j) = coefficientMatrix(i * 8 + p_num1d - j - 1, 0); // X方向
  //     tmpPolyCoeff(i, j + p_num1d) = coefficientMatrix(i * 8 + p_num1d - j - 1, 1); // Y方向
  //     tmpPolyCoeff(i, j + 2 * p_num1d) = coefficientMatrix(i * 8 + p_num1d - j - 1, 2); // Z方向
  //   }
  // }


  // std::cout << "Shape and size of tmpPolyCoeff: " << tmpPolyCoeff.rows() << "x" << tmpPolyCoeff.cols() << std::endl;
  // std::cout << "tmpPolyCoeff = " << std::endl;
  // std::cout << tmpPolyCoeff << std::endl;

  PolyCoeff = tmpPolyCoeff;

  return PolyCoeff;
}

double TrajectoryGeneratorWaypoint::getObjective() {
  _qp_cost = (_Px.transpose() * _Q * _Px + _Py.transpose() * _Q * _Py +
    _Pz.transpose() * _Q * _Pz)(0);
  return _qp_cost;
}

Vector3d TrajectoryGeneratorWaypoint::getPosPoly(MatrixXd polyCoeff, int k,
  double t) {
  Vector3d ret;
  int _poly_num1D = (int)polyCoeff.cols() / 3;
  for (int dim = 0; dim < 3; dim++) {
    VectorXd coeff = (polyCoeff.row(k)).segment(dim * _poly_num1D, _poly_num1D);
    VectorXd time = VectorXd::Zero(_poly_num1D);

    for (int j = 0; j < _poly_num1D; j++)
      if (j == 0)
        time(j) = 1.0;
      else
        time(j) = pow(t, j);

    ret(dim) = coeff.dot(time);
    // cout << "dim:" << dim << " coeff:" << coeff << endl;
  }

  return ret;
}

Vector3d TrajectoryGeneratorWaypoint::getVelPoly(MatrixXd polyCoeff, int k,
  double t) {
  Vector3d ret;
  int _poly_num1D = (int)polyCoeff.cols() / 3;
  for (int dim = 0; dim < 3; dim++) {
    VectorXd coeff = (polyCoeff.row(k)).segment(dim * _poly_num1D, _poly_num1D);
    VectorXd time = VectorXd::Zero(_poly_num1D);

    for (int j = 0; j < _poly_num1D; j++)
      if (j == 0)
        time(j) = 0.0;
      else
        time(j) = j * pow(t, j - 1);

    ret(dim) = coeff.dot(time);
  }

  return ret;
}

Vector3d TrajectoryGeneratorWaypoint::getAccPoly(MatrixXd polyCoeff, int k,
  double t) {
  Vector3d ret;
  int _poly_num1D = (int)polyCoeff.cols() / 3;
  for (int dim = 0; dim < 3; dim++) {
    VectorXd coeff = (polyCoeff.row(k)).segment(dim * _poly_num1D, _poly_num1D);
    VectorXd time = VectorXd::Zero(_poly_num1D);

    for (int j = 0; j < _poly_num1D; j++)
      if (j == 0 || j == 1)
        time(j) = 0.0;
      else
        time(j) = j * (j - 1) * pow(t, j - 2);

    ret(dim) = coeff.dot(time);
  }

  return ret;
}