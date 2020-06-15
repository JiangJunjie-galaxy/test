#pragma once
#include <Eigen/Dense>

using namespace Eigen;

class motion
{
  public:
  double x;
  double y;
  double theta;
};

class odom_parameter
{
  public:
  double rot1;
  double trans;
  double rot2;
};

Matrix<double, 3, 3> m_cov;
Matrix<double, 3, 1> m_mu;
Matrix<double, 3, 3> primitive_cov;
