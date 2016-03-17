#pragma once

#include <Core/array.h>
#include <Eigen/Dense>

arr eigen2mt(const Eigen::MatrixXd& in);
Eigen::MatrixXd mt2eigen(const arr& in);
