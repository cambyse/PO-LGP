#include "util.h"
#include <Core/array.h>
#include <Eigen/Dense>

arr eigen2mt(const Eigen::MatrixXd& in) {
  arr out(in.rows(), in.cols());
  for(uint i = 0; i<in.rows(); i++)
    for(uint j = 0; j<in.cols(); j++)
      out(i, j) = in(i, j);
  return out; 
}

Eigen::MatrixXd mt2eigen(const arr& in) {
  if(in.nd == 1) {
    Eigen::MatrixXd out(in.d0, 1);  
    for(uint i = 0; i<in.d0; i++)
        out(i, 0) = in(i);
    return out;
  }
  else if(in.nd == 2) {
    Eigen::MatrixXd out(in.d0, in.d1);  
    for(uint i = 0; i<in.d0; i++)
      for(uint j = 0; j<in.d1; j++)
        out(i, j) = in(i, j);
    return out;
  }
}

