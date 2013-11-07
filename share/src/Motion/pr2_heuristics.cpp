#include "pr2_heuristics.h"

arr pr2_zero_pose(){
  arr q(pr2_q_dim());
  q.setZero();
  q(30) = .5;
  return q;
}

arr pr2_reasonable_W(){
  arr W(pr2_q_dim());
  W=1.;
  W(12)=1e2;
  return W;
}
