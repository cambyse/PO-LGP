#include "gp_control.h"
#include <Core/util.h>

GPControl::GPControl() {
  n = 7;
  // load matrices for gp prediction
  for (uint i = 1; i<=n ; i++){
    arr l;
    l << FILE(STRING("../../pr2_record_data/data/prediction_data/out/lambda_"<<i));
    lambdas.append(l);

    arr a;
    a << FILE(STRING("../../pr2_record_data/data/prediction_data/out/alpha_"<<i));
    alphas.append(a);

    arr x;
    x << FILE(STRING("../../pr2_record_data/data/prediction_data/out/x_subset_"<<i));
    x_subsets.append(x);
  }
}

void GPControl::predict(arr &state, arr &pred) {
  pred.clear();

  // map roll joints to range [-PI,PI]
  state(4) = fmod(state(4)+M_PI, 2.*M_PI) - M_PI;
  state(6) = fmod(state(6)+M_PI, 2.*M_PI) - M_PI;

  for (uint i = 0; i<n ; i++){
    arr y ;
    y = x_subsets(i) - repmat(~state,x_subsets(i).d0,1);
    y = y%y;
    y = y *lambdas(i);
    y = exp(-y);
    y = ~alphas(i)*y;
    pred.append(y(0,0));
  }
}
