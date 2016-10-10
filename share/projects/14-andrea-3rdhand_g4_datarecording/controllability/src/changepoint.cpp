#include "changepoint.h"

struct sChangePoint_offline {
  arr Q, P, Pcp, ePcp, pcp;
};


ChangePoint_offline::ChangePoint_offline() {
  s = new sChangePoint_offline;
}

ChangePoint_offline::~ChangePoint_offline() {
  delete s;
}

void ChangePoint_offline::detect(const arr &data, function<double(uint)> prior_function, function<double(arr, uint, uint)> data_loglike_function) {
  uint n = data.d0;

  arr g(n), G(n);
  arr Q(n), P(n, n), Pcp(n, n);

  g.setZero();
  G.setZero();
  Q.setZero();
  P.setZero(1);
  Pcp.setZero(1);

  for(int t = 0; t < n; t++)
    for(int s = t; s < n; s++)
      P(t, s) = data_loglike_function(data, t, s+1);

  //precompute g and G
  g(0) = log(prior_function(0));
  G(0) = g(0);
  for(int t = 1; t < n; t++) {
    g(t) = log(prior_function(t));
    G(t) = log(exp(G(t-1)) + exp(g(t)));
  }

  // computes Q and P
  //P(n-1, n-1) = data_loglike_function(data, n-1, n);
  for(int t = n-2; t >= 0; t--) {
    double P_next_cp = 1;
    for(int s = t; s < n-1; s++) {
      //P(t, s) = data_loglike_function(data, t, s+1);
      double summand = P(t, s) + Q(s+1) + g(s+1-t);
      if(P_next_cp == 1)
        P_next_cp = summand;
      else
        P_next_cp = log(exp(P_next_cp) + exp(summand));
      //if(summand - P_next_cp < truncate)
        //break;
    }
    //if(P(t, n-1) == 1)
      //P(t, n-1) = data_loglike_function(data, t, n);

    double antiG;
    if(G(n-1-t) < -1e-15)
      antiG = log(1-exp(G(n-1-t)));
    else
      antiG = log(-G(n-1-t));
    Q(t) = log(exp(P_next_cp) + exp(P(t, n-1)+antiG));
  }

  // Computes other stuff
  Pcp = ones(n-1, n-1);
  for(int t = 0; t < n-1; t++) {
    //if(P(0, t) == 1)
      //P(0, t) = data_loglike_function(data, 0, t+1);
    Pcp(0, t) = P(0, t) + Q(t+1) + g(t) - Q(0);
  }
  for(int j = 1; j < n-1; j++) {
    for(int t = j; t < n-1; t++) {
      for(int tau = j-1; tau < t; tau++) {
        //if(P(tau+1, t) == 1)
          //P(tau+1, t) = data_loglike_function(data, tau+1, t+1);
        double tmp_cond = P(tau+1, t) + Q(t+1) + g(t-tau) - Q(tau+1);
        if(Pcp(j-1, tau) != 1)
          tmp_cond += Pcp(j-1, tau);

        if(Pcp(j, t) == 1)
          Pcp(j, t) = tmp_cond;
        else
          Pcp(j, t) = log(exp(Pcp(j, t)) + exp(tmp_cond));
      }
    }
  }

  s->P.takeOver(P);
  s->Q.takeOver(Q);
  s->Pcp.takeOver(Pcp);

  s->ePcp = exp(Pcp);
  for(uint i = 0; i < Pcp.d0; i++)
    for(uint j = 0; j < i; j++)
      s->ePcp(i, j) = 0;

  s->pcp = sum(s->ePcp, 0);
}

arr& ChangePoint_offline::P() { return s->P; };
arr& ChangePoint_offline::Q() { return s->Q; };
arr& ChangePoint_offline::Pcp() { return s->Pcp; };
arr& ChangePoint_offline::ePcp() { return s->ePcp; };
arr& ChangePoint_offline::pcp() { return s->pcp; };

