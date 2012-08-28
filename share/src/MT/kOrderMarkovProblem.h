#ifndef kOrderMarkovProblem_h
#define kOrderMarkovProblem_h

#include <MT/array.h>
#include <MT/optimization.h>

//A $k$-order Markov problems is fully defined by the following functions:
struct KOrderMarkovFunction {
  //returns the function $\phi_t$ that defines a k-order Markov process
  //  argument $\phi$ is the returned vector
  //  if &J==NULL (that is, NoArr was given as parameter), then J is NOT returned
  //  otherwise the Jacobian of $\phi$ at $\bar x$ is returned
  //  t indexes the factor
  //  x_bar is a (k+1)-times-n array that contains the states $x_{t:t+k}$
  virtual void phi_t(arr& phi, arr& J, uint t, const arr& x_bar) = 0;

  //functions to get the parameters $T$, $k$ and $n$ of the $k$-order Markov Process
  virtual uint get_T() = 0;
  virtual uint get_k() = 0;
  virtual uint get_n() = 0; //the dimensionality of $x_t$
  virtual uint get_m(uint t) = 0; //the dimensionality of $\phi_t$
};

struct conv_KOrderMarkovFunction:VectorFunction {
  KOrderMarkovFunction *f;
  conv_KOrderMarkovFunction(KOrderMarkovFunction& _f):f(&_f) {}
  void fv(arr& y, arr& J, const arr& x);
};


#endif
