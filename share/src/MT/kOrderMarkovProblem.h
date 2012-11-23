#ifndef kOrderMarkovProblem_h
#define kOrderMarkovProblem_h

#include "optimization.h"

//A $k$-order Markov problems is fully defined by the following functions:

//returns the function $\phi_t$ that defines a k-order Markov process
//  argument $\phi$ is the returned vector
//  if &J==NULL (that is, NoArr was given as parameter), then J is NOT returned
//  otherwise the Jacobian of $\phi$ at $\bar x$ is returned
//  t indexes the factor
//  x_bar is a (k+1)-times-n array that contains the states $x_{t:t+k}$



#endif
