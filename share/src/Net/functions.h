#include "net.h"

//===========================================================================

struct Constant : Function{
  arr c;
  virtual void fwd(arr& out, const arrA& in);
  virtual void bwd(arrA& Jin, const arr& Jout, const arr& out, const arrA& in);
};

//===========================================================================

struct Sigmoid : Function {
  virtual void fwd(arr& out, const arrA& in);
  virtual void bwd(arrA& Jin, const arr& Jout, const arr& out, const arrA& in);
};

//===========================================================================

struct Linear : Function {
  virtual void fwd(arr& out, const arrA& in);
  virtual void bwd(arrA& Jin, const arr& Jout, const arr& out, const arrA& in);
};
