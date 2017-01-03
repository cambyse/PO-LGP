#include "net.h"

//===========================================================================

struct Constant : Function{
  arr c;
  bool isParameter;
  Constant() : isParameter(false) {}
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

//===========================================================================

struct Difference : Function{
  virtual void fwd(arr& out, const arrA& in);
  virtual void bwd(arrA& Jin, const arr& Jout, const arr& out, const arrA& in);
};

//===========================================================================

struct HighPassIntegrator : Function{
  virtual void fwd(arr& out, const arrA& in);
  virtual void bwd(arrA& Jin, const arr& Jout, const arr& out, const arrA& in);
};

//===========================================================================

struct LowPassFilter : Function{
  virtual void fwd(arr& out, const arrA& in);
  virtual void bwd(arrA& Jin, const arr& Jout, const arr& out, const arrA& in);
};

//===========================================================================

struct Concat : Function{
  virtual void fwd(arr& out, const arrA& in);
  virtual void bwd(arrA& Jin, const arr& Jout, const arr& out, const arrA& in);
};
