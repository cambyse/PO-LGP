#ifndef INNERCOSTFUNCTION_H
#define INNERCOSTFUNCTION_H
#include <Ors/ors.h>
#include <Motion/taskMaps.h>


struct InnerCostFunction{
  virtual void setParam(const arr &param,const ors::KinematicWorld &world,uint T) = 0;
  virtual ~InnerCostFunction(){}
  uint numParam;
  MT::Array<Task*> taskCosts;
};

struct SimpleICF:InnerCostFunction{
  SimpleICF(ors::KinematicWorld world);
  virtual void setParam(const arr &param,const ors::KinematicWorld &world,uint T);
};

#endif // INNERCOSTFUNCTION_H
