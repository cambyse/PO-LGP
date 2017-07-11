#ifndef INNERCOSTFUNCTION_H
#define INNERCOSTFUNCTION_H
#include <Kin/kin.h>
#include <Kin/taskMaps.h>


struct InnerCostFunction{
  virtual void setParam(const arr &param,const mlr::KinematicWorld &world,uint T) = 0;
  virtual ~InnerCostFunction(){}
  uint numParam;
  mlr::Array<Task*> taskCosts;
};

struct SimpleICF:InnerCostFunction{
  SimpleICF(mlr::KinematicWorld world);
  virtual void setParam(const arr &param,const mlr::KinematicWorld &world,uint T);
};

#endif // INNERCOSTFUNCTION_H
