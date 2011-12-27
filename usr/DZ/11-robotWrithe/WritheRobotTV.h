#include "MT/ors.h"
#include <sstream>


struct WritheTaskVariable:public DefaultTaskVariable{
  const char* obj_name;
  int param;
  int segments;
  
  WritheTaskVariable(const char* _name,ors::Graph& _ors,
                                         const char* _obj_name,
				         int _segments,
				         int _param);
  virtual TaskVariable* newClone(){ return new WritheTaskVariable(*this); }
  virtual void userUpdate();
  virtual void epsilon_check(arr& delta_q);
  virtual void delta_check(arr& delta_q);
};

void plot_writhe(arr WM,int dim);