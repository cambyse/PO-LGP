#include "MT/ors.h"
#include <sstream>


struct WritheTaskVariable:public DefaultTaskVariable{
  const char* obj_name;
  int param;
  int segments1;
  int segments2;
  
  WritheTaskVariable(const char* _name,ors::Graph& _ors,
                                         const char* _obj_name,
				         int _segments1,int _segments2,
				         int _param);
  virtual TaskVariable* newClone(){ return new WritheTaskVariable(*this); }
  virtual void userUpdate();
  virtual void epsilon_check(arr& delta_q);
  virtual void delta_check(arr& delta_q);
};

void plot_writhe(arr WM,int dim1,int dim2s);