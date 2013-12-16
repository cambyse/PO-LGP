#include <Ors/ors.h>
#include <sstream>


struct WritheTaskVariable:public DefaultTaskVariable{
  const char* obj_name;
  int param;
  int segments1;
  int segments2;
  
  WritheTaskVariable(const char* _name,ors::KinematicWorld& _ors,
                                         const char* _obj_name,
				         int _segments1,int _segments2,
				         int _param);
  virtual TaskVariable* newClone(){ return new WritheTaskVariable(*this); }
  virtual void userUpdate(const ors::KinematicWorld& ors);
  virtual void epsilon_check(arr& delta_q, const ors::KinematicWorld& ors);
  virtual void delta_check(arr& delta_q, const ors::KinematicWorld& ors);
};

void plot_writhe(arr WM,int dim1,int dim2s);
