#define MT_IMPLEMENT_TEMPLATES
#include <MT/registry.h>

struct NewType{
  int x;
  void write(std::ostream& os) const{ os <<x <<endl; }
  void read(std::istream& is){ is >>x; }
};
stdPipes(NewType);

struct Derived:NewType{
  double y;
};

REGISTER_TYPE(NewType)
REGISTER_TYPE(double)
REGISTER_DERIVED_TYPE(Derived,NewType)

int main(int argn,char** argv){

  reg_report();

  cout <<"derived = ";
  listWrite(reg_findDerived<NewType>(), cout);

  return 0;
}
