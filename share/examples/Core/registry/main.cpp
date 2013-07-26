#include <Core/registry.h>

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
REGISTER_ITEM(double, mykey, 3.)
REGISTER_TYPE_DERIVED(Derived,NewType)



int main(int argn,char** argv){

  cout <<"** REGISTRY:\n" <<registry() <<endl;

  cout <<"** derived from TypeRegistration:\n";
  listWrite(registry().getDerivedValues<Type>(), cout, "\n");
  cout <<endl;

  return 0;
}