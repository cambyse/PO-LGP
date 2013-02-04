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
REGISTER_ITEM(double, mykey, 3.)
REGISTER_TYPE_DERIVED(Derived,NewType)

// minimalistic explicit example for using the registrator tool

struct Container:Registrator<Container,void>{
  //void *dummy1(){ return reg.force(); } //don't need this if we also call forceSub
  void *dummy2(){ return staticRegistrator.forceSub<double>(); }
};


int main(int argn,char** argv){

  cout <<"** REGISTRY:\n" <<registry() <<endl;

  cout <<"** derived from TypeRegistration:\n";
  listWrite(registry().getDerivedItems<TypeRegistration>(), cout, "\n");
  cout <<endl;

  return 0;
}
