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
REGISTER_TYPE_DERIVED(Derived, NewType)


void TEST(Registry){
  cout <<"** REGISTRY:\n" <<registry() <<endl;

  Item *it;
  it = reg_findType("NewType");
  CHECK(it, "could't retrieve type derived from NewType");
  cout <<"retrieved Type item: " <<*it <<endl;

  it = reg_findType("double");
  CHECK(it, "could't retrieve type derived from double");
  cout <<"retrieved Type item: " <<*it <<endl;

  it = reg_findType("Derived");
  CHECK(it, "could't retrieve type derived from Derived");
  cout <<"retrieved Type item: " <<*it <<endl;
}

int MAIN(int argn,char** argv){
  testRegistry();

  return 0;
}
