#include "modules.h"
#include <MT/mapGraph.h>

//===========================================================================
//
// declaration of a module
//

struct ArrRepresentation:arr,TypeBase{};
struct DoubleRepresentation:arr,TypeBase{};

//-- this is how the developer should provide a module
struct ComputeSum: MODULE_BASE(ComputeSum){
  VAR(ArrRepresentation, x);    //input
  VAR(DoubleRepresentation, s); //output

  ComputeSum(){}          //replaces old 'open'
  virtual ~ComputeSum(){} //replaces old 'close'

  virtual void step();
  virtual bool test(){ return true; }
};

void ComputeSum::step(){
  get_s().resize(1); // = ARR( sum(get_x()) );
}


//===========================================================================
//
// testing
//

template<class T>
void testModule(){
  Item *modit = registry().getTypedItem<TypeRegistration_typed<T, void> >("unit");
  ItemL children = modit->parentOf;
  cout <<"testModule: " <<*modit <<endl;

  T *mod = dynamic_cast<T*>(modit->value<TypeRegistration_typed<T, void> >().newInstance());

  MapGraph container;

  for_list_(Item, it, children){
    CHECK(it->keys(0)=="unit","??");
    TypeRegistration *val = NULL;
    if(it->is_derived_from_TypeBase()){
      val = dynamic_cast<TypeRegistration*>(&(((Item_typed<TypeBase>*)it)->value));
    }
    if(val){
      cout <<"element:" <<*val <<endl;
    }else{
      MT_MSG("strange element:" <<*it);
    }

    void *var = val->newInstance();
  }
}

//-- this is how the top-level manager should get access
int main(int argc, char** argv){

  cout <<"**** ENTER_MAIN" <<endl;

  cout <<registry() <<endl;

  testModule<ComputeSum>();

  return 0;
}


