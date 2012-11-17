#include <MT/util.h>
#include <memory>

//-- two abstract types and a method
struct A{
  virtual const char* name()=0;
};

struct B{
  virtual const char* string()=0;
};

struct C{
  virtual const char* type()=0;
};

void write(A& a){ cout <<a.name() <<endl; }


//-- generic converter between all abstractions
struct Convert{
  struct sConvert* s;
  Convert(A&);
  Convert(B&);
  Convert(C&);
  ~Convert();
  operator A&();
  operator B&();
  operator C&();
};


//-- generic implementation of C
struct Cderiv:C{
  const char* type(){ return typeid(*this).name(); }
};

int main(int argn, char**argv){
  Cderiv c;
  write(Convert(c));
  return 0;
};

//******************************************************************************


struct sConvert{
  A* a;
  B* b;
  C* c;
  struct B_A:A{ //actual converter objects
    B *b;
    B_A(B& _b):b(&_b){}
    const char* name(){ return b->string(); }
  } *b_a;
  struct C_B:B{ //actual converter objects
    C *c;
    C_B(C& _c):c(&_c){}
    const char* string(){ return c->type(); }
  } *c_b;
  sConvert():a(NULL),b(NULL),c(NULL),b_a(NULL),c_b(NULL) {};
};

//the Convert is essentially only a ``garb_age collector'', creating all the necessary conversion objects and then deleting them on destruction
Convert::Convert(A& a){  s = new sConvert();  s->a = &a;  }
Convert::Convert(B& b){  s = new sConvert();  s->b = &b;  }
Convert::Convert(C& c){  s = new sConvert();  s->c = &c;  }
Convert::~Convert(){
  if(s->c_b) delete s->c_b;
  if(s->b_a) delete s->b_a;
  delete s;
}
Convert::operator A&(){
  if(!s->a){
    if(!s->b) operator B&();
    s->a = new sConvert::B_A(*s->b);
  }
  return *s->a;
}
Convert::operator B&(){
  if(!s->b){
    if(!s->c) HALT("");
    s->b = new sConvert::C_B(*s->c);
  }
  return *s->b;
}
Convert::operator C&(){
  if(!s->c){
    HALT("");
  }
  return *s->c;
}
