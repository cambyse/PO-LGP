#include "net.h"
#include "functions.h"

void Variable::write(ostream& os) const {
  os <<"{ dim=<" <<dim <<">";
  os <<"\n   x="; value.write(os,NULL,NULL,"[]",true);
  os <<"\n del="; del.write(os,NULL,NULL,"[]",true);
  os <<"\n   J="; J.write(os,NULL,NULL,"[]",true);
  os <<"  }" <<endl;
}

Variable* Net::newConstant(const char* key, const uintA& dim){
  Node_typed<Variable> *n = G.newNode<Variable>({key, "const", STRING(dim)}, {});
  n->value.f = new Constant();
  n->value.n = n;
  n->value.dim = dim;
  n->value.type = new Type_typed<Constant>();
  return &n->value;
}

Variable* Net::newFunction(const char* key, const VariableL& parents, Function* f, uintA dim){
  NodeL par(parents.N);
  for(uint i=0;i<par.N;i++) par(i) = parents(i)->n;
  Node_typed<Variable> *n = G.newNode<Variable>({key, typeid(Function).name(), STRING(dim)}, par);
  n->value.f = f;
  n->value.n = n;
  n->value.dim = dim;
  return &n->value;
}

void Net::fwdCompute(){
  for(Node *n:G){
    Variable& var = n->get<Variable>();

    //-- collect parent values
    arrA &y = var.in;
    y.resize(n->parents.N);
    uint i=0;
    for(Node *p : n->parents){
      Variable& var_p = p->get<Variable>();
      y(i).referTo(var_p.value);
      i++;
    }

    //-- compute function
    var.f->fwd(var.value, y);
  }
}

void Net::bwdCompute(){
  //-- initialize all gradients with the partial derivative
  for(Node *n:G){
    Variable& var = n->get<Variable>();
    var.J = var.del;
  }

  for(uint i=G.N;i--;){
    Variable& var = G.elem(i)->get<Variable>();
    var.f->bwd(var.Jin, var.J, var.value, var.in);

    //-- push back to parents
    uint j=0;
    for(Node *p : G.elem(i)->parents){
      Variable& var_p = p->get<Variable>();
      var_p.J += var.Jin(j);
      j++;
    }
  }
}

const arr& Net::getValue(Variable* n){
  return n->value;
}

void Net::zeroAllPartialDerivatives(uint d){
  for(Node *n:G){
    Variable& var = n->get<Variable>();
    uintA dim = TUP(d);
    dim.append(var.dim);
    var.del.resize(dim).setZero();
  }
}

void Net::setPartialDerivative(Variable* n, const arr& del){
  n->del = del;
}

const arr& Net::getTotalDerivative(Variable* n){
  return n->J;
}

void Net::randConstants(){
  for(Node *n:G){
    Variable& var = n->get<Variable>();
    Constant *f = dynamic_cast<Constant*>(var.f);
    if(f){
      uintA dim;
      f->c = randn(var.dim);
    }
  }
}

arr Net::getAllConstants(){
  arr w;
  for(Node *n:G){
    Variable& var = n->get<Variable>();
    Constant *f = dynamic_cast<Constant*>(var.f);
    if(f) w.append(f->c);
  }
  return w;
}

arr Net::getAllConstantJacobians(uint ddim, uint wdim){
  arr J(ddim, wdim);
  uint i=0;
  for(Node *n:G){
    Variable& var = n->get<Variable>();
    Constant *f = dynamic_cast<Constant*>(var.f);
    if(f){
      CHECK_EQ(ddim, var.J.d0, "");
      arr Jref;
      Jref.referTo(var.J);
      Jref.reshape(ddim, f->c.N);
      J.setMatrixBlock(Jref, 0, i);
      i += Jref.d1;
    }
  }
  CHECK_EQ(i, wdim, "not right number of constants");
  return J;
}

void Net::setAllConstants(const arr& w){
  uint i=0;
  for(Node *n:G){
    Variable& var = n->get<Variable>();
    Constant *f = dynamic_cast<Constant*>(var.f);
    if(f){
      CHECK(i+f->c.N <= w.N, "not enough constants");
      memmove(f->c.p, w.p+i, f->c.N*w.sizeT);
//      f->c.setCarray(w.p+i, f->c.N);
      i += f->c.N;
    }
  }
  CHECK_EQ(i, w.N, "not right number of constants");
}

void Net::reportAllConstants(){
  uint i=0;
  for(Node *n:G){
    Variable& var = n->get<Variable>();
    Constant *f = dynamic_cast<Constant*>(var.f);
    if(f){
      cout <<i <<' ' <<*var.n <<endl;
      i += f->c.N;
    }
  }
  cout <<"#constants = " <<i <<endl;
}

void Net::write(ostream& os) const{
  for(Node *n:G){
    Variable& var = n->get<Variable>();
    os <<var <<endl;
  }
}



