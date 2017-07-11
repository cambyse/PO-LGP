#include "net.h"
#include "functions.h"
#include <Optim/optimization.h>

void Variable::write(ostream& os) const {
  os <<"{ dim=<" <<dim <<">";
  os <<"\n   x="; value.write(os,NULL,NULL,"[]",true);
  os <<"\n del="; del.write(os,NULL,NULL,"[]",true);
  os <<"\n   J="; J.write(os,NULL,NULL,"[]",true);
  os <<"  }" <<endl;
}

Variable* Net::newConstant(const char* key, const uintA& dim, bool isParameter){
  Constant *c = new Constant();
  c->isParameter = isParameter;

  Node_typed<Variable> *n = G.newNode<Variable>({key, (isParameter?"param":"const"), STRING(dim)}, {});
  n->value.f = c;
  n->value.n = n;
  n->value.dim = dim;
  return &n->value;
}

Variable* Net::newConstant(const char* key, const arr& value, bool isParameter){
  Constant *c = new Constant();
  c->c = value;
  c->isParameter = isParameter;

  Node_typed<Variable> *n = G.newNode<Variable>({key, (isParameter?"param":"const"), STRING(value.dim())}, {});
  n->value.f = c;
  n->value.n = n;
  n->value.dim = value.dim();
  return &n->value;
}

Variable* Net::newFunction(const char* key, const VariableL& parents, Function* f, uintA dim, ObjectiveType ot){
  NodeL par(parents.N);
  for(uint i=0;i<par.N;i++){
    if(parents(i)) par(i) = parents(i)->n;
    else par(i) = NULL;
  }
  Node_typed<Variable> *n = G.newNode<Variable>({key, typeid(*f).name(), STRING(dim)}, par);
  if(ot) n->keys.append(ObjectiveTypeString[ot]);
  n->value.ot = ot;
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
    CHECK_EQ(var.value.dim(), var.dim, "dimension mismatch for variable " <<*var.n);
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

void Net::clearAllPartialDerivatives(){
  for(Node *n:G){
    Variable& var = n->get<Variable>();
    var.del.clear();
    var.J.clear();
  }
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

void Net::randParameters(){
  for(Node *n:G){
    Variable& var = n->get<Variable>();
    Constant *f = dynamic_cast<Constant*>(var.f);
    if(f && f->isParameter) f->c = randn(var.dim);
  }
}

arr Net::getAllParameters(){
  arr w;
  for(Node *n:G){
    Variable& var = n->get<Variable>();
    Constant *f = dynamic_cast<Constant*>(var.f);
    if(f && f->isParameter) w.append(f->c);
  }
  return w;
}

arr Net::getAllParameterJacobians(uint ddim, uint wdim){
  arr J(ddim, wdim);
  uint i=0;
  for(Node *n:G){
    Variable& var = n->get<Variable>();
    Constant *f = dynamic_cast<Constant*>(var.f);
    if(f && f->isParameter){
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

void Net::setAllParameters(const arr& w){
  uint i=0;
  for(Node *n:G){
    Variable& var = n->get<Variable>();
    Constant *f = dynamic_cast<Constant*>(var.f);
    if(f && f->isParameter){
      CHECK(i+f->c.N <= w.N, "not enough constants");
      memmove(f->c.p, w.p+i, f->c.N*w.sizeT);
//      f->c.setCarray(w.p+i, f->c.N);
      i += f->c.N;
    }
  }
  CHECK_EQ(i, w.N, "not right number of constants");
}

void Net::reportAllParameters(){
  uint i=0;
  for(Node *n:G){
    Variable& var = n->get<Variable>();
    Constant *f = dynamic_cast<Constant*>(var.f);
    if(f && f->isParameter){
      cout <<i <<" '" <<var.n->keys <<"' " <<f->c <<endl;
      i += f->c.N;
    }
  }
  cout <<"#constants = " <<i <<endl;
}

void Net::checkAllDerivatives(Variable* out){
  //-- set random weights
  randParameters();

  //-- define vector-valued function
  VectorFunction f =
      [this,out](arr& y, arr& J, const arr& x)->void{
    this->setAllParameters(x);
    this->fwdCompute();
    y = out->value;
    if(&J){
      this->zeroAllPartialDerivatives(y.N);
      out->del.setId();
      this->bwdCompute();
      J = this->getAllParameterJacobians(y.N, x.N);
    }
  };

  //-- check gradient w.r.t. all constants (input and weights
  arr w = getAllParameters();
  checkJacobian(f, w, 1e-4);
}

void Net::phi(arr& phi, arr& J, arr& H, ObjectiveTypeA& ot, const arr& x){
  setAllParameters(x);
  fwdCompute();
  phi.clear();
  if(&ot) ot.clear();
  if(&H) H.clear();
  for(Node *n:G){
    Variable& var = n->get<Variable>();
    if(var.ot){
      phi.append(var.value);
      if(&ot) ot.append(consts<ObjectiveType>(var.ot, var.value.N));
    }
  }
  uint N = phi.N;
  if(&J){
    //set all partial derivatives
    zeroAllPartialDerivatives(N);
    uint m=0;
    for(Node *n:G){
      Variable& var = n->get<Variable>();
      if(var.ot){
        var.del = zeros(N, var.value.N);
        var.del.setMatrixBlock(eye(var.value.N), m, 0);
        m += var.value.N;
      }
    }
    CHECK_EQ(m,N,"");

    //compute
    bwdCompute();

    //collect
    J = getAllParameterJacobians(N, x.N);
  }
}

void Net::write(ostream& os) const{
#if 1
  G.write(os);
#else
  for(Node *n:G){
    Variable& var = n->get<Variable>();
    os <<var <<endl;
  }
#endif
}





