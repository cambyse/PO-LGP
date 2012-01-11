/*  
Copyright 2011   Tobias Lang

E-mail:    tobias.lang@fu-berlin.de

This file is part of libPRADA.

libPRADA is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

libPRADA is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with libPRADA.  If not, see <http://www.gnu.org/licenses/>.
*/


#include "logicObjectManager_database.h"


uint TL::LOGIC__MAX_LIMIT_OBJECT_ID = 100;

namespace TL {

// one per predicate, e.g. for "on(.,.)"
struct AtomList {
  Predicate* pred;
  AtomL mem;

  AtomList();
  AtomList(Predicate* pred);
  void init(Predicate* pred);
  void clear();
};

// one per predicate, e.g. for "on(.,.)"
struct LiteralList {
  Predicate* pred;
  AtomList atom_list;
  LitL mem;

  LiteralList();
  LiteralList(Predicate* pred);
  void init(Predicate* pred);
  void clear();
};




struct ComparisonAtomListConstant {
  Function* f;
  MT::Array< CompAtomL > mem;  // Outer index:  args;    Inner index: bound

  ComparisonAtomListConstant();
  ComparisonAtomListConstant(Function* f);
  void init(Function* f);
  void clear();
};

struct ComparisonLiteralListConstant {
  Function* f;
  MT::Array< CompLitL > mem;  // Outer index:  args;    Inner index: bound

  ComparisonLiteralListConstant();
  ComparisonLiteralListConstant(Function* f);
  void init(Function* f);
  void clear();
};


struct ComparisonAtomListDynamic {
  Function* f;
  CompAtomL mem;  // Index 1:  args

  ComparisonAtomListDynamic();
  ComparisonAtomListDynamic(Function* f);
  void init(Function* f);
  void clear();
};

struct ComparisonLiteralListDynamic {
  Function* f;
  CompLitL mem;  // Index 1:  args

  ComparisonLiteralListDynamic();
  ComparisonLiteralListDynamic(Function* f);
  void init(Function* f);
  void clear();
};




struct FunctionAtomList {
  Function* f;
  FuncAL mem;

  FunctionAtomList();
  FunctionAtomList(Function* f);
  void init(Function* f);
  void clear();
};

struct FunctionValueList {
  Function* f;
  MT::Array< FuncVL > mem;  // Index 1:  args;    index 2: bound
  
  FunctionValueList();
  FunctionValueList(Function* f);
  void init(Function* f);
  void clear();
};

}



// ------------------------------------------------------
// AtomList

TL::AtomList::AtomList() {
  this->pred = NULL;
}

TL::AtomList::AtomList(TL::Predicate* pred) {
  init(pred);
}

void TL::AtomList::clear() {
  uint i;
  FOR_ALL(mem, i) {
    if (mem.p[i] != NULL)
      delete mem.p[i];
  }
}

void TL::AtomList::init(TL::Predicate* pred) {
  this->pred = pred;
  
  if (pred->d <= 3) {
    uint dim[pred->d];
    uint i;
    for (i=0; i<pred->d; i++) {
      dim[i] = LOGIC__MAX_LIMIT_OBJECT_ID;
    }
    mem.resize(pred->d, dim);
    mem.setUni(NULL);
  }
}




// ------------------------------------------------------
// LiteralList


TL::LiteralList::LiteralList() {
  this->pred = NULL;
}

TL::LiteralList::LiteralList(TL::Predicate* pred) {
  init(pred);
}

void TL::LiteralList::clear() {
  uint i;
  FOR_ALL(mem, i) {
    if (mem.p[i] != NULL)
      delete mem.p[i];
  }
}

void TL::LiteralList::init(TL::Predicate* pred) {
  this->pred = pred;
  
  if (pred->d <= 3) {
    uint dim[pred->d];
    uint i;
    for (i=0; i<pred->d; i++) {
      dim[i] = LOGIC__MAX_LIMIT_OBJECT_ID;
    }
    mem.resize(pred->d, dim);
    mem.setUni(NULL);
  }
}





// ------------------------------------------------------
// ComparisonAtomListConstant

TL::ComparisonAtomListConstant::ComparisonAtomListConstant() {
  init(NULL);
}

TL::ComparisonAtomListConstant::ComparisonAtomListConstant(TL::Function* f) {
  init(f);
}

void TL::ComparisonAtomListConstant::clear() {
  uint i, k;
  FOR_ALL(mem, i) {
    for (k=0; k<mem.p[i].N; k++) {
      delete mem.p[i](k);
    }
  }
}

void TL::ComparisonAtomListConstant::init(TL::Function* f) {
  this->f = f;
  
  mem.resize(0);
  if (f!=NULL) {
    uint arg_length = f->d;
    uint dim[arg_length];
    uint i;
    for (i=0; i<arg_length; i++) {
      dim[i] = LOGIC__MAX_LIMIT_OBJECT_ID;
    }
    mem.resize(arg_length, dim);
    CompAtomL empty;
    mem.setUni(empty);
  }
}


// ------------------------------------------------------
// ComparisonLiteralListConstant

TL::ComparisonLiteralListConstant::ComparisonLiteralListConstant() {
  init(NULL);
}

TL::ComparisonLiteralListConstant::ComparisonLiteralListConstant(TL::Function* f) {
  init(f);
}

void TL::ComparisonLiteralListConstant::clear() {
  uint i, k;
  FOR_ALL(mem, i) {
    for (k=0; k<mem.p[i].N; k++) {
      delete mem.p[i](k);
    }
  }
}

void TL::ComparisonLiteralListConstant::init(TL::Function* f) {
  this->f = f;
  
  mem.resize(0);
  if (f!=NULL) {
    uint arg_length = f->d;
    uint dim[arg_length];
    uint i;
    for (i=0; i<arg_length; i++) {
      dim[i] = LOGIC__MAX_LIMIT_OBJECT_ID;
    }
    mem.resize(arg_length, dim);
    CompLitL empty;
    mem.setUni(empty);
  }
}





// ------------------------------------------------------
// ComparisonAtomListDynamic

TL::ComparisonAtomListDynamic::ComparisonAtomListDynamic() {
  init(NULL);
}

TL::ComparisonAtomListDynamic::ComparisonAtomListDynamic(TL::Function* f) {
  init(f);
}

void TL::ComparisonAtomListDynamic::clear() {
  uint i;
  FOR_ALL(mem, i) {
    delete mem.p[i];
  }
}

void TL::ComparisonAtomListDynamic::init(TL::Function* f) {
  this->f = f;
  
  mem.resize(0);
  if (f!=NULL) {
    uint arg_length = f->d * 2;
    uint dim[arg_length];
    uint i;
    for (i=0; i<arg_length; i++) {
      dim[i] = LOGIC__MAX_LIMIT_OBJECT_ID;
    }
    mem.resize(arg_length, dim);
    mem.setUni(NULL);
  }
}




// ------------------------------------------------------
// ComparisonLiteralListDynamic

TL::ComparisonLiteralListDynamic::ComparisonLiteralListDynamic() {
  init(NULL);
}

TL::ComparisonLiteralListDynamic::ComparisonLiteralListDynamic(TL::Function* f) {
  init(f);
}

void TL::ComparisonLiteralListDynamic::clear() {
  uint i;
  FOR_ALL(mem, i) {
    delete mem.p[i];
  }
}

void TL::ComparisonLiteralListDynamic::init(TL::Function* f) {
  this->f = f;
  
  mem.resize(0);
  if (f!=NULL) {
    uint arg_length = f->d * 2;
    uint dim[arg_length];
    uint i;
    for (i=0; i<arg_length; i++) {
      dim[i] = LOGIC__MAX_LIMIT_OBJECT_ID;
    }
    mem.resize(arg_length, dim);
    mem.setUni(NULL);
  }
}





// ------------------------------------------------------
// FunctionAtom

TL::FunctionAtomList::FunctionAtomList() {
  this->f = NULL;
}

TL::FunctionAtomList::FunctionAtomList(TL::Function* f) {
  init(f);
}

void TL::FunctionAtomList::clear() {
  uint i;
  FOR_ALL(mem, i) {
    if (mem.p[i] != NULL)
      delete mem.p[i];
  }
}

void TL::FunctionAtomList::init(TL::Function* f) {
  this->f = f;
  
  uint dim[f->d];
  uint i;
  for (i=0; i<f->d; i++) {
    dim[i] = LOGIC__MAX_LIMIT_OBJECT_ID;
  }
  mem.resize(f->d, dim);
  mem.setUni(NULL);
}



// ------------------------------------------------------
// FunctionValue

TL::FunctionValueList::FunctionValueList() {
  this->f = NULL;
}

TL::FunctionValueList::FunctionValueList(TL::Function* f) {
  init(f);
}

void TL::FunctionValueList::clear() {
  uint i, k;
  FOR_ALL(mem, i) {
    for (k=0; k<mem.p[i].N; k++) {
      delete mem.p[i](k);
    }
  }
}

void TL::FunctionValueList::init(TL::Function* f) {
  this->f = f;
  mem.resize(0);
  if (f!=NULL) {
    uint dim[f->d];
    uint i;
    for (i=0; i<f->d; i++) {
      dim[i] = LOGIC__MAX_LIMIT_OBJECT_ID;
    }
    mem.resize(f->d, dim);
    FuncVL empty;
    mem.setUni(empty);
  }
}



















// --------------------------------------------------------
// --------------------------------------------------------
// --------------------------------------------------------
// --------------------------------------------------------
// --------------------------------------------------------
//
// LOGIC MANAGER

// Lists for objects

TL::Predicate* p_comp_constant;
TL::Predicate* p_comp_dynamic;
  
MT::Array< TL::AtomList > lists_atoms; // for each predicate its own AtomList
MT::Array< TL::LiteralList > lists_lits_pos; // for each predicate its own positive LiteralList
MT::Array< TL::LiteralList > lists_lits_neg; // for each predicate its own negative LiteralList

MT::Array< TL::ComparisonAtomListConstant > lists_atoms_comp_constant;  //  Outer Array:  (Index 1) function-id,  (Index 2) comparison type
MT::Array< TL::ComparisonLiteralListConstant > lists_lits_comp_constant;   //  Outer Array:  (Index 1) function-id,  (Index 2) comparison type
  
MT::Array< TL::ComparisonAtomListDynamic > lists_atoms_comp_dynamic;  //  Outer Array:  (Index 1) function-id,  (Index 2) comparison type
MT::Array< TL::ComparisonLiteralListDynamic > lists_lits_comp_dynamic;  //  Outer Array:  (Index 1) function-id,  (Index 2) comparison type

MT::Array< TL::FunctionAtomList > lists_function_atoms;
MT::Array< TL::FunctionValueList > lists_function_values;



void __init_logicObjectManager_database() {
  // Predicates
  uint INIT_SIZE_P = 100;
  lists_atoms.resize(INIT_SIZE_P);
  lists_lits_pos.resize(INIT_SIZE_P);
  lists_lits_neg.resize(INIT_SIZE_P);
  
  // Functions
  uint INIT_SIZE_F = 100;
  
  uint max_compType_index = 0;
  max_compType_index = TL_MAX(max_compType_index, TL::comparison_equal);
  max_compType_index = TL_MAX(max_compType_index, TL::comparison_less);
  max_compType_index = TL_MAX(max_compType_index, TL::comparison_lessEqual);
  max_compType_index = TL_MAX(max_compType_index, TL::comparison_greater);
  max_compType_index = TL_MAX(max_compType_index, TL::comparison_greaterEqual);
  
  // ComparisonAtoms and ComparisonLiterals
  lists_atoms_comp_constant.resize(INIT_SIZE_F, max_compType_index+1);
  lists_atoms_comp_dynamic.resize(INIT_SIZE_F, max_compType_index+1);
  
  lists_lits_comp_constant.resize(INIT_SIZE_F, max_compType_index+1);
  lists_lits_comp_dynamic.resize(INIT_SIZE_F, max_compType_index+1);
  
  // FunctionAtoms and FunctionValues
  lists_function_atoms.resize(INIT_SIZE_F);
  lists_function_values.resize(INIT_SIZE_F);
}


void TL::logicObjectManager_database::setComparisonPredicates(ComparisonPredicate* _p_comp_constant, ComparisonPredicate* _p_comp_dynamic) {
  __init_logicObjectManager_database();
  p_comp_constant = _p_comp_constant;
  p_comp_dynamic = _p_comp_dynamic;
}


void TL::logicObjectManager_database::addPredicate(Predicate* p) {
  uint DEBUG = 0;
  if (DEBUG>0) {cout<<"logicObjectManager_database::addPredicate [START]"<<endl;}
  if (lists_atoms.N == 0) {
    if (DEBUG>0) {cout<<"logicObjectManager_database gets inited first."<<endl;}
    __init_logicObjectManager_database();
  }
  if (DEBUG>0) {cout<<"Predicate p: ";  p->writeNice();  cout<<endl;}
  CHECK(p->id < lists_atoms.N, "bad new id="<<p->id<<"  vs  max_id=" << lists_atoms.N);
  AtomList atom_list(p);
  lists_atoms(p->id) = atom_list;
  LiteralList lit_list(p);
  lists_lits_pos(p->id) = lit_list;
  lists_lits_neg(p->id) = lit_list;
  if (DEBUG>0) {PRINT(atom_list.mem.nd);}
  if (DEBUG>0) {cout<<"logicObjectManager_database::addPredicate [END]"<<endl;}
}



void TL::logicObjectManager_database::addFunction(Function* f) {
  if (lists_function_atoms.N == 0) {
    __init_logicObjectManager_database();
  }
  
  uint i;
  
  // resize lists if necessary
  if (lists_atoms_comp_constant.d0 < f->id) {
    uint a, b;
    
    MT::Array< ComparisonAtomListConstant > new__lists_atoms_comp_constant(f->id + 1, lists_atoms_comp_constant.d1);
    FOR2D(lists_atoms_comp_constant, a, b) {
      new__lists_atoms_comp_constant(a,b) = lists_atoms_comp_constant(a,b);
    }
    lists_atoms_comp_constant = new__lists_atoms_comp_constant;
    
    MT::Array< ComparisonLiteralListConstant > new__lists_lits_comp_constant(f->id + 1, lists_lits_comp_constant.d1);
    FOR2D(lists_lits_comp_constant, a, b) {
      new__lists_lits_comp_constant(a,b) = lists_lits_comp_constant(a,b);
    }
    lists_lits_comp_constant = new__lists_lits_comp_constant;
    
    MT::Array< ComparisonAtomListDynamic > new__lists_atoms_comp_dynamic(f->id + 1, lists_atoms_comp_dynamic.d1);
    FOR2D(lists_atoms_comp_dynamic, a, b) {
      new__lists_atoms_comp_dynamic(a,b) = lists_atoms_comp_dynamic(a,b);
    }
    lists_atoms_comp_dynamic = new__lists_atoms_comp_dynamic;
    
    MT::Array< ComparisonLiteralListDynamic > new__lists_lits_comp_dynamic(f->id + 1, lists_lits_comp_dynamic.d1);
    FOR2D(lists_lits_comp_dynamic, a, b) {
      new__lists_lits_comp_dynamic(a,b) = lists_lits_comp_dynamic(a,b);
    }
    lists_lits_comp_dynamic = new__lists_lits_comp_dynamic;
    
    
    
    MT::Array< FunctionAtomList > new__lists_function_atoms(f->id + 1);
    FOR1D(lists_function_atoms, a) {
      new__lists_function_atoms(a) = lists_function_atoms(a);
    }
    lists_function_atoms = new__lists_function_atoms;
    
    MT::Array< FunctionValueList > new__lists_function_values(f->id + 1);
    FOR1D(lists_function_values, a) {
      new__lists_function_values(a) = lists_function_values(a);
    }
    lists_function_values = new__lists_function_values;
  }
  
  // ComparisonAtoms and ComparisonLiterals
  for (i=0; i<lists_atoms_comp_constant.d1; i++) {
    lists_atoms_comp_constant(f->id, i).init(f);
    lists_atoms_comp_dynamic(f->id, i).init(f);
  
    lists_lits_comp_constant(f->id, i).init(f);
    lists_lits_comp_dynamic(f->id, i).init(f);
  }
  
  // FunctionAtoms and FunctionValues
  lists_function_atoms(f->id).init(f);
  lists_function_values(f->id).init(f);
}

void TL::logicObjectManager_database::shutdown() {
  lists_atoms.clear();
  lists_lits_pos.clear();
  lists_lits_neg.clear();
  
  lists_atoms_comp_constant.clear();
  lists_lits_comp_constant.clear();
  
  lists_atoms_comp_dynamic.clear();
  lists_lits_comp_dynamic.clear();
  
  lists_function_atoms.clear();
  lists_function_values.clear();
}




TL::Atom* TL::logicObjectManager_database::get(TL::Predicate* pred, const uintA& args) {
  uint index = pred->id;
  Atom* a = NULL;
  a = lists_atoms(index).mem(args);
  if (a == NULL) {
    a = new Atom;
    a->pred = pred;
    a->args = args;
    lists_atoms(index).mem(args) = a;
//     cout<<"NEW ATOM  ***"<<*a<<"***   "<<a<<"  "<<a->pred->name<<"  "<<a->args<<endl;
  }
  return a;
}

TL::Literal* TL::logicObjectManager_database::get(TL::Predicate* pred, const bool positive, const uintA& args) {
  uint index = pred->id;
  Literal* lit = NULL;
  if (positive) {
    lit = lists_lits_pos(index).mem(args);
    if (lit == NULL) {
      lit = new Literal;
      lit->atom = get(pred, args);
      lit->positive = true;
      lists_lits_pos(index).mem(args) = lit;
    }
  }
  else {
    lit = lists_lits_neg(index).mem(args);
    if (lit == NULL) {
      lit = new Literal;
      lit->atom = get(pred, args);
      lit->positive = false;
      lists_lits_neg(index).mem(args) = lit;
    }
  }
  return lit;
}




TL::ComparisonAtom* TL::logicObjectManager_database::get_compA(TL::Function* f, ComparisonType compType, const uintA& args1, const uintA& args2) {
  uintA args_all;  args_all.append(args1);  args_all.append(args2);
  TL::ComparisonAtom* ca = lists_atoms_comp_dynamic(f->id, compType).mem(args_all);
  if (ca == NULL) {
    ca = new ComparisonAtom;
    ca->pred = p_comp_dynamic;
    ca->fa1 = get(f, args1);
    ca->fa2 = get(f, args2);
    ca->comparisonType = compType;
    ca->args.resize(0);
    lists_atoms_comp_dynamic(f->id, compType).mem(args_all) = ca;
  }
  return ca;
}


TL::ComparisonLiteral* TL::logicObjectManager_database::get_compL(TL::Function* f, ComparisonType compType, const uintA& args1, const uintA& args2) {
  uintA args_all;  args_all.append(args1);  args_all.append(args2);
  TL::ComparisonLiteral* clit = lists_lits_comp_dynamic(f->id, compType).mem(args_all);
  if (clit == NULL) {
    clit = new ComparisonLiteral;
    clit->atom = get_compA(f, compType, args1, args2);
    clit->positive = true;
    lists_lits_comp_dynamic(f->id, compType).mem(args_all) = clit;
  }
  return clit;
}


TL::ComparisonAtom* TL::logicObjectManager_database::get_compA(TL::Function* f, ComparisonType compType, double bound, const uintA& args) {
  CompAtomL& clist = lists_atoms_comp_constant(f->id, compType).mem(args);
  TL::ComparisonAtom* ca = NULL;
  uint i = 0;
  if (clist.N > 0) {
    int left = 0;
    int right = clist.N-1;
    while (left <= right) {
      i = (int) ((left + right) / 2);
      if (areEqual(clist(i)->bound, bound)) {
        ca = clist(i);
        return ca;
      }
      else if (clist(i)->bound < bound) {
        left = i+1;
      }
      else {
        right = i-1;
      }
    }
  }
  // not found
  ca = new TL::ComparisonAtom;
  ca->pred = p_comp_constant;
  ca->fa1 = get(f, args);
  ca->fa2 = NULL;
  ca->comparisonType = compType;
  ca->bound = bound;
  ca->args.resize(0);
  
  clist.memMove = true;
  if (clist.N == 0)
    clist.append(ca);
  else if (clist(i)->bound < ca->bound) {
    if (i+1 < clist.N)
      clist.insert(i+1, ca);
    else
      clist.append(ca);
  }
  else
    clist.insert(i, ca);
  
  return ca;
}


TL::ComparisonLiteral* TL::logicObjectManager_database::get_compL(TL::Function* f, ComparisonType compType, double bound, const uintA& args) {
  ComparisonLiteralListConstant& big_list = lists_lits_comp_constant(f->id, compType);
  CompLitL& clist = big_list.mem(args);
  TL::ComparisonLiteral* clit = NULL;
  uint i = 0;
  if (clist.N > 0) {
    int left = 0;
    int right = clist.N-1;
    while (left <= right) {
      i = (int) ((left + right) / 2);
      if (areEqual(((ComparisonAtom*) clist(i)->atom)->bound, bound)) {
        clit = clist(i);
        return clit;
      }
      else if (((ComparisonAtom*) clist(i)->atom)->bound < bound) {
        left = i+1;
      }
      else {
        right = i-1;
      }
    }
  }
  // not found
  // create new
  clit = new TL::ComparisonLiteral;
  clit->atom = get_compA(f, compType, bound, args);
  clit->positive = true;
  // insert in list
  clist.memMove = true;
  if (clist.N == 0)
    clist.append(clit);
  else if (((ComparisonAtom*)clist(i)->atom)->bound < ((ComparisonAtom*)clit->atom)->bound) {
    if (i+1 < clist.N)
      clist.insert(i+1, clit);
    else
      clist.append(clit);
  }
  else
    clist.insert(i, clit);
  
  return clit;
}




TL::FunctionAtom* TL::logicObjectManager_database::get(TL::Function* f, const uintA& args) {
  uint index = f->id;
  FunctionAtom* a = NULL;
  a = lists_function_atoms(index).mem(args);
  if (a == NULL) {
    a = new FunctionAtom;
    a->f = f;
    a->args = args;
    lists_function_atoms(index).mem(args) = a;
  }
  return a;
}


TL::FunctionValue* TL::logicObjectManager_database::get(TL::Function* f, const uintA& args, double value) {
  FuncVL& fvs = lists_function_values(f->id).mem(args);
  TL::FunctionValue* fv = NULL;
  uint i = 0;
  if (fvs.N > 0) {
    int left = 0;
    int right = fvs.N-1;
    while (left <= right) {
      i = (int) ((left + right) / 2);
      if (areEqual(fvs(i)->value, value)) {
        fv = fvs(i);
        return fv;
      }
      else if (fvs(i)->value < value) {
        left = i+1;
      }
      else {
        right = i-1;
      }
    }
  }
  // not found
  // create new
  fv = new TL::FunctionValue;
  fv->atom = get(f, args);
  fv->value = value;
  // insert in list
  fvs.memMove = true;
  if (fvs.N == 0)
    fvs.append(fv);
  else if (fvs(i)->value < fv->value) {
    if (i+1 < fvs.N)
      fvs.insert(i+1, fv);
    else
      fvs.append(fv);
  }
  else
    fvs.insert(i, fv);
  
  return fv;
}


