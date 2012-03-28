/*  
    Copyright 2012   Tobias Lang
    
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
#include "logicObjectManager.h"
#include "logicReasoning.h"



uintA TL::logicObjectManager::constants;
TermTypeL TL::logicObjectManager::constants_types;
  
PredL TL::logicObjectManager::p_actions;
PredL TL::logicObjectManager::p_prim;
PredL TL::logicObjectManager::p_derived;
TL::ComparisonPredicate TL::logicObjectManager::p_comp_constant;
TL::ComparisonPredicate TL::logicObjectManager::p_comp_dynamic;
FuncL TL::logicObjectManager::f_prim;
FuncL TL::logicObjectManager::f_derived;
TermTypeL TL::logicObjectManager::types;
  
TL::ConceptDependencyGraph TL::logicObjectManager::dependencyGraph;





// =============================================================================================
// =============================================================================================
// =============================================================================================
// =============================================================================================
// =============================================================================================



void __init_logicObjectManager () {
  if (TL::logicObjectManager::getPredicate(MT::String("default")) != NULL) {
    // has already been inited: in p_actions there has to be default action
    return;
  }
  
  // Build ComparisonPredicates
  TL::logicObjectManager::p_comp_constant.id = TL::logicObjectManager::getLowestFreeConceptID(30);
  TL::logicObjectManager::p_comp_constant.name = MT::String("comp_constant");
  TL::logicObjectManager::p_comp_constant.d = 0;
  TL::logicObjectManager::p_comp_constant.type = TL::Predicate::predicate_comparison;
  TL::logicObjectManager::p_comp_constant.constantBound = true;
  
  TL::logicObjectManager::p_comp_dynamic.id = TL::logicObjectManager::getLowestFreeConceptID(30)+1;
  TL::logicObjectManager::p_comp_dynamic.name = MT::String("comp_dynamic");
  TL::logicObjectManager::p_comp_dynamic.d = 0;
  TL::logicObjectManager::p_comp_dynamic.type = TL::Predicate::predicate_comparison;
  TL::logicObjectManager::p_comp_dynamic.constantBound = false;
  
  TL::logicObjectManager_database::setComparisonPredicates(&TL::logicObjectManager::p_comp_constant, &TL::logicObjectManager::p_comp_dynamic);
  
  // Build default action predicate   (always created by hand)
  TL::Predicate* p_action_default = NULL;
  p_action_default = new TL::Predicate;
  p_action_default->id = TL::DEFAULT_ACTION_PRED__ID;
  p_action_default->name = "default";
  p_action_default->d = 0;
  p_action_default->type = TL::Predicate::predicate_action;
  PredL wrapper;  wrapper.append(p_action_default);
  TL::logicObjectManager::p_actions.append(p_action_default);
  TL::logicObjectManager_database::addPredicate(p_action_default);
}


void TL::logicObjectManager::init(const char* language_file, uint fileType) {
  PredL _p_prim, _p_derived, _p_actions;
  FuncL _f_prim, _f_derived;
  TermTypeL _types;
  if (fileType == 1)
    readLanguage(language_file, _p_prim, _p_derived, _p_actions, _f_prim, _f_derived, _types);
  else if (fileType == 2)
    readLanguageSimpleFormat(language_file, _p_prim, _p_derived, _p_actions, _f_prim, _f_derived, _types);
  else
    HALT("");
  addStatePredicates(_p_prim);
  addStatePredicates(_p_derived);
  addStateFunctions(_f_prim);
  addStateFunctions(_f_derived);
  addActionPredicates(_p_actions);
  TL::logicObjectManager::types = _types;
}



void TL::logicObjectManager::shutdown() {
  TL::logicObjectManager_database::shutdown();
  uint i=0;
  FOR1D(TL::logicObjectManager::p_actions, i) {
    delete TL::logicObjectManager::p_actions(i);
  }
  FOR1D(p_prim, i) {
//     PRINT(i);
//     p_prim(i)->writeNice();
    delete p_prim(i);
//     cout<<"  deleted"<<endl;
  }
  FOR1D(p_derived, i) {delete p_derived(i);}
  FOR1D(f_prim, i) {delete f_prim(i);}
  FOR1D(f_derived, i) {delete f_derived(i);}
}




void TL::logicObjectManager::setConstants(uintA& constants) {
  TL::logicObjectManager::constants = constants;
  constants_types.resize(constants.N);
  uint i;
  TL::TermType* default_type = NULL;
  FOR1D(types, i) {
    if (types(i)->type_id == TL::TermType::ANY_id)
      default_type = types(i);
  }
  FOR1D(constants_types, i) {
    constants_types(i) = default_type;
  }

  // for initing database
  uint k, v;
  FOR1D(p_prim, i) {
    MT::Array< uintA > sas;
    TL::allPossibleLists(sas, constants, p_prim(i)->d, true, true);
    FOR1D(sas, k) {
      getLiteral(p_prim(i), true, sas(k));
      getLiteral(p_prim(i), false, sas(k));
//       getLiteral(p_prim(i), true, sas(k))->writeNice(cout);cout<<" ";
    }
  }
  FOR1D(p_derived, i) {
    MT::Array< uintA > sas;
    TL::allPossibleLists(sas, constants, p_derived(i)->d, true, true);
    FOR1D(sas, k) {
      getLiteral(p_derived(i), true, sas(k));
      getLiteral(p_derived(i), false, sas(k));
//       getLiteral(p_derived(i), true, sas(k))->writeNice(cout);cout<<" ";
    }
  }
  
  
  FOR1D(f_prim, i) {
    MT::Array< uintA > sas;
    TL::allPossibleLists(sas, constants, f_prim(i)->d, true, true);
    FOR1D(sas, k) {
      getFA(f_prim(i), sas(k));
      FOR1D(f_prim(i)->range, v) {
        getFV(f_prim(i), sas(k), f_prim(i)->range(v));
//         getFV(f_prim(i), sas(k), f_prim(i)->range(v))->writeNice(cout);cout<<" ";
      }
    }
  }
  
  FOR1D(f_derived, i) {
    MT::Array< uintA > sas;
    TL::allPossibleLists(sas, constants, f_derived(i)->d, true, true);
    FOR1D(sas, k) {
      getFA(f_derived(i), sas(k));
      FOR1D(f_derived(i)->range, v) {
        getFV(f_derived(i), sas(k), f_derived(i)->range(v));
//         getFV(f_derived(i), sas(k), f_prim(i)->range(v))->writeNice(cout);cout<<" ";
      }
    }
  }

}


void TL::logicObjectManager::setConstants(uintA& constants, const TermTypeL& constants_types) {
  setConstants(constants);
  TL::logicObjectManager::constants_types = constants_types;
}


void TL::logicObjectManager::addStatePredicates(PredL& preds) {
  __init_logicObjectManager();
  TL::sort(preds);
  uint i;
  FOR1D(preds, i) {
    if (preds(i)->category == category_derived) {
      TL::DerivedPredicate* dp = dynamic_cast<TL::DerivedPredicate*>(preds(i));
      CHECK(dp!=NULL, "cast failed");
      p_derived.append(dp);
    }
    else
      p_prim.append(preds(i));
  }
  
  // update LogicObjectDatabase
  FOR1D(preds, i) {
    logicObjectManager_database::addPredicate(preds(i));
  }
  
  // update dependency graph
  PredL ps_all;
  ps_all.append(p_prim);
  FOR1D(p_derived, i) ps_all.append(p_derived(i));
  FuncL fs_all;
  fs_all.append(f_prim);
  fs_all.append(f_derived);
  dependencyGraph.setNodes(ps_all, fs_all);
  CHECK(dependencyGraph.isAcyclic(), "Dependencies contain cycle!");
}


void TL::logicObjectManager::addActionPredicates(PredL& _p_action) {
  __init_logicObjectManager();
  TL::sort(_p_action);
  TL::logicObjectManager::p_actions.append(_p_action);
  // update LogicObjectDatabase
  uint i;
  FOR1D(_p_action, i) {
    logicObjectManager_database::addPredicate(_p_action(i));
  }
}


void TL::logicObjectManager::addStateFunctions(FuncL& funcs) {
  __init_logicObjectManager();
  TL::sort(funcs);
  uint i;
  FOR1D(funcs, i) {
    if (funcs(i)->category == category_primitive)
      f_prim.append(funcs(i));
    else
      f_derived.append(funcs(i));
  }
  
  // update LogicObjectDatabase
  FOR1D(funcs, i) {
    logicObjectManager_database::addFunction(funcs(i));
  }
  
  // update dependency graph
  PredL ps_all;
  ps_all.append(p_prim);
  FOR1D(p_derived, i) ps_all.append(p_derived(i));
  FuncL fs_all;
  fs_all.append(f_prim);
  fs_all.append(f_derived);
  dependencyGraph.setNodes(ps_all, fs_all);
  CHECK(dependencyGraph.isAcyclic(), "Dependencies contain cycle!");
}




uint TL::logicObjectManager::getLowestFreeConceptID(uint min) {
  uintA used_ids;
  uint i;
  FOR1D(p_actions, i) {used_ids.append(p_actions(i)->id);}
  FOR1D(p_prim, i) {used_ids.append(p_prim(i)->id);}
  FOR1D(p_derived, i) {used_ids.append(p_derived(i)->id);}
  FOR1D(f_prim, i) {used_ids.append(f_prim(i)->id);}
  FOR1D(f_derived, i) {used_ids.append(f_derived(i)->id);}
  used_ids.append(p_comp_constant.id);
  used_ids.append(p_comp_dynamic.id);
  uint min_free_id = min;
  while (used_ids.findValue(min_free_id) >= 0) {
    min_free_id++;
  }
  return min_free_id;
}

// 

TL::TermType* TL::logicObjectManager::getTermTypeOfObject(uint object_id) {
  int idx = constants.findValue(object_id);
  if (idx >= 0) {
    return constants_types(idx);
  }
  else {
    CHECK(types(0)->type_id == TL::TermType::ANY_id, "ANY-TermType is missing");
    return types(0);
  }
}


TL::Atom* atom_doNothing = NULL;

TL::Atom* TL::logicObjectManager::getAtom_doNothing() {
  if (atom_doNothing == NULL) {
    TL::Predicate* doNothing = new TL::Predicate;
    uint i;
    uint doNothing__id = 0;
    FOR1D(TL::logicObjectManager::p_actions, i) {
      if (TL::logicObjectManager::p_actions(i)->id >= doNothing__id)
        doNothing__id = TL::logicObjectManager::p_actions(i)->id+1;
    }
    doNothing->id = doNothing__id;
    FOR1D(p_prim, i) {
      if (p_prim(i)->id == doNothing->id)
        HALT("doNothing action predicate shall be assigned id="<<doNothing->id<<" although this is already assigned to predicate "<<p_prim(i)->name);
    }
    doNothing->d = 0;
    doNothing->name = "doNothing";
    doNothing->type = TL::Predicate::predicate_action;
    doNothing->category = category_primitive;
    PredL wrapper;
    wrapper.setAppend(doNothing);
    addActionPredicates(wrapper);
  
     // create atom
    uintA empty;
    atom_doNothing = getAtom(doNothing, empty);
  }
  return atom_doNothing;
}



TL::Predicate* TL::logicObjectManager::getPredicate(const MT::String& name) {
  uint i;
  FOR1D(p_prim, i) {
    if (p_prim(i)->name == name)
      return p_prim(i);
  }
  FOR1D(p_derived, i) {
    if (p_derived(i)->name == name)
      return p_derived(i);
  }
  FOR1D(TL::logicObjectManager::p_actions, i) {
    if (TL::logicObjectManager::p_actions(i)->name == name)
      return TL::logicObjectManager::p_actions(i);
  }
  return NULL;
}

TL::Predicate* TL::logicObjectManager::getPredicate(uint id) {
  TL::Predicate* p = dependencyGraph.getPredicate(id);
  if (p != NULL) {
    return p;
  }
  uint i;
  FOR1D(TL::logicObjectManager::p_actions, i) {
    if (TL::logicObjectManager::p_actions(i)->id == id)
      return TL::logicObjectManager::p_actions(i);
  }
  HALT("Predicate with id="<<id<<" not found."<<endl);
  return NULL;
}


TL::Function* TL::logicObjectManager::getFunction(const MT::String& name) {
  uint i;
  FOR1D(f_prim, i) {
    if (f_prim(i)->name == name)
      return f_prim(i);
  }
  FOR1D(f_derived, i) {
    if (f_derived(i)->name == name)
      return f_derived(i);
  }
  return NULL;
}


TL::Function* TL::logicObjectManager::getFunction(uint id) {
  return dependencyGraph.getFunction(id);
}







  /****************************************
      MANAGING OBJECT INSTANCES
  ***************************************/



TL::Atom* TL::logicObjectManager::getAtom(TL::Predicate* pred, uintA& args) {
  TL::Literal* lit = getLiteral(pred, true, args);
  return lit->atom;
}

TL::Literal* TL::logicObjectManager::getLiteral(TL::Predicate* pred, bool positive, uintA& args) {
  if (pred == NULL) HALT("Predicate == NULL");
  CHECK(args.N == pred->d, "Invalid predicate instance: "<<pred->name<<" has d="<<pred->d << "   vs. args="<<args);
  return logicObjectManager_database::get(pred, positive, args);
}

TL::ComparisonAtom* TL::logicObjectManager::getCompAtom_constant(TL::Function* f, ComparisonType compType, double bound, uintA& args) {
  ComparisonLiteral* clit = getCompLiteral_constant(f, compType, bound, args);
  return (ComparisonAtom*) clit->atom;
}

TL::ComparisonAtom* TL::logicObjectManager::getCompAtom_dynamic(TL::Function* f, ComparisonType compType, uintA& args1, uintA& args2) {
  ComparisonLiteral* clit = getCompLiteral_dynamic(f, compType, args1, args2);
  return (ComparisonAtom*) clit->atom;
}

TL::ComparisonLiteral* TL::logicObjectManager::getCompLiteral_constant(TL::Function* f, ComparisonType compType, double bound, uintA& args) {
    return logicObjectManager_database::get_compL(f, compType, bound, args);
}

TL::ComparisonLiteral* TL::logicObjectManager::getCompLiteral_dynamic(TL::Function* f, ComparisonType compType, uintA& args1, uintA& args2) {
    return logicObjectManager_database::get_compL(f, compType, args1, args2);
}

TL::FunctionValue* TL::logicObjectManager::getFV(TL::Function* f, uintA& args, double value) {
  CHECK(args.N == f->d, "");
  return logicObjectManager_database::get(f, args, value);
}

TL::FunctionAtom* TL::logicObjectManager::getFA(TL::Function* f, const uintA& args) {
  CHECK(args.N == f->d, "");
  return logicObjectManager_database::get(f, args);
}

TL::Literal* TL::logicObjectManager::getLiteralNeg(TL::Literal* lit) {
    CHECK(lit->atom->pred->type != TL::Predicate::predicate_comparison, "not defined for p_comp")
    return logicObjectManager_database::get(lit->atom->pred, !lit->positive, lit->atom->args);
}

TL::Literal* TL::logicObjectManager::getLiteralOrig(TL::Literal* lit_copy) {
  if (lit_copy->atom->pred->type == TL::Predicate::predicate_comparison) {
    TL::ComparisonLiteral* clit_copy = (TL::ComparisonLiteral*) lit_copy;
    TL::ComparisonAtom* ca_copy = (TL::ComparisonAtom*) clit_copy->atom;
    TL::ComparisonLiteral* clit;
    if (ca_copy->hasConstantBound()) {
      clit = getCompLiteral_constant(ca_copy->fa1->f, ca_copy->comparisonType, ca_copy->bound, ca_copy->fa1->args);
    }
    else {
      clit = getCompLiteral_dynamic(ca_copy->fa1->f, ca_copy->comparisonType, ca_copy->fa1->args, ca_copy->fa2->args);
    }
    if (clit->atom->args.N != lit_copy->atom->args.N) {
      lit_copy->atom->pred->writeNice();
      clit->atom->pred->writeNice();
      HALT("error occurred");
    }
    CHECK(clit->atom->args.N == lit_copy->atom->args.N, "something gone wrong");

    if (((TL::ComparisonAtom*) clit->atom)->fa1 != ca_copy->fa1)
      delete ca_copy->fa1;
    if (((TL::ComparisonAtom*) clit->atom)->fa2 != ca_copy->fa2)
      delete ca_copy->fa2;
    if (clit->atom != ca_copy)
      delete ca_copy;
    if (clit != lit_copy)
      delete lit_copy;
    return clit;
  }
  else {
    TL::Literal* lit = getLiteral(lit_copy->atom->pred, lit_copy->positive, lit_copy->atom->args);
    if (lit->atom != lit_copy->atom) {
      delete lit_copy->atom;
    }
    if (lit != lit_copy)
      delete lit_copy;
    return lit;
  }
}


TL::Atom* TL::logicObjectManager::getAtomOrig(TL::Atom* a_copy) {
  if (a_copy->pred->type == TL::Predicate::predicate_comparison) {
    TL::ComparisonAtom* ca_copy = (ComparisonAtom*) a_copy;
    TL::ComparisonAtom* ca;
    if (ca_copy->hasConstantBound()) {
      ca = getCompAtom_constant(ca_copy->fa1->f, ca_copy->comparisonType, ca_copy->bound, ca_copy->fa1->args);
    }
    else {
      ca = getCompAtom_dynamic(ca_copy->fa1->f, ca_copy->comparisonType, ca_copy->fa1->args, ca_copy->fa2->args);
    }
    #if 0 // debugging
    if (ca->args.N != ca_copy->args.N) {
      ca_copy->pred->writeNice();
      PRINT(ca_copy->hasConstantBound());
      ca->pred->writeNice();
      PRINT(ca->hasConstantBound());
    }
    CHECK(ca->args.N == a_copy->args.N, "something gone wrong");
    #endif
    if (ca->fa1 != ca_copy->fa1)
      delete ca_copy->fa1;
    if (ca->fa2 != ca_copy->fa2)
      delete ca_copy->fa2;
    if (ca != a_copy)
      delete a_copy;
    return ca;
  }
  else {
    TL::Atom* a = getAtom(a_copy->pred, a_copy->args);
    if (a != a_copy)
      delete a_copy;
    return a;
  }
}


TL::FunctionValue* TL::logicObjectManager::getFVorig(TL::FunctionValue* fv_copy) {
    TL::FunctionValue* fv = getFV(fv_copy->atom->f, fv_copy->atom->args, fv_copy->value);
    if (fv->atom != fv_copy->atom) {
      delete fv_copy->atom;
    }
    if (fv != fv_copy)
        delete fv_copy;
    return fv;
}


TL::FunctionAtom* TL::logicObjectManager::getFAorig(TL::FunctionAtom* fa_copy) {
  TL::FunctionAtom* fa = getFA(fa_copy->f, fa_copy->args);
  if (fa != fa_copy)
    delete fa_copy;
  return fa;
}




void TL::logicObjectManager::makeOriginal(TL::SymbolicState& s) {
  uint i;
  
  LitL p_prim_orig;
  FOR1D(s.lits_prim, i) {
//     s.lits_prim(i)->atom = getAtomOrig(s.lits_prim(i)->atom);
    p_prim_orig.append(getLiteralOrig(s.lits_prim(i)));
  }
  s.lits_prim = p_prim_orig;
  
  LitL complex_lits_orig;
  FOR1D(s.lits_derived, i) {
//     s.lits_derived(i)->atom = getAtomOrig(s.lits_derived(i)->atom);
    complex_lits_orig.append(getLiteralOrig(s.lits_derived(i)));
  }
  s.lits_derived = complex_lits_orig;
  
  // function values
  FuncVL fv_prim_orig;
  FOR1D(s.fv_prim, i) {
//     s.fv_prim(i)->atom = getFAorig(s.fv_prim(i)->atom);
    fv_prim_orig.append(getFVorig(s.fv_prim(i)));
  }
  s.fv_prim = fv_prim_orig;
    
  FuncVL fv_derived_orig;
  FOR1D(s.fv_derived, i) {
//     s.fv_derived(i)->atom = getFAorig(s.fv_derived(i)->atom);
    fv_derived_orig.append(getFVorig(s.fv_derived(i)));
  }
  s.fv_derived = fv_derived_orig;
}

void TL::logicObjectManager::makeOriginal(TL::Trial& w) {
  AtomL actions_orig;
  uint i;
  FOR1D(w.actions, i) {
//         cout<<w.actions(i)<<" ";w.actions(i)->writeNice(cout);cout<<endl;
    actions_orig.append(getAtomOrig(w.actions(i)));
//         cout<<actions_orig.last()<<" ";actions_orig.last()->writeNice(cout);cout<<endl;
  }
  w.actions = actions_orig;
  FOR1D(w.states, i) {
    makeOriginal(*w.states(i));
  }
}


void TL::logicObjectManager::makeOriginal(TL::Rule& r) {
  uint i, k;
  FOR1D(r.context, i) {
//     r.context(i)->atom = TL::logicObjectManager::getAtomOrig(r.context(i)->atom);
    r.context(i) = TL::logicObjectManager::getLiteralOrig(r.context(i));
  }
  r.action = TL::logicObjectManager::getAtomOrig(r.action);
  FOR1D(r.outcomes, i) {
    FOR1D(r.outcomes(i), k) {
//       r.outcomes(i)(k)->atom = TL::logicObjectManager::getAtomOrig(r.outcomes(i)(k)->atom);
      r.outcomes(i)(k) = TL::logicObjectManager::getLiteralOrig(r.outcomes(i)(k));
    }
  }
}




TL::Atom* TL::logicObjectManager::getAtom(const char* text) {
  TL::Literal* lit = getLiteral(text);
  return lit->atom;
}

TL::Literal* TL::logicObjectManager::getLiteral(TL::Atom* atom) {
  return getLiteral(atom->pred, true, atom->args);
}

MT::Array< MT::String > string_variables;
TL::Literal* TL::logicObjectManager::getLiteral(const char* text) {
  uint DEBUG = 0;
  if (DEBUG>0) {cout<<"getLiteral [START]"<<endl;}
  MT::String mt_string(text);
  if (DEBUG>0) {PRINT(mt_string);  PRINT(string_variables);}
  // Negation
  bool positive = true;
  if (peerNextChar(mt_string) == '-') {
    positive = false;
    skip(mt_string,"-");
  }
  // Predicate
  MT::String name;
  name.read(mt_string, NULL, "(");
  if (DEBUG>0) PRINT(name);
  TL::Predicate* pred = NULL;
  TL::Function* f = NULL;
  pred = getPredicate(name);
  if (DEBUG>0) {PRINT(pred);}
  if (pred == NULL)
    f = getFunction(name);
  if (DEBUG>0) {PRINT(f);  if (f!=NULL) {f->writeNice(); cout<<endl;}}
  // Arguments
  if (DEBUG>0) {cout<<"Reading arguments"<<endl;}
  uintA args;
  if (pred != NULL) args.resize(pred->d);
  else args.resize(f->d);
  uint i;
  FOR1D(args, i) {
    MT::String arg;
    arg.read(mt_string, NULL, "/, )");
    if (DEBUG>0) {PRINT(arg);}
    if (isdigit(MT::peerNextChar(arg))) {
      arg >> args(i);
    }
    else {
      int index = string_variables.findValue(arg);
      if (index >= 0) {
        args(i) = index;
      }
      else {
        string_variables.append(arg);
        args(i) = string_variables.N-1;
      }
    }
  }
  if (DEBUG>0) {PRINT(args);}
  TL::Literal* lit = NULL;
  if (pred != NULL) {
    if (DEBUG>0) {cout<<"+++ getting simple literal"<<endl;}
    lit = getLiteral(pred, positive, args);
  }
  else {
    if (DEBUG>0) {cout<<"+++ getting comparison literal"<<endl;}
    if (DEBUG>0) {PRINT(f); f->writeNice();  cout<<endl;}
    ComparisonType compType = comparison_equal;
    char op;
    mt_string >> op;
    if (op == '>') {
      if (MT::peerNextChar(mt_string) == '=') {
        mt_string >> op;
        compType = comparison_greaterEqual;
      }
      else
        compType = comparison_greater;
    }
    else if (op == '<') {
      if (MT::peerNextChar(mt_string) == '=') {
        mt_string >> op;
        compType = comparison_lessEqual;
      }
      else
        compType = comparison_less;
    }
    else {
      mt_string >> op;
      mt_string >> op; // for second '='
    }
    double bound;
    mt_string >> bound;
    lit = getCompLiteral_constant(f, compType, bound, args);
  }
  if (DEBUG>0) {cout<<"==> "; PRINT(*lit);}
  if (DEBUG>0) {cout<<"getLiteral [END]"<<endl;}
  return lit;
}


void TL::logicObjectManager::getAtoms(AtomL& as, const char* text) {
  as.clear();
  MT::String mt_string(text);
  MT::String name;
  while (MT::skip(mt_string) != -1) {
    MT::String a_text;
    a_text.read(mt_string, NULL, ")", 1);
    as.append(getAtom(a_text));
  }
}


void TL::logicObjectManager::getAtoms(AtomL& as, const uintA& arguments) {
  as.clear();
  LitL lits;
  getLiterals(lits, arguments, true);
  uint i;
  FOR1D(lits, i) {
    as.append(lits(i)->atom);
  }
}


void TL::logicObjectManager::getAtoms(AtomL& as, TL::Predicate* pred, const uintA& arguments) {
  as.clear();
  LitL lits;
  getLiterals(lits, pred, arguments, true);
  uint i;
  FOR1D(lits, i) {
    as.append(lits(i)->atom);
  }
}


void TL::logicObjectManager::getLiterals(LitL& lits, const char* text) {
  uint DEBUG = 0;
  if (DEBUG>0) {cout<<"getLiterals [START]"<<endl;}
  if (DEBUG>0) {PRINT(text);}
  lits.clear();
  MT::String mt_string(text);
  MT::String name;
  while (MT::skip(mt_string) != -1) {
    MT::String lit_text;
    lit_text.read(mt_string, NULL, ")", 1);
    if (MT::peerNextChar(mt_string) == ',') MT::skipOne(mt_string);
//     MT::skip(std::istream& is,const char *skipchars=" \n\r\t",bool skipCommentLines=true);
    char c = MT::peerNextChar(mt_string);
    if (c == '='  ||  c == '>'  ||  c == '<') {
      MT::String comparison;
      comparison.read(mt_string, NULL, " \n", 1);
      lit_text << ")" << comparison;
    }
    if (DEBUG>0) {PRINT(lit_text);}
    lits.append(getLiteral(lit_text));
  }
  if (DEBUG>0) {write(lits); cout<<endl;}
  if (DEBUG>0) {cout<<"getLiterals [END]"<<endl;}
}




TL::FunctionAtom* TL::logicObjectManager::getFA(const char* text) {
  uint DEBUG = 0;
  MT::String mt_string(text);
  if (DEBUG>0) {PRINT(mt_string);}
  // Function
  MT::String name;
  name.read(mt_string, NULL, "(");
  if (DEBUG>0) PRINT(name);
  TL::Function* f = getFunction(name);
  if (DEBUG>0) {PRINT(f);  if (f!=NULL) {f->writeNice(); cout<<endl;}}
  // Arguments
  uintA args;
  args.resize(f->d);
  uint i;
  FOR1D(args, i) {
    MT::String arg;
    arg.read(mt_string, NULL, ", )");
    if (isdigit(MT::peerNextChar(arg))) {
      arg >> args(i);
    }
    else {
      int index = string_variables.findValue(arg);
      if (index >= 0) {
        args(i) = index;
      }
      else {
        string_variables.append(arg);
        args(i) = string_variables.N-1;
      }
    }
  }
  if (DEBUG>0) {PRINT(args);}
  TL::FunctionAtom* fa = getFA(f, args);
  if (DEBUG>0) PRINT(*fa);
  return fa;
}


TL::FunctionValue* TL::logicObjectManager::getFV(const char* text) {
  uint DEBUG = 0;
  MT::String mt_string(text);
  if (DEBUG>0) {PRINT(mt_string);}
  // Function
  MT::String name;
  name.read(mt_string, NULL, "(");
  if (DEBUG>0) PRINT(name);
  TL::Function* f = getFunction(name);
  if (DEBUG>0) {PRINT(f);  if (f!=NULL) {f->writeNice(); cout<<endl;}}
  // Arguments
  uintA args;
  args.resize(f->d);
  uint i;
  FOR1D(args, i) {
    MT::String arg;
    arg.read(mt_string, NULL, ", )");
    if (isdigit(MT::peerNextChar(arg))) {
      arg >> args(i);
    }
    else {
      int index = string_variables.findValue(arg);
      if (index >= 0) {
        args(i) = index;
      }
      else {
        string_variables.append(arg);
        args(i) = string_variables.N-1;
      }
    }
  }
  if (DEBUG>0) {PRINT(args);}
  char op;  // '='
  mt_string >> op;
  double value;
  mt_string >> value;
  TL::FunctionValue* fv = getFV(f, args, value);
  if (DEBUG>0) PRINT(*fv);
  return fv;
}

void TL::logicObjectManager::getFVs(FuncVL& fvs, const char* text) {
  fvs.clear();
  MT::String mt_string(text);
  MT::String name;
  while (MT::skip(mt_string) != -1) {
    MT::String fv_text;
    fv_text.read(mt_string, NULL, "\n ", 1);
    fvs.append(getFV(fv_text));
  }
}



















TL::Literal* TL::logicObjectManager::getConjunctionLiteral(TL::ConjunctionPredicate* cp,
              TL::Substitution* sub) {
  uintA args;
  uint i;
  for (i=0; i<cp->d; i++) {
    args.append(sub->getSubs(i));
  }
  return getLiteral(cp, true, args);
}


TL::Literal* TL::logicObjectManager::getTransClosureLiteral(TL::TransClosurePredicate* p, uintA& args) {
    return getLiteral(p, true, args);
}

TL::Literal* TL::logicObjectManager::getCountLiteral(TL::CountPredicate* p, uintA& args) {
    return getLiteral(p, true, args);
}




// only EQUAL so far
void TL::logicObjectManager::getCompLiterals_constantBound(LitL& lits, const uintA& arguments, const TL::SymbolicState& s, uint what) {
  uint DEBUG = 0;
  if (DEBUG>0) cout<<"getCompLiterals [START]"<<endl;
  if (DEBUG>0) {PRINT(arguments); PRINT(what);  cout<<"SymbolicState:  ";s.write();cout<<endl;}
  lits.clear();
  uint i, j, k;
  // equality
  FOR1D(f_prim, i) {
    if (DEBUG>0) cout<<"Looking at function "<<f_prim(i)->name<<endl;
    MT::Array< uintA > lists;
    TL::allPossibleLists(lists, arguments, f_prim(i)->d, true, true);
    FOR1D(lists, j) {
      if (DEBUG>0) cout<<"Looking at possible slotAssignment "<<lists(j)<<endl;
      FOR1D(s.fv_prim, k) {
        if (DEBUG>0) {cout<<"Comparing with function value ";s.fv_prim(k)->write(cout);cout<<endl;}
        if (s.fv_prim(k)->atom->f == f_prim(i)) {
          if (DEBUG>0) {PRINT(s.fv_prim(k)->atom->args);  PRINT(lists(j));}
          if (s.fv_prim(k)->atom->args == lists(j)) {
            lits.append(getCompLiteral_constant(f_prim(i), comparison_equal, s.fv_prim(k)->value, s.fv_prim(k)->atom->args));
            if (DEBUG>0) {cout<<"Found:  "<<*lits.last()<<endl;}
            break;
          }
        }
      }
      if (DEBUG>0) {if(k==s.fv_prim.d0) cout<<"Function value not given."<<endl;}
    }
  }
  
  // equality
  FOR1D(f_derived, i) {
    if (DEBUG>0) cout<<"Looking at function "<<f_derived(i)->name<<endl;
    MT::Array< uintA > lists;
    TL::allPossibleLists(lists, arguments, f_derived(i)->d, true, true);
    FOR1D(lists, j) {
      if (DEBUG>0) cout<<"Looking at possible slotAssignment "<<lists(j)<<endl;
      FOR1D(s.fv_derived, k) {
        if (DEBUG>0) {cout<<"Comparing with function value ";s.fv_derived(k)->write(cout);cout<<endl;}
        if (s.fv_derived(k)->atom->f == f_derived(i)) {
          if (s.fv_derived(k)->atom->args == lists(j)) {
            lits.append(getCompLiteral_constant(f_derived(i), comparison_equal, s.fv_derived(k)->value, s.fv_derived(k)->atom->args));
            break;
          }
        }
      }
      if (DEBUG>0) {if(k==s.fv_derived.d0) cout<<"Function value not given."<<endl;}
    }
  }

  if (what > 0)
      NIY;

  if (DEBUG>0) {cout<<"Found comparison predicate tuples: ";write(lits);cout<<endl;}
  if (DEBUG>0) cout<<"getCompLiterals [END]"<<endl;
}


// works only on unary functions!
void TL::logicObjectManager::getCompLiterals_dynamicBound(LitL& lits, const uintA& arguments, const TL::SymbolicState& s, uint what) {
    lits.clear();
    uint c, f, v;
    TL::FunctionValue* fv1;
    TL::FunctionValue* fv2 = NULL;
    TL::ComparisonLiteral* clit;
    MT::Array< uintA > combos;
    TL::allPossibleLists(combos, arguments, 2, false, true);
    FOR1D(combos, c) {
        if (combos(c)(0) >= combos(c)(1))
            continue;
        FOR1D(f_prim, f) {
            if (f_prim(f)->d != 1)
                continue;
            fv1 = NULL;
            fv2 = NULL;
            FOR1D(s.fv_prim, v) {
                if (s.fv_prim(v)->atom->f != f_prim(f))
                    continue;
                if (s.fv_prim(v)->atom->args(0) == combos(c)(0))
                    fv1 = s.fv_prim(v);
                if (s.fv_prim(v)->atom->args(0) == combos(c)(1))
                    fv2 = s.fv_prim(v);
            }
            if (fv1==NULL || fv2==NULL)
                continue;
            uintA args1, args2;
            uint l;
            for (l=0; l<combos(c).N / 2; l++) {args1.append(combos(c)(l));}
            for (l=combos(c).N / 2; l<combos(c).N; l++) {args2.append(combos(c)(l));}
            if (fv1->value < fv2->value)
                clit = getCompLiteral_dynamic(f_prim(f), comparison_less, args1, args2);
            else if (TL::areEqual(fv1->value, fv2->value))
                clit = getCompLiteral_dynamic(f_prim(f), comparison_equal, args1, args2);
            else
                clit = getCompLiteral_dynamic(f_prim(f), comparison_greater, args1, args2);
            lits.append(clit);
        }
        FOR1D(f_derived, f) {
            if (f_derived(f)->d != 1)
                continue;
            fv1 = NULL;
            fv2 = NULL;
            FOR1D(s.fv_derived, v) {
                if (s.fv_derived(v)->atom->f != f_derived(f))
                    continue;
                if (s.fv_derived(v)->atom->args(0) == combos(c)(0))
                    fv1 = s.fv_derived(v);
                if (s.fv_derived(v)->atom->args(0) == combos(c)(1))
                    fv2 = s.fv_derived(v);
            }
            if (fv1==NULL || fv2==NULL)
                continue;
            uintA args1, args2;
            uint l;
            for (l=0; l<combos(c).N / 2; l++) {args1.append(combos(c)(l));}
            for (l=combos(c).N / 2; l<combos(c).N; l++) {args2.append(combos(c)(l));}
            if (fv1->value < fv2->value)
                clit = getCompLiteral_dynamic(f_derived(f), comparison_less, args1, args2);
            else if (TL::areEqual(fv1->value, fv2->value))
                clit = getCompLiteral_dynamic(f_derived(f), comparison_equal, args1, args2);
            else
                clit = getCompLiteral_dynamic(f_derived(f), comparison_greater, args1, args2);
            lits.append(clit);
        }
    }
}


void TL::logicObjectManager::getLiterals(LitL& lits, TL::Predicate* pred, const uintA& arguments, const bool positiveOnly) {
  uint j, k;
  if (pred->d== 0)  {
    uintA empty;
    TL::Literal* lit = getLiteral(pred, true, empty);
    lits.append(lit);
    if (!positiveOnly) {
      TL::Literal* pt_neg = getLiteral(pred, false, empty);
      lits.append(pt_neg);
    }
  }
  else {
    MT::Array< uintA > lists;
    TL::allPossibleLists(lists, arguments, pred->d, true, false);
    FOR1D(lists, j) {
      if (pred->arg_types.N > 0) {
        FOR1D(lists(j), k) {
          TermType* object_type = getTermTypeOfObject(lists(j)(k));
          if (!pred->arg_types(k)->subsumes(*object_type))
            break;
        }
        if (k != lists(j).N)
          continue;
      }
      TL::Literal* lit = getLiteral(pred, true, lists(j));
      lits.append(lit);
      if (!positiveOnly) {
        TL::Literal* pt_neg = getLiteral(pred, false, lists(j));
        lits.append(pt_neg);
      }
    }
  }
}


void TL::logicObjectManager::getLiterals(LitL& lits, const uintA& arguments, bool positiveOnly) {
  lits.clear();
  uint i;
  FOR1D(p_prim, i) {
    LitL pts_local;
    getLiterals(pts_local, p_prim(i), arguments, positiveOnly);
    lits.setAppend(pts_local);
  }
  FOR1D(p_derived, i) {
    LitL pts_local;
    getLiterals(pts_local, p_derived(i), arguments, positiveOnly);
    lits.setAppend(pts_local);
  }
}



// only normal and p_derived thus far!
void TL::logicObjectManager::getLiterals(LitL& lits, const uintA& arguments, const uintA& arguments_mustBeContained, bool positiveOnly) {
  uint DEBUG=0;
  if (DEBUG>0) cout<<"getLiterals [START]"<<endl;
  lits.clear();
  uint i, j;
  FOR1D(p_prim, i) {
    MT::Array< uintA > lists;
    TL::allPossibleLists(lists, arguments, p_prim(i)->d, true, true);
    FOR1D(lists, j) {
      if (DEBUG>1) cout<<lists(j)<<endl;
      if (numberSharedElements(lists(j), arguments_mustBeContained) > 0) {
        TL::Literal* lit = getLiteral(p_prim(i), true, lists(j));
        lits.append(lit);
        if (!positiveOnly) {
          TL::Literal* pt_neg = getLiteral(p_prim(i), false, lists(j));
          lits.append(pt_neg);
        }
      }
    }
  }
  if (DEBUG>1) write(lits);
  FOR1D(p_derived, i) {
    MT::Array< uintA > lists;
    TL::allPossibleLists(lists, arguments, p_derived(i)->d, true, true);
    FOR1D(lists, j) {
      if (numberSharedElements(lists(j), arguments_mustBeContained) > 0) {
                TL::Literal* lit = getLiteral(p_derived(i), true, lists(j));
        lits.append(lit);
        if (!positiveOnly) {
                    TL::Literal* pt_neg = getLiteral(p_derived(i), false, lists(j));
          lits.append(pt_neg);
        }
      }
    }
  }
  if (DEBUG>0) cout<<"getLiterals [END]"<<endl;
}
  


void TL::logicObjectManager::getAtoms_actions(AtomL& as, const uintA& arguments) {
  as.clear();
  uint i;
  FOR1D(p_actions, i) {
    AtomL as2;
    getAtoms(as2, p_actions(i), arguments);
    as.setAppend(as2);
  }
}

void TL::logicObjectManager::getFAs(FuncAL& fas, Function* f, const uintA& arguments) {
  fas.clear();
  MT::Array< uintA > lists;
  TL::allPossibleLists(lists, arguments, f->d, true, true);
  uint j;
  FOR1D(lists, j) {
    TL::FunctionAtom* fvw = getFA(f, lists(j));
    fas.append(fvw);
  }
}


void TL::logicObjectManager::getFAs(FuncAL& fas, const uintA& arguments) {
  fas.clear();
  uint i;
  FOR1D(f_prim, i) {
    FuncAL fas_local;
    getFAs(fas_local, f_prim(i), arguments);
    fas.setAppend(fas_local);
  }
  FOR1D(f_derived, i) {
    FuncAL fas_local;
    getFAs(fas_local, f_derived(i), arguments);
    fas.setAppend(fas_local);
  }
}





void TL::logicObjectManager::getAllPrecessors(const TL::Predicate& p, PredL& pre_preds, FuncL& pre_funcs) {
    TL::logicObjectManager::dependencyGraph.getAllPrecessors(p, pre_preds, pre_funcs);
}

void TL::logicObjectManager::getAllPrecessors(const TL::Function& f, PredL& pre_preds, FuncL& pre_funcs) {
    TL::logicObjectManager::dependencyGraph.getAllPrecessors(f, pre_preds, pre_funcs);
}



void TL::logicObjectManager::writeLanguage(const char* filename) {
  PredL all_preds;
  all_preds.append(TL::logicObjectManager::p_prim);
  all_preds.append(TL::logicObjectManager::p_derived);
  
  FuncL all_funcs;
  all_funcs.append(TL::logicObjectManager::f_prim);
  all_funcs.append(TL::logicObjectManager::f_derived);
  
  TL::writeLanguage(all_preds, all_funcs, TL::logicObjectManager::p_actions, TL::logicObjectManager::types, filename);
}









TL::Predicate* TL::logicObjectManager::readPredicate(ifstream& in, uintA& baseIds) {
  uint DEBUG = 0;
  if (DEBUG>0) {cout<<"readPredicate [START]"<<endl;}
  MT::String line;
  line.read(in, NULL, "\n");
  if (DEBUG>0) PRINT(line);
  if(line.N == 0) return NULL;
  
  uint id, d, type_uint, category;
  MT::String name;
  
  line >> id;
  name.read(line, NULL, " ");
  line >> d;
  line >> type_uint;
  line >> category;
  
  uint i;
  if (type_uint == TL::Predicate::predicate_action) {
    TL::Predicate* p = new TL::Predicate;
    p->id = id;
    p->name = name;
    p->d = d;
    p->type = TL::Predicate::predicate_action;
    p->arg_types.resize(p->d);
    FOR1D(p->arg_types, i) {
      TermType* t = new TermType;
      line >> t->type_id;
      p->arg_types(i) = t;
    }
    return p;
  }
  else if (type_uint == TL::Predicate::predicate_simple) {
    TL::Predicate* p = new TL::Predicate;
    p->id = id;
    p->name = name;
    p->d = d;
    p->type = TL::Predicate::predicate_simple;
    return p;
  }
  else if (type_uint == TL::Predicate::predicate_comparison) {
    NIY;
  }
  else if (type_uint == TL::Predicate::predicate_conjunction) {
    TL::ConjunctionPredicate* p = new TL::ConjunctionPredicate;
    p->id = id;
    p->name = name;
    p->d = d;
    char bracket_left_dummy;
    line >> bracket_left_dummy;
    uint no_basePreds;
    line >> no_basePreds;
    line >> p->freeVarsAllQuantified;
    line >> bracket_left_dummy;
    for(i=0; i<no_basePreds; i++) {
      line >> id;
      baseIds.append(id);
    }
    char bracket_right_dummy;
    line >> bracket_right_dummy;
    line >> p->basePreds_positive;
    line >> p->basePreds_mapVars2conjunction;
    if (DEBUG>1) {
      PRINT(no_basePreds);
      PRINT(p->freeVarsAllQuantified);
      PRINT(baseIds);
      PRINT(p->basePreds_positive);
      PRINT(p->basePreds_mapVars2conjunction);
    }
    return p;
  }
  else if (type_uint == TL::Predicate::predicate_transClosure) {
    TL::TransClosurePredicate* p = new TL::TransClosurePredicate;
    p->id = id;
    p->name = name;
    p->d = d;
    char bracket_left_dummy;
    line >> bracket_left_dummy;
    line >> id;
    baseIds.append(id);
    char bracket_right_dummy;
    line >> bracket_right_dummy;
    return p;
  }
  else if (type_uint == TL::Predicate::predicate_count) {
    TL::CountPredicate* p = new TL::CountPredicate;
    p->id = id;
    p->name = name;
    p->d = d;
    MT::String trash;
    trash.read(line, " "); // "["
    line >> id;
    baseIds.append(id);
    line >> p->countedPred_mapVars2derived;
    line >> p->bound;
    uint uint__comp_type;
    line >> uint__comp_type;
    switch (uint__comp_type) {
      case 0: p->compType = comparison_equal; break;
      case 1: p->compType = comparison_less; break;
      case 2: p->compType = comparison_lessEqual; break;
      case 3: p->compType = comparison_greater; break;
      case 4: p->compType = comparison_greaterEqual; break;
      default: NIY;
    }
    line >> p->countedPred_isComparison;
    if (p->countedPred_isComparison) {
      line >> id;
      baseIds.append(id);
      line >> p->comparison_bound;
      line >> uint__comp_type;
      switch (uint__comp_type) {
        case 0: p->comparison_compType = comparison_equal; break;
        case 1: p->comparison_compType = comparison_less; break;
        case 2: p->comparison_compType = comparison_lessEqual; break;
        case 3: p->comparison_compType = comparison_greater; break;
        case 4: p->comparison_compType = comparison_greaterEqual; break;
        default: NIY;
      }
    }
    return p;
  }
  else {
    PRINT(type_uint);
    NIY;
    return NULL;
  }
}


TL::Function* TL::logicObjectManager::readFunction(ifstream& in, uintA& baseIds) {
  uint DEBUG = 0;
  MT::String line;
  line.read(in, NULL, "\n");
  if (DEBUG>0) PRINT(line);
  if(line.N == 0) return NULL;
    
  uint id, d, type, category;
  MT::String name;
  uintA range;
  
  line >> id;
  name.read(line, NULL, " ");
  line >> d;
  line >> type;
  line >> category;
  line >> range;
  
  if (type == TL::Function::function_simple) {
    TL::Function* f = new TL::Function;
    f->id = id;
    f->name = name;
    f->d = d;
    f->range = range;
    if (f->range.N ==0) { // if no range specified, set it to [1,2,3,4,5]
      uint h;
      for (h=1; h<=5; h++)
        f->range.append(h);
    }
    return f;
  }
  else if (type == TL::Function::function_count) {
    TL::CountFunction* f = new TL::CountFunction;
    f->id = id;
    f->name = name;
    f->d = d;
    f->range = range;
    if (f->range.N ==0) { // if no range specified, set it to [0,...,9]
      uint h;
      for (h=0; h<=9; h++)
        f->range.append(h);
    }
    line >> id;
    baseIds.append(id);
    line >> f->countedPred_mapVars2derived;
    return f;
  }
  else if (type == TL::Function::function_sum) {
    TL::SumFunction* f = new TL::SumFunction;
    f->id = id;
    f->name = name;
    f->d = d;
    line >> id;
    baseIds.append(id);
    return f;
  }
  else {
    PRINT(type);
    NIY;
    return NULL;
  }
}


TL::TermType* TL::logicObjectManager::readTermType(ifstream& in, const TermTypeL& existing_types) {
  uint DEBUG = 0;
  MT::String line;
  line.read(in, NULL, "\n");
  if (DEBUG>0) PRINT(line);
  if(line.N == 0) return NULL;
 
  uint type_id, typeI;
  MT::String name;
  line >> type_id;
  name.read(line, NULL, " ");
  line >> typeI;
  
  if (typeI == TL::TermType::term_type_simple) {
    TL::TermType* t = new TL::TermType;
    t->type_id = type_id;
    t->name = name;
    if (DEBUG>0) {t->writeNice(); cout<<endl;}
    return t;
  }
  else if (typeI == TL::TermType::term_type_disjunction) {
    uintA base_types__ids;
    line >> base_types__ids;
    
    TermTypeL base_types;
    uint i, k;
    FOR1D(base_types__ids, i) {
      FOR1D(existing_types, k) {
        if (existing_types(k)->type_id == base_types__ids(i)) {
          base_types.append(existing_types(k));
          break;
        }
      }
      CHECK(i < base_types__ids.N, "base type has not been found");
    }
    
    TL::DisjunctionTermType* t = new TL::DisjunctionTermType;
    t->type_id = type_id;
    t->name = name;
    t->base_types = base_types;
    if (DEBUG>0) {t->writeNice(); cout<<endl;}
    return t;
  }
  else
    NIY;
}



void TL::logicObjectManager::readLanguage(const char *filename, PredL& p_prim, PredL& p_derived, PredL& actions, FuncL& f_prim, FuncL& f_derived, TermTypeL& types) {
  ifstream in;
  in.open(filename);
  CHECK(in.is_open(), "File can't be opened!");
  readLanguage(in, p_prim, p_derived, actions, f_prim, f_derived, types);
}


void TL::logicObjectManager::readLanguage(ifstream& in, PredL& p_prim, PredL& p_derived, PredL& actions, FuncL& f_prim, FuncL& f_derived, TermTypeL& types) {
  uint DEBUG = 0;
  p_prim.clear();
  p_derived.clear();
  actions.clear();
  f_prim.clear();
  f_derived.clear();
  types.clear();
  
  // READING
  std::map<uint, uintA> f_id2baseIds;
  std::map<uint, uintA> p_id2baseIds;
  
  // Ignore description beginning of file.
  MT::skipUntil(in, "[");
  MT::skipLine(in); // skip "["
  // (1) Predicates
  while (MT::peerNextChar(in) != ']') {
//       PRINT(MT::peerNextChar(in));
    uintA baseIds;
    TL::Predicate* p = readPredicate(in, baseIds);
    p_id2baseIds[p->id] = baseIds;
    CHECK(p!=NULL, "wrong input format");
    if (DEBUG>0) {cout<<"Read predicate  \""<<p->name<<"\"  baseIds="<<baseIds<<endl;}
    if (p->type == TL::Predicate::predicate_comparison) {
      NIY;
    }
    else if (p->category == category_primitive)
      p_prim.append(p);
    else
      p_derived.append(p);
  }
  // (2) Functions
  MT::skipUntil(in, "[");  // skip "]\n["
  MT::skipLine(in);
  while (MT::peerNextChar(in) != ']') {
    uintA baseIds;
    TL::Function* f = readFunction(in, baseIds);
    f_id2baseIds[f->id] = baseIds;
    CHECK(f!=NULL, "wrong input format");
    if (DEBUG>0) {cout<<"Read function  \""<<f->name<<"\"  baseIds="<<baseIds<<endl;}
    if (f->category == category_primitive)
      f_prim.append(f);
    else
      f_derived.append(f);
  }
  // (3) Actions
  MT::skipUntil(in, "[");  // skip "]\n["
  MT::skipLine(in);
  while (MT::peerNextChar(in) != ']') {
    uintA baseIds;
    TL::Predicate* p = readPredicate(in, baseIds);
    actions.append(p);
  }
  MT::skipLine(in); // skip "]"
  // (4) Types
  if (MT::skip(in) == '[') {
    // build default type
    TL::TermType* default_t = new TermType;
    default_t->type_id = TL::TermType::ANY_id;
    default_t->name = "any";
    types.append(default_t);
    // read in other types
    MT::skipLine(in);
    while (MT::peerNextChar(in) != ']') {
      TL::TermType* t = readTermType(in, types);
      if (t->type_id == TermType::ANY_id) {
        HALT("ANY type should not be specified in file");
      }
      types.append(t);
//         t->writeNice(); cout<<endl;
    }
    // set correct TermType instances for Actions
    uint i, k, l;
    FOR1D(actions, i) {
      FOR1D(actions(i)->arg_types, k) {
        FOR1D(types, l) {
          if (types(l)->type_id == actions(i)->arg_types(k)->type_id) {
            delete actions(i)->arg_types(k);
            actions(i)->arg_types(k) = types(l);
            break;
          }
        }
        if (l == types.N) {
          HALT("Could not find correct type for action with id="<<actions(i)->id);
        }
      }
    }
  }
  else {
    // build default type
    TL::TermType* default_t = new TermType;
    default_t->type_id = TL::TermType::ANY_id;
    default_t->name = "any";
    types.append(default_t);
    
    // set correct TermType instances for Actions
    uint i, k;
    FOR1D(actions, i) {
      FOR1D(actions(i)->arg_types, k) {
        actions(i)->arg_types(k) = default_t;
      }
    }
  }
  
  
  // POSTPROCESSING
  // postprocessing for base predicates
  uint i, k=400, l;
  FOR1D(p_derived, i) {
    if (DEBUG>0) {cout<<"Postproc. of pred. "<<p_derived(i)->name<<" ...   "<<std::flush;}
    if (p_derived(i)->type == TL::Predicate::predicate_conjunction) {
      TL::ConjunctionPredicate* p = dynamic_cast<TL::ConjunctionPredicate*>(p_derived(i));
      CHECK(p!=NULL, "cast failed");
      FOR1D(p_id2baseIds[p->id], k) {
        FOR1D(p_prim, l) {
          if (p_prim(l)->id == p_id2baseIds[p->id](k)) {
            p->basePreds.append(p_prim(l));
            break;
          }
        }
        FOR1D(p_derived, l) {
          if (p_derived(l)->id == p_id2baseIds[p->id](k)) {
            p->basePreds.append(p_derived(l));
            break;
          }
        }
      }
      if (DEBUG>0) {cout<<"done: "; p->writeNice(cout); cout<<endl;}
    }
    else if (p_derived(i)->type == TL::Predicate::predicate_transClosure) {
      TL::TransClosurePredicate* p = dynamic_cast<TL::TransClosurePredicate*>(p_derived(i));
      CHECK(p!=NULL, "cast failed");
      CHECK(p_id2baseIds[p->id].N == 1, "invalid trans closure predicate");
      p->basePred = NULL;
      FOR1D(p_prim, l) {
        if (p_prim(l)->id == p_id2baseIds[p->id](0)) {
          p->basePred = p_prim(l);
          break;
        }
      }
      if (p->basePred == NULL) {
        FOR1D(p_derived, l) {
          if (p_derived(l)->id == p_id2baseIds[p->id](0)) {
            p->basePred = p_derived(l);
            break;
          }
        }
      }
      if (DEBUG>0) {cout<<"done: "; p->writeNice(cout); cout<<endl;}
    }
    else if (p_derived(i)->type == TL::Predicate::predicate_count) {
      TL::CountPredicate* p = dynamic_cast<TL::CountPredicate*>(p_derived(i));
      CHECK(p!=NULL, "cast failed");
      CHECK(p_id2baseIds[p->id].N > 0, "invalid count predicate");
      p->countedPred = NULL;
      FOR1D(p_prim, l) {
        if (p_prim(l)->id == p_id2baseIds[p->id](k)) {
          p->countedPred = p_prim(l);
          break;
        }
      }
      if (p->countedPred== NULL) {
        FOR1D(p_derived, l) {
          if (p_derived(l)->id == p_id2baseIds[p->id](k)) {
            p->countedPred = p_derived(l);
            break;
          }
        }
      }
      if (p->countedPred_isComparison) {
        CHECK(p_id2baseIds[p->id].N == 2, "invalid count predicate with comparison");
        p->comparison_f = NULL;
        FOR1D(f_prim, l) {
          if (f_prim(l)->id == p_id2baseIds[p->id](1)) {
            p->comparison_f = f_prim(l);
            break;
          }
        }
        if (p->comparison_f == NULL) {
          FOR1D(f_derived, l) {
            if (f_derived(l)->id == p_id2baseIds[p->id](1)) {
              p->comparison_f = f_derived(l);
              break;
            }
          }
        }
      }
      if (DEBUG>0) {cout<<"done: "; p->writeNice(cout); cout<<endl;}
    }
  }
  
  FOR1D(f_derived, i) {
    if (DEBUG>0) {cout<<"Postproc. of function "<<f_derived(i)->name<<" ...   "<<std::flush;}
    if (f_derived(i)->type == TL::Function::function_count) {
      TL::CountFunction* f = dynamic_cast<TL::CountFunction*>(f_derived(i));
      CHECK(f!=NULL, "cast failed");
      CHECK(f_id2baseIds[f->id].N == 1, "invalid count function");
      f->countedPred = NULL;
      FOR1D(p_prim, l) {
        if (p_prim(l)->id == f_id2baseIds[f->id](0)) {
          f->countedPred = p_prim(l);
          break;
        }
      }
      if (f->countedPred == NULL) {
        FOR1D(p_derived, l) {
          if (p_derived(l)->id == f_id2baseIds[f->id](0)) {
            f->countedPred = p_derived(l);
            break;
          }
        }
      }
      if (DEBUG>0) {cout<<"done: "; f->writeNice(cout); cout<<endl;}
    }
    else if (f_derived(i)->type == TL::Function::function_sum) {
      TL::SumFunction* f = dynamic_cast<TL::SumFunction*>(f_derived(i));
      CHECK(f!=NULL, "cast failed");
      CHECK(f_id2baseIds[f->id].N == 1, "invalid sum function");
      f->f_base = NULL;
      FOR1D(f_prim, l) {
        if (f_prim(l)->id == f_id2baseIds[f->id](0)) {
          f->f_base = f_prim(l);
          break;
        }
      }
      if (f->f_base == NULL) {
        FOR1D(f_derived, l) {
          if (f_derived(l)->id == f_id2baseIds[f->id](0)) {
            f->f_base = f_derived(l);
            break;
          }
        }
      }
      if (DEBUG>0) {cout<<"done: "; f->writeNice(cout); cout<<endl;}
    }
    else {
      NIY;
    }
  }
  
  // SORTING
  sort(p_prim);
  sort(p_derived);
  sort(f_prim);
  sort(f_derived);
}







void TL::logicObjectManager::readLanguageSimpleFormat(const char* filename, PredL& p_prim, PredL& p_derived, PredL& actions, FuncL& f_prim, FuncL& f_derived, TermTypeL& types) {
  uint DEBUG = 0;
  if (DEBUG>0) {cout<<"readLanguageSimpleFormat [START]"<<endl;}
  MT_MSG("TermTypeL types IGNORED THUS FAR!!");
  
  ifstream in;
  in.open(filename);
  CHECK(in.is_open(), "File can't be opened!");

  p_prim.clear();
  p_derived.clear();
  actions.clear();
  f_prim.clear();
  f_derived.clear();

  // READING
  // Ignore description beginning of file.
  MT::skipUntil(in, "[");
  MT::skipLine(in); // skip "["
  // -----------------------------------
  // Predicates
  uint pred_id = 11;
  while (MT::peerNextChar(in) != ']') {
    MT::skip(in);
//     if (MT::peerNextChar(in) == '#')
//       MT::skipLine(in);
//     MT::skip(in);
    if (MT::peerNextChar(in) != '*') {
      TL::Predicate* p = new Predicate;
      MT::String line;
      line.read(in, NULL, "\n");
      p->name.read(line, NULL, " ");
      line >> p->d;
      p->id = pred_id++;
      p_prim.append(p);
      if (DEBUG>0) {p->writeNice(cout); cout<<endl;}
    }
    else {  // * human-rule1-helper() 0 human-onboard() human-alive()
      MT::skipUntil(in, " ");
      TL::ConjunctionPredicate* p = new ConjunctionPredicate;
      MT::String line;
      line.read(in, NULL, "\n");
      p->name.read(line, NULL, " ");
      line >> p->d;
      // Very simple conjunction predicate: conjunction of positive primitives of 0-arity
      if (p->d == 0) {
        MT_MSG("Reading derived predicate: attention -- severe restrictions (only conjunction of positive primitives of 0-arity)");
        while (MT::skip(line) != -1) {
          MT::String name_base_p;
          name_base_p.read(line, NULL, " ");
  //         PRINT(name_base_p);
          uint q;
          FOR1D(p_prim, q) {
  //           PRINT(p_prim(q)->name);
            if (p_prim(q)->name == name_base_p) {
              p->basePreds.append(p_prim(q));
              p->basePreds_positive.append(true);
              break;
            }
          }
  //         line >> ")";
        }
        p->id = pred_id++;
        p_derived.append(p);
      }
      // Conjunction predicate: conjunction of positive potentially existentially quantified primitives
      else {
//         PRINT(MT::peerNextChar(line));
        if (MT::peerNextChar(line) == '%') {
          line >> "%";
          MT::skip(line);
//           PRINT(MT::peerNextChar(line));
          if (MT::peerNextChar(line) == 'e') {
            p->freeVarsAllQuantified = 0;
            line >> "e";
          }
          else if (MT::peerNextChar(line) == 'a') {
            p->freeVarsAllQuantified = 1;
            line >> "e";
          }
          else
            HALT("");
          while (MT::skip(line,"\n\r\t ,") != -1) {
            // Sub-predicate name
            MT::String name_base_p;
            name_base_p.read(line, NULL, "(");
//             PRINT(name_base_p);
            uint q;
            FOR1D(p_prim, q) {
              if (p_prim(q)->name == name_base_p) {
//                 PRINT(p_prim(q)->name);
                p->basePreds.append(p_prim(q));
                p->basePreds_positive.append(true);
                break;
              }
            }
  //         line >> ")";
            // Sub-predicate args
            while (MT::peerNextChar(line) != ')') {
              uint arg;
              line >> arg;
//               PRINT(arg);
//               PRINT(MT::peerNextChar(line));
              MT::skip(line, ",");
//               PRINT(MT::peerNextChar(line));
              p->basePreds_mapVars2conjunction.append(arg);
            }
            MT::skip(line, ")");
//             PRINT(MT::peerNextChar(line));
          }
          p->id = pred_id++;
          p_derived.append(p);
        }
      }
      if (DEBUG>0) {p->writeNice(cout); cout<<endl;}
    }
  }
  MT::skipUntil(in, "[");  // skip "]\n["
  MT::skipLine(in);
  // -----------------------------------
  // Functions
  uint func_id = 11;
  while (MT::peerNextChar(in) != ']') {
    MT::String line;
    line.read(in, NULL, "\n");
    if (MT::peerNextChar(line) == '*') {
      MT::skip(line, "* ");
      MT::String name_f;
      name_f.read(line, NULL, " ");
//       PRINT(name_f);
      uint arity, type;
      line >> arity;
      MT::skip(line);
      line >> type;
      MT::skip(line);
//       PRINT(arity);
//       PRINT(type);
      if (type == TL::Function::function_count) {
        TL::CountFunction* cf = new TL::CountFunction;
        cf->d = arity;
        cf->name = name_f;
        CHECK(arity == 0, "");
        cf->countedPred_mapVars2derived.append(0);
        MT::String name_base_p;
        name_base_p.read(line, NULL, " ");
//         PRINT(name_base_p);
        uint q;
        FOR1D(p_prim, q) {
          if (p_prim(q)->name == name_base_p) {
//             PRINT(p_prim(q)->name);
            cf->countedPred = p_prim(q);
            break;
          }
        }
        if (cf->countedPred == NULL) {
          FOR1D(p_derived, q) {
            if (p_derived(q)->name == name_base_p) {
//               PRINT(p_derived(q)->name);
              cf->countedPred = p_derived(q);
              break;
            }
          }
        }
        cf->id = func_id++;
        if (DEBUG>0) {cf->writeNice(cout); cout<<endl;}
        f_derived.append(cf);
      }
      else
        HALT("");
    }
    // Funktionen machen mer net.
  }
  MT::skipUntil(in, "[");  // skip "]\n["
  MT::skipLine(in);
  // -----------------------------------
  // Actions
  uint action_id = 1;
  bool action_rewards = false;
  if (MT::peerNextChar(in) == 'R') {
    action_rewards = true;
    MT::skipLine(in);
  }
  while (MT::peerNextChar(in) != ']') {
    TL::Predicate* p = new Predicate;
    MT::String line;
    line.read(in, NULL, "\n");
    p->name.read(line, NULL, " ");
    line >> p->d;
//     if (action_rewards)
//       line >> p->reward;
    p->type = TL::Predicate::predicate_action;
    p->id = action_id++;
    actions.append(p);
    if (DEBUG>0) {p->writeNice(cout); cout<<endl;}
  }
  MT::skipLine(in); // skip "]"

  // SORTING
  sort(p_prim);
  sort(p_derived);
  sort(f_prim);
  sort(f_derived);
  if (DEBUG>0) {cout<<"readLanguageSimpleFormat [END]"<<endl;}
}















TL::Trial* TL::logicObjectManager::readTrial_withConstants(const char* filename, bool take_constants) {
  uint DEBUG = 0;
  if (DEBUG>0) {cout<<"readTrial_withConstants [START]"<<endl;}
  if (DEBUG>0) {PRINT(filename);}
  TL::Trial* trial = new TL::Trial;
  ifstream in(filename);
  if (!in.is_open()) {HALT("File not found: "<<filename);}
  MT::skip(in);
  in >> trial->constants;
  trial->constants.reshape(trial->constants.N);
  if (DEBUG>0) {PRINT(trial->constants);}
  if (take_constants) TL::logicObjectManager::setConstants(trial->constants);
  if (DEBUG>0) {cout<<"Constants have been set."<<endl;}
  bool read_state = true;
  while (MT::skip(in) != -1) {
    if (read_state) {
      TL::SymbolicState* state = new TL::SymbolicState;
      MT::String line;
      line.read(in, NULL, "\n");
      if (DEBUG>0) {cout<<"READING LITERALS:"<<endl; PRINT(line);}
      TL::logicObjectManager::getLiterals(state->lits_prim, line);
      line.read(in, NULL, "\n");
      if (DEBUG>0) {cout<<"READING FUNCTION-VALUES:"<<endl; PRINT(line);}
      TL::logicObjectManager::getFVs(state->fv_prim, line);
      trial->states.append(state);
      // skip derived concepts
      while (MT::peerNextChar(in, "") != 0  &&  MT::peerNextChar(in, "") != -1  &&  MT::peerNextChar(in, "") != '\n') {
        line.read(in, NULL, "\n");
      }
      TL::logicReasoning::derive(state);
    }
    else {
      MT::String line;
      line.read(in, NULL, "\n");
      if (DEBUG>0) {cout<<"READING ACTION:"<<endl; PRINT(line);}
      TL::Atom* action = TL::logicObjectManager::getAtom(line);
      trial->actions.append(action);
    }
    read_state = !read_state;
  }
  if (trial->actions.N + 1 != trial->states.N) {
    HALT("Trial reading from file failed:  bad numbers of actions (" << trial->actions.N << ") and states (" << trial->states.N << ")")
  }
  if (DEBUG>0) {cout<<"readTrial_withConstants [END]"<<endl;}
  return trial;
}







TL::Rule* TL::logicObjectManager::readRule(ifstream& in) {
  uint DEBUG = 0;
  if (DEBUG>0) {cout<<"readRule [START]"<<endl;}
  
  CHECK(in.is_open(), "Input stream ain't open!");
  TL::Rule* r = new TL::Rule;
  
  MT::String line, untrimmed_line;
  
  // Action
  line.read(in, NULL, "\n"); // ACTION:
  if (DEBUG>1) PRINT(line);
  CHECK(line(0)=='A',"bad action (expecting action first);  line="<<line);   CHECK(line.N < 10, "bad action: too short");
  line.read(in, NULL, "\n");
  if (DEBUG>1) PRINT(line);
  AtomL actions_wrapper;
  getAtoms(actions_wrapper, line);
  r->action = actions_wrapper(0);
  if (DEBUG>1) PRINT(*r->action);
  
  // Context
  untrimmed_line.read(in, NULL, "\n"); // CONTEXT:
  line.read(untrimmed_line, NULL, " \n\t");
  if (DEBUG>1) PRINT(line);
  CHECK((line(0)=='C' && line(1)=='O'), "bad context:  line="<<line);
  CHECK(line.N < 10, "bad context due to length (line.N="<<line.N<<"; line=\""<<line<<"\")");
  line.read(in, NULL, "\n");
  if (DEBUG>1) PRINT(line);
  if (line(0) != '-'   ||   (line.N > 1  &&  line(1) != '-'  &&  line(1) != ' '))
    getLiterals(r->context, line);
  
  // Outcomes
  if (DEBUG>0) {cout<<"Reading the outcomes:"<<endl;}
  line.read(in, NULL, "\n"); // OUTCOMES:
  CHECK((line(0)=='O' && line(1)=='U'),"bad outcomes:  "<<line);   CHECK(line.N < 11, "bad outcomes due to length");
  MT::Array< LitL > outcomes;
  bool noise_outcome_has_been_read = false;
  while ( MT::peerNextChar(in) != ' '  &&  MT::peerNextChar(in) != 'P'  &&   MT::peerNextChar(in) != 'C'
                                       &&  MT::peerNextChar(in) != 'R'&&  MT::peerNextChar(in) != 'A') {
    line.read(in, NULL, "=\n");
    if (DEBUG>1) PRINT(line);
    if (line.N<2) {
      cout<<"bad line: "<<endl;
      PRINT(line);
      HALT("");
    }
    double prob;
    line >> prob;
    r->probs.append(prob);
    LitL outcome;
    MT::String rest_line;
    rest_line.read(line);
    if (!(rest_line(0)=='n' && rest_line(1)=='o')) // if not noise outcome
      getLiterals(outcome, rest_line);
    else
      noise_outcome_has_been_read = true;
    r->outcomes.append(outcome);
    if (MT::skip(in) == -1)
      break;
    //  mglw. noch reward fuers outcome einlesen
    if (MT::peerNextChar(in) == 'R') {
      while (r->outcome_rewards.N < r->outcomes.N-1) {  // possibly fill for previous unrewarded outcomes
        r->outcome_rewards.append(0.);
      }
      MT::skipUntil(in, " ");
      double outcome_reward;
      in >> outcome_reward;
      r->outcome_rewards.append(outcome_reward);
      if (MT::skip(in) == -1)
        break;
    }
  }
  if (!noise_outcome_has_been_read) {
    LitL dummy_noise_outcome;
    r->outcomes.append(dummy_noise_outcome);
    r->noise_changes = 0.;
    r->probs.append(1.0 - sum(r->probs));
    if (r->outcome_rewards.N > 0)
      r->outcome_rewards.append(0.);
  }
  if (TL::isZero(r->probs.last())) {
    r->probs(0) -= 0.000001;
    r->probs.last() = 0.000001;  // hack to avoid zero-prob noise-outcome
  }
  
  if (DEBUG>0) {
    cout<<"New rule:"<<endl;
    r->write(cout);
  }
  
  if (DEBUG>0) {cout<<"readRule [END]"<<endl;}
  return r;
}


void TL::logicObjectManager::readRules(const char* filename, RuleSet& rules) {
  uint DEBUG = 0;
  if (DEBUG>0) {cout<<"readRules [START]"<<endl;}
  if (DEBUG>0) {PRINT(filename);}
  rules.clear();
  ifstream in(filename);
//   PRINT(filename);
  if (!in.is_open()) {
    cerr<<"Rule-file " << filename << " can't be opened!"<<endl;
    HALT("");
  }
  
  // read other rules
  while (MT::skip(in) != -1) {
    rules.append(readRule(in));
    if (DEBUG>0) {rules.elem(rules.num()-1)->write();}
  }
  // Technicality to ensure that read rules use the same Literal etc. objects.
  // (These objects are managed by the logicObjectManager object accessed by the RuleEngine.)
  uint i;
  FOR1D_(rules, i) {
    TL::logicObjectManager::makeOriginal(*rules.elem(i));
  }
  // default rule
  if (rules.elem(0)->action->pred->id != TL::DEFAULT_ACTION_PRED__ID) {
    TL::Rule* newDefaultRule = new TL::Rule;
    uintA args_empty;
    newDefaultRule->action = TL::logicObjectManager::getAtom(TL::logicObjectManager::getPredicate(TL::DEFAULT_ACTION_PRED__ID), args_empty);
    LitL sameOutcome;
    newDefaultRule->outcomes.append(sameOutcome);
    LitL noiseOutcome;
    newDefaultRule->outcomes.append(noiseOutcome);
    newDefaultRule->probs.append(0.5);
    newDefaultRule->probs.append(1.-newDefaultRule->probs(0));
    newDefaultRule->noise_changes = 2.0;
    rules.append(newDefaultRule);
  }
  rules.sort();
  if (DEBUG>0) {cout<<"readRules [END]"<<endl;}
}






// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------
//    ConceptDependencyGraph
// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------


void TL::ConceptDependencyGraph::resetDefault() {
    edges.clear();
    derivedStartID=0;
    id2pred.clear();
    pred2graph.clear();
    func2graph.clear();
    graph2pred.clear();
    graph2func.clear();
    predicates.clear();
    functions.clear();
}

void TL::ConceptDependencyGraph::setNodes(PredL& predicates, FuncL& functions) {
  resetDefault();
  this->predicates = predicates;
  this->functions = functions;
  uint graph_id=0;
  uint i;
  edges.resize(predicates.N + functions.N, predicates.N + functions.N);
  edges.setUni(false);
  // (1) Nodes
  // first p_prim
  FOR1D(predicates, i) {
    if (predicates(i)->category == category_primitive) {
      id2pred.append(true);
      pred2graph[predicates(i)->id] = graph_id;
      graph2pred[graph_id] = predicates(i)->id;
      graph_id++;
    }
  }
  FOR1D(functions, i) {
    if (functions(i)->category == category_primitive) {
      id2pred.append(false);
      func2graph[functions(i)->id] = graph_id;
      graph2func[graph_id] = functions(i)->id;
      graph_id++;
    }
  }
  derivedStartID=graph_id;
  // then p_derived
  FOR1D(predicates, i) {
    if (predicates(i)->category == category_derived) {
      id2pred.append(true);
      pred2graph[predicates(i)->id] = graph_id;
      graph2pred[graph_id] = predicates(i)->id;
      graph_id++;
    }
  }
  FOR1D(functions, i) {
    if (functions(i)->category == category_derived) {
      id2pred.append(false);
      func2graph[functions(i)->id] = graph_id;
      graph2func[graph_id] = functions(i)->id;
      graph_id++;
    }
  }
  // (2) Edges
  FOR1D(predicates, i) {
    if (predicates(i)->category == category_derived) {
      // set direct precessors
      if (predicates(i)->type == TL::Predicate::predicate_conjunction) {
        TL::ConjunctionPredicate* scp = dynamic_cast<TL::ConjunctionPredicate*>(predicates(i));
        CHECK(scp!=NULL,"cast failed");
        uint p;
        FOR1D(scp->basePreds, p) {
          edges(pred2graph[scp->basePreds(p)->id], pred2graph[scp->id]) = true;
        }
      }
      else if (predicates(i)->type == TL::Predicate::predicate_transClosure) {
        TL::TransClosurePredicate* tcp = dynamic_cast<TL::TransClosurePredicate*>(predicates(i));
        CHECK(tcp!=NULL,"cast failed");
        edges(pred2graph[tcp->basePred->id], pred2graph[tcp->id]) = true;
      }
      else if (predicates(i)->type == TL::Predicate::predicate_count) {
        TL::CountPredicate* cp = dynamic_cast<TL::CountPredicate*>(predicates(i));
        CHECK(cp!=NULL,"cast failed");
        edges(pred2graph[cp->countedPred->id], pred2graph[cp->id]) = true;
        if (cp->countedPred_isComparison) {
          edges(func2graph[cp->comparison_f->id], pred2graph[cp->id]) = true;
        }
      }
      else
        HALT("unknown predicate type")
    }
  }
  FOR1D(functions, i) {
    if (functions(i)->category == category_derived) {
      // set direct precessors
      if (functions(i)->type == TL::Function::function_count) {
        TL::CountFunction* cf = dynamic_cast<TL::CountFunction*>(functions(i));
        CHECK(cf!=NULL,"cast failed");
        edges(pred2graph[cf->countedPred->id], func2graph[cf->id]) = true;
      }
      else if (functions(i)->type == TL::Function::function_avg) {
        TL::AverageFunction* af = dynamic_cast<TL::AverageFunction*>(functions(i));
        CHECK(af!=NULL,"cast failed");
        edges(func2graph[af->f_base->id], func2graph[af->id]) = true;
      }
      else if (functions(i)->type == TL::Function::function_max) {
        TL::MaxFunction* mf = dynamic_cast<TL::MaxFunction*>(functions(i));
        CHECK(mf!=NULL,"cast failed");
        edges(func2graph[mf->f_base->id], func2graph[mf->id]) = true;
      }
      else if (functions(i)->type == TL::Function::function_sum) {
        TL::SumFunction* sf = dynamic_cast<TL::SumFunction*>(functions(i));
        CHECK(sf!=NULL,"cast failed");
        edges(func2graph[sf->f_base->id], func2graph[sf->id]) = true;
      }
      else if (functions(i)->type == TL::Function::function_reward) {
        TL::RewardFunction* grf = dynamic_cast<TL::RewardFunction*>(functions(i));
        CHECK(grf!=NULL,"cast failed");
        uint l;
        FOR1D(grf->grounded_pis, l) {
          edges(pred2graph[grf->grounded_pis(l)->atom->pred->id], func2graph[grf->id]) = true;
        }
      }
      else
        HALT("unknown function type")
    }
  }
  
  calcWellDefinedOrder(order, order_isPredicate, false);
  calcWellDefinedOrder(order__derOnly, order_isPredicate__derOnly, true);
}

TL::Function* TL::ConceptDependencyGraph::getFunction(uint ID) const {
    uint i;
    FOR1D(functions, i) {
        if (functions(i)->id == ID)
            return functions(i);
    }
    return NULL;
}

TL::Predicate* TL::ConceptDependencyGraph::getPredicate(uint ID) const {
    uint i;
    FOR1D(predicates, i) {
        if (predicates(i)->id == ID)
            return predicates(i);
    }
    return NULL;
}

bool TL::ConceptDependencyGraph::isAcyclic() {
    return TL::isAcyclic(edges);
}


void TL::ConceptDependencyGraph::getAllPrecessors(const TL::Predicate& p, PredL& ps_pre, FuncL& fs_pre) {
    ps_pre.clear();
    fs_pre.clear();
    uint n;
    PredL ps_pre_direct;
    FuncL fs_pre_direct;
    for(n=0; n<edges.d0; n++) {
        if (edges(n, pred2graph[p.id])) {
            if (id2pred(n)) ps_pre_direct.append(getPredicate(graph2pred[n]));
            else fs_pre_direct.append(getFunction(graph2func[n])); 
        }
    }
    ps_pre.append(ps_pre_direct);
    fs_pre.append(fs_pre_direct);
    FOR1D(ps_pre_direct, n) {
        PredL ps_pre_recursive;
        FuncL fs_pre_recursive;
        getAllPrecessors(*ps_pre_direct(n), ps_pre_recursive, fs_pre_recursive);
        ps_pre.append(ps_pre_recursive);
        fs_pre.append(fs_pre_recursive);
    }
    FOR1D(fs_pre_direct, n) {
        PredL ps_pre_recursive;
        FuncL fs_pre_recursive;
        getAllPrecessors(*fs_pre_direct(n), ps_pre_recursive, fs_pre_recursive);
        ps_pre.append(ps_pre_recursive);
        fs_pre.append(fs_pre_recursive);
    }
}

void TL::ConceptDependencyGraph::getAllPrecessors(const TL::Function& f, PredL& ps_pre, FuncL& fs_pre) {
    ps_pre.clear();
    fs_pre.clear();
    uint n;
    PredL ps_pre_direct;
    FuncL fs_pre_direct;
    for(n=0; n<edges.d0; n++) {
        if (edges(n, func2graph[f.id])) {
            if (id2pred(n)) ps_pre_direct.append(getPredicate(graph2pred[n]));
            else fs_pre_direct.append(getFunction(graph2func[n])); 
        }
    }
    ps_pre.append(ps_pre_direct);
    fs_pre.append(fs_pre_direct);
    FOR1D(ps_pre_direct, n) {
        PredL ps_pre_recursive;
        FuncL fs_pre_recursive;
        getAllPrecessors(*ps_pre_direct(n), ps_pre_recursive, fs_pre_recursive);
        ps_pre.append(ps_pre_recursive);
        fs_pre.append(fs_pre_recursive);
    }
    FOR1D(fs_pre_direct, n) {
        PredL ps_pre_recursive;
        FuncL fs_pre_recursive;
        getAllPrecessors(*fs_pre_direct(n), ps_pre_recursive, fs_pre_recursive);
        ps_pre.append(ps_pre_recursive);
        fs_pre.append(fs_pre_recursive);
    }
}

// int HORST_depth = 0;
void TL::ConceptDependencyGraph::getAllSuccessors(const TL::Predicate& p, PredL& ps_suc, FuncL& fs_suc) {
  ps_suc.clear();
  fs_suc.clear();
  uint n;
  PredL ps_suc_direct;
//   HORST_depth++;
//   cout<<"---- ";PRINT(HORST_depth);
//   cout<<"Getting successors of: ";p.writeNice();cout<<endl;
//   writeNice();
//   PRINT(pred2graph[p.id]);
  FuncL fs_suc_direct;
  for(n=0; n<edges.d0; n++) {
    if (edges(pred2graph[p.id], n)) {
      if (id2pred(n)) ps_suc_direct.append(getPredicate(graph2pred[n]));
      else fs_suc_direct.append(getFunction(graph2func[n])); 
    }
  }
  ps_suc.append(ps_suc_direct);
  fs_suc.append(fs_suc_direct);
//   cout<<"ps_suc: ";logicObjectManager::write(ps_suc);if (ps_suc.N == 0) cout<<endl;
//   cout<<"fs_suc: ";logicObjectManager::write(fs_suc);if (fs_suc.N == 0) cout<<endl;
  FOR1D(ps_suc_direct, n) {
    PredL ps_suc_recursive;
    FuncL fs_suc_recursive;
    getAllSuccessors(*ps_suc_direct(n), ps_suc_recursive, fs_suc_recursive);
    ps_suc.setAppend(ps_suc_recursive);
    fs_suc.setAppend(fs_suc_recursive);
  }
  FOR1D(fs_suc_direct, n) {
    PredL ps_suc_recursive;
    FuncL fs_suc_recursive;
    getAllSuccessors(*fs_suc_direct(n), ps_suc_recursive, fs_suc_recursive);
    ps_suc.setAppend(ps_suc_recursive);
    fs_suc.setAppend(fs_suc_recursive);
  }
//   HORST_depth--;
}


void TL::ConceptDependencyGraph::getAllSuccessors(const TL::Function& f, PredL& ps_suc, FuncL& fs_suc) {
  ps_suc.clear();
  fs_suc.clear();
  uint n;
  PredL ps_suc_direct;
  FuncL fs_suc_direct;
//   HORST_depth++;
//   cout<<"---- ";PRINT(HORST_depth);
//   cout<<"Getting successors of: ";f.writeNice();cout<<endl;
//   writeNice();
  for(n=0; n<edges.d0; n++) {
    if (edges(func2graph[f.id], n)) {
      if (id2pred(n)) ps_suc_direct.append(getPredicate(graph2pred[n]));
      else fs_suc_direct.append(getFunction(graph2func[n])); 
    }
  }
  ps_suc.append(ps_suc_direct);
  fs_suc.append(fs_suc_direct);
//   cout<<"ps_suc: ";logicObjectManager::write(ps_suc);if (ps_suc.N == 0) cout<<endl;
//   cout<<"fs_suc: ";logicObjectManager::write(fs_suc);if (fs_suc.N == 0) cout<<endl;
  FOR1D(ps_suc_direct, n) {
    PredL ps_suc_recursive;
    FuncL fs_suc_recursive;
    getAllSuccessors(*ps_suc_direct(n), ps_suc_recursive, fs_suc_recursive);
    ps_suc.setAppend(ps_suc_recursive);
    fs_suc.setAppend(fs_suc_recursive);
  }
  FOR1D(fs_suc_direct, n) {
    PredL ps_suc_recursive;
    FuncL fs_suc_recursive;
    getAllSuccessors(*fs_suc_direct(n), ps_suc_recursive, fs_suc_recursive);
    ps_suc.setAppend(ps_suc_recursive);
    fs_suc.setAppend(fs_suc_recursive);
  }
//   HORST_depth--;
}








void TL::ConceptDependencyGraph::writeNice(ostream& os) {
    uint i;
    os<<"ConceptDependencyGraph:"<<endl;
    os<<"-- Predicates:"<<endl;
    FOR1D(predicates, i) {
        os<<pred2graph[predicates(i)->id]<<": ";predicates(i)->writeNice(cout);cout<<endl;
    }
    os<<"-- Functions:"<<endl;
    FOR1D(functions, i) {
        os<<func2graph[functions(i)->id]<<": ";functions(i)->writeNice(cout);cout<<endl;
    }
    os<<"*Direct* dependencies:"<<endl;
    os<<edges<<endl;
}


void TL::ConceptDependencyGraph::calcWellDefinedOrder(uintA& order, boolA& isPredicate, bool derivedOnly) {
  uint DEBUG = 0;
  if (DEBUG > 0) {
    PRINT(derivedOnly);
    PRINT(edges);
  }
  // standard graph problem
  boolA used(edges.d0);
  used.setUni(false);
  uint n, i;
  boolA edges_copy1, edges_copy2;
  edges_copy1 = edges;
  uintA order_local_ids;
  while (true) {
    edges_copy2 = edges_copy1;
    if (DEBUG>0) {PRINT(edges_copy1);}
    FOR1D(edges_copy1, n) {
      // First condition: no incoming edges
      // Second condition: not in order list yet
      if (!sum(edges_copy1.sub(0, edges_copy1.d0-1, n, n)) && !used(n)) {
        order_local_ids.append(n);
        used(n)=true;
        for(i=0; i<edges_copy1.d1; i++) {
          edges_copy2(n, i) = false;
        }
      }
    }
    if (DEBUG>0) {PRINT(order_local_ids);}
    edges_copy1 = edges_copy2;
    if (order_local_ids.N == edges.d0)
      break;
  }
  if (DEBUG>0) {PRINT(order_local_ids);}
    // map local ids to general ids
  order.clear();
  isPredicate.clear();
  i=0;
  for(; i<order_local_ids.N; i++) {
    if (derivedOnly) {
      if (order_local_ids(i) < derivedStartID)
        continue;
    }
    if (id2pred(order_local_ids(i))) {
      order.append(graph2pred[order_local_ids(i)]);
      isPredicate.append(true);
    }
    else {
      order.append(graph2func[order_local_ids(i)]);
      isPredicate.append(false);
    }
  }
  if (DEBUG>0) {PRINT(order);}
  if (DEBUG>0) {PRINT(isPredicate);}
}

void TL::ConceptDependencyGraph::getWellDefinedOrder(uintA& order, boolA& isPredicate, bool derivedOnly) {
  if (derivedOnly) {
    order = this->order__derOnly;
    isPredicate = this->order_isPredicate__derOnly;
  }
  else {
    order = this->order;
    isPredicate = this->order_isPredicate;
  }
}





