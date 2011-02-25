/*  
    Copyright 2009   Tobias Lang
    
    Homepage:  cs.tu-berlin.de/~lang/
    E-mail:    lang@cs.tu-berlin.de
    
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


#include "logicEngine.h"

#define LE_fast 1


// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------
//    L O G I C   E N G I N E
// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------



TL::LogicEngine::LogicEngine(PredA& p_prim, PredA& p_derived, PredA& p_comp, FuncA& f_prim, FuncA& f_derived, PredA& actions) {
  initLanguage(p_prim, p_derived, p_comp, f_prim, f_derived, actions);
}

TL::LogicEngine::LogicEngine(uintA& constants, PredA& p_prim, PredA& p_derived, PredA& p_comp, FuncA& f_prim, FuncA& f_derived, PredA& actions) {
	setConstants(constants);
  initLanguage(p_prim, p_derived, p_comp, f_prim, f_derived, actions);
}

TL::LogicEngine::LogicEngine(const char* language_file, uint fileType) {
  PredA _p_prim, _p_derived, _p_comp, _actions;
  FuncA _f_prim, _f_derived;
  TermTypeA _types;
  if (fileType == 1)
    TL::readLanguage(language_file, _p_prim, _p_derived, _p_comp, _actions, _f_prim, _f_derived, _types);
  else if (fileType == 2)
    TL::readLanguageSimpleFormat(language_file, _p_prim, _p_derived, _p_comp, _actions, _f_prim, _f_derived, _types);
  else
    HALT("");
  initLanguage(_p_prim, _p_derived, _p_comp, _f_prim, _f_derived, _actions);
  this->types = _types;
}


TL::LogicEngine::~LogicEngine() {
  delete lom;
  uint i=0;
  FOR1D(actions, i) {
    delete actions(i);
  }
  FOR1D(p_prim, i) {
//     PRINT(i);
//     p_prim(i)->writeNice();
    delete p_prim(i);
//     cout<<"  deleted"<<endl;
  }
  FOR1D(p_derived, i) {delete p_derived(i);}
  FOR1D(p_comp, i) {delete p_comp(i);}
  FOR1D(f_prim, i) {delete f_prim(i);}
  FOR1D(f_derived, i) {delete f_derived(i);}
}


void TL::LogicEngine::initLanguage(PredA& _p_prim, PredA& _p_derived, PredA& _p_comp, FuncA& _f_prim, FuncA& _f_derived, PredA& _actions) {
  uint i;
  this->p_prim = _p_prim;   TL::sort(this->p_prim);
  this->p_derived = _p_derived;    TL::sort(this->p_derived);
  this->p_comp = _p_comp;    TL::sort(this->p_comp);
  this->f_prim = _f_prim;    TL::sort(this->f_prim);
  this->f_derived = _f_derived;    TL::sort(this->f_derived);
  
  this->actions = _actions;    TL::sort(this->actions);
  // default action predicate
  TL::Predicate* p_action_default = NULL;
  FOR1D(this->actions, i) {
    if (this->actions(i)->id == TL_DEFAULT_ACTION_PRED__ID) {
      p_action_default = this->actions(i);
      break;
    }
  }
  if (p_action_default == NULL) {
    p_action_default = new TL::Predicate;
    p_action_default->id = TL_DEFAULT_ACTION_PRED__ID;
    p_action_default->name = "default";
    p_action_default->d = 0;
    p_action_default->type = TL_PRED_ACTION;
    this->actions.append(p_action_default);
  }
//   p_action_default->write(cout);  cout<<endl;
  
  
  // LogicObjectManager
  PredA allPreds;
  allPreds.append(p_prim);
  FOR1D(p_derived, i) {
    allPreds.append(p_derived(i));
  }
  FOR1D(p_comp, i) {
    allPreds.append(p_comp(i));
  }
  allPreds.append(this->actions);
    
  FuncA allFunctions;
  allFunctions.append(f_prim);
  allFunctions.append(f_derived);
  lom = new TL::LogicObjectManager(allPreds, allFunctions);
    
  // determine concept dependencies
  PredA ps_all;
  ps_all.append(p_prim);
  ps_all.append(p_derived);
  FuncA fs_all;
  fs_all.append(f_prim);
  fs_all.append(f_derived);
  dependencyGraph.setNodes(ps_all, fs_all);
  CHECK(dependencyGraph.isAcyclic(), "Dependencies contain cycle!");
}


void TL::LogicEngine::setConstants(uintA& constants) {
  this->constants = constants;
}


void TL::LogicEngine::setConstants(uintA& constants, const TermTypeA& constants_types) {
  setConstants(constants);
  this->constants_types = constants_types;
}


void TL::LogicEngine::addPredicates(PredA& preds) {
  uint i;
  FOR1D(preds, i) {
    if (preds(i)->category == TL_DERIVED) {
      TL::DerivedPredicate* dp = dynamic_cast<TL::DerivedPredicate*>(preds(i));
      CHECK(dp!=NULL, "cast failed");
      p_derived.append(dp);
    }
    else
      p_prim.append(preds(i));
  }
  
  // update LogicObjectManager
  FOR1D(preds, i) {
    lom->addPredicate(preds(i));
  }
  
  // update dependency graph
  PredA ps_all;
  ps_all.append(p_prim);
  FOR1D(p_derived, i) ps_all.append(p_derived(i));
  FuncA fs_all;
  fs_all.append(f_prim);
  fs_all.append(f_derived);
  dependencyGraph.setNodes(ps_all, fs_all);
  CHECK(dependencyGraph.isAcyclic(), "Dependencies contain cycle!");
}


void TL::LogicEngine::addActions(PredA& _actions) {
  this->actions.append(_actions);
  // update LogicObjectManager
  uint i;
  FOR1D(_actions, i) {
    lom->addPredicate(_actions(i));
  }
}


void TL::LogicEngine::addFunctions(FuncA& funcs) {
  uint i;
  FOR1D(funcs, i) {
    if (funcs(i)->category == TL_PRIMITIVE)
      f_prim.append(funcs(i));
    else
      f_derived.append(funcs(i));
  }
  
  // update LogicObjectManager
  FOR1D(funcs, i) {
    lom->addFunction(funcs(i));
  }
  
  // update dependency graph
  PredA ps_all;
  ps_all.append(p_prim);
  FOR1D(p_derived, i) ps_all.append(p_derived(i));
  FuncA fs_all;
  fs_all.append(f_prim);
  fs_all.append(f_derived);
  dependencyGraph.setNodes(ps_all, fs_all);
  CHECK(dependencyGraph.isAcyclic(), "Dependencies contain cycle!");
}




// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------
//    H E L P E R   M E T H O D S
// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------

TL::PredicateInstance* PI_doNothing = NULL;


TL::TermType* TL::LogicEngine::getTermTypeOfObject(uint object_id) {
  int idx = constants.findValue(object_id);
  if (idx >= 0) {
    return constants_types(idx);
  }
  else {
    CHECK(types(0)->type_id == TL::TermType::ANY_id, "ANY-TermType is missing");
    return types(0);
  }
}

TL::PredicateInstance* TL::LogicEngine::getPI_doNothing() {
  if (PI_doNothing == NULL)
    create_doNothing_actionAndPI();
  return PI_doNothing;
}

TL::Predicate* TL::LogicEngine::create_doNothing_actionAndPI() {
  TL::Predicate* doNothing = new TL::Predicate;
  uint i;
  uint doNothing__id = 0;
  FOR1D(actions, i) {
    if (actions(i)->id == TL_DEFAULT_ACTION_PRED__ID)
      continue;
    if (actions(i)->id >= doNothing__id)
      doNothing__id = actions(i)->id+1;
  }
  doNothing->id = doNothing__id;
  FOR1D(p_prim, i) {
    CHECK(p_prim(i)->id != doNothing->id, "Invalid action id");
  }
  doNothing->d = 0;
  doNothing->name = "doNothing";
  doNothing->type = TL_PRED_ACTION;
  doNothing->category = TL_PRIMITIVE;
  PredA wrapper;
  wrapper.setAppend(doNothing);
  addActions(wrapper);
  
  // create PT also
  uintA empty;
  PI_doNothing = getPI(doNothing, true, empty);
  
  return doNothing;
}


TL::Predicate* TL::LogicEngine::getPredicate(const MT::String& name) const {
  uint i;
  FOR1D(p_prim, i) {
    if (p_prim(i)->name == name)
      return p_prim(i);
  }
  FOR1D(p_derived, i) {
    if (p_derived(i)->name == name)
      return p_derived(i);
  }
  FOR1D(actions, i) {
    if (actions(i)->name == name)
      return actions(i);
  }
  return NULL;
}

TL::Predicate* TL::LogicEngine::getPredicate(uint id) const {
  TL::Predicate* p = dependencyGraph.getPredicate(id);
  if (p != NULL) {
    return p;
  }
  uint i;
  FOR1D(actions, i) {
    if (actions(i)->id == id)
      return actions(i);
  }
  return NULL;
}


TL::Function* TL::LogicEngine::getFunction(const MT::String& name) const {
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


TL::Function* TL::LogicEngine::getFunction(uint id) const {
  return dependencyGraph.getFunction(id);
}


bool TL::LogicEngine::isConstant(uint obj) {
  return obj >= constants.min();  // Die Objekte koennen sich inzwischen aendern
// 	return constants.findValue(obj) >= 0;
}

uint TL::LogicEngine::calcNumTerms(const TL::PredicateInstance& pt) {
  uintA terms;
  return calcTerms(pt, terms);
}

uint TL::LogicEngine::calcTerms(const TL::PredicateInstance& pt, uintA& terms) {
  uint i;
  FOR1D(pt.args, i) {
    terms.setAppend(pt.args(i));
  }
  return terms.N;
}

uint TL::LogicEngine::calcTerms(const PredIA& pis, uintA& terms) {
  uint i;
  FOR1D(pis, i) {
    uintA local_terms;
    calcTerms(*pis(i), local_terms);
    terms.setAppend(local_terms);
  }
  return terms.N;
}


void TL::LogicEngine::orderPredicates(PredA& p_prim, PredA& p_derived, PredA& p_comp, const PredA& preds) {
  p_prim.clear();
  p_derived.clear();
  p_comp.clear();
    uint i;
    FOR1D(preds, i) {
        if (preds(i)->category == TL_DERIVED)
            p_derived.setAppend((TL::DerivedPredicate*) preds(i));
        else if (preds(i)->type == TL_PRED_COMPARISON)
            p_comp.setAppend((TL::ComparisonPredicate*) preds(i));
        else
            p_prim.setAppend(preds(i));
    }
}

void TL::LogicEngine::orderFunctions(FuncA& f_prim, FuncA& f_derived, const FuncA& funcs) {
  f_prim.clear();
  f_derived.clear();
    uint i;
    FOR1D(funcs, i) {
        if (funcs(i)->category == TL_DERIVED)
            f_derived.setAppend(funcs(i));
        else
            f_prim.setAppend(funcs(i));
    }
}


bool TL::LogicEngine::containsNegativeBasePredicate(TL::Predicate* p) {
    TL::ConjunctionPredicate* cp = dynamic_cast<TL::ConjunctionPredicate*>(p);
	if (cp == NULL)
		return false;
	uint i;
	FOR1D(cp->basePreds_positive, i) {
		if (!cp->basePreds_positive(i))
			return true;
	}
	return false;
}


void TL::LogicEngine::filterState_full(TL::State& s_filtered, const TL::State& s_full, const uintA& filter_objects, bool primOnly) {
  uint i, k;
  
  // pi_prim
  s_filtered.pi_prim.clear();
  FOR1D(s_full.pi_prim, i) {
    FOR1D(s_full.pi_prim(i)->args, k) {
      if (filter_objects.findValue(s_full.pi_prim(i)->args(k))<0)
        break;
    }
    if (k==s_full.pi_prim(i)->args.N)
      s_filtered.pi_prim.append(s_full.pi_prim(i));
  }
  
  // pi_comp
  s_filtered.pi_comp.clear();
  FOR1D(s_full.pi_comp, i) {
    FOR1D(s_full.pi_comp(i)->args, k) {
      if (filter_objects.findValue(s_full.pi_comp(i)->args(k))<0)
        break;
    }
    if (k==s_full.pi_comp(i)->args.N)
      s_filtered.pi_comp.append(s_full.pi_comp(i));
  }
  
  
  // fv_prim
  s_filtered.fv_prim.clear();
  FOR1D(s_full.fv_prim, i) {
    FOR1D(s_full.fv_prim(i)->args, k) {
      if (filter_objects.findValue(s_full.fv_prim(i)->args(k))<0)
        break;
    }
    if (k==s_full.fv_prim(i)->args.N)
      s_filtered.fv_prim.append(s_full.fv_prim(i));
  }

  if (!primOnly) {
    // pi_derived
    s_filtered.pi_derived.clear();
    FOR1D(s_full.pi_derived, i) {
      FOR1D(s_full.pi_derived(i)->args, k) {
        if (filter_objects.findValue(s_full.pi_derived(i)->args(k))<0)
          break;
      }
      if (k==s_full.pi_derived(i)->args.N)
        s_filtered.pi_derived.append(s_full.pi_derived(i));
    }
   
    // fv_derived
    s_filtered.fv_derived.clear();
    FOR1D(s_full.fv_derived, i) {
      FOR1D(s_full.fv_derived(i)->args, k) {
        if (filter_objects.findValue(s_full.fv_derived(i)->args(k))<0)
          break;
      }
      if (k==s_full.fv_derived(i)->args.N)
        s_filtered.fv_derived.append(s_full.fv_derived(i));
    }
  }
  else {
    s_filtered.pi_derived.clear();
    s_filtered.fv_derived.clear();
    s_filtered.derivedDerived = false;
//     derive(&s_filtered);  // don't derive!
  }
}



void TL::LogicEngine::filterState_atleastOne(TL::State& s_filtered, const TL::State& s_full, const uintA& filter_objects, bool primOnly) {
  uint i, k;
  
  // pi_prim
  s_filtered.pi_prim.clear();
  FOR1D(s_full.pi_prim, i) {
    if (s_full.pi_prim(i)->args.N == 0)
      s_filtered.pi_prim.append(s_full.pi_prim(i));
    FOR1D(s_full.pi_prim(i)->args, k) {
      if (filter_objects.findValue(s_full.pi_prim(i)->args(k))>=0) {
        s_filtered.pi_prim.append(s_full.pi_prim(i));
        break;
      }
    }
  }
  
  // pi_comp
  s_filtered.pi_comp.clear();
  FOR1D(s_full.pi_comp, i) {
    if (s_full.pi_comp(i)->args.N == 0)
      s_filtered.pi_comp.append(s_full.pi_comp(i));
    FOR1D(s_full.pi_comp(i)->args, k) {
      if (filter_objects.findValue(s_full.pi_comp(i)->args(k))>=0) {
        s_filtered.pi_comp.append(s_full.pi_comp(i));
        break;
      }
    }
  }
  
  // fv_prim
  s_filtered.fv_prim.clear();
  FOR1D(s_full.fv_prim, i) {
    if (s_full.fv_prim(i)->args.N == 0)
      s_filtered.fv_prim.append(s_full.fv_prim(i));
    FOR1D(s_full.fv_prim(i)->args, k) {
      if (filter_objects.findValue(s_full.fv_prim(i)->args(k))>=0) {
        s_filtered.fv_prim.append(s_full.fv_prim(i));
        break;
      }
    }
  }

  if (!primOnly) {
    // pi_derived
    s_filtered.pi_derived.clear();
    FOR1D(s_full.pi_derived, i) {
      if (s_full.pi_derived(i)->args.N == 0)
        s_filtered.pi_derived.append(s_full.pi_derived(i));
      FOR1D(s_full.pi_derived(i)->args, k) {
        if (filter_objects.findValue(s_full.pi_derived(i)->args(k))>=0) {
          s_filtered.pi_derived.append(s_full.pi_derived(i));
          break;
        }
      }
    }
    
    // fv_derived
    s_filtered.fv_derived.clear();
    FOR1D(s_full.fv_derived, i) {
      if (s_full.fv_derived(i)->args.N == 0)
        s_filtered.fv_derived.append(s_full.fv_derived(i));
      FOR1D(s_full.fv_derived(i)->args, k) {
        if (filter_objects.findValue(s_full.fv_derived(i)->args(k))>=0) {
          s_filtered.fv_derived.append(s_full.fv_derived(i));
          break;
        }
      }
    }
  }
  else {
    s_filtered.pi_derived.clear();
    s_filtered.fv_derived.clear();
    s_filtered.derivedDerived = false;
//     derive(&s_filtered);  // don't derive!
  }
}


void TL::LogicEngine::getConstants(const FuncVA& fvs, uintA& constants) {
    constants.clear();
    uint i, s;
    FOR1D(fvs, i) {
        FOR1D(fvs(i)->args, s) {
            constants.setAppend(fvs(i)->args(s));
        }
    }
}


void TL::LogicEngine::getConstants(const PredIA& pts, uintA& constants) {
  constants.clear();
  uint i, s;
  FOR1D(pts, i) {
    FOR1D(pts(i)->args, s) {
      constants.setAppend(pts(i)->args(s));
    }
  }
}



void TL::LogicEngine::getConstants(const TL::State& s, uintA& constants) {
    uintA localConstants;
    getConstants(s.pi_prim, localConstants);
    constants.setAppend(localConstants);
    localConstants.clear();
    
    getConstants(s.pi_comp, localConstants);
    constants.setAppend(localConstants);
    localConstants.clear();
    
    getConstants(s.pi_derived, localConstants);
    constants.setAppend(localConstants);
    localConstants.clear();
    
    getConstants(s.fv_prim, localConstants);
    constants.setAppend(localConstants);
    localConstants.clear();
    
    getConstants(s.fv_derived, localConstants);
    constants.setAppend(localConstants);
    localConstants.clear();
    
    TL::sort_asc(constants);
}


uint TL::LogicEngine::getArgument(const TL::State& s, const TL::Predicate& pred) {
  CHECK(pred.d == 1, "");
  uintA args;
  getArguments(args, s, pred);
  if (args.N == 1)
    return args(0);
  else
    return UINT_MAX;
}

void TL::LogicEngine::getArguments(uintA& args, const TL::State& s, const TL::Predicate& pred) {
  args.clear();
  uint i;
  if (pred.category == TL_PRIMITIVE) {
    FOR1D(s.pi_prim, i) {
      if (*s.pi_prim(i)->pred == pred)
        args.setAppend(s.pi_prim(i)->args);
    }
  }
  else {
    FOR1D(s.pi_derived, i) {
      if (*s.pi_derived(i)->pred == pred)
        args.setAppend(s.pi_derived(i)->args);
    }
  }
}


void TL::LogicEngine::getRelatedObjects(uintA& objs_related, uint id, bool id_covers_first, const TL::Predicate& pred, const TL::State& s) {
  objs_related.clear();
  uint i;
  if (pred.category == TL_PRIMITIVE) {
    FOR1D(s.pi_prim, i) {
      if (*s.pi_prim(i)->pred == pred) {
        if (id_covers_first) {
          if (s.pi_prim(i)->args(0) == id)
            objs_related.append(s.pi_prim(i)->args(1));
        }
        else {
          if (s.pi_prim(i)->args(1) == id)
            objs_related.append(s.pi_prim(i)->args(0));
        }
      }
    }
  }
  else {
    FOR1D(s.pi_derived, i) {
      if (*s.pi_derived(i)->pred == pred) {
        if (id_covers_first) {
          if (s.pi_derived(i)->args(0) == id)
            objs_related.append(s.pi_derived(i)->args(1));
        }
        else {
          if (s.pi_derived(i)->args(1) == id)
            objs_related.append(s.pi_derived(i)->args(0));
        }
      }
    }
  }
  
}


void TL::LogicEngine::getGeneralRelatedObjectsX(uintA& objs_related, uint id, const TL::State& s) {
  objs_related.clear();
  uint i;
  FOR1D(s.pi_prim, i) {
    if (s.pi_prim(i)->pred->d > 1) {
      if (s.pi_prim(i)->args.findValue(id) >= 0)
        objs_related.setAppend(s.pi_prim(i)->args);
    }
  }
  FOR1D(s.pi_derived, i) {
    if (s.pi_derived(i)->pred->d > 1) {
      if (s.pi_derived(i)->args.findValue(id) >= 0)
        objs_related.setAppend(s.pi_derived(i)->args);
    }
  }
  objs_related.removeValueSafe(id);
}




void TL::LogicEngine::getValues(arr& values, const TL::State& s, const TL::Function& f, const uintA& objs) {
  uint i;
  int k;
  values.resize(objs.N);
  if (f.category == TL_PRIMITIVE) {
    FOR1D(s.fv_prim, i) {
      if (*s.fv_prim(i)->f == f) {
        k = objs.findValue(s.fv_prim(i)->args(0));
        if (k >= 0)
          values(k) = s.fv_prim(i)->value;
      }
    }
  }
  else {
    FOR1D(s.fv_derived, i) {
      if (*s.fv_derived(i)->f == f) {
        k = objs.findValue(s.fv_derived(i)->args(0));
        if (k >= 0)
          values(objs.findValue(s.fv_derived(i)->args(0))) = s.fv_derived(i)->value;
      }
    }
  }
}

double TL::LogicEngine::getValue(uint id, const MT::String& function_name, const TL::State& s) {
  return getValue(id, getFunction(function_name), s);
}

double TL::LogicEngine::getValue(TL::Function* f, const TL::State& s) {
  CHECK(f->d == 0, "incorrect arity of function");
  uint i;
  if (f->category == TL_PRIMITIVE) {
    FOR1D(s.fv_prim, i) {
      if (s.fv_prim(i)->f == f) {
        return s.fv_prim(i)->value;
      }
    }
  }
  else {
    FOR1D(s.fv_derived, i) {
      if (s.fv_derived(i)->f == f) {
        return s.fv_derived(i)->value;
      }
    }
  }
  HALT("no function value found");
  return -10000000;
}

double TL::LogicEngine::getValue(uint id, TL::Function* f, const TL::State& s) {
  uint i;
  if (f->category == TL_PRIMITIVE) {
    FOR1D(s.fv_prim, i) {
      if (s.fv_prim(i)->f == f) {
        if (s.fv_prim(i)->args(0) == id)
          return s.fv_prim(i)->value;
      }
    }
  }
  else {
    FOR1D(s.fv_derived, i) {
      if (s.fv_derived(i)->f == f) {
        if (s.fv_derived(i)->args(0) == id)
          return s.fv_derived(i)->value;
      }
    }
  }
  HALT("no function value found");
  return -10000000;
}


double TL::LogicEngine::getValue(const uintA& args, TL::Function* f, const TL::State& s) {
  uint i;
  if (f->category == TL_PRIMITIVE) {
    FOR1D(s.fv_prim, i) {
      if (s.fv_prim(i)->f == f) {
        if (s.fv_prim(i)->args == args)
          return s.fv_prim(i)->value;
      }
    }
  }
  else {
    FOR1D(s.fv_derived, i) {
      if (s.fv_derived(i)->f == f) {
        if (s.fv_derived(i)->args == args)
          return s.fv_derived(i)->value;
      }
    }
  }
  HALT("no function value found");
  return -10000000;
}


bool TL::LogicEngine::holds_straight(uint id, const MT::String& predicate_name, const TL::State& s) const {
  TL::Predicate* pred = getPredicate(predicate_name);
  uint i;
  if (pred->category == TL_PRIMITIVE) {
    FOR1D(s.pi_prim, i) {
      if (s.pi_prim(i)->pred == pred) {
        if (s.pi_prim(i)->args(0) == id)
          return true;
      }
    }
  }
  else {
    FOR1D(s.pi_derived, i) {
      if (s.pi_derived(i)->pred == pred) {
        if (s.pi_derived(i)->args(0) == id)
          return true;
      }
    }
  }
  return false;
}



bool TL::LogicEngine::negativesLast(const PredIA& predTs) {
	bool positivesDone = false;
	uint i;
	FOR1D(predTs, i) {
// 		PRINT(predTs(i)->positive)
		if (!predTs(i)->positive && !positivesDone) {
			positivesDone = true;
			continue;
		}
		else if (predTs(i)->positive && positivesDone)
			return false;
	}
	return true;
}

bool TL::LogicEngine::negativesUngroundedLast(const PredIA& predTs) {
	bool negativesUngroundedStarted = false;
	uint i;
	FOR1D(predTs, i) {
		if (!isGrounded(predTs(i)) && !predTs(i)->positive && !negativesUngroundedStarted) {
			negativesUngroundedStarted = true;
			continue;
		}
		else if (predTs(i)->positive && negativesUngroundedStarted)
			return false;
	}
	return true;
}


uint TL::LogicEngine::numberLiterals(const MT::Array< PredIA >& PredIAs) {
	uint no = 0;
	uint i;
	FOR1D(PredIAs, i) {
		no += PredIAs(i).N;
	}
	return no;
}


bool TL::LogicEngine::containsNegativePredicateInstances(const PredIA& PredIAs) {
	uint i;
	FOR1D(PredIAs, i) {
		if (!PredIAs(i)->positive)
			return true;
	}
	return false;
}


bool TL::LogicEngine::containsNegativePredicateInstances(const TL::State& s) {
  if (containsNegativePredicateInstances(s.pi_prim))
		return true;
  if (containsNegativePredicateInstances(s.pi_comp))
		return true;
	return false;
}


// positives first
void TL::LogicEngine::order(PredIA& pis) {
  
//   cout<<endl<<endl; TL::writeNice(pis); cout<<endl;
  
  // sowas aehnliches schon mal in RuleSet::sort
  uint i;
  uintA keys(pis.N);
  // Stellen 1-4: Argumente
  // Stellen 5-6: Praedikate-ID
  // Stelle 7: primitive Praedikat
  // Stelle 8: positiv
  FOR1D(pis, i) {
    uint key = 0;
    // Stellen 1-4: Argumente
    if (pis(i)->args.N > 0) {
      CHECK(pis(i)->args(0) < 100, "");
      key += 10e2 * pis(i)->args(0);
//       key += 10e2 * (99 - pis(i)->args(0));
    }
    if (pis(i)->args.N > 1) {
      CHECK(pis(i)->args(1) < 100, "");
      key += pis(i)->args(1);
//       key += (99 - pis(i)->args(1));
    }
    CHECK(pis(i)->args.N <= 2, "");
    // Stellen 5-6: Praedikate-ID
    key += 10e4 * pis(i)->pred->id;
    // Stelle 7: primitive Praedikat
    if (pis(i)->pred->category == TL_DERIVED) {
      key += 10e6;
    }
    // Stelle 8: positiv
    if (!pis(i)->positive) {
      key += 10e7;
    }
    keys(i) = key;
//     cout<<"("<<i<<") "<<*pis(i)<<"\t"<<keys(i)<<endl;
  }

  uintA sortedIndices;
//   PRINT(sortedIndices);
  TL::sort_asc_keys(sortedIndices, keys);
//   PRINT(sortedIndices);

  PredIA pis_sorted;
  FOR1D(sortedIndices, i) {
    pis_sorted.append(pis(sortedIndices(i)));
  }

  pis = pis_sorted;
  
//   TL::writeNice(pis); cout<<endl<<endl;
}


void TL::LogicEngine::orderNegativesUngroundedLast(PredIA& p) {
    PredIA p_ordered;
    PredIA p_negUngrounded;
    uint i;
    FOR1D(p, i) {
        if (!p(i)->positive && !isGrounded(p(i))) {
            p_negUngrounded.append(p(i));
        }
        else
            p_ordered.append(p(i));
    }
    p_ordered.append(p_negUngrounded);
    p=p_ordered;
}




bool TL::LogicEngine::isEqualityLiteral(TL::PredicateInstance* pt) {
    if (pt->pred->type != TL_PRED_COMPARISON)
        return false;
    TL::ComparisonPredicateInstance* cpt = dynamic_cast<TL::ComparisonPredicateInstance*>(pt);
    CHECK(cpt!=NULL, "cast failed")
    if (cpt->comparisonType == TL_COMPARISON_EQUAL)
        return true;
    else
        return false;
}


void TL::LogicEngine::removeRedundant(PredIA& p) {
	PredIA p1;
	uint i, j;
	FOR1D(p, i) {
		FOR1D(p1, j) {
			if (*p(i) == *p1(j))
				break;
		}
		if (j == p1.N)
			p1.append(p(i));
	}
	p = p1;
}








void TL::LogicEngine::generatePredicateInstances(TL::Predicate* p, const uintA& arguments, PredIA& pts) {
    MT::Array< uintA > combos;
    TL::allPossibleLists(combos, arguments, p->d, true, true);
    uint c;
    FOR1D(combos, c) {
        pts.append(getPI(p, true, combos(c)));
    }
}


TL::PredicateInstance* TL::LogicEngine::generateConjunctionPredicateInstance(TL::ConjunctionPredicate* cp,
							TL::Substitution* sub) {
	uintA args;
	uint i;
	for (i=0; i<cp->d; i++) {
		args.append(sub->getSubs(i));
	}
	return getPI(cp, true, args);
}


TL::PredicateInstance* TL::LogicEngine::generateTransClosurePredicateInstance(TL::TransClosurePredicate* p, uintA& args) {
    return getPI(p, true, args);
}

TL::PredicateInstance* TL::LogicEngine::generateCountPredicateInstance(TL::CountPredicate* p, uintA& args) {
    return getPI(p, true, args);
}


TL::FunctionValue* TL::LogicEngine::generateFunctionValue(TL::Function* f, uintA& args, double value) {
    return getFV(f, args, value);
}



void TL::LogicEngine::generateBaseInstantiations(PredIA& pts_base, FuncVA& fvs_base, TL::PredicateInstance* pt) {
  generateBaseInstantiations(pts_base, fvs_base, pt, this->constants);
}

void TL::LogicEngine::generateBaseInstantiations(PredIA& pts_base, FuncVA& fvs_base, TL::PredicateInstance* pt, uintA& free_constants) {
    uint DEBUG = 0;
    if (pt->pred->category == TL_PRIMITIVE)
        return;
    uint i, k, c;
    if (pt->pred->type == TL_PRED_CONJUNCTION) {
        TL::ConjunctionPredicate* scp = (TL::ConjunctionPredicate*) pt->pred;
        if (DEBUG>0) {scp->writeNice(cout);cout<<endl;}
        uint l=0;
        uint basePred_startID;
        FOR1D(scp->basePreds, i) {
            uintA localFreeVars;
            basePred_startID=l;
            for (k=0; k<scp->basePreds(i)->d; k++) {
                if (scp->basePreds_mapVars2conjunction(basePred_startID+k) >= scp->d)
                    localFreeVars.setAppend(scp->basePreds_mapVars2conjunction(basePred_startID+k));
            }
            if (localFreeVars.N > 0) {
                MT::Array< uintA > combos;
                TL::allPossibleLists(combos, free_constants, localFreeVars.N, true, true);
                FOR1D(combos, c) {
                    uintA base_args(scp->basePreds(i)->d);
                    FOR1D(base_args, k) {
                        if (scp->basePreds_mapVars2conjunction(basePred_startID+k) < scp->d)
                            base_args(k) = pt->args(scp->basePreds_mapVars2conjunction(basePred_startID+k));
                        else
                            base_args(k) = combos(c)(scp->basePreds_mapVars2conjunction(basePred_startID+k) - scp->d);
                    }
                    TL::PredicateInstance* base_pt = getPI(scp->basePreds(i), scp->basePreds_positive(i), base_args);
                    pts_base.append(base_pt);
                }
            }
            else {
                uintA base_args(scp->basePreds(i)->d);
                FOR1D(base_args, k) {
                    base_args(k) = pt->args(scp->basePreds_mapVars2conjunction(basePred_startID+k));
                }
                TL::PredicateInstance* base_pt = getPI(scp->basePreds(i), scp->basePreds_positive(i), base_args);
                pts_base.append(base_pt);
            }
            basePred_startID += scp->basePreds(i)->d;
        }
    }
    else if (pt->pred->type == TL_PRED_COUNT) {
        NIY
//         TL::CountPredicate* cp = (TL::CountPredicate*) p->pred;
//         uintA base_args(scp->basePreds(i)->d);
//         FOR1D(base_args, k) {
//             if (scp->basePreds_mapVars2conjunction(k) < scp->basePreds(i)->d)
//                 base_args(k) = p->args(scp->basePreds_mapVars2conjunction(k));
//             else
//                 base_args(k) = scp->basePreds_mapVars2conjunction(k);
//         }
//         countedPred_mapVars2derived
//                 
//         TL::PredicateInstance* pt = getPI(cp->countedPred, true, base_args);
//         pts_base.append(pt);
    }
    else if (pt->pred->type == TL_PRED_TRANS_CLOSURE) {
        TL::TransClosurePredicate* tcp = (TL::TransClosurePredicate*) pt->pred;
        CHECK(tcp->d == 2, "tcps so far implemented nur fuer binary");
        PredIA pts_base_cands;
        generatePredicateInstances(tcp->basePred, free_constants, pts_base_cands);
        FOR1D(pts_base_cands, i) {
            if (pt->args(0) != pts_base_cands(i)->args(1)
                &&  pt->args(1) != pts_base_cands(i)->args(0))
                pts_base.append(pts_base_cands(i));
        }
    }
    else {
        NIY
    }
}


// only EQUAL so far
void TL::LogicEngine::generateComparisonPredicateInstances(PredIA& pis, const uintA& arguments, const TL::State& s, uint what) {
  uint DEBUG = 0;
  if (DEBUG>0) cout<<"generateComparisonPredicateInstances [START]"<<endl;
  if (DEBUG>0) {PRINT(arguments) PRINT(what)}
  pis.clear();
  uint i, j, k;
  // equality
  FOR1D(f_prim, i) {
    if (DEBUG>0) cout<<"Looking at function "<<f_prim(i)->name<<endl;
    MT::Array< uintA > lists;
    TL::allPossibleLists(lists, arguments, f_prim(i)->d, true, true);
    FOR1D(lists, j) {
      if (DEBUG>0) cout<<"Looking at possible slotAssignment "<<lists(j)<<endl;
      FOR1D(s.fv_prim, k) {
        if (DEBUG>0) {cout<<"Comparing with function value ";s.fv_prim(k)->writeNice(cout);cout<<endl;}
        if (s.fv_prim(k)->f == f_prim(i)) {
          if (s.fv_prim(k)->args == lists(j)) {
            pis.append(getCompPT_constant(f_prim(i), TL_COMPARISON_EQUAL, s.fv_prim(k)->value, s.fv_prim(k)->args));
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
        if (DEBUG>0) {cout<<"Comparing with function value ";s.fv_derived(k)->writeNice(cout);cout<<endl;}
        if (s.fv_derived(k)->f == f_derived(i)) {
          if (s.fv_derived(k)->args == lists(j)) {
            pis.append(getCompPT_constant(f_derived(i), TL_COMPARISON_EQUAL, s.fv_derived(k)->value, s.fv_derived(k)->args));
            break;
          }
        }
      }
      if (DEBUG>0) {if(k==s.fv_derived.d0) cout<<"Function value not given."<<endl;}
    }
  }

  if (what > 0)
      NIY;

//   if (DEBUG>0) {cout<<"Found comparison predicate tuples: ";writeNice(pis);cout<<endl;}
  if (DEBUG>0) cout<<"generateComparisonPredicateInstances [END]"<<endl;
}


// works only on unary functions!
void TL::LogicEngine::generateComparisonPredicateInstances_dynamic(PredIA& pts, const uintA& arguments, const TL::State& s, uint what) {
    pts.clear();
    uint c, f, v;
    TL::FunctionValue* fv1;
    TL::FunctionValue* fv2 = NULL;
    TL::ComparisonPredicateInstance* cpt;
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
                if (s.fv_prim(v)->f != f_prim(f))
                    continue;
                if (s.fv_prim(v)->args(0) == combos(c)(0))
                    fv1 = s.fv_prim(v);
                if (s.fv_prim(v)->args(0) == combos(c)(1))
                    fv2 = s.fv_prim(v);
            }
            if (fv1==NULL || fv2==NULL)
                continue;
            if (fv1->value < fv2->value)
                cpt = getCompPT_dynamic(f_prim(f), TL_COMPARISON_LESS, combos(c));
            else if (TL::areEqual(fv1->value, fv2->value))
                cpt = getCompPT_dynamic(f_prim(f), TL_COMPARISON_EQUAL, combos(c));
            else
                cpt = getCompPT_dynamic(f_prim(f), TL_COMPARISON_GREATER, combos(c));
            pts.append(cpt);
        }
        FOR1D(f_derived, f) {
            if (f_derived(f)->d != 1)
                continue;
            fv1 = NULL;
            fv2 = NULL;
            FOR1D(s.fv_derived, v) {
                if (s.fv_derived(v)->f != f_derived(f))
                    continue;
                if (s.fv_derived(v)->args(0) == combos(c)(0))
                    fv1 = s.fv_derived(v);
                if (s.fv_derived(v)->args(0) == combos(c)(1))
                    fv2 = s.fv_derived(v);
            }
            if (fv1==NULL || fv2==NULL)
                continue;
            if (fv1->value < fv2->value)
                cpt = getCompPT_dynamic(f_derived(f), TL_COMPARISON_LESS, combos(c));
            else if (TL::areEqual(fv1->value, fv2->value))
                cpt = getCompPT_dynamic(f_derived(f), TL_COMPARISON_EQUAL, combos(c));
            else
                cpt = getCompPT_dynamic(f_derived(f), TL_COMPARISON_GREATER, combos(c));
            pts.append(cpt);
        }
    }
}


void TL::LogicEngine::generateAllPossiblePredicateInstances(PredIA& pts, TL::Predicate* pred, const uintA& arguments, bool positiveOnly) {
  uint j, k;
  if (pred->d== 0)  {
    uintA empty;
    TL::PredicateInstance* pt = getPI(pred, true, empty);
    pts.append(pt);
    if (!positiveOnly) {
      TL::PredicateInstance* pt_neg = getPI(pred, false, empty);
      pts.append(pt_neg);
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
      TL::PredicateInstance* pt = getPI(pred, true, lists(j));
      pts.append(pt);
      if (!positiveOnly) {
        TL::PredicateInstance* pt_neg = getPI(pred, false, lists(j));
        pts.append(pt_neg);
      }
    }
  }
}


void TL::LogicEngine::generateAllPossiblePredicateInstances(PredIA& pts, const uintA& arguments, bool positiveOnly) {
	pts.clear();
	uint i;
	FOR1D(p_prim, i) {
    PredIA pts_local;
    generateAllPossiblePredicateInstances(pts_local, p_prim(i), arguments, positiveOnly);
    pts.setAppend(pts_local);
	}
	FOR1D(p_derived, i) {
    PredIA pts_local;
    generateAllPossiblePredicateInstances(pts_local, p_derived(i), arguments, positiveOnly);
    pts.setAppend(pts_local);
  }
}



// only normal and p_derived thus far!
void TL::LogicEngine::generateAllPossiblePredicateInstances(PredIA& pts, const uintA& arguments, const uintA& arguments_mustBeContained, bool positiveOnly) {
  uint DEBUG=0;
  if (DEBUG>0) cout<<"generateAllPossiblePredicateInstances [START]"<<endl;
  pts.clear();
  uint i, j;
  FOR1D(p_prim, i) {
    MT::Array< uintA > lists;
    TL::allPossibleLists(lists, arguments, p_prim(i)->d, true, true);
    FOR1D(lists, j) {
      if (DEBUG>1) cout<<lists(j)<<endl;
      if (numberSharedElements(lists(j), arguments_mustBeContained) > 0) {
        TL::PredicateInstance* pt = getPI(p_prim(i), true, lists(j));
        pts.append(pt);
        if (!positiveOnly) {
          TL::PredicateInstance* pt_neg = getPI(p_prim(i), false, lists(j));
          pts.append(pt_neg);
        }
      }
    }
  }
  if (DEBUG>1) writeNice(pts);
  FOR1D(p_derived, i) {
    MT::Array< uintA > lists;
    TL::allPossibleLists(lists, arguments, p_derived(i)->d, true, true);
    FOR1D(lists, j) {
      if (numberSharedElements(lists(j), arguments_mustBeContained) > 0) {
                TL::PredicateInstance* pt = getPI(p_derived(i), true, lists(j));
        pts.append(pt);
        if (!positiveOnly) {
                    TL::PredicateInstance* pt_neg = getPI(p_derived(i), false, lists(j));
          pts.append(pt_neg);
        }
      }
    }
  }
  if (DEBUG>0) cout<<"generateAllPossiblePredicateInstances [END]"<<endl;
}
  


void TL::LogicEngine::generateAllPossiblePredicateInstances_actions(PredIA& pts, const uintA& arguments) {
  pts.clear();
  uint i;
  FOR1D(actions, i) {
    PredIA pts_local;
    generateAllPossiblePredicateInstances(pts_local, actions(i), arguments, true);
    pts.setAppend(pts_local);
  }
}

void TL::LogicEngine::generateAllPossibleFunctionInstances(FuncIA& fvws, Function* f, const uintA& arguments) {
  fvws.clear();
  MT::Array< uintA > lists;
  TL::allPossibleLists(lists, arguments, f->d, true, true);
  uint j;
  FOR1D(lists, j) {
    TL::FunctionInstance* fvw = getFI(f, lists(j));
    fvws.append(fvw);
  }
}


void TL::LogicEngine::generateAllPossibleFunctionInstances(FuncIA& fvws, const uintA& arguments) {
  fvws.clear();
	uint i;
  FOR1D(f_prim, i) {
    FuncIA fvws_local;
    generateAllPossibleFunctionInstances(fvws_local, f_prim(i), arguments);
    fvws.setAppend(fvws_local);
  }
  FOR1D(f_derived, i) {
    FuncIA fvws_local;
    generateAllPossibleFunctionInstances(fvws_local, f_derived(i), arguments);
    fvws.setAppend(fvws_local);
  }
}



void TL::LogicEngine::usedValues(const TL::Function& f, const TL::State& s, arr& values) {
    values.clear();
    uint i;
    if (f.category == TL_PRIMITIVE) {
        FOR1D(s.fv_prim, i) {
            if (s.fv_prim(i)->f->id == f.id) {
                values.setAppend(s.fv_prim(i)->value); 
            }
        }
    }
    else {
        FOR1D(s.fv_derived, i) {
            if (s.fv_derived(i)->f->id == f.id) {
                values.setAppend(s.fv_derived(i)->value); 
            }
        }
    }
}


void TL::LogicEngine::calcFreeVars(const TL::ConjunctionPredicate& scp, uintA& freeVars_pos, uintA& freeVars_neg) {
    uint all_i = 0;
    uint j, k;
    FOR1D(scp.basePreds, j) {
        for(k=0; k<scp.basePreds(j)->d; k++) {
            // if free var
            if (scp.basePreds_mapVars2conjunction(all_i) >= scp.d) {
                if (scp.basePreds_positive(j))
                    freeVars_pos.setAppend(scp.basePreds_mapVars2conjunction(all_i));
                else
                    freeVars_neg.setAppend(scp.basePreds_mapVars2conjunction(all_i));
            }
            all_i++;
        }
    }
}









void TL::LogicEngine::filterPurelyAbstract(const PredIA& allPreds, PredIA& filteredPreds) {
  filteredPreds.clear();
  uint i;
  FOR1D(allPreds, i) {
    if (isAbstract(allPreds(i)))
      filteredPreds.append(allPreds(i));
  }
}



void TL::LogicEngine::negate(const PredIA& predTs, PredIA& predTs_negated) {
  predTs_negated.clear();
  uint i;
  TL::PredicateInstance* pt;
  FOR1D(predTs, i) {
    pt = getPI(predTs(i)->pred, !predTs(i)->positive, predTs(i)->args);
    predTs_negated.append(pt);
  }
}


int TL::LogicEngine::findPattern(const PredIA& actions, uint minRepeats) {
  uint length, repeat, pos;
  int max_repeat_length = 0;
  for (length=1; length<=actions.N / minRepeats; length++) {
    bool successful_repeat = true;
    for (repeat=1; repeat<minRepeats && successful_repeat; repeat++) {
      for (pos=0; pos<length && successful_repeat; pos++) {
        if (actions(repeat * length + pos) != actions(pos))
          successful_repeat = false;
      }
    }
    if (successful_repeat)
      max_repeat_length = length;
  }
  return max_repeat_length;
}




bool TL::LogicEngine::containsLiteral(const PredIA& p, const TL::PredicateInstance& literal) {
  uint i;
  FOR1D(p, i) {
    if (literal == *p(i))
      return true;
  }
  return false;
}




/*
	Example: clear(X) <-- not on(Y,X)
	Then, we need to ground X before we start cover, but _not_ Y, since X is bound
	while Y is not. In other words, the meaning is: for a fixed X, for all Y it holds
	that they are not on X.
	This method returns X.
*/
void TL::LogicEngine::calcUnconstrainedNegatedArguments(TL::ConjunctionPredicate* cp, uintA& vars) {
  uint DEBUG = 0;
  if (DEBUG>0) cout<<"calcUnconstrainedNegatedArguments [START]"<<endl;
  if (DEBUG>0) {cp->writeNice(cout); cout<<endl;}
  uint i, j;
  // Start by taking all vars.
  FOR1D(cp->basePreds_mapVars2conjunction, i)
            if (vars.findValue(cp->basePreds_mapVars2conjunction(i)) < 0)
            vars.append(cp->basePreds_mapVars2conjunction(i));
  if (DEBUG>0) PRINT(vars)
  j = 0;
  // Then, remove all vars which are constrained by a positive literal.
  FOR1D(cp->basePreds, i) {
    if (cp->basePreds_positive(i)) {
      for(;j<cp->basePreds(i)->d;j++)
                vars.removeValueSafe(cp->basePreds_mapVars2conjunction(j++));
    }
    else {
      j+=cp->basePreds(i)->d;
    }
  }
  // Finally, remove all vars which appear as arguments in the p_derived predicate.
  for (i=vars.N; i>0; i--) {
    if (vars(i-1) >= cp->d)
      vars.remove(i-1);
  }
    
  if (DEBUG>0) PRINT(vars)
  if (DEBUG>0) cout<<"calcUnconstrainedNegatedArguments [END]"<<endl;
}












void TL::LogicEngine::createAllPossibleSubstitutions(const uintA& vars, TL::SubstitutionSet& subs) {
  uint DEBUG = 0;
  if (DEBUG>0) cout<<"createAllPossibleSubstitutions [START]"<<endl;
  if (DEBUG>0) {PRINT(vars) PRINT(constants)}
  TL::Substitution* sub;
  uint j;
  uintA assignment(vars.N);
  assignment.setUni(0);
  int id = vars.N-1;
  // beim change ganz nach hinten rutschen und wieder alle zuruecksetzen
  while (true) {
    if (DEBUG>0) PRINT(assignment)
    sub = new TL::Substitution;
    FOR1D(vars, j)
      sub->addSubs(vars(j), constants(assignment(j)));
    if (DEBUG>1) {sub->writeNice(cout); cout<<endl;}
    subs.append(sub);
    
    while (id>=0 && assignment(id) >= constants.N-1) {
      id--;
    }
    if (id<0)
      break;
    
    assignment(id)++;
    while (++id < (int) assignment.N) {
      assignment(id) = 0;
    }
    id=assignment.N-1;
  }
  CHECK(subs.num() == pow(constants.N, vars.N), "Oh no, we forgot some subs!")
  if (DEBUG>0) cout<<"createAllPossibleSubstitutions [END]"<<endl;
}






// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------
//    C H E C K   G R O U N D   T R U T H
// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------



bool TL::LogicEngine::holds_positive(const PredIA& positive_groundedPredTs, TL::PredicateInstance* positive_pt) {
  uint DEBUG = 0;
  if (DEBUG>0) cout << "holds_positive [START]" << endl;
  if (DEBUG>0) {
    cout<<"Checking truth of: "; positive_pt->writeNice(cout); cout<<endl;
    cout<<"In knowledge-base: "; writeNice(positive_groundedPredTs);cout<<endl;
  }
  
  uint i;
  CHECK(positive_pt->positive, "Predicate is negative!");
  CHECK(!containsNegativePredicateInstances(positive_groundedPredTs), "KB findValue negative predicates!");
    
  bool literalFound = false;
  FOR1D(positive_groundedPredTs, i) {
    if (*positive_groundedPredTs(i) == *positive_pt) {
      literalFound = true;
      break;
    }
  }
  
  if (DEBUG>0) cout<<"  ---> "<<literalFound<<endl;
  if (DEBUG>0) cout << "holds_positive [END]" << endl;
  
  return literalFound;
}

bool TL::LogicEngine::holds(const PredIA& positive_groundedPredTs, TL::PredicateInstance* pt) {
  uint DEBUG = 0;
  if (DEBUG>0) cout << "holds [START]" << endl;
//     CHECK(!findValueNegativePredicateInstances(positive_groundedPredTs), "PredIA is only allowed to contain positives.")
  if (DEBUG>0) {
    cout<<"Checking truth of: "; pt->writeNice(cout); cout<<endl;
  }
  bool isTrue;
  if (pt->positive)
    isTrue = holds_positive(positive_groundedPredTs, pt);
  else {
    TL::PredicateInstance pt_pos = *pt;
    pt_pos.positive = true;
    isTrue = !holds_positive(positive_groundedPredTs, &pt_pos);
  }
  if (DEBUG>0) cout<<" --> "<<isTrue<<endl;
  if (DEBUG>0) cout << "holds [END]" << endl;
  return isTrue;
}


bool TL::LogicEngine::holds(const FuncVA& fvs, TL::ComparisonPredicateInstance* pt) {
    uint DEBUG = 0;
    if (pt->hasConstantBound()) {
        uint i;
        FOR1D(fvs, i) {
            if (fvs(i)->f == pt->f  &&  fvs(i)->args == pt->args)
                return pt->compare(fvs(i)->value);
        }
        return false;
    }
    else {
        uint i;
        uintA args_left;
        for(i=0; i<pt->args.d0/2; i++) {
            args_left.append(pt->args(i));
        }
        uintA args_right;
        for(i=pt->args.d0/2; i<pt->args.d0; i++) {
            args_right.append(pt->args(i));
        }
        if (DEBUG>0) {
          cout<<"holds of ";pt->writeNice(cout);cout<<" in : ";writeNice(fvs);cout<<endl;
          PRINT(args_left);
          PRINT(args_right);
        }
        bool leftFound = false, rightFound = false;
        double leftValue=0., rightValue=0.;
        FOR1D(fvs, i) {
            if (fvs(i)->f == pt->f) {
                if (fvs(i)->args == args_left) {
                    leftValue = fvs(i)->value;
                    leftFound = true;
                }
                if (fvs(i)->args == args_right) {
                    rightValue = fvs(i)->value;
                    rightFound = true;
                }
            }
            if (leftFound && rightFound) {
                bool isTrue = pt->compare(leftValue, rightValue);
                if (DEBUG>0) {PRINT(leftValue) PRINT(rightValue) PRINT(isTrue)}
                return isTrue;
            }
        }
        if (DEBUG>0) {if(leftFound){PRINT(leftValue)} if(rightFound){PRINT(rightValue)} cout<<"0"<<endl;}
        return false;
    }
}



bool TL::LogicEngine::holds(const TL::State& s, TL::PredicateInstance* pt) {
	uint DEBUG = 0;
	if (DEBUG>0) cout << "holds [START]" << endl;
	if (DEBUG>0) {
		cout<<"Checking truth of: "; pt->writeNice(cout); cout<<endl;
		cout<<"In state: "; s.writeNice(cout); cout<<endl;
	}	
	bool isTrue;
  if (pt->pred->category == TL_DERIVED) {
    isTrue = holds(s.pi_derived, pt);
  }
  else {
    if (pt->pred->type == TL_PRED_SIMPLE) {
      isTrue = holds(s.pi_prim, pt);
    }
    else if (pt->pred->type == TL_PRED_COMPARISON) {
      TL::ComparisonPredicateInstance* cpt = dynamic_cast<TL::ComparisonPredicateInstance*>(pt);
      CHECK(cpt!=NULL, "Invalid comparison predicate tuple")
      if (cpt->f->category == TL_PRIMITIVE)
        isTrue = holds(s.fv_prim, cpt);
      else
        isTrue = holds(s.fv_derived, cpt);
    }
    else
      HALT("A crazy reference predicate indeed.");
  }
	
	if (DEBUG>0) cout<<" --> "<<isTrue<<endl;
	if (DEBUG>0) cout << "holds [END]" << endl;
	
	return isTrue;
}


bool TL::LogicEngine::holds(const TL::State& s, const PredIA& groundedPredTs) {
	uint i;
	FOR1D(groundedPredTs, i) {
		if (!holds(s, groundedPredTs(i)))
      return false;
	}
	return true;
}

bool TL::LogicEngine::holds(const TL::State& s1, const TL::State& s2) {
	return holds(s1, s2.pi_prim);
}













// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------
//    U N I F I C A T I O N   A N D   C O V E R I N G
// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------


bool TL::LogicEngine::unify_basic(uintA& grounded_args, uintA& other_args, TL::Substitution& sub) {
    uint i;
    FOR1D(grounded_args, i) {
        CHECK(isConstant(grounded_args(i)), "So not, my friend, I got some ungrounded grounded_args here!")
    }
    CHECK(grounded_args.N==other_args.N, "wrong slot assignments");
    FOR1D(other_args, i) {
        if (isConstant(other_args(i))) {
            if (other_args(i) != grounded_args(i))
                return false;
        }
        else {
            if (sub.hasSubs(other_args(i))) {
                if (sub.getSubs(other_args(i)) != grounded_args(i))
                    return false;
            }
            else {
                sub.addSubs(other_args(i), grounded_args(i));
            }
        }
    }
    return true;
}


bool TL::LogicEngine::unify(TL::PredicateInstance* grounded_pt, TL::PredicateInstance* other_pt, TL::Substitution& sub) {
	uint DEBUG = 0;
  CHECK(grounded_pt->positive, "We only unify with positives, my friend!");
  if (!isGrounded(grounded_pt)) {
    grounded_pt->writeNice(cerr);
    HALT(" <--- Dis pi guy ain't grounded!");
  }

	if (grounded_pt->pred != other_pt->pred)
		return false;
    if (grounded_pt->positive != other_pt->positive)
        return false;

	if (DEBUG>0) {grounded_pt->writeNice(cout); cout << endl;}
	
    if (grounded_pt->args.N != other_pt->args.N) { 
        cout<<"In ogicEngine::unify(TL::PredicateInstance* grounded_pt, TL::PredicateInstance* other_pt, TL::Substitution& sub"<<endl;
        cout<<"grounded_pt: ";grounded_pt->writeNice(cout);cout<<endl;
        cout<<"other_pt: ";other_pt->writeNice(cout);cout<<endl;
        sub.writeNice(cout);
    }
    
    return unify_basic(grounded_pt->args, other_pt->args, sub);
}


bool TL::LogicEngine::unify(TL::FunctionValue* grounded_fv, TL::ComparisonPredicateInstance* cpt_constant, TL::Substitution& sub) {
    CHECK(cpt_constant->hasConstantBound(), "this unify deals only with constant-bound cpts")
    CHECK(isGrounded(grounded_fv), "Dis functionValue ain't grounded!")
	
    if (grounded_fv->f != cpt_constant->f)
		return false;
    // check whether comparison would fit at all
    if (!cpt_constant->compare(grounded_fv->value))
        return false;
    
    if (grounded_fv->args.N != cpt_constant->args.N) {
        cout<<"LogicEngine::unify(TL::FunctionValue* grounded_fv, TL::ComparisonPredicateInstance* cpt_constant, TL::Substitution& sub)"<<endl;
        cout<<"grounded_fv: ";grounded_fv->writeNice(cout);cout<<endl;
        cout<<"cpt_constant: ";cpt_constant->writeNice(cout);cout<<endl;
        sub.writeNice(cout);
    }
    
	// check whether unification possible
    return unify_basic(grounded_fv->args, cpt_constant->args, sub);
}




bool TL::LogicEngine::unify(TL::FunctionValue* grounded_fv_left, TL::FunctionValue* grounded_fv_right, TL::ComparisonPredicateInstance* cpt_dynamic, TL::Substitution& sub) {
  uint DEBUG = 0;
  CHECK(isGrounded(grounded_fv_left), "Dis functionValue ain't grounded!")
  CHECK(isGrounded(grounded_fv_right), "Dis functionValue ain't grounded!")
  CHECK(!cpt_dynamic->hasConstantBound(), "Dis cpt ain't dynamic-bounded!")
  if (grounded_fv_left->f != cpt_dynamic->f)
    return false;
  if (grounded_fv_right->f != cpt_dynamic->f)
    return false;
  // check whether comparison would fit at all
  if (!cpt_dynamic->compare(grounded_fv_left->value, grounded_fv_right->value))
    return false;
  // check whether unification possible
  uintA args_left, args_right;
  uint i;
  for(i=0;i<cpt_dynamic->args.d0;i++) {
    if (i<cpt_dynamic->args.d0/2)
      args_left.append(cpt_dynamic->args(i));
    else
      args_right.append(cpt_dynamic->args(i));
  }
  if (DEBUG>0) {cout<<"Init sub: ";sub.writeNice(cout);cout<<endl; PRINT(args_left) PRINT(args_right)}
  
  if (grounded_fv_left->args.N != args_left.N) {
    cout<<"LogicEngine::unify(TL::FunctionValue* grounded_fv_left, TL::FunctionValue* grounded_fv_right, TL::ComparisonPredicateInstance* cpt_dynamic, TL::Substitution& sub)"<<endl;
    cout<<"grounded_fv_left: ";grounded_fv_left->writeNice(cout);cout<<endl;
    cout<<"grounded_fv_right: ";grounded_fv_right->writeNice(cout);cout<<endl;
    cout<<"cpt_dynamic: ";cpt_dynamic->writeNice(cout);cout<<endl;
    PRINT(args_left);
    sub.writeNice(cout);
  }
  
  if (unify_basic(grounded_fv_left->args, args_left, sub)) { 
    if (grounded_fv_right->args.N != args_right.N) {
      cout<<"LogicEngine::unify(TL::FunctionValue* grounded_fv_left, TL::FunctionValue* grounded_fv_right, TL::ComparisonPredicateInstance* cpt_dynamic, TL::Substitution& sub)"<<endl;
      cout<<"grounded_fv_right: ";grounded_fv_right->writeNice(cout);cout<<endl;
      cout<<"cpt_dynamic: ";cpt_dynamic->writeNice(cout);cout<<endl;
      PRINT(args_right);
      sub.writeNice(cout);
    }
    if (unify_basic(grounded_fv_right->args, args_right, sub)) {
      return true;
    }
    else
      return false;
  }
  else
    return false;
}



// BASIC COVERAGE
// FOR ALL
bool TL::LogicEngine::cover(const TL::State& s, TL::PredicateInstance* pt, TL::SubstitutionSet& subs, bool freeNegVarsAllQuantified, TL::Substitution* initSub) {
  uint DEBUG = 0;
  if (DEBUG>0) cout<<"cover - basic [START]"<<endl;

  CHECK(subs.num()==0, "Already subs given, Aldaaaaaa!")
  
  if (DEBUG>1) {
    cout << "State: "; s.writeNice(cout); cout<<endl;
    cout << "pt: "; pt->writeNice(cout); cout<<endl;
    PRINT(freeNegVarsAllQuantified)
    cout << "initSub: "; if (initSub==NULL) cout<<" NULL"; else initSub->writeNice(cout); cout<<endl;
  }
    
  // ------------------------------------------------------------
  // Grounded after initSub
  // ASSUMPTION: states contain only positive PredicateInstances
  // determine grounded pred tuple as determined by initSub
  TL::PredicateInstance* init_grounded_pt;
  if (initSub != NULL)
    init_grounded_pt = applyOriginalSub(*initSub, pt);
  else
    init_grounded_pt = pt;
  if (isGrounded(init_grounded_pt)) {
    if (DEBUG>1) cout<<"pt is grounded after initSub"<<endl;
    if (init_grounded_pt->positive) {
      if (holds(s, init_grounded_pt)) {
        TL::Substitution* s = new TL::Substitution;
        if (initSub != NULL) *s = *initSub;
        subs.append(s);
      }
    }
    else {
      TL::PredicateInstance* init_grounded_pt_pos;
      init_grounded_pt_pos = getPIneg(init_grounded_pt);
      if (!holds(s, init_grounded_pt_pos)) {
        TL::Substitution* s = new TL::Substitution;
        if (initSub != NULL) *s = *initSub;
        subs.append(s);
      }
    }
    if (DEBUG>1) cout<<"covered? " << (subs.num()>0) << endl;
    if (DEBUG>0) cout<<"cover - basic [END]"<<endl;
    if (subs.num() > 0)
        return true;
    else
        return false;
  }
    
      
    
  // ------------------------------------------------------------
  // Non-grounded after initSub
  uint i;
  // ------------------------------------------------------------
  // Positive
  if (pt->positive) {
    TL::Substitution* sub1;
    bool literalTrue;
    if (pt->pred->category == TL_DERIVED) {
      CHECK(s.derivedDerived, "calculate derived predicates first")
      FOR1D(s.pi_derived, i) {
        sub1 = new TL::Substitution;
        if (initSub != NULL) *sub1 = *initSub;
        literalTrue = unify(s.pi_derived(i), pt, *sub1);
        if (literalTrue) subs.append(sub1);
        else delete sub1;
      }
    }
    else if (pt->pred->type == TL_PRED_SIMPLE) {
      FOR1D(s.pi_prim, i) {
        sub1 = new TL::Substitution;
        if (initSub != NULL) *sub1 = *initSub;
        literalTrue = unify(s.pi_prim(i), pt, *sub1);
        if (literalTrue) subs.append(sub1);
        else delete sub1;
      }
    }
    else if (pt->pred->type == TL_PRED_COMPARISON) {
      TL::ComparisonPredicateInstance* cpt = static_cast<TL::ComparisonPredicateInstance*>(pt);
      CHECK(cpt!=NULL, "Maldefined comparison predicate")
      if (cpt->hasConstantBound()) {
        if (cpt->f->category == TL_PRIMITIVE) {
          FOR1D(s.fv_prim, i) {
            sub1 = new TL::Substitution;
            if (initSub != NULL) *sub1 = *initSub;
            literalTrue = unify(s.fv_prim(i), cpt, *sub1);
            if (literalTrue) subs.append(sub1);
            else delete sub1;
          }
        }
        else {
          FOR1D(s.fv_derived, i) {
            sub1 = new TL::Substitution;
            if (initSub != NULL) *sub1 = *initSub;
            literalTrue = unify(s.fv_derived(i), cpt, *sub1);
            if (literalTrue) subs.append(sub1);
            else delete sub1;
          }
        }
      }
      // dynamic CPT
      else {
        if (DEBUG>1) cout<<"Coverage of dynamic cpt [START]"<<endl;
        if (DEBUG>1) {cout<<"Checking coverage of  ";pt->writeNice(cout);cout<<"  with init sub  ("; initSub->writeNice(cout);cout<<")  in  ";writeNice(s.fv_prim);writeNice(s.fv_derived);cout<<"."<<endl;}
        uint l, r;
        uintA args_left, args_right;
        for(l=0;l<cpt->args.d0;l++) {
          if (l<cpt->args.d0/2)
            args_left.append(cpt->args(l));
          else
            args_right.append(cpt->args(l));
        }
        if (DEBUG>0) {PRINT(args_left) PRINT(args_right)}
        FuncVA allValues;
        allValues.append(s.fv_prim);
        allValues.append(s.fv_derived);
        // checking left side
        FOR1D(allValues, l) {
          if (allValues(l)->f != cpt->f)
            continue;
          TL::Substitution sub_left;
          sub_left = *initSub;
            
          if (allValues(l)->args.N != args_left.N) {
            cout<<"LogicEngine::cover(TL::State* s, TL::PredicateInstance* pt, TL::SubstitutionSet& subs, bool freeNegVarsAllQuantified, TL::Substitution* initSub)"<<endl;
            cout<<"allValues(l): ";allValues(l)->writeNice(cout);cout<<endl;
            cout<<"cpt: ";cpt->writeNice(cout);cout<<endl;
            sub_left.writeNice(cout);
          }
            
          if (unify_basic(allValues(l)->args, args_left, sub_left)) {
            // checking right side
            FOR1D(allValues, r) {
              if (allValues(r)->f != cpt->f)
                  continue;
              TL::Substitution sub_right;
              sub_right = sub_left;
                  
              if (allValues(r)->args.N != args_right.N) {
                  cout<<"LogicEngine::cover(TL::State* s, TL::PredicateInstance* pt, TL::SubstitutionSet& subs, bool freeNegVarsAllQuantified, TL::Substitution* initSub)"<<endl;
                  cout<<"allValues(r): ";allValues(r)->writeNice(cout);cout<<endl;
                  cout<<"cpt: ";cpt->writeNice(cout);cout<<endl;
                  sub_right.writeNice(cout);
              }
                  
              if (unify_basic(allValues(r)->args, args_right, sub_right)) {
                if (cpt->compare(allValues(l)->value, allValues(r)->value)) {
                  sub1 = new TL::Substitution;
                  *sub1 = sub_right;
                  subs.append(sub1);
                  if (DEBUG>1) {cout<<"Possible substitution: ";subs.last()->writeNice(cout);cout<<"   since left: ";allValues(l)->writeNice(cout);cout<<"  and right: ";allValues(r)->writeNice(cout);cout<<endl;}
                }
                else {
                  if (DEBUG>1) {
                    cout<<"Subs "; sub_right.writeNice(cout); cout<<" failing due to comparison."<<endl;
                  }
                }
              }
            }
          }
//                     // TODO more efficiency cuts (if slot assignments fail already here...)
//                     FOR1D(s->fValues, r) {
//                         if (s->fValues(r)->f != cpt->f)  // efficiency cut
//                             continue;
//                         sub1 = new TL::Substitution;
//                         if (initSub != NULL) *sub1 = *initSub;
//                         literalTrue = unify(s->fValues(l), s->fValues(r), cpt, *sub1);
//                         if (literalTrue) subs.append(sub1);
//                         else delete sub1;
//                         if (DEBUG>0) {cout<<"left: ";s->fValues(l)->writeNice(cout);cout<<"  right: ";s->fValues(r)->writeNice(cout);cout<<"   --> "<<literalTrue;if(literalTrue){cout<<" New substitution: ";subs.last()->writeNice(cout);}cout<<endl;}
//                     }
        }
        if (DEBUG>1) {
          FOR1D_(subs, r) {
            subs.elem(r)->writeNice(cout); cout<<endl;
          }
        }
        if (DEBUG>1) cout<<"Coverage of dynamic cpt [END]"<<endl;
      }
    }
    else {
      HALT("A crazy reference predicate indeed.")
    }
  }
  // ------------------------------------------------------------
  // Negative
  else {
    // ------------------------------------------------------------
    // All-quantified
    //
    // Algorithm checks that the positive version cannot be unified with state.
    if (freeNegVarsAllQuantified) {
      bool literalTrue = false;
      uint i;
      TL::Substitution* sub1;
      TL::PredicateInstance* pt_pos;
      pt_pos = getPIneg(pt);
      bool heldOnce = false;
      if (pt->pred->type == TL_PRED_SIMPLE) {
        FOR1D(s.pi_prim, i) {
          sub1 = new TL::Substitution;
          if (initSub != NULL) *sub1 = *initSub;
          literalTrue = unify(s.pi_prim(i), pt_pos, *sub1);
          delete sub1;
          if (literalTrue) heldOnce = true;
        }
      }
      else if (pt->pred->type == TL_PRED_CONJUNCTION) {
        CHECK(s.derivedDerived, "derive p_derived predicates first")
        FOR1D(s.pi_derived, i) {
          sub1 = new TL::Substitution;
          if (initSub != NULL) *sub1 = *initSub;
          literalTrue = unify(s.pi_derived(i), pt_pos, *sub1);
          delete sub1;
          if (literalTrue) heldOnce = true;
        }
      }
      else if (pt->pred->type == TL_PRED_COMPARISON) {
        HALT("Comparison Predicate Tuples should never be used in negated form (so far)");
        NIY
      }
      // If positive predicate tuple never held, the negated version is true.
      // We add the init substitution or an empty if the first does not exist.
      if (!heldOnce) {
          TL::Substitution* sub2 = new TL::Substitution;
          if (initSub != NULL)
              *sub2 = *initSub;
          subs.append(sub2);
      }
    }
    // ------------------------------------------------------------
    // Existential-quantified
    // 
    // Algorithm tries all possible substitutions.
    else {
      // determine grounded pred tuple as determined by initSub
      TL::PredicateInstance* init_grounded_pt;
      init_grounded_pt = applyOriginalSub(*initSub, pt);
      // determine free variables
      uintA freeVars;
      FOR1D(init_grounded_pt->args, i) {
        if (!isConstant(init_grounded_pt->args(i)))
          freeVars.setAppend(init_grounded_pt->args(i));
      }
      // create all possible substitutions
      TL::SubstitutionSet cand_subs;
      createAllPossibleSubstitutions(freeVars, cand_subs);
      // check which lead to coverage by creating positive tuples and unifying them with the given state
      TL::PredicateInstance* grounded_pt;
      TL::PredicateInstance* grounded_pt_pos;
      FOR1D_(cand_subs, i) {
        grounded_pt = applyOriginalSub(*cand_subs.elem(i), init_grounded_pt);
        grounded_pt_pos = getPIneg(grounded_pt);
        // If positive version of grounded literal does not hold,
        // then its negation _does_ hold.
        if (!holds(s, grounded_pt_pos)) {
          subs.append(TL::Substitution::combine(*initSub, *cand_subs.elem(i)));
        }
      }
    }
  }
    

  if (DEBUG>1) {
    if (subs.num()==0) cout << "Not covered." << endl;
    else {
      cout <<"Covered by the following substitutions:"<<endl;
      FOR1D_(subs, i) {
        subs.elem(i)->writeNice(cout); cout<<endl;
      }
    }
  }
  if (DEBUG>0) cout<<"cover - basic [END]"<<endl;
  return subs.num() > 0;
}








bool TL::LogicEngine::cover(const TL::State& s, const PredIA& predIs, TL::SubstitutionSet& subs, bool freeNegVarsAllQuantified, TL::Substitution* initSub) {
  int DEBUG = 0;
// DEBUGGING ONLY IN CASE OF CPTS
//     uint q;
//     FOR1D(predTs, q) {
//         if (predTs(q)->pred->type == TL_PRED_COMPARISON) {
//             TL::ComparisonPredicate* cp = dynamic_cast<TL::ComparisonPredicate*>(predTs(q)->pred);
//             if (!cp->constantBound) {
//                 DEBUG = 3;
//                 break;
//             }
//         }
//     }
    
    
  if (DEBUG>0) cout<<"cover [START]"<<endl;
  if (DEBUG > 0) {
    cout << "pts: "; writeNice(predIs); cout << endl;
    cout << "State: ";
    s.writeNice(cout);
    cout << endl;
  }
  
//   orderNegativesUngroundedLast(predTs);
  CHECK(negativesUngroundedLast(predIs), "Positive predicate tuples need to be first!");
  CHECK(subs.num()==0, "Already subs given, Aldaaaaaa!");

  TL::Substitution* initSub_copy = new TL::Substitution;
  if (initSub != NULL)
    *initSub_copy = *initSub;
  subs.append(initSub_copy);

  if (DEBUG > 0) {
    cout << "initSub: ";
    if (initSub != NULL) {initSub->writeNice(cout); cout<<endl;}
    else {cout<<"-"<<endl;}
  }

  uint i, j, k;
  FOR1D(predIs, i) {
    TL::SubstitutionSet all_next_subs;
    if (DEBUG > 0) {
      cout << "Inspecting: ";
      predIs(i)->writeNice(cout);
      cout << endl;
    }
    FOR1D_(subs, j) {
      TL::SubstitutionSet next_subs;
      if (cover(s, predIs(i), next_subs, freeNegVarsAllQuantified, subs.elem(j))) {
        FOR1D_(next_subs, k) {
            all_next_subs.append(next_subs.elem(k));
        }
      }
    }
        
    subs = all_next_subs;
    // if no more subs found = if no more coverage
    if (all_next_subs.num() == 0) {
      break;
    }
    if (DEBUG > 2) {
      PRINT(subs.num())
      FOR1D_(subs, j) {
        subs.elem(j)->writeNice(cout); cout << endl;
      }
    }
  }
  
  if (DEBUG>0) {
    cout << "Covered: " << (subs.num() > 0) << endl;
  }
  if (DEBUG>0) cout<<"cover [END]"<<endl;
  return subs.num() > 0;
}





// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------
//    STATE UNIFICATION
// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------


void TL::LogicEngine::calcDifferences(PredIA& pi_diff_1to2, FuncVA& fv_diff_1to2, PredIA& pi_diff_2to1, FuncVA& fv_diff_2to1, const TL::State state1, const TL::State state2) {
  uint DEBUG = 0;
  if (DEBUG>0) {cout<<"calcDifferences [START]"<<endl;}
  
  if (DEBUG>0) {
    cout<<"State 1:"<<endl;  state1.writeNice();  cout<<endl;
    cout<<"State 2:"<<endl;  state2.writeNice();  cout<<endl;
  }
  
  pi_diff_1to2.clear();
  fv_diff_1to2.clear();
  pi_diff_2to1.clear();
  fv_diff_2to1.clear();
  
  uint i;
  FOR1D(state1.pi_prim, i) {
    if (state2.pi_prim.findValue(state1.pi_prim(i)) < 0)
      pi_diff_1to2.append(state1.pi_prim(i));
  }
  FOR1D(state2.pi_prim, i) {
    if (state1.pi_prim.findValue(state2.pi_prim(i)) < 0)
      pi_diff_2to1.append(state2.pi_prim(i));
  }
  
  FOR1D(state1.fv_prim, i) {
    if (state2.fv_prim.findValue(state1.fv_prim(i)) < 0)
      fv_diff_1to2.append(state1.fv_prim(i));
  }
  FOR1D(state2.fv_prim, i) {
    if (state1.fv_prim.findValue(state2.fv_prim(i)) < 0)
      fv_diff_2to1.append(state2.fv_prim(i));
  }
  
  if (DEBUG>0) {
    cout<<"pi_diff_1to2:    "; TL::writeNice(pi_diff_1to2); cout<<endl;
    cout<<"pi_diff_2to1:    "; TL::writeNice(pi_diff_2to1); cout<<endl;
    cout<<"fv_diff_1to2:    "; TL::writeNice(fv_diff_1to2); cout<<endl;
    cout<<"fv_diff_2to1:    "; TL::writeNice(fv_diff_2to1); cout<<endl;
  }
  
  if (DEBUG>0) {cout<<"calcDifferences [END]"<<endl;}
}



uint TL::LogicEngine::unifyAsMuchAsPossible(SubstitutionSet& subs, const TL::State& state1, const TL::State& state2, TL::Substitution* initSub) {
  uint DEBUG = 0;
  if (DEBUG>0) {cout<<"unifyAsMuchAsPossible [START]"<<endl;}
  
  if (DEBUG>0) {
    cout << "State 1:   "; state1.writeNice(cout, false, true);  cout<<endl;
    cout << "State 2:   "; state2.writeNice(cout, false, true);  cout<<endl;
    cout<<"initSub:  "<<flush; if (initSub == NULL) cout<<"NULL"<<endl;  else initSub->writeNice();  cout<<endl;
  }
  
  // (1) Calc Differences (with initSub)
//   TL::State* state1_initSubbed = initSub->apply(state1);
//   makeOriginal(*state1_initSubbed);
  PredIA pi_diff_1to2, pi_diff_2to1;
  FuncVA fv_diff_1to2, fv_diff_2to1;
  calcDifferences(pi_diff_1to2, fv_diff_1to2, pi_diff_2to1, fv_diff_2to1, state1, state2);
  if (DEBUG>0) {
//     cout << "State 1 (with init-sub):  "; state1_initSubbed->writeNice(cout, false, true);  cout<<endl;
    cout<<"Differences:"<<endl;
    cout<<"pi_diff_1to2:    "; TL::writeNice(pi_diff_1to2); cout<<endl;
    cout<<"fv_diff_1to2:    "; TL::writeNice(fv_diff_1to2); cout<<endl;
    cout<<"pi_diff_2to1:    "; TL::writeNice(pi_diff_2to1); cout<<endl;
    cout<<"fv_diff_2to1:    "; TL::writeNice(fv_diff_2to1); cout<<endl;
  }
//   delete state1_initSubbed;
  
  
  uint i, k, l;
  
  
  // (2) Determine substitutions leading from state1 to state2
  uintA in_ids, out_ids;
  SubstitutionSet potential_subs;
  if (initSub == NULL)
    potential_subs.append(new TL::Substitution);
  else {
    potential_subs.append(initSub);
    initSub->getIns(in_ids);
    initSub->getIns(out_ids);
  }
  
  if (DEBUG>0) {PRINT(in_ids);}
  
  // (2.1) LOOK AT UNARIES
  // PredIA
  FOR1D(pi_diff_1to2, i) {
    if (pi_diff_1to2(i)->pred->d == 1   &&  in_ids.findValue(pi_diff_1to2(i)->args(0)) < 0) {
      uintA possibleSubstitutes;
      FOR1D(pi_diff_2to1, k) {
        if (pi_diff_1to2(i)->pred == pi_diff_2to1(k)->pred    // same predicate
//                &&    substituted_ids.findValue(pi_diff_2to1(k)->args(0)) < 0      // substituting-id not already used
               ){
          possibleSubstitutes.append(pi_diff_2to1(k)->args(0));
        }
      }
      FOR1D(possibleSubstitutes, k) {
        FOR1D_(potential_subs, l) {
          potential_subs.elem(l)->addSubs(pi_diff_1to2(i)->args(0), possibleSubstitutes(k));
          in_ids.setAppend(pi_diff_1to2(i)->args(0));
        }
      }
    }
  }
  // FVs
  FOR1D(fv_diff_1to2, i) {
    if (fv_diff_1to2(i)->f->d == 1   &&  in_ids.findValue(fv_diff_1to2(i)->args(0)) < 0) {
      uintA possibleSubstitutes;
      FOR1D(fv_diff_2to1, k) {
        if (fv_diff_1to2(i)->f == fv_diff_2to1(k)->f    // same function
//                &&   substituted_ids.findValue(fv_diff_2to1(k)->args(0)) < 0  // substituting-id not already used
        ) {
          possibleSubstitutes.append(fv_diff_2to1(k)->args(0));
        }
      }
      FOR1D(possibleSubstitutes, k) {
        FOR1D_(potential_subs, l) {
          potential_subs.elem(l)->addSubs(fv_diff_1to2(i)->args(0), possibleSubstitutes(k));
          in_ids.setAppend(fv_diff_1to2(i)->args(0));
        }
      }
    }
  }
  
  // (2.2) LOOK AT BINARIES
  FOR1D(pi_diff_1to2, i) {
    if (pi_diff_1to2(i)->pred->d == 2) {
      bool first_arg_covered = in_ids.findValue(pi_diff_1to2(i)->args(0)) >= 0;
      bool second_arg_covered = in_ids.findValue(pi_diff_1to2(i)->args(1)) >= 0;
      if (DEBUG>2) {pi_diff_1to2(i)->writeNice();  cout<<"  "<<first_arg_covered<<"  "<<second_arg_covered<<endl;}
      if (!first_arg_covered) {
        uintA possibleSubstitutes;
        FOR1D(pi_diff_2to1, k) {
          if (pi_diff_1to2(i)->pred == pi_diff_2to1(k)->pred     // same predicate
//                 &&   substituted_ids.findValue(pi_diff_2to1(k)->args(0)) < 0         // substituting-id not already used
                &&   pi_diff_1to2(i)->args(0) != pi_diff_2to1(k)->args(0)) {  // different arguments --> true substitution
            possibleSubstitutes.append(pi_diff_2to1(k)->args(0));
          }
        }
        if (DEBUG>2) {cout<<"1:  "<<possibleSubstitutes<<endl;}
        FOR1D(possibleSubstitutes, k) {
          FOR1D_(potential_subs, l) {
            potential_subs.elem(l)->addSubs(pi_diff_1to2(i)->args(0), possibleSubstitutes(k));
            in_ids.setAppend(pi_diff_1to2(i)->args(0));
          }
        }
      }
      if (!second_arg_covered) {
        uintA possibleSubstitutes;
        FOR1D(pi_diff_2to1, k) {
          if (pi_diff_1to2(i)->pred == pi_diff_2to1(k)->pred     // same predicate
//                 &&   substituted_ids.findValue(pi_diff_2to1(k)->args(1)) < 0         // substituting-id not already used
                &&   pi_diff_1to2(i)->args(1) != pi_diff_2to1(k)->args(1)) {  // different arguments --> true substitution
            possibleSubstitutes.append(pi_diff_2to1(k)->args(1));
          }
        }
        if (DEBUG>2) {cout<<"2:  "<<possibleSubstitutes<<endl;}
        FOR1D(possibleSubstitutes, k) {
          FOR1D_(potential_subs, l) {
            potential_subs.elem(l)->addSubs(pi_diff_1to2(i)->args(1), possibleSubstitutes(k));
            in_ids.setAppend(pi_diff_1to2(i)->args(1));
          }
        }
      }
    }
  }
  
  if (DEBUG>0) {
    PRINT(in_ids);
  }
  
  
  // (3) RESUBSTITUTE ALL VARS which are not covered yet; e.g. falls nur 71-->70 bisher, fuehere dann auch 70-->71 ein
  FOR1D_(potential_subs, i) {
    if (DEBUG>0) {cout<<"Completing potential substitution: ";  potential_subs.elem(i)->writeNice();  cout<<endl;}
//     CHECK(potential_subs.elem(i)->mapsToDistinct(), "should map to distinct");
    if (!potential_subs.elem(i)->mapsToDistinct())
      continue;
    
    uintA ins, outs;
    potential_subs.elem(i)->getIns(ins);
    potential_subs.elem(i)->getOuts(outs);
    uintA unsubstituted_outs;
    FOR1D(outs, k) {
      int idx = ins.findValue(outs(k));
      if (idx < 0) {
        unsubstituted_outs.append(outs(k));
      }
    }
    
    if (DEBUG>0) {PRINT(unsubstituted_outs);}
    while (unsubstituted_outs.N > 0) {
      potential_subs.elem(i)->getIns(ins);
      potential_subs.elem(i)->getOuts(outs);
      uintA free_ins;  // for backward substitution
      FOR1D(ins, k) {
        int idx = outs.findValue(ins(k));
        if (idx < 0) {
          free_ins.append(ins(k));
        }
      }
      CHECK(free_ins.N <= unsubstituted_outs.N, "");
      if (DEBUG>0) {PRINT(free_ins);}
      FOR1D_DOWN(free_ins, k) {
        potential_subs.elem(i)->addSubs(unsubstituted_outs(k), free_ins(k));
        unsubstituted_outs.remove(k);
      }
    }
    if (DEBUG>0) {
      cout<<"Final potential substitution: ";  potential_subs.elem(i)->writeNice();  cout<<endl;
    }
  }
  
  
  // (4) TRY OUT POTENTIAL SUBS
  uint minDiff = 10000; // over all subsitutions
  FOR1D_(potential_subs, i) {
    // "potential_state2 should be state2
    TL::State* potential_state2 = potential_subs.elem(i)->apply(state1);
    makeOriginal(*potential_state2);
    if (DEBUG>0) {
      cout<<"Potential sub #" << i << ":"<<endl;
      potential_subs.elem(i)->writeNice(cout);  cout<<endl;
      cout<<"Potential state 2:"<<endl; potential_state2->writeNice(cout, false, true); cout<<endl;
    }
    if (*potential_state2 == state2) {
      subs.append(potential_subs.elem(i));
      if (DEBUG>0) {cout<<"FITS!"<<endl;}
    }
    
    PredIA local__pi_diff_1to2, local__pi_diff_2to1;
    FuncVA local__fv_diff_1to2, local__fv_diff_2to1;
    calcDifferences(local__pi_diff_1to2, local__fv_diff_1to2, local__pi_diff_2to1, local__fv_diff_2to1, *potential_state2, state2);
    uint diff = local__pi_diff_1to2.N  +  local__fv_diff_1to2.N  +  local__pi_diff_2to1.N  +  local__fv_diff_2to1.N;
    
    if (DEBUG>0) {
      cout<<"local__pi_diff_1to2:    "; TL::writeNice(local__pi_diff_1to2); cout<<endl;
      cout<<"local__fv_diff_1to2:    "; TL::writeNice(local__fv_diff_1to2); cout<<endl;
      cout<<"local__pi_diff_2to1:    "; TL::writeNice(local__pi_diff_2to1); cout<<endl;
      cout<<"local__fv_diff_2to1:    "; TL::writeNice(local__fv_diff_2to1); cout<<endl;
      PRINT(diff);
    }
    
    minDiff = TL_MIN(diff,  minDiff);
  }
  
  
  if (DEBUG>0) {
    cout<<"Final subs:"<<endl;
    FOR1D_(subs, i) {
      subs.elem(i)->writeNice(cout);  cout<<endl;
    }
    PRINT(minDiff);
  }
  
  if (DEBUG>0) {cout<<"unifyAsMuchAsPossible [END]"<<endl;}
  
  return minDiff;
}


bool TL::LogicEngine::unify(SubstitutionSet& subs, const TL::State& state1, const TL::State& state2, TL::Substitution* initSub) {
  uint minDiff = unifyAsMuchAsPossible(subs, state1, state2, initSub);
  return minDiff == 0;
}


bool TL::LogicEngine::unifiable(const TL::State& state1, const TL::State& state2) {
  SubstitutionSet subs;
  return unify(subs, state1, state2);
}


uint TL::LogicEngine::unifyAsMuchAsPossible(SubstitutionSet& subs,
                                const TL::State& state1, const TL::PredicateInstance& action1,
                                const TL::State& state2, const TL::PredicateInstance& action2) {
  TL::Substitution* action_sub = new TL::Substitution; // don't delete
  CHECK(action1.pred == action2.pred, "Try to unify nicht-zusammenpassende predicates:  " << action2.pred->id << " vs " << action1.pred);
  uint i;
  FOR1D(action1.args, i) {
    action_sub->addSubs(action1.args(i), action2.args(i));
  }
  return unifyAsMuchAsPossible(subs, state1, state2, action_sub);
}



// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------
//    
// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------


// abstracting
void TL::LogicEngine::createInverseSubstitution(const TL::PredicateInstance& predT, TL::Substitution& sub) {
    CHECK(sub.empty(), "Sub already findValue stuff, krassinger, wa?")
	uint i;
	FOR1D(predT.args, i) {
		if (isConstant(predT.args(i)) && !sub.hasSubs(predT.args(i))) {
			sub.addSubs2Variable(predT.args(i));
		}
	}
}


// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------
//    G R O U N D I N G S
// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------

bool TL::LogicEngine::isGrounded(TL::PredicateInstance* predT) {
	uint i;
	FOR1D(predT->args, i) {
		if (!isConstant(predT->args(i)))
			return false;
	}
	return true;
}

bool TL::LogicEngine::isGrounded(TL::FunctionValue* funcV) {
	uint i;
	FOR1D(funcV->args, i) {
		if (!isConstant(funcV->args(i)))
			return false;
	}
	return true;
}




bool TL::LogicEngine::isAbstract(TL::PredicateInstance* predT) {
	uint i;
	FOR1D(predT->args, i)
		if (isConstant(predT->args(i)))
			return false;
	return true;
}









// derives all ConjunctionPredicates that hold in this state (at the current moment)
bool TL::LogicEngine::derivePredicateInstances_conjunction(TL::ConjunctionPredicate& p, TL::State& s) {
  uint DEBUG = 0;
  if (DEBUG>0) {cout<<"derivePredicateInstances_conjunction [START]"<<endl;}
//     double t_start = MT::cpuTime();
  if (DEBUG>1) {cout<<"ConjunctionPredicate: ";p.writeNice(cout);cout<<endl;cout<<"State: ";s.writeNice(cout);cout<<endl;}
  uint i, j, k;
  // (1) create base predicate tuples = BPT
  PredIA basePTs;
  k=0;
  FOR1D(p.basePreds, i) {
    uintA args1;
    for (j=0; j<p.basePreds(i)->d; j++) {
      args1.append(p.basePreds_mapVars2conjunction(k++));
    }
    TL::PredicateInstance* pt = getPI(p.basePreds(i), p.basePreds_positive(i), args1);
    basePTs.append(pt);
  }
  order(basePTs);
  if (DEBUG>1) {cout<<"Abstract base predicate tuples "; writeNice(basePTs); cout<<endl;}
  // (2) check whether BPT hold
  bool newFound=false;
  // Free Vars EXISTENTIAL
  if (!p.freeVarsAllQuantified) {
    // simply call cover
    TL::SubstitutionSet subs;
    if (cover(s, basePTs, subs, false)) {
      FOR1D_(subs, j) {
        if (p.d == 2  &&  !subs.elem(j)->mapsToDistinct())
          continue;
        TL::PredicateInstance* cpt_cand = generateConjunctionPredicateInstance(&p, subs.elem(j));
        s.pi_derived.setAppend(cpt_cand);
        if (DEBUG>1) {cout<<"p_derived "; s.pi_derived.last()->writeNice(cout); cout<<endl;}
        newFound = true;
      }
    }
  }
  // Free Vars ALL
  else {
    // We must treat positive vars with special care, since in cover(.) positives are always ex quantified.
    // calc free vars in base predicates
    uintA freeVars_pos, freeVars_neg;
    calcFreeVars(p, freeVars_pos, freeVars_neg);
    // (1) Create possible argument-combos for ConjunctionPredicate.
    MT::Array< uintA > argument_lists;
    // only use constants which are provided in the state
    uintA state_constants;
    getConstants(s, state_constants);
    TL::allPossibleLists(argument_lists, state_constants, p.d, false, true);
    // (2) Test each possible argument-combo.
    FOR1D(argument_lists, i) {
      TL::Substitution s_arguments;
      FOR1D(argument_lists(i), j) {
        s_arguments.addSubs(j, argument_lists(i)(j));
      }
      // (2a) Create possible subs for free positive arguments.
      // --> ALL must hold.
      MT::Array< uintA > argument_freePos_lists;
      // here use indeed all available constants
      TL::allPossibleLists(argument_freePos_lists, state_constants, freeVars_pos.d0, true, true);
      FOR1D(argument_freePos_lists, j) {
        TL::Substitution s_arguments_inclFreePos;
        s_arguments_inclFreePos = s_arguments;
        FOR1D(freeVars_pos, k) {
          s_arguments_inclFreePos.addSubs(freeVars_pos(k), argument_freePos_lists(j)(k));
        }
        TL::SubstitutionSet subs_unneeded;
        if (!cover(s, basePTs, subs_unneeded, true, &s_arguments_inclFreePos))
          break;
      }
      // If all possible subs for freePos covered, then we can use the current s_arguments!
      if (argument_freePos_lists.d0 == j) {
          TL::PredicateInstance* cpt_cand = generateConjunctionPredicateInstance(&p, &s_arguments);
        s.pi_derived.setAppend(cpt_cand);
        if (DEBUG>1) {cout<<"p_derived "; s.pi_derived.last()->writeNice(cout); cout<<endl;}
        newFound = true;
      }
    }
  }
//     double t_finish = MT::cpuTime();
//     cout<<"derivePredicateInstances_conjunction time = "<<(t_finish - t_start)<<endl;
  if (DEBUG>0) {cout<<"derivePredicateInstances_conjunction [END]"<<endl;}
  return newFound;
}


// assumes acyclicity!
// seems standard graph problem i don't know standard solution of
bool TL::LogicEngine::derivePredicateInstances_transClosure(TL::TransClosurePredicate& p, TL::State& s) {
    uint DEBUG = 0;
    if (DEBUG>0) {cout<<"derivePredicateInstances_transClosure [START]"<<endl;}
    if (DEBUG>0) {p.writeNice(); cout<<endl;}
    CHECK(p.d==2, "transitive closure defined only for binary preds")
    uint i;
    // (1) find edges
    std::map< uint, uintA > right;
    std::map< uint, uintA >::iterator iter;
    if (DEBUG>0) {cout<<"Given: ";}
    if (p.basePred->category == TL_PRIMITIVE) {
        if (p.basePred->type != TL_PRED_COMPARISON) {
            FOR1D(s.pi_prim, i) {
                if (s.pi_prim(i)->pred->id == p.basePred->id) {
                    if (DEBUG>0) {s.pi_prim(i)->writeNice(cout);cout<<" ";}
                    right[s.pi_prim(i)->args(0)].setAppend(s.pi_prim(i)->args(1));
                }
            }
        }
        else {
          NIY;
            FOR1D(s.pi_comp, i) {
                // are only created on demand!
                if (s.pi_comp(i)->pred->id == p.basePred->id) {
                    if (DEBUG>0) {s.pi_comp(i)->writeNice(cout);cout<<" ";}
                    right[s.pi_comp(i)->args(0)].setAppend(s.pi_comp(i)->args(1));
                }
            }
        }
    }
    else {
        FOR1D(s.pi_derived, i) {
            if (s.pi_derived(i)->pred->id == p.basePred->id) {
                if (DEBUG>0) {s.pi_derived(i)->writeNice(cout);cout<<" ";}
                right[s.pi_derived(i)->args(0)].setAppend(s.pi_derived(i)->args(1));
            }
        }
    }
    if (DEBUG>0) {cout<<endl;}
    
    if (DEBUG>1) {
        cout<<"Direct right neighbors:"<<endl;
        for (iter = right.begin(); iter != right.end(); iter++) {
            cout<<iter->first<<": "<<iter->second<<endl;
        }
    }
    if (right.empty())
        return false;
    // (2) build connections
    bool extended;
    uint no;
    do {
        extended = false;
        for (iter = right.begin(); iter != right.end(); iter++) {
            no = iter->second.d0;
            uintA newGuys_candidates;
            FOR1D(iter->second, i) {
                newGuys_candidates.setAppend(right[iter->second(i)]);
            }
            iter->second.setAppend(newGuys_candidates);
            if (iter->second.d0 > no)
                extended = true;
        }
    } while (extended);
    if (DEBUG>1) {
        cout<<"All right neighbors:"<<endl;
        for (iter = right.begin(); iter != right.end(); iter++) {
            cout<<iter->first<<": "<<iter->second<<endl;
        }
//         cout<<"All left neighbors:"<<endl;
//         for (iter = left.begin(); iter != left.end(); iter++) {
//             cout<<iter->first<<": "<<iter->second<<endl;
//         }
    }
    // (3) build TCP tuples
    if (DEBUG>0) {cout<<"p_derived: ";}
    for (iter = right.begin(); iter != right.end(); iter++) {
        FOR1D(iter->second, i) {
            uintA args(2);
            args(0)=iter->first;
            args(1)=iter->second(i);
            TL::PredicateInstance* pt = generateTransClosurePredicateInstance(&p, args);
            s.pi_derived.setAppend(pt);
            if (DEBUG>0) {pt->writeNice(cout);cout<<" ";}
        }
    }
    if (DEBUG>0) {cout<<endl;}
    
    if (DEBUG>0) {cout<<"derivePredicateInstances_transClosure [END]"<<endl;}
    return true;
}



bool TL::LogicEngine::derivePredicateInstances_count(TL::CountPredicate& p, TL::State& s) {
    uint DEBUG=0;
    if (DEBUG>0) {cout<<"derivePredicateInstances_count [START]"<<endl;}
    if (DEBUG>0) {p.writeNice(cout);cout<<endl;}
    uint i, j;
    uint counter;
    MT::Array< uintA > possibleArgumentLists;
    TL::allPossibleLists(possibleArgumentLists, constants, p.d, true, true);
    bool heldOnce = false;
    FOR1D(possibleArgumentLists, i) {
        if (DEBUG>1) {PRINT(possibleArgumentLists(i))}
        counter=0;
        uintA tester_args;
        tester_args = p.countedPred_mapVars2derived;
        FOR1D(tester_args, j) {
            if (tester_args(j) < p.d)
                tester_args(j) = possibleArgumentLists(i)(tester_args(j));
        }
        if (p.countedPred->category == TL_PRIMITIVE) {
            if (p.countedPred->type == TL_PRED_SIMPLE) {
                TL::PredicateInstance* tester = getPI(p.countedPred, true, tester_args);
                if (DEBUG>1) {cout<<"Tester: ";tester->writeNice(cout);}
                FOR1D(s.pi_prim, j) {
                    TL::Substitution sub;
                    if (unify(s.pi_prim(j), tester, sub)) {
                        counter++;
                        if (DEBUG>1) {cout<<" 1";}
                    }
                }
            }
            else {
                // comp pts only created on demand...
                CHECK(dynamic_cast<TL::ComparisonPredicate*>(p.countedPred)!=NULL, "cast failed");
                CHECK(((TL::ComparisonPredicate*) p.countedPred)->constantBound, "dynamic bound NIY");
                TL::ComparisonPredicateInstance* tester = getCompPT_constant(p.comparison_f, p.comparison_compType, p.comparison_bound, tester_args);
                if (DEBUG>1) {cout<<"Tester: ";tester->writeNice(cout);}
                if (tester->f->category == TL_PRIMITIVE) {
                    FOR1D(s.fv_prim, j) {
                        if (DEBUG>1) {cout<<" ";s.fv_prim(j)->writeNice(cout);}
                        TL::Substitution sub;
                        if (unify(s.fv_prim(j), tester, sub)) {
                            counter++;
                            if (DEBUG>1) {cout<<" 1";}
                        }
                    }
                }
                else {
                    FOR1D(s.fv_derived, j) {
                        if (DEBUG>1) {cout<<" ";s.fv_derived(j)->writeNice(cout);}
                        TL::Substitution sub;
                        if (unify(s.fv_derived(j), tester, sub)) {
                            counter++;
                            if (DEBUG>1) {cout<<" 1";}
                        }
                    }
                }
            }
        }
        else {
            TL::PredicateInstance* tester = getPI(p.countedPred, true, tester_args);
            if (DEBUG>1) {cout<<"Tester: ";tester->writeNice(cout);}
            FOR1D(s.pi_derived, j) {
                TL::Substitution sub;
                if (unify(s.pi_derived(j), tester, sub)) {
                    counter++;
                    if (DEBUG>1) {cout<<" 1";}
                }
            }
        }
        if (DEBUG>1) {cout<<endl;}
        bool holds = TL::compare((int) counter, (int) p.bound, p.compType);
        if (holds) heldOnce=true;
        if (holds) {
            TL::PredicateInstance* pt = generateCountPredicateInstance(&p, possibleArgumentLists(i));
            s.pi_derived.setAppend(pt);
            if (DEBUG>0) {cout<<"Found ";pt->writeNice(cout);cout<<endl;}
        }
    }
    if (DEBUG>0) {cout<<"derivePredicateInstances_count [END]"<<endl;}
    return heldOnce;
}




bool TL::LogicEngine::deriveFunctionValues_count(TL::CountFunction& f, TL::State& s) {
    uint DEBUG = 0;
    if (DEBUG>0) {cout<<"deriveFunctionValues_count [START]"<<endl;}
    if (DEBUG>0) {f.writeNice();cout<<endl;}
    uint i, j;
    MT::Array< uintA > possibleArgumentLists;
    uintA local_constants;
    getConstants(s, local_constants);
    TL::allPossibleLists(possibleArgumentLists, local_constants, f.d, true, true);
    uint counter;
    FOR1D(possibleArgumentLists, i) {
        if (DEBUG>1) {PRINT(possibleArgumentLists(i))}
        counter=0;
        uintA tester_args;
        tester_args = f.countedPred_mapVars2derived;
        FOR1D(tester_args, j) {
            if (tester_args(j) < f.d)
                tester_args(j) = possibleArgumentLists(i)(tester_args(j));
        }
        TL::PredicateInstance* tester = getPI(f.countedPred, true, tester_args);
        if (f.countedPred->category == TL_PRIMITIVE) {
          if (f.countedPred->type == TL_PRED_COMPARISON) {
            HALT("not implemented comparison predicates");
          }
          FOR1D(s.pi_prim, j) {
            TL::Substitution sub;
            if (unify(s.pi_prim(j), tester, sub)) {
              counter++;
            }
          }
        }
        else {
            FOR1D(s.pi_derived, j) {
                TL::Substitution sub;
                if (unify(s.pi_derived(j), tester, sub)) {
                    counter++;
                }
            }
        }
        // function value bauen
        TL::FunctionValue* fv = generateFunctionValue(&f, possibleArgumentLists(i), counter);
        if (DEBUG>0) {cout<<" ";fv->writeNice(cout);}
        s.fv_derived.setAppend(fv);
    }
    if (DEBUG>0) {cout<<"deriveFunctionValues_count [END]"<<endl;}
    return true; // always holds
}


bool TL::LogicEngine::deriveFunctionValues_avg(TL::AverageFunction& f, TL::State& s) {
  uint DEBUG = 0;
  if (DEBUG>0) {cout<<"deriveFunctionValues_avg [START]"<<endl;}
  CHECK(f.d==0, "only implemented for zero-ary avg functions");
  uint i, k;
  MT::Array< uintA > combos;
  uintA local_constants;
  getConstants(s, local_constants);
  TL::allPossibleLists(combos, local_constants, f.f_base->d, true, true);
  double avg = 0.0;
  FOR1D(combos, i) {
    if (DEBUG>1) {PRINT(combos(i))}
    if (f.f_base->category == TL_PRIMITIVE) {
      FOR1D(s.fv_prim, k) {
        if (s.fv_prim(k)->f == f.f_base  
            &&  s.fv_prim(k)->args == combos(i)) {
          avg += s.fv_prim(k)->value;
          break;
        }
      }
    }
    else {
      FOR1D(s.fv_derived, k) {
        if (s.fv_derived(k)->f == f.f_base  
            &&  s.fv_derived(k)->args == combos(i)) {
          avg += s.fv_derived(k)->value;
          break;
        }
      }
    }
  }
  avg /= combos.N;
  // function value bauen
  uintA empty;
  TL::FunctionValue* fv = generateFunctionValue(&f, empty, avg);
  s.fv_derived.setAppend(fv);
  if (DEBUG>0) {cout<<" ";fv->writeNice(cout);}
  if (DEBUG>0) {cout<<"deriveFunctionValues_avg [END]"<<endl;}
  return true; // always holds
}


bool TL::LogicEngine::deriveFunctionValues_sum(TL::SumFunction& f, TL::State& s) {
  uint DEBUG = 0;
  if (DEBUG>0) {cout<<"deriveFunctionValues_sum [START]"<<endl;}
  CHECK(f.d==0, "only implemented for zero-ary avg functions");
  uint i, k;
  MT::Array< uintA > combos;
  uintA local_constants;
  getConstants(s, local_constants);
  TL::allPossibleLists(combos, local_constants, f.f_base->d, true, true);
  double sum = 0.0;
  FOR1D(combos, i) {
    if (DEBUG>1) {PRINT(combos(i))}
    if (f.f_base->category == TL_PRIMITIVE) {
      FOR1D(s.fv_prim, k) {
        if (s.fv_prim(k)->f == f.f_base  
            &&  s.fv_prim(k)->args == combos(i)) {
          sum += s.fv_prim(k)->value;
          break;
        }
      }
    }
    else {
      FOR1D(s.fv_derived, k) {
        if (s.fv_derived(k)->f == f.f_base  
            &&  s.fv_derived(k)->args == combos(i)) {
          sum += s.fv_derived(k)->value;
          break;
        }
      }
    }
  }
  // function value bauen
  uintA empty;
  TL::FunctionValue* fv = generateFunctionValue(&f, empty, sum);
  s.fv_derived.setAppend(fv);
  if (DEBUG>0) {cout<<" ";fv->writeNice(cout);}
  if (DEBUG>0) {cout<<"deriveFunctionValues_avg [END]"<<endl;}
  return true; // always holds
}



bool TL::LogicEngine::deriveFunctionValues_max(TL::MaxFunction& f, TL::State& s) {
  uint DEBUG = 0;
  if (DEBUG>0) {cout<<"deriveFunctionValues_max [START]"<<endl;}
  CHECK(f.d==0, "only implemented for zero-ary max functions");
  uint i, k;
  MT::Array< uintA > combos;
  uintA local_constants;
  getConstants(s, local_constants);
  TL::allPossibleLists(combos, local_constants, f.f_base->d, true, true);
  arr values;
  FOR1D(combos, i) {
    if (DEBUG>1) {PRINT(combos(i))}
    if (f.f_base->category == TL_PRIMITIVE) {
      FOR1D(s.fv_prim, k) {
        if (s.fv_prim(k)->f == f.f_base  
            &&  s.fv_prim(k)->args == combos(i)) {
          values.append(s.fv_prim(k)->value);
          break;
        }
      }
    }
    else {
      FOR1D(s.fv_derived, k) {
        if (s.fv_derived(k)->f == f.f_base  
            &&  s.fv_derived(k)->args == combos(i)) {
          values.append(s.fv_derived(k)->value);
          break;
        }
      }
    }
  }
  double maxVal = values.max();
  // function value bauen
  uintA empty;
  TL::FunctionValue* fv = generateFunctionValue(&f, empty, maxVal);
  s.fv_derived.setAppend(fv);
  if (DEBUG>0) {cout<<" ";fv->writeNice(cout);}
  if (DEBUG>0) {cout<<"deriveFunctionValues_max [END]"<<endl;}
  return true; // always holds
}


bool TL::LogicEngine::deriveFunctionValues_reward(TL::RewardFunction& f, TL::State& s) {
  uint DEBUG = 0;
  if (DEBUG>0) {cout<<"deriveFunctionValues_reward [START]"<<endl;}
  CHECK(f.d==0, "only zero-ary functions");
  double value = 0;
  if (LogicEngine::holds(s, f.grounded_pis))
    value = f.reward_value;
  // function value bauen
  uintA empty;
  empty.resize(f.d);
  TL::FunctionValue* fv = generateFunctionValue(&f, empty, value);
  s.fv_derived.setAppend(fv);
  if (DEBUG>0) {cout<<" ";fv->writeNice(cout);}
  if (DEBUG>0) {cout<<"deriveFunctionValues_reward [END]"<<endl;}
  return true; // always holds
}






// construct p_derived predicates and functions
// assumptions: no self recursion
// negation in base predicates is only allowed for primitive predicates
void TL::LogicEngine::derive(const PredIA& primitiveTuples, const FuncVA& fv_prim, PredIA& derivedTuples, FuncVA& fv_derived) {
    uint DEBUG = 0;
    if (DEBUG > 0) cout << "derive [START]" << endl;
    if (DEBUG > 1) {
        cout << "Primitive Tuples: ";
        writeNice(primitiveTuples);
        cout << endl;
    }
  
    TL::State s;
    s.derivedDerived = true; // needs to be set here, since p_derived predicates that build on other p_derived predicates require that the state be p_derived; in short: setting this state as derivedDerived is safe here since we will derive all the stuff now anyway
    s.pi_prim.append(primitiveTuples);
    s.fv_prim.append(fv_prim);
    
    uintA ordered_ids;
    boolA isPredicate;
    dependencyGraph.getWellDefinedOrder(ordered_ids, isPredicate, true);
    
    uint i, d;
    FOR1D(ordered_ids, i) {
        if (isPredicate(i)) {
            FOR1D(p_derived, d) {
                if (p_derived(d)->id == ordered_ids(i)) break;
            }
            CHECK(d<p_derived.N, "p_derived predicate not found");
            if (p_derived(d)->type == TL_PRED_CONJUNCTION) {
                TL::ConjunctionPredicate* scp = dynamic_cast<TL::ConjunctionPredicate*>(p_derived(d));
                CHECK(scp!=NULL, "cast failed");
                derivePredicateInstances_conjunction(*scp, s);
            }
            else if (p_derived(d)->type == TL_PRED_TRANS_CLOSURE) {
                TL::TransClosurePredicate* tcp = dynamic_cast<TL::TransClosurePredicate*>(p_derived(d));
                CHECK(tcp!=NULL, "cast failed");
                derivePredicateInstances_transClosure(*tcp, s);
            }
            else if (p_derived(d)->type == TL_PRED_COUNT) {
                TL::CountPredicate* p = dynamic_cast<TL::CountPredicate*>(p_derived(d));
                CHECK(p!=NULL, "cast failed");
                derivePredicateInstances_count(*p, s);
            }
            else {
                HALT("Unknown predicate")
            }
        }
        else {
            FOR1D(f_derived, d) {
                if (f_derived(d)->id == ordered_ids(i)) break;
            }
            CHECK(d<f_derived.N, "p_derived function not found; with id="<<ordered_ids(i));
            if (f_derived(d)->type == TL_FUNC_COUNT) {
                TL::CountFunction* f = dynamic_cast<TL::CountFunction*>(f_derived(d));
                CHECK(f!=NULL, "cast failed");
                deriveFunctionValues_count(*f, s);
            }
            else if (f_derived(d)->type == TL_FUNC_AVG) {
              TL::AverageFunction* f = dynamic_cast<TL::AverageFunction*>(f_derived(d));
              CHECK(f!=NULL, "cast failed");
              deriveFunctionValues_avg(*f, s);
            }
            else if (f_derived(d)->type == TL_FUNC_MAX) {
              TL::MaxFunction* f = dynamic_cast<TL::MaxFunction*>(f_derived(d));
              CHECK(f!=NULL, "cast failed");
              deriveFunctionValues_max(*f, s);
            }
            else if (f_derived(d)->type == TL_FUNC_SUM) {
              TL::SumFunction* f = dynamic_cast<TL::SumFunction*>(f_derived(d));
              CHECK(f!=NULL, "cast failed");
              deriveFunctionValues_sum(*f, s);
            }
            else if (f_derived(d)->type == TL_FUNC_REWARD) {
              TL::RewardFunction* f = dynamic_cast<TL::RewardFunction*>(f_derived(d));
              CHECK(f!=NULL, "cast failed");
              deriveFunctionValues_reward(*f, s);
            }
            else {
                HALT("Unknown function")
            }
        }
    }
  
    derivedTuples.clear();
    FOR1D(s.pi_derived, i) {derivedTuples.append(s.pi_derived(i));}
    
    fv_derived.clear();
    FOR1D(s.fv_derived, i) {fv_derived.append(s.fv_derived(i));}
  
    if (DEBUG > 1) {
        cout << "p_derived Tuples: ";
        writeNice(derivedTuples);
        cout << endl;
        cout << "p_derived funcValues: ";
        writeNice(fv_derived);
        cout << fv_derived;
    }
  
    if (DEBUG > 0) cout << "derive [END]" << endl;
}

void TL::LogicEngine::derive(TL::State* s) {
	if (s->derivedDerived)
		return;
	PredIA derivedPTs;
  FuncVA derivedFVs;
  derive(s->pi_prim, s->fv_prim, derivedPTs, derivedFVs);
  s->pi_derived.append(derivedPTs);
  s->fv_derived.append(derivedFVs);
	s->derivedDerived = true;
//     s->writeNice(cout, true);
}

void TL::LogicEngine::dederive(TL::State* s) {
    s->pi_derived.clear();
    s->fv_derived.clear();
    s->derivedDerived = false;
}


void TL::LogicEngine::getAllPrecessors(const TL::Predicate& p, PredA& pre_preds, FuncA& pre_funcs) {
    dependencyGraph.getAllPrecessors(p, pre_preds, pre_funcs);
}

void TL::LogicEngine::getAllPrecessors(const TL::Function& f, PredA& pre_preds, FuncA& pre_funcs) {
    dependencyGraph.getAllPrecessors(f, pre_preds, pre_funcs);
}


// -----------------------------------------------------------------
// -----------------------------------------------------------------
// -----------------------------------------------------------------
//    G E N E R A L   L O G I C  stuff der sonst net einzuordnen ist
// -----------------------------------------------------------------
// -----------------------------------------------------------------
// -----------------------------------------------------------------


bool TL::LogicEngine::nonContradicting(const PredIA& p1, const PredIA& p2) {
	uint i, j, k;
	FOR1D(p1, i) {
		FOR1D(p2, j) {
			if (p1(i)->pred == p2(j)->pred) {
				if (p1(i)->positive == !p2(j)->positive) {
					FOR1D(p1(i)->args, k) {
						if (p1(i)->args(k) != p2(j)->args(k))
							break;
					}
					if (k == p1(i)->args.N)
						return false;
				}
			}
		}
	}
	return true;
}




// -----------------------------------------------------------------
// -----------------------------------------------------------------
// -----------------------------------------------------------------
//    Advanced Logic
// -----------------------------------------------------------------
// -----------------------------------------------------------------
// -----------------------------------------------------------------

void TL::LogicEngine::determineStupidDerivedConcepts(MT::Array< TL::Trial* >& data, PredA& stupidPredicates, FuncA& stupidFunctions, bool deleteFromLogicEngine) {
    uint DEBUG=0;
    if (DEBUG>0) {cout<<"determineStupidDerivedPredicates [START]"<<endl;}
    boolA dp_values;
    uint w, s, c, p;
    // first derive states
    FOR1D(data, w) {
        FOR1D(data(w)->states, s) {
            derive(data(w)->states(s));
        }
    }
    
    bool value, change, localTruth;
    value = change = localTruth = false;
    FOR1D(p_derived, p) {
        MT::Array< uintA > combos;
        TL::allPossibleLists(combos, constants, p_derived(p)->d, true, true);
        change = false;
        if (DEBUG>2) {cout<<"Examining ";p_derived(p)->writeNice(cout);cout<<endl;}
        FOR1D(combos, c) {
            TL::PredicateInstance* pt = getPI(p_derived(p), true, combos(c));
            if (DEBUG>2) {cout<<"Examining pt ";pt->writeNice(cout);cout<<endl;}
            FOR1D(data, w) {
                FOR1D(data(w)->states, s) {
                    localTruth = holds(*data(w)->states(s), pt);
                    if (DEBUG>3) {
                        cout<<"w"<<w<<" s"<<s<<":"; data(w)->states(s)->writeNice(cout);cout<<endl<<"  --> "<<localTruth<<endl;
                    }
                    if (s==0) {
                        value = localTruth;
                    }
                    else {
                        if (value != localTruth) {
                            change = true;
                            break;
                        }
                    }
                }
                if (change)
                    break;
            }
            if (DEBUG>2) {cout<<"  Changed? "<<change<<endl;}
            if (change)
                break;
        }
        if (!change) {
            stupidPredicates.append(p_derived(p));
            if (DEBUG>2) {cout<<p_derived(p)->name<<" is a stupid predicate."<<endl;}
        }
    }
    
    p_derived.memMove = true;
    f_derived.memMove = true;
    if (deleteFromLogicEngine) {
        FOR1D_DOWN(stupidPredicates, p) {
            p_derived.removeValueSafe(stupidPredicates(p));
        }
        // check which functions use some of the stupidPredicates and delete them as well
        FOR1D_DOWN(f_derived, p) {
            PredA p_base;
            FuncA f_base;
            baseConcepts(f_derived(p), p_base, f_base);
            PredA stupidPredicates_mask;
            uint r;
            FOR1D(stupidPredicates, r) {stupidPredicates_mask.append(stupidPredicates(r));}
            if (numberSharedElements(p_base, stupidPredicates_mask) > 0) {
                f_derived.removeValueSafe(f_derived(p));
            }
        }
    }
    
    // reset dependency graph
    PredA ps_all;
    ps_all.append(p_prim);
    FOR1D(p_derived, p) ps_all.append(p_derived(p));
    FuncA fs_all;
    fs_all.append(f_prim);
    fs_all.append(f_derived);
    dependencyGraph.setNodes(ps_all, fs_all);

    if (DEBUG>0) {
        cout<<"Stupid predicates:"<<endl;
        writeNice(stupidPredicates);
        if (deleteFromLogicEngine) {
            cout<<"This leaves the LogicEngine with the following p_derived predicates:"<<endl;
            writeNice(p_derived);
        }
    }
    
    if (DEBUG>0) {cout<<"determineStupidDerivedPredicates [END]"<<endl;}
}




// only works for (i) ConjunctionPredicates that are (ii) positive and (iii) w/o free vars in base preds
// void TL::LogicEngine::integrateSimpleConcepts(PredIA& pts) {
//     uint DEBUG = 3;
//     if (DEBUG>0) {cout<<"integrateSimpleConcepts [START]"<<endl;}
//     if (DEBUG>0) {cout<<"in ["<<pts.N<<"]: ";write(pts);cout<<endl;}
//     // (1) derive
//     // Problem: deriving works unfortunately only for grounded predicate tuples
//     // Workaround: ground predicates, derive, and abstract again
//     uint i, k, t, d;
//     uintA terms;
//     FOR1D(pts, i) {
//         uintA localTerms;
//         calcTerms(*pts(i), localTerms);
//         terms.setAppend(localTerms);
//     }
//     TL::Substitution grounder;
//     FOR1D(terms, i) {
//         grounder.addSubs(terms(i), constants(i));
//     }
//     PredIA pts_grounded; // findValue only positives!
//     FOR1D(pts, i) {
//         if (pts(i)->pred->category == TL_PRIMITIVE
//                 &&  pts(i)->positive) {
//             pts_grounded.append(applyOriginalSub(grounder, pts(i)));
//         }
//     }
//     if (DEBUG>2) {cout<<"-- pts_grounded: ";write(pts_grounded);cout<<endl;}
//     FuncVA fv_prim_empty;
//     PredIA pts_derived_grounded;
//     FuncVA fv_derived_unneeded;
//     derive(pts_grounded, fv_prim_empty, pts_derived_grounded, fv_derived_unneeded);
// //     CHECK(fv_derived_unneeded.N==0, "doch net empty");
//     PredIA pts_derived_grounded_filtered;
//     // filter for positive tuples of ConjunctionPredicates without free variables
//     FOR1D(pts_derived_grounded, i) {
//         if (pts_derived_grounded(i)->positive  &&
//                     pts_derived_grounded(i)->pred->type == TL_PRED_CONJUNCTION) {
//             TL::ConjunctionPredicate* scp = dynamic_cast<TL::ConjunctionPredicate*>(pts_derived_grounded(i)->pred);
//             CHECK(scp!=NULL, "cast failed");
//             uintA freeVars_pos, freeVars_neg;
//             calcFreeVars(*scp, freeVars_pos, freeVars_neg);
//             if (freeVars_pos.N==0 && freeVars_neg.N==0) {
//                 pts_derived_grounded_filtered.append(pts_derived_grounded(i));
//             }
//         }
//     }
//     pts_derived_grounded = pts_derived_grounded_filtered;
//     if (DEBUG>2) {cout<<"-- pts_derived_grounded: ";write(pts_derived_grounded);cout<<endl;}
//     if (DEBUG>2) {cout<<"-- fv_derived_unneeded: ";write(fv_derived_unneeded);cout<<endl;}
//     TL::Substitution abstracter;
//     grounder.getInverse(abstracter);
//     PredIA pts_derived;
//     applyOriginalSub(abstracter, pts_derived_grounded, pts_derived);
//     if (DEBUG>1) {cout<<"==filtered pts_derived== : ";write(pts_derived);cout<<endl;}
//     PredIA all;
//     all.memMove = true;
//     all.setAppend(pts);
//     all.setAppend(pts_derived);
//     if (DEBUG>1) {cout<<"==After deriving ["<<all.N<<"]== : ";write(all);cout<<endl;}
//     // (2) delete sons of each ConjunctionPredicateInstance
//     PredIA removed;
//     FOR1D(pts_derived, i) {
//         if (DEBUG>3) {cout<<"Deleting??";pts_derived(i)->writeNice(cout);cout<<endl;}
// //         if (pts_derived(i)->pred->type == TL_PRED_CONJUNCTION) {
//             TL::ConjunctionPredicate* scp = dynamic_cast<TL::ConjunctionPredicate*>(pts_derived(i)->pred);
//             CHECK(scp!=NULL, "cast failed");
// //             uintA freeVars_pos, freeVars_neg;
// //             calcFreeVars(*scp, freeVars_pos, freeVars_neg);
// //             if (DEBUG>1) {cout<<"-- Checking ";pts_derived(i)->writeNice(cout);cout<<" with free vars "<<freeVars_pos<<" and "<<freeVars_neg<<endl;}
// //             if (freeVars_pos.N==0 && freeVars_neg.N==0) {
//             if (DEBUG>1) {cout<<"--> Removing base preds."<<endl;}
//                 TL::Substitution sub;
//                 for(k=0; k<scp->d; k++) {
//                     sub.addSubs(k, pts_derived(i)->args(k));
//                 }
//                 t=0;
//                 FOR1D(scp->basePreds, k) {
//                     uintA basePred_args(scp->basePreds(k)->d);
//                     FOR1D(basePred_args, d) {
//                         basePred_args(d) = sub.getSubs(scp->basePreds_mapVars2conjunction(t++));
//                     }
//                     removed.append(getPI(scp->basePreds(k), scp->basePreds_positive(k), basePred_args));
//                 }
// //             }
// //         }
//     }
//     pts.clear();
//     FOR1D(all, i) {
//         if (removed.findValue(all(i))<0)
//             pts.setAppend(all(i));
//     }
//     if (DEBUG>0) {cout<<"out ["<<pts.N<<"]: ";write(pts);cout<<endl;}
//     if (DEBUG>0) {cout<<"integrateSimpleConcepts [END]"<<endl;}
// }

void TL::LogicEngine::killBaseConcepts(PredIA& pts) {
    uint DEBUG = 0;
    if (DEBUG>0) {cout<<"killBaseConcepts[START]"<<endl;}
    if (DEBUG>0) {cout<<"in ["<<pts.N<<"]: ";writeNice(pts);cout<<endl;}
    uint i, k, d, t;
    PredIA removed;
    FOR1D(pts, i) {
        if (pts(i)->positive  &&  pts(i)->pred->type == TL_PRED_CONJUNCTION) {
            TL::ConjunctionPredicate* scp = dynamic_cast<TL::ConjunctionPredicate*>(pts(i)->pred);
            CHECK(scp!=NULL, "cast failed");
            uintA freeVars_pos, freeVars_neg;
            calcFreeVars(*scp, freeVars_pos, freeVars_neg);
            if (freeVars_pos.N==0 && freeVars_neg.N==0) {
                if (DEBUG>1) {cout<<"Removing base preds of ";scp->writeNice(cout);cout<<endl;}
                TL::Substitution sub;
                for (k=0; k<scp->d; k++) {
                    sub.addSubs(k, pts(i)->args(k));
                }
                t=0;
                FOR1D(scp->basePreds, k) {
                    uintA basePred_args(scp->basePreds(k)->d);
                    FOR1D(basePred_args, d) {
                        basePred_args(d) = sub.getSubs(scp->basePreds_mapVars2conjunction(t++));
                    }
                    removed.append(getPI(scp->basePreds(k), scp->basePreds_positive(k), basePred_args));
                }
            }
        }
    }
    if (DEBUG>1) {cout<<"Removed guys: ";writeNice(removed);cout<<endl;}
    PredIA filtered;
    FOR1D(pts, i) {
        if (removed.findValue(pts(i))<0)
            filtered.setAppend(pts(i));
    }
    pts = filtered;
    if (DEBUG>0) {cout<<"out ["<<pts.N<<"]: ";writeNice(pts);cout<<endl;}
    if (DEBUG>0) {cout<<"killBaseConcepts [END]"<<endl;}
}







// -----------------------------------------------------------------
// -----------------------------------------------------------------
// -----------------------------------------------------------------
//    T R A N S I T I O N S
// -----------------------------------------------------------------
// -----------------------------------------------------------------
// -----------------------------------------------------------------






void TL::LogicEngine::changes(const TL::State& pre, const TL::State& post, uintA& changedConstants, PredIA& holdOnlyPre, PredIA& holdOnlyPost) {
  uint DEBUG = 0;
  if (DEBUG>0) cout<<"changes [START]" << endl;
  if (DEBUG>1) {
    cout << "Pre: "; pre.writeNice(cout); cout << endl;
    cout << "Post: "; post.writeNice(cout); cout << endl;
  }
  
  // only look at p_prim
  changedConstants.clear();
  holdOnlyPre.clear();
  holdOnlyPost.clear();
  uint i;
  // pre+, post-
  FOR1D(pre.pi_prim, i) {
    if (post.pi_prim.findValue(pre.pi_prim(i)) < 0) {
      holdOnlyPre.append(pre.pi_prim(i));
      changedConstants.setAppend(pre.pi_prim(i)->args);
    }
  }
  // pre-, post+
  FOR1D(post.pi_prim, i) {
    if (pre.pi_prim.findValue(post.pi_prim(i)) < 0) {
      holdOnlyPost.append(post.pi_prim(i));
      changedConstants.setAppend(post.pi_prim(i)->args);
    }
  }

  if (DEBUG>0) {
    cout<<"only in pre: "; writeNice(holdOnlyPre); cout<<endl;
    cout<<"only in post: "; writeNice(holdOnlyPost); cout<<endl;
    PRINT(changedConstants)
  }
  if (DEBUG>0) cout<<"changes [END]" << endl;
}


















// -----------------------------------------------------------------
// -----------------------------------------------------------------
// -----------------------------------------------------------------
//     W R I T E
// -----------------------------------------------------------------
// -----------------------------------------------------------------
// -----------------------------------------------------------------





void TL::LogicEngine::writeLanguage(const char* filename) {
  PredA all_preds;
  all_preds.append(p_prim);
  all_preds.append(p_comp);
  all_preds.append(p_derived);
  
  FuncA all_funcs;
  all_funcs.append(f_prim);
  all_funcs.append(f_derived);
  
  TL::writeLanguage(all_preds, all_funcs, actions, types, filename);
}


TL::State* TL::LogicEngine::readState(const char* filename, bool takeConstants) {
  ifstream in(filename);
  TL::State* s = TL::readState(in, p_prim, p_derived, p_comp, f_prim, f_derived);
  makeOriginal(*s);
  in.close();
  if (takeConstants) {
    getConstants(*s, this->constants);
  }
  derive(s);
  return s;
}





	/****************************************
			MANAGING OBJECT INSTANCES
	***************************************/



TL::PredicateInstance* TL::LogicEngine::getPI(TL::Predicate* pred, bool positive, uintA& args) {
  CHECK(args.N == pred->d, "Invalid predicate instance: "<<pred->name<<" has d="<<pred->d << "   vs. args="<<args);
  return lom->get(pred, positive, args);
}

TL::ComparisonPredicateInstance* TL::LogicEngine::getCompPT_constant(TL::Function* f, uint compType, double bound, uintA& args) {
    return lom->getComp_constant(f, compType, bound, args);
}

TL::ComparisonPredicateInstance* TL::LogicEngine::getCompPT_dynamic(TL::Function* f, uint compType, uintA& args) {
    return lom->getComp_dynamic(f, compType, args);
}

TL::FunctionValue* TL::LogicEngine::getFV(TL::Function* f, uintA& args, double value) {
  CHECK(args.N == f->d, "");
  return lom->get(f, args, value);
}

TL::FunctionInstance* TL::LogicEngine::getFI(TL::Function* f, uintA& args) {
  CHECK(args.N == f->d, "");
  return lom->get(f, args);
}

TL::PredicateInstance* TL::LogicEngine::getPIneg(TL::PredicateInstance* pt) {
    CHECK(pt->pred->type != TL_PRED_COMPARISON, "not defined for p_comp")
    return lom->get(pt->pred, !pt->positive, pt->args);
}

TL::PredicateInstance* TL::LogicEngine::getPIorig(TL::PredicateInstance* pi_copy) {
    if (pi_copy->pred->type == TL_PRED_COMPARISON) {
//       cout<<"Copy: "; pi_copy->writeNice(); cout<<endl;
        TL::ComparisonPredicateInstance* comp_pi_copy = dynamic_cast<TL::ComparisonPredicateInstance*>(pi_copy);
        CHECK(comp_pi_copy!=NULL, "Cast failed")
        TL::ComparisonPredicateInstance* pt;
        if (comp_pi_copy->hasConstantBound()) {
            pt = getCompPT_constant(comp_pi_copy->f, comp_pi_copy->comparisonType, comp_pi_copy->bound, comp_pi_copy->args);
//             if (pt->args.N != pi_copy->args.N) {
//               comp_pi_copy->pred->writeNice();
//               comp_pi_copy->writeNice();
//               PRINT((comp_pi_copy->hasConstantBound()));
//               pt->pred->writeNice();
//               pt->writeNice();
//               PRINT((pt->hasConstantBound()));
//               pt = getCompPT_constant(comp_pi_copy->f, comp_pi_copy->comparisonType, comp_pi_copy->bound, comp_pi_copy->args);
//             }
//             CHECK(pt->args.N == pi_copy->args.N, "something gone wrong");
        }
        else {
            pt = getCompPT_dynamic(comp_pi_copy->f, comp_pi_copy->comparisonType, comp_pi_copy->args);
        }
        if (pt->args.N != pi_copy->args.N) {
          comp_pi_copy->pred->writeNice();
          PRINT((comp_pi_copy->hasConstantBound()));
          pt->pred->writeNice();
          PRINT((pt->hasConstantBound()));
        }
        CHECK(pt->args.N == pi_copy->args.N, "something gone wrong");
        if (pt != pi_copy)
            delete pi_copy;
//         cout<<"Orig: "; pt->writeNice(); cout<<endl;
        return pt;
    }
    else {
        TL::PredicateInstance* pt = getPI(pi_copy->pred, pi_copy->positive, pi_copy->args);
        if (pt != pi_copy)
            delete pi_copy;
        return pt;
    }
}


TL::FunctionValue* TL::LogicEngine::getFVorig(TL::FunctionValue* fv_copy) {
    TL::FunctionValue* fv = getFV(fv_copy->f, fv_copy->args, fv_copy->value);
    if (fv != fv_copy)
        delete fv_copy;
    return fv;
}


TL::FunctionInstance* TL::LogicEngine::getFIorig(TL::FunctionInstance* fi_copy) {
  TL::FunctionInstance* fi = getFI(fi_copy->f, fi_copy->args);
  if (fi != fi_copy)
    delete fi_copy;
  return fi;
}




void TL::LogicEngine::makeOriginal(TL::State& s) {
	uint i;
	
	PredIA p_prim_orig;
	FOR1D(s.pi_prim, i) {
		p_prim_orig.append(getPIorig(s.pi_prim(i)));
	}
	s.pi_prim = p_prim_orig;
	
	PredIA complexPredTs_orig;
	FOR1D(s.pi_derived, i) {
		complexPredTs_orig.append(getPIorig(s.pi_derived(i)));
	}
	s.pi_derived = complexPredTs_orig;
	
	PredIA compPredTs_orig;
	FOR1D(s.pi_comp, i) {
		TL::ComparisonPredicateInstance* cp = dynamic_cast<TL::ComparisonPredicateInstance*>(getPIorig(s.pi_comp(i)));
		CHECK(cp != NULL, "cast failed")
        compPredTs_orig.append(cp);
        if (cp != s.pi_comp(i))
            delete s.pi_comp(i);
	}
  s.pi_comp = compPredTs_orig;
	
	// function values
  FuncVA fv_prim_orig;
  FOR1D(s.fv_prim, i) {
    fv_prim_orig.append(getFVorig(s.fv_prim(i)));
  }
  s.fv_prim = fv_prim_orig;
    
  FuncVA fv_derived_orig;
  FOR1D(s.fv_derived, i) {
    fv_derived_orig.append(getFVorig(s.fv_derived(i)));
  }
  s.fv_derived = fv_derived_orig;
}

void TL::LogicEngine::makeOriginal(TL::Trial& w) {
	PredIA actions_orig;
	uint i;
	FOR1D(w.actions, i) {
//         cout<<w.actions(i)<<" ";w.actions(i)->writeNice(cout);cout<<endl;
		actions_orig.append(getPIorig(w.actions(i)));
//         cout<<actions_orig.last()<<" ";actions_orig.last()->writeNice(cout);cout<<endl;
	}
	w.actions = actions_orig;
	FOR1D(w.states, i) {
		makeOriginal(*w.states(i));
	}
}




TL::PredicateInstance* TL::LogicEngine::applyOriginalSub(TL::Substitution& sub, TL::PredicateInstance* pt) {
    return getPIorig(sub.apply(pt));
}

void TL::LogicEngine::applyOriginalSub(TL::Substitution& sub, const PredIA& unsubPreds, PredIA& subPreds) {
	subPreds.clear();
	uint i;
	FOR1D(unsubPreds, i)
		subPreds.append(applyOriginalSub(sub, unsubPreds(i)));
}



void TL::LogicEngine::initLOM(uintA& objects) {
  uint i, k, v;
  FOR1D(p_prim, i) {
    MT::Array< uintA > sas;
    TL::allPossibleLists(sas, constants, p_prim(i)->d, true, true);
    FOR1D(sas, k) {
      getPI(p_prim(i), true, sas(k));
      getPI(p_prim(i), false, sas(k));
//       getPI(p_prim(i), true, sas(k))->writeNice(cout);cout<<" ";
    }
  }
  FOR1D(p_derived, i) {
    MT::Array< uintA > sas;
    TL::allPossibleLists(sas, constants, p_derived(i)->d, true, true);
    FOR1D(sas, k) {
      getPI(p_derived(i), true, sas(k));
      getPI(p_derived(i), false, sas(k));
//       getPI(p_derived(i), true, sas(k))->writeNice(cout);cout<<" ";
    }
  }
  
  
  FOR1D(f_prim, i) {
    MT::Array< uintA > sas;
    TL::allPossibleLists(sas, constants, f_prim(i)->d, true, true);
    FOR1D(sas, k) {
      getFI(f_prim(i), sas(k));
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
      getFI(f_derived(i), sas(k));
      FOR1D(f_derived(i)->range, v) {
        getFV(f_derived(i), sas(k), f_derived(i)->range(v));
//         getFV(f_derived(i), sas(k), f_prim(i)->range(v))->writeNice(cout);cout<<" ";
      }
    }
  }
}

MT::Array< MT::String > string_variables;
TL::PredicateInstance* TL::LogicEngine::getPI(const char* text) {
  MT::String mt_string(text);
  // Negation
  bool positive = true;
  if (peerNextChar(mt_string) == '-') {
    positive = false;
    skip(mt_string,"-");
  }
  // Predicate
  MT::String name;
  name.read(mt_string, NULL, "(");
//   PRINT(name);
  TL::Predicate* pred = getPredicate(name);
  // Arguments
  uintA args(pred->d);
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
//   PRINT(args);
  return getPI(pred, positive, args);
}


void TL::LogicEngine::getPIs(PredIA& pis, const char* text) {
  pis.clear();
  MT::String mt_string(text);
  MT::String name;
//   while (mt_string != -1) {
  while (MT::skip(mt_string) != -1) {
    MT::String pi_text;
    pi_text.read(mt_string, NULL, ")", 1);
    pis.append(getPI(pi_text));
  }

}







// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------
//    L O G I C   O B J E C T   M A N A G E R
// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------



uint TL::LOGIC_ENGINE__MAX_ID_NUMBER = 100;

// ------------------------------------------------------
// PredicateInstanceList


TL::PredicateInstanceList::PredicateInstanceList() {
  this->pred = NULL;
  this->positive = false;
}

TL::PredicateInstanceList::PredicateInstanceList(TL::Predicate* pred, bool positive) {
  init(pred, positive);
}

TL::PredicateInstanceList::~PredicateInstanceList() {
  uint i;
  FOR_ALL(mem, i) {
    if (mem.p[i] != NULL)
      delete mem.p[i];
  }
}

void TL::PredicateInstanceList::init(TL::Predicate* pred, bool positive) {
  this->pred = pred;
  this->positive = positive;
  
  if (pred->d <= 3) {
    uint dim[pred->d];
    uint i;
    for (i=0; i<pred->d; i++) {
      dim[i] = LOGIC_ENGINE__MAX_ID_NUMBER;
    }
    mem.resize(pred->d, dim);
    mem.setUni(NULL);
  }
}

TL::PredicateInstance* TL::PredicateInstanceList::get(uintA& args) {
  if (pred->d <= 3) {
    PredicateInstance* pi = mem(args);
    if (pi == NULL) {
      switch(pred->type) {
        case TL_PRED_SIMPLE: pi = new PredicateInstance; break;
        case TL_PRED_ACTION: pi = new PredicateInstance; break;
        case TL_PRED_CONJUNCTION: pi = new PredicateInstance; break;
        case TL_PRED_TRANS_CLOSURE: pi = new PredicateInstance; break;
        case TL_PRED_COUNT: pi = new PredicateInstance; break;
        default: pred->writeNice(cerr); cerr<<"  args="<<args<<endl; NIY // needed since newly defined predicates might require their own PredicateInstance definitions
      }
      pi->positive = this->positive;
      pi->pred = this->pred;
      pi->args = args;
      mem(args) = pi;
    }
    return pi;
  }
  else {
    PredicateInstance* pi = NULL;
    uint i;
    FOR_ALL(mem, i) {
      if (mem(i)->args == args) {
        pi = mem(i);
        break;
      }
    }
    if (pi == NULL) {
      switch(pred->type) {
        case TL_PRED_SIMPLE: pi = new PredicateInstance; break;
        case TL_PRED_ACTION: pi = new PredicateInstance; break;
        case TL_PRED_CONJUNCTION: pi = new PredicateInstance; break;
        case TL_PRED_TRANS_CLOSURE: pi = new PredicateInstance; break;
        case TL_PRED_COUNT: pi = new PredicateInstance; break;
        default: NIY // needed since newly defined predicates might require their own PredicateInstance definitions
      }
      pi->positive = this->positive;
      pi->pred = this->pred;
      pi->args = args;
      mem.append(pi);
    }
    return pi;
  }
}



// ------------------------------------------------------
// ComparisonPredicateInstanceList

TL::ComparisonPredicateInstanceList::ComparisonPredicateInstanceList() {
  init (NULL, NULL, 0);
}

TL::ComparisonPredicateInstanceList::ComparisonPredicateInstanceList(ComparisonPredicate* pred, TL::Function* f, uint compType) {
  init(pred, f, compType);
}

TL::ComparisonPredicateInstanceList::~ComparisonPredicateInstanceList() {
  uint i, k;
  FOR_ALL(mem, i) {
    for (k=0; k<mem.p[i].N; k++) {
      delete mem.p[i](k);
    }
  }
}

void TL::ComparisonPredicateInstanceList::init(ComparisonPredicate* pred, TL::Function* f, uint compType) {
  this->pred = pred;
  this->f = f;
  this->compType = compType;
  
  mem.resize(0);
  if (f!=NULL) {
    if (pred->constantBound) {
      uint dim[f->d];
      uint i;
      for (i=0; i<f->d; i++) {
        dim[i] = LOGIC_ENGINE__MAX_ID_NUMBER;
      }
      mem.resize(f->d, dim);
      CompPredIA empty;
      mem.setUni(empty);
    }
    else {
      uint dim[2 * f->d];
      uint i;
      for (i=0; i<2 * f->d; i++) {
        dim[i] = LOGIC_ENGINE__MAX_ID_NUMBER;
      }
      mem.resize(2 * f->d, dim);
      CompPredIA empty;
      mem.setUni(empty);
    }
  }
}


TL::ComparisonPredicateInstance* TL::ComparisonPredicateInstanceList::get_constant(uintA& args, double bound) {
  uint DEBUG = 0;
  if (DEBUG>0) {
    cout<<"LOOKING for  "<< f->name << "   "<< args << "   compType=" << compType << "  bound=" << bound << endl;
  }
  CompPredIA& cis = mem(args);
  if (DEBUG>0) {
    uint k;
    FOR1D(cis, k) {
      cis(k)->writeNice(cout); cout<<endl;
    }
  }
  uint i = 0;
  if (cis.N > 0) {
    int left = 0;
    int right = cis.N-1;
    while (left <= right) {
      i = (int) ((left + right) / 2);
      if (areEqual(cis(i)->bound, bound)) {
        if (DEBUG>0) {cout<<"Found:  "; cis(i)->writeNice(cout);  cout<<endl;}
        return cis(i);
      }
      else if (cis(i)->bound < bound) {
        left = i+1;
      }
      else {
        right = i-1;
      }
    }
  }
  ComparisonPredicateInstance* ci =  new ComparisonPredicateInstance;
  ci->pred = this->pred;
  ci->f = this->f;
  ci->comparisonType = this->compType;
  ci->positive = true;
  ci->bound = bound;
  ci->args = args;
  cis.memMove = true;
  if (cis.N == 0)
    cis.append(ci);
  else if (cis(i)->bound < ci->bound) {
    if (i+1 < cis.N)
      cis.insert(i+1, ci);
    else
      cis.append(ci);
  }
  else
    cis.insert(i, ci);
  if (DEBUG>0) {
    uint k;
    cout << "Updated:"<<endl;
    FOR1D(cis, k) {
      cis(k)->writeNice(cout); cout<<endl;
    }
  }
  return ci;
}

TL::ComparisonPredicateInstance* TL::ComparisonPredicateInstanceList::get_dynamic(uintA& args) {
  uint DEBUG = 0;
  if (DEBUG>0) {cout<<"LOOKING FOR DYNAMIC "<<f->name<< "   " << args << "   " << compType << endl;}
  CompPredIA& cis = mem(args);
  if (cis.N > 0) {
    CHECK(cis.N==1, "demasiado cis");
    if (DEBUG>0) {cout <<"Found "; cis(0)->writeNice(); cout<<endl;}
    return cis(0);
  }
  // create
  ComparisonPredicateInstance* ci =  new ComparisonPredicateInstance();
  ci->pred = this->pred;
  ci->f = this->f;
  ci->comparisonType = this->compType;
  ci->positive = true;
  ci->args = args;
  cis.append(ci);
  if (DEBUG>0) {
    uint k;
    cout << "Updated:"<<endl;
    FOR1D(cis, k) {
      cis(k)->writeNice(cout); cout<<endl;
    }
    cout <<"Returned"; cis(0)->writeNice(); cout<<endl;
  }
  return ci;
}




// ------------------------------------------------------
// FunctionValue

TL::FunctionValueList::FunctionValueList() {
  this->f = NULL;
}

TL::FunctionValueList::FunctionValueList(TL::Function* f) {
  init(f);
}

TL::FunctionValueList::~FunctionValueList() {
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
      dim[i] = LOGIC_ENGINE__MAX_ID_NUMBER;
    }
    mem.resize(f->d, dim);
    FuncVA empty;
    mem.setUni(empty);
  }
}


TL::FunctionValue* TL::FunctionValueList::get(uintA& args, double value) {
  uint DEBUG = 0;
  FuncVA& fvs = mem(args);
  if (DEBUG>0) {
    cout<<"LOOKING for  "<< f->name << "   "<< args << "  " << value << endl;
    uint k;
    FOR1D(fvs, k) {
      fvs(k)->writeNice(cout); cout<<endl;
    }
  }
  uint i = 0;
  if (fvs.N > 0) {
    int left = 0;
    int right = fvs.N-1;
    while (left <= right) {
      i = (uint) ((left + right) / 2);
      if (areEqual(fvs(i)->value, value)) {
        if (DEBUG>0) {cout<<"Found:  "; fvs(i)->writeNice(cout);  cout<<endl;}
        return fvs(i);
      }
      else if (fvs(i)->value < value) {
        left = i+1;
      }
      else {
        right = i-1;
      }
    }
  }
  // create
  FunctionValue* fv =  new FunctionValue;
  fv->f = this->f;
  fv->value= value;
  fv->args = args;
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
  
  if (DEBUG>0) {
    uint k;
    cout << "Updated:"<<endl;
    FOR1D(fvs, k) {
      fvs(k)->writeNice(cout); cout<<endl;
    }
  }
  
  return fv;
}








// ------------------------------------------------------
// FunctionInstance

TL::FunctionInstanceList::FunctionInstanceList() {
  this->f = NULL;
}

TL::FunctionInstanceList::FunctionInstanceList(TL::Function* f) {
  init(f);
}

TL::FunctionInstanceList::~FunctionInstanceList() {
  uint i;
  FOR_ALL(mem, i) {
    if (mem.p[i] != NULL)
      delete mem.p[i];
  }
}

void TL::FunctionInstanceList::init(TL::Function* f) {
  this->f = f;
  
  uint dim[f->d];
  uint i;
  for (i=0; i<f->d; i++) {
    dim[i] = LOGIC_ENGINE__MAX_ID_NUMBER;
  }
  mem.resize(f->d, dim);
  mem.setUni(NULL);
}


TL::FunctionInstance* TL::FunctionInstanceList::get(uintA& args) {
  FunctionInstance* fi = mem(args);
  if (fi == NULL) {
    fi = new TL::FunctionInstance;
    fi->f = this->f;
    fi->args = args;
    mem(args) = fi;
  }
  return fi;
}






// -------------------------------
// LOGIC MANAGER




TL::LogicObjectManager::LogicObjectManager(PredA& predicates, FuncA& functions) {
  p_comp.clear();
  uint i;
  FOR1D(predicates, i) {
    if (predicates(i)->type == TL_PRED_COMPARISON)
      p_comp.append((ComparisonPredicate*) predicates(i));
  }
  setPredicates(predicates);
  setFunctions(functions);
}


#define LOM__DEFAULT_ACTION__ID 144
#define LOM__LISTS_PI__POS_COEFF 100

void TL::LogicObjectManager::setPredicates(PredA& predicates) {
  uint i;
  uint max_id = lists_pi.d0;
  FOR1D(predicates, i) {
    if (predicates(i)->id == TL_DEFAULT_ACTION_PRED__ID)
      continue;
    if (predicates(i)->id > max_id)
      max_id = predicates(i)->id;
  }
  max_id = TL_MAX(LOM__DEFAULT_ACTION__ID, max_id);
  lists_pi.resize(max_id+1 + LOM__LISTS_PI__POS_COEFF);
  
  FOR1D(predicates, i) {
    addPredicate(predicates(i));
  }
}

void TL::LogicObjectManager::addPredicate(Predicate* p) {
  if (p->id == TL_DEFAULT_ACTION_PRED__ID) {
    PredicateInstanceList pi_list_neg(p, false);
    lists_pi(calcID_pred(LOM__DEFAULT_ACTION__ID, 0)) = pi_list_neg;
    PredicateInstanceList pi_list_pos(p, true);
    lists_pi(calcID_pred(LOM__DEFAULT_ACTION__ID, 1)) = pi_list_pos;
  }
  else {
    CHECK(p->id < lists_pi.N, "bad new id="<<p->id<<"  vs  max_id=" << lists_pi.N);
    PredicateInstanceList pi_list_neg(p, false);
    lists_pi(calcID_pred(p->id, 0)) = pi_list_neg;
    PredicateInstanceList pi_list_pos(p, true);
    lists_pi(calcID_pred(p->id, 1)) = pi_list_pos;
  }
}



void TL::LogicObjectManager::setFunctions(FuncA& functions) {
  uint i;
  uint max_id;
  
  // DEALING WITH COMPARISON_PREDICATE_TUPLES
  max_id = 100;
//   max_id = lists_comp.d0;
//   FOR1D(functions, i) {
//     if (functions(i)->id > max_id)
//       max_id = functions(i)->id;
//   }
  
  uint max_compType_index = 0;
  max_compType_index = TL_MAX(max_compType_index, TL_COMPARISON_EQUAL);
  max_compType_index = TL_MAX(max_compType_index, TL_COMPARISON_LESS);
  max_compType_index = TL_MAX(max_compType_index, TL_COMPARISON_LESS_EQUAL);
  max_compType_index = TL_MAX(max_compType_index, TL_COMPARISON_GREATER);
  max_compType_index = TL_MAX(max_compType_index, TL_COMPARISON_GREATER_EQUAL);
  
  lists_comp.resize(max_id+1, max_compType_index+1, 2);
  
  // DEALING WITH FUNCTION INSTANCES AND VALUES
  lists_fv.resize(max_id+1);
  lists_fi.resize(max_id+1);
  
  FOR1D(functions, i) {
    addFunction(functions(i));
  }
}

void TL::LogicObjectManager::addFunction(Function* f) {
  // DEALING WITH COMPARISON_PREDICATE_TUPLES
  uint i;
  FOR1D(p_comp, i) {
    TL::ComparisonPredicate* cp = (TL::ComparisonPredicate*) p_comp(i);
    lists_comp((f->id), TL_COMPARISON_EQUAL, cp->constantBound) = ComparisonPredicateInstanceList(cp, f, TL_COMPARISON_EQUAL);
    lists_comp((f->id), TL_COMPARISON_LESS, cp->constantBound) = ComparisonPredicateInstanceList(cp, f, TL_COMPARISON_LESS);
    lists_comp((f->id), TL_COMPARISON_LESS_EQUAL, cp->constantBound) = ComparisonPredicateInstanceList(cp, f, TL_COMPARISON_LESS_EQUAL);
    lists_comp((f->id), TL_COMPARISON_GREATER, cp->constantBound) = ComparisonPredicateInstanceList(cp, f, TL_COMPARISON_GREATER);
    lists_comp((f->id), TL_COMPARISON_GREATER_EQUAL, cp->constantBound) = ComparisonPredicateInstanceList(cp, f, TL_COMPARISON_GREATER_EQUAL);
  }
  
  // DEALING WITH FUNCTION INSTANCES AND VALUES
  if (lists_fv.N < f->id+1) {
    lists_fv.resize(f->id+1);
    lists_fi.resize(f->id+1);
  }
  lists_fv(f->id).init(f);
  lists_fi(f->id).init(f);
}

TL::LogicObjectManager::~LogicObjectManager() {
}


TL::PredicateInstance* TL::LogicObjectManager::get(TL::Predicate* pred, bool positive, uintA& args) {
  if (pred->id == TL_DEFAULT_ACTION_PRED__ID)
    return lists_pi(calcID_pred(LOM__DEFAULT_ACTION__ID, positive)).get(args);
  else
    return lists_pi(calcID_pred(pred->id, positive)).get(args);
}

uint TL::LogicObjectManager::calcID_pred(uint pred_id, bool positive) {
  uint id = pred_id + positive * 100;
//     cout<<p->name<<": "<<id<<endl;
  return id;
}



TL::ComparisonPredicateInstance* TL::LogicObjectManager::getComp_constant(TL::Function* f, uint compType, double bound, uintA& args) {
  return lists_comp(f->id, compType, 1).get_constant(args, bound);
}

TL::ComparisonPredicateInstance* TL::LogicObjectManager::getComp_dynamic(TL::Function* f, uint compType, uintA& args) {
  return lists_comp(f->id, compType, 0).get_dynamic(args);
}



TL::FunctionValue* TL::LogicObjectManager::get(Function* f, uintA& args, double value) {
  return lists_fv(f->id).get(args, value);
//     uint id = calcID(f);
// 	cout << "For " << pred->name << positive << " using id " << id << endl;
//     return lists_fv[id]->get(args, value);
}

TL::FunctionInstance* TL::LogicObjectManager::get(Function* f, uintA& args) {
  return lists_fi(f->id).get(args);
//   uint id = calcID(f);
// 	cout << "For " << pred->name << positive << " using id " << id << endl;
//   return lists_fvw[id]->get(args);
}








































// ----------------------------------------------------------------------------
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

void TL::ConceptDependencyGraph::setNodes(PredA& predicates, FuncA& functions) {
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
    if (predicates(i)->category == TL_PRIMITIVE) {
      id2pred.append(true);
      pred2graph[predicates(i)->id] = graph_id;
      graph2pred[graph_id] = predicates(i)->id;
      graph_id++;
    }
  }
  FOR1D(functions, i) {
    if (functions(i)->category == TL_PRIMITIVE) {
      id2pred.append(false);
      func2graph[functions(i)->id] = graph_id;
      graph2func[graph_id] = functions(i)->id;
      graph_id++;
    }
  }
  derivedStartID=graph_id;
  // then p_derived
  FOR1D(predicates, i) {
    if (predicates(i)->category == TL_DERIVED) {
      id2pred.append(true);
      pred2graph[predicates(i)->id] = graph_id;
      graph2pred[graph_id] = predicates(i)->id;
      graph_id++;
    }
  }
  FOR1D(functions, i) {
    if (functions(i)->category == TL_DERIVED) {
      id2pred.append(false);
      func2graph[functions(i)->id] = graph_id;
      graph2func[graph_id] = functions(i)->id;
      graph_id++;
    }
  }
  // (2) Edges
  FOR1D(predicates, i) {
    if (predicates(i)->category == TL_DERIVED) {
      // set direct precessors
      if (predicates(i)->type == TL_PRED_CONJUNCTION) {
        TL::ConjunctionPredicate* scp = dynamic_cast<TL::ConjunctionPredicate*>(predicates(i));
        CHECK(scp!=NULL,"cast failed");
        uint p;
        FOR1D(scp->basePreds, p) {
          edges(pred2graph[scp->basePreds(p)->id], pred2graph[scp->id]) = true;
        }
      }
      else if (predicates(i)->type == TL_PRED_TRANS_CLOSURE) {
        TL::TransClosurePredicate* tcp = dynamic_cast<TL::TransClosurePredicate*>(predicates(i));
        CHECK(tcp!=NULL,"cast failed");
        edges(pred2graph[tcp->basePred->id], pred2graph[tcp->id]) = true;
      }
      else if (predicates(i)->type == TL_PRED_COUNT) {
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
    if (functions(i)->category == TL_DERIVED) {
      // set direct precessors
      if (functions(i)->type == TL_FUNC_COUNT) {
        TL::CountFunction* cf = dynamic_cast<TL::CountFunction*>(functions(i));
        CHECK(cf!=NULL,"cast failed");
        edges(pred2graph[cf->countedPred->id], func2graph[cf->id]) = true;
      }
      else if (functions(i)->type == TL_FUNC_AVG) {
        TL::AverageFunction* af = dynamic_cast<TL::AverageFunction*>(functions(i));
        CHECK(af!=NULL,"cast failed");
        edges(func2graph[af->f_base->id], func2graph[af->id]) = true;
      }
      else if (functions(i)->type == TL_FUNC_MAX) {
        TL::MaxFunction* mf = dynamic_cast<TL::MaxFunction*>(functions(i));
        CHECK(mf!=NULL,"cast failed");
        edges(func2graph[mf->f_base->id], func2graph[mf->id]) = true;
      }
      else if (functions(i)->type == TL_FUNC_SUM) {
        TL::SumFunction* sf = dynamic_cast<TL::SumFunction*>(functions(i));
        CHECK(sf!=NULL,"cast failed");
        edges(func2graph[sf->f_base->id], func2graph[sf->id]) = true;
      }
      else if (functions(i)->type == TL_FUNC_REWARD) {
        TL::RewardFunction* grf = dynamic_cast<TL::RewardFunction*>(functions(i));
        CHECK(grf!=NULL,"cast failed");
        uint l;
        FOR1D(grf->grounded_pis, l) {
          edges(pred2graph[grf->grounded_pis(l)->pred->id], func2graph[grf->id]) = true;
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


void TL::ConceptDependencyGraph::getAllPrecessors(const TL::Predicate& p, PredA& ps_pre, FuncA& fs_pre) {
    ps_pre.clear();
    fs_pre.clear();
    uint n;
    PredA ps_pre_direct;
    FuncA fs_pre_direct;
    for(n=0; n<edges.d0; n++) {
        if (edges(n, pred2graph[p.id])) {
            if (id2pred(n)) ps_pre_direct.append(getPredicate(graph2pred[n]));
            else fs_pre_direct.append(getFunction(graph2func[n])); 
        }
    }
    ps_pre.append(ps_pre_direct);
    fs_pre.append(fs_pre_direct);
    FOR1D(ps_pre_direct, n) {
        PredA ps_pre_recursive;
        FuncA fs_pre_recursive;
        getAllPrecessors(*ps_pre_direct(n), ps_pre_recursive, fs_pre_recursive);
        ps_pre.append(ps_pre_recursive);
        fs_pre.append(fs_pre_recursive);
    }
    FOR1D(fs_pre_direct, n) {
        PredA ps_pre_recursive;
        FuncA fs_pre_recursive;
        getAllPrecessors(*fs_pre_direct(n), ps_pre_recursive, fs_pre_recursive);
        ps_pre.append(ps_pre_recursive);
        fs_pre.append(fs_pre_recursive);
    }
}

void TL::ConceptDependencyGraph::getAllPrecessors(const TL::Function& f, PredA& ps_pre, FuncA& fs_pre) {
    ps_pre.clear();
    fs_pre.clear();
    uint n;
    PredA ps_pre_direct;
    FuncA fs_pre_direct;
    for(n=0; n<edges.d0; n++) {
        if (edges(n, func2graph[f.id])) {
            if (id2pred(n)) ps_pre_direct.append(getPredicate(graph2pred[n]));
            else fs_pre_direct.append(getFunction(graph2func[n])); 
        }
    }
    ps_pre.append(ps_pre_direct);
    fs_pre.append(fs_pre_direct);
    FOR1D(ps_pre_direct, n) {
        PredA ps_pre_recursive;
        FuncA fs_pre_recursive;
        getAllPrecessors(*ps_pre_direct(n), ps_pre_recursive, fs_pre_recursive);
        ps_pre.append(ps_pre_recursive);
        fs_pre.append(fs_pre_recursive);
    }
    FOR1D(fs_pre_direct, n) {
        PredA ps_pre_recursive;
        FuncA fs_pre_recursive;
        getAllPrecessors(*fs_pre_direct(n), ps_pre_recursive, fs_pre_recursive);
        ps_pre.append(ps_pre_recursive);
        fs_pre.append(fs_pre_recursive);
    }
}

// int HORST_depth = 0;
void TL::ConceptDependencyGraph::getAllSuccessors(const TL::Predicate& p, PredA& ps_suc, FuncA& fs_suc) {
  ps_suc.clear();
  fs_suc.clear();
  uint n;
  PredA ps_suc_direct;
//   HORST_depth++;
//   cout<<"---- ";PRINT(HORST_depth);
//   cout<<"Getting successors of: ";p.writeNice();cout<<endl;
//   writeNice();
//   PRINT(pred2graph[p.id]);
  FuncA fs_suc_direct;
  for(n=0; n<edges.d0; n++) {
    if (edges(pred2graph[p.id], n)) {
      if (id2pred(n)) ps_suc_direct.append(getPredicate(graph2pred[n]));
      else fs_suc_direct.append(getFunction(graph2func[n])); 
    }
  }
  ps_suc.append(ps_suc_direct);
  fs_suc.append(fs_suc_direct);
//   cout<<"ps_suc: ";LogicEngine::write(ps_suc);if (ps_suc.N == 0) cout<<endl;
//   cout<<"fs_suc: ";LogicEngine::write(fs_suc);if (fs_suc.N == 0) cout<<endl;
  FOR1D(ps_suc_direct, n) {
    PredA ps_suc_recursive;
    FuncA fs_suc_recursive;
    getAllSuccessors(*ps_suc_direct(n), ps_suc_recursive, fs_suc_recursive);
    ps_suc.setAppend(ps_suc_recursive);
    fs_suc.setAppend(fs_suc_recursive);
  }
  FOR1D(fs_suc_direct, n) {
    PredA ps_suc_recursive;
    FuncA fs_suc_recursive;
    getAllSuccessors(*fs_suc_direct(n), ps_suc_recursive, fs_suc_recursive);
    ps_suc.setAppend(ps_suc_recursive);
    fs_suc.setAppend(fs_suc_recursive);
  }
//   HORST_depth--;
}


void TL::ConceptDependencyGraph::getAllSuccessors(const TL::Function& f, PredA& ps_suc, FuncA& fs_suc) {
  ps_suc.clear();
  fs_suc.clear();
  uint n;
  PredA ps_suc_direct;
  FuncA fs_suc_direct;
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
//   cout<<"ps_suc: ";LogicEngine::write(ps_suc);if (ps_suc.N == 0) cout<<endl;
//   cout<<"fs_suc: ";LogicEngine::write(fs_suc);if (fs_suc.N == 0) cout<<endl;
  FOR1D(ps_suc_direct, n) {
    PredA ps_suc_recursive;
    FuncA fs_suc_recursive;
    getAllSuccessors(*ps_suc_direct(n), ps_suc_recursive, fs_suc_recursive);
    ps_suc.setAppend(ps_suc_recursive);
    fs_suc.setAppend(fs_suc_recursive);
  }
  FOR1D(fs_suc_direct, n) {
    PredA ps_suc_recursive;
    FuncA fs_suc_recursive;
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

