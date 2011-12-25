#define MT_IMPLEMENT_TEMPLATES

#include <relational/logicObjectManager.h>
#include <relational/logicReasoning.h>

#include "symbolGrounding.h"






// ------------------------------------------------------------------
// ------------------------------------------------------------------
// ------------------------------------------------------------------
// ------------------------------------------------------------------
// ------------------------------------------------------------------
//
//  SymbolGrounding

relational::SymbolGrounding::SymbolGrounding(MT::String& _name, uint _arity, bool build_derived_predicates) { 
  this->arity = _arity;
  this->name = _name;
  this->pred = TL::logicObjectManager::getPredicate(this->name);
  if (this->pred != NULL)
    return;
  this->pred = new TL::Predicate;
  this->pred->id = TL::logicObjectManager::getLowestFreeConceptID(11);
  this->pred->name = this->name;
  this->pred->d = this->arity;
  PredL new_predicates;
  new_predicates.append(this->pred);
  TL::logicObjectManager::addStatePredicates(new_predicates);
  
  if (build_derived_predicates) {
    if (this->pred->d == 2) {
      MT::String derived_name;
      derived_name << "not_" << this->pred->name << "_related_to_second";
      if (TL::logicObjectManager::getPredicate(derived_name) == NULL) {
        // add 
        // du1(X) = forall Y; not b1(X,Y)
        TL::ConjunctionPredicate* p_NOT_P_RELATED_TO_SECOND = new TL::ConjunctionPredicate;
        p_NOT_P_RELATED_TO_SECOND->d = 1;
        p_NOT_P_RELATED_TO_SECOND->name = derived_name;
        p_NOT_P_RELATED_TO_SECOND->id = TL::logicObjectManager::getLowestFreeConceptID(11);
        p_NOT_P_RELATED_TO_SECOND->basePreds.append(TL::logicObjectManager::getPredicate(MT::String("b1")));
        p_NOT_P_RELATED_TO_SECOND->basePreds_positive.append(false);
        p_NOT_P_RELATED_TO_SECOND->basePreds_mapVars2conjunction.resize(2);
        p_NOT_P_RELATED_TO_SECOND->basePreds_mapVars2conjunction(0) = 0;
        p_NOT_P_RELATED_TO_SECOND->basePreds_mapVars2conjunction(1) = 1;
        p_NOT_P_RELATED_TO_SECOND->freeVarsAllQuantified = true;
    //     p_NOT_P_RELATED_TO_SECOND->writeNice(cout); cout<<endl;

        TL::ConjunctionPredicate* p_NOT_P_RELATED_TO_FIRST = new TL::ConjunctionPredicate;
        p_NOT_P_RELATED_TO_FIRST->d = 1;
        p_NOT_P_RELATED_TO_FIRST->name << "not_" << this->pred->name << "_related_to_first";
        p_NOT_P_RELATED_TO_FIRST->id = TL::logicObjectManager::getLowestFreeConceptID(11) + 1;
        p_NOT_P_RELATED_TO_FIRST->basePreds.append(TL::logicObjectManager::getPredicate(MT::String("b1")));
        p_NOT_P_RELATED_TO_FIRST->basePreds_positive.append(false);
        p_NOT_P_RELATED_TO_FIRST->basePreds_mapVars2conjunction.resize(2);
        p_NOT_P_RELATED_TO_FIRST->basePreds_mapVars2conjunction(0) = 1;
        p_NOT_P_RELATED_TO_FIRST->basePreds_mapVars2conjunction(1) = 0;
        p_NOT_P_RELATED_TO_FIRST->freeVarsAllQuantified = true;
    //     p_NOT_P_RELATED_TO_FIRST->writeNice(cout); cout<<endl;
        
        PredL preds;
        preds.append(p_NOT_P_RELATED_TO_SECOND);
        preds.append(p_NOT_P_RELATED_TO_FIRST);
        TL::logicObjectManager::addStatePredicates(preds);
      }
    }
  }
}




void relational::SymbolGrounding::calculateLiterals(LitL& lits, const uintA& objects_ids, const MT::Array< arr > & objects_data) const {
  uint DEBUG = 0;
  if (DEBUG>0) {cout<<"calculateLiterals [START]"<<endl;}
  if (DEBUG>0) {PRINT(objects_ids);  PRINT(objects_data);}
  lits.clear();
  uint i;
  if (arity == 1) {
    FOR1D(objects_ids, i) {
      if (DEBUG>1) {PRINT(objects_ids(i));  PRINT(objects_data(i));}
      bool does_hold = holds(objects_data(i));
      if (DEBUG>1) {PRINT(does_hold);}
      if (does_hold) {
        uintA args;  args.append(objects_ids(i));
        lits.append(TL::logicObjectManager::getLiteral(this->pred, true, args));
      }
    }
  }
  else if (arity == 2) {
    MT::Array< uintA > lists;
    uintA objs_numbers;
    FOR1D(objects_ids, i) {objs_numbers.append(i);}
    TL::allPossibleLists(lists, objs_numbers, arity, false, true);
    FOR1D(lists, i) {
      if (DEBUG>1) {cout<<"Next:  "<<this->pred->name<<lists(i)<<endl;}
      arr x = objects_data(lists(i)(0)) - objects_data(lists(i)(1));
      if (DEBUG>1) {PRINT(x);}
      bool does_hold = holds(x);
      if (DEBUG>1) {PRINT(does_hold);}
      if (does_hold) {
        uintA args;  args.append(objects_ids(lists(i)(0)));  args.append(objects_ids(lists(i)(1)));
        lits.append(TL::logicObjectManager::getLiteral(this->pred, true, args));
        if (DEBUG>1) {cout<<"--> "<<*lits.last()<<endl;}
      }
    }
  }
  else NIY;
  if (DEBUG>0) {PRINT(lits.N);  PRINT(lits);}
  if (DEBUG>0) {cout<<"calculateLiterals [END]"<<endl;}
}






void relational::SymbolGrounding::calculateLiterals(LitL& lits, const MT::Array<relational::SymbolGrounding*>& sgs,
                       const uintA& objects_ids, const MT::Array< arr > & objects_data) {
  uint DEBUG = 0;
  if (DEBUG>0) {cout<<"calculateLiterals [START]"<<endl;}
  if (DEBUG>0) {PRINT(objects_ids);  PRINT(objects_data);}
  lits.clear();
  uint i;
  FOR1D(sgs, i) {
    CHECK(sgs(i)->pred != NULL, "No predicate object");
    if (DEBUG>0) {cout<<"Predicate "<<sgs(i)->pred->name<<endl;}
    LitL lits_p;
    sgs(i)->calculateLiterals(lits_p, objects_ids, objects_data);
    if (DEBUG>0) {PRINT(lits_p);}
    lits.append(lits_p);
  }
  if (DEBUG>0) {PRINT(lits);}
  if (DEBUG>0) {cout<<"calculateLiterals [END]"<<endl;}
}


void relational::calculateLiterals(MT::Array<SymbolGrounding*>& sgs, FullExperience& e) {
  uint DEBUG = 0;
  if (DEBUG>0) {cout<<"calculateLiterals [START]"<<endl;}
  // Pre-State
  uintA objs;
  uint k;
  for (k=0; k<e.state_continuous_pre.N; k++) {objs.append(buildConstant_nikolayData(k));}
  SymbolGrounding::calculateLiterals(e.experience_symbolic.pre.lits_prim, sgs, objs, e.state_continuous_pre);
  SymbolGrounding::calculateLiterals(e.experience_symbolic.post.lits_prim, sgs, objs, e.state_continuous_post);
  TL::logicReasoning::derive(&e.experience_symbolic.pre);
  TL::logicReasoning::derive(&e.experience_symbolic.post);
  e.experience_symbolic.calcChanges();
  if (DEBUG>2) {PRINT(e.action_target);}
  uintA args = TUP(buildConstant_nikolayData(e.action_target));
  if (e.action_type == 1)
    e.experience_symbolic.action = TL::logicObjectManager::getAtom(TL::logicObjectManager::getPredicate(MT::String("grab")), args);
  else
    e.experience_symbolic.action = TL::logicObjectManager::getAtom(TL::logicObjectManager::getPredicate(MT::String("puton")), args);
  if (DEBUG>0) {e.experience_symbolic.write();}
  if (DEBUG>0) {cout<<"calculateLiterals [END]"<<endl;}
}


void relational::SymbolGrounding::write() const {
  PRINT(name);  PRINT(arity);
}


void relational::read(MT::Array<SymbolGrounding*>& sgs, const char* prefix, SymbolGrounding::GroundingType grounding_type) {
  uint DEBUG = 0;
  if (DEBUG>0) {cout<<"SymbolGrounding::read [START]"<<endl;}
  if (DEBUG>0) {PRINT(prefix);  PRINT(grounding_type);}
  
  MT::Array< MT::String > pn_names;
  pn_names.append(MT::String("u1"));  //pn_names.append(MT::String("u2"));
  pn_names.append(MT::String("b1"));  //pn_names.append(MT::String("b2"));
  
  uint i;
  FOR1D(pn_names, i) {
    SymbolGrounding* sg;
    uint arity;
    if (pn_names(i)(0) == 'b')
      arity = 2;
    else
      arity = 1;
    if (grounding_type == relational::SymbolGrounding::NN) {
      sg = new NN_Grounding(pn_names(i), arity, true);
      MT::String file_w1a, file_w1b, file_w2a, file_w2b;
      file_w1a<<prefix<<pn_names(i)<<"w1a.txt";
      file_w1b<<prefix<<pn_names(i)<<"w1b.txt";
      file_w2a<<prefix<<pn_names(i)<<"w2a.txt";
      file_w2b<<prefix<<pn_names(i)<<"w2b.txt";
      ((NN_Grounding*) sg)->read(file_w1a, file_w1b, file_w2a, file_w2b);
    }
    else if (grounding_type == relational::SymbolGrounding::RBF) {
      sg = new RBF_Grounding(pn_names(i), arity, true);
      MT::String file_w_c, file_w_sigma;
      file_w_c<<prefix<<pn_names(i)<<"w1a.txt";
      file_w_sigma<<prefix<<pn_names(i)<<"w1b.txt";
      ((RBF_Grounding*) sg)->read(file_w_c, file_w_sigma);
    }
    else
      NIY;
    sgs.append(sg);
    if (DEBUG>0) {sg->write();}
  }
  
  if (DEBUG>0) {cout<<"SymbolGrounding::read [END]"<<endl;}
}









// ------------------------------------------------------------------
// ------------------------------------------------------------------
// ------------------------------------------------------------------
// ------------------------------------------------------------------
// ------------------------------------------------------------------
//
// NN_Grounding


relational::NN_Grounding::NN_Grounding(MT::String& name, uint arity, bool build_derived_predicates) : 
SymbolGrounding(name, arity, build_derived_predicates) {
  this->type = NN;
}


bool relational::NN_Grounding::holds(arr& x) const {
  uint DEBUG = 0;
  if (DEBUG>0) {cout<<"holds [START]"<<endl;}
  if (DEBUG>1) {PRINT(x);  write();}
  // apply network
  arr z_argument = ~w1a * x + w1b;
  arr z;
  uint k;
  FOR1D(z_argument, k) {
    z.append((1. / ( 1. + exp( z_argument(k) ) )));
  }
  z.reshape(z.N, 1);
  if (DEBUG>0) {PRINT(z);}
  arr g_argument = w2a * z + w2b;
  if (DEBUG>0) {PRINT(g_argument);}
  double g = (1. / ( 1. + exp( g_argument(0,0)) ));
  double p = g - 0.5;
  if (DEBUG>0) {PRINT(g);  PRINT(p);}
  if (DEBUG>0) {PRINT((p>0.));}
  if (DEBUG>0) {cout<<"holds [END]"<<endl;}
  return p>0.;
}


void relational::NN_Grounding::read(const char* file_w1a, const char* file_w1b, const char* file_w2a, const char* file_w2b) {
  uint DEBUG = 0;
  ifstream in;
  in.open(file_w1a);
  CHECK(in.is_open(), "File "<<file_w1a<<" can't be opened!");
  this->w1a.read(in);
  if (DEBUG>0) {PRINT(this->w1a);}
  in.close();

  in.open(file_w1b);
  CHECK(in.is_open(), "File "<<file_w1b<<" can't be opened!");
  this->w1b.read(in);
  if (DEBUG>0) {PRINT(this->w1b);}
  in.close();
  
  in.open(file_w2a);
  CHECK(in.is_open(), "File "<<file_w2a<<" can't be opened!");
  this->w2a.read(in);
  if (DEBUG>0) {PRINT(this->w2a);}
  in.close();
  
  in.open(file_w2b);
  CHECK(in.is_open(), "File "<<file_w2b<<" can't be opened!");
  this->w2b.read(in);
  if (DEBUG>0) {PRINT(this->w2b);}
  in.close();
}


void relational::NN_Grounding::write() const {
  SymbolGrounding::write();
  PRINT(w1a);  PRINT(w1b);  PRINT(w2a);  PRINT(w2b);
}




// ------------------------------------------------------------------
// ------------------------------------------------------------------
// ------------------------------------------------------------------
// ------------------------------------------------------------------
// ------------------------------------------------------------------
//
//  RBF_Grounding


relational::RBF_Grounding::RBF_Grounding(MT::String& name, uint arity, bool build_derived_predicates) : 
SymbolGrounding(name, arity, build_derived_predicates) {
  this->type = RBF;
}

bool relational::RBF_Grounding::holds(arr& x) const {
  uint DEBUG = 0;
  if (DEBUG>0) {cout<<"holds [START]"<<endl;}
  if (DEBUG>1) {PRINT(x);  write();}
  double p;
  arr x_diff = x - w_c;
  arr w_mean(4);  w_mean(0) = w_sigma(0,0);  w_mean(1) = w_sigma(1,0);  w_mean(2) = w_sigma(2,0);  w_mean(3) = w_sigma(3,0);
  double exponent = 4.;
  uint i;
  FOR1D(w_mean, i) {w_mean(i) = pow(w_mean(i), 4);}
  arr cov_matrix = diag(w_mean);
  arr cov_matrix_inv = inverse(cov_matrix);
  arr g1 = cov_matrix_inv * x_diff;
  arr g2 = ~g1 * x_diff;
  if (DEBUG>1) {PRINT(x_diff);  PRINT(cov_matrix);  PRINT(cov_matrix_inv);  PRINT(g1);  PRINT(g2);}
  double g = exp(-g2(0));
  p = g - 0.5;
  if (DEBUG>0) {PRINT(g);  PRINT(p);}
  if (DEBUG>0) {PRINT((p>0.));}
  if (DEBUG>0) {cout<<"holds [END]"<<endl;}
  return p>0.;
}

void relational::RBF_Grounding::read(const char* file_w_c, const char* file_w_sigma) {
  uint DEBUG = 0;
  ifstream in;
  in.open(file_w_c);
  CHECK(in.is_open(), "File "<<file_w_c<<" can't be opened!");
  this->w_c.read(in);
  if (DEBUG>0) {PRINT(this->w_c);}
  in.close();

  in.open(file_w_sigma);
  CHECK(in.is_open(), "File "<<file_w_sigma<<" can't be opened!");
  this->w_sigma.read(in);
  if (DEBUG>0) {PRINT(this->w_sigma);}
  in.close();
}


void relational::RBF_Grounding::write() const {
  SymbolGrounding::write();  PRINT(w_c);  PRINT(w_sigma);
}



// ------------------------------------------------------------------
// ------------------------------------------------------------------
// ------------------------------------------------------------------
// ------------------------------------------------------------------
// ------------------------------------------------------------------
//
// ors::Graph interface

void relational::getFeatureVector(arr& f, const ors::Graph& C, uint obj) {
  f.clear();
  f.resize(4);
  f(0) = C.bodies(obj)->X.pos.p[0];  // position
  f(1) = C.bodies(obj)->X.pos.p[1];  // position
  f(2) = C.bodies(obj)->X.pos.p[2];  // position
  f(2) = C.bodies(obj)->shapes(0)->size[0];  // size
}


void relational::calculateLiterals(LitL& lits, const MT::Array<relational::SymbolGrounding*>& sgs, ors::Graph* C) {
  uint DEBUG = 0;
  if (DEBUG>0) {cout<<"SymbolGrounding::calculateLiterals [START]"<<endl;}
  
  uintA obj_ids;
  std::stringstream ss;
  uint i;
  for (i=1;;i++) {
    ss.str("");
    ss << "o" << i;
    ors::Body* n = C->getBodyByName(ss.str().c_str());
    if (n==NULL)
      break;
    obj_ids.append(n->index);
  }
  if (DEBUG>0) {PRINT(obj_ids);}
  
  MT::Array< arr > objects_data;
  FOR1D(obj_ids, i) {
    arr f1;
    getFeatureVector(f1, *C, obj_ids(i));
    objects_data.append(f1);
  }
  if (DEBUG>0) {PRINT(objects_data);}
  
  SymbolGrounding::calculateLiterals(lits, sgs, obj_ids, objects_data);
  if (DEBUG>0) {PRINT(lits);}
  if (DEBUG>0) {cout<<"SymbolGrounding::calculateLiterals [END]"<<endl;}
}






// ------------------------------------------------------------------
// ------------------------------------------------------------------
// ------------------------------------------------------------------
// ------------------------------------------------------------------
// ------------------------------------------------------------------
//
//  FullExperience


void relational::FullExperience::write_continuous(ostream& out) {
  out<<"PRE: "<<state_continuous_pre.N<<endl;
  out<<state_continuous_pre<<endl;
  out<<"ACTION:"<<endl;
  out<<"action_type="<<action_type<<endl;
  out<<"action_target="<<action_target<<endl;
  out<<"POST: "<<state_continuous_post.N<<endl;
  out<<state_continuous_post<<endl;
}
  
void relational::FullExperience::write_symbolic(ostream& out) {
  experience_symbolic.write(out);
}
  
void relational::FullExperience::write_symbolic(MT::Array<FullExperience* > exps, ostream& out) {
  uint i;
  FOR1D(exps, i) {
    out<<"["<<i<<"] ("<<(i+1)<<")"<<endl;
    exps(i)->write_symbolic(out);
  }
}

void relational::FullExperience::read_nikolayFormat(MT::Array< FullExperience* >& experiences, const char* file_name) {
  uint DEBUG = 0;
  if (DEBUG>0) {cout<<"read_data [START]"<<endl;}
  ifstream in(file_name);
  CHECK(in.is_open(), "File "<<file_name<<" can't be opened!");
  uint OBJ_NUM = 5;
  while (MT::skip(in) != -1) {
    FullExperience* exp = new FullExperience;
    MT::String line;
    in >> line;
    if (DEBUG>0) PRINT(line);
    uint o;
    for (o=0; o<OBJ_NUM; o++) {
      arr data(4);
      line >> data(0);  line >> data(1);  line >> data(2);  line >> data(3);
      if (DEBUG>0) PRINT(data);
      exp->state_continuous_pre.append(data);
    }
    for (o=0; o<OBJ_NUM; o++) {
      arr data(4);
      line >> data(0);  line >> data(1);  line >> data(2);  line >> data(3);
      if (DEBUG>0) PRINT(data);
      exp->state_continuous_post.append(data);
    }
    double action_type_double, action_target_double;
    line >> action_type_double;
    exp->action_type = (uint) action_type_double;
    line >> action_target_double;
    exp->action_target = (uint) action_target_double - 1;  // -1 since Nikolay starts to count at 1
    line >> exp->reward;
    experiences.append(exp);
    if (DEBUG>0) {experiences.last()->write_continuous(cout);}
  }
  
  if (DEBUG>0) {cout<<"read_data [END]"<<endl;}
}
