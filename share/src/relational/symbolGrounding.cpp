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
  // HACK
//   if (arity == 1) DEBUG = 2;
  if (DEBUG>0) {cout<<"calculateLiterals [START]"<<endl;}
  if (DEBUG>0) {cout<<"***********  "<<this->name<<endl;  PRINT(objects_ids);  PRINT(objects_data);}
  lits.clear();
  uint i;
  if (arity == 1) {
    FOR1D(objects_ids, i) {
      if (DEBUG>1) {cout<<"*** Next:  "<<this->pred->name<<"("<<objects_ids(i)<<")"<<endl;}
      if (DEBUG>1) {PRINT(objects_data(i));}
      bool does_hold = holds(objects_data(i));
      if (DEBUG>1) {PRINT(does_hold);}
      if (does_hold) {
        uintA args;  args.append(objects_ids(i));
        lits.append(TL::logicObjectManager::getLiteral(this->pred, true, args));
        if (DEBUG>1) {cout<<" $$$$$$ --> "<<*lits.last()<<endl;}
      }
    }
  }
  else if (arity == 2) {
    MT::Array< uintA > lists;
    uintA objs_numbers;
    FOR1D(objects_ids, i) {objs_numbers.append(i);}
    TL::allPossibleLists(lists, objs_numbers, arity, false, true);
    FOR1D(lists, i) {
      if (DEBUG>1) {cout<<"*** Next:  "<<this->pred->name<<"("<<objects_ids(lists(i)(0))<<" "<<objects_ids(lists(i)(1))<<")"<<endl;}
      arr x = objects_data(lists(i)(0)) - objects_data(lists(i)(1));
      if (DEBUG>1) {PRINT(x);}
      bool does_hold = holds(x);
      if (DEBUG>1) {PRINT(does_hold);}
      if (does_hold) {
        uintA args;  args.append(objects_ids(lists(i)(0)));  args.append(objects_ids(lists(i)(1)));
        lits.append(TL::logicObjectManager::getLiteral(this->pred, true, args));
        if (DEBUG>1) {cout<<" $$$$$$ --> "<<*lits.last()<<endl;}
      }
    }
  }
  else NIY;
  if (DEBUG>0) {PRINT(lits.N);  PRINT(lits);}
  if (DEBUG>0) {cout<<"calculateLiterals [END]"<<endl;}
}



void relational::SymbolGrounding::calculateLiterals(LitL& lits, const MT::Array<relational::SymbolGrounding*>& sgs, const ContinuousState& cont_state) {
  uint DEBUG = 0;
  if (DEBUG>0) {cout<<"calculateLiterals [START]"<<endl;}
  if (DEBUG>0) {PRINT(cont_state.object_ids);  PRINT(cont_state.data);}
  lits.clear();
  uint i;
  FOR1D(sgs, i) {
    CHECK(sgs(i)->pred != NULL, "No predicate object");
    if (DEBUG>0) {cout<<"Predicate "<<sgs(i)->pred->name<<endl;}
    LitL lits_p;
    sgs(i)->calculateLiterals(lits_p, cont_state.object_ids, cont_state.data);
    if (DEBUG>0) {PRINT(lits_p);}
    lits.append(lits_p);
  }
  if (DEBUG>0) {PRINT(lits);}
  if (DEBUG>0) {cout<<"calculateLiterals [END]"<<endl;}
}


void relational::calculateLiterals(const MT::Array<SymbolGrounding*>& sgs, FullExperience& e) {
  uint DEBUG = 0;
  if (DEBUG>0) {cout<<"calculateLiterals [START]"<<endl;}
  e.experience_symbolic.pre.state_objects = e.state_continuous_pre.object_ids;
  SymbolGrounding::calculateLiterals(e.experience_symbolic.pre.lits_prim, sgs, e.state_continuous_pre);
  TL::logicReasoning::derive(&e.experience_symbolic.pre);
  e.experience_symbolic.post.state_objects = e.state_continuous_post.object_ids;
  SymbolGrounding::calculateLiterals(e.experience_symbolic.post.lits_prim, sgs, e.state_continuous_post);
  TL::logicReasoning::derive(&e.experience_symbolic.post);
  e.experience_symbolic.calcChanges();
  if (DEBUG>1) {PRINT(e.action_args);}
  if (e.action_type == 0)
    e.experience_symbolic.action = TL::logicObjectManager::getAtom(TL::logicObjectManager::getPredicate(MT::String("grab")), e.action_args);
  else if (e.action_type == 1)
    e.experience_symbolic.action = TL::logicObjectManager::getAtom(TL::logicObjectManager::getPredicate(MT::String("puton")), e.action_args);
  else NIY;
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
  if (DEBUG>1) {PRINT((p>0.));}
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
  // HACK
//   if (this->arity == 1) DEBUG = 2;
  if (DEBUG>0) {cout<<"holds [START]"<<endl;}
  if (DEBUG>1) {write();  PRINT(x);}
  double p;
  arr x_diff = x - w_c;
  arr w_sigma_hoch_zwei = pow(w_sigma, 2.0);
  arr cov_matrix = diag(w_sigma_hoch_zwei);
  arr cov_matrix_inv = inverse(cov_matrix);
  arr g1 = cov_matrix_inv * x_diff;
  arr g2 = ~g1 * x_diff;
  if (DEBUG>1) {PRINT(x_diff);  PRINT(cov_matrix);  PRINT(cov_matrix_inv);  PRINT(g1);  PRINT(g2);}
  double g = exp(-g2(0));
  p = g - 0.5;
  if (DEBUG>0) {PRINT(g);  PRINT(p);}
  if (DEBUG>1) {PRINT((p>0.));}
  if (DEBUG>0) {cout<<"holds [END]"<<endl;}
  return p>0.;
}

void relational::RBF_Grounding::read(const char* file_w_c, const char* file_w_sigma) {
  uint DEBUG = 0;
  if (DEBUG>0) {cout<<"RBF_Grounding::read [START]"<<endl;}
  ifstream in;
  in.open(file_w_c);
  CHECK(in.is_open(), "File "<<file_w_c<<" can't be opened!");
  this->w_c.read(in);
  if (DEBUG>0) {PRINT(this->w_c);}
  in.close();

  in.open(file_w_sigma);
  CHECK(in.is_open(), "File "<<file_w_sigma<<" can't be opened!");
  this->w_sigma.read(in);
  this->w_sigma.reshape(4);
  if (DEBUG>0) {PRINT(this->w_sigma);}
  in.close();
  if (DEBUG>0) {cout<<"RBF_Grounding::read [END]"<<endl;}
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
  f(3) = C.bodies(obj)->shapes(0)->size[0];  // size
}


void relational::getFeatureVectors(MT::Array< arr >& fs, const ors::Graph& C, const uintA& objs) {
  fs.clear();
  uint i;
  FOR1D(objs, i) {
    arr f;
    getFeatureVector(f, C, objs(i));
    fs.append(f);
  }
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
  
  ContinuousState* cont_state = getContinuousState(*C, obj_ids);
  SymbolGrounding::calculateLiterals(lits, sgs, *cont_state);
  delete cont_state;
  if (DEBUG>0) {PRINT(lits);}
  if (DEBUG>0) {cout<<"SymbolGrounding::calculateLiterals [END]"<<endl;}
}





// ------------------------------------------------------------------
// ------------------------------------------------------------------
// ------------------------------------------------------------------
// ------------------------------------------------------------------
// ------------------------------------------------------------------
//
//  ContinuousState

void relational::ContinuousState::read(istream& in) {
  uint DEBUG = 0;
  MT::skip(in);
  in >> object_ids;
  if (DEBUG>0) {PRINT(object_ids);}
  uint i;
  FOR1D(object_ids, i) {
    MT::skip(in);
    arr o_data;
    in >> o_data;
    data.append(o_data);
    if (DEBUG>0) {PRINT(o_data);}
  }
  if (DEBUG>0) {cout<<"Read continuous state:"<<endl;  write(cout);}
}

void relational::ContinuousState::write(ostream& out) const {
  out << object_ids << endl;
  uint i;
  FOR1D(data, i) {
    out << data(i) << endl;
  }
}

bool relational::ContinuousState::operator==(const ContinuousState& other) const {
  return this->object_ids == other.object_ids
         &&  this->data == other.data;
}

bool relational::ContinuousState::operator!=(const ContinuousState& other) const {
  return !(*this==other);
}

relational::ContinuousState* relational::getContinuousState(const ors::Graph& C, const uintA& objects) {
  MT::Array< arr > objects_data;
  relational::getFeatureVectors(objects_data, C, objects);
  relational::ContinuousState* cont_state = new relational::ContinuousState;
  cont_state->object_ids = objects;
  cont_state->data = objects_data;
  return cont_state;
}



  



// ------------------------------------------------------------------
// ------------------------------------------------------------------
// ------------------------------------------------------------------
// ------------------------------------------------------------------
// ------------------------------------------------------------------
//
//  FullExperience


void relational::FullExperience::write_continuous_nice(ostream& out) const {
  out<<"PRE: "<<state_continuous_pre.object_ids.N<<endl;
  state_continuous_pre.write(out);
  out<<"ACTION:"<<endl;
  out<<"action_type="<<action_type<<endl;
  out<<"action_args="<<action_args<<endl;
  out<<"POST: "<<state_continuous_post.object_ids.N<<endl;
  state_continuous_post.write(out);
}

void relational::FullExperience::write_continuous(ostream& out) const {
  out<<"{"<<endl;
  out<<action_type<<endl;
  out<<action_args<<endl;
  out<<reward<<endl;
  out<<endl;
  state_continuous_pre.write(out);
  out<<endl;
  state_continuous_post.write(out);
  out<<"}"<<endl;
}

void relational::FullExperience::write_continuous(MT::Array<FullExperience* > exps, ostream& out) {
  uint i;
  FOR1D(exps, i) {
    exps(i)->write_continuous(out);
    out<<endl;
  }
}

void relational::FullExperience::write_symbolic(ostream& out) const {
  experience_symbolic.write(out);
}
  
void relational::FullExperience::write_symbolic(MT::Array<FullExperience* > exps, ostream& out) {
  uint i;
  FOR1D(exps, i) {
    out<<"["<<i<<"] ("<<(i+1)<<")"<<endl;
    exps(i)->write_symbolic(out);
  }
}


//  Nikolays Format -- Experiences 
// state: 20
// 5 Objekte, 5. Objekt ist die Roboterhand
// (1-3): x,y,z-Koords
void relational::FullExperience::read_continuous_nikolayFormat(MT::Array< FullExperience* >& experiences, const char* file_name) {
  uint DEBUG = 0;
  if (DEBUG>0) {cout<<"read_nikolayFormat [START]"<<endl;}
  if (DEBUG>0) {PRINT(file_name);}
  ifstream in(file_name);
  CHECK(in.is_open(), "File "<<file_name<<" can't be opened!");
  uint OBJ_NUM = 5;
  uintA object_ids;
  uint o;
  for (o=0; o<OBJ_NUM; o++) {
    object_ids.append(o+61);
  }
  while (MT::skip(in) != -1) {
    FullExperience* fex = new FullExperience;
    MT::String line;
    in >> line;
    if (DEBUG>0) PRINT(line);
    fex->state_continuous_pre.object_ids = object_ids;
    for (o=0; o<OBJ_NUM; o++) {
      arr o_data(4);
      line >> o_data(0);  line >> o_data(1);  line >> o_data(2);  line >> o_data(3);
      if (DEBUG>0) PRINT(o_data);
      fex->state_continuous_pre.data.append(o_data);
    }
    fex->state_continuous_post.object_ids = object_ids;
    for (o=0; o<OBJ_NUM; o++) {
      arr o_data(4);
      line >> o_data(0);  line >> o_data(1);  line >> o_data(2);  line >> o_data(3);
      if (DEBUG>0) PRINT(o_data);
      fex->state_continuous_post.data.append(o_data);
    }
    double action_type_double, action_target_double;
    line >> action_type_double;
    if (TL::areEqual(action_type_double, 1.0))  // ACHTUNG: andere Nummerierung (Nikolay startet von 1)
      fex->action_type = grab;
    else if (TL::areEqual(action_type_double, 0.0))
      fex->action_type = puton;
    else
      HALT("Unknown action_type_double="<<action_type_double);
    line >> action_target_double;
    fex->action_args.append((uint) action_target_double + 60);  // -1 since Nikolay starts to count at 1
    if (DEBUG>0) {PRINT(action_type_double);  PRINT(fex->action_type);  PRINT(action_target_double);}
    line >> fex->reward;
    experiences.append(fex);
    if (DEBUG>0) {experiences.last()->write_continuous(cout);}
  }
  
  if (DEBUG>0) {cout<<"read_nikolayFormat [END]"<<endl;}
}





relational::FullExperience* relational::FullExperience::read_continuous(ifstream& in) {
  uint DEBUG = 0;
  if (DEBUG>0) {cout<<"read_continuous [START]"<<endl;}
  
  CHECK(in.is_open(), "Input stream ain't open!");
  relational::FullExperience* e = new relational::FullExperience;
  
  MT::skip(in);
  MT::skipLine(in);  // "{"
  if (DEBUG>0) {PRINT(MT::peerNextChar(in));}
  
  MT::String line;
  // action_type__uint
  MT::skip(in);  line.read(in, NULL, "\n");
  uint action_type__uint;
  MT::skip(in);  line >> action_type__uint;
  e->action_type = ActionType(action_type__uint);
  if (DEBUG>0) {PRINT(action_type__uint);  PRINT(e->action_type);}
  // action_args
  MT::skip(in);  line.read(in, NULL, "\n");
  line >> e->action_args;
  // reward
  MT::skip(in);  line.read(in, NULL, "\n");
  line >> e->reward;
  // pre
  e->state_continuous_pre.read(in);
  // post
  e->state_continuous_post.read(in);
  MT::skip(in);
  MT::skipLine(in);  // "}"
  
  if (DEBUG>0) {e->write_continuous(cout);}
  if (DEBUG>0) {cout<<"read_continuous [END]"<<endl;}
  return e;
}


void relational::FullExperience::read_continuous(MT::Array< relational::FullExperience* >& experiences, const char* filename) {
  uint DEBUG = 0;
  if (DEBUG>0) {cout<<"read_continuous list [START]"<<endl;}
  if (DEBUG>0) {PRINT(filename);}
  experiences.clear();
  ifstream in(filename);
//   PRINT(filename);
  if (!in.is_open()) {
    cerr<<"File " << filename << " can't be opened!"<<endl;
    HALT("");
  } 
  // read other rules
  while (MT::skip(in) != -1) {
    experiences.append(read_continuous(in));
    if (DEBUG>0) {experiences.last()->write_continuous(cout);}
  }
  if (DEBUG>0) {cout<<"read_continuous list [END]"<<endl;}
}


void relational::FullExperience::sanityCheck(MT::Array< FullExperience* >& experiences) {
  uint i;
  FOR1D(experiences, i) {
    if (i==0) continue;
    if (i%30==0) continue;
    if (i%10==0) continue;
    if (experiences(i)->state_continuous_pre != experiences(i-1)->state_continuous_post) {
      cerr << "FullExperience::sanityCheck failed:  continuous states different "<<(i-1)<<" to "<<i << endl;
      cerr << "(i-1)=" << (i-1) << " [--> (" << i << ")]" << endl
           << "experiences(i-1)->state_continuous_post:" << endl;
      experiences(i-1)->write_continuous(cerr);
//            << experiences(i-1)->state_continuous_post << endl;
      cerr << "i=" << i << " [--> (" << (i+1) << ")]" << endl
           << "experiences(i)->state_continuous_pre:" << endl;
      experiences(i)->write_continuous(cerr);
//            << experiences(i)->state_continuous_pre << endl;
      HALT("");
    }
    if (experiences(i)->experience_symbolic.pre != experiences(i-1)->experience_symbolic.post)
      HALT("FullExperience::sanityCheck failed:  symbolic states different "<<(i-1)<<" to "<<i);
  }
}


