#include "prada.h"

#include "ruleReasoning.h"
#include "logicReasoning.h"


#define FAST 1



#define MAX_FUNCTION_VALUE 10
#define RULE_MIN_PROB 0.03
#define TRANS_CLOSURE_STOP_PROB 0.001




namespace TL {

// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------
//    RANDOM VARIABLES
// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------


void PredicateRV::write(ostream& os) {
  os<<"Var over ";lit->write(os);os<<endl;
  os<<"id="<<id<<endl;
  os<<"dim="<<dim<<endl;
  os<<"range="<<range<<endl;
  os<<"changeable="<<changeable<<endl;
  uint d,t;
  for(d=0; d<dim; d++) {
    FOR1D(P, t) {
      os<<"  "<<P(t, d);
    }
    os<<endl;
  }
}


void FunctionRV::write(ostream& os) {
  os<<"Var over ";fi->write(os);os<<endl;
  os<<"id="<<id<<endl;
  os<<"dim="<<dim<<endl;
  os<<"range="<<range<<endl;
  os<<"changeable="<<changeable<<endl;
  uint d,t;
  for(d=0; d<dim; d++) {
    FOR1D(P, t) {
      os<<"  "<<P(t,d);
    }
    os<<endl;
  }
}


void ExpectationFunctionRV::write(ostream& os) {
  os<<"Var over ";fi->write(os);os<<endl;
  os<<"id="<<id<<endl;
  os<<"changeable="<<changeable<<endl;
  os<<"Expectation"<<endl;
  uint t;
  FOR1D(P, t) {
    os<<"  "<<P(t,0);
  }
  os<<endl;
}



void write(PredVA& vars) {
  uint i;
  FOR1D(vars, i) {
    vars(i)->write(cout);
  }
}

void write(FuncVarA& vars) {
  uint i;
  FOR1D(vars, i) {
    vars(i)->write(cout);
  }
}





// -----------------------------------------------------------------
// -----------------------------------------------------------------
// -----------------------------------------------------------------
//         RV_Manager
// -----------------------------------------------------------------
// -----------------------------------------------------------------
// -----------------------------------------------------------------


RV_Manager::RV_Manager(const PredL& preds, const FuncL& funcs, const uintA& constants) {
  this->preds = preds;
  this->funcs = funcs;
  this->constants = constants;

  uint max_d_p = 0;
  uint i;
  FOR1D(preds, i) {
    if (preds(i)->d>max_d_p)
      max_d_p = preds(i)->d;
  }
  pvA.resize(preds.N, (uint) pow(constants.N, max_d_p));
  pvA.setUni(NULL);

  uint max_d_f = 0;
  FOR1D(funcs, i) {
    if (funcs(i)->d>max_d_f)
      max_d_f = funcs(i)->d;
  }
  fvA.resize(preds.N, (uint) pow(constants.N, max_d_f));

  vA.resize(pvA.N+fvA.N+10);

  fiA.resize(preds.N, (uint) pow(constants.N, max_d_f));
}


inline uint getIndex(const uintA& constants, const uintA& args) {
  uint args_idx=0;
  uint i;
  FOR1D(args, i) {
    args_idx += ((uint) pow(constants.N, i)) * constants.findValue(args(i));
  }
//   cout<<"getIndex: constants="<<constants<<"  args="<<args<<"    args_idx="<<args_idx<<endl;
  return args_idx;
}

PredicateRV* RV_Manager::l2v(TL::Literal* lit) const {
  int preds_idx = preds.findValue(lit->atom->pred);
  if (preds_idx<0) {
    MT_MSG("Predicate '"<<lit->atom->pred->name<<"' for lit="<<*lit<<" not in RV_Manager.");
    return NULL;
  }
  PredicateRV* rv = pvA(preds_idx, getIndex(constants, lit->atom->args));
  if (rv == NULL) {
    MT_MSG("RV_Manager::l2v -- No PredicateRV for "<<*lit);
    // manche funktionen rechnen damit, dass NULL uebergeben wird...
  }
  return rv;
}

FunctionRV* RV_Manager::fi2v(TL::FunctionAtom* fi) const {
  return fvA(funcs.findValue(fi->f), getIndex(constants, fi->args));
}

LogicRV* RV_Manager::id2var(uint id_var) const {
  return vA(id_var);
}


void RV_Manager::set(TL::Literal* lit, PredicateRV* var) {
  vA(var->id) = var;
  pvA(preds.findValue(lit->atom->pred), getIndex(constants, lit->atom->args)) = var;
}

void RV_Manager::set(TL::FunctionAtom* fi, FunctionRV* var) {
  vA(var->id) = var;
  uint f_id = funcs.findValue(fi->f);
  uint sa_id = getIndex(constants, fi->args);
  fvA(f_id, sa_id) = var;
  fiA(f_id, sa_id) = fi;
}


TL::FunctionAtom* RV_Manager::getFVW(TL::Function* f, uintA& sa) {
  return fiA(funcs.findValue(f), getIndex(constants, sa));
}






// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------
//    NID_DBN
// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------


NID_DBN::NID_DBN(const uintA& objects, const PredL& preds, const FuncL& funcs, const PredL& actions, TL::RuleSet& ground_rules, double noise_softener, uint horizon) {
  rvm = NULL;
  this->objects = objects;
  this->ground_rules = ground_rules;
  this->noise_softener = noise_softener;
  this->horizon = horizon;
  create_dbn_structure(objects, preds, funcs, actions);
  create_dbn_params();
}


NID_DBN::~NID_DBN() {
  if (rvm!=NULL) delete rvm;
  uint i;
  FOR1D(rvs_state__p_prim, i) {
    delete rvs_state__p_prim(i);
  }
  FOR1D(rvs_state__p_derived, i) {
    delete rvs_state__p_derived(i);
  }
  FOR1D(rvs_state__f_prim, i) {
    delete rvs_state__f_prim(i);
  }
  FOR1D(rvs_state__f_derived, i) {
    delete rvs_state__f_derived(i);
  }
  FOR1D(rvs_action, i) {
    delete rvs_action(i);
  }
}




// ----------------------------------------------------
//                 NID_DBN: Construction
// ----------------------------------------------------


void NID_DBN::create_dbn_structure(const uintA& constants, const PredL& preds, const FuncL& funcs, const PredL& actions) {
  uint DEBUG = 0;
  if (DEBUG) {cout<<"NID_DBN::create_dbn_structure [START]"<<endl;}
  CHECK(!constants.containsDoubles(), "constants ain't distinct, brother! look here: "<<constants);
  
  uint i;
  for (i=0; preds.N>0 && i<preds.N-1; i++) {
    CHECK(!(preds(i)->category == category_derived  &&  preds(i)->category == category_primitive), "unordered preds");
  }
  for (i=0; funcs.N>0 && i<funcs.N-1; i++) {
    CHECK(!(funcs(i)->category == category_derived  &&  funcs(i)->category == category_primitive), "unordered funcs");
  }
  
  // Build RV_Manager
  PredL preds_all;  preds_all.append(preds);  preds_all.append(actions);
  CHECK(rvm == NULL, "DBN has already been built before!");
  rvm = new RV_Manager(preds_all, funcs, constants);
  rvs_state__p_prim.clear();
  rvs_state__p_derived.clear();
  rvs_state__f_prim.clear();
  rvs_state__f_derived.clear();
  rvs_action.clear();
  
  // Build mapping: action --> rules
  // Calc max #rules per grounded action
  uint max_rules_per_action=0, rules_per_action = 0;
  TL::Atom* last_action = NULL;
  FOR1D_(ground_rules, i) {
    if (ground_rules.elem(i)->action != last_action) {
      rules_per_action = 1;
      last_action = ground_rules.elem(i)->action;
    }
    else
      rules_per_action++;
//     ground_rules.elem(i)->action->write(cout); cout<<"   rules_per_action="<<rules_per_action<<endl;
    if (rules_per_action > max_rules_per_action)
      max_rules_per_action = rules_per_action;
  }
  // max action arity
  uint max_arity_actions = 0;
  FOR1D(actions,i) {
    if (actions(i)->d > max_arity_actions)
      max_arity_actions = actions(i)->d;
  }
  if (DEBUG>0) {PRINT(max_rules_per_action);  PRINT(max_arity_actions); PRINT(actions.N);  PRINT(constants.N);}
  
  action2rules.resize(actions.N, pow(constants.N, max_arity_actions), max_rules_per_action);
  action2rules.setUni(9999); // dummy value
  action2rules_no.resize(actions.N, pow(constants.N, max_arity_actions));
  action2rules_no.setZero();
  FOR1D_(ground_rules, i) {
    uint id_pred = actions.findValue(ground_rules.elem(i)->action->pred);
    uint id_args = getIndex(constants, ground_rules.elem(i)->action->args);
    action2rules_no(id_pred, id_args)++;
//     ground_rules.elem(i)->action->write(cout);   cout<<"  id_pred="<<id_pred<<"  id_args="<<id_args<<"  ";   cout<<"   action2rules_no(id_pred, id_args)="<<action2rules_no(id_pred, id_args)<<endl;
    action2rules(id_pred, id_args, action2rules_no(id_pred, id_args)-1) = i;
  }
  
  // Determine changeable predicates and functions
  uintA changeable_ids_p, changeable_ids_f;
  TL::ruleReasoning::changingConcepts(changeable_ids_p, changeable_ids_f, ground_rules);
  if (changeable_ids_p.N == 0  &&  changeable_ids_f.N ==0)
    HALT("no changing concepts! would be stupid to plan");
  
  // Determine predicate and function instances
  LitL lits_state;
  FOR1D(preds, i) {
    LitL lits;
    logicObjectManager::getLiterals(lits, preds(i), constants, true);
    lits_state.setAppend(lits);
    if (DEBUG>3) {cout<<"PIs for "<<preds(i)->name << ":  "; TL::write(lits);  cout<<endl;}
  }
  // Filter PIs with ground-rules
  //   Guideline: If no noise outcome, then only PIs appearing in ground rules are interesting and we can filter.
  bool do_filter = true; // filter if no noise outcomes
  FOR1D_(ground_rules, i) {
    if (!TL::isZero(ground_rules.elem(i)->probs.last())) {
      do_filter = false;
      break;
    }
  }
  
//   cout<<endl<<endl<<endl<<endl;
//   cout<<"GROUND RULES"<<endl;
//   ground_rules.writeNice(cout);
//   cout<<endl<<endl<<endl<<endl;

  if (do_filter) {
    uint k;
    // (1) Use all derived predicates and their precessors.
    //     Only a hack: to account for rewards (which are often specified by means of derived predicates)
    PredL necessary_preds;
    FOR1D(logicObjectManager::p_derived, i) {
      PredL pre_preds;
      FuncL pre_funcs;
      logicObjectManager::getAllPrecessors(*logicObjectManager::p_derived(i), pre_preds, pre_funcs);
      FOR1D(pre_preds, k) {
//         if (changeable_ids_p.findValue(pre_preds(k)->id) < 0) // only take non-changeable into account for sure; OB DAS SO PASST??
          necessary_preds.setAppend(pre_preds(k));
      }
    }
    necessary_preds.setAppend(logicObjectManager::p_derived);
    
    // (2) Use all zero-ary preds
    //     Another HACK: since these might be used for the rewards
    FOR1D(preds, i) {
      if (preds(i)->d == 0)
        necessary_preds.append(preds(i));
    }
    
//     FOR1D(necessary_preds, i) {
//       necessary_preds(i)->writeNice(cout); cout<<endl;
//     }
    
    // (3) Filter
    LitL lits_filtered;
    bool lit_used;
    FOR1D(lits_state, i) {
      // (i)  keep all PIs from "necessary_preds"
      if (necessary_preds.findValue(lits_state(i)->atom->pred) >= 0) { // only primitives taken into account
        lit_used = true;
      }
      // (ii) keep all PIs appearing in ground rules
      else {
        lit_used = false;
        FOR1D_(ground_rules, k) {
          if (TL::ruleReasoning::usesLiteral(*ground_rules.elem(k), lits_state(i))) {
            lit_used = true;
            break;
          }
        }
      }
      if (lit_used)
        lits_filtered.append(lits_state(i));
    }
    lits_state = lits_filtered;
  }
  
  AtomL atoms_action;
  FOR1D_(ground_rules, i) {
    atoms_action.setAppend(ground_rules.elem(i)->action);
    CHECK(atoms_action.last()->pred->d < 1  ||  logicObjectManager::constants.findValue(atoms_action.last()->args(0)) >= 0, "logicObjectManager::constants "<<logicObjectManager::constants<<" don't contain action arg for action "<<*atoms_action.last());
  }
  
  FuncAL fas;
  FOR1D(funcs, i) {
    FuncAL fvws_local;
    logicObjectManager::getFAs(fvws_local, funcs(i), constants);
    fas.setAppend(fvws_local);
  }
  if (DEBUG>0) {
    cout<<"+++++   FINAL LITERALS (after filtering)   +++++"<<endl;
    cout<<"State lits["<<lits_state.N<<"]:  ";TL::write(lits_state);cout<<endl;
    cout<<"fas["<<fas.N<<"]:  ";TL::write(fas);cout<<endl;
    cout<<"Action lits["<<atoms_action.N<<"]:  ";TL::write(atoms_action);cout<<endl;
  }
  uint a, r, v;
  uint rv_ids = 1;
  
  
  // Build random variables
  // (1) for predicate tuples
  start_id_rv__predicate = rv_ids;
  FOR1D(lits_state, v) { // assumed to be in order p_prim to p_derived
    PredicateRV* var = new PredicateRV;
    var->lit = lits_state(v);
    var->dim = 2;
    var->range.resize(2); var->range(0)=0; var->range(1)=1;
    var->P.resize(horizon+1, var->dim);
    var->P.setZero();
    var->id = rv_ids++;
    if (changeable_ids_p.findValue(var->lit->atom->pred->id)>=0)
      var->changeable = true;
    else
      var->changeable = false;
    if (lits_state(v)->atom->pred->category == category_primitive)
      rvs_state__p_prim.append(var);
    else
      rvs_state__p_derived.append(var);
    rvm->set(lits_state(v), var);
  }
  // (2) for function values
  start_id_rv__function = rv_ids;
  uintA rule_functions_ids; // functions that are used in rules
  FOR1D_(ground_rules, r) {
    FOR1D(ground_rules.elem(r)->context, v) {
      if (ground_rules.elem(r)->context(v)->atom->pred->type == TL::Predicate::predicate_comparison) {
        ComparisonLiteral* clit = (TL::ComparisonLiteral*) ground_rules.elem(r)->context(v);
        rule_functions_ids.setAppend(((ComparisonAtom*)clit->atom)->fa1->f->id);
      }
    }
  }
  FOR1D(fas, v) {
    if (rule_functions_ids.findValue(fas(v)->f->id)<0  &&
        (fas(v)->f->type == TL::Function::function_count  ||
        fas(v)->f->type == TL::Function::function_avg  ||
        fas(v)->f->type == TL::Function::function_sum  ||
        fas(v)->f->type == TL::Function::function_max  ||
        fas(v)->f->type == TL::Function::function_reward)) {
      ExpectationFunctionRV* var = new ExpectationFunctionRV;
      var->fi = fas(v);
      var->P.resize(horizon+1, var->dim);
      var->P.setZero();
      var->id = rv_ids++;
      if (changeable_ids_f.findValue(var->fi->f->id)>=0)
        var->changeable = true;
      else
        var->changeable = false;
      rvs_state__f_derived.append(var);
      rvm->set(fas(v), var);
    }
    else {
      FunctionRV* var = new FunctionRV;
      var->fi = fas(v);
      var->dim = var->fi->f->range.N;
      var->range = var->fi->f->range;
      var->P.resize(horizon+1, var->dim);
      var->P.setZero();
      var->id = rv_ids++;
      if (changeable_ids_f.findValue(var->fi->f->id)>=0)
        var->changeable = true;
      else
        var->changeable = false;
      if (fas(v)->f->category == category_primitive)
        rvs_state__f_prim.append(var);
      else
        rvs_state__f_derived.append(var);
      rvm->set(fas(v), var);
    }
  }
  num_state_vars = rv_ids-1;
  // (3) for actions
  FOR1D(atoms_action, a) {
    if (atoms_action(a)->pred->id == TL::DEFAULT_ACTION_PRED__ID) continue; // Noisy default rule
    PredicateRV* var = new PredicateRV;
    var->lit = logicObjectManager::getLiteral(atoms_action(a)->pred, true, atoms_action(a)->args);
    var->dim = 2; // binary variables
    var->range.resize(2); var->range(0)=0; var->range(1)=1;
    var->P.resize(horizon+1, var->dim);
    var->P.setZero();
    var->id = rv_ids++;
    rvs_action.append(var);
    rvm->set(logicObjectManager::getLiteral(atoms_action(a)->pred, true, atoms_action(a)->args), var);
  }
  if (DEBUG>1) {
    cout<<"State rvs prim:  "<<endl;write(rvs_state__p_prim);cout<<endl;
    cout<<"State rvs p_derived:  "<<endl;write(rvs_state__p_derived);cout<<endl;
    cout<<"Action rvs:  "<<endl;write(rvs_action);cout<<endl;
    cout<<"Function rvs prim:  "<<endl;write(rvs_state__f_prim);cout<<endl;
    cout<<"Function rvs p_derived:  "<<endl;write(rvs_state__f_derived);cout<<endl;
  }
  
  // (4) Pseudo-RVs for rules
  rvs_rules_simple.resize(horizon, ground_rules.num());
  rvs_rules.resize(horizon, ground_rules.num());
  
  if (DEBUG) {cout<<"NID_DBN::create_dbn_structure [END]"<<endl;}
}



void NID_DBN::create_dbn_params() {
  uint DEBUG = 0;
  if (DEBUG) {cout<<"create_dbn_params [START]"<<endl;}
  
  uint r, v, val, o;
  uint V_prim = rvs_state__p_prim.N;
  uint F_prim = rvs_state__f_prim.N;
  uint R = ground_rules.num();
  impacts_val_p.resize(R, V_prim, 2);
  impacts_val_p.setZero();
  impacts_val_f.resize(R, F_prim, MAX_FUNCTION_VALUE);
  impacts_val_f.setZero();
  double prob_noise_change;
  uint no_changeable_rvs = 0;
  FOR1D(rvs_state__p_prim, v) {
    if (rvs_state__p_prim(v)->changeable) no_changeable_rvs++;
  }
  FOR1D(rvs_state__f_prim, v) {
    if (rvs_state__f_prim(v)->changeable) no_changeable_rvs++;
  }
  FOR1D_(ground_rules, r) {
    TL::Rule* r_grounded = ground_rules.elem(r);
    prob_noise_change = r_grounded->noise_changes / no_changeable_rvs * noise_softener;
    FOR1D(r_grounded->outcomes, o) {
      // noise outcome! (= last outcome)
      if (o==r_grounded->outcomes.N-1) {
        // touches every changeable random variable in the state!
        FOR1D(rvs_state__p_prim, v) {
          if (rvs_state__p_prim(v)->changeable) {
            FOR1D(rvs_state__p_prim(v)->range, val) {
              impacts_val_p(r,rvs_state__p_prim(v)->id-start_id_rv__predicate,val) += prob_noise_change * r_grounded->probs(o);
            }
          }
        }
        FOR1D(rvs_state__f_prim, v) {
          if (rvs_state__f_prim(v)->changeable) {
            FOR1D(rvs_state__f_prim(v)->range, val) {
              impacts_val_f(r,rvs_state__f_prim(v)->id-start_id_rv__function,val) += prob_noise_change * r_grounded->probs(o);
              CHECK(false, "u better never come here, my friend");
            }
          }
        }
      }
      // non-noise
      else {
        FOR1D(r_grounded->outcomes(o), v) {
          if (r_grounded->outcomes(o)(v)->atom->pred->type == TL::Predicate::predicate_comparison) {
            // updatet nen functionvalue
            // kommt nie vor bisher...
            NIY;
          }
          else {
            LogicRV* var;
            // check whether same variable appears multiple times in this outcome
            uint v2;
            uintA indices;
            FOR1D(r_grounded->outcomes(o), v2) {
              if (r_grounded->outcomes(o)(v) == r_grounded->outcomes(o)(v2))
                indices.append(v2);
              else if (r_grounded->outcomes(o)(v) == logicObjectManager::getLiteralNeg(r_grounded->outcomes(o)(v2)))
                indices.append(v2);
            }
            // Case 1: appears only once
            if (indices.N == 1) {
              if (r_grounded->outcomes(o)(v)->positive) {
//                 r_grounded->writeNice(cout);
//                 PRINT(o);
//                 PRINT(v);
                var = rvm->l2v(r_grounded->outcomes(o)(v));
//                 cout<<"fu"<<endl;
              }
              else
                var = rvm->l2v(logicObjectManager::getLiteralNeg(r_grounded->outcomes(o)(v)));
              if (r_grounded->outcomes(o)(v)->positive)
                impacts_val_p(r,var->id-1,1) += r_grounded->probs(o);
              else
                impacts_val_p(r,var->id-1,0) += r_grounded->probs(o);
            }
            // Case 2: appears several times --> may happen due to rule groundings
            // Case 2.1: already covered before
            else if (indices(0) < v)
              continue;
            // Case 2.2: not covered yet
            else {
              uint num_pos = 0;
              uint num_neg = 0;
              FOR1D(indices, v2) {
                if (r_grounded->outcomes(o)(indices(v2))->positive)
                  num_pos++;
                else
                  num_neg++;
              }
              if (r_grounded->outcomes(o)(v)->positive)
                var = rvm->l2v(r_grounded->outcomes(o)(v));
              else
                var = rvm->l2v(logicObjectManager::getLiteralNeg(r_grounded->outcomes(o)(v)));
              if (num_pos > num_neg) {  // more positive occurrences
                impacts_val_p(r,var->id-1,1) += r_grounded->probs(o);
              }
              else if (num_neg > num_pos) {  // more negative occurrences
                impacts_val_p(r,var->id-1,0) += r_grounded->probs(o);
              }
              // else num_pos == num_neg --> don't do anything / ignore
            }
          }
        }
      }
    }
  }
  impacts_V_p.resize(R, V_prim);
  impacts_V_p.setUni(0.0);
  impacts_V_f.resize(R, F_prim);
  impacts_V_f.setUni(0.0);
#ifndef FAST
  FOR3D(impacts_val_p, r, v, val) {
    impacts_V_p(r, v) += impacts_val_p(r,v,val);
  }
  FOR3D(impacts_val_f, r, v, val) {
    impacts_V_f(r, v) += impacts_val_f(r,v,val);
  }
#endif
#ifdef FAST
  uint p_id_1;
  uint p_id_2;
  p_id_1 = p_id_2 = 0;
  for (r=0; r<R; r++) {
    for (v=0; v<V_prim; v++) {
      impacts_V_p.p[p_id_2] += impacts_val_p.p[p_id_1] + impacts_val_p.p[p_id_1+1];
      p_id_1 += 2;
      p_id_2++;
    }
  }
  p_id_1 = p_id_2 = 0;
  for (r=0; r<R; r++) {
    for (v=0; v<F_prim; v++) {
      for (val=0; val<MAX_FUNCTION_VALUE; val++) {
        impacts_V_f.p[p_id_2] += impacts_val_f.p[p_id_1];
        p_id_1++;
      }
      p_id_2++;
    }
  }
#endif
  
  
//   FOR2D(impacts_V_f, r, v) {
//     CHECK(TL::isZero(impacts_V_f(r,v)), "shouldn't be an impact on functions (not implemented yet)");
//   }

  if (DEBUG>1) {
    cout<<"+++++   IMPACTS   +++++"<<endl;
    for (r=0; r<R; r++) {
      cout<<"["<<r<<"]"<<endl;
      ground_rules.elem(r)->write(cout);
      FOR1D(rvs_state__p_prim, v) {
        rvs_state__p_prim(v)->lit->write(cout);cout<<":  ";
        for (val=0; val<rvs_state__p_prim(v)->dim; val++) {
          cout<<rvs_state__p_prim(v)->range(val)<<":"<<impacts_val_p(r,v,val)<<"  ";
        }
        cout<<" -->  "<< impacts_V_p(r, v) << endl;
      }
      FOR1D(rvs_state__f_prim, v) {
        rvs_state__f_prim(v)->fi->write(cout);cout<<":  ";
        for (val=0; val<rvs_state__f_prim(v)->dim; val++) {
          cout<<rvs_state__f_prim(v)->range(val)<<":"<<impacts_val_f(r,v,val)<<"  ";
        }
        cout<<" -->  "<< impacts_V_f(r, v) << endl;
      }
    }
  }
  
  // build helper array vars 
  vars_context.resize(ground_rules.num(), 20);
  FOR1D_(ground_rules, r) {
    uint v_current = 0;
    FOR1D(ground_rules.elem(r)->context,v) {
      if (v_current == vars_context.d1) {
        HALT("Too large context for ground rule r="<<r<<" (PRADA only copes with maximum 20 literals in rule contexts)");
      }
      if (ground_rules.elem(r)->context(v)->atom->pred->type == TL::Predicate::predicate_comparison) {
        TL::ComparisonLiteral* clit = (TL::ComparisonLiteral*) ground_rules.elem(r)->context(v);
        // f(x) < c
        if (((TL::ComparisonPredicate*) clit->atom->pred)->constantBound) {
          TL::FunctionAtom* fi = rvm->getFVW(((ComparisonAtom*)clit->atom)->fa1->f, ((ComparisonAtom*)clit->atom)->args);
          vars_context(r, v_current++) = rvm->fi2v(fi);
        }
        // f(x) < f(y)
        else {
          uintA sa_1, sa_2;
          FOR1D(((ComparisonAtom*)clit->atom)->args, val) {
            if (val<((ComparisonAtom*)clit->atom)->args.N/2)
              sa_1.append(((ComparisonAtom*)clit->atom)->args(val));
            else
              sa_2.append(((ComparisonAtom*)clit->atom)->args(val));
          }
          TL::FunctionAtom* fvw1 = rvm->getFVW(((ComparisonAtom*)clit->atom)->fa1->f, sa_1);
          TL::FunctionAtom* fvw2 = rvm->getFVW(((ComparisonAtom*)clit->atom)->fa1->f, sa_2);
          vars_context(r, v_current++) = rvm->fi2v(fvw1);
          vars_context(r, v_current++) = rvm->fi2v(fvw2);
        }
      }
      else {
        vars_context(r, v_current++) = rvm->l2v(ground_rules.elem(r)->context(v));
      }
    }
  }
  
  if (DEBUG) {cout<<"create_dbn_params [END]"<<endl;}
}




// ----------------------------------------------------
//                 NID_DBN:   inference rules
// ----------------------------------------------------


#ifdef FAST
double inferPhi(const TL::Rule& grounded_rule, uint rule_id, uint t, RV_Manager* rvm, const LVA& vars_context) {
  uint DEBUG = 0;
  if (DEBUG>0) {cout<<"inferPhi [START]"<<endl;}
  if (DEBUG>0) {
    PRINT(t);
    grounded_rule.write();
  }
  double phi = 1.0;
  uint i, val, val2;
  double prob;
  uint v_current=0;
  uint id_2D__val1, id_2D__val2;
  FOR1D(grounded_rule.context, i) {
    prob = 0.0;
    if (grounded_rule.context(i)->atom->pred->type == TL::Predicate::predicate_comparison) {
      TL::ComparisonLiteral* clit = (TL::ComparisonLiteral*) grounded_rule.context(i);
      TL::ComparisonAtom* ca = (TL::ComparisonAtom*) clit->atom;
      // f(x) < c
      if (((TL::ComparisonPredicate*) grounded_rule.context(i)->atom->pred)->constantBound) {
        LogicRV* var = vars_context(rule_id, v_current++);
        if (var->type == RV_TYPE__FUNC_EXPECT) {
          if (TL::compare(var->P(t,0), ca->bound, ca->comparisonType))
            prob += 1.;
        }
        else {
          id_2D__val1 = var->P.d1 * t;
          FOR1D(var->range, val) {
            if (TL::compare(var->range(val), ca->bound, ca->comparisonType))
              prob += var->P.p[id_2D__val1];
            id_2D__val1++;
          }
        }
      }
      // f(x) < f(y)
      else {
        LogicRV* variable1 = vars_context(rule_id, v_current++);
        LogicRV* variable2 = vars_context(rule_id, v_current++);
        if (DEBUG>2) {
          variable1->write(cout);
          variable2->write(cout);
        }
        if (variable1->type == RV_TYPE__FUNC_EXPECT) {
          if (TL::compare(variable1->P(t,0), variable2->P(t,0), ((ComparisonAtom*)clit->atom)->comparisonType))
            prob +=  1.0;
        }
        else {
          id_2D__val1 = variable1->P.d1 * t; // Wird unten am Schleifenende je 1 hochgezaehlt.
          FOR1D(variable1->range, val) {
            if (((ComparisonAtom*)clit->atom)->comparisonType == comparison_equal) {
              prob += variable1->P.p[id_2D__val1] * variable2->P.p[id_2D__val1];
            }
            else if (((ComparisonAtom*)clit->atom)->comparisonType == comparison_less) {
              id_2D__val2 = variable2->P.d1 * t + val + 1;
              for (val2 = val+1; val2 < variable2->range.d0; val2++) {
                prob += variable1->P.p[id_2D__val1] * variable2->P.p[id_2D__val2];
                id_2D__val2++;
              }
            }
            else if (((ComparisonAtom*)clit->atom)->comparisonType == comparison_lessEqual) {
              id_2D__val2 = variable2->P.d1 * t + val;
              for (val2 = val; val2 < variable2->range.d0; val2++) {
                prob += variable1->P.p[id_2D__val1] * variable2->P.p[id_2D__val2];
                id_2D__val2++;
              }
            }
            else if (((ComparisonAtom*)clit->atom)->comparisonType == comparison_greater) {
              id_2D__val2 = variable2->P.d1 * t + val - 1;
              if (val>0) {
                for (val2 = val-1; val2--; ) {
                  prob += variable1->P.p[id_2D__val1] * variable2->P.p[id_2D__val2];
                  id_2D__val2--;
                }
              }
            }
            else if (((ComparisonAtom*)clit->atom)->comparisonType == comparison_greaterEqual) {
              id_2D__val2 = variable2->P.d1 * t + val;
              for (val2 = val; val2--; ) {
                prob += variable1->P.p[id_2D__val1] * variable2->P.p[id_2D__val2];
                id_2D__val2--;
              }
            }
            
            id_2D__val1++;
          }
//           FOR1D(var2->range, val2) {
// //               if (((ComparisonAtom*)clit->atom)->compare(var1->range(val), var2->range(val2))) prob += var1->P(t,val) * var2->P(t,val2);
// //               if (((ComparisonAtom*)clit->atom)->compare(var1->range(val), var2->range(val2))) prob += var1->P.p[id_2D__val1] * var2->P.p[id_2D__val2];
//             if (((ComparisonAtom*)clit->atom)->compare(var1->range.p[id_1D__val1], var2->range.p[id_1D__val2])) prob += var1->P.p[id_2D__val1] * var2->P.p[id_2D__val2];
//             id_2D__val2++;
//             id_1D__val2++;
//           }
          
        }
      }
    }
    else {
      // Can be safely assumed to be binary (since function values only appear in p_comp).
      LogicRV* var = vars_context(rule_id, v_current++);
      if (grounded_rule.context(i)->positive)
        prob = var->P(t,1);
      else
        prob = var->P(t,0);
    }
    if (DEBUG>0) {cout<<i<<":"<<prob<<"  ";}
    phi *= prob;
    if (phi < RULE_MIN_PROB) {phi=0.0; break;}
  }
  if (DEBUG>0) {
    cout<<"  -->  phi="<<phi<<endl;
    if (phi>0.) {grounded_rule.action->write(); cout<<" *****"<<endl;}
    cout<<"inferPhi [END]"<<endl;
  }
  return phi;
}
#endif
#ifndef FAST
double inferPhi(const TL::Rule& grounded_rule, uint t, RV_Manager* rvm) {
  uint DEBUG = 0;
  if (DEBUG>0) {cout<<"inferPhi [START]"<<endl;}
  if (DEBUG>0) {
    PRINT(t);
    grounded_rule.writeNice();
  }
  double phi = 1.0;
  uint i, val, val2;
  double prob;
  FOR1D(grounded_rule.context, i) {
    prob = 0.0;
    if (grounded_rule.context(i)->atom->pred->type == TL::Predicate::predicate_comparison) {
      TL::ComparisonLiteral* clit = (TL::ComparisonLiteral*) grounded_rule.context(i);
      // f(x) < c
      if (((TL::ComparisonPredicate*) clit->pred)->constantBound) {
        TL::FunctionAtom* fi = rvm->getFVW(((ComparisonAtom*)clit->atom)->f, ((ComparisonAtom*)clit->atom)->args);
        LogicRV* var = rvm->fi2v(fi);
        if (var->type == RV_TYPE__FUNC_EXPECT) {
          if (((ComparisonAtom*)clit->atom)->compare(var->P(t,0))) prob += 1.;
        }
        else {
          FOR1D(var->range, val) {
            if (((ComparisonAtom*)clit->atom)->compare(var->range(val))) prob += var->P(t,val);
          }
        }
      }
      // f(x) < f(y)
      else {
        uintA sa_1, sa_2;
        FOR1D(((ComparisonAtom*)clit->atom)->args, val) {
          if (val<((ComparisonAtom*)clit->atom)->args.N/2)
            sa_1.append(((ComparisonAtom*)clit->atom)->args(val));
          else
            sa_2.append(((ComparisonAtom*)clit->atom)->args(val));
        }
        TL::FunctionAtom* fvw1 = rvm->getFVW(((ComparisonAtom*)clit->atom)->f, sa_1);
        TL::FunctionAtom* fvw2 = rvm->getFVW(((ComparisonAtom*)clit->atom)->f, sa_2);
        LogicRV* var1 = rvm->fi2v(fvw1);
        LogicRV* var2 = rvm->fi2v(fvw2);
        if (var1->type == RV_TYPE__FUNC_EXPECT) {
          if (((ComparisonAtom*)clit->atom)->compare(var1->P(t,0), var2->P(t,0))) prob +=  1.0;
        }
        else {
          FOR1D(var1->range, val) {
            FOR1D(var2->range, val2) {
              if (((ComparisonAtom*)clit->atom)->compare(var1->range(val), var2->range(val2))) prob += var1->P(t,val) * var2->P(t,val2);
            }
          }
        }
      }
    }
    else {
      // Can be safely assumed to be binary (since function values only appear in p_comp).
      if (grounded_rule.context(i)->positive)
        prob = rvm->l2v(grounded_rule.context(i))->P(t,1);
      else
        prob = rvm->l2v(grounded_rule.context(i))->P(t,0);
    }
    if (DEBUG>0) {cout<<i<<":"<<prob<<"  ";}
    alpha *= prob;
    if (alpha < RULE_MIN_PROB) {
      alpha=0.0; 
      if (DEBUG>0) {cout<<"Pruning rule to 0."<<endl;}
      break;
    }
  }
  if (DEBUG>0) {cout<<"  -->  phi="<<phi<<endl<<"inferPhi [END]"<<endl;}
  return phi;
}
#endif








// rvs_rules_simple:  P(\phi_r | s)
// rvs_rules: P(\phi_r | -\phi_r', s)
void NID_DBN::inferRules(uint t) {
  uint DEBUG = 0;
  if (DEBUG>0) {cout<<"inferRules [START]"<<endl;}
  if (DEBUG>0) {PRINT(t);}
  uint r, r2;
  
  // rvs_rules_simple
  uint p_id = t * rvs_rules_simple.d1;
  FOR1D_(ground_rules, r) {
#ifdef FAST
    rvs_rules_simple.p[p_id++] = inferPhi(*ground_rules.elem(r), r, t, rvm, vars_context);
#endif
#ifndef FAST
    rvs_rules_simple(t,r) = inferPhi(*ground_rules.elem(r), t, rvm);
#endif
  }
  if (DEBUG>1) {
    cout<<"phi  P(phi_r | s): (for >0)";
    for(r=0; r<ground_rules.num(); r++) {
      if (rvs_rules_simple(t,r)>0.0) {
        ground_rules.elem(r)->write();
        cout << " -->  P(\\phi_r | s) = " << rvs_rules_simple(t,r) << endl;
      }
    }
    cout<<endl;
  }
  
  // rvs_rules
  p_id = t * rvs_rules_simple.d1;
  FOR1D_(ground_rules, r) {
#ifdef FAST
    if (r>0) p_id++;
//     PRINT(p_id);
//     PRINT(rvs_rules.d1 * t + r);
    rvs_rules.p[p_id] = rvs_rules_simple.p[p_id];
    if (rvs_rules.p[p_id] < RULE_MIN_PROB) {rvs_rules.p[p_id]=0.0; continue;}
    uint id_pred = logicObjectManager::p_actions.findValue(ground_rules.elem(r)->action->pred);
    uint id_args = getIndex(logicObjectManager::constants, ground_rules.elem(r)->action->args);
    for (r2=0; r2<action2rules_no(id_pred, id_args); r2++) {
      if (action2rules(id_pred, id_args, r2) == r) continue;
      else {
        rvs_rules.p[p_id] *= (1. - rvs_rules_simple(t,action2rules(id_pred, id_args,r2)));
        if (rvs_rules.p[p_id] < RULE_MIN_PROB) {rvs_rules.p[p_id]=0.0; break;}
      }
    }
#endif
#ifndef FAST
    rvs_rules(t,r) = alpha(t,r);
    if (rvs_rules(t,r) < RULE_MIN_PROB) {rvs_rules(t,r)=0.0; continue;}
    uint id_pred = logicObjectManager::p_actions.findValue(ground_rules.elem(r)->action->pred);
    uint id_args = getIndex(constants, ground_rules.elem(r)->action->args);
    for (r2=0; r2<action2rules_no(id_pred, id_args); r2++) {
      if (action2rules(id_pred, id_args,r2) == r) continue;
      else {
        rvs_rules(t,r) *= (1. - alpha(t,action2rules(id_pred, id_args,r2)));
        if (rvs_rules(t,r) < RULE_MIN_PROB) {rvs_rules(t,r)=0.0; break;}
      }
    }
#endif
  }
  if (DEBUG>0) {
    cout<<"rvs_rules  P(phi_r | s, phi_r'):";
    for(r=0; r<ground_rules.num(); r++) {
      cout<<"     "<<*ground_rules.elem(r)->action<<" "<<rvs_rules(t,r)<<" (" << rvs_rules_simple(t,r) << ")";
    }
    cout<<endl;
    if (DEBUG>1) {
      cout<<"Rules with prob>0.0:"<<endl;
      for(r=0; r<ground_rules.num(); r++) {
        if (rvs_rules(t,r)>0.0) {
          ground_rules.elem(r)->write();
          cout << " --> P(phi_r | s, phi_r') = " << rvs_rules(t,r) << endl;
        }
      }
      cout<<endl;
    }
  }
  
  if (DEBUG>0) {cout<<"inferRules [END]"<<endl;}
}





// ----------------------------------------------------
//       NID_DBN:   inference state
// ----------------------------------------------------

void NID_DBN::inferState(uint t, TL::Atom* given_action) {
  uint id_pred = logicObjectManager::p_actions.findValue(given_action->pred);
  uint id_args = getIndex(logicObjectManager::constants, given_action->args);
  double action_weight = 0.;
  uint r;
  for (r=0; r<action2rules_no(id_pred, id_args); r++) {
    action_weight += rvs_rules(t-1, action2rules(id_pred, id_args, r));
  }
  inferState(t, given_action, action_weight);
}

// from state and action at t-1
void NID_DBN::inferState(uint t, TL::Atom* given_action, double given_action_weight) {
  uint DEBUG = 0;
  
  if (DEBUG>0) {cout<<"inferState [START]"<<endl;}
  CHECK(t>0, "only works for state > 0");
  if (DEBUG>0) {
    PRINT(t);
    given_action->write(cout); cout<<endl;
    PRINT(given_action_weight);
  }
   
  uint r, r2, v, val;
  
  // Determine which action has been taken
  uint action_id_pred = logicObjectManager::p_actions.findValue(given_action->pred);
  uint action_id_args = getIndex(logicObjectManager::constants, given_action->args);
//   uint given_action_rv_id = rvm->l2v(given_action)->id - (start_id_rv__predicate + num_state_vars);
//   PRINT(given_action->args);
//   PRINT(action_id_pred);
//   PRINT(action_id_args);
  
  // (1) predicates
  // (1a) calc P_t+1(v_i|r, a_t, s)
  arr P_r_v__p(ground_rules.num(), rvs_state__p_prim.N, 2);
  uintA rules_with_sampled_action;
  uint p_id_start = (action_id_pred * action2rules.d1  +  action_id_args) * action2rules.d2;
  
  for(r=0; r<action2rules_no(action_id_pred, action_id_args); r++) {
    rules_with_sampled_action.append(action2rules.p[p_id_start+r]);
  }
  if (DEBUG>1) {PRINT(rules_with_sampled_action);}
  if (DEBUG>1) {
    cout<<"Important rules: "<<endl;
    FOR1D(rules_with_sampled_action, r) {
      ground_rules.elem(rules_with_sampled_action(r))->write(cout);
      cout<<"  --> applicability "<<rvs_rules(t-1, rules_with_sampled_action(r));
      if (rvs_rules(t-1, rules_with_sampled_action(r)) > 0.05) {
        cout<<" ***";
      }
      cout<<endl;
    }
  }
#ifdef FAST
  // SCHNELLE VARIANTE
  double one_minus_impacts_V_p__r2_v;
  LogicRV* rvs_state__p_prim__v;
  uint p_id;
  FOR1D(rules_with_sampled_action, r) {
    r2 = rules_with_sampled_action(r);
    p_id = r2 * P_r_v__p.d1 * P_r_v__p.d2;
    if (given_action != ground_rules.elem(r2)->action) {
      cout<<"given_action: "; given_action->write(cout); cout<<endl;
      cout<<"ground_rules.elem(r2)->action: "; ground_rules.elem(r2)->action->write(cout); cout<<endl;
      HALT("rule has incorrect action!");
    }
    FOR1D(rvs_state__p_prim, v) {
      one_minus_impacts_V_p__r2_v = 1. - impacts_V_p(r2,v);
      rvs_state__p_prim__v = rvs_state__p_prim(v);
      P_r_v__p.p[p_id] = impacts_val_p.p[p_id] + one_minus_impacts_V_p__r2_v * rvs_state__p_prim__v->P(t-1,0);
      p_id++;
      P_r_v__p.p[p_id] = impacts_val_p.p[p_id] + one_minus_impacts_V_p__r2_v * rvs_state__p_prim__v->P(t-1,1);
      p_id++;
    }
  }
#endif
#ifndef FAST
  // RICHTIGE VARIANTE
  FOR1D(rules_with_sampled_action, r) {
    r2 = rules_with_sampled_action(r);
    FOR1D(rvs_state__p_prim, v) {
      CHECK(sampled_action == ground_rules.elem(r2)->action, "rule has incorrect action!");
      FOR1D(rvs_state__p_prim(v)->range, val) {
        P_r_v__p(r2,v,val) = impacts_val_p(r2,v,val) + (1.-impacts_V_p(r2,v)) * rvs_state__p_prim(v)->P(t-1,val);
      }
    }
  }
#endif
  if (DEBUG>2) {
    cout<<"P_r_v__p  =  P(v_i | r, a_t, s) for predicates:"<<endl;
    for(r=0; r<ground_rules.num(); r++) {
      cout<<"rule "<<r<<""<<endl;
      for(v=0; v<rvs_state__p_prim.N; v++) {
        rvs_state__p_prim(v)->lit->write();cout<<":  ";
        FOR1D(rvs_state__p_prim(v)->range, val) {
          cout<<rvs_state__p_prim(v)->range(val)<<":"<<P_r_v__p(r,v,val);
          cout<<"  ";
        }
        cout<<endl;
      }
    }
  }
  // (1b) calc P_t+1(v_i|a,s) = next time-step distribution
#ifdef FAST
  // SCHNELLE VARIANTE
  FOR1D(rvs_state__p_prim, v) {
    FOR1D(rvs_state__p_prim(v)->range, val) {
      rvs_state__p_prim(v)->P(t,val) = (1.-given_action_weight) * rvs_state__p_prim(v)->P(t-1,val);
    }
  }
  for(r=0; r<action2rules_no(action_id_pred, action_id_args); r++) {
    r2 = action2rules.p[p_id_start+r];
    p_id = r2 * P_r_v__p.d1 * P_r_v__p.d2;
    FOR1D(rvs_state__p_prim, v) {
      rvs_state__p_prim(v)->P(t,0) += P_r_v__p.p[p_id++] * rvs_rules(t-1, r2);
      rvs_state__p_prim(v)->P(t,1) += P_r_v__p.p[p_id++] * rvs_rules(t-1, r2);
    }
  }
#endif
#ifndef FAST
  // RICHTIGE VARIANTE
  FOR1D(rvs_state__p_prim, v) {
    FOR1D(rvs_state__p_prim(v)->range, val) {
      rvs_state__p_prim(v)->P(t,val) = 0.0;
      for(r=0; r<action2rules_no(action_id_pred, action_id_sa); r++) {
        r2 = action2rules(action_id_pred, action_id_sa, r);
        rvs_state__p_prim(v)->P(t,val) += P_r_v__p(r2,v,val) * rvs_rules(t-1, r2);
      }
      rvs_state__p_prim(v)->P(t,val) += (1.-given_action_weight) * rvs_state__p_prim(v)->P(t-1,val);
    }
  }
#endif
  // (2) functions
  // (2a) calc P_t+1(v_i|r, a_t, s)
  arr P_v_r__f(ground_rules.num(), rvs_state__f_prim.N, MAX_FUNCTION_VALUE);
  if (DEBUG>1) {PRINT(rules_with_sampled_action);}
#ifdef FAST
  LogicRV* rvs_state__f_prim__v;
  double one_minus_impacts_V_f__r2_v;
  FOR1D(rules_with_sampled_action, r) {
    r2 = rules_with_sampled_action(r);
    p_id = r2 * P_v_r__f.d1 * P_v_r__f.d2;
//       cout<<"CHANGE1"<<endl;
    FOR1D(rvs_state__f_prim, v) {
      rvs_state__f_prim__v = rvs_state__f_prim(v);
      one_minus_impacts_V_f__r2_v = 1. - impacts_V_f(r2,v);
//         cout<<"CHANGE2"<<endl;
//         p_id = r2 * P_v_r__f.d1 * P_v_r__f.d2 + v * P_v_r__f.d2;
      FOR1D(rvs_state__f_prim__v->range, val) {
//           PRINT(p_id);
//           PRINT(((r2*P_v_r__f.d1+v)*P_v_r__f.d2+val));
//           PRINT(((r2*impacts_val_f.d1+v)*impacts_val_f.d2+val));
        P_v_r__f.p[p_id] = impacts_val_f.p[p_id] + one_minus_impacts_V_f__r2_v * rvs_state__f_prim__v->P(t-1,val);
        p_id++;
      }
      p_id += MAX_FUNCTION_VALUE - val;
    }
  }
#endif
#ifndef FAST
  FOR1D(rules_with_sampled_action, r) {
    r2 = rules_with_sampled_action(r);
    FOR1D(rvs_state__f_prim, v) {
      FOR1D(rvs_state__f_prim(v)->range, val) {
        P_v_r__f(r2,v,val) = impacts_val_f(r2,v,val) + (1.-impacts_V_f(r2,v)) * rvs_state__f_prim(v)->P(t-1,val);
      }
    }
  }
#endif
  if (DEBUG>2) {
    cout<<"P_v_r__f =  P(v_i | r, a_t, s) for function values:"<<endl;
    for(r=0; r<ground_rules.num(); r++) {
      cout<<"rule "<<r<<""<<endl;
      for(v=0; v<rvs_state__f_prim.N; v++) {
        rvs_state__f_prim(v)->fi->write();cout<<":  ";
        FOR1D(rvs_state__f_prim(v)->range, val) {
          cout<<rvs_state__f_prim(v)->range(val)<<":"<<P_v_r__f(r,v,val);
          cout<<"  ";
        }
        cout<<endl;
      }
    }
  }
  // (2b) calc P_t+1(v_i|a,s) = next time-step distribution
#ifdef FAST
  FOR1D(rvs_state__f_prim, v) {
    FOR1D(rvs_state__f_prim(v)->range, val) {
      rvs_state__f_prim(v)->P(t,val) = (1.-given_action_weight) * rvs_state__f_prim(v)->P(t-1,val);
    }
  }
  for(r=0; r<action2rules_no(action_id_pred, action_id_args); r++) {
    r2 = action2rules.p[p_id_start+r];
    p_id = r2 * P_v_r__f.d1 * P_v_r__f.d2;
    FOR1D(rvs_state__f_prim, v) {
      rvs_state__f_prim__v = rvs_state__f_prim(v);
      FOR1D(rvs_state__f_prim__v->range, val) {
        rvs_state__f_prim(v)->P(t,val) += P_v_r__f.p[p_id++] * rvs_rules(t-1, r2);
      }
      p_id += MAX_FUNCTION_VALUE - val;
    }
  }
#endif
#ifndef FAST
  FOR1D(rvs_state__f_prim, v) {
    FOR1D(rvs_state__f_prim(v)->range, val) {
      rvs_state__f_prim(v)->P(t,val) = 0.0;
      for(r=0; r<action2rules_no(action_id_pred, action_id_sa); r++) {
        r2 = action2rules(action_id_pred, action_id_sa, r);
        rvs_state__f_prim(v)->P(t,val) += P_v_r__f(r2,v,val) * rvs_rules(t-1, r2);
      }
      rvs_state__f_prim(v)->P(t,val) += (1.-given_action_weight) * rvs_state__f_prim(v)->P(t-1,val);
    }
  }
#endif
  
    
    
  // (5) INFER DERIVED PREDICATES
  if (DEBUG>1) {cout<<"Calc erste Runde:" << endl; writeState(t);}
  calcDerived(t);
  checkStateSoundness(t);


  if (DEBUG>1) {writeState(t);}
  if (DEBUG>0) {cout<<"inferState [END]"<<endl;}
}



void NID_DBN::inferStates(const AtomL& given_action_sequence) {
//   CHECK(given_action_sequence.N == horizon, "");
  uint t;
  FOR1D(given_action_sequence, t) {
    inferRules(t);
//     setAction(t, given_action_sequence(t));
    inferState(t+1, given_action_sequence(t));
  }
}





// ----------------------------------------------------
//                 NID_DBN:   State probability
// ----------------------------------------------------


double NID_DBN::log_probability(uint t, const State& state) const {
  uint DEBUG = 0;
  if (DEBUG>0) {
    cout<<"log_probability [START]"<<endl;
    PRINT(t);
    state.write(cout); cout<<endl;
  }
  
  uint i, val;
  
  double total_log_prob = 0.;
  double prob_variable = 0.;
  FOR1D(rvs_state__p_prim, i) {
    if (logicReasoning::holds(state, rvs_state__p_prim(i)->lit)) { // teuer, das jedes Mal wieder neu zu machen
      prob_variable = rvs_state__p_prim(i)->P(t, 1);
      if (DEBUG>1) {
        MT::String name;
        rvs_state__p_prim(i)->lit->name(name);
        printf("%-14s",(char*)name);
        cout << "  has prob=" << prob_variable << "   log-prob=" << log(prob_variable);
      }
    }
    else {
      prob_variable = rvs_state__p_prim(i)->P(t, 0);
      if (DEBUG>1) {
        MT::String name;
        rvs_state__p_prim(i)->lit->name(name);
        printf("-%-13s",(char*)name);
        cout << "  has prob=" << prob_variable << "   log-prob=" << log(prob_variable);
      }
    }
    total_log_prob += log(prob_variable);
    if (DEBUG>1) {
      cout << "  -->  total_log_prob=" << total_log_prob << endl;
    }
  }
  
  FOR1D(rvs_state__f_prim, i) {
    FunctionAtom* fi = rvs_state__f_prim(i)->fi;
    CHECK(fi->args.N == 1, "");
    double value = logicReasoning::getValue(fi->args(0), fi->f, state);
    for (val=0; val<rvs_state__f_prim(i)->dim; val++) {
      if (TL::areEqual(value, rvs_state__f_prim(i)->range(val))) {
        prob_variable = rvs_state__f_prim(i)->P(t, val);
        break;
      }
    }
    total_log_prob += log(prob_variable);
    if (DEBUG>1) {
      rvs_state__f_prim(i)->fi->write(cout);
      cout << "=" << value << "   prob=" << prob_variable << "  " << log(prob_variable) << "  --> total_log_prob=" <<  total_log_prob << endl;
    }
    CHECK(val < rvs_state__f_prim(i)->dim, "value has not been found");
  }
  
  if (DEBUG>0) {
    PRINT(total_log_prob);
    cout<<"log_probability [END]"<<endl;
  }
  
  return total_log_prob; 
}



double NID_DBN::belief_difference(uint t, const arr& probs_p_prim, const arr& probs_f_prim) {
  uint DEBUG = 0;
  uint i, val;
  double total_diff = 0.;
  double local_diff;
  if (DEBUG>0) {cout<<"+++ belief_difference at "<<t<<endl;}
  FOR1D(rvs_state__p_prim, i) {
    local_diff = 0.;
    if (DEBUG>0) {rvs_state__p_prim(i)->lit->write(cout);cout<<": ";}
    FOR1D(rvs_state__p_prim(i)->range, val) {
      local_diff += fabs(probs_p_prim(i,val) - rvs_state__p_prim(i)->P(t,val));
      if (DEBUG>0) {cout << " (" << probs_p_prim(i,val) << " - " << rvs_state__p_prim(i)->P(t,val)<<")";}
    }
    if (DEBUG>0) {cout<<"   --> "<<local_diff<<endl;}
    local_diff /= 1.0 * rvs_state__p_prim(i)->range.N;
    total_diff += local_diff;
  }
  FOR1D(rvs_state__f_prim, i) {
    local_diff = 0.;
    if (DEBUG>0) {rvs_state__f_prim(i)->fi->write(cout);cout<<": ";}
    FOR1D(rvs_state__f_prim(i)->range, val) {
      local_diff += fabs(probs_f_prim(i,val) - rvs_state__f_prim(i)->P(t,val));
      if (DEBUG>0) {cout << " (" << probs_f_prim(i,val) << " - " << rvs_state__f_prim(i)->P(t,val)<<")";}
    }
    if (DEBUG>0) {cout<<"   --> "<<local_diff<<endl;}
    local_diff /= 1.0 * rvs_state__f_prim(i)->range.N;
    total_diff += local_diff;
  }
  if (DEBUG>0) {PRINT(total_diff);}
  return total_diff;
}







// ----------------------------------------------------
//                 NID_DBN:   Helpers
// ----------------------------------------------------



void NID_DBN::checkStateSoundness(uint t, bool omit_derived) {
  uint v,val;
  FOR1D(rvs_state__p_prim, v) {
    if (!TL::areEqual(1.0, rvs_state__p_prim(v)->P(t,0)+rvs_state__p_prim(v)->P(t,1))
         ||   rvs_state__p_prim(v)->P(t,0) < -0.01   ||   rvs_state__p_prim(v)->P(t,1) < -0.01) {
      cerr << "Invalid distribution for rv ";
      rvs_state__p_prim(v)->lit->write(cerr);
      cerr << "  at t=" << t << " with id="<<rvs_state__p_prim(v)->id << endl;
      cerr << "  --> Because:  1.0  !=   P(t,0)=" << rvs_state__p_prim(v)->P(t,0) << "   +  P(t,1)=" <<rvs_state__p_prim(v)->P(t,1)<<endl;
      HALT("");
    }
  }
  double sum;
  FOR1D(rvs_state__f_prim, v) {
    sum = 0.;
    FOR1D(rvs_state__f_prim(v)->range, val) {
      sum += rvs_state__f_prim(v)->P(t,val);
      if (rvs_state__f_prim(v)->P(t,val) < -0.01) {
        cerr<<"invalid distribution for rv with id="<<rvs_state__f_prim(v)->id<<"  for value="<<val<<": "<< rvs_state__f_prim(v)->P(t,val) <<endl;
        HALT("");
      }
    }
    CHECK(TL::areEqual(1.0, sum), "invalid distribution for rv with id="<<rvs_state__f_prim(v)->id);
  }
  if (!omit_derived) {
    FOR1D(rvs_state__p_derived, v) {
      CHECK(TL::areEqual(1.0, rvs_state__p_derived(v)->P(t,0)+rvs_state__p_derived(v)->P(t,1)), "invalid distribution for rv with id="<<rvs_state__p_derived(v)->id);
    }
    FOR1D(rvs_state__f_derived, v) {
      if (rvs_state__f_derived(v)->type == RV_TYPE__FUNC_EXPECT) continue;
      sum = 0.;
      FOR1D(rvs_state__f_derived(v)->range, val) {
        sum += rvs_state__f_derived(v)->P(t,val);
      }
      CHECK(TL::areEqual(1.0, sum), "invalid distribution (sum="<<sum<<") for rv with id="<<rvs_state__p_derived(v)->id);
    }
  }
}


void calcDerived(uint t, RV_Manager* rvm, uintA& constants);

void NID_DBN::setState(const LitL& lits, const FuncVL& fvs, uint t) {
  uint DEBUG = 0;
  uint v, val;
  if (DEBUG>0) {cout<<"State: ";TL::write(lits);cout<<endl;TL::write(fvs);cout<<endl;}
  // Everything not specified is set to false!
  FOR1D(rvs_state__p_prim, v) {
    rvs_state__p_prim(v)->P(t,0) = 1.0;
    for (val=1; val<rvs_state__p_prim(v)->dim; val++) {
      rvs_state__p_prim(v)->P(t,val) = 0.0;
    }
  }
  // Incorporating evidence
  FOR1D(lits, v) {
    if (DEBUG>0) {cout<<"setState:  "; lits(v)->write(); cout<<endl;}
    TL::PredicateRV* prv = rvm->l2v(lits(v));
    if (prv == NULL) {
      MT_MSG("Omitting setting state lit "<<*lits(v)<<" in DBN^(t=0).");
      continue;
    }
    if (lits(v)->positive) {
      prv->P(t,0) = 0.0;
      prv->P(t,1) = 1.0;
    }
    else {
      prv->P(t,0) = 1.0;
      prv->P(t,1) = 0.0;
    }
  }
  FOR1D(fvs, v) {
    TL::FunctionAtom* fi = rvm->getFVW(fvs(v)->atom->f, fvs(v)->atom->args);
    if (DEBUG>0) {cout<<"setState:  "; fvs(v)->write(); cout<<endl;}
    FOR1D(fi->f->range, val) {
      if (TL::areEqual(fi->f->range(val), fvs(v)->value))
        rvm->fi2v(fi)->P(t,val) = 1.0;
      else
        rvm->fi2v(fi)->P(t,val) = 0.0;
    }
    if (DEBUG>0) {cout<<" --> "; rvm->fi2v(fi)->write(); cout<<endl;}
  }

  calcDerived(t);
  if (DEBUG>1){writeState(0);}
  checkStateSoundness(t);
}


void NID_DBN::setStateUniform(uint t) {
  uint v, val;
  FOR1D(rvs_state__p_prim, v) {
    rvs_state__p_prim(v)->P(t,0) = 0.5;
    rvs_state__p_prim(v)->P(t,1) = 0.5;
  }
  
  FOR1D(rvs_state__f_prim, v) {
    FOR1D(rvs_state__f_prim(v)->range, val) {
      rvs_state__f_prim(v)->P(t,val) = 1. / (1. * rvs_state__f_prim(v)->range.N);
    }
  }
  
  checkStateSoundness(t, true);
}


void NID_DBN::setAction(uint t, TL::Atom* action) {
  PredicateRV* action_rv = rvm->l2v(logicObjectManager::getLiteral(action->pred, true, action->args));
  uint i;
  FOR1D(rvs_action, i) {
    if (action_rv == rvs_action(i)) {
      rvs_action(i)->P(t, 0) = 0.0;
      rvs_action(i)->P(t, 1) = 1.0;
    }
    else {
      rvs_action(i)->P(t, 0) = 1.0;
      rvs_action(i)->P(t, 1) = 0.0;
    }
  }
}


void NID_DBN::getBelief(uint t, arr& beliefs_p_prim, arr& beliefs_f_prim) const {
  uint i, val;
  beliefs_p_prim.resize(rvs_state__p_prim.N, 2);
  beliefs_p_prim.setZero();
  FOR1D(rvs_state__p_prim, i) {
    FOR1D(rvs_state__p_prim(i)->range, val) {
      beliefs_p_prim(i, val) += rvs_state__p_prim(i)->P(t,val);
    }
  }
  beliefs_f_prim.resize(rvs_state__f_prim.N, MAX_FUNCTION_VALUE);
  beliefs_f_prim.setZero();
  FOR1D(rvs_state__f_prim, i) {
    FOR1D(rvs_state__f_prim(i)->range, val) {
      beliefs_f_prim(i, val) += rvs_state__f_prim(i)->P(t,val);
    }
  }
}







// ----------------------------------------------------
//                 NID_DBN:   writing
// ----------------------------------------------------


void NID_DBN::writeAllStates(bool prim_only, double threshold, ostream& out) const {
  uint t;
  for(t=0; t<=horizon; t++) {
    out<<"+++++  t=" << t << "  +++++"<<endl;
    writeState(t, prim_only, threshold, out);
  }
}


void NID_DBN::writeState(uint t, bool prim_only, double threshold, ostream& out) const {
  uint v, val;
//   TL::Predicate* p_HOMIES = logicObjectManager::getPredicate(MT::String("homies"));
//   out<<"homies supressed"<<endl;
  FOR1D(rvs_state__p_prim, v) {
//     if (rvs_state__p_prim(v)->lit->atom->pred == p_HOMIES)
//       continue;
    if (rvs_state__p_prim(v)->P(t,1) >= threshold) {
      rvs_state__p_prim(v)->lit->write(out);
      out<<"  ";
      out<<rvs_state__p_prim(v)->P(t,1);
      out<<endl;
    }
  }
  if (!prim_only) {
    FOR1D(rvs_state__p_derived, v) {
      if (rvs_state__p_derived(v)->P(t,1) >= threshold) {
        rvs_state__p_derived(v)->lit->write(out);
        out<<" ";
        out<<rvs_state__p_derived(v)->P(t,1)<<"    ";
        out<<endl;
      }
    }
  }
  FOR1D(rvs_state__f_prim, v) {
    rvs_state__f_prim(v)->fi->write(out);
    out<<"  ";
    FOR1D(rvs_state__f_prim(v)->range, val) {
      out<<rvs_state__f_prim(v)->range(val)<<":"<<rvs_state__f_prim(v)->P(t,val)<<"  ";
    }
    out<<endl;
  }
  if (!prim_only) {
    FOR1D(rvs_state__f_derived, v) {
      if (rvs_state__f_derived(v)->type == RV_TYPE__FUNC_EXPECT) {
        rvs_state__f_derived(v)->fi->write(out); out<<"  "<<rvs_state__f_derived(v)->P(t,0)<<" [E]"<<endl;
      }
      else {
        rvs_state__f_derived(v)->fi->write(out);
        out<<"  ";
        FOR1D(rvs_state__f_derived(v)->range, val) {
          out<<rvs_state__f_derived(v)->range(val)<<":"<<rvs_state__f_derived(v)->P(t,val)<<"  ";
        }
        out<<endl;
      }
    }
  }
}


void NID_DBN::writeStateSparse(uint t, bool prim_only, ostream& out) const {
  uint v, val;
  FOR1D(rvs_state__p_prim, v) {
    if (TL::areEqual(rvs_state__p_prim(v)->P(t,1), 1.0)) {
      rvs_state__p_prim(v)->lit->write(out); out<<"  ";
    }
  }
  if (!prim_only) {
    FOR1D(rvs_state__p_derived, v) {
      if (TL::areEqual(rvs_state__p_derived(v)->P(t,1), 1.0)) {
        rvs_state__p_derived(v)->lit->write(out); out<<"  ";
      }
    }
  }
  FOR1D(rvs_state__f_prim, v) {
    if (rvs_state__f_prim(v)->type == RV_TYPE__FUNC_EXPECT) {
      rvs_state__f_prim(v)->fi->write(out);
      out<<"="<<rvs_state__f_prim(v)->P(t,0)<<"  ";
    }
    else {
      FOR1D(rvs_state__f_prim(v)->range, val) {
        if (TL::areEqual(rvs_state__f_prim(v)->P(t,val), 1.0)) {
          rvs_state__f_prim(v)->fi->write(out);
          out<<"="<<rvs_state__f_prim(v)->range(val)<<"  ";
        }
      }
    }
  }
  if (!prim_only) {
    FOR1D(rvs_state__f_derived, v) {
      if (rvs_state__f_derived(v)->type == RV_TYPE__FUNC_EXPECT) {
        rvs_state__f_derived(v)->fi->write(out);
        out<<"="<<rvs_state__f_derived(v)->P(t,0)<<"  ";
      }
      else {
        FOR1D(rvs_state__f_derived(v)->range, val) {
          if (TL::areEqual(rvs_state__f_derived(v)->P(t,val), 1.0)) {
            rvs_state__f_derived(v)->fi->write(out);
            out<<"="<<rvs_state__f_derived(v)->range(val)<<"  ";
          }
        }
      }
    }
  }
  out<<endl;
}


void NID_DBN::getActions(AtomL& actions, uint horizon) const {
  actions.clear();
  uint a, t;
  CHECK(horizon <= rvs_action(0)->P.d0, "");
  for (t=0; t<horizon; t++) {
    FOR1D(rvs_action, a) {
      if (TL::isZero(rvs_action(a)->P(t,1) - 1.0)) {
        actions.append(rvs_action(a)->lit->atom);
        break;
      }
      if (rvs_action.N == a) {
        HALT("action has not been found");
      }
    }
  }
}




// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------
//    PRADA
// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------





PRADA::PRADA() : NID_Planner(0.0), num_samples(100), noise_softener(1.0)  {
  this->net = NULL;
  this->prada_reward = NULL;
  this->threshold_reward = 0.05;
  this->reward_calculation__sum = true;
  this->action_choice__max = true;
}


PRADA::~PRADA() {
  if (net!=NULL) delete net;
  if (prada_reward!=NULL) delete prada_reward;
}



// ----------------------------------------------------
//                 PRADA:   DBN construction
// ----------------------------------------------------

// The DBN used by (A-)PRADA consists of grounded predicate and 
// function instances which are used 
//       (i)  in the (ground) rules and/or 
//       (ii) in the reward description.
void PRADA::calc_dbn_concepts() {
  uint DEBUG = 0;
  if (DEBUG>0) {cout<<"PRADA::calc_dbn_concepts [START]"<<endl;}
  CHECK(reward != NULL, "Reward has to be set first");
  CHECK(ground_rules.num() > 0, "Ground rules have to be set first");
  if (DEBUG>0) 
  
  dbn_preds.clear();
  dbn_funcs.clear();
  uint i, k, l;
  // (i-a) rule concepts - primitive  (only predicates; functions are covered below with comparison predicates) 
  FOR1D_(ground_rules, i) {
    FOR1D(ground_rules.elem(i)->context, k) {
      if (ground_rules.elem(i)->context(k)->atom->pred->category == category_primitive
          &&   ground_rules.elem(i)->context(k)->atom->pred->type == TL::Predicate::predicate_simple) {
        dbn_preds.setAppend(ground_rules.elem(i)->context(k)->atom->pred);
      }
    }
    FOR1D(ground_rules.elem(i)->outcomes, k) {
      FOR1D(ground_rules.elem(i)->outcomes(k), l) {
        if (ground_rules.elem(i)->outcomes(k)(l)->atom->pred->category == category_primitive
          &&   ground_rules.elem(i)->outcomes(k)(l)->atom->pred->type == TL::Predicate::predicate_simple) {
          dbn_preds.setAppend(ground_rules.elem(i)->outcomes(k)(l)->atom->pred);
        }
      }
    }
  }
  // (i-b) rule concepts - derived and comparison (--> function)
  FOR1D_(ground_rules, i) {
    FOR1D(ground_rules.elem(i)->context, k) {
      if (ground_rules.elem(i)->context(k)->atom->pred->category == category_derived) {
        dbn_preds.setAppend(ground_rules.elem(i)->context(k)->atom->pred);
        // get also precessors
        PredL pre_preds;
        FuncL pre_funcs;
        logicObjectManager::dependencyGraph.getAllPrecessors(*ground_rules.elem(i)->context(k)->atom->pred, pre_preds, pre_funcs);
        dbn_preds.setAppend(pre_preds);
        dbn_funcs.setAppend(pre_funcs);
      }
      else if (ground_rules.elem(i)->context(k)->atom->pred->type == TL::Predicate::predicate_comparison) {
        TL::ComparisonLiteral* clit = (TL::ComparisonLiteral*) ground_rules.elem(i)->context(k);
        dbn_funcs.setAppend(((ComparisonAtom*)clit->atom)->fa1->f);
        if (((ComparisonAtom*)clit->atom)->fa1->f->category == category_derived) {
          // get also precessors
          PredL pre_preds;
          FuncL pre_funcs;
          logicObjectManager::dependencyGraph.getAllPrecessors(*((ComparisonAtom*)clit->atom)->fa1->f, pre_preds, pre_funcs);
          dbn_preds.setAppend(pre_preds);
          dbn_funcs.setAppend(pre_funcs);
        }
      }
    }
  }
  // (ii) reward concepts
  if (reward->reward_type == TL::Reward::reward_literal) {
    PredL pre_preds;
    FuncL pre_funcs;
    logicObjectManager::getAllPrecessors(*((TL::LiteralReward*) reward)->lit->atom->pred, pre_preds, pre_funcs);
    dbn_preds.setAppend(((TL::LiteralReward*) reward)->lit->atom->pred);
    dbn_preds.setAppend(pre_preds);
    dbn_funcs.setAppend(pre_funcs);
  }
  else if (reward->reward_type == TL::Reward::reward_literalList) {
    FOR1D(((TL::LiteralListReward*) reward)->lits, k) {
      PredL pre_preds;
      FuncL pre_funcs;
      logicObjectManager::getAllPrecessors(*((TL::LiteralListReward*) reward)->lits(k)->atom->pred, pre_preds, pre_funcs);
      dbn_preds.setAppend(((TL::LiteralListReward*) reward)->lits(k)->atom->pred);
      dbn_preds.setAppend(pre_preds);
      dbn_funcs.setAppend(pre_funcs);
    }
  }
  else if (reward->reward_type == TL::Reward::reward_maximize_function) {
    TL::MaximizeFunctionReward* mfr = (TL::MaximizeFunctionReward*) reward;
    if (mfr->fa != NULL) {
      PredL pre_preds;
      FuncL pre_funcs;
      logicObjectManager::getAllPrecessors(*mfr->fa->f, pre_preds, pre_funcs);
      dbn_preds.setAppend(pre_preds);
      dbn_funcs.setAppend(pre_funcs);
      dbn_funcs.setAppend(mfr->fa->f);
    }
    if (DEBUG>0) {cout<<"Adding concepts for MaximizeFunctionReward"<<endl; PRINT(mfr->important_literals);}
    FOR1D(mfr->important_literals, k) {
      PredL pre_preds;
      FuncL pre_funcs;
      logicObjectManager::getAllPrecessors(*mfr->important_literals(k)->atom->pred, pre_preds, pre_funcs);
      dbn_preds.setAppend(mfr->important_literals(k)->atom->pred);
      dbn_preds.setAppend(pre_preds);
      dbn_funcs.setAppend(pre_funcs);
    }
  }
  else {
    NIY;
  }
  if (DEBUG>0) {
    cout<<"(A-) PRADA's DBN will be built with nodes for the following concepts:"<<endl;
    TL::writeNice(dbn_preds);
    TL::writeNice(dbn_funcs);
    TL::writeNice(logicObjectManager::p_actions);
  }
  if (DEBUG>0) {cout<<"PRADA::calc_dbn_concepts [END]"<<endl;}
}


void PRADA::build_dbn(const uintA& constants) {
  calc_dbn_concepts();
  build_dbn(constants, dbn_preds, dbn_funcs, logicObjectManager::p_actions);
}

void PRADA::build_dbn(const uintA& constants, const PredL& preds, const FuncL& funcs, const PredL& actions) {
  net = new NID_DBN(constants, preds, funcs, actions, ground_rules, noise_softener, horizon);
}










// ----------------------------------------------------
//                 PRADA:   Basic Helpers
// ----------------------------------------------------


void PRADA::setState(const TL::State& s, uint t) {
  uint DEBUG = 0;
  if (DEBUG>0) {cout<<"PRADA::setState [START]"<<endl;}
  // Construct DBN if necessary
  if (net == NULL) {
    if (DEBUG>0) {cout<<"Constructing net."<<endl;}
    uintA constants;
    logicReasoning::getConstants(s, constants);
    build_dbn(logicObjectManager::constants);
  }
  if (DEBUG>0) {cout<<"Creating lits_prim_filtered and fv_prim_filtered."<<endl;}
  
  // Set
  LitL lits_prim_filtered;
  uint i;
  FOR1D(s.lits_prim, i) {
    if (dbn_preds.findValue(s.lits_prim(i)->atom->pred) >= 0)
      lits_prim_filtered.append(s.lits_prim(i));
  }
  FuncVL fv_prim_filtered;
  FOR1D(s.fv_prim, i) {
    if (dbn_funcs.findValue(s.fv_prim(i)->atom->f) >= 0)
      fv_prim_filtered.append(s.fv_prim(i));
  }
  
  if (DEBUG>0) {cout<<"Setting in net."<<endl;}
  net->setState(lits_prim_filtered, fv_prim_filtered, t);
  if (DEBUG>0) {cout<<"PRADA::setState [END]"<<endl;}
}





// TODO can be removed some day; is only for visualization
void calcBloxworld_homies_2(MT::Array< uintA >& gangs, const State& s) {
  Predicate* p_HOMIES = logicObjectManager::getPredicate(MT::String("homies"));
  gangs.clear();
  uint i, k;
  boolA constants_covered(logicObjectManager::constants.N);
  constants_covered.setUni(false);
  FOR1D(logicObjectManager::constants, i) {
    if (logicReasoning::holds_straight(logicObjectManager::constants(i), MT::String("table"), s))
      continue;
    if (constants_covered(i))
      continue;
    uintA homies;
    logicReasoning::getRelatedObjects(homies, logicObjectManager::constants(i), true, *p_HOMIES, s);
    homies.insert(0, logicObjectManager::constants(i));
    constants_covered(i) = true;
    FOR1D(homies, k) {
      constants_covered(logicObjectManager::constants.findValue(homies(k))) = true;
    }
    gangs.append(homies);
  }
}

void calcBloxworld_piles_2(MT::Array< uintA >& piles, const State& s) {
  Predicate* p_ON = logicObjectManager::getPredicate(MT::String("on"));
  bool inserted;
  uint i, k;
  FOR1D(s.lits_prim, i) {
    // on(A,B)
    if (s.lits_prim(i)->atom->pred == p_ON) {
      inserted = false;
      FOR1D(piles, k) {
        // pile with [A, ..., top]  -->  put B below
        if (piles(k)(0) == s.lits_prim(i)->atom->args(0)) {
          piles(k).insert(0, s.lits_prim(i)->atom->args(1));
          inserted = true;
        }
        // pile with [tablastBlock, ..., B]  -->  put A on top
        else if (piles(k).last() == s.lits_prim(i)->atom->args(1)) {
          piles(k).append(s.lits_prim(i)->atom->args(0));
          inserted = true;
        }
      }
      if (!inserted) {
        uintA newPile;
        newPile.append(s.lits_prim(i)->atom->args(1));
        newPile.append(s.lits_prim(i)->atom->args(0));
        piles.append(newPile);
      }
    }
  }
}


void PRADA::setStartState(const TL::State& s1) {
  setState(s1, 0);
}


void PRADA::setReward(Reward* reward) {
//   MT_MSG("Automatic conversion of reward!");
  this->reward = reward;
  LiteralReward* pg = dynamic_cast<LiteralReward*>(reward);
  if (pg!= NULL) {
    this->prada_reward = convert_reward((LiteralReward*) reward);
  }
  else {
    LiteralListReward* plg = dynamic_cast<LiteralListReward*>(reward);
    if (plg!= NULL) {
      this->prada_reward = convert_reward((LiteralListReward*) reward);
    }
    else {
      MaximizeFunctionReward* fg = dynamic_cast<MaximizeFunctionReward*>(reward);
      if (fg!= NULL) {
        this->prada_reward = convert_reward((MaximizeFunctionReward*) reward);
      }
      else {
        NotTheseStatesReward* ntsg = dynamic_cast<NotTheseStatesReward*>(reward);
        if (ntsg!= NULL)
          this->prada_reward = convert_reward((NotTheseStatesReward*) reward);
        else {
          DisjunctionReward* dg = dynamic_cast<DisjunctionReward*>(reward);
          if (dg!= NULL)
            this->prada_reward = convert_reward((DisjunctionReward*) reward);
          else
            NIY;
        }
      }
    }
  }
}

void PRADA::setReward(Reward* reward, PRADA_Reward* prada_reward) {
  this->reward = reward;
  this->prada_reward = prada_reward;
}


void PRADA::setNumberOfSamples(uint num_samples) {
  this->num_samples = num_samples;
}


void PRADA::setThresholdReward(double threshold_reward) {
  this->threshold_reward = threshold_reward;
}


void PRADA::setNoiseSoftener(double noise_softener) {
  if (noise_softener > 1.0 || noise_softener < 0.) {
    MT_MSG("noise_softener has to be in [0,1]");
  }
  else
    this->noise_softener = noise_softener;
}

void PRADA::setRewardCalculation(bool reward_calculation__sum) {
  this->reward_calculation__sum = reward_calculation__sum;
}

void PRADA::setActionChoice(bool action_choice__max) {
  this->action_choice__max = action_choice__max;
}



void PRADA::writeState(uint t, ostream& out) {
  net->writeState(t, out);
}

void PRADA::writeStateSparse(uint t, bool prim_only, ostream& out) {
  net->writeStateSparse(t, prim_only, out);
}




// ----------------------------------------------------
//                 PRADA:   Inference
// ----------------------------------------------------


void PRADA::infer(const AtomL& plan) {
  AtomL sampled_actions;
  sampleActionsAndInfer(sampled_actions, plan, net, plan.N);
}

void PRADA::sampleActionsAndInfer(AtomL& sampled_actions) {
  AtomL fixed_actions(horizon);
  fixed_actions.setUni(NULL);
  sampleActionsAndInfer(sampled_actions, fixed_actions);
}

void PRADA::sampleActionsAndInfer(AtomL& sampled_actions, const AtomL& fixed_actions) {
  sampleActionsAndInfer(sampled_actions, fixed_actions, net, horizon);
}

void PRADA::sampleActionsAndInfer(AtomL& sampled_actions, const AtomL& fixed_actions1, NID_DBN* local_net, uint local_horizon) {
  uint DEBUG = 0;
  if (DEBUG>0) {cout<<"sampleActionsAndInfer [START]"<<endl;}
  AtomL fixed_actions;  fixed_actions = fixed_actions1;
  if (DEBUG>0) {cout<<"fixed_actions: " << fixed_actions <<endl;}
  if (fixed_actions.N < local_horizon) {
    if (DEBUG>>0) {cout<<"fixed_actions have wrong number " << fixed_actions.N << " instead of " << local_horizon << " and will be refilled"<<endl;}
    do {
      fixed_actions.append(NULL);
    } while (fixed_actions.N < local_horizon);
  }
  bool TIMING = false;
  clock_t start,finish;
  double time;
  sampled_actions.clear();
  uint A = ground_actions.N;
  arr action_weights(local_horizon, A);
  uint t, r, a;
  for (t=0; t<local_horizon; t++) {
    if (DEBUG>1) {cout<<"+++++ TIME-STEP "<<t<<" +++++"<<endl;}
    if (DEBUG>1) {local_net->writeState(t, false, 0.05);}
    
    // (1) INFER rule random variables at t
    if (TIMING) {start = clock();}
    local_net->inferRules(t);
    if (TIMING) {
      finish = clock();
      time = (double(finish)-double(start))/CLOCKS_PER_SEC;
      cerr<<"inferPhis"<<time<<endl;
    }
    
    // (2) INFER ACTION-WEIGHT at t,  cf. paper Eq. (24)
    if (TIMING) {start = clock();}
    arr action_weights__sampling(ground_actions.N);  // --> need this for sampling distribution which shall ignore non-manipulating rules
    FOR1D(ground_actions, a) {
      action_weights(t,a) = 0.0;
      action_weights__sampling(a) = 0.0;
      uint id_pred = logicObjectManager::p_actions.findValue(ground_actions(a)->pred);
      uint id_args = getIndex(logicObjectManager::constants, ground_actions(a)->args);
      for (r=0; r<local_net->action2rules_no(id_pred, id_args); r++) {
        action_weights(t,a) += local_net->rvs_rules(t, local_net->action2rules(id_pred, id_args, r));
        if (logicObjectManager::getAtom_doNothing() == ground_actions(a)  ||  is_manipulating_rule(local_net->action2rules(id_pred, id_args, r)))  // only consider manipulating rules
          action_weights__sampling(a) += local_net->rvs_rules(t, local_net->action2rules(id_pred, id_args, r));
      }
      // Manually forbid doNothing-action at first time-step if free choice 
      if (t==0  &&  logicObjectManager::getAtom_doNothing() == ground_actions(a)  &&  fixed_actions(t) == NULL) {
        action_weights(t,a) = 0.0;
        action_weights__sampling(a) = 0.0;
      }
    }
    if (DEBUG>2) {
      cout<<"action_weights__sampling (action applicability):";
      for(a=0; a<A; a++) {
        cout<<"  "<<action_weights__sampling(a);
      }
      cout<<endl;
      cout<<"APPLICABLE ACTIONS:   ";
      for(a=0; a<A; a++) {
        if (action_weights(t,a)>0) {
          cout<<"   "<<action_weights__sampling(a)<<" ("<<action_weights(t,a)<<") ";
          ground_actions(a)->write();
        }
      }
      cout<<endl;
    }
    if (TL::isZero(sum(action_weights__sampling))) {
//       MT_MSG("No more action applicabso we stop!");
      break;
    }
    arr P_action = action_weights__sampling / sum(action_weights__sampling);
        
    // (3) SELECT ACTION
    int sampled_action_id;
    TL::Atom* sampled_action;
    // (3a) take fixed action
    if (fixed_actions(t) != NULL) {
      sampled_action_id = ground_actions.findValue(fixed_actions(t));
      sampled_action = fixed_actions(t);
      if (sampled_action_id < 0) {
        HALT("fixed_actions(t)="<<*fixed_actions(t)<<"  has not been found in ground_actions "<<ground_actions);
      }
      if (DEBUG>1) {cout<<"*** SIMULATED ACTION (fixed):   ";  sampled_action->write(cout);cout<<endl;}
    }
    // (3b) or sample action according to sampling distribution
    else {
      sampled_action_id = TL::basic_sample(P_action);
      sampled_action = ground_actions(sampled_action_id);
      if (DEBUG>1) {cout<<"*** SIMULATED ACTION (sampled):   ";  sampled_action->write(cout);cout<<endl;}
    }
    sampled_actions.append(sampled_action);
    FOR1D(local_net->rvs_action, a) {
      if (local_net->rvs_action(a)->lit->atom == sampled_action) {
        local_net->rvs_action(a)->P(t,0) = 0.;
        local_net->rvs_action(a)->P(t,1) = 1.;
      }
      else {
        local_net->rvs_action(a)->P(t,0) = 1.;
        local_net->rvs_action(a)->P(t,1) = 0.;
      }
    }
    if (DEBUG>2) {
      cout<<"P_action:  ";
      uint i;
      FOR1D(P_action, i) {
        if (P_action(i) > 0.) {
          cout<<P_action(i)<< " ";  ground_actions(i)->write();  cout<<"   ";
        }
      }
      cout<<endl;
      PRINT(sampled_action_id);
      if (fixed_actions(t) != NULL) {cout<<"Fixed ";}
      else {cout<<"Sampled ";}
      cout<<"action: ";sampled_action->write();cout<<endl;
//       cerr<<"Action: ";sampled_action->write(cerr);cerr<<endl;
    }
    if (TIMING) {
      finish = clock();
      time = (double(finish)-double(start))/CLOCKS_PER_SEC;
      cerr<<"TIME calcaction_weights and sample action "<<time<<endl;
    }
    
    
    // (4) INFER STATE t+1   (--> propagate action effects)
    if (TIMING) {start = clock();}
    local_net->inferState(t+1, sampled_action, action_weights(t,sampled_action_id));
    if (TIMING) {
      finish = clock();
      time = (double(finish)-double(start))/CLOCKS_PER_SEC;
      cerr<<"TIME infer state t+1 "<<time<<endl;
    }
  }

  if (DEBUG>1) {cout<<"+++++ TIME-STEP "<<t<<" +++++ (sans action)"<<endl;}
  if (DEBUG>1) {local_net->writeState(t, false, 0.05);}
  if (DEBUG>0) {cout<<"sampleActionsAndInfer [END]"<<endl;}
}









// ----------------------------------------------------
//                 PRADA:   Planning
// ----------------------------------------------------


void my_handmade_plan(AtomL& plan) {
//   NIY;
  logicObjectManager::getAtoms(plan, "grab(71) puton(70)");
//   logicObjectManager::getAtoms(plan, "grab_puton(72 71) grab_puton(66 72)");
#if 0
  uint i;
  TL::Predicate* p_MOVE = logicObjectManager::getPredicate(MT::String("move"));
  uintA args(3);
  FOR1D(logicObjectManager::constants, i) {
    if (i < 2) continue;
    args(0) = logicObjectManager::constants(i);
    args(1) = 60;
    args(2) = logicObjectManager::constants(i-1);
    plan.append(logicObjectManager::getLiteral(p_MOVE, true, args));
  }
#endif
  
  // search-and-rescue
//   char* geiler_plan = "takeoff(11) takeoff(14) goto(12) explore(12) land(12) takeoff(12) goto(11) explore(11) land(11) end-mission()";
  // Triangle-tireworld
//   char* geiler_plan = "move-car(11 14) changetire() move-car(14 17) loadtire(17) changetire() move-car(17 15) loadtire(15) changetire() move-car(15 13)";
  // Ex-Blocksworld 1
//   char* geiler_plan = "pick-up(13 12) put-down(13) pick-up(11 14) put-on-block(11 13) pick-up(14 15) put-down(14) pick-up-from-table(12) put-on-block(12 14)";
  // Ex-Blocksworld 5
//   char* geiler_plan = "pick-up(15 13) put-on-block(15 17) pick-up(13 14) put-on-block(13 16) pick-up(15 17) put-on-block(15 13)";
//   logicObjectManager::getLiterals(plan, geiler_plan);
//   cout<<endl<<endl<<endl<<"USING HAND-MADE PLAN!!"<<endl<<endl<<endl;
//   MT_MSG("USING HAND-MADE PLAN!");
}


bool PRADA::plan(AtomL& best_plan, double& bestValue, uint num_samples) {
  uint DEBUG = 1;
  if (DEBUG>0) {cout<<"PRADA::plan [START]"<<endl;}
  
  uint t;

  if (DEBUG>1) {
    PRINT(num_samples);
    PRINT(threshold_reward);
    cout<<"STARTING STATE:  "<<endl;
    net->writeState(0, true, 0.9);
    cout<<endl;
    cout<<"REWARD:  ";  reward->writeNice();  cout<<endl;
  }
  
//   FOR1D_(ground_rules, t) {
//     if (ground_rules.elem(t)->action == logicObjectManager::getLiteral("grab(69)"))
//       ground_rules.elem(t)->writeNice();
//   }
 
  // doNothing-value [START]
  uint v, val;
  for (t=1; t<=horizon; t++) {
    FOR1D(net->rvs_state__p_prim, v) {
      for (val=0; val<net->rvs_state__p_prim(v)->dim; val++) {
        net->rvs_state__p_prim(v)->P(t,val) = net->rvs_state__p_prim(v)->P(0,val);
      }
    }
    FOR1D(net->rvs_state__p_derived, v) {
      for (val=0; val<net->rvs_state__p_derived(v)->dim; val++) {
        net->rvs_state__p_derived(v)->P(t,val) = net->rvs_state__p_derived(v)->P(0,val);
      }
    }
    FOR1D(net->rvs_state__f_prim, v) {
      for (val=0; val<net->rvs_state__f_prim(v)->dim; val++) {
        net->rvs_state__f_prim(v)->P(t,val) = net->rvs_state__f_prim(v)->P(0,val);
      }
    }
    FOR1D(net->rvs_state__f_derived, v) {
      for (val=0; val<net->rvs_state__f_derived(v)->dim; val++) {
        net->rvs_state__f_derived(v)->P(t,val) = net->rvs_state__f_derived(v)->P(0,val);
      }
    }
    // Rules -- need to be inferred first!!!  (depending on reward!)
    net->inferRules(0);
    if (t<net->rvs_rules_simple.d0) {
      for (v=0; v<net->rvs_rules_simple.d1; v++) {
        net->rvs_rules_simple(t,v) = net->rvs_rules_simple(0,v);
        net->rvs_rules(t,v) = net->rvs_rules(0,v);
      }
    }
  }
  double do_nothing_reward = inferStateRewards();
  if (DEBUG>0) {cout<<"Calculated do_nothing_reward="<<do_nothing_reward<<endl;}
  // doNothing-value [END]
 
 
 
  action_choices.resize(horizon, ground_actions.N);
  action_choices.setUni(0);

  uint s;
  MT::Array< AtomL > plans;
  arr reward_values;
  arr action_values;
  arr values;
//   MT::Array< arr > reward_probs(num_samples);
  
  any_sensible_action = false;
  bool good_old_plan__is_good = false;
  for(s=0; s<num_samples; s++) {
    if (DEBUG>2) cout <<"--- SAMPLING ROUND "<<s<<"  (/" << num_samples << ") ---"<<endl;
    if (s%10==0) cerr <<"."<<std::flush;
    AtomL sampled_actions;
    
    if (good_old_plans.N > 0 && s==0) {
      AtomL old_inspiration_plan = good_old_plans(0);
      if (DEBUG>0) {cout<<"Candidate good old (complete) - s==0:  "<<old_inspiration_plan<<endl;}
      old_inspiration_plan.memMove = true;
      FOR1D_DOWN(old_inspiration_plan, t) {
        if (ground_actions.findValue(old_inspiration_plan(t)) < 0) {
          old_inspiration_plan.remove(t);
        }
      }
      if (DEBUG>0) {cout<<"Retrying filtered good old (complete) - s==0:  "<<old_inspiration_plan<<endl;}
      sampleActionsAndInfer(sampled_actions, old_inspiration_plan);
    }
    else if (good_old_plans.N > 0 && s==1) {
      uint q;
      AtomL old_inspiration_plan;
      FOR1D(good_old_plans(0), q) {
        if (q==0) continue;
        if (ground_actions.findValue(good_old_plans(0)(q)) < 0) continue;
        old_inspiration_plan.append(good_old_plans(0)(q));
      }
      if (DEBUG>0) {cout<<"Retrying good old (kill action 0) - s==1:  "<<old_inspiration_plan<<endl;}
      sampleActionsAndInfer(sampled_actions, old_inspiration_plan);
    }
    else if (good_old_plans.N > 0 && s==2) {
      uint q;
      AtomL old_inspiration_plan;
      FOR1D(good_old_plans(0), q) {
        if (q==1) continue;
        if (ground_actions.findValue(good_old_plans(0)(q)) < 0) continue;
        old_inspiration_plan.append(good_old_plans(0)(q));
      }
      if (DEBUG>0) {cout<<"Retrying good old (kill action 1) - s==2:  "<<old_inspiration_plan<<endl;}
      sampleActionsAndInfer(sampled_actions, old_inspiration_plan);
    }
    else if (good_old_plans.N > 0 && s==3) {
      uint q;
      AtomL old_inspiration_plan;
      FOR1D(good_old_plans(0), q) {
        if (q==0 || q==1) continue;
        if (ground_actions.findValue(good_old_plans(0)(q)) < 0) continue;
        old_inspiration_plan.append(good_old_plans(0)(q));
      }
      if (DEBUG>0) {cout<<"Retrying good old (kill actions 0 and 1) - s==3:  "<<old_inspiration_plan<<endl;}
      sampleActionsAndInfer(sampled_actions, old_inspiration_plan);
    }
    else sampleActionsAndInfer(sampled_actions);
    
#if 0
    if (s==0) {
      AtomL hand_plan;
      my_handmade_plan(hand_plan);
      // ensure that all actions are possible
      uint q;
//       FOR1D(hand_plan, q) {
//         if (net->rvm->l2v(hand_plan(q)) == NULL)
//           HALT("handmade plan undoable / not recognized");
//       }
      while (hand_plan.N < horizon) {
        hand_plan.append(NULL);
      }
      cout<<"Hand-made plan:  ";  TL::write(hand_plan);  cout<<endl;
      sampleActionsAndInfer(sampled_actions, hand_plan, net, horizon);
      cout<<"---> Reward = " << inferStateRewards(hand_plan.N) << endl;
      exit(0);
    }
#endif
    
    plans.append(sampled_actions);
    if (DEBUG>2) {cout<<"Sampled actions: ";TL::write(sampled_actions); cout<<endl;}
    // bis zur ersten NULL evaluieren...
    FOR1D(sampled_actions, t) {
      if (sampled_actions(t) == NULL) {
        break;
      }
    }
    reward_values.append(inferStateRewards(t));  // t ist hier horizont.
    action_values.append(0.);
    if (use_ruleOutcome_rewards) {
      action_values(s) = calcRuleRewards(sampled_actions);
    }
    values.append(reward_values(s) + action_values(s));
    // statistics
    FOR1D(sampled_actions, t) {
      uint action_id = ground_actions.findValue(sampled_actions(t));
      action_choices(t, action_id)++;
    }
    
    CHECK(plans.N == reward_values.N   &&  plans.N == action_values.N  &&  plans.N == values.N, "");
    
    if (DEBUG>2) {cout<<"--> values(s)="<<values(s)<<"  (reward_values(s)="<<reward_values(s)<<",  action_values(s)="<<action_values(s)<<")"<<endl;}
    
    
    if (!use_ruleOutcome_rewards  &&  reward->reward_type != TL::Reward::reward_maximize_function) {
//       if (any_sensible_action  &&  s>300) {
//         if (DEBUG>0) {cout<<"Great plan already found (s="<<s<<")!"<<endl;}
//         break;
//       }
    }
    else if (!use_ruleOutcome_rewards  &&  reward->reward_type == TL::Reward::reward_maximize_function) {
      if (values(s) > do_nothing_reward + threshold_reward  &&  s<4) {
        good_old_plan__is_good = true;
      }
      else if (good_old_plans.N > 0  &&  values(s) > do_nothing_reward + threshold_reward  &&  s>200  &&  good_old_plan__is_good) {
//       else if (good_old_plans.N > 0  &&  values(s) > do_nothing_reward + threshold_reward  &&  s>500  &&  good_old_plan__is_good) {
// MT_MSG("STACK version!");
        if (DEBUG>0) {cout<<"Great good old plan (s="<<s<<")!"<<endl;}
        break;
      }
      else if (values(s) > do_nothing_reward + threshold_reward  &&  s>800) {
        if (DEBUG>0) {cout<<"Great plan already found (s="<<s<<")!"<<endl;}
        break;
      }
    }
  }
  if (DEBUG>0) {cerr<<s<<" samples taken."<<endl;}

  int max_id = values.maxIndex();
  double best_value = values(max_id);
  TL::Atom* max_action = NULL;
  if (plans(max_id).N > 0)
   max_action = plans(max_id)(0);
  if (DEBUG>0) {
    if (DEBUG>2) {
      cout<<"Sample values:  "<<endl;
      FOR1D(plans, s) {
        cout<<"("<<s<<") "<<values(s)<<":  ";TL::write(plans(s));cout<<endl;
      }
    }
    arr sorted_values;
    uintA sortedIndices;
    sort_desc(sorted_values, sortedIndices, values);
    cout<<"All plans sorted (horizon="<< horizon << "):"<<endl;
    FOR1D(sortedIndices, s) {
      if (DEBUG<3 && s>10) break;
      printf("#%4u",s);
      cout<<":   ";
      printf("(%4u)", sortedIndices(s));
      cout<< "  ";
      printf("%5.3f", values(sortedIndices(s)));
      printf(" (%5.3f + %5.3f)", reward_values(sortedIndices(s)), action_values(sortedIndices(s)));
      cout<< "  ";
      TL::write(plans(sortedIndices(s)));
      cout<<endl;
    }
    
    PRINT(max_id);
    if (!any_sensible_action) {cout<<"NO SENSIBLE ACTION WAS FOUND!"<<endl;}
    cout<<"Max action:  ";
    if (max_action != NULL) {
      max_action->write();cout<<"  v="<<values(max_id)<<""<<endl;
    }
    else
      cout<<" -"<<endl;
    cout<<"Max sequence: ("<<max_id<<") ";TL::write(plans(max_id));cout<<endl;
    reward->writeNice();cout<<endl;
  }
  
   // statistics
  if ((DEBUG>0 && ground_actions.N < 20)
       || DEBUG>1 ) {
//   if (true) {
    cout<<"Action application statistics:"<<endl;
    printf("%-14s","Time");
    for(t=0;t<horizon;t++) printf("%5u",t);
    cout<<endl;
    uint i;
    FOR1D(ground_actions, i) {
      uint number = 0;
      for(t=0;t<action_choices.d0;t++) {
        number += action_choices(t,i);
      }
      if (number == 0)
        continue;
      MT::String name;
      ground_actions(i)->name(name);
      printf("%-14s",(char*)name);
      for(t=0;t<action_choices.d0;t++) printf("%5u", action_choices(t,i));
      cout<<endl;
    }
  }
  
  // ACTION CHOICE
  // (1)  Choose maximizing action sequence
  if (action_choice__max) {
    best_plan = plans(max_id);
    bestValue = best_value;
  }
  // (2)  Choose action with maximal normalized sum
  else {
    MT::Array< arr > action_values__all(ground_actions.N);
    FOR1D(plans, s) {
      if (plans(s)(0) != NULL)
        action_values__all(ground_actions.findValue(plans(s)(0))).append(values(s));
    }
    uint a;
    arr action_values(ground_actions.N);
    FOR1D(ground_actions, a) {
      if (action_values__all(a).N > 0) {
        #if 0
        action_values(a) = sum(action_values__all(a));
        action_values(a) /= 1.0 * action_values__all(a).N;
        #else
        // taking only X best into account
        uint X = 10;
        uint q;
        arr sorted;
        uintA sortedIndices;
        sort_desc(sorted, sortedIndices, action_values__all(a));
        for (q=0; q<TL_MIN(sorted.N, X); q++) {
          action_values(a) += sorted(q);
        }
        action_values(a) /= 1.0 * TL_MIN(sorted.N, X);
        #endif
        
      }
      else
        action_values(a) = -10000.;
    }
    
    if (DEBUG>0) {
      FOR1D(ground_actions, a) {
        MT::String name;
        ground_actions(a)->name(name);
        printf("%-14s",(char*)name);
        cout << "  " << action_values(a) <<  "  "<<action_values__all(a).N;
        cout<<action_values__all(a)<<endl;
      }
    }
    
    uint max_id2 = action_values.maxIndex();
    bestValue = action_values(max_id2);
    // Random choice of plan: just use first plan with the best action
    FOR1D(plans, s) {
      if (plans(s)(0) == ground_actions(max_id2)) {
        best_plan = plans(s);
        break;
      }
    }
  }


  // Return value: depends on type of reward
  if (use_ruleOutcome_rewards) {
    if (DEBUG>0) {cout<<"PRADA::plan [END]"<<endl;}
    return true;
  }
  else if (reward->reward_type == TL::Reward::reward_maximize_function) {
    if (DEBUG>0) {
      PRINT(bestValue);  PRINT(do_nothing_reward);  PRINT(threshold_reward);  PRINT(do_nothing_reward+threshold_reward);
      PRINT((bestValue > do_nothing_reward + threshold_reward));
      cout<<"Returning-1 "<<(bestValue > do_nothing_reward + threshold_reward)<<endl;
    }
    if (DEBUG>0) {cout<<"PRADA::plan [END]"<<endl;}
    return (bestValue > do_nothing_reward + threshold_reward);
  }
  else {
    if (DEBUG>0) cout<<"Returning02 "<<any_sensible_action<<endl;
    if (DEBUG>0) {cout<<"PRADA::plan [END]"<<endl;}
    return any_sensible_action;
  }
}





void PRADA::generatePlan(AtomL& generated_plan, double& value, const TL::State& s, uint max_runs) {
  uint DEBUG = 2;
  if (DEBUG>0) {cout<<"PRADA::generatePlan [START]"<<endl;}
  if (DEBUG>0) {
    cout<<"Reward:  ";  if (reward != NULL) reward->writeNice();  else cout<<"NULL";  cout<<endl;
    PRINT(num_samples);
  }
  
  // Check 1 whether planning makes sense at all:  are there changing concepts?
  uintA changingIds_p, changingIds_f;
  ruleReasoning::changingConcepts(changingIds_p, changingIds_f, ground_rules);
  if (changingIds_p.N == 0  &&  changingIds_f.N == 0) {
    value = -100000.;
    generated_plan.clear();
    if (DEBUG>0) {cout<<"Planning does not make sense here: no changing concepts"<<endl;}
    if (DEBUG>0) {cout<<"PRADA::generatePlan [END]"<<endl;}
    return;
  }
  
  if (DEBUG>0) {cout<<"State:  ";  s.write();  cout<<endl;  PRINT(max_runs);  cout<<"Trying to set state..."<<endl;}
  setStartState(s);
  if (DEBUG>0) {cout<<"State has been set."<<endl;}
  
  // Check 2 whether planning makes sense at all:  are reward concepts among random variables?
  if (reward != NULL) {
    LitL lits_reward;
    LiteralReward* pg = dynamic_cast<LiteralReward*>(reward);
    if (pg!= NULL) {
      lits_reward.append(pg->lit);
    }
    else {
      LiteralListReward* plg = dynamic_cast<LiteralListReward*>(reward);
      if (plg!= NULL) {
        lits_reward.append(plg->lits);
      }
    }
    if (DEBUG>0) {cout<<"lits_reward:  ";  TL::write(lits_reward);  cout<<endl;}
    uint i;
    FOR1D(lits_reward, i) {
      PredicateRV* var = net->rvm->l2v(lits_reward(i));
      if (var == NULL) {
        value = -100000.;
        generated_plan.clear();
        if (DEBUG>0) {cout<<"Planning does not make sense here: reward concept " << *lits_reward(i) << " not among changing concepts"<<endl;}
        if (DEBUG>0) {cout<<"PRADA::generatePlan [END]"<<endl;}
        return;
      }
    }
  }
  
  
  uint run = 0;
  AtomL run_plan;
  double run_value = 0.0;
  do {
    bool improve = plan(run_plan, run_value, num_samples);
    if (improve) {
      generated_plan = run_plan;
      value = run_value;
      break;
    }
    cerr<<endl<<"PRADA run "<< (run+1) << " of "<<max_runs<<": no good sample found."<<endl;
  } while (++run < max_runs);
  
  if (run == max_runs) {
    cerr<<"PRADA: Can't find a good plan, sire!"<<endl;
    cout<<"PRADA: Can't find a good plan, sire!"<<endl;
    value = -100000.;
    generated_plan.clear();
  }
  if (DEBUG>0) {cout<<"PRADA::generatePlan [END]"<<endl;}
}



TL::Atom* PRADA::generateAction(const TL::State& s, uint max_runs) {
  AtomL plan;
  double value;
  generatePlan(plan, value, s, max_runs);
  if (plan.N == 0) {
    return NULL;
  }
  else
    return plan(0);
}



















// ----------------------------------------------------
//                 PRADA:   Rewards
// ----------------------------------------------------


double PRADA::inferStateRewards() {
  return inferStateRewards(horizon);
}

double PRADA::inferStateRewards(uint depth) {
  if (reward_calculation__sum) {
    return inferStateRewards_limited_sum(depth);
  }
  else {
    return inferStateRewards_limited_max(depth);
  }
}

double PRADA::inferStateRewards_limited_max(uint depth) {
  uint DEBUG = 0;
  if (DEBUG>0) {cout<<"inferStateRewards_limited_max [START]"<<endl;}
  uint t;
  if (DEBUG>0) {
    reward->writeNice();cout<<endl;
  }
  double value_t, value_0, value_max=-50.;
  
  for (t=0; t<=depth; t++) {
//     if (t>=T) break;
    value_t = discount_pow(t) * inferStateRewards_single(t);
    if (t==0) value_0 = value_t;
    else {
      if (value_t - value_0 > threshold_reward) {
        any_sensible_action = true;
      }
    }
    if (value_t > value_max)
      value_max = value_t;
    if (DEBUG>1) {cout<<"t="<<t<<":  "<<value_t<<endl;}
  }
  
  if (DEBUG>0) {
    PRINT(value_0);
    PRINT(any_sensible_action);
  }
  if (DEBUG>0) {cout<<"--> value="<<value_max<<endl;}
  if (DEBUG>0) {cout<<"inferStateRewards_limited_max [END]"<<endl;}
  return value_max;
}

double PRADA::inferStateRewards_limited_sum(uint depth) {
  uint DEBUG = 0;
  if (DEBUG>0) {cout<<"inferStateRewards_limited_sum [START]"<<endl;}
  uint t;
  if (DEBUG>1) {
    reward->writeNice();cout<<endl;
    PRINT(threshold_reward);
    PRINT(any_sensible_action);
  }
  double value_t, value_0=0.0, value_total=0.;
  arr debug_values;
  for (t=0; t<=depth; t++) {
//     if (t>=T) break;
    value_t = inferStateRewards_single(t);
    if (t==0) value_0 = value_t;
    else {
      if (value_t - value_0 > threshold_reward)
        any_sensible_action = true;
    }
    value_total += discount_pow(t) * value_t;
    debug_values.append(value_t);
    if (DEBUG>1) {cout<<"t="<<t<<":  "<<value_t<<endl;}
  }
  
  if (DEBUG>0) {
    AtomL actions;
    net->getActions(actions, depth);
    TL::write(actions);  cout<<endl;
    PRINT(debug_values);
    PRINT(value_0);
    PRINT(any_sensible_action);
  }
  if (DEBUG>0) {cout<<"--> value="<<value_total<<endl;}
  if (DEBUG>0) {cout<<"inferStateRewards_limited_sum [END]"<<endl;}
  return value_total;
}


double PRADA::inferStateRewards_single(uint t) {
  return prada_reward->evaluate_prada_reward(*net, t);
}





double PRADA::calcRuleRewards(const AtomL& actions) {
  uint DEBUG = 0;
  if (DEBUG>0) {cout<<"calcRuleRewards [START]"<<endl;}
  double total_reward = 0.;
  uint t, r;
  FOR1D(actions, t) {
    if (DEBUG>0) {cout<<"+++++  t="<<t<<"  ";  actions(t)->write();  cout<<endl;}
    uint id_pred = logicObjectManager::p_actions.findValue(actions(t)->pred);
    uint id_args = getIndex(logicObjectManager::constants, actions(t)->args);
    double t_reward = 0.;
    for (r=0; r<net->action2rules_no(id_pred, id_args); r++) {
      uint rule_number = net->action2rules(id_pred, id_args, r);
      TL::Rule* rule = ground_rules.elem(rule_number);
      double expected_reward = this->expected_rule_rewards(rule_number);
      double probability = net->rvs_rules(t, rule_number);
      t_reward += discount_pow(t) * probability * expected_reward;
      if (DEBUG>1) {
        if (net->rvs_rules(t, rule_number) > 0.01) {
          cout<<"Rule #"<<r<<endl;
          rule->write(cout);
          cout<<" expected_reward="<<expected_reward << "   *   probability="<<probability<<"    *   discount="<<discount_pow(t)<<endl;
          cout<<" -> Brings "<< (discount_pow(t) * probability * expected_reward) << endl;
        }
      }
    }
    if (DEBUG>0) {cout<<"==> t_reward="<<t_reward<<endl;}
    total_reward += t_reward;
  }
  if (DEBUG>0) {PRINT(total_reward);}
  if (DEBUG>0) {cout<<"calcRuleRewards [END]"<<endl;}
  return total_reward;
}





// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------
//    A-PRADA
// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------


A_PRADA::A_PRADA() : PRADA() {
}

A_PRADA::~A_PRADA() {
}


double A_PRADA::shorten_plan(AtomL& seq_best, const AtomL& seq_old, double value_old) {
  uint DEBUG = 0;
  if (DEBUG>0) {cout<<"A_PRADA::shorten_plan [START]"<<endl;}
  if (DEBUG>0) {
    cout<<"seq_old: ";TL::write(seq_old);cout<<endl;
    PRINT(value_old);
  }
  if (seq_old.N == 0) {
    seq_best.clear();
    if (DEBUG>0) {cout<<"No sensible input sequence"<<endl;  cout<<"A_PRADA::shortenSeq [END]"<<endl;}
    return -5999999999.;
  }
  double value_best = value_old;
  double value = 0.;
  AtomL seq_new;
  seq_best = seq_old;
  uint DEBUG_TOTAL_R=0;
  // always leave out t
  uint t=0, t2;
  while (t<horizon-1 && t<seq_best.N && seq_best.N > 1) {
//   int t=T-2, t2;
//   while (t>=0) {
//     PRINT(DEBUG_TOTAL_R);
    if (DEBUG_TOTAL_R++>1000) {
      cerr<<"too many round"<<endl;
      break;
    }
    if (DEBUG>1) {cout<<"*** t="<<t<<" ***"<<endl;}
    if (DEBUG>1) {cout<<"seq_best:   ";TL::write(seq_best);cout<<endl;}
    #if 0
    // A-PRADA samplet auch noch die letzten Sequenzen
    AtomL seq_fixed(horizon);
    seq_fixed.setUni(NULL);
    for (t2=0; t2<seq_best.N; t2++) {
      if (t2 < t)
        seq_fixed(t2) = seq_best(t2);
      else if (t2 > t)
        seq_fixed(t2-1) = seq_best(t2);
    }
    if (DEBUG>2) {cout<<"t="<<t<<"  --  seq_fixed:  ";TL::writeNice(seq_fixed);cout<<endl;}
    //     sampleActionsAndInfer(seq_new, seq_fixed, net, horizon)
    if (DEBUG>2) {cout<<" seq_fixed + sampled:  ";TL::writeNice(seq_new);cout<<endl;}
    #else
    AtomL seq_new;
    for (t2=0; t2<seq_best.N; t2++) {
      if (t2 < t)
        seq_new.append(seq_best(t2));
      else if (t2 > t)
        seq_new.append(seq_best(t2));
    }
    infer(seq_new);
    if (DEBUG>1) {cout<<"kill:    "; seq_best(t)->write(cout); cout<<endl<<"seq_new:    ";TL::write(seq_new);cout<<endl;}
    #endif
    // bis zur ersten NULL evaluieren...
    FOR1D(seq_new, t2) {
      if (seq_new(t2) == NULL)
        break;
    }
    value = inferStateRewards(t2-1);
    double ruleOutcome_rewards_sum = 0.;
    if (use_ruleOutcome_rewards) {
      ruleOutcome_rewards_sum = calcRuleRewards(seq_new);
      value += ruleOutcome_rewards_sum;
    }
    if (DEBUG>1) {cout<<"Shortened value = "<<value<<"  (use_ruleOutcome_rewards="<<use_ruleOutcome_rewards<<", ruleOutcome_rewards_sum="<<ruleOutcome_rewards_sum<<")"<<endl;}
    cout<<std::flush;
    if (value > value_best) {
      if (DEBUG>1) {cout<<"ACCEPTED."<<endl;}
      value_best = value;
      seq_best = seq_new;
    }
    else {
      if (DEBUG>1) {cout<<"Rejected."<<endl;}
      t++;
    }
  }
  if (DEBUG>0) {
    cout<<"Result:"<<endl;
    cout<<"seq_best: ";TL::write(seq_best);cout<<endl;
    if (seq_best.N > 0   &&  seq_best(0) != seq_old(0)) cout<<"  --> Shortened!"<<endl;
    PRINT(value_best);
  }
  if (DEBUG>0) {cout<<"A_PRADA::shorten_plan [END]"<<endl;}
  return value_best;
}


TL::Atom* A_PRADA::generateAction(const TL::State& s, uint max_runs) {
  uint DEBUG = 0;
  if (DEBUG>0) {cout<<"A_PRADA::generateAction [START]"<<endl;}
  setStartState(s);
  
  // (1) take into account last best sequence: NOT IMPLEMENTED YET
//   if (last_seq.N > 0) {
//   uint i;
//     AtomL last_seq_without_first;
//     FOR1D(last_seq, i) {
//       if (i>0)
//         last_seq_without_first.append(last_seq(i));
//     }
//     last_seq_without_first.append(NULL);
//     AtomL last_seq_without_first__full;
//     DEBUG_SINGLE_SAMPLE = 0; // 100
//     infer(last_seq_without_first__full); //  fehlt ne aktion
//     DEBUG_SINGLE_SAMPLE = 0;
//     double v_last_new = inferStateRewards();
//     if (DEBUG>1) {
//       cout<<"Checking last sequence: ";
//       cout<<"Last:  ";TL::writeNice(last_seq);cout<<endl;
//       PRINT(last_value);
//       cout<<"Last minus first:  ";TL::writeNice(last_seq_without_first__full);cout<<endl;
//       PRINT(v_last_new);
//     }
//     double ESTIMATED_NOISE_GAIN = 0.03;
//     // take only if last action has sensible effects
//     // hacked: +1000
//     if (v_last_new >=  last_value / discount + 1000  &&  v_last_new >= horizon * ESTIMATED_NOISE_GAIN) {
//       if (DEBUG>1) {cout<<" --> taken!"<<endl;}
//       last_seq = last_seq_without_first__full;
//       last_value = v_last_new;
//       return last_seq_without_first__full(0);
//     }
//   }
  
  // (2) do usual PRADA planning and thereafter try to shorten returned plan
  uint i;
  AtomL plan_short;
  for (i=0; i<3; i++) {
    AtomL plan;
    double plan_value;
    generatePlan(plan, plan_value, s, max_runs);
    if (DEBUG>1) {cout<<"Best seq found: ";TL::write(plan);cout<<endl;}
    AtomL plan_short__local;
    last_value = shorten_plan(plan_short__local, plan, plan_value);
    if (DEBUG>1) {cout<<"plan_short__local (value="<<last_value<<"): ";TL::write(plan_short__local);cout<<endl;}
    if (plan_short__local.N > 0 
      && ruleReasoning::uniqueCoveringRule_groundedRules_groundedAction(this->ground_rules, s, plan_short__local(0)) == 0) {
      MT_MSG("stupid action which we can't use");
      if (DEBUG>1) {cout<<"stupid action which we can't use"<<endl;}
    }
    else if (plan_short__local.N > 0 
      && ruleReasoning::uniqueCoveringRule_groundedRules_groundedAction(this->ground_rules, s, plan_short__local(0)) != 0) {
      plan_short = plan_short__local;
      last_seq = plan_short;
      break;
    }
    if (i==2) {// nur fuer den Abbruch
      plan_short = plan_short__local;
      last_seq = plan_short;
      break;
    }
  }

  if (DEBUG>1) {
    cout<<"Best seq shortened: ";TL::write(last_seq);cout<<endl;
    PRINT(last_value);
  }
  if (DEBUG>0) {cout<<"A_PRADA::generateAction [END]"<<endl;}
  if (plan_short.N > 0)
    return plan_short(0);
  else
    return NULL;
}


void A_PRADA::reset() {
  last_seq.clear();
  last_value = -5000.;
}









// -----------------------------------------------------------------
// -----------------------------------------------------------------
// -----------------------------------------------------------------
//         PRADA_Reward
// -----------------------------------------------------------------
// -----------------------------------------------------------------
// -----------------------------------------------------------------


class PRADA_Reward_PT : public PRADA_Reward {
  TL::Literal* lit;
  TL::LogicRV* rv;
  uint value;
  
  public:
    PRADA_Reward_PT(TL::Literal* lit) {
      this->lit = lit;
      rv = NULL;
      if (lit->positive)
        value = 1;
      else
        value = 0;
    }
    
    double evaluate_prada_reward(const NID_DBN& net, uint t) {
      if (rv == NULL) {
        rv = net.rvm->l2v(lit);
      }
      return rv->P(t,value);
    }
};


class PRADA_Reward_PTL : public PRADA_Reward {
  LitL lits;
  PredVA rvs;
  
  public:
    PRADA_Reward_PTL(LitL& lits) {
      this->lits = lits;
      rvs.resize(this->lits.N);
      rvs.setUni(NULL);
    }
    
    double evaluate_prada_reward(const NID_DBN& net, uint t) {
      uint i;
      if (rvs(0) == NULL) {
        FOR1D(lits, i) {
          rvs(i) = net.rvm->l2v(lits(i));
        }
      }
      double prob = 1.0;
      FOR1D(rvs, i) {
        if (lits(i)->positive)
          prob *= rvs(i)->P(t,1);
        else
          prob *= rvs(i)->P(t,0);
      }
      return prob;
    }
};


class PRADA_Reward_Disjunction : public PRADA_Reward {
  LitL lits;
  PredVA rvs;
  arr weights;
  
  public:
    PRADA_Reward_Disjunction(LitL& lits, arr& weights) {
      this->lits = lits;
      rvs.resize(this->lits.N);
      rvs.setUni(NULL);
      this->weights = weights;
    }
    
    double evaluate_prada_reward(const NID_DBN& net, uint t) {
      uint i;
      if (rvs(0) == NULL) {
        FOR1D(lits, i) {
          rvs(i) = net.rvm->l2v(lits(i));
        }
      }
      double max_prob = 0.0;
      FOR1D(rvs, i) {
        if (lits(i)->positive)
          max_prob = weights(i) * TL_MAX(max_prob, rvs(i)->P(t,1));
        else
          max_prob = weights(i) * TL_MAX(max_prob, rvs(i)->P(t,0));
      }
      return max_prob;
    }
};


class PRADA_Reward_FVW : public PRADA_Reward {
  TL::FunctionAtom* fi;
  TL::FunctionRV* rv;
  
  public:
    PRADA_Reward_FVW(TL::FunctionAtom* fi) {
      this->fi = fi;
      rv = NULL;
    }
    
    double evaluate_prada_reward(const NID_DBN& net, uint t) {
      if (rv == NULL) {
        rv = net.rvm->fi2v(fi);
      }
      return rv->P(t,0);
    }
};


class PRADA_Reward_S : public PRADA_Reward {
  StateL undesired_states;
  double penalty;
  
  public:
    PRADA_Reward_S(StateL& _undesired_states) {
      undesired_states = _undesired_states;
      penalty = -1.;
    }
    
    double evaluate_prada_reward(const NID_DBN& net, uint t) {
      uint DEBUG = 0;
      double value = 10.;
      uint i;
      if (DEBUG>0) {cout<<"=== t="<<t<<" ===="<<endl;   net.writeState(t, true, 0.1);}
      FOR1D(undesired_states, i) {
        if (DEBUG>0) {cout<<"-----"<<endl; undesired_states(i)->write(cout); cout<<endl;}
        double state_log_probability = net.log_probability(t, *undesired_states(i));
        if (DEBUG>0) {PRINT(state_log_probability);  PRINT(exp(state_log_probability));  PRINT(penalty * exp(state_log_probability));}
        value += penalty * exp(state_log_probability);
      }
      if (DEBUG>0) {cout<<"-->  ";  PRINT(value);}
      return value;
    }
};




PRADA_Reward* PRADA::convert_reward(TL::LiteralReward* reward) {
  return new PRADA_Reward_PT(reward->lit);
}


PRADA_Reward* PRADA::convert_reward(TL::LiteralListReward* reward) {
  return new PRADA_Reward_PTL(reward->lits);
}


PRADA_Reward* PRADA::convert_reward(TL::MaximizeFunctionReward* reward) {
  return new PRADA_Reward_FVW(reward->fa);
}

PRADA_Reward* PRADA::convert_reward(TL::NotTheseStatesReward* reward) {
  return new PRADA_Reward_S(reward->undesired_states);
}

PRADA_Reward* PRADA::convert_reward(TL::DisjunctionReward* reward) {
  return new PRADA_Reward_Disjunction(reward->lits, reward->weights);
}


PRADA_Reward* PRADA::create_PRADA_Reward(TL::Reward* reward) {
  switch(reward->reward_type) {
    case TL::Reward::reward_literal: return convert_reward((TL::LiteralReward*) reward);
    case TL::Reward::reward_literalList: return convert_reward((TL::LiteralListReward*) reward);
    case TL::Reward::reward_maximize_function: return convert_reward((TL::MaximizeFunctionReward*) reward);
    case TL::Reward::reward_not_these_states: return convert_reward((TL::NotTheseStatesReward*) reward);
    case TL::Reward::reward_one_of_literal_list: return convert_reward((TL::DisjunctionReward*) reward);    
    default: NIY;
  }
}



// -----------------------------------------------------------------
// -----------------------------------------------------------------
// -----------------------------------------------------------------
//         calcDerived
// -----------------------------------------------------------------
// -----------------------------------------------------------------
// -----------------------------------------------------------------


void calcDerived1(TL::ConjunctionPredicate* p, uint t, RV_Manager* rvm, uintA& constants);
void calcDerived1(TL::TransClosurePredicate* p, uint t, RV_Manager* rvm, uintA& constants);
void calcDerived1(TL::CountFunction* f, uint t, RV_Manager* rvm, uintA& constants);
void calcDerived1(TL::SumFunction* f, uint t, RV_Manager* rvm, uintA& constants);
void calcDerived1(TL::RewardFunction* f, uint t, RV_Manager* rvm, uintA& constants);




void NID_DBN::calcDerived(uint t) {
  uintA order;
  boolA isPredicate;
  logicObjectManager::dependencyGraph.getWellDefinedOrder(order, isPredicate, true);
  uint i;
  FOR1D(order, i) {
    if (isPredicate(i)) {
      TL::Predicate* pred = logicObjectManager::dependencyGraph.getPredicate(order(i));
      CHECK(pred->category == category_derived, "something wrong with dependency graph");
      if (rvm->preds.findValue(pred) < 0)
        continue;
      if (pred->type == TL::Predicate::predicate_conjunction) {
        calcDerived1((TL::ConjunctionPredicate*) pred, t, rvm, objects);
      }
      else if (pred->type == TL::Predicate::predicate_transClosure) {
        calcDerived1((TL::TransClosurePredicate*) pred, t, rvm, objects);
      }
      else
        NIY;
    }
    else {
      TL::Function* f = logicObjectManager::dependencyGraph.getFunction(order(i));
      CHECK(f->category == category_derived, "something wrong with dependency graph");
      if (rvm->funcs.findValue(f) < 0)
        continue;
      if (f->type == TL::Function::function_count) {
        calcDerived1((TL::CountFunction*) f, t, rvm, objects);
      }
      else if (f->type == TL::Function::function_sum) {
        calcDerived1((TL::SumFunction*) f, t, rvm, objects);
      }
      else if (f->type == TL::Function::function_reward) {
        calcDerived1((TL::RewardFunction*) f, t, rvm, objects);
      }
      else
        NIY;
    }
  }
}


// void PRADA::calcDerived(uint t, RV_Manager* rvm, uintA& constants) {
//   uint i;
//   FOR1D(rvm->preds, i) {
//     if (rvm->preds(i)->category == category_primitive)
//       continue;
//     else if (rvm->preds(i)->type == predicate_simple_COMPOUND) {
//       calcDerived1((TL::ConjunctionPredicate*) rvm->preds(i), t, rvm, constants);
//     }
//     else if (rvm->preds(i)->type == predicate_transClosure) {
//       calcDerived1((TL::TransClosurePredicate*) rvm->preds(i), t, rvm, constants);
//     }
//     else
//       NIY;
//   }
//   FOR1D(rvm->funcs, i) {
//     if (rvm->funcs(i)->category == category_primitive)
//       continue;
//     else if (rvm->funcs(i)->type == function_count) {
//       calcDerived1((TL::CountFunction*) rvm->funcs(i), t, rvm, constants);
//     }
//     else if (rvm->funcs(i)->type == function_sum) {
//       calcDerived1((TL::SumFunction*) rvm->funcs(i), t, rvm, constants);
//     }
//     else
//       NIY;
//   }
// }




// With free vars!
void calcDerived1(TL::ConjunctionPredicate* p, uint t, RV_Manager* rvm, uintA& constants) {
  uint DEBUG = 0;
  if (DEBUG>0) {cout<<"calcDerived [START]"<<endl;}
  if (DEBUG>0) {
    p->writeNice(cout);cout<<endl;
  }
  uintA freeVars;
  p->getFreeVars(freeVars);
//   CHECK(freeVars.N > 0, "No free var - scp should have been dissolved before!");
  uint i,j,k;
  
  // (1) create abstract base predicate tuples = BPT
  LitL base_lits_abstract;
  k=0;
  FOR1D(p->basePreds, i) {
    uintA args1;
    for (j=0; j<p->basePreds(i)->d; j++) {
      args1.append(p->basePreds_mapVars2conjunction(k++));
    }
    TL::Literal* lit = logicObjectManager::getLiteral(p->basePreds(i), p->basePreds_positive(i), args1);
    base_lits_abstract.append(lit);
  }
  logicReasoning::sort(base_lits_abstract);
  if (DEBUG>0) {cout<<"Abstract base predicate tuples ";TL::write(base_lits_abstract); cout<<endl;}
  
  // (2) check whether grounded BPTs hold
  MT::Array< uintA > combos_args;
  TL::allPossibleLists(combos_args, constants, p->d, true, true);
  uint c1, c2;
  double prob;
  
  uintA sa(p->d);
  FOR1D(combos_args, c1) {
    TL::Literal* target = logicObjectManager::getLiteral(p, true, combos_args(c1));
    if (DEBUG>1) {cout<<"Target: ";target->write();cout<<endl;}
    // Free Vars EXISTENTIAL
    // P(p) = 1 - PRODUCT[over free-var combos c](1 - P(basePTs[sub=c]))
    // Intuition: Predicate true if not all base-pred combinations are false.
    if (!p->freeVarsAllQuantified) {
      MT::Array< uintA > combos_freevars;
      uintA constants_freevars = constants;
      setMinus(constants_freevars, combos_args(c1));
      TL::allPossibleLists(combos_freevars, constants_freevars, freeVars.N, false, true);
      arr combo_probs(combos_freevars.N);
      FOR1D(combos_freevars, c2) {
        TL::Substitution sub;
        for(i=0;i<p->d;i++) {
          sub.addSubs(i, combos_args(c1)(i));
        }
        FOR1D(freeVars, i) {
          sub.addSubs(freeVars(i), combos_freevars(c2)(i));
        }
        LitL base_lits_ground;
        logicReasoning::applyOriginalSub(sub, base_lits_abstract, base_lits_ground);
        prob = 1.0;
        FOR1D(base_lits_ground, i) {
//           base_lits_ground(i)->writeNice();  cout<<endl;
          if (base_lits_ground(i)->positive)
            prob *= rvm->l2v(base_lits_ground(i))->P(t,1); // assume binary variables
          else
            prob *= rvm->l2v(base_lits_ground(i))->P(t,0);
        }
        combo_probs(c2) = prob;
      }
      prob = 1.0;
      FOR1D(combo_probs, c2) {
        prob *= 1-combo_probs(c2);
      }
      prob = 1.-prob;
    }
    // Free Vars ALL
    // Intuition: Predicate true if all base-pred combinations are true.
    else {
      TL::Substitution sub;
      for(i=0;i<p->d;i++) {
        sub.addSubs(i, combos_args(c1)(i));
      }
      LitL base_lits_ground;
      FOR1D(base_lits_abstract, i) {
        if (numberSharedElements(base_lits_abstract(i)->atom->args, freeVars)==0) {
          base_lits_ground.append(logicReasoning::applyOriginalSub(sub, base_lits_abstract(i)));
        }
      }
      uintA constants_freevars = constants;
      setMinus(constants_freevars, combos_args(c1));
      MT::Array< uintA > combos_freevars;
      TL::allPossibleLists(combos_freevars, constants_freevars, freeVars.N, false, true);
      FOR1D(combos_freevars, c2) {
        TL::Substitution sub2;
        sub2 = sub;
        FOR1D(freeVars, i) {
          sub2.addSubs(freeVars(i), combos_freevars(c2)(i));
        }
        FOR1D(base_lits_abstract, i) {
          if (numberSharedElements(base_lits_abstract(i)->atom->args, freeVars)>0) {
            base_lits_ground.append(logicReasoning::applyOriginalSub(sub2, base_lits_abstract(i)));
          }
        }
      }
      if (DEBUG>1) {cout<<"Grounded base PTs:  ";TL::write(base_lits_ground);cout<<endl;}
      prob = 1.0;
      FOR1D(base_lits_ground, i) {
        if (base_lits_ground(i)->positive)
          prob *= rvm->l2v(base_lits_ground(i))->P(t,1);
        else
          prob *= rvm->l2v(base_lits_ground(i))->P(t,0);
      }
    }
    rvm->l2v(target)->P(t,0) = 1.-prob;
    rvm->l2v(target)->P(t,1) = prob;
    if (DEBUG>0) {cout<<" ---> "<<prob<<endl;}
  }
  if (DEBUG>0) {cout<<"calcDerived [END]"<<endl;}
}


double getPredicateProb(TL::Predicate* p, uintA& sa, uint t, RV_Manager* rvm) {
  TL::Literal* lit = logicObjectManager::getLiteral(p, true, sa);
  return rvm->l2v(lit)->P(t,1);
}


#if 0
void calcDerived_tcp_dfs(arr& probs, uintA& remaining_constants, std::map<uint, double>& probs_last, double prev_prob, uint prev_constant, TL::Predicate& base_pred, uint t, RV_Manager* rvm, uintA& constants) {
  uint DEBUG = 0;
  uint i;
  double prob_final, prob_single;
  uintA sa(2);
  sa(0)=prev_constant;
  FOR1D(remaining_constants, i) {
    sa(1)=remaining_constants(i);
    prob_single = getPredicateProb(&base_pred, sa, t, rvm);
    if (DEBUG>0) {PRINT(sa);PRINT(prob_single);PRINT(prev_prob);PRINT(probs_last[remaining_constants(i)]);}
    prob_final = prob_single * prev_prob * probs_last[remaining_constants(i)];
    probs.append(prob_final);
    double prev_prob_new = prob_single * prev_prob;
    if (prev_prob_new > TRANS_CLOSURE_STOP_PROB) {
      uintA remaining_constants2 = remaining_constants;
      remaining_constants2.removeValue(remaining_constants(i));
      calcDerived_tcp_dfs(probs, remaining_constants2, probs_last, prev_prob_new, remaining_constants(i), base_pred, t, rvm, constants);
    }
    else {
    //       if (DEBUG>0) {cerr<<"  give-up "<<(constants.N - remaining_constants.N)/*<<endl*/;}
    }
  }
}

void calcDerived(TL::TransClosurePredicate& p, uint t, RV_Manager* rvm, uintA& constants) {
  uint DEBUG = 0;
  if (DEBUG>0) {cout<<"calcDerived - TransClosurePredicate [START]"<<endl;}
  if (DEBUG>0) {
    p->writeNice(cout);cout<<endl;
  }
  uint i,c;
  CHECK(p->d==2, "TransClosurePredicate has to be 2dim");
  uintA sa(2);
  MT::Array< uintA > combos;
  double prob;
  TL::allPossibleLists(combos, constants, p->d, true, true);
  FOR1D(combos, c) {
    if (DEBUG>0) {cout<<"-- "<<combos(c)<<endl;}
    arr probs;
    uintA c_small = constants;
    setMinus(c_small, combos(c));
    // A-B
    sa(0) = combos(c)(0);
    sa(1) = combos(c)(1);
    prob = getPredicateProb(p->basePred, sa, t, rvm);
    probs.append(prob);
    if (combos(c)(0)!=combos(c)(1)) {
      // A-...X...-B
      std::map<uint, double> probs_last;
      FOR1D(c_small, i) {
        sa(0) = c_small(i);
        probs_last[c_small(i)] = getPredicateProb(p->basePred, sa, t, rvm);
      }
      calcDerived_tcp_dfs(probs, c_small, probs_last, 1.0, combos(c)(0), *p->basePred, t, rvm, constants);
    }
    sa(0) = combos(c)(0);
    TL::Literal* target = logicObjectManager::getLiteral(&p, true, sa);
    // true if not all possible chains are false
    prob = 1.;
    FOR1D(probs, i) {
      prob *= (1-probs(i));
    }
    prob = 1 - prob;
    if (DEBUG>0) {
      target->writeNice();PRINT(probs);
    }
    rvm->l2v(target)->P(t,0) = 1.-prob;
    rvm->l2v(target)->P(t,1) = prob;
  }
  if (DEBUG>0) {cout<<"calcDerived - TransClosurePredicate [END]"<<endl;}
}
#else
// FAST version
void calcDerived_tcp_dfs(arr& probs, uintA& remaining_constants, arr& probs_last, double prev_prob, uint prev_constant, uintA& constants, arr& PROBS_TABLE) {
  uint DEBUG = 0;
  uint i;
  double prob_final, prob_single;
  int idx1, idx2;
  idx1 = constants.findValue(prev_constant);
  FOR1D(remaining_constants, i) {
    idx2 = constants.findValue(remaining_constants(i));
    prob_single = PROBS_TABLE(idx1, idx2);
    if (DEBUG>0) {PRINT(prev_constant);PRINT(remaining_constants(i));PRINT(prob_single);PRINT(prev_prob);PRINT(probs_last(remaining_constants(i)));}
    prob_final = prob_single * prev_prob * probs_last(remaining_constants(i));
    probs.append(prob_final);
    double prev_prob_new = prob_single * prev_prob;
    if (prev_prob_new > TRANS_CLOSURE_STOP_PROB) {
      uintA remaining_constants2 = remaining_constants;
      remaining_constants2.removeValue(remaining_constants(i));
      calcDerived_tcp_dfs(probs, remaining_constants2, probs_last, prev_prob_new, remaining_constants(i), constants, PROBS_TABLE);
    }
    else {
//       if (DEBUG>0) {cerr<<"  give-up "<<(constants.N - remaining_constants.N)/*<<endl*/;}
    }
  }
}

void calcDerived1(TL::TransClosurePredicate* p, uint t, RV_Manager* rvm, uintA& constants) {
  uint DEBUG = 0;
  if (DEBUG>0) {cout<<"calcDerived - TransClosurePredicate [START]"<<endl;}
  if (DEBUG>0) {
    p->writeNice(cout);cout<<endl;
    PRINT(constants);
  }
  uint i,c;
  CHECK(p->d==2, "TransClosurePredicate has to be 2dim");
  MT::Array< uintA > combos;
  double prob;
  TL::allPossibleLists(combos, constants, p->d, true, true);

  // precompute all probs
  arr PROBS_TABLE(constants.N, constants.N);
  PROBS_TABLE.setUni(0.);
  int idx1, idx2;
  uintA sa(2);
  FOR1D(combos, c) {
    idx1 = constants.findValue(combos(c)(0));
    idx2 = constants.findValue(combos(c)(1));
    sa(0) = combos(c)(0);
    sa(1) = combos(c)(1);
    PROBS_TABLE(idx1, idx2) = getPredicateProb(p->basePred, sa, t, rvm);
  }
  if (DEBUG>0) {PRINT(PROBS_TABLE);}

  FOR1D(combos, c) {
    if (DEBUG>0) {cout<<"-- "<<combos(c)<<endl;}
    arr probs;
    uintA c_small = constants;
    setMinus(c_small, combos(c));
    // A-B
    idx1 = constants.findValue(combos(c)(0));
    idx2 = constants.findValue(combos(c)(1));
    probs.append(PROBS_TABLE(idx1, idx2));
    sa(0) = combos(c)(1);
    sa(1) = combos(c)(1);
    if (combos(c)(0)!=combos(c)(1)) {
      // A-...X...-B
      arr probs_last(c_small.max()+1);
      FOR1D(c_small, i) {
        idx1 = constants.findValue(c_small(i));
        probs_last(c_small(i)) = PROBS_TABLE(idx1, idx2);
      }
      calcDerived_tcp_dfs(probs, c_small, probs_last, 1.0, combos(c)(0), constants, PROBS_TABLE);
    }
    sa(0) = combos(c)(0);
    TL::Literal* target = logicObjectManager::getLiteral(p, true, sa);
    // true if not all possible chains are false
    prob = 1.;
    FOR1D(probs, i) {
      prob *= (1-probs(i));
    }
    prob = 1 - prob;
    if (DEBUG>0) {
      target->write();PRINT(probs);
    }
    rvm->l2v(target)->P(t,0) = 1.-prob;
    rvm->l2v(target)->P(t,1) = prob;
  }
  if (DEBUG>1) {
    FOR1D(combos, c) {
      sa(0) = combos(c)(0);
      sa(1) = combos(c)(1);
      TL::Literal* target = logicObjectManager::getLiteral(p, true, sa);
      cout<<sa<<" :"<<rvm->l2v(target)->P(t,1)<<endl;
    }
  }
  if (DEBUG>0) {cout<<"calcDerived - TransClosurePredicate [END]"<<endl;}
}
#endif




void calcDerived1(TL::CountFunction* cf, uint t, RV_Manager* rvm, uintA& constants) {
  uint DEBUG = 0;
  if (DEBUG>0) {cout<<"calcDerived - CountFunction [START]"<<endl;}
  if (DEBUG>0) {
    cf->writeNice(cout);cout<<endl;
    PRINT(t);
  }
  CHECK(cf->countedPred->type != TL::Predicate::predicate_comparison, "NIY for comparison predicates");
  uint i, c, c2, v, val;
  MT::Array< uintA > combos;
  TL::allPossibleLists(combos, constants, cf->d, true, true);
  uintA sa_base(cf->countedPred->d);
  double expect, prob_total, prob_single;
  TL::FunctionAtom* fi;
  TL::Literal* pt_inner;
  uintA freeVars;
  FOR1D(cf->countedPred_mapVars2derived, i) {
    if (cf->countedPred_mapVars2derived(i) >= cf->d)
      freeVars.setAppend(cf->countedPred_mapVars2derived(i));
  }
  FOR1D(combos, c) {
    if (DEBUG>0) {cout<<"-- "<<combos(c)<<endl;}
    fi = logicObjectManager::getFA(cf, combos(c));
    LogicRV* var = rvm->fi2v(fi);
    FOR1D(cf->countedPred_mapVars2derived, i) {
      if (cf->countedPred_mapVars2derived(i) < cf->d)
        sa_base(i) = combos(c)(cf->countedPred_mapVars2derived(i));
    }
    uintA local_constants = constants;
    setMinus(local_constants, combos(c));
    MT::Array< uintA > inner_combos;
    TL::allPossibleLists(inner_combos, local_constants, freeVars.N, true, true);
    // (1) Expectation
    if (var->type == RV_TYPE__FUNC_EXPECT) {
      expect = 0.0;
      FOR1D(inner_combos, c2) {
        FOR1D(cf->countedPred_mapVars2derived, i) {
          if (cf->countedPred_mapVars2derived(i) >= cf->d)
            sa_base(i) = inner_combos(c2)(cf->countedPred_mapVars2derived(i) - cf->d);
        }
        pt_inner = logicObjectManager::getLiteral(cf->countedPred, true, sa_base);
        expect += rvm->l2v(pt_inner)->P(t,1);
        if (DEBUG>2) {
          cout<<"inner combo: "<<inner_combos(c2)<<"  ";
          pt_inner->write();
          cout<<" "<<rvm->l2v(pt_inner)->P(t,1)<<endl;
        }
      }
      if (DEBUG>1) {PRINT(expect);}
      var->P(t,0) = expect;
    }
    // or (2) True Distribution
    else {
      // find all base pred rvs
      PredVA rvs_base;
      FOR1D(inner_combos, c2) {
        FOR1D(cf->countedPred_mapVars2derived, i) {
          if (cf->countedPred_mapVars2derived(i) >= cf->d)
            sa_base(i) = inner_combos(c2)(cf->countedPred_mapVars2derived(i) - cf->d);
        }
        pt_inner = logicObjectManager::getLiteral(cf->countedPred, true, sa_base);
        rvs_base.append((PredicateRV*) rvm->l2v(pt_inner));
        if (DEBUG>2) {
          cout<<"inner combo: "<<inner_combos(c2)<<"  ";
          pt_inner->write();
          cout<<"  "<<(rvs_base.last()->P(t,1))<<endl;
        }
      }
      uintA rvs_base_ids;
      FOR1D(rvs_base, v) {rvs_base_ids.append(rvs_base(v)->id);}
      FOR1D(var->range, val) {
        if (DEBUG>3) {cout<<"Calculating for value "<<var->range(val)<<" ["<<val<<"]"<<endl;}
        if (var->range(val)>rvs_base_ids.N) break;
        MT::Array< uintA > base_combos;
        TL::allSubsets(base_combos, rvs_base_ids, var->range(val));
        if (DEBUG>3) {PRINT(base_combos);}
        prob_total = 0.0;
        FOR1D(base_combos, c2) {
          prob_single = 1.0;
          FOR1D(rvs_base, v) {
            if (base_combos(c2).findValue(rvs_base(v)->id)>=0)
              prob_single *= rvs_base(v)->P(t,1);
            else
              prob_single *= rvs_base(v)->P(t,0);
            if (DEBUG>3) {cout<<" After using ";rvs_base(v)->lit->write();cout<<rvs_base(v)->id<<"  "<<prob_single<<endl;}
          }
          if (DEBUG>3) {cout<<base_combos(c2)<<": "<<prob_single<<endl;}
          prob_total += prob_single;
        }
        var->P(t,val) = prob_total;
        if (DEBUG>3) {cout<<"--> P(V="<<var->range(val)<<") "<<var->P(t,val)<<endl;}
      }
    }
    if (DEBUG>0) {cout<<"Updated variable: ";var->write();}
  }
  if (DEBUG>0) {cout<<"calcDerived - CountFunction [END]"<<endl;}
}



void calcDerived1(TL::SumFunction* f, uint t, RV_Manager* rvm, uintA& constants) {
  uint DEBUG = 0;
  if (DEBUG>0) {cout<<"calcDerived - SumFunction [START]"<<endl;}
  if (DEBUG>0) {
    f->writeNice(cout);cout<<endl;
    PRINT(t);
  }
  CHECK(f->d == 0, "so far implemented only for zero-ary");
  uintA empty;
  TL::FunctionAtom* fi = rvm->getFVW(f, empty);
  LogicRV* var = rvm->fi2v(fi);
  CHECK(var->type == RV_TYPE__FUNC_EXPECT, "only defined for expectation random vars");
  uint c;
  MT::Array< uintA > combos;
  TL::allPossibleLists(combos, constants, f->f_base->d, true, true);
  double sum=0.;
  FOR1D(combos, c) {
//     if (DEBUG>1) {cout<<"-- "<<combos(c)<<endl;}
    fi = rvm->getFVW(f->f_base, combos(c));
    if (DEBUG>1) {fi->write();}
    LogicRV* sub_var = rvm->fi2v(fi);
    if (DEBUG>1) {cout<<"  "<<sub_var->P(t,0)<<endl;}
    CHECK(sub_var->type == RV_TYPE__FUNC_EXPECT, "only defined for expectation random vars");
    sum += sub_var->P(t,0);
  }
  var->P(t,0) = sum;
  if (DEBUG>0) {var->write();}
  if (DEBUG>0) {cout<<"calcDerived - SumFunction [END]"<<endl;}
}


void calcDerived1(TL::RewardFunction* f, uint t, RV_Manager* rvm, uintA& constants) {
  uint DEBUG = 0;
  if (DEBUG>0) {cout<<"calcDerived - RewardFunction [START]"<<endl;}
  if (DEBUG>0) {
    f->writeNice(cout);cout<<endl;
    PRINT(t);
  }
  CHECK(f->d == 0, "so far implemented only for zero-ary");
  uintA empty;
  TL::FunctionAtom* fi = rvm->getFVW(f, empty);
  LogicRV* var = rvm->fi2v(fi);
  double prob = 1.0;
  uint i;
  FOR1D(f->grounded_pis, i) {
    PredicateRV* sub_var = rvm->l2v(f->grounded_pis(i));
    if (f->grounded_pis(i)->positive)
      prob *= sub_var->P(t,1);
    else
      prob *= sub_var->P(t,0);
  }
  var->P(t,0) = prob * f->reward_value;
  if (DEBUG>0) {PRINT(prob);  var->write();}
  if (DEBUG>0) {cout<<"calcDerived - RewardFunction [END]"<<endl;}
}


}
