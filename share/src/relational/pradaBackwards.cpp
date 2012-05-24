#if 0

#include "pradaBackwards.h"
// #include <gsl/gsl_sf_log.h>

// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//  ATTENTION
//  This file needs significant revision!!
// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

#if 0

namespace TL {


  
  
// -------------------------------------------------------------------------------------------
// -------------------------------------------------------------------------------------------
// -------------------------------------------------------------------------------------------
//
//         VARIOUS
//
// -------------------------------------------------------------------------------------------
// -------------------------------------------------------------------------------------------
// -------------------------------------------------------------------------------------------


void copy_beliefs(arr& beliefs_p, arr& beliefs_f, const NID_DBN& net, uint horizon) {
  beliefs_p.resize(horizon+1, net.rvs_state__p_prim.N, 2);
  beliefs_f.resize(horizon+1, net.rvs_state__f_prim.N, 10);
  beliefs_p.setZero();
  beliefs_f.setZero();
  uint t, v, val;
  for (t=0; t<=horizon; t++) {
    arr local_betas__p, local_betas__f;
    net.getBelief(t, local_betas__p, local_betas__f);
    FOR2D(local_betas__p, v, val) {
      beliefs_p(t,v,val) = local_betas__p(v,val);
    }
    FOR2D(local_betas__f, v, val) {
      beliefs_f(t,v,val) = local_betas__f(v,val);
    }
  }
  
//   cout<<"copy_beliefs: "<<endl;
//   for (t=0; t<=horizon; t++) {
//     cout<<"=====    t="<<t<<endl;
//     net.writeState(t, true);
//     for (v=0; v<beliefs_p.d1; v++) {
//       for (val=0; val<beliefs_p.d2; val++) {
//         cout << beliefs_p(t,v,val) << " ";
//       }
//       cout << endl;
//     }
//     for (v=0; v<beliefs_f.d1; v++) {
//       for (val=0; val<beliefs_f.d2; val++) {
//         cout << beliefs_f(t,v,val) << " ";
//       }
//       cout << endl;
//     }
//   }
}
  

void setUnchangeableValues(NID_DBN& net, const TL::State& state) {
  uint i, t;
  FOR1D(net.rvs_state__p_prim, i) {
    if (!net.rvs_state__p_prim(i)->changeable) {
      if (LogicEngine::holds(state, net.rvs_state__p_prim(i)->pi)) { // teuer, das jedes Mal wieder neu zu machen
        for (t=0; t<=net.horizon; t++) {
          net.rvs_state__p_prim(i)->P(t, 0) = 0.0;
          net.rvs_state__p_prim(i)->P(t, 1) = 1.0;
        }
      }
      else {
        for (t=0; t<=net.horizon; t++) {
          net.rvs_state__p_prim(i)->P(t, 0) = 1.0;
          net.rvs_state__p_prim(i)->P(t, 1) = 0.0;
        }
      }
    }
  }
  
  uint val, idx_target_value;
  FOR1D(net.rvs_state__f_prim, i) {
    if (!net.rvs_state__f_prim(i)->changeable) {
      FunctionInstance* fi = net.rvs_state__f_prim(i)->fi;
      CHECK(fi->args.N == 1, "");
      uint target_value = (uint) LogicEngine::getValue(fi->args(0), fi->f, state);
      idx_target_value = net.rvs_state__f_prim(i)->range.findValue(target_value);
      for (t=0; t<=net.horizon; t++) {
        for (val=0; val<net.rvs_state__f_prim(i)->dim; val++) {
          if (val == idx_target_value)
            net.rvs_state__f_prim(i)->P(t, val) = 1.0;
          else
            net.rvs_state__f_prim(i)->P(t, val) = 0.0;
        }
      }
    }
  }
}



void calcBloxworld_homies(MT::Array< uintA >& gangs, const State& s, const LogicEngine& le) {
  Predicate* p_HOMIES = le.getPredicate(MT::String("homies"));
  gangs.clear();
  uint i, k;
  boolA constants_covered(le.constants.N);
  constants_covered.setUni(false);
  FOR1D(le.constants, i) {
    if (le.holds_straight(le.constants(i), MT::String("table"), s))
      continue;
    if (constants_covered(i))
      continue;
    uintA homies;
    LogicEngine::getRelatedObjects(homies, le.constants(i), true, *p_HOMIES, s);
    homies.insert(0, le.constants(i));
    constants_covered(i) = true;
    FOR1D(homies, k) {
      constants_covered(le.constants.findValue(homies(k))) = true;
    }
    gangs.append(homies);
  }
}

void calcBloxworld_piles(MT::Array< uintA >& piles, const State& s, const LogicEngine& le) {
  Predicate* p_ON = le.getPredicate(MT::String("on"));
  bool inserted;
  uint i, k;
  FOR1D(s.pi_prim, i) {
    // on(A,B)
    if (s.pi_prim(i)->pred == p_ON) {
      inserted = false;
      FOR1D(piles, k) {
        // pile with [A, ..., top]  -->  put B below
        if (piles(k)(0) == s.pi_prim(i)->args(0)) {
          piles(k).insert(0, s.pi_prim(i)->args(1));
          inserted = true;
        }
        // pile with [table, lastBlock, ..., B]  -->  put A on top
        else if (piles(k).last() == s.pi_prim(i)->args(1)) {
          piles(k).append(s.pi_prim(i)->args(0));
          inserted = true;
        }
      }
      if (!inserted) {
        uintA newPile;
        newPile.append(s.pi_prim(i)->args(1));
        newPile.append(s.pi_prim(i)->args(0));
        piles.append(newPile);
      }
    }
  }
}













// -------------------------------------------------------------------------------------------
// -------------------------------------------------------------------------------------------
// -------------------------------------------------------------------------------------------
//
//         GOAL STATE SAMPLING
//
// -------------------------------------------------------------------------------------------
// -------------------------------------------------------------------------------------------
// -------------------------------------------------------------------------------------------



#define final_state_determinism_softern__others 0.99
void mergeStartStateWithGoal(Goal* goal, NID_DBN& net, uint t, const State& s0, double final_state_determinism_softener) {
  uint DEBUG = 0;
  if (DEBUG>0) {
    cout<< "mergeStartStateWithGoal [START]"<<endl;
  }
  CHECK(goal->goal_type == GOAL_TYPE__PREDICATE_INSTANCE  ||  goal->goal_type == GOAL_TYPE__PREDICATE_INSTANCE_LIST, "invalid goal");
  
  PredIA goal_specification;
  if (goal->goal_type == GOAL_TYPE__PREDICATE_INSTANCE) {
    goal_specification.append(((PredicateGoal*) goal)->pi);
  }
  else if (goal->goal_type == GOAL_TYPE__PREDICATE_INSTANCE_LIST) {
    goal_specification.append(((PredicateListGoal*) goal)->pis);
  }
  else {
    NIY;
  }
  
  // INIT:
#if 0
  // Alternative (1):  set everything uniform
  uint v, val;
  FOR1D(net->rvs_state__p_prim, v) {
  if (net.rvs_state__p_prim(v)->changeable) {
  net.rvs_state__p_prim(v)->P(t,0) = 0.5;
  net.rvs_state__p_prim(v)->P(t,1) = 0.5;
//       for (val=0; val<net.rvs_state__p_prim(v)->dim; val++) {
//         net.rvs_state__p_prim(v)->P(t,val) = 1. / net.rvs_state__p_prim(v)->dim;
//       }
}
}
  FOR1D(net.rvs_state__f_prim, v) {
  if (net.rvs_state__f_prim(v)->changeable) {
  for (val=0; val<net.rvs_state__f_prim(v)->dim; val++) {
  net.rvs_state__f_prim(v)->P(t,val) = 1. / net.rvs_state__f_prim(v)->dim;
}
}
}
#else
  // Alternative (2):  set like in start state
  uint v, val;
  FOR1D(net.rvs_state__p_prim, v) {
    if (net.rvs_state__p_prim(v)->changeable) {
      if (LogicEngine::holds(s0, net.rvs_state__p_prim(v)->pi)) {
        net.rvs_state__p_prim(v)->P(t,0) = (1.-final_state_determinism_softern__others);
        net.rvs_state__p_prim(v)->P(t,1) = final_state_determinism_softern__others;
      }
      else {
        net.rvs_state__p_prim(v)->P(t,0) = final_state_determinism_softern__others;
        net.rvs_state__p_prim(v)->P(t,1) = (1.-final_state_determinism_softern__others);
      }
    }
    else {
      if (LogicEngine::holds(s0, net.rvs_state__p_prim(v)->pi)) {
        net.rvs_state__p_prim(v)->P(t,0) = 0.0;
        net.rvs_state__p_prim(v)->P(t,1) = 1.0;
      }
      else {
        net.rvs_state__p_prim(v)->P(t,0) = 1.0;
        net.rvs_state__p_prim(v)->P(t,1) = 0.0;
      }
    }
  }
  FOR1D(net.rvs_state__f_prim, v) {
//     if (net.rvs_state__f_prim(v)->changeable) {
    FunctionInstance* fi = net.rvs_state__f_prim(v)->fi;
    CHECK(fi->args.N == 1, "");
    uint target_value = (uint) LogicEngine::getValue(fi->args(0), fi->f, s0);
    uint idx_target_value = net.rvs_state__f_prim(v)->range.findValue(target_value);
    for (val=0; val<net.rvs_state__f_prim(v)->dim; val++) {
      if (val == idx_target_value)
        net.rvs_state__f_prim(v)->P(t, val) = 1.0;
      else
        net.rvs_state__f_prim(v)->P(t, val) = 0.0;
    }
//     }
  }
#endif
  
  // Introduce knowledge from goal
  FOR1D(goal_specification, v) {
    if (goal_specification(v)->positive) {
      net.rvm->pi2v(goal_specification(v))->P(t,0) = (1.-final_state_determinism_softener);
      net.rvm->pi2v(goal_specification(v))->P(t,1) = final_state_determinism_softener;
    }
    else {
      net.rvm->pi2v(goal_specification(v))->P(t,0) = final_state_determinism_softener;
      net.rvm->pi2v(goal_specification(v))->P(t,1) = (1.-final_state_determinism_softener);
    }
  }
  
  net.calcDerived(t);
  
  if (DEBUG>0) {
    cout<<"GOAL:  "; goal->writeNice();  cout<<endl;
    cout<<"STATE s0:  "<<endl;
    s0.writeNice(); cout<<endl;
    cout<<"DBN t=" << t << ":"<<endl;
    net.writeState(t);
  }
  
  if (DEBUG>0) {
    cout<< "mergeStartStateWithGoal [END]"<<endl;
  }
}



//-----------------------------------------
// TODO adapt to problem
uintA goal_boxes__in_usage;
uintA goal_boxes__already_used;
void ZICKZACK_PRADA::resetGoalMixture(bool within) {
	if (!within) {
		goal_boxes__in_usage.clear();
    goal_boxes__already_used.clear();
  }
	else {
    goal_boxes__in_usage.clear();
  }
}




//-----------------------------------------

void sampleGoalState_tower(Goal* goal, NID_DBN& net, uint t, const State& s0, double final_state_determinism_softener, LogicEngine* le, uintA& usedComponents) {
  PredicateListGoal det_goal(((PredicateListGoal*) goal)->pis);
  uintA boxes;
  LogicEngine::getArguments(boxes, s0, *le->getPredicate(MT::String("box")));
  uintA args(2);
  uint k;
  FOR1D(det_goal.pis, k) {
    if (det_goal.pis(k)->pred->name == "onBox")
      break;
  }
  CHECK(k!=det_goal.pis.N, "");
  args(0) = det_goal.pis(k)->args(0);
  if (goal_boxes__in_usage.N == 0) {
    uintA fresh_target_boxes;
    uint l;
    FOR1D(boxes, l) {
      if (goal_boxes__already_used.findValue(boxes(l)) < 0)
        fresh_target_boxes.append(boxes(l));
    }
    if (fresh_target_boxes.N == 0)
      fresh_target_boxes = boxes;
    uint new_box = fresh_target_boxes(rnd.num(fresh_target_boxes.N));
    goal_boxes__in_usage.append(new_box);
    goal_boxes__already_used.append(new_box);
    PRINT(goal_boxes__in_usage);
    PRINT(goal_boxes__already_used);
		cout<<"SAMPLING NEW BOX " << new_box << " from " << boxes << "!!"<<endl;
  }
  args(1) = goal_boxes__in_usage(0);
  PredicateInstance* pi = le->getPI(le->getPredicate(MT::String("on")), true, args);
  det_goal.pis(k) = pi;
  cout<<"SAMPLED GOAL STATE: "; det_goal.writeNice();  cout<<endl;
  mergeStartStateWithGoal(&det_goal, net, 0, s0, final_state_determinism_softener);
  // HACK
  // Set closed() non-deterministic
  uint q;
  FOR1D(boxes, q) {
    uintA args(1);
    args(0) = boxes(q);
    PredicateInstance* pi = le->getPI(le->getPredicate(MT::String("closed")), true, args);
    PredicateRV* var = net.rvm->pi2v(pi);
    if (LogicEngine::holds(s0, pi)) {
      var->P(t,0) = 0.2;
      var->P(t,1) = 0.8;
    }
    else {
      var->P(t,0) = 0.8;
      var->P(t,1) = 0.2;
    }
  }
}


MT::Array< MT::Array< uintA > > hack__usedComponents__piles; // Outer: gang, Inner: used gang-piles

void sampleGoalState_clearance(NID_DBN& net, uintA& usedComponents, const TL::State& s0, TL::LogicEngine* le, double final_state_determinism_softener) {
  int DEBUG = 1;  // -1 = no info
  if (DEBUG>0) {
    cout<<"sampleGoalState_clearance [START]"<<endl;
  }
  
  CHECK(s0.derivedDerived, "");
  
  TL::Predicate* p_TABLE = le->getPredicate(MT::String("table"));
  TL::Predicate* p_ON = le->getPredicate(MT::String("on"));
  TL::Predicate* p_ABOVE = le->getPredicate(MT::String("above"));
  TL::Predicate* p_INHAND = le->getPredicate(MT::String("inhand"));
  //   TL::Predicate* p_CLEARED = le->getPredicate(MT::String("cleared"));
  
  uint i, k;
  
  uintA dummy;
  LogicEngine::getArguments(dummy, s0, *p_TABLE);
  uint id_table = dummy(0);
    
  //-----------------------------------------------------------------------------
  // (1) Calculate state information
  
  // Calculate all piles
  
  MT::Array< uintA > all_piles;
  calcBloxworld_piles(all_piles, s0, *le);
  
  if (DEBUG>0) {
    cout<<"PILES:"<<endl;
    FOR1D(all_piles, i) {
      cout << all_piles(i) << endl;
    }
  }
  
  
  
  //-----------------------------------------------------------------------------
  // (2) Determine classes to be ordered
  
  // Determine gangs (= gangs of homies)
  MT::Array< uintA > gangs;
  calcBloxworld_homies(gangs, s0, *le);
  
  if (DEBUG>0) {
    cout<<"GANGS:"<<endl;
    FOR1D(gangs, i) {
      cout << i << ":  " << gangs(i) << endl;
    }
  }
  
  if (usedComponents.N == 0) {
    FOR1D(hack__usedComponents__piles, i) {
      hack__usedComponents__piles(i).clear();
    }
    hack__usedComponents__piles.clear();
    hack__usedComponents__piles.resize(gangs.N);
  }
  
  // Determine the min number of missing homies per gang
  uintA num_missing_homies(gangs.N);
  uintA cleared_objects;
  uintA args(2);
  FOR1D(gangs, i) {
    // If gang already inorder:
    //    completely copy all properties connected to these objects only.
//     if (le->holds_straight(gangs(i)(0), MT::String("inorder"), s0)) {
    if (false) {
      // HIEEEEEEEEEEEEEEEEEER TODO TODO TODO TODO TODO
      // TODO anpassen: falls einer inorder ist, sagt mglw. nichts aus; da andere  trotzdem nichts
      // inorder sein koennen wg. komplizierter on-Struktur
      num_missing_homies(i) = 0;
      cleared_objects.append(gangs(i));
    }
    // If gang not inorder yet:
    //   calculate max missing homies.
    else {
      num_missing_homies(i) = UINT_MAX;
      FOR1D(gangs(i), k) {
        // HACK for gangs with out-objects
        if (le->holds_straight(gangs(i)(k), MT::String("out"), s0)) {
          num_missing_homies(i) = UINT_MAX;
          break;
        }
        
        uintA objects_below;
        LogicEngine::getRelatedObjects(objects_below, gangs(i)(k), true, *p_ABOVE, s0);
        if (DEBUG>2) {
          cout << "Objects below " << gangs(i)(k) << ": " << objects_below <<endl;
        }
        
        uint local_num_missing_homies = 0;
        
        uintA alien_objects_below = objects_below;
        setMinus(alien_objects_below, gangs(i));
        alien_objects_below.removeValueSafe(id_table);
        // If above non-gang-member object which is not table, then the whole pile needs to be moved.
        if (alien_objects_below.N > 0)
          local_num_missing_homies = gangs(i).N - 1;
        // Otherwise, we define all gang-member as missing which are not below the current object.
        else {
          local_num_missing_homies = gangs(i).N - 1 - objects_below.N;
          if (objects_below.findValue(id_table) >= 0)
            local_num_missing_homies += 1;
        }
        
        num_missing_homies(i) = TL_MIN(num_missing_homies(i), local_num_missing_homies);
      }
    }
  }
  
  // Sanity check: needed as strange world situations may happen, where one block is on two blocks!!
  FOR1D(gangs, i) {
    if (num_missing_homies(i) == 0) {
      FOR1D(gangs(i), k) {
        if (!le->holds_straight(gangs(i)(k), MT::String("inorder"), s0)) {
          num_missing_homies(i) = 1;
        }
      }
    }
  }
  
  if (DEBUG>0) {
    cout<<"MIN MISSING HOMIES PER GANG: " << endl;
    FOR1D(gangs, i) {
      cout << i << ":  " << gangs(i) << "  -->  " << num_missing_homies(i) << endl;
    }
  }
  
  // Determine gangs to be ordered
  MT::Array< uintA > combos;
  uintA unordered_gang_ids;
  FOR1D(gangs, i) {
    if (num_missing_homies(i) > 0)
      unordered_gang_ids.append(i);
  }
  allSubsets(combos, unordered_gang_ids, false, false);
  arr weights(combos.N);
  FOR1D(combos, i) {
    // Hack to give sets with out-objects very low probability
    weights(i) = 1.0;
    FOR1D(combos(i), k) {
      if (num_missing_homies(combos(i)(k)) == UINT_MAX)
        weights(i) *= 0.1;
    }
    if (usedComponents.findValue(i) >= 0)
      weights(i) *= 0.5;
    else
      weights(i) *= 1.0;
  }
  
  CHECK(weights.N > 0, "wrong reconnaissance of non-inorder groups");
  
  if (TL::isZero(sum(weights))) {
    weights.setUni(1.0);
  }
  
  if (DEBUG>1) {
    PRINT(unordered_gang_ids);
    cout<<"POTENTIAL GANG COMBOS with weight:"<<endl;
    FOR1D(combos, i) {
      cout<<i<< ": "<< combos(i) << "   --> " << weights(i) << endl;
    }
  }
  
  uint combo_target = basic_sample(weights);
  usedComponents.setAppend(combo_target);
  uintA target_gangs;
  FOR1D(combos(combo_target), i) {
    target_gangs.append(combos(combo_target)(i));
  }
  
  if (DEBUG>=0) {
    cout<<"***** CHOSEN COMBO = "<<combo_target << "  with  ";
    FOR1D(target_gangs, i) {
      cout<<"  "<<gangs(target_gangs(i));
    }
    cout<<endl;
  }
  
  boolA pileContainsTargetGangster(all_piles.N);
  FOR1D(all_piles, i) {
    pileContainsTargetGangster(i) = false;
    FOR1D(target_gangs, k) {
      if (numberSharedElements(all_piles(i), gangs(target_gangs(k))) >= 1) {
        pileContainsTargetGangster(i) = true;
        break;
      }
    }
  }
  
  if (DEBUG>0) {
    PRINT(pileContainsTargetGangster);
  }
  
  
  //-----------------------------------------------------------------------------
  // (3) Set values
  
  net.setStateUniform(0);
  
  // (3.1)  Non-changeable values
  setUnchangeableValues(net, s0);
  
  // (3.2)  All properties of objects which are not in manipulated piles
  uintA untouched_objects__ordered;
  uintA untouched_objects__unordered;
  FOR1D(all_piles, i) {
    if (!pileContainsTargetGangster(i)) {
      FOR1D(all_piles(i), k) {
        if (cleared_objects.findValue(all_piles(i)(k)) >= 0)
          untouched_objects__ordered.setAppend(all_piles(i)(k));
        else
          untouched_objects__unordered.setAppend(all_piles(i)(k));
      }
    }
  }
  uintA untouched_objects__all;
  untouched_objects__all.append(untouched_objects__ordered);
  untouched_objects__all.append(untouched_objects__unordered);
  FOR1D(net.rvs_state__p_prim, i) {
    if (!net.rvs_state__p_prim(i)->changeable)
      continue;
    if (containsAllElements(untouched_objects__ordered, net.rvs_state__p_prim(i)->pi->args)) {
      if (le->holds(s0, net.rvs_state__p_prim(i)->pi)) {
        net.rvs_state__p_prim(i)->P(0,0) = 0.0;
        net.rvs_state__p_prim(i)->P(0,1) = 1.0;
      }
      else {
        net.rvs_state__p_prim(i)->P(0,0) = 1.0;
        net.rvs_state__p_prim(i)->P(0,1) = 0.0;
      }
    }
    else if (containsAllElements(untouched_objects__all, net.rvs_state__p_prim(i)->pi->args)) {
      if (le->holds(s0, net.rvs_state__p_prim(i)->pi)) {
        net.rvs_state__p_prim(i)->P(0,0) = (1.-final_state_determinism_softener);
        net.rvs_state__p_prim(i)->P(0,1) = final_state_determinism_softener;
      }
      else {
        net.rvs_state__p_prim(i)->P(0,0) = final_state_determinism_softener;
        net.rvs_state__p_prim(i)->P(0,1) = (1.-final_state_determinism_softener);
      }
    }
  }
  
  if (DEBUG>0) {
    PRINT(untouched_objects__ordered);
    PRINT(untouched_objects__unordered);
    PRINT(untouched_objects__all);
  }
 
  
  // (3.3)  Gang structures
  uint q;
  FOR1D(target_gangs, q) {
    if (DEBUG>1) {
      cout<<"SETTING FOR TARGET_GANG #" << q << " " << gangs(target_gangs(q))<<endl;
    }
    uintA target_gang = gangs(target_gangs(q));
    
    // (3.3.1)  No objects below or on top of target-gangsters (except the ones we specify later)
    PredicateInstance* pi;
    FOR1D(target_gang, i) {
      FOR1D(le->constants, k) {
        args(0) = target_gang(i);
        args(1) = le->constants(k);
        pi = le->getPI(p_ON, true, args);
        net.rvm->pi2v(pi)->P(0,0) = final_state_determinism_softener;
        net.rvm->pi2v(pi)->P(0,1) = (1.-final_state_determinism_softener);
        
        args(0) = le->constants(k);
        args(1) = target_gang(i);
        pi = le->getPI(p_ON, true, args);
        net.rvm->pi2v(pi)->P(0,0) = final_state_determinism_softener;
        net.rvm->pi2v(pi)->P(0,1) = (1.-final_state_determinism_softener);
      }
    }
    
    // (3.3.2)  No target-gang object inhand
    args.resize(1);
    FOR1D(target_gang, i) {
      args(0) = target_gang(i);
      pi = le->getPI(p_INHAND, true, args);
      net.rvm->pi2v(pi)->P(0,0) = final_state_determinism_softener;
      net.rvm->pi2v(pi)->P(0,1) = (1.-final_state_determinism_softener);
    }
    args.resize(2);
    
    
    // (3.3.3)  Build target-gang pile
    if (DEBUG>1) {
      cout<<"Setting target pile in the following."<<endl;
      cout<<"Previously used component-piles:";
      FOR1D(hack__usedComponents__piles(target_gangs(q)), i) {
        cout<<"  "<<hack__usedComponents__piles(target_gangs(q))(i);
      }
      cout<<endl;
    }
    
#define MAX_TARGET_PILE_TRIES 10
    uint current_try = 1;
    do {
      if (DEBUG>0) {PRINT(current_try);}
      uintA goal__target_pile;
      
      // (3.3.3.1)  Piles with targets on table
      MT::Array< uintA > piles__target_on_table;
      FOR1D(all_piles, i) {
        if (target_gang.findValue(all_piles(i)(1)) >= 0) {
          uintA newPile;
          newPile.append(all_piles(i)(0));
          newPile.append(all_piles(i)(1));
          k=2;
          while (k < all_piles(i).N  &&  target_gang.findValue(all_piles(i)(k)) >= 0) {
            newPile.append(all_piles(i)(k));
            k++;
          }
          piles__target_on_table.append(newPile);
        }
      }
      
      // (3.3.3.2)    Choose root pile
      // (3.3.3.2.1)  Either table pile...
      if (piles__target_on_table.N > 0) {
        arr weights(piles__target_on_table.N);
        FOR1D(piles__target_on_table, i) {
          weights(i) = pow(10., piles__target_on_table(i).N);
        }
        uint id__persisting_gang_table_pile = TL::basic_sample(weights);
      
        if (DEBUG>1) {
          cout<<"piles__target_on_table:"<<endl;
          FOR1D(piles__target_on_table, i) {
            cout << piles__target_on_table(i) << "   ---> " << weights(i) << endl;
          }
          PRINT(id__persisting_gang_table_pile);
        }
        
        goal__target_pile.append(piles__target_on_table(id__persisting_gang_table_pile));
      }
      // (3.3.3.2.1)  ...or random other object on table.
      else {
        goal__target_pile.append(id_table);
        goal__target_pile.append(target_gang(rnd.num(target_gang.N)));
        if (DEBUG>1) {
          cout<<"No piles__target_on_table. Thus randomly choose as starter:  "<<goal__target_pile<<endl;
        }
      }
      
      // (3.3.3.3)  Shuffle remaining gangsters
      uintA remaining_gangsters = target_gang;
      setMinus(remaining_gangsters, goal__target_pile);
      remaining_gangsters.permuteRandomly();
      goal__target_pile.append(remaining_gangsters);
      
      if (DEBUG>1) {
        PRINT(goal__target_pile);
      }
      
      // (3.3.3.4)  Check whether "goal__target_pile" is new
      FOR1D(hack__usedComponents__piles(target_gangs(q)), i) {
        if (goal__target_pile == hack__usedComponents__piles(target_gangs(q))(i))
          break;
      }
      
      // (3.3.3.5)  If new: Write information in net  &  break
      if (hack__usedComponents__piles(target_gangs(q)).N == i  || current_try == MAX_TARGET_PILE_TRIES) {
        hack__usedComponents__piles(target_gangs(q)).setAppend(goal__target_pile);
        for (i=1; i<goal__target_pile.N; i++) {
          args(0) = goal__target_pile(i);
          args(1) = goal__target_pile(i-1);
          PredicateInstance* pi = le->getPI(p_ON, true, args);
          net.rvm->pi2v(pi)->P(0,0) = (1.-final_state_determinism_softener);
          net.rvm->pi2v(pi)->P(0,1) = final_state_determinism_softener;
        }
        break;
      }
      
    } while (current_try++ <= MAX_TARGET_PILE_TRIES);
  }
  
  
  net.calcDerived(0);
  net.checkStateSoundness(0);
  
  
  if (DEBUG>0) {
    cout<<endl;
    cout<<"FINAL STATE BELIEF:"<<endl;
    net.writeState(0, true, 0.2);
  }
  
  if (DEBUG>0) {
    cout<<"sampleGoalState_clearance [END]"<<endl;
  }
}










// -------------------------------------------------------------------------------------------
// -------------------------------------------------------------------------------------------
// -------------------------------------------------------------------------------------------
//
//         ZICKZACK-PRADA
//
// -------------------------------------------------------------------------------------------
// -------------------------------------------------------------------------------------------
// -------------------------------------------------------------------------------------------


ZICKZACK_PRADA::ZICKZACK_PRADA(TL::LogicEngine* le) : PRADA(le) {
  backward_net = NULL;
}

ZICKZACK_PRADA::~ZICKZACK_PRADA() {
  if (backward_net != NULL)
    delete backward_net;
}


void ZICKZACK_PRADA::setStartState(const TL::State& s0) {
  setState(s0, 0);
  this->s0 = s0;
  if (backward_net == NULL) {
    backward_net = new NID_DBN(net->objects, dbn_preds, dbn_funcs, le->actions, ground_rules_backwards, noise_softener, horizon_backward, le);
  }
}

void ZICKZACK_PRADA::setGroundRulesBackwards(TL::RuleSet& ground_rules_backwards) {
  this->ground_rules_backwards = ground_rules_backwards;
}


void ZICKZACK_PRADA::forward_sampling(PredIA& sampled_actions, NID_DBN& net, uint horizon) {
  PredIA fixed_actions(net.horizon);
  fixed_actions.setUni(NULL);
  sampleActionsAndInfer(sampled_actions, fixed_actions, &net, horizon);
}


void ZICKZACK_PRADA::setNumberOfSamplesForward(uint num_samples_forward) {
  this->num_samples_forward = num_samples_forward;
}

void ZICKZACK_PRADA::setNumberOfSamplesBackward(uint num_samples_backward) {
  this->num_samples_backward = num_samples_backward;
}


void ZICKZACK_PRADA::setNumGoalStateSamples(uint num_goal_state_samples) {
  this->num_goal_state_samples = num_goal_state_samples;
}

void ZICKZACK_PRADA::setHorizonForward(uint horizon_forward) {
  this->horizon_forward = horizon_forward;
}

void ZICKZACK_PRADA::setHorizonBackward(uint horizon_backward) {
  this->horizon_backward = horizon_backward;
}

void ZICKZACK_PRADA::setFinalStateDeterminismSoftener(double final_state_determinism_softener) {
  this->final_state_determinism_softener = final_state_determinism_softener;
}







// -------------------------------------------------------------------------------------------
// -------------------------------------------------------------------------------------------
//
//         BACKWARD PLANNING


#define GOAL_STATE_SAMPLING__MERGE 1
#define GOAL_STATE_SAMPLING__CLEARANCE 2
void ZICKZACK_PRADA::sampleGoalState(NID_DBN& net, uintA& usedComponents, const TL::State& s) {
  uint type;
  MaximizeFunctionGoal* g = dynamic_cast<MaximizeFunctionGoal*>(goal);
  if (g!= NULL) {
    type = GOAL_STATE_SAMPLING__CLEARANCE;
  }
  else
    type = GOAL_STATE_SAMPLING__MERGE;
  
  if (type == GOAL_STATE_SAMPLING__MERGE) {
//     mergeStartStateWithGoal(goal, net, 0, s, final_state_determinism_softener);
    sampleGoalState_tower(goal, net, 0, s, final_state_determinism_softener, le, usedComponents);
  }
  else if (type == GOAL_STATE_SAMPLING__CLEARANCE)
    sampleGoalState_clearance(net, usedComponents, s, le, final_state_determinism_softener);
}


#define BACKWARD_VARIANT__SIMPLE 1
void ZICKZACK_PRADA::backward_sampling(PredIA& sampled_actions, NID_DBN& net) {
  uint DEBUG = 0;
  uint type = BACKWARD_VARIANT__SIMPLE;
  if (type == BACKWARD_VARIANT__SIMPLE) {
    if (DEBUG > 0) net.writeState(0);
    PredIA fixed_actions(net.horizon);
    fixed_actions.setUni(NULL);
    sampleActionsAndInfer(sampled_actions, fixed_actions, &net, net.horizon); 
    if (DEBUG > 0) {
      TL::writeNice(sampled_actions); cout<<endl;
      net.writeState(net.horizon);
    }
  }
  else
    NIY;
}



void ZICKZACK_PRADA::createBetas(arr& betas_p, arr& betas_f, NID_DBN& backward_net, uintA& usedComponents, uint horizon, uint num_plan_samples) {
  uint DEBUG = 0;
  if (DEBUG>0)
    cout << "createBetas [START]"<<endl;
  
  betas_p.resize(backward_net.horizon+1, backward_net.rvs_state__p_prim.N, 2);
  betas_f.resize(backward_net.horizon+1, backward_net.rvs_state__f_prim.N, 10);
  betas_p.setZero();
  betas_f.setZero();
  
  // for statistics
  action_choices.resize(backward_net.horizon, ground_actions.N);
  action_choices.setZero();
  
  uint k, t;
  sampleGoalState(backward_net, usedComponents, s0);
    
  if (DEBUG>1) {
    cout<<"SAMPLED GOAL STATE:"<<endl;
    backward_net.writeState(0, true, 0.05);
  }
  
  for(k=0; k<num_plan_samples; k++) {
    if (DEBUG>2) cout<<"Plan #" << k << endl;
    if (k%10==0) cerr <<"."<<std::flush;
    PredIA sampled_actions;
    backward_sampling(sampled_actions, backward_net);
    arr local_betas_p, local_betas_f;
    copy_beliefs(local_betas_p, local_betas_f, backward_net, backward_net.horizon);
    betas_p += local_betas_p;
    betas_f += local_betas_f;
    // statistics
    FOR1D(sampled_actions, t) {
      uint action_id = ground_actions.findValue(sampled_actions(t));
      action_choices(t, action_id)++;
    }
    if (DEBUG>2) {
      cout<<"Sampled plan:  "; writeNice(sampled_actions); cout<<endl;
    }
  }
  
  betas_p /= 1.0 * num_plan_samples;
  betas_f /= 1.0 * num_plan_samples;
  
  // statistics
  if (DEBUG>0) {
    cout<<"Action application statistics (backward):"<<endl;
    printf("%-14s","Time");
    for(t=0;t<horizon;t++) printf("%5u",t);
    cout<<endl;
    FOR1D(ground_actions, k) {
      uint number = 0;
      for(t=0;t<action_choices.d0;t++) {
        number += action_choices(t,k);
      }
      if (number == 0)
        continue;
      MT::String name;
      ground_actions(k)->name(name);
      printf("%-14s",(char*)name);
      for(t=0;t<action_choices.d0;t++) printf("%5u", action_choices(t,k));
      cout<<endl;
    }
    cout<<endl;
  }

  
  if (DEBUG>4) {
    for (t=0; t<betas_p.d0; t++) {
      cout << endl << "Significant average betas at t=" << t << " using " << num_plan_samples << " samples:"<<endl;
      for (k=0; k<betas_p.d1; k++) {
        if (betas_p(t, k, 1) > 0.05) {
          backward_net.rvs_state__p_prim(k)->pi->writeNice(cout); cout<<"  "<<betas_p(t, k, 1);
          cout << endl;
        }
      }
      cout<<"Function values omitted"<<endl;
    }
  }
  
  if (DEBUG>1) {
    Predicate* p_TABLE = le->getPredicate(MT::String("table"));
    Predicate* p_HOMIES = le->getPredicate(MT::String("homies"));
    Predicate* p_BLOCK = le->getPredicate(MT::String("block"));
    Predicate* p_BALL = le->getPredicate(MT::String("ball"));
    cout<<"Significant beliefs (backward):"<<endl;
    printf("%-14s", "Time");
    for (t=0; t<betas_p.d0; t++) {
      cout<<"     ";
      printf("%4d", t);
    }
    cout<<endl;
    for (k=0; k<betas_p.d1; k++) {
      for (t=0; t<betas_p.d0; t++) {
        if (betas_p(t, k, 1) > 0.05)
          break;
      }
      if (t == betas_p.d0)
        continue;
      if (backward_net.rvs_state__p_prim(k)->pi->pred == p_HOMIES  ||  backward_net.rvs_state__p_prim(k)->pi->pred == p_TABLE
          ||  backward_net.rvs_state__p_prim(k)->pi->pred == p_BLOCK  ||  backward_net.rvs_state__p_prim(k)->pi->pred == p_BALL)
        continue;
      String name;
      backward_net.rvs_state__p_prim(k)->pi->name(name);
      printf("%-14s", name.p);
      for (t=0; t<betas_p.d0; t++) {
        cout<<"     ";
        if (betas_p(t, k, 1) > 0.05)
          printf("%-2.2f", betas_p(t, k, 1));
        else
          printf("%-4d", 0);
      }
      cout << endl;
    }
    cout<<"Function values omitted"<<endl;
  }
  
  if (DEBUG>0)
    cout << "createBetas [END]"<<endl;
}









// -------------------------------------------------------------------------------------------
// -------------------------------------------------------------------------------------------
//
//         PLANNING

// #define USE_HAND_MADE_PLANS
void my_handmade_plan(PredIA& plan, LogicEngine* le, uint i) {
  plan.clear();
  if (i==0) {
    const char* plan_text =  
        "place(69 65) lift(69 65) place(69 68) lift(70 67) place(70 65) lift(67 60)";
    le->getPIs(plan, plan_text);
  }
  else if (i==1) {
    const char* plan_text =  
			"place(69 68) lift(70 67) place(70 69) lift(68 66) lift(67 60) closeBox(67)";
    le->getPIs(plan, plan_text);
  }
}


bool ZICKZACK_PRADA::plan(PredIA& best_actions, double& best_value, uint num_samples) {
  uint DEBUG = 3;
  if (DEBUG>0) {cout<<"plan [START]"<<endl;}
  
  calc_dbn_concepts();
  
  uint i, t, k;
  if (DEBUG>0) {
//     MT::Array< uintA > piles;
//     calcBloxworld_piles(piles, s0, *le);
//     MT::Array< uintA > gangs;
//     calcBloxworld_homies(gangs, s0, *le);
//     uint id_inhand = LogicEngine::getArgument(s0, *le->getPredicate(MT::String("inhand")));
//     cout<<"WORLD STATE INFO:"<<endl;
// //     cout<<"- Homies:"<<endl;
// //     FOR1D(gangs, i) {
// //       cout<<i<<": ";
// //       FOR1D(gangs(i), k) {
// //         cout<<" "<<gangs(i)(k);
// //         if (le->holds_straight(gangs(i)(k), MT::String("block"), s0))
// //           cout<<"";
// //         else if (le->holds_straight(gangs(i)(k), MT::String("ball"), s0))
// //           cout<<"o";
// //         else HALT("");
// //       }
// //       cout<<endl;
// //     }
//     cout<<"- Piles:"<<endl;
//     FOR1D(piles, i) {
//       cout<<i<<":  "<<piles(i);
//       cout << "   -- ";
//       FOR1D(piles(i), k) {
//         FOR1D(gangs, t) {
//           if (gangs(t).findValue(piles(i)(k)) >= 0) {
//             cout << " " << t;
//             break;
//           }
//         }
// //         CHECK(t != gangs.N, "");
//       }
//       cout<<endl;
//     }
//     cout<<"H:  ";
//     if (id_inhand != UINT_MAX) {
//       cout << id_inhand;
//       FOR1D(gangs, t) {
//         if (gangs(t).findValue(id_inhand) >= 0) {
//           cout << "   --    " << t;
//           break;
//         }
//       }
//     }
//     else
//       cout << "-";
//     cout << endl;
  }
  
  any_sensible_action = true;
  
  // Forward net
  CHECK(net != NULL, "forward net should've been built by now!");
  // Backward net
  CHECK(backward_net != NULL, "backward net should've been built by now!");
  
  MT::Array< PredIA > overall_best_plans(num_goal_state_samples);
  arr overall_best_values(num_goal_state_samples);
  uintA overall_best_max_depths(num_goal_state_samples, 2);
  
 
  // -----------------------------
  //  (1) Backward
  
  if (DEBUG>0) {cout << endl << "+++++++++++  BACKWARD ++++++++++++++" <<endl;}
  MT::Array< arr > array_betas_p(num_goal_state_samples);
  MT::Array< arr > array_betas_f(num_goal_state_samples);
  uintA used_final_state_components;
  for (k=0; k<num_goal_state_samples; k++) {
    arr betas_p, betas_f;
    createBetas(betas_p, betas_f, *backward_net, used_final_state_components, horizon_backward, num_samples_backward);
    array_betas_p(k) = betas_p;
    array_betas_f(k) = betas_f;
  }
  
  arr all_betas_p(horizon_backward+1, backward_net->rvs_state__p_prim.N, 2);
  arr all_betas_f(horizon_backward+1, backward_net->rvs_state__f_prim.N, 10);
  
  all_betas_p.setUni(0.);
  FOR1D(array_betas_p, k) {
    all_betas_p += array_betas_p(k);
  }
  all_betas_p /= 1.0 * array_betas_p.N;
  
  all_betas_f.setUni(0.);
  FOR1D(array_betas_f, k) {
    all_betas_f += array_betas_f(k);
  }
  all_betas_f /= 1.0 * array_betas_f.N;
  
  
  if (DEBUG>1) {
    Predicate* p_TABLE = le->getPredicate(MT::String("table"));
    Predicate* p_HOMIES = le->getPredicate(MT::String("homies"));
    Predicate* p_BLOCK = le->getPredicate(MT::String("block"));
    Predicate* p_BALL = le->getPredicate(MT::String("ball"));
    cout<<"Significant beliefs (backward):"<<endl;
    printf("%-14s", "Time");
    for (t=0; t<all_betas_p.d0; t++) {
      cout<<"     ";
      printf("%4d", t);
    }
    cout<<endl;
    for (i=0; i<all_betas_p.d1; i++) {
      for (t=0; t<all_betas_p.d0; t++) {
        if (all_betas_p(t, i, 1) > 0.05)
          break;
      }
      if (t == all_betas_p.d0)
        continue;
      if (backward_net->rvs_state__p_prim(i)->pi->pred == p_HOMIES  ||  backward_net->rvs_state__p_prim(i)->pi->pred == p_TABLE
          ||  backward_net->rvs_state__p_prim(i)->pi->pred == p_BLOCK  ||  backward_net->rvs_state__p_prim(i)->pi->pred == p_BALL)
        continue;
      String name;
      backward_net->rvs_state__p_prim(i)->pi->name(name);
      printf("%-14s", name.p);
      for (t=0; t<all_betas_p.d0; t++) {
        cout<<"     ";
        if (all_betas_p(t, i, 1) > 0.05)
          printf("%-2.2f", all_betas_p(t, i, 1));
        else
          printf("%-4d", 0);
      }
      cout << endl;
    }
    cout<<"Function values omitted"<<endl;
  }
  
  
  
  
  
  
  // -----------------------------
  // (2) Forward
  
  if (DEBUG>0) {cout << endl << "+++++++++++  FORWARD ++++++++++++++" <<endl;}
  net->setState(s0.pi_prim, s0.fv_prim, 0);
  net->checkStateSoundness(0);
  
  MT::Array< PredIA > action_seqs(num_samples_forward);
  // for statistics
  action_choices.resize(horizon_forward, ground_actions.N);
  action_choices.setZero();
  
  arr values(num_samples_forward);
  uintA max_horizons_forward(num_samples_forward);
  
  for(i=0; i<num_samples_forward; i++) {
    if (DEBUG>3) cout << "Round " << i << ":  ";
    if (i%10==0) cerr <<"."<<std::flush;
    // Sample forward seq
    PredIA action_seq;
    forward_sampling(action_seq, *net, horizon_forward);
    action_seqs(i) = action_seq;
      
#ifdef USE_HAND_MADE_PLANS
    if (i<2) {
      my_handmade_plan(action_seqs(i), le, i);
      PredIA sampled_actions;
      sampleActionsAndInfer(sampled_actions, action_seqs(i), net, horizon_forward);
    }
    else
      break;
#endif
      
    if (DEBUG>3) {
      cout << endl;
      TL::writeNice(action_seq); cout<<endl;
    }
    // statistics
    FOR1D(action_seq, t) {
      uint action_id = ground_actions.findValue(action_seq(t));
      action_choices(t, action_id)++;
    }
    
    // -----------------------------
    // (3) Evaluation:  combine forward and backward
    combine_alpha_beta(values(i), max_horizons_forward(i), *net, horizon_forward, all_betas_p, all_betas_f, discount);
    if (DEBUG>3) {cout<<"value = "<<values(i)<<endl;}
  }
    
  
  // statistics
  if (DEBUG>1) {
    cout << endl;
    cout<<"Action application statistics (forward):"<<endl;
    printf("%-14s","Time");
    for(t=0;t<horizon;t++) printf("%5u",t);
    cout<<endl;
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
  


  if (DEBUG>2) {
    arr sorted_values;
    uintA sortedIndices;
    sort_desc(sorted_values, sortedIndices, values);
    cout<<"All plans sorted:"<<endl;
    FOR1D(sortedIndices, i) {
      printf("#%4u",i);
      cout<<":   ";
      printf("(%4u)", sortedIndices(i));
      cout<< "  ";
      printf("%3.2f", values(sortedIndices(i)));
      cout<< "  ";
      writeNice(action_seqs(sortedIndices(i)));
      cout << endl;
    }
    cout<<endl;
//     FOR1D(action_seqs, i) {
//       cout<<i<<":  "<<values(i)<<" ";
//       for (t=0; t<horizon_forward; t++) {
//         if (t == max_horizons_forward(i)) {
//           cout << " ***** ";
//         }
//         action_seqs(i)(t)->writeNice(cout); cout<<" ";
//       }
//       cout<<endl;
//     }
  }

  
  // -----------------------------
  // (4) Choosing the best
  
  uint max_id = values.maxIndex();
  best_actions = action_seqs(max_id);
  best_value = values(max_id);
    
  if (DEBUG>0) {
    cout<<endl<<endl;
    cout << "Plan:  #" << max_id << " with value=" << best_value  << ": ";
    TL::writeNice(best_actions);  cout<<endl;
  }

  if (DEBUG>0) {cout<<"plan [END]"<<endl;}

  return true;
//   return any_sensible_action;
}







// -------------------------------------------------------------------------------------------
// -------------------------------------------------------------------------------------------
//
//         COMBINING


// void ZICKZACK_PRADA::combine_alpha_beta(double& total_value, uint& max_d_forward, const NID_DBN& net, uint horizon_forward, const MT::Array< arr > & all_betas_p, const MT::Array< arr > & all_betas_f, double discount) {
void ZICKZACK_PRADA::combine_alpha_beta(double& total_value, uint& max_d_forward, const NID_DBN& net, uint horizon_forward, const arr& betas_p, const arr& betas_f, double discount) {
  uint DEBUG = 0;
#ifdef USE_HAND_MADE_PLANS
  DEBUG = 5;
#endif
  if (DEBUG > 0) {cout<< "combine_alpha_beta [START]"<<endl;}
  if (DEBUG > 0) {
    cout<<"Forward action sequence:  ";
    uint t, q;
    for (t=0; t<horizon_forward; t++) {
      FOR1D(net.rvs_action, q) {
        if (TL::areEqual(net.rvs_action(q)->P(t,1), 1.0)) {
          net.rvs_action(q)->pi->writeNice();
          cout<<" "<<flush;
          break;
        }
      }
    }
    cout << endl;
  }
  
  uint t, tf, tb, v, val;
  uint horizon_backward = betas_p.d0 - 1;
  
  if (DEBUG>0) {
    PRINT(horizon_forward);
    PRINT(horizon_backward);
  }
  
  // (1) CALCULATE PROBABILITIES WITHIN TIME-SLICES
  
  arr log__t_values; // starts at t=1 !!!
  //  at least one forward action needs to be taken!!
  for (t=1; t<=horizon_forward+horizon_backward; t++) {
    if (DEBUG>1) {cout<<"---------  t="<< t <<"  ---------"<<endl;}
    double max_log__t_value = -5000.;
    // determine max combination
    if (t > horizon_backward)
      tf = t-horizon_backward;
    else
      tf = 1;
    uint min = TL_MIN(horizon_forward,t);
    uint tf_max = 0;
    uint tb_max = 0;
    for (; tf<=min; tf++) {
      tb = t - tf;
      // (A) Calculate Combo  alpha(tf) * beta(tb)
      if (DEBUG>2) {cout<<"tf="<< tf << "  tb=" << tb << "..." << endl;}
      double combo_log__t_value = 0.;
//       double attribute_value;
      
     
      FOR1D(net.rvs_state__p_prim, v) {
        double attribute_value = 0.;
        for (val = 0; val < net.rvs_state__p_prim(v)->dim; val++) {
          attribute_value += net.rvs_state__p_prim(v)->P(tf, val) * betas_p(tb, v, val);
        }
        
        if (fabs(1.0 - attribute_value) < 0.0001)
          continue;
        
        if(!(attribute_value > 0.000001  &&   attribute_value < 1.0001
               &&    !(attribute_value != attribute_value))) {
          net.writeAllStates(true, -1.0);
          net.rvs_state__p_prim(v)->write(cout);
          cout<<"Strange attribute_value="<<attribute_value<<endl;
          HALT("");
        }
       
        combo_log__t_value += log(attribute_value);
        
        if (DEBUG>4) {
          if (net.rvs_state__p_prim(v)->changeable  && attribute_value < 0.4) {
            net.rvs_state__p_prim(v)->pi->writeNice(cout);
            for (val = 0; val < net.rvs_state__p_prim(v)->dim; val++) {
              cout<<"  "<<val<<":a="<<net.rvs_state__p_prim(v)->P(tf, val)<<" * b=" << betas_p(tb, v, val) << "  ";
            }
            cout<<"  -->  P=" << attribute_value << "  log(P)=" << log(attribute_value) << endl;
          }
        }
      }
      
      FOR1D(net.rvs_state__f_prim, v) {
        double  attribute_value = 0.;
        for (val = 0; val < net.rvs_state__f_prim(v)->dim; val++) {
          attribute_value += net.rvs_state__f_prim(v)->P(tf, val) * betas_f(tb, v, val);
        }
         
        if (fabs(1.0 - attribute_value) < 0.0001)
          continue;
        
        CHECK(attribute_value > 0.000001  &&   attribute_value < 1.0001
            &&    !(attribute_value != attribute_value), "Strange attribute_value="<<attribute_value);
       
        combo_log__t_value += log(attribute_value);
        
        if (DEBUG>4) {
          if (net.rvs_state__f_prim(v)->changeable  && attribute_value < 0.4) {
            net.rvs_state__f_prim(v)->fi->writeNice(cout);
            for (val = 0; val < net.rvs_state__f_prim(v)->dim; val++) {
              cout<<"  "<<val<<":a="<<net.rvs_state__f_prim(v)->P(tf, val)<<" * b=" << betas_f(tb, v, val) << "  ";
            }
            cout<<"  -->  P=" << attribute_value << "  log(P)=" << log(attribute_value) << endl;
          }
        }
      }
      
      if (DEBUG > 2) {cout<<"tf=" << tf << "  tb=" << tb << "  combo_log__t_value=" << combo_log__t_value << endl;}
      
      if (combo_log__t_value > max_log__t_value) {
        max_log__t_value = combo_log__t_value;
        tf_max = tf;
        tb_max = tb;
      }
    }
    
    log__t_values.append(log(pow(discount, t)) + max_log__t_value);
    
    
    
    // SOME MORE DEBUG INFO  --> ALLES DEBUG VON HIER ON!!
    if (DEBUG>1) {cout<<"max_log__t_value="<< max_log__t_value <<"   for t=" << t << "  with  tf_max="<<tf_max << "  and  tb_max=" << tb_max << "  log(pow(discount, t)=" << log(pow(discount, t)) << endl; cout<<"--> *total value* = " << log__t_values.last() << endl;}
    
    if (DEBUG>3) {
      cout<<"Best combo:"<<endl;
      cout<<"Partial forward action sequence:  ";
      uint t, q;
      for (t=0; t<tf_max; t++) {
        FOR1D(net.rvs_action, q) {
          if (TL::areEqual(net.rvs_action(q)->P(t,1), 1.0)) {
            net.rvs_action(q)->pi->writeNice();
            cout<<" "<<flush;
            break;
          }
        }
      }
      cout << endl;
      
      double attribute_value;
      
      FOR1D(net.rvs_state__p_prim, v) {
        attribute_value = 0.;
        for (val = 0; val < net.rvs_state__p_prim(v)->dim; val++) {
          attribute_value += net.rvs_state__p_prim(v)->P(tf_max, val) * betas_p(tb_max, v, val);
        }
        
        if (DEBUG>3) {
          if (net.rvs_state__p_prim(v)->changeable  && attribute_value < 0.6) {
            net.rvs_state__p_prim(v)->pi->writeNice(cout);
            for (val = 0; val < net.rvs_state__p_prim(v)->dim; val++) {
              cout<<"  "<<val<<":a="<<net.rvs_state__p_prim(v)->P(tf_max, val)<<" * b=" << betas_p(tb_max, v, val) << "  ";
            }
            cout<<"  -->  P=" << attribute_value << "  log(P)=" << log(attribute_value);
            if (log(attribute_value) < -3.0) cout<<" *****";
            cout << endl;
          }
        }
      }
      
      FOR1D(net.rvs_state__f_prim, v) {
        attribute_value = 0.;
        for (val = 0; val < net.rvs_state__f_prim(v)->dim; val++) {
          attribute_value += net.rvs_state__f_prim(v)->P(tf_max, val) * betas_f(tb_max, v, val);
        }
        
        if (DEBUG>3) {
          if (net.rvs_state__f_prim(v)->changeable  && attribute_value < 0.7) {
            net.rvs_state__f_prim(v)->fi->writeNice(cout);
            for (val = 0; val < net.rvs_state__f_prim(v)->dim; val++) {
              cout<<"  "<<val<<":a="<<net.rvs_state__f_prim(v)->P(tf_max, val)<<" * b=" << betas_f(tb_max, v, val) << "  ";
            }
            cout<<"  -->  P=" << attribute_value << "  log(P)=" << log(attribute_value);
            if (log(attribute_value) < -3.0) cout<<" *****";
            cout << endl;
          }
        }
      }
    }
  }

  // (2) SUM OVER ALL t
  double max__log__t_value = log__t_values.max();
  double sum = 0.0;
  double diff = 0.;
  FOR1D(log__t_values, t) {
    diff = max__log__t_value - log__t_values(t);
    sum += exp(-diff);
//     cout << "log__t_values(t="<<t<<")=" << log__t_values(t) << "   diff="<<diff<<"   exp(-diff)="<<exp(-diff)<<endl;
  }
//   PRINT(sum);
//   PRINT(log(sum));
  total_value = max__log__t_value + log(sum);
      
  if (DEBUG > 0) {
    cout<<"=============== "<<endl;
    PRINT(log__t_values);
    PRINT(max__log__t_value);
    PRINT(total_value);
  }
  
  if (DEBUG > 0) {cout<< "combine_alpha_beta [END]"<<endl<<endl;}
}


#if 0
void ZICKZACK_PRADA::combine_alpha_beta(double& total_value, const NID_DBN& net, uint horizon_forward, const arr& betas_p, const arr& betas_f, double discount) {
  uint DEBUG = 0;
  if (DEBUG > 0) {cout<< "combine_alpha_beta [START]"<<endl;}
  if (DEBUG > 0) {
    cout<<"Forward action sequence:  ";
    uint t, q;
    for (t=0; t<horizon_forward; t++) {
      FOR1D(net.rvs_action, q) {
        if (TL::areEqual(net.rvs_action(q)->P(t,1), 1.0)) {
          net.rvs_action(q)->pi->writeNice();
          cout<<" "<<flush;
          break;
        }
      }
    }
    cout << endl;
  }
  
  uint t, tf, tb, v, val;
  uint horizon_backward = betas_p.d0 - 1;
  
  if (DEBUG>0) {
    PRINT(horizon_forward);
    PRINT(horizon_backward);
  }
  
  // (1) CALCULATE PROBABILITIES WITHIN TIME-SLICES
  
  arr log__t_values; // starts at t=1 !!!
  //  at least one forward action needs to be taken!!
  for (t=1; t<=horizon_forward+horizon_backward; t++) {
    if (DEBUG>1) {cout<<"++++++++++++  t="<< t <<"  ++++++++++++++"<<endl;}
    double max_log__t_value = -5000.;
    // determine max combination
    if (t > horizon_backward)
      tf = t-horizon_backward;
    else
      tf = 1;
    uint min = TL_MIN(horizon_forward,t);
    uint tf_max = 0;
    uint tb_max = 0;
    for (; tf<=min; tf++) {
      tb = t - tf;
      if (DEBUG>2) {cout<<"tf="<< tf << "  tb=" << tb << "..." << endl;}
      double combo_log__t_value = 0.;
      double attribute_value;
      
      FOR1D(net.rvs_state__p_prim, v) {
        attribute_value = 0.;
        for (val = 0; val < net.rvs_state__p_prim(v)->dim; val++) {
          attribute_value += net.rvs_state__p_prim(v)->P(tf, val) * betas_p(tb, v, val);
        }
        combo_log__t_value += log(attribute_value);
        
        if (DEBUG>4) {
          if (net.rvs_state__p_prim(v)->changeable  && attribute_value < 0.4) {
            net.rvs_state__p_prim(v)->pi->writeNice(cout);
            for (val = 0; val < net.rvs_state__p_prim(v)->dim; val++) {
              cout<<"  "<<val<<":a="<<net.rvs_state__p_prim(v)->P(tf, val)<<" * b=" << betas_p(tb, v, val) << "  ";
            }
            cout<<"  -->  P=" << attribute_value << "  log(P)=" << log(attribute_value) << endl;
          }
        }
      }
      
      FOR1D(net.rvs_state__f_prim, v) {
        attribute_value = 0.;
        for (val = 0; val < net.rvs_state__f_prim(v)->dim; val++) {
          attribute_value += net.rvs_state__f_prim(v)->P(tf, val) * betas_f(tb, v, val);
        }
        combo_log__t_value += log(attribute_value);
        
        if (DEBUG>4) {
          if (net.rvs_state__f_prim(v)->changeable  && attribute_value < 0.4) {
            net.rvs_state__f_prim(v)->fi->writeNice(cout);
            for (val = 0; val < net.rvs_state__f_prim(v)->dim; val++) {
              cout<<"  "<<val<<":a="<<net.rvs_state__f_prim(v)->P(tf, val)<<" * b=" << betas_f(tb, v, val) << "  ";
            }
            cout<<"  -->  P=" << attribute_value << "  log(P)=" << log(attribute_value) << endl;
          }
        }
      }
      
      if (DEBUG > 2) {cout<<"tf=" << tf << "  tb=" << tb << "  combo_log__t_value=" << combo_log__t_value << endl;}
      
      if (combo_log__t_value > max_log__t_value) {
        max_log__t_value = combo_log__t_value;
        tf_max = tf;
        tb_max = tb;
      }
    }
    
    log__t_values.append(log(pow(discount, t)) + max_log__t_value);
    
    
    
    // SOME MORE DEBUG INFO
    if (DEBUG>1) {cout<<"max_log__t_value="<< max_log__t_value <<"   for t=" << t << "  with  tf_max="<<tf_max << "  and  tb_max=" << tb_max << "  log(pow(discount, t)=" << log(pow(discount, t)) << "   --> total value = " << log__t_values.last() << endl;}
    
    if (DEBUG>3) {
      cout<<"Best combo:"<<endl;
      cout<<"Partial forward action sequence:  ";
      uint t, q;
      for (t=0; t<tf_max; t++) {
        FOR1D(net.rvs_action, q) {
          if (TL::areEqual(net.rvs_action(q)->P(t,1), 1.0)) {
            net.rvs_action(q)->pi->writeNice();
            cout<<" "<<flush;
            break;
          }
        }
      }
      cout << endl;
      
      double attribute_value;
      
      FOR1D(net.rvs_state__p_prim, v) {
        attribute_value = 0.;
        for (val = 0; val < net.rvs_state__p_prim(v)->dim; val++) {
          attribute_value += net.rvs_state__p_prim(v)->P(tf_max, val) * betas_p(tb_max, v, val);
        }
        
        if (DEBUG>3) {
          if (net.rvs_state__p_prim(v)->changeable  && attribute_value < 0.4) {
            net.rvs_state__p_prim(v)->pi->writeNice(cout);
            for (val = 0; val < net.rvs_state__p_prim(v)->dim; val++) {
              cout<<"  "<<val<<":a="<<net.rvs_state__p_prim(v)->P(tf_max, val)<<" * b=" << betas_p(tb_max, v, val) << "  ";
            }
            cout<<"  -->  P=" << attribute_value << "  log(P)=" << log(attribute_value);
            if (log(attribute_value) < -3.0) cout<<" *****";
            cout << endl;
          }
        }
      }
      
      FOR1D(net.rvs_state__f_prim, v) {
        attribute_value = 0.;
        for (val = 0; val < net.rvs_state__f_prim(v)->dim; val++) {
          attribute_value += net.rvs_state__f_prim(v)->P(tf_max, val) * betas_f(tb_max, v, val);
        }
        
        if (DEBUG>3) {
          if (net.rvs_state__f_prim(v)->changeable  && attribute_value < 0.4) {
            net.rvs_state__f_prim(v)->fi->writeNice(cout);
            for (val = 0; val < net.rvs_state__f_prim(v)->dim; val++) {
              cout<<"  "<<val<<":a="<<net.rvs_state__f_prim(v)->P(tf_max, val)<<" * b=" << betas_f(tb_max, v, val) << "  ";
            }
            cout<<"  -->  P=" << attribute_value << "  log(P)=" << log(attribute_value);
            if (log(attribute_value) < -3.0) cout<<" *****";
            cout << endl;
          }
        }
      }
    }
  }
  
  if (DEBUG>0) {
    PRINT(log__t_values);
  }
  
  
  // (2) SUM OVER ALL t
  double max__log__t_value = log__t_values.max();
  double sum = 0.0;
  double diff = 0.;
  FOR1D(log__t_values, t) {
    diff = max__log__t_value - log__t_values(t);
    sum += exp(-diff);
  }
  total_value = max__log__t_value + log(sum);
      
  if (DEBUG > 0) {
    PRINT(max__log__t_value);
    PRINT(total_value);
    cout<<"++++++++++++++ "<<endl;
  }
  
  if (DEBUG > 0) {cout<< "combine_alpha_beta [END]"<<endl;}
}
#endif




}

#endif

#endif
