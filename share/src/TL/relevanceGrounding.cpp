#include "relevanceGrounding.h"
#include <TL/robotManipulationDomain.h>

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


#define GOAL__HEIGHT_CHANGE
// #define GOAL__CLEARANCE

plan_params PLAN_PARAMS;

uint _tableID;

MT::Array< ObjectInfo* > object_infos;


/*



  --------------------------------------------------------
  RELEVANCE DISTRIBUTION
  --------------------------------------------------------




*/



void RelevanceDistribution::init(uintA& objects) {
  this->objects = objects;
  weights.resize(objects.N);
  weights.setZero();
  uint i;
  FOR1D(objects, i) {
    obj2id[objects(i)] = i;
  }
}


void RelevanceDistribution::write(std::ostream& os) {
  uint i;
  FOR1D(objects, i) {
    os<<objects(i)<<" "<<weights(i)<<endl;
  }
}


void determineNecessaryObjects(uintA& necessaryObjects, Goal* goal, uintA& objects, const TL::State& s, bool INCLUDE_TABLE) {
  // TODO
  // Necessary objects:
  // (1) Objs in goal
  // (2) Table [?]
  // (3) Object inhand [?]
  
  // (1) Objs in goal
  necessaryObjects.clear();
  uint i;
  FOR1D(((PredicateGoal*) goal)->pt->slotAssignments, i) {
    necessaryObjects.setAppend(((PredicateGoal*) goal)->pt->slotAssignments(i));
  }
  
  // (2) Table
  if (INCLUDE_TABLE) {
    necessaryObjects.append(_tableID);
  }
  
  // (3) Inhand
  TL::Predicate* p_INHAND = PLAN_PARAMS.le->getPredicate(MT::String("inhand"));
  FOR1D(s.pt_prim, i) {
    if (s.pt_prim(i)->pred == p_INHAND) {
      necessaryObjects.append(s.pt_prim(i)->slotAssignments(0));
      break;
    }
  }
//   PRINT(necessaryObjects);
}



void createRelevanceDistribution_random(RelevanceDistribution& relDist, const TL::State& s, Goal* goal, uintA& objects) {
  relDist.init(objects);
  uint i;
  
  relDist.weights.setUni(1);
  
  uintA necessaryObjects;
  determineNecessaryObjects(necessaryObjects, goal, objects, s, true);
  FOR1D(necessaryObjects, i) {
    relDist.weights(relDist.obj2id[necessaryObjects(i)]) = 1000.;
  }
}


void createRelevanceDistribution_random_classwise(RelevanceDistribution& relDist, const TL::State& s, Goal* goal, uintA& objects) {
  double SAMPLE_WEIGHT__HIGH = 1e8;
  double SAMPLE_WEIGHT__LOW = 1e-8;
  
  relDist.init(objects);
  relDist.weights.setUni(SAMPLE_WEIGHT__LOW);
  
  uint start_id = TL::UINT_NIL;
  uint FIX_TYPE = rnd.num(2); // immer abwechselnd blocks and baellscher
  uint i, k;
  
  uint start_size, start_type, start_color;
  while (start_id == TL::UINT_NIL   ||  start_id == _tableID) {
    start_id = objects(rnd.num(objects.N));
    
    FOR1D(object_infos, k) {
      if (start_id == object_infos(k)->id) {
        start_size = object_infos(k)->size;
        start_type = object_infos(k)->type;
        start_color = object_infos(k)->color;
        break;
      }
    }
    if (start_type != FIX_TYPE)
      continue;
  }
    
  uint FIX_TWO = rnd.num(2);
//   FIX_TWO = 0;
  uint FIXED_ATTR = rnd.num(3);
//   uint FIXED_ATTR = 0;
  
  
  
  if (FIX_TWO == 0) {
    FOR1D(objects, i) {
      FOR1D(object_infos, k) {
        if (objects(i) == object_infos(k)->id) {
          break;
        }
      }
      if (FIXED_ATTR == 0) { // type
        if (object_infos(k)->type == start_type)
          relDist.weights(relDist.obj2id[objects(i)]) = SAMPLE_WEIGHT__HIGH;
      }
      else if (FIXED_ATTR == 1) { // size
        if (object_infos(k)->size == start_size)
          relDist.weights(relDist.obj2id[objects(i)]) = SAMPLE_WEIGHT__HIGH;
      }
      else if (FIXED_ATTR == 2) { // color
        if (object_infos(k)->color == start_color)
          relDist.weights(relDist.obj2id[objects(i)]) = SAMPLE_WEIGHT__HIGH;
      }
    }
  }
  else {
    FOR1D(objects, i) {
      FOR1D(object_infos, k) {
        if (objects(i) == object_infos(k)->id) {
          break;
        }
      }
      if (object_infos(k)->size == start_size  &&  object_infos(k)->type == start_type)
        relDist.weights(relDist.obj2id[objects(i)]) = SAMPLE_WEIGHT__HIGH;
    }
  }
  
  // only use clear objects!
  bool isClear;
  FOR1D(objects, i) {
    isClear = PLAN_PARAMS.le->holds(objects(i), MT::String("clear"), s);
    if (!isClear) {
      relDist.weights(relDist.obj2id[objects(i)]) = SAMPLE_WEIGHT__LOW;
    }
  }
  
  PRINT(FIXED_ATTR);
  PRINT(FIX_TWO);
  relDist.write();
}




//      3.134  * type=0 +
//          1.2149 * size=2 +
//          1.4096 * height=0,5,4,1,3 +
//          0.2214 * height=5,4,1,3 +
//              0.2214 * height=4,1,3 +
//                  0.2214 * height=1,3 +
//                      3.7836 * height=3 +
//                          -0.665  * clear=0 +
//                          0      * color +
//                          -1.2045

//   3.1313 * type=0 +
//       1.2562 * size=2 +
//       2.8052 * height=2,0,4,5,1 +
//       0.8864 * height=0,4,5,1 +
//           0.2264 * height=4,5,1 +
//               3.1988 * clear=1 +
//                   0      * color +
//                   -6.529 

void createRelevanceDistribution_learned(RelevanceDistribution& relDist, const TL::State& s, Goal* goal, uintA& objects) {
  relDist.init(objects);
  uint i, k;
  double value;
  uint type, size, height;
  FOR1D(objects, i) {
    FOR1D(object_infos, k) {
      if (object_infos(k)->id == objects(i)) {
        type = object_infos(k)->type;
        size = object_infos(k)->size;
        break;
      }
    }
    height = (uint) PLAN_PARAMS.le->getValue(objects(i), MT::String("height"), s);
    
    value = -6.53;
    
    if (type == 0)
      value += 3.134;
    
    if (size == 2)
      value += 1.26 ;
    
    if (height > 0)
      value += 0.23;
    
//     if (height == 1)
//       value += 0.66;
//     else if (height == 2)
//       value += 0.66;
//     else if (height == 3)
//       value += 4.3;
//     else if (height == 4)
//       value += 0.44;
          
    relDist.weights(relDist.obj2id[objects(i)]) = value;
  }
  norm(relDist.weights);
}





void createRelevanceDistribution_samePile(RelevanceDistribution& relDist, const TL::State& s, Goal* goal, uintA& objects) {
  bool ONLY_DIRECT_CONTACT_AND_ABOVE = true;
  uint i, p, k=0;
  
  relDist.init(objects);
  relDist.weights.setUni(0.001);
  
  uintA necessaryObjects;
  determineNecessaryObjects(necessaryObjects, goal, objects, s, false); // table wird unten eingebaut
  FOR1D(necessaryObjects, i) {
    relDist.weights(relDist.obj2id[necessaryObjects(i)]) = 1000.;
  }
  
  uintA piles;
  TL::logic_world_interface::bw::calcPiles(s, piles, _tableID);
//   PRINT(piles);
  bool found = false;
  FOR1D(necessaryObjects, i) {
    for (p=0; p<piles.d0; p++) {
      for (k=0; k<piles.d1; k++) {
        if (piles(p,k)==necessaryObjects(i)) {
          found = true;
          break;
        }
      }
      if (found) break;
    }
    if (!found) continue; // block might be inhand
    if (ONLY_DIRECT_CONTACT_AND_ABOVE) k=k-1;
    else k=0;
    for (; k<piles.d1; k++) {
      if (piles(p,k) == TL::UINT_NIL)
        break;
      relDist.weights(relDist.obj2id[piles(p,k)]) = 50.;
    }
  }
  
  relDist.weights(relDist.obj2id[_tableID]) = 1000.;

  //   relDist.write();
}







// Prior: Table = 100,000, then big blocks = 10,000, then small blocks = 100, then balls = 1
// + omit blocks which are not clear
void createRelevanceDistribution_bigBlocks(RelevanceDistribution& relDist, const TL::State& s, Goal* goal, uintA& objects) {
  relDist.init(objects);
  relDist.weights.setUni(0.000001);
  
  relDist.weights(relDist.obj2id[_tableID]) = 10e10;
  
  uint SMALL = 1, BIG = 2;
  
  uintA piles;
  TL::logic_world_interface::bw::calcPiles(s, piles, _tableID);
  
  uint i;
  TL::Predicate* p_CLEAR = PLAN_PARAMS.le->getPredicate(MT::String("clear"));
  uint obj, size, height;
  FOR1D(s.pt_derived, i) {
    if (s.pt_derived(i)->pred == p_CLEAR) {
      obj = s.pt_derived(i)->slotAssignments(0);
      if (TL::logic_world_interface::bw::isBlock(obj, s, PLAN_PARAMS.le)) {
        size = (uint) PLAN_PARAMS.le->getValue(obj, MT::String("size"), s);
        height = (uint) PLAN_PARAMS.le->getValue(obj, MT::String("height"), s);
        if (size == SMALL)
          relDist.weights(relDist.obj2id[obj]) = 0.01 * pow(10., height*2);
        else if (size == BIG) {
          relDist.weights(relDist.obj2id[obj]) = 100 * pow(10., height*2);
        }
        else NIY;
        // Wegen der Scheissregeln muessen dann auch die 2 Objekte drunter rein!
        uint below = TL::logic_world_interface::bw::getBelow(obj, s, PLAN_PARAMS.le);
        if (below != UINT_MAX) {
          relDist.weights(relDist.obj2id[below]) = relDist.weights(relDist.obj2id[obj]);
          uint below_zwo = TL::logic_world_interface::bw::getBelow(below, s, PLAN_PARAMS.le);
          if (below_zwo != UINT_MAX) {
            relDist.weights(relDist.obj2id[below_zwo]) = relDist.weights(relDist.obj2id[obj]);
          }
        }
      }
    }
  }  
//   relDist.write();
}












/*



  --------------------------------------------------------
  RELEVANCE GROUNDING
  --------------------------------------------------------




*/



void RelevanceGrounding::initPlanParams(LogicEngine* le, RewardType rewardType, TL::RuleSet& rules, uint no_runs, double discount, uint T, ActionInterface* ai) {
  PLAN_PARAMS.le = le;
  PLAN_PARAMS.rewardType = rewardType;
  PLAN_PARAMS.rules = rules;
  PLAN_PARAMS.no_runs = no_runs;
  PLAN_PARAMS.discount = discount;
  PLAN_PARAMS.T = T;
  PLAN_PARAMS.ai = ai;
  _tableID = TL::UINT_NIL;
}


void RelevanceGrounding::setObjectInfos(const MT::Array< ObjectInfo* >& oinfos) {
  object_infos = oinfos;
}


void ObjectInfo::write(std::ostream& out) {
  out<<id<<" "<<type<<" "<<size<<" "<<color;
}



// Includes for sure (i) NECESSARY objects and (ii) TABLE
void sampleObjects_heightchange(uintA& sampledObjects, const TL::State& s, Goal* goal, RelevanceGrounding::RelevanceType relevanceType, uint subnets_size) {
  RelevanceDistribution relevanceDistribution;
  if (relevanceType == RelevanceGrounding::RANDOM) {
//     createRelevanceDistribution_random(relevanceDistribution, s, goal, PLAN_PARAMS.le->constants);
    createRelevanceDistribution_random_classwise(relevanceDistribution, s, goal, PLAN_PARAMS.le->constants);
//     relevanceDistribution.write();
  }
  else if (relevanceType == RelevanceGrounding::OPTIMAL) {
    createRelevanceDistribution_bigBlocks(relevanceDistribution, s, goal, PLAN_PARAMS.le->constants);
  }
  else if (relevanceType == RelevanceGrounding::LEARNED) {
    createRelevanceDistribution_learned(relevanceDistribution, s, goal, PLAN_PARAMS.le->constants);
  }
  else {NIY;}
  relevanceDistribution.write();
  
  // add necessary objects! includes table !
  uintA necessaryObjects;
  determineNecessaryObjects(necessaryObjects, goal, PLAN_PARAMS.le->constants, s, true);
  sampledObjects.setAppend(necessaryObjects);
  uint i;
  FOR1D(necessaryObjects, i) {
    relevanceDistribution.weights(relevanceDistribution.obj2id[necessaryObjects(i)]) = 0.;
  }

  uint id;
  arr killedWeights = relevanceDistribution.weights;
  arr probs;
  PRINT(killedWeights);
  while (sampledObjects.N < subnets_size) {
    probs = killedWeights;
    normalizeDist(probs);
    id = TL::basic_sample(probs);
    sampledObjects.append(relevanceDistribution.objects(id));
    killedWeights(id)=0.;
  }
  PRINT(necessaryObjects);
  PRINT(sampledObjects);
}




// Includes for sure (i) NECESSARY objects and (ii) TABLE
void sampleObjects_clearance(uintA& sampledObjects, const TL::State& s, Goal* goal, RelevanceGrounding::RelevanceType relevanceType, uint subnets_size) {
  uint DEBUG = 3;
  uint i, k;
  RelevanceDistribution relevanceDistribution;
  if (relevanceType == RelevanceGrounding::RANDOM) {
    createRelevanceDistribution_random(relevanceDistribution, s, goal, PLAN_PARAMS.le->constants);
    // add necessary objects! includes table !
    uintA necessaryObjects;
    determineNecessaryObjects(necessaryObjects, goal, PLAN_PARAMS.le->constants, s, true);
    sampledObjects.setAppend(necessaryObjects);
    FOR1D(necessaryObjects, i) {
      relevanceDistribution.weights(relevanceDistribution.obj2id[necessaryObjects(i)]) = 0.;
    }
    uint id;
    arr killedWeights = relevanceDistribution.weights;
    arr probs;
    while (sampledObjects.N < subnets_size) {
      probs = killedWeights;
      normalizeDist(probs);
      id = TL::basic_sample(probs);
      sampledObjects.append(relevanceDistribution.objects(id));
      killedWeights(id)=0.;
    }
  }
  else if (relevanceType == RelevanceGrounding::LEARNED) {
    createRelevanceDistribution_learned(relevanceDistribution, s, goal, PLAN_PARAMS.le->constants);
    // add necessary objects! includes table !
    uintA necessaryObjects;
    determineNecessaryObjects(necessaryObjects, goal, PLAN_PARAMS.le->constants, s, true);
    sampledObjects.setAppend(necessaryObjects);
    FOR1D(necessaryObjects, i) {
      relevanceDistribution.weights(relevanceDistribution.obj2id[necessaryObjects(i)]) = 0.;
    }
    uint id;
    arr killedWeights = relevanceDistribution.weights;
    arr probs;
    while (sampledObjects.N < subnets_size) {
      probs = killedWeights;
      normalizeDist(probs);
      id = TL::basic_sample(probs);
      sampledObjects.append(relevanceDistribution.objects(id));
      killedWeights(id)=0.;
    }
  }
  else if (relevanceType == RelevanceGrounding::OPTIMAL) {
    double WEIGHT_INIT = 10e-6;
    double WEIGHT_ORDERED = 10e3;
    double WEIGHT_TOP = 10e10;
    double WEIGHT_SECOND = 10e8;
    
    // (1) INIT relevance dist
    relevanceDistribution.init(PLAN_PARAMS.le->constants);
    relevanceDistribution.weights.setUni(WEIGHT_INIT);
    
    // (2) determine RELEVANT objects
    // Goal: pick an UNORDERED guy whose gang is almost ordered (and none of his homies is out)
    TL::Predicate* p_INORDER = PLAN_PARAMS.le->getPredicate(MT::String("inorder"));
    // ordered objects
    uintA objs_inorder;
    FOR1D(s.pt_derived, i) {
      if (s.pt_derived(i)->pred == p_INORDER) {
        objs_inorder.append(s.pt_derived(i)->slotAssignments(0));
      }
    }
    // unordered objects
    uintA objs_un = PLAN_PARAMS.le->constants;
    setMinus(objs_un, objs_inorder);
    if (DEBUG>1) {PRINT(objs_inorder); PRINT(objs_un);}
    FOR1D(objs_un, i) {
      relevanceDistribution.weights(relevanceDistribution.obj2id[objs_un(i)]) = WEIGHT_ORDERED;
    }
    // unordered objects without any class mates off the table (out)
    uintA objs_un_X;
    FOR1D(objs_un, i) {
      uintA gang;
      LogicEngine::getRelatedObjects(gang, objs_un(i), true, *PLAN_PARAMS.le->getPredicate(MT::String("homies")), s);
      gang.append(objs_un(i));
      FOR1D(gang, k) {
        if (TL::logic_world_interface::bw::isOut(gang(k), s, PLAN_PARAMS.le))
          break;
      }
      if (gang.N == k) {
        objs_un_X.setAppend(gang);
//         objs_un_X.append(objs_un(i));
      }
    }
    // inhand
    TL::Predicate* p_INHAND = PLAN_PARAMS.le->getPredicate(MT::String("inhand"));
    uint inhand = TL::UINT_NIL;
    FOR1D(s.pt_prim, i) {
      if (s.pt_prim(i)->pred == p_INHAND) {
        inhand = s.pt_prim(i)->slotAssignments(0);
        break;
      }
    }
    if (DEBUG>1) {PRINT(objs_un); PRINT(objs_un_X); PRINT(inhand);}
    // determine boss = guy who determines the class to take into account
    // boss object = either inhand or a random unordered guy
    uint obj_boss = TL::UINT_NIL;
    if (objs_un_X.N > 0) {
      if (objs_un_X.findValue(inhand) >= 0) {
        obj_boss = inhand;
      }
      else {
        arr boss_probs(objs_un_X.N);
        boss_probs.setUni(0.0);
        FOR1D(objs_un_X, i) {
          if (objs_un_X(i) == _tableID)
            boss_probs(i) = 0.0;
          if (!TL::isZero(boss_probs(i))) { // if class was already taken into account
            continue;
          }
          uintA gang;
          LogicEngine::getRelatedObjects(gang, objs_un_X(i), true, *PLAN_PARAMS.le->getPredicate(MT::String("homies")), s);
          gang.append(objs_un_X(i));
          // calc singles of class (objects that are alone in a tower)
          uint singles = 0;
          FOR1D(gang, k) {
            uint getBelow(uint id, const TL::State& s, LogicEngine* le);
            if (gang.findValue(TL::logic_world_interface::bw::getBelow(objs_un_X(i), s, PLAN_PARAMS.le)) < 0
                &&  gang.findValue(TL::logic_world_interface::bw::getAbove(objs_un_X(i), s, PLAN_PARAMS.le)) < 0)
              singles++;
          }
          FOR1D(gang, k) {
            boss_probs(objs_un_X.findValue(gang(k))) = pow(1.5, (6.-singles)); // NICHT ZU HOCH MACHEN! wegen der kugeln, die runter fallen
//             boss_probs(objs_un_X.findValue(gang(k))) = pow(20., (6.-singles));
          }
          if (DEBUG>2) {PRINT(gang); PRINT(singles); PRINT(boss_probs)}
        }
        obj_boss = objs_un_X(TL::basic_sample(boss_probs));
      }
    }
    else {
      obj_boss = PLAN_PARAMS.le->constants.last();
      MT_MSG("no more unordered objects with all class members on table!!!")
    }
    if (DEBUG>1) {PRINT(obj_boss);}
    // build relevance distribution according to boss
    uintA objs_emphasized;
    objs_emphasized.append(obj_boss);
    if (DEBUG>1) {PRINT(obj_boss);}
    // give high weight to its homies
    TL::Predicate* p_HOMIES = PLAN_PARAMS.le->getPredicate(MT::String("homies"));
    FOR1D(s.pt_prim, i) {
      if (s.pt_prim(i)->pred == p_HOMIES) {
        if (s.pt_prim(i)->slotAssignments(0) == obj_boss) {
          objs_emphasized.setAppend(s.pt_prim(i)->slotAssignments(1));
        }
        else if (s.pt_prim(i)->slotAssignments(1) == obj_boss) {
          objs_emphasized.setAppend(s.pt_prim(i)->slotAssignments(0));
        }
      }
    }
    if (DEBUG>1) {PRINT(objs_emphasized);}
    // give high weights to important friends of homies
    FOR1D(objs_emphasized, i) {
      relevanceDistribution.weights(relevanceDistribution.obj2id[objs_emphasized(i)]) = WEIGHT_TOP;
      // Wegen der Scheissregeln muessen dann auch die 2 Objekte drunter rein!
      uint below = TL::logic_world_interface::bw::getBelow(objs_emphasized(i), s, PLAN_PARAMS.le);
      uint below_zwo = UINT_MAX;
      if (below != UINT_MAX) {
        relevanceDistribution.weights(relevanceDistribution.obj2id[below]) = WEIGHT_TOP;
        below_zwo = TL::logic_world_interface::bw::getBelow(below, s, PLAN_PARAMS.le);
        if (below_zwo != UINT_MAX) {
          relevanceDistribution.weights(relevanceDistribution.obj2id[below_zwo]) = WEIGHT_TOP;
        }
      }
      // die oben drueber sind auch wichtig
      uint above = TL::logic_world_interface::bw::getAbove(objs_emphasized(i), s, PLAN_PARAMS.le);
      if (above != UINT_MAX) {
        relevanceDistribution.weights(relevanceDistribution.obj2id[above]) = WEIGHT_TOP;
      }
      if (DEBUG>2) {PRINT(objs_emphasized(i)); PRINT(below); PRINT(below_zwo); PRINT(above);}
    }
    
    // (3) add NECESSARY objects! includes table !
    uintA necessaryObjects;
    determineNecessaryObjects(necessaryObjects, goal, PLAN_PARAMS.le->constants, s, true);
    sampledObjects.setAppend(necessaryObjects);
    uint i;
    FOR1D(necessaryObjects, i) {
      relevanceDistribution.weights(relevanceDistribution.obj2id[necessaryObjects(i)]) = 0.;
    }
    
    if (DEBUG>1) {relevanceDistribution.write();}
    
    // (4) Finally, sample relevant objects.
    uint id;
    arr killedWeights = relevanceDistribution.weights;
    arr probs;
    while (sampledObjects.N < subnets_size) {
      probs = killedWeights;
      normalizeDist(probs);
      id = TL::basic_sample(probs);
      sampledObjects.append(relevanceDistribution.objects(id));
      killedWeights(id)=0.;
    }
  }
  else {NIY;}
}








void RelevanceGrounding::sampleObjects(uintA& sampledObjects, const TL::State& s, Goal* goal, RelevanceType relevanceType, uint subnets_size) {
  sampledObjects.clear();
  if (relevanceType == TAKE_ALL) {
    sampledObjects = PLAN_PARAMS.le->constants;
    return;
  }
  
  // determine tableID
  if (PLAN_PARAMS.le->constants.findValue(32) >= 0) _tableID = 32;
  else if (PLAN_PARAMS.le->constants.findValue(60) >= 0) _tableID = 60;
  else HALT("no table");
  


#ifdef GOAL__HEIGHT_CHANGE
  sampleObjects_heightchange(sampledObjects, s, goal, relevanceType, subnets_size);
#endif
#ifdef GOAL__CLEARANCE
  sampleObjects_clearance(sampledObjects, s, goal, relevanceType, subnets_size);
#endif
}





void RelevanceGrounding::plan_in_single_subnet(PredIA& plan, double& value, const uintA& objects, const TL::State& s, Goal* goal,  RelevanceType relevanceType) {
  uint DEBUG = 2;
  if (DEBUG>0) {cout<<endl<<"RelevanceGrounding::plan_in_single_subnet [START]"<<endl;}
  
  double T_start, T_end;
  T_start = MT::cpuTime();
  
  TL::State s_small;
  PLAN_PARAMS.le->filterState(s_small, s, objects);
  if (DEBUG>0) {    cout<<"Relevant objects [N="<<objects.N<<"]: "<<objects<<endl;  }
  if (DEBUG>1) {
    if (DEBUG>2) {cout<<"Original state:  ";s.writeNice();cout<<endl;}
    cout<<"Filtered state:  ";s_small.writeNice();cout<<endl;
  }
  ActionSampler sampler(PLAN_PARAMS.le, PLAN_PARAMS.rewardType);
//   TL::State s_copy = s;
  sampler.init(objects, PLAN_PARAMS.rules, PLAN_PARAMS.T, &s_small);
  
  T_end = MT::cpuTime();
  if (DEBUG>0) {
    cerr<<endl<<"Time subnet initing: "<<(T_end - T_start)<<"s."<<endl;
    cout<<"Time subnet initing: "<<(T_end - T_start)<<"s."<<endl;
  }
  T_start = MT::cpuTime();
  
  sampler.generatePlan(plan, value, goal, s_small,
            PLAN_PARAMS.no_runs, PLAN_PARAMS.discount, PLAN_PARAMS.T);
  
  T_end = MT::cpuTime();
  if (DEBUG>0) {
    cerr<<endl<<"Time subnet plan generation: "<<(T_end - T_start)<<"s."<<endl;
    cout<<"Time subnet plan generation: "<<(T_end - T_start)<<"s."<<endl;
  }
  
  if (DEBUG>1) {cout<<"Found plan: "; LogicEngine::write(plan); cout<<endl;}
  if (DEBUG>0) {cout<<"RelevanceGrounding::plan_in_single_subnet [END]"<<endl;}
}






void findNearestNeighbors(uintA& neighbors_nearest, const uintA& houseowners, const uintA& neighbors, uint N) {
  neighbors_nearest.clear();
  uint h, n;
  // (0) remove all houseowners from neighbors
  uintA neighbors_filtered;
  FOR1D(neighbors, n) {
    if (houseowners.findValue(neighbors(n)) < 0)
      neighbors_filtered.append(neighbors(n));
  }
  // (1) determine distances
  double NEUTRAL_DIST = 100000.;
  arr dist(houseowners.N, neighbors_filtered.N);
  dist.setUni(NEUTRAL_DIST);
  FOR1D(houseowners, h) {
    // ignore table
    if (houseowners(h) == _tableID)
      continue;
    FOR1D(neighbors_filtered, n) {
      dist(h, n) = PLAN_PARAMS.ai->euclideanDistance(houseowners(h), neighbors_filtered(n));
    }
  }
//   PRINT(neighbors);
//   PRINT(houseowners);
//   PRINT(neighbors_filtered);
//   PRINT(dist);
  // (2) take N closest
  double min_dist;
  uint min_h, min_n;
  while (neighbors_nearest.N < N) {
    min_dist = NEUTRAL_DIST;
    min_h = min_n = 100000;
    FOR2D(dist, h, n) {
      if (dist(h,n) < min_dist) {
        min_dist = dist(h,n);
        min_h = h;
        min_n = n;
      }
    }
//     PRINT(min_h);
//     PRINT(min_n);
    neighbors_nearest.append(neighbors_filtered(min_n));
    FOR1D(houseowners, h) {
      dist(h, min_n) = NEUTRAL_DIST;
    }
//     PRINT(dist);
  }
//   PRINT(neighbors_nearest);
}







void plan_in_different_subnets(PredIA& plan, double& value, const TL::State& s, Goal* goal, uint subnets_num, uint verification_num, RelevanceGrounding::RelevanceType relevanceType, uint subnets_size, uint verification_size) {
  uint DEBUG = 3;
  
  MT::Array< PredIA > plans(subnets_num);
  arr values_reduced(subnets_num);
  MT::Array< uintA > objects_reduced(subnets_num);
  
  if (subnets_size > PLAN_PARAMS.le->constants.N)
    subnets_size = PLAN_PARAMS.le->constants.N;
  
  uint i, k, l;
  
  double t_start_subnets, t_end_subnets, t_end_verinet;
  t_start_subnets = MT::cpuTime();
  
  // create subnets_num
  for (i=0; i<subnets_num; i++) {
    uintA relevantObjects;
    RelevanceGrounding::sampleObjects(relevantObjects, s, goal, relevanceType, subnets_size);
    uintA relevantObjects_sorted;
    TL::sort_asc(relevantObjects_sorted, relevantObjects);
    relevantObjects = relevantObjects_sorted;
    CHECK(!relevantObjects.containsDoubles(), "some objects too often sampled! "<<relevantObjects);
    objects_reduced(i) = relevantObjects;
    
    PredIA plan;
    double value2;
    RelevanceGrounding::plan_in_single_subnet(plan, value2, relevantObjects, s, goal, relevanceType);
    plans(i) = plan;
    values_reduced(i) = value2;
  }
  
  t_end_subnets = MT::cpuTime();
  if (DEBUG>0) {
    cerr<<endl<<"Time for subnet planning: "<<(t_end_subnets - t_start_subnets)<<"s."<<endl;
    cout<<"Time for subnet planning: "<<(t_end_subnets - t_start_subnets)<<"s."<<endl;
  }
  
  if (subnets_num < verification_num)
    verification_num = subnets_num;
  
  // rank subnets
  uintA ranking_ids(subnets_num); // indiziert nach ranking
  arr ranking_values(subnets_num);  // indiziert nach ranking
  ranking_values.setUni(-1000.);
  FOR1D(plans, i) {
    FOR1D(ranking_ids, k) {
      if (ranking_values(k) < values_reduced(i)) {
        // move other stuff
        for(l=subnets_num-2; l>=k && l!=TL::UINT_NIL; l--) {
          ranking_ids(l+1) = ranking_ids(l);
          ranking_values(l+1) = ranking_values(l);
        }
        // insert current
        ranking_values(k) = values_reduced(i);
        ranking_ids(k) = i;
        break;
      }
    }
  }
  
  if (DEBUG>1) {
    cout<<"Subnet plans:"<<endl;
    FOR1D(plans, i) {
      cout<<values_reduced(i)<<":  ";LogicEngine::write(plans(i));cout<<endl;
    }
    cout<<"Ranked plans:"<<endl;
    FOR1D(ranking_ids, i) {
      cout<<ranking_values(i)<<":  ";LogicEngine::write(plans(ranking_ids(i)));cout<<endl;
    }
  }
  
  // *************************************8
  // VERIFICATION
  arr values_ver(subnets_num); // indiziert nach partial models!
  values_ver.setUni(-10e10);
  MT::Array< uintA > objects_ver(subnets_num); // indiziert nach partial models

// #define LOGGING_FOR_LEARNING
#ifdef LOGGING_FOR_LEARNING
  arr s0_values_ver(subnets_num);
  arr values_doNothing_ver(subnets_num);
#endif
  
  if (verification_num > 0  &&  verification_size > subnets_size) {
    if (DEBUG>1) cout<<endl<<"VERIFICATION in original net:"<<endl;
    for (i=0; i<verification_num ; i++) {
      uint id_subnet = ranking_ids(i);
      ActionSampler sampler(PLAN_PARAMS.le, PLAN_PARAMS.rewardType);
      // determine verification objects
      uintA objects_ver_local;
      if (PLAN_PARAMS.le->constants.N < verification_size)
        objects_ver_local = PLAN_PARAMS.le->constants;
      else {
        uintA neighbors_nearest;
        findNearestNeighbors(neighbors_nearest, objects_reduced(id_subnet), PLAN_PARAMS.le->constants, verification_size-objects_reduced(id_subnet).N);
        objects_ver_local.append(objects_reduced(id_subnet));
        objects_ver_local.append(neighbors_nearest);
      }
      // build verification init state
      objects_ver(id_subnet) = objects_ver_local;
      TL::State s_small_ver;
      PLAN_PARAMS.le->filterState(s_small_ver, s, objects_ver_local);
      // evaluate plan in verification net
      sampler.init(objects_ver_local, PLAN_PARAMS.rules, PLAN_PARAMS.T, &s_small_ver);
      sampler.discount_pow.resize(PLAN_PARAMS.T);
      FOR1D(sampler.discount_pow, k) {
        sampler.discount_pow(k) = pow(PLAN_PARAMS.discount, k);
      }
      sampler.setStartState(s_small_ver);
      sampler.propagatePlan(plans(id_subnet), goal, PLAN_PARAMS.T);
      values_ver(id_subnet) = sampler.evaluate_all(goal, PLAN_PARAMS.discount, PLAN_PARAMS.T);
  
      if (DEBUG>1) {
        cout<<"+++ Verification of "<<i<<"-best plan: +++"<<endl;
        PRINT(objects_ver_local);
        if (DEBUG>2) { cout<<"s0 verification: ";s_small_ver.writeNice();cout<<endl; }
        cout<<"Plan of partial model "<<id_subnet<<": ";LogicEngine::write(plans(id_subnet));cout<<endl;
        cout<<" --> Performance: "<<values_ver(id_subnet)<<"   (vs. performance in partial model: "<<values_reduced(id_subnet)<<")"<<endl;
      }
      
#ifdef LOGGING_FOR_LEARNING
      s0_values_ver(id_subnet) = -10e10;
      FOR1D(s_small_ver.fv_derived, k) {
        if (s_small_ver.fv_derived(k)->f == ((FunctionGoal*) goal)->fvw->f) {
          s0_values_ver(id_subnet) = s_small_ver.fv_derived(k)->value;
          break;
        }
      }
      CHECK(k<s_small_ver.fv_derived.N, "value in start of ver net not found!");
      
      values_doNothing_ver(id_subnet) = 0.;
      for (k=0; k<=PLAN_PARAMS.T; k++) {
        values_doNothing_ver(id_subnet) += pow(PLAN_PARAMS.discount, k) * s0_values_ver(id_subnet);
      }
      
      if (DEBUG>1) {
        PRINT(id_subnet);
        PRINT(s0_values_ver(id_subnet));
        PRINT(values_doNothing_ver(id_subnet));
        PRINT(values_ver(id_subnet));
        double change_ver = values_ver(id_subnet) - values_doNothing_ver(id_subnet);
        PRINT(change_ver);
        PRINT(verification_size);
        double height_change = change_ver * verification_size;
        PRINT(height_change);
      }
#endif

    }
    
    t_end_verinet = MT::cpuTime();
    if (DEBUG>0) {
      cerr<<endl<<"Time for verification: "<<(t_end_verinet - t_end_subnets)<<"s."<<endl;
      cout<<"Time for verification: "<<(t_end_verinet - t_end_subnets)<<"s."<<endl;
    }
  
    uint maxId = values_ver.maxIndex();
    plan = plans(maxId);
    value = values_ver(maxId);
  }
  // NO verification
  else {
    if (DEBUG>1) cout<<"NO verification"<<endl;
    plan = plans(ranking_ids(0));
    value = ranking_values(0);
  }
  
  

#ifdef LOGGING_FOR_LEARNING
  TL::Predicate* p_INHAND = PLAN_PARAMS.le->getPredicate(MT::String("inhand"));
  bool object_inhand = false;
  FOR1D(s.pt_prim, i) {
    if (s.pt_prim(i)->pred == p_INHAND) {
      object_inhand = true;
      break;
    }
  }
  
  if (!object_inhand) {
    uint DEBUG_LOGGING = 3;
    if (DEBUG_LOGGING > 0)
      cout<<"====== LOGGING ======"<<endl;
    CHECK(object_infos.N>0, "no object information provided!");
    CHECK(verification_num == subnets_num, "insufficient verifications");
    
    ofstream log_file;
    log_file.open("relevance.arff", std::ios::app);
    /*
      W.r.t. to plan quality, we are looking at differences in values:  v_t - v_0.
      The value of a plan shall be the sum over differences over all time-steps.
      As the value we get returned from the ActionSampler is a total (and not a
      difference), we have to get rid of the proportion of the intial value.
    */
    
    
    uint q;
    double v_diff_reduced, v_diff_ver;
    double value_portion_s0_reduced;
    
    uint height;
    bool isClear;
    
    FOR1D(plans, i) {
      // find s0 value in reduced net
      // TODO mglw. schon oben berechnen
      TL::State s_reduced;
      PLAN_PARAMS.le->filterState(s_reduced, s, objects_reduced(i));
      double s0_value_reduced;
      FOR1D(s_reduced.fv_derived, k) {
        if (s_reduced.fv_derived(k)->f == ((FunctionGoal*) goal)->fvw->f) {
          s0_value_reduced = s.fv_derived(k)->value;
        }
      }
      // precalculate proportion of inital value in reduced net evaluation
      value_portion_s0_reduced = 0.;
      for (k=0; k<=PLAN_PARAMS.T; k++) {
        value_portion_s0_reduced += pow(PLAN_PARAMS.discount, k) * s0_value_reduced;
      }
      v_diff_reduced = values_reduced(i) - value_portion_s0_reduced;
      v_diff_ver = values_ver(i) - values_doNothing_ver(i);
      v_diff_reduced *= objects_reduced(i).N;
      v_diff_ver *= verification_size;
      
      TL::State s_ver;
      PLAN_PARAMS.le->filterState(s_ver, s, objects_ver(i));
      
      if (DEBUG_LOGGING > 0) {
        cout<<"Evaluating subplan with relevant objects "<<objects_reduced(i)<<endl;
        PRINT(values_reduced(i)); PRINT(values_ver(i));
        PRINT(s0_value_reduced);  PRINT(value_portion_s0_reduced);
        PRINT(v_diff_reduced);  PRINT(v_diff_ver);
      }
  //     log_file<<"# ";LogicEngine::write(plans(i), log_file);log_file<<endl;
      // write relevance data for objects
      FOR1D(objects_reduced(i), k) {
        if (objects_reduced(i)(k) == _tableID) // don't use table
          continue;
        FOR1D(object_infos, q) {
          if (object_infos(q)->id == objects_reduced(i)(k))
            break;
        }
        CHECK(q<object_infos.N, "object has information attached: "<<objects_reduced(i)(k));
        height = (uint) PLAN_PARAMS.le->getValue(objects_reduced(i)(k), MT::String("height"), s_ver);
        isClear = PLAN_PARAMS.le->holds(objects_reduced(i)(k), MT::String("clear"), s_ver);
        log_file
//                 <<object_infos(q)->id<<","
                <<object_infos(q)->type
                <<","<<object_infos(q)->size
                <<","<<height
                <<","<<isClear
                <<","<<object_infos(q)->color
//                 <<","<<v_diff_reduced
                <<","<<v_diff_ver
                <<endl;
      }
    }
  //   log_file<<endl;
    log_file.close();
  //   exit(0);
    
    /*
    ERKLAERUNGEN:
    - Es ist zu erwarten, dass v_diff_ver groesser als v_diff_reduced ist im Falle des heightchange-Goals.
      Denn aufgrund des Noise outcomes hat jeder Block eine gewisse erwartete Hoehe in der simulierten
      Zukunft -- nicht nur der manipulierte Block. Das wird aber nicht rausgerechnet, falls man die
      Durchschnittsbildung aufhebt durch Dranmultiplizieren der Objektanzahl.
      Beispiel: Sei \alpha die erwartete Hoehe eines nicht-manipulierten Objekts nur aufgrund des Noise-Outcomes.
      Dann gilt bei 2 Zeitschritten ungefaehr:   average_height = (1 + (N-1) \alpha ) / N.
      Dann ist:    height = N * average_height  =  1 + (N-1) \alpha.
      Die Groesse des betrachtetens Netzs, N, spielt also eine enorme Rolle.
    */
  }
#endif

  if (DEBUG>0) cout<<"Found big plan:  "<<value<<" ";LogicEngine::write(plan);cout<<endl;
}


void RelevanceGrounding::plan(PredIA& plan, const TL::State& s, Goal* goal, uint subnets_num, uint verification_num, RelevanceType relevanceType, uint subnets_size, uint verification_size) {
//   cout<<endl<<endl;
//   cout<<"RELEVANCE GROUNDING [OVERALL START]"<<endl;
  double THRESHOLD = 0.001;
  uint MAX_PLANS = 1000;
  double value = 0.;
  uint plans = 0;
  while (value < THRESHOLD) {
    plan_in_different_subnets(plan, value, s, goal, subnets_num, verification_num, relevanceType, subnets_size, verification_size);
    if (plans++ > MAX_PLANS) {
      break;
    }
//     PRINT(value);
  }
//   cout<<"RELEVANCE GROUNDING [OVERALL END]"<<endl;
}



#endif
