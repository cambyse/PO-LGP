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

#ifndef TL__BW_LANGUAGE
#define TL__BW_LANGUAGE

#include "TL/logicEngine.h"
#include "TL/plan.h"


// Blocksworld-IDs
// Specify primitives before derived

// Primitive predicates
#define HAND_ID__PRED_TABLE 11
#define HAND_ID__PRED_BLOCK 12
#define HAND_ID__PRED_BALL 13
#define HAND_ID__PRED_ON 14
#define HAND_ID__PRED_INHAND 15
#define HAND_ID__PRED_UPRIGHT 16
#define HAND_ID__PRED_OUT 17
#define HAND_ID__PRED_HOMIES 18
#define HAND_ID__PRED_BOX 19
#define HAND_ID__PRED_CONTAINS 20
#define HAND_ID__PRED_CLOSED 21

// Comparisons
#define HAND_ID__PRED_COMP_CONSTANT 31
#define HAND_ID__PRED_COMP_DYNAMIC 32

// Derived predicates
#define HAND_ID__PRED_CLEAR 41
#define HAND_ID__PRED_INHAND_NIL 42
#define HAND_ID__PRED_UPRIGHT_BLOCK 43
#define HAND_ID__PRED_NOUT_CLEAR 44
#define HAND_ID__PRED_ABOVE 45
#define HAND_ID__PRED_SEVERAL_OBJS_OUT 46
#define HAND_ID__PRED_ABOVE_SEVERAL 47
#define HAND_ID__PRED_BELOW_FOUR 48
#define HAND_ID__PRED_SEVERAL_SMALL 49
#define HAND_ID__PRED_ABOVE_NOTABLE 50
#define HAND_ID__PRED_DIRTY_GUY_BELOW 51
#define HAND_ID__PRED_DIFF_TOWER 52
#define HAND_ID__PRED_WITHOUT_HOMIES 53
#define HAND_ID__PRED_INORDER 54
#define HAND_ID__PRED_ON_BOX 55

// Primitive functions
#define HAND_ID__FUNCTION_SIZE 71

// Derived functions
#define HAND_ID__FUNCTION_HEIGHT 81
#define HAND_ID__FUNCTION_AVG_HEIGHT 82
#define HAND_ID__FUNCTION_SUM_HEIGHT 83
#define HAND_ID__FUNCTION_COUNT_INORDER 84

// Actions
#define HAND_ID__GRAB 1
#define HAND_ID__PUTON 2
#define HAND_ID__LIFT 3
#define HAND_ID__PLACE 4
#define HAND_ID__OPEN_BOX 5
#define HAND_ID__CLOSE_BOX 6


namespace TL {
  
namespace bwLanguage {
  
  /* ---------------------
    LANGUAGE DESCRIPTION
  --------------------- */
  
  // logic engine with blocksworld vocabulary
  TL::LogicEngine* createLogicEngine(uintA& constants); // adapt this to your needs!!
  void shutdownLogic();
  
  // CONCEPTS
  // Concept creation for basic blocksworld vocabulary
//   void createActions(PredA& actions);
//   void createPrimitives(PredA& ps, PredA& p_comparisons, FuncA& fs);
//   void createDerived(PredA& ps, FuncA& fs, LogicEngine* le);
  
  // predicates  -  primitives
  TL::Predicate* getPredicate_table();
  TL::Predicate* getPredicate_block();
  TL::Predicate* getPredicate_ball();
  TL::Predicate* getPredicate_box();
  TL::Predicate* getPredicate_on();
  TL::Predicate* getPredicate_inhand();
  TL::Predicate* getPredicate_upright();
  TL::Predicate* getPredicate_out();
  TL::Predicate* getPredicate_homies();
  TL::Predicate* getPredicate_contains();
  TL::Predicate* getPredicate_closed();
  
  // predicates  -  comparisons
  TL::ComparisonPredicate* getPredicate_comparison_constant();
  TL::ComparisonPredicate* getPredicate_comparison_dynamic();
  
  // predicates  -  actions
  TL::Predicate* getPredicate_action_default();
  TL::Predicate* getPredicate_action_grab();
  TL::Predicate* getPredicate_action_puton();
  TL::Predicate* getPredicate_action_lift(); // 2 args
  TL::Predicate* getPredicate_action_place(); // 2 args
  TL::Predicate* getPredicate_action_openBox();
  TL::Predicate* getPredicate_action_closeBox();
  
  // predicates  -  derived
  TL::ConjunctionPredicate* getPredicate_clear();
  TL::TransClosurePredicate* getPredicate_above();
  TL::ConjunctionPredicate* getPredicate_aboveNotable();
  TL::ConjunctionPredicate* getPredicate_dirtyGuyBelow();
  TL::ConjunctionPredicate* getPredicate_diffTower();
  TL::ConjunctionPredicate* getPredicate_withoutHomies();
  TL::ConjunctionPredicate* getPredicate_inorder();
  TL::ConjunctionPredicate* getPredicate_inhandNil();
  TL::ConjunctionPredicate* getPredicate_onBox();

  
  // functions  -  primitives
  TL::Function* getFunction_size();
  
  // functions  -  derived
  TL::CountFunction* getFunction_height();
  TL::AverageFunction* getFunction_avgheight();
  TL::SumFunction* getFunction_sumheight();
  TL::CountFunction* getFunction_countInorder();
  
  
  
  /* ---------------------
      Instance creation
  --------------------- */
  
  FunctionValue* createFunctionValue_size(uint obj, double size, LogicEngine* le);
  PredicateInstance* createPredicateInstance_on(uint above, uint below, LogicEngine* le);
  PredicateInstance* createPredicateInstance_inhand(uint obj, LogicEngine* le);
  PredicateInstance* createPredicateInstance_table(uint obj, LogicEngine* le);
  PredicateInstance* createPredicateInstance_block(uint obj, LogicEngine* le);
  PredicateInstance* createPredicateInstance_box(uint obj, LogicEngine* le);
  PredicateInstance* createPredicateInstance_homies(uint obj1, uint obj2,  LogicEngine* le);
  PredicateInstance* createPredicateInstance_ball(uint obj, LogicEngine* le);
  PredicateInstance* createPredicateInstance_upright(uint obj, LogicEngine* le);
  PredicateInstance* createPredicateInstance_out(uint obj, LogicEngine* le);
  PredicateInstance* createPredicateInstance_grab(uint obj, LogicEngine* le);
  PredicateInstance* createPredicateInstance_puton(uint obj, uint on, LogicEngine* le);
  PredicateInstance* createPredicateInstance_inhand_nil(LogicEngine* le);
  PredicateInstance* createPredicateInstance_contains(uint box, uint obj, LogicEngine* le);
  PredicateInstance* createPredicateInstance_closed(uint box, LogicEngine* le);

  
  
  /* ---------------------
     Rewards
  --------------------- */
  namespace RewardLibrary {
    Reward* stack(LogicEngine*);
    Reward* on(uint o1, uint o2, LogicEngine*);
    Reward* inhand(uint o1, LogicEngine*);
    Reward* tower(uintA& tower_objects, LogicEngine*);
    Reward* clearance(LogicEngine*);
  };
  
  
  // Goal state sampling
  TL::PredicateListReward* sampleGroundGoal__stack(const uintA& blocks, const uintA& balls, uint table_id, LogicEngine& le, bool leave_existing_towers = false, TL::State* = NULL);
  TL::PredicateListReward* sampleGroundGoal__clearance(const TL::State& current_state, uint table_id, LogicEngine& le);

  
  
  /* ---------------------
    State information helpers
  --------------------- */
  
  bool isBlock(uint id, const TL::State& s, LogicEngine* le);
  bool isBall(uint id, const TL::State& s, LogicEngine* le);
  bool isBox(uint id, const TL::State& s, LogicEngine* le);
  bool isTable(uint id, const TL::State& s, LogicEngine* le);
  bool isOut(uint id, const TL::State& s, LogicEngine* le);
  bool isInhand(uint id, const TL::State& s, LogicEngine* le);
  bool isClosed(uint box_id, const TL::State& s, LogicEngine* le);
  bool isInorderGang(const uintA gang, const TL::State& s, LogicEngine* le);
  void getBelowObjects(uintA& ids, uint id, const TL::State& s, LogicEngine* le);
  void getAboveObjects(uintA& ids, uint id, const TL::State& s, LogicEngine* le); // directly above!!!
  uint getBelow(uint id, const TL::State& s, LogicEngine* le);
  uint getAbove(uint id, const TL::State& s, LogicEngine* le); // directly above!!!
  uint getInhand(const TL::State& s, LogicEngine* le);
  void getBoxes(uintA& ids, const TL::State& s, LogicEngine* le);
  uint getContainingBox(uint obj_id, const TL::State& s, LogicEngine* le);
  uint getContainedObject(uint box_id, const TL::State& s, LogicEngine* le);
  void getHomieGangs(MT::Array< uintA >& homieGangs, const TL::State& s, LogicEngine* le);
  
    // Reward functions
  double reward_buildTower(const State& s);
  void calcSkyscraperWeights(const uintA& objects, const uintA& piles, double skyscraper_bias, arr& weights, bool highGood, uint id_table);
  void calcSkyscraperWeights(const uintA& heights, double skyscraper_bias, arr& weights, bool highGood, uint id_table);
  // Piles
  // sorted by heights! piles(0)=highest
  void calcPiles(const State& s, uintA& piles, uint id_table);
  void calcHeights(const uintA& objects, const uintA& piles, uintA& object_heights, uint id_table);

  
  // Nice state information
  void writeStateInfo(const State& s, LogicEngine* le, ostream& out = std::cout);
}

}

#endif // TL__BW_LANGUAGE
