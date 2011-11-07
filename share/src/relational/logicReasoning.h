/*  
    Copyright 2011   Tobias Lang
    
    E-mail:    tobias.lang@fu-berlin.de
    
    This file is part of libPRADA.

    libPRADA is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your olition) any later version.

    libPRADA is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with libPRADA.  If not, see <http://www.gnu.org/licenses/>.
*/


#ifndef TL__LOGIC_REASONING
#define TL__LOGIC_REASONING

#include <relational/logicDefinitions.h>



namespace TL {




namespace logicReasoning {
	

  
  /****************************************
      BASIC HELPERS
   ***************************************/
  
  uint calcNumTerms(const TL::Atom& a);
  uint calcNumTerms(const TL::Literal& lit);
  uint getArguments(uintA& args, const TL::Literal& lit);
  uint calcTerms(const TL::Atom& a, uintA& terms);
  uint calcTerms(const TL::Literal& lit, uintA& terms);
  uint calcTerms(const AtomL& as, uintA& terms);
  uint calcTerms(const LitL& lits, uintA& terms);
  void calcFreeVars(const TL::ConjunctionPredicate& scp, uintA& freeVars_pos, uintA& freeVars_neg);
  void calcUnconstrainedNegatedArguments(TL::ConjunctionPredicate* cp, uintA& vars);
  bool containsNegativeBasePredicate(TL::Predicate* cp);
  bool isEqualityLiteral(TL::Literal* lit);
  uint numberLiterals(const MT::Array< LitL >& LitLs);
  
  // lists
  void getConstants(const FuncVL& fvs, uintA& constants);
  void getConstants(const LitL& lits, uintA& constants);
  bool containsLiteral(const LitL& p, const TL::Literal& literal);
  bool containsNegativeLiterals(const LitL& LitLs);
  void getUnconstrainedNegatedArguments(uintA& args, const LitL& lits);
  void filterPurelyAbstract(const LitL& allPreds, LitL& filteredPreds);
  void negate(const LitL& lits, LitL& lits_negated);
  int findPattern(const AtomL& actions, uint minRepeats);
  
  double getValue(const FunctionAtom& fa, const FuncVL& fvs);
  
  // sort lists
  void sort(LitL& lits); // order
  void sort(AtomL& as); // order
  void sortNegativesUngroundedLast(LitL& lits);
  bool negativesLast(const LitL& lits);
  bool negativesUngroundedLast(const LitL& lits);
  
  // in case of free vars, instantiates all possible guys!
  void generateBaseInstantiations(LitL& lits_base, FuncVL& fvs_base, TL::Literal* lit);
  void generateBaseInstantiations(LitL& lits_base, FuncVL& fvs_base, TL::Literal* lit, uintA& free_constants);
  
  
  /****************************************
     HELPERS FOR STATE
   ***************************************/
  void getConstants(const TL::State& s, uintA& constants);
  uint getArgument(const TL::State& s, const TL::Predicate& pred);
  void getArguments(uintA& args, const TL::State& s, const TL::Predicate& pred);
  void usedValues(const TL::Function& f, const TL::State& s, arr& values);
  bool containsNegativeLiterals(const TL::State& s);
  void filterState_full(TL::State& s_filtered, const TL::State& s_full, const uintA& filter_objects, bool primOnly = true);  // only "filter_objects" as args
  void filterState_atleastOne(TL::State& s_filtered, const TL::State& s_full, const uintA& filter_objects, bool primOnly = true); // at least one in "filter_objects" as arg
  void changes(const TL::State& pre, const TL::State& post, uintA& changedConstants, LitL& holdOnlyPre, LitL& holdOnlyPost);
  // pred(X,Y), pred(X,Z) --> {Y,Z}
  void getRelatedObjects(uintA& objs_related, uint id, bool id_covers_first, const TL::Predicate& pred, const TL::State& s);
  // TODO hieran weiter arbeiten fuer den Graphen
  void getGeneralRelatedObjects(uintA& objs_related, uint id, const TL::State& s);
  void getValues(arr& values, const TL::State& s, const TL::Function& f, const uintA& objs);
  double getValue(uint id, const MT::String& function_name, const TL::State& s);
  double getValue(TL::Function* f, const TL::State& s);
  double getValue(uint id, TL::Function* f, const TL::State& s);
  double getValue(const uintA& args, TL::Function* f, const TL::State& s);
  double getValue(const TL::FunctionAtom* fa, const TL::State& s);
  
  
    
  
	
  /****************************************
    ABSTRACTING AND GROUNDING
  ***************************************/
  
  bool isConstant(uint id);
  bool isGrounded(const TL::Atom* a);
  bool isGrounded(const TL::Literal* lit);
  bool isGrounded(const TL::FunctionAtom* fa);
  bool isGrounded(const TL::FunctionValue* funcV);
  bool isAbstract(const TL::Atom* a);
  bool isAbstract(const TL::Literal* lit);
	

    
  /****************************************
      MISC BASIC LOGIC
  ***************************************/
    
  bool nonContradicting(const LitL& p1, const LitL& p2);
  void removeRedundant(LitL& p);
  
  
  
  /****************************************
      SUBSTITUTIONS
  ***************************************/
    
  // original means: orignal objects of logicEngine
  TL::Atom* applyOriginalSub(TL::Substitution& sub, TL::Atom* lit);
  TL::Literal* applyOriginalSub(TL::Substitution& sub, TL::Literal* lit);
  void applyOriginalSub(TL::Substitution& sub, const LitL& unsubPreds, LitL& subPreds);
  
  void createInverseSubstitution(const TL::Atom& a, TL::Substitution& sub);
  void createInverseSubstitution(const TL::Literal& lit, TL::Substitution& sub);
  void createAllPossibleSubstitutions(const uintA& vars, TL::SubstitutionSet& subs);
   
  
  
  /****************************************
    MISC ADVANCED LOGIC
  ***************************************/
    
  void determineStupidDerivedConcepts(MT::Array< TL::Trial* >& data, PredL& stupidPredicates, FuncL& stupidFunctions, bool deleteFromLogicEngine);
    
//     delete sons of each ConjunctionLiteral without free variables
  void killBaseConcepts(LitL& lits);
  
  
    
    
  /****************************************
      HOLDS (--> for grounded)
  ***************************************/

  // GOAL: Coverage for grounded concelit instances (-> simple existence check)
  
  // Allows for positive and negative predicate instances.
  // In contrast to "cover(.)" which covers abstract / ungrounded predicates (see below)
  // Assumptions:
  // (1) Everything is grounded. (States, Literals)
  // (2) Knowledge base "grounded_lits" contains only positive literals!!
  bool holds_positive(const LitL& positive_grounded_lits, TL::Literal* lit);
  bool holds(const LitL& positive_grounded_lits, TL::Literal* lit);
  // special function for ComparisonLiterals
  bool holds(const FuncVL& fvs, TL::ComparisonLiteral* lit);
  // Additional assumptions for State variants:
  // (1) All complex predicate tuples have been derived in the states.
  // --> simple search
  bool holds(const TL::State& s, TL::Literal* grounded_lit);
  bool holds(const TL::State& s, const LitL& grounded_lits);
  // Attention: we look only at the primitive predicates thus far!!
  bool holds(const TL::State& s1, const TL::State& s2);
    
  // "holds_straight(.)": fast variant of "holds(TL::State* s, TL::Literal* grounded_pi)"
  bool holds_straight(uint id, const MT::String& predicate_name, const TL::State& s);
	
  
  /****************************************
          UNIFY
    ***************************************/
  
  // "unify" unifies literals.
  // "cover" uses "unify" to check coverage of literals in states.
    
  bool unify_basic(const uintA& grounded_args, const uintA& other_args, TL::Substitution& sub);
  bool unify(const TL::Atom* ground_lit, const TL::Atom* other_lit, TL::Substitution& sub);
  bool unify(const TL::Literal* ground_lit, const TL::Literal* other_lit, TL::Substitution& sub);
  // used for constant-bound clit
  bool unify(TL::FunctionValue* grounded_fv, TL::ComparisonLiteral* clit_constant, TL::Substitution& sub);
  // used for dynamic-bound clit
  bool unify(TL::FunctionValue* grounded_fv_left, TL::FunctionValue* grounded_fv_right, TL::ComparisonLiteral* clit_dynamic, TL::Substitution& sub);

  
  /****************************************
      COVER (--> for abstract)
   ***************************************/
  
  // GOAL: Coverage for abstract concelit instance arrays
  
  // SEMANTICS
  // Free variables:
  // In "cover", free variables are dealt with depending upon the literal they are involved in:
  // (i)  Positive literals: existential quantification, i.e., we need to find a single substitution
  //      for the literal to hold
  // (ii) Negative literals: there are two possibilities specified by the "freeNegVarsAllQuantified"-flag.
  //      In either case, free variables in negative literals lead to a substantial computational burden
  //      and should be avoided. Therefore, we usually order Literal arrays such that
  //      the positive literals appear first (which bind many variables).
  //      (A) All-quantification:
  //          A predicate tuple -p(X1,...,Xn) is only true if p does not hold for any
  //          substitution of the free variables.
  //      (B) Existential-quantification:
  //          A predicate tuple -p(X1,...,Xn) is true with a substition S={X1->c1,...,Xn-->cn}
  //          such that p(c1,...,cn) does not hold.
  
  // USAGE
  // Substitution object instances:
  // - All substitutions in "subs" arrays found by "cover" are newly created Substition instances. It is
  //   the responsibility of the calling methods to delete them!
  // - If a cover method returns false, the "subs" array is supposed to be emlity. (So, the caller does
  //   not have to care about deleting object instances.)
    
  // BASIC FUNCTION 1
	// Calculates substitutions "subs" (extensions of "initSub") which
	// substitute the arguments in "lit" such that the resulting
	// grounded predicate is satisfied in state "s".
  bool cover(const TL::State& s, TL::Literal* lit, TL::SubstitutionSet& subs, bool freeNegVarsAllQuantified, TL::Substitution* initSub = NULL);
  // BASIC FUNCTION 2
	// Calculates substitutions "subs" (extensions of "initSub") which
	// substitute the arguments in the predicate list "lits" such that the resulting
	// grounded predicates are satisfied in state "s".
  bool cover(const TL::State& s, const LitL& lits, TL::SubstitutionSet& subs, bool freeNegVarsAllQuantified, TL::Substitution* initSub = NULL);
    
    
    
	
  /****************************************
            STATE UNIFICATION 
   ***************************************/
  void calcDifferences(LitL& pi_diff_1to2, FuncVL& fv_diff_1to2, LitL& pi_diff_2to1, FuncVL& fv_diff_2to1, const TL::State state1, const TL::State state2);
  uint createLeastGeneralSuperState(const TL::Atom& action1, const TL::State& state1, const TL::Atom& action2, const TL::State& state2);
  // returns min-difference in number of literals
  uint unifyAsMuchAsPossible(SubstitutionSet& subs, const TL::State& state1, const TL::State& state2, TL::Substitution* initSub = NULL);
  bool unify(SubstitutionSet& subs, const TL::State& state1, const TL::State& state2, TL::Substitution* initSub = NULL);
  bool unifiable(const TL::State& state1, const TL::State& state2);
  
  uint unifyAsMuchAsPossible(SubstitutionSet& subs, const TL::State& state1, const TL::Atom& action1, const TL::State& state2, const TL::Atom& action2);

	
  /****************************************
      DERIVING COMPLEX
    ***************************************/
	
  // specialized methods
  bool deriveLiterals_conjunction(TL::ConjunctionPredicate& p, TL::State& s);
  bool deriveLiterals_transClosure(TL::TransClosurePredicate& p, TL::State& s);
  bool deriveLiterals_count(TL::CountPredicate& p, TL::State& s);
  bool deriveFunctionValues_count(TL::CountFunction& f, TL::State& s);
  bool deriveFunctionValues_avg(TL::AverageFunction& f, TL::State& s);
  bool deriveFunctionValues_max(TL::MaxFunction& f, TL::State& s);
  bool deriveFunctionValues_sum(TL::SumFunction& f, TL::State& s);
  bool deriveFunctionValues_reward(TL::RewardFunction& f, TL::State& s);
    
  void derive(const LitL& pi_prim, const FuncVL& fv_prim, LitL& pi_derived, FuncVL& fv_derived);
  void derive(TL::State* s);
  
  void dederive(TL::State* s);

  
};


}



#endif // TL__LOGIC_REASONING
