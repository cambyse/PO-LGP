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


#ifndef TL__LOGIC_OBJECT_MANAGER
#define TL__LOGIC_OBJECT_MANAGER

#include <relational/logicDefinitions.h>


namespace TL {
  

class ConceptDependencyGraph;



namespace logicObjectManager {
  
  /****************************************
     SET AND GET:  PREDICATES, FUNCTIONS, CONSTANTS
   ***************************************/
  
  void setConstants(uintA& constants);
  void setConstants(uintA& constants, const TermTypeL& constants_types);
  
  void init(const char* language_file, uint fileType = 1);
  
  void addActionPredicates(PredL& actions);
  void addStatePredicates(PredL& preds);
  void addStateFunctions(FuncL& funcs);
    
  TL::Predicate* getPredicate(const MT::String& name);
  TL::Function* getFunction(const MT::String& name);
  TL::Predicate* getPredicate(uint id);
  TL::Function* getFunction(uint id);
  
  void getAllPrecessors(const TL::Predicate& p, PredL& pre_preds, FuncL& pre_funcs);
  void getAllPrecessors(const TL::Function& f, PredL& pre_preds, FuncL& pre_funcs);
  
  void shutdown();
  void writeLanguage(const char* filename);
  
  uint getLowestFreeConceptID(uint min);
  
  
  /****************************************
      GET:  ATOMS AND LITERALS
  ***************************************/
  
  // Atoms
  TL::Atom* getAtomOrig(TL::Atom* a_copy);
  TL::Atom* getAtom(TL::Predicate* pred, uintA& args);
  TL::Atom* getAtom(const char* text);
  TL::Atom* getAtom_doNothing();
  void getAtoms(AtomL& as, const uintA& arguments);
  void getAtoms(AtomL& as, TL::Predicate* pred, const uintA& arguments);
  void getAtoms(AtomL& as, const char* text);
  void getAtoms_actions(AtomL& as_actions, const uintA& arguments);  // get all action atoms
  
  // ComparisonAtoms
  TL::ComparisonAtom* getCompAtom_constant(TL::Function* f, ComparisonType compType, double bound, uintA& args);
  TL::ComparisonAtom* getCompAtom_dynamic(TL::Function* f, ComparisonType compType, uintA& args1, uintA& args2);
  
  // Literals
  TL::Literal* getLiteralOrig(TL::Literal* lit_copy);
  TL::Literal* getLiteralNeg(TL::Literal* lit);
  TL::Literal* getLiteral(TL::Predicate* pred, bool positive, uintA& args);
  TL::Literal* getLiteral(const char* text);
  TL::Literal* getLiteral(TL::Atom* atom);
  void getLiterals(LitL& lits, const uintA& arguments, const bool positiveOnly = false);
  void getLiterals(LitL& lits, const uintA& arguments, const uintA& arguments_mustBeContained, const bool positiveOnly = false);
  void getLiterals(LitL& lits, TL::Predicate* pred, const uintA& arguments, const bool positiveOnly = false);
  void getLiterals(LitL& lits, const char* text);

  // derived literals
  TL::Literal* getConjunctionLiteral(TL::ConjunctionPredicate* cp, TL::Substitution* sub);
  TL::Literal* getTransClosureLiteral(TL::TransClosurePredicate* p, uintA& args);
  TL::Literal* getCountLiteral(TL::CountPredicate* p, uintA& args);
  
  // ComparisonLiterals
  TL::ComparisonLiteral* getCompLiteral_constant(TL::Function* f, ComparisonType compType, double bound, uintA& args);
  TL::ComparisonLiteral* getCompLiteral_dynamic(TL::Function* f, ComparisonType compType, uintA& args1, uintA& args2);
  // what = 0 --> equality
  // what = 1 --> less than[?]
  // what = 2 --> greater than[?]
  // what = 3 --> all, even compare two objects
  // all p_comp to constants!
  void getCompLiterals_constantBound(LitL& pis, const uintA& args, const TL::SymbolicState& s, uint what);
  void getCompLiterals_dynamicBound(LitL& lits, const uintA& args, const TL::SymbolicState& s, uint what);
  
  // FunctionAtoms
  TL::FunctionAtom* getFAorig(TL::FunctionAtom* fi_copy);
  TL::FunctionAtom* getFA(TL::Function* f, const uintA& args);
  TL::FunctionAtom* getFA(const char* text);
  void getFAs(FuncAL& fis, TL::Function* func, const uintA& args);
  void getFAs(FuncAL& fis, const uintA& args);
  
  // FunctionValues
  TL::FunctionValue* getFVorig(TL::FunctionValue* fv_copy);
  TL::FunctionValue* getFV(TL::Function* f, uintA& args, double value);
  TL::FunctionValue* getFV(const char* text);
  void getFVs(FuncVL& pis, const char* text);

  // TermType
  TL::TermType* getTermTypeOfObject(uint object_id);
    
  
  
  
  /****************************************
      MAKE ORIGINAL:  STATE AND TRIAL
  ***************************************/
  
  // "original" means: replace Literal-objects by the objects of
  // the logicObjectManager if required.
  
  void makeOriginal(TL::SymbolicState& s);
  void makeOriginal(TL::Trial& t);
  void makeOriginal(TL::Rule& r);
  


  /****************************************
         READING
   ***************************************/
  
  // Basic Language
  // eats whole line
  TL::Predicate* readPredicate(ifstream& in, uintA& baseIds);
  // eats whole line
  TL::Function* readFunction(ifstream& in, uintA& baseIds);
  // eats whole line
  TL::TermType* readTermType(ifstream& in, const TermTypeL& existing_types);

  void readLanguage(const char *filename, PredL& p_prim, PredL& p_derived, PredL& actions, FuncL& f_prim, FuncL& f_derived, TermTypeL& types);
  void readLanguage(ifstream& in, PredL& p_prim, PredL& p_derived, PredL& actions, FuncL& f_prim, FuncL& f_derived, TermTypeL& types);
  void readLanguageSimpleFormat(const char *filename, PredL& p_prim, PredL& p_derived, PredL& actions, FuncL& f_prim, FuncL& f_derived, TermTypeL& types);

  
  // Complex Constructs
  TL::Trial* readTrial_withConstants(const char* filename, bool take_constants);
  
  TL::Rule* readRule(ifstream& in);
  void readRules(const char* filename, RuleSet& rules);
  void readRules(ifstream& in, RuleSet& rules);

  
  
   
  
  /****************************************
         RELATIONAL LOGIC LANGUAGE
   ***************************************/
  
  // should not be set directly!
  
  extern uintA constants;
  extern TermTypeL constants_types;
  
  extern PredL p_actions;
  extern PredL p_prim;
  extern PredL p_derived;
  extern ComparisonPredicate p_comp_constant;
  extern ComparisonPredicate p_comp_dynamic;
  extern FuncL f_prim;
  extern FuncL f_derived;
  extern TermTypeL types;
  
   
  extern ConceptDependencyGraph dependencyGraph;
  
}






/****************************************
     DEPENDENCY GRAPH
     (for derived concepts)
 ***************************************/

class ConceptDependencyGraph {
  boolA edges;
  uint derivedStartID;
  boolA id2pred;
  std::map<uint, uint> pred2graph;
  std::map<uint, uint> func2graph;
  std::map<uint, uint> graph2pred;
  std::map<uint, uint> graph2func;
  PredL predicates;
  FuncL functions;
  
  uintA order, order__derOnly;
  boolA order_isPredicate, order_isPredicate__derOnly;
  void calcWellDefinedOrder(uintA& order, boolA& isPredicate, bool derivedOnly);
  
  public:
    TL::Predicate* getPredicate(uint id) const;
    TL::Function* getFunction(uint id) const;
    
    void resetDefault();
    void setNodes(PredL& predicates, FuncL& functions);

    bool isAcyclic();

    void getAllPrecessors(const TL::Predicate& p, PredL& pre_preds, FuncL& pre_funcs);
    void getAllPrecessors(const TL::Function& f, PredL& pre_preds, FuncL& pre_funcs);
    void getAllSuccessors(const TL::Predicate& p, PredL& suc_preds, FuncL& suc_funcs);
    void getAllSuccessors(const TL::Function& f, PredL& suc_preds, FuncL& suc_funcs);
    
    // order on functions and predicates according to dependencies
    // (richtige bezeichnung dafuer grad meinem gehirn entfallen)
    void getWellDefinedOrder(uintA& order, boolA& isPredicate, bool derivedOnly);
    
    void writeNice(ostream& os = std::cout);
};







}


#endif // TL__LOGIC_OBJECT_MANAGER
