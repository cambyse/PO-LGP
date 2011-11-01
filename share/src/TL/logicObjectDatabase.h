/*  
    Copyright 2011   Tobias Lang
    
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


#ifndef TL__LOGIC_OBJECT_DATABASE
#define TL__LOGIC_OBJECT_DATABASE

#include <TL/logicDefinitions.h>



namespace TL {

extern uint LOGIC_ENGINE__MAX_ID_NUMBER;


// one per predicate, e.g. for "on(.,.)"
struct AtomList {
  Predicate* pred;
  AtomL mem;

  AtomList();
  AtomList(Predicate* pred);
  void init(Predicate* pred);
  void clear();
};

// one per predicate, e.g. for "on(.,.)"
struct LiteralList {
  Predicate* pred;
  AtomList atom_list;
  LitL mem;

  LiteralList();
  LiteralList(Predicate* pred);
  void init(Predicate* pred);
  void clear();
};




struct ComparisonAtomListConstant {
  Function* f;
  MT::Array< CompAtomL > mem;  // Outer index:  args;    Inner index: bound

  ComparisonAtomListConstant();
  ComparisonAtomListConstant(Function* f);
  void init(Function* f);
  void clear();
};

struct ComparisonLiteralListConstant {
  Function* f;
  MT::Array< CompLitL > mem;  // Outer index:  args;    Inner index: bound

  ComparisonLiteralListConstant();
  ComparisonLiteralListConstant(Function* f);
  void init(Function* f);
  void clear();
};


struct ComparisonAtomListDynamic {
  Function* f;
  CompAtomL mem;  // Index 1:  args

  ComparisonAtomListDynamic();
  ComparisonAtomListDynamic(Function* f);
  void init(Function* f);
  void clear();
};

struct ComparisonLiteralListDynamic {
  Function* f;
  CompLitL mem;  // Index 1:  args

  ComparisonLiteralListDynamic();
  ComparisonLiteralListDynamic(Function* f);
  void init(Function* f);
  void clear();
};




struct FunctionAtomList {
  Function* f;
  FuncAL mem;

  FunctionAtomList();
  FunctionAtomList(Function* f);
  void init(Function* f);
  void clear();
};

struct FunctionValueList {
  Function* f;
  MT::Array< FuncVL > mem;  // Index 1:  args;    index 2: bound
  
  FunctionValueList();
  FunctionValueList(Function* f);
  void init(Function* f);
  void clear();
};





struct LogicObjectDatabase {

  LogicObjectDatabase(PredL& predicates, FuncL& functions, ComparisonPredicate* p_comp_constant, ComparisonPredicate* p_comp_dynamic);
  ~LogicObjectDatabase();

  void setPredicates(PredL& preds);
  void addPredicate(Predicate* pred);
  void setFunctions(FuncL& preds);
  void addFunction(Function* f);
  
  Predicate* p_comp_constant;
  Predicate* p_comp_dynamic;
  
  
  // Standard Atoms and Literals
  
  MT::Array< AtomList > lists_atoms; // for each predicate its own AtomList
  MT::Array< LiteralList > lists_lits_pos; // for each predicate its own positive LiteralList
  MT::Array< LiteralList > lists_lits_neg; // for each predicate its own negative LiteralList
  
  Literal* get(Predicate* p, bool positive, const uintA& args);
  Atom* get(Predicate* p, const uintA& args);
  
  
  // Comparison Atoms and Literals
  
  MT::Array< ComparisonAtomListConstant > lists_atoms_comp_constant;  //  Outer Array:  (Index 1) function-id,  (Index 2) comparison type
  MT::Array< ComparisonLiteralListConstant > lists_lits_comp_constant;   //  Outer Array:  (Index 1) function-id,  (Index 2) comparison type
  
  MT::Array< ComparisonAtomListDynamic > lists_atoms_comp_dynamic;  //  Outer Array:  (Index 1) function-id,  (Index 2) comparison type
  MT::Array< ComparisonLiteralListDynamic > lists_lits_comp_dynamic;  //  Outer Array:  (Index 1) function-id,  (Index 2) comparison type

  ComparisonAtom* get_compA(TL::Function* f, ComparisonType compType, double bound, const uintA& args);
  ComparisonLiteral* get_compL(TL::Function* f, ComparisonType compType, double bound, const uintA& args);
  ComparisonAtom* get_compA(TL::Function* f, ComparisonType compType, const uintA& args1, const uintA& args2);
  ComparisonLiteral* get_compL(TL::Function* f, ComparisonType compType, const uintA& args1, const uintA& args2);
  
  
  // FunctionAtoms and FunctionValues
  
  MT::Array< FunctionAtomList > lists_function_atoms;
  MT::Array< FunctionValueList > lists_function_values;
  
  FunctionValue* get(Function* f, const uintA& args, double value);
  FunctionAtom* get(Function* f, const uintA& args);
  
};
}

#endif  // TL__LOGIC_OBJECT_DATABASE
