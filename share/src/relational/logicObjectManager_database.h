/*  
    Copyright 2012   Tobias Lang
    
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


#ifndef TL__LOGIC_OBJECT_MANAGER_DATABASE
#define TL__LOGIC_OBJECT_MANAGER_DATABASE

#include <relational/logicDefinitions.h>



namespace TL {

extern uint LOGIC__MAX_LIMIT_OBJECT_ID;


namespace logicObjectManager_database {

  void shutdown();
  
  void setComparisonPredicates(ComparisonPredicate* p_comp_constant, ComparisonPredicate* p_comp_dynamic);

  void addPredicate(Predicate* pred);
  void addFunction(Function* f);
    
  // Standard Atoms and Literals  
  Literal* get(Predicate* p, bool positive, const uintA& args);
  Atom* get(Predicate* p, const uintA& args);
  
  // Comparison Atoms and Literal
  ComparisonAtom* get_compA(TL::Function* f, ComparisonType compType, double bound, const uintA& args);
  ComparisonLiteral* get_compL(TL::Function* f, ComparisonType compType, double bound, const uintA& args);
  ComparisonAtom* get_compA(TL::Function* f, ComparisonType compType, const uintA& args1, const uintA& args2);
  ComparisonLiteral* get_compL(TL::Function* f, ComparisonType compType, const uintA& args1, const uintA& args2);
  
  // FunctionAtoms and FunctionValues  
  FunctionValue* get(Function* f, const uintA& args, double value);
  FunctionAtom* get(Function* f, const uintA& args);
  
};
}

#endif  // TL__LOGIC_OBJECT_MANAGER_DATABASE
