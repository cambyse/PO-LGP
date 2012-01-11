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

#ifndef TL__RM_SAMPLING
#define TL__RM_SAMPLING

#include <relational/robotManipulationSampling.h>
#include <relational/logicObjectManager.h>

namespace TL {



namespace robotManipulationSampling {

  
  // SAMPLING PARAMETERS
  extern bool SAMPLING__WATCH_AFTER_ACTION;
  extern uint SAMPLING__WAIT_SEC_AFTER_ACTION;
  extern double SAMPLING__PROB_SENSIBLE_ACTION;
  extern double SAMPLING__PROB_GRAB_CLEARGUY;
  extern double SAMPLING__PROB_PUTON_CLEARGUY;
  
	// Action generation
  TL::Atom* generateAction(const SymbolicState& s, uint id_table);
  TL::Atom* generateAction_onlyPossible(const SymbolicState& s, uint id_table);
  TL::Atom* generateAction_trulyRandom(const SymbolicState& s, uint id_table);
  TL::Atom* generateAction_wellBiased(const SymbolicState& s, uint id_table);
  TL::Atom* generateAction_wellBiased_2Dactions(const SymbolicState& s, uint id_table);
  TL::Trial* generateSimulationSequence(RobotManipulationSimulator* sim, uint maxSeqLength, uint id_table);	
  // generate & write
  void generateSimulationSequence_realistic(std::ostream& os, RobotManipulationSimulator* sim, uint maxSeqLength, uint id_table);

}

}

#endif // TL__RM_SAMPLING