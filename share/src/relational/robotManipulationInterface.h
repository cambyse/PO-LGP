/*  
    Copyright 2008-2012   Tobias Lang
    
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

#ifndef PRADA__ROBOT_MANIPULATION_DOMAIN
#define PRADA__ROBOT_MANIPULATION_DOMAIN

#include "relational/reason.h"
#include "relational/plan.h"
#include "relational/robotManipulationSimulator.h"



namespace RMSim {
  
namespace RobotManipulationInterface {
  
  /* ---------------------
    TOP-LEVEL LOGIC-SIMULATOR-INTERFACE
  --------------------- */
  // Observation
  PRADA::SymbolicState* calculateSymbolicState(RobotManipulationSimulator* sim);
  
  // Action
  void performAction(PRADA::Literal* action, RobotManipulationSimulator* sim, uint secs_wait_after_action, const char* message = "");

  
  
  // SAMPLING PARAMETERS
  extern bool SAMPLING__WATCH_AFTER_ACTION;
  extern uint SAMPLING__WAIT_SEC_AFTER_ACTION;
  extern double SAMPLING__PROB_SENSIBLE_ACTION;
  extern double SAMPLING__PROB_GRAB_CLEARGUY;
  extern double SAMPLING__PROB_PUTON_CLEARGUY;
  
  // Action generation
  PRADA::Literal* generateAction(const PRADA::SymbolicState& s, uint id_table);
  PRADA::Literal* generateAction_onlyPossible(const PRADA::SymbolicState& s, uint id_table);
  PRADA::Literal* generateAction_trulyRandom(const PRADA::SymbolicState& s, uint id_table);
  PRADA::Literal* generateAction_wellBiased(const PRADA::SymbolicState& s, uint id_table);
  PRADA::Literal* generateAction_wellBiased_2Dactions(const PRADA::SymbolicState& s, uint id_table);
  PRADA::StateTransitionL& generateSimulationSequence(RobotManipulationSimulator* sim, uint maxSeqLength, uint id_table);  
  // generate & write
  void generateSimulationSequence_realistic(std::ostream& os, RobotManipulationSimulator* sim, uint maxSeqLength, uint id_table);

}

}

#endif // PRADA__ROBOT_MANIPULATION_DOMAIN

