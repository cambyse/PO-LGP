/*  
    Copyright 2008-2012   Tobias Lang
    
    E-mail:    tobias.lang@fu-berlin.de
    
    This file is part of libARMANI.

    libARMANI is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    libARMANI is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with libARMANI.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef RELATIONAL_robotManipulationSimulator_h
#define RELATIONAL_robotManipulationSimulator_h

#include "relational/reason.h"
#include "relational/plan.h"
#include "relational/robotManipulationSimulator.h"



namespace relational {
  
namespace RobotManipulationInterface {
  
  /************************************************
  * 
  *     High-level interface between
  *     symbolic representations and
  *     geometric simulator
  * 
  ************************************************/
  
  // Observation
  relational::SymbolicState* calculateSymbolicState(RobotManipulationSimulator* sim);
  
  // Action
  void executeAction(relational::Literal* action, RobotManipulationSimulator* sim, uint secs_wait_after_action, const char* message = "");

  // Other convenience methods
  void getTypes(relational::ArgumentTypeL& objects_types, const uintA& objects, const relational::ArgumentTypeL& types, RobotManipulationSimulator* sim);
  
  
  /************************************************
  * 
  *     Sampling actions
  * 
  ************************************************/
  
  relational::Literal* generateAction(const relational::SymbolicState& s, uint id_table);
  relational::Literal* generateAction_trulyRandom(const relational::SymbolicState& s, uint id_table);
  relational::Literal* generateAction_wellBiased(const relational::SymbolicState& s, uint id_table);
  relational::Literal* generateAction_wellBiased_2Dactions(const relational::SymbolicState& s, uint id_table);
  relational::StateTransitionL generateSimulationSequence(RobotManipulationSimulator* sim, uint maxSeqLength, uint id_table);  
  void generateSimulationSequence_realistic(std::ostream& os, RobotManipulationSimulator* sim, uint maxSeqLength, uint id_table);

  
  // sampling parameters
  extern bool SAMPLING__WATCH_AFTER_ACTION;
  extern uint SAMPLING__WAIT_SEC_AFTER_ACTION;
  extern double SAMPLING__PROB_SENSIBLE_ACTION;
  extern double SAMPLING__PROB_GRAB_CLEARGUY;
  extern double SAMPLING__PROB_PUTON_CLEARGUY;
}

}

#endif // RELATIONAL_robotManipulationSimulator_h

