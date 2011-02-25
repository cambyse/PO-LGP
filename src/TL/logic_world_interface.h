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

#ifndef TL__LOGIC_WORLD_INTERFACE
#define TL__LOGIC_WORLD_INTERFACE

#include <TL/ors_actionInterface.h>
#include <TL/logicEngine.h>


namespace TL {



namespace logic_world_interface {
  
  
  
namespace bw {
    
  // Observation
	State* observeLogic(ActionInterface* ai, LogicEngine* le);
  void observeAngles(arr& angles, ActionInterface* ai, LogicEngine* le);
  void observePositions(arr& angles, ActionInterface* ai, LogicEngine* le);
  
  // Action
  void performAction(PredicateInstance* action, ActionInterface* ai, uint secs_wait_after_action, const char* message = "");
  
  // Helpers
  void writeFeatures(std::ostream& os, ActionInterface* ai);

}

}

}

#endif // TL__LOGIC_WORLD_INTERFACE
