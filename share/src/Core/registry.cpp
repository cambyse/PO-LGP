/*  ---------------------------------------------------------------------
    Copyright 2014 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de
    
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
    
    You should have received a COPYING file of the GNU General Public License
    along with this program. If not, see <http://www.gnu.org/licenses/>
    -----------------------------------------------------------------  */


#include "registry.h"

//===========================================================================
//
// global singleton TypeRegistrationSpace
//

Singleton<Graph> registry;

void initRegistry(int argc, char* argv[]){
  int n;
  for(n=1; n<argc; n++){
    if(argv[n][0]=='-'){
      mlr::String key(argv[n]+1);
      if(n+1<argc && argv[n+1][0]!='-'){
        mlr::String value;
        value <<'=' <<argv[n+1];
        registry().readNode(value, false, false, key);
//        new Node_typed<mlr::String>(registry(), {key}, {}, new mlr::String(argv[n+1]), true);
        n++;
      }else{
        new Node_typed<bool>(registry(), {key}, {}, true);
      }
    }else{
      MLR_MSG("non-parsed cmd line argument:" <<argv[n]);
    }
  }

  mlr::openConfigFile();
  globalThings().cfgFile >>registry();
}
