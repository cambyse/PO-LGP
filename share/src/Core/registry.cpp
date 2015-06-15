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

Singleton<Graph> SingleRegistry;

Graph& registry(){ return SingleRegistry(); }

namespace MT {
extern std::ifstream cfgFile;
extern bool cfgFileOpen;
extern Mutex cfgFileMutex;
}

extern Node *readNode(Graph& containingGraph, std::istream& is, bool verbose, bool parseInfo, MT::String prefixedKey);

void initRegistry(int argc, char* argv[]){

  int n;
  for(n=1; n<argc; n++){
    if(argv[n][0]=='-'){
      MT::String key(argv[n]+1);
      if(n+1<argc && argv[n+1][0]!='-'){
        MT::String value;
        value <<'=' <<argv[n+1];
        readNode(registry(), value, false, false, key);
//        new Node_typed<MT::String>(registry(), {key}, {}, new MT::String(argv[n+1]), true);
        n++;
      }else{
        new Node_typed<bool>(registry(), {key}, {}, new bool(true), true);
      }
    }else{
      MT_MSG("non-parsed cmd line argument:" <<argv[n]);
    }
  }

  MT::openConfigFile();
  MT::cfgFile >>registry();

}
