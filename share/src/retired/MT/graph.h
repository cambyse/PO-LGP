/*  ---------------------------------------------------------------------
    Copyright 2012 Marc Toussaint
    email: mtoussai@cs.tu-berlin.de
    
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



//graph representations
uintA E; //Nx2 array of edge tuples
MT::Array<uintA> del; //array of in-neighbors lists

void EdgesToNeighbors(MT::Array<uintA>& del, uint N, const uintA& E){
  uint i;
  del.resize(N);
  for(i=0; i<E.d0; i++) del(E(i, 1)).append(i);
  //cout <<"E=" <<E <<"del=" <<del <<endl;
}

void writeToDotFile(const char* filename, uint N, const uintA& E){
  ofstream fil(filename);
  fil <<"digraph " <<"N_" <<N <<"_" <<E.d0 <<" {\n"
  <<"\trankdir=TB;\n";
  
  for(uint i=0; i<E.d0; ++i){
    fil <<E(i, 0) <<'-' <<E(i, 1) <<endl;
  }
  
  fil <<"}\n";
  fil.close();
}
