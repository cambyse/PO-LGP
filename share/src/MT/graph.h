

//graph representations
uintA E; //Nx2 array of edge tuples
MT::Array<uintA> del; //array of in-neighbors lists

void EdgesToNeighbors(MT::Array<uintA>& del, uint N, const uintA& E){
  uint i;
  del.resize(N);
  for(i=0; i<E.d0; i++) del(E(i, 1)).append(i);
  //cout  <<"E="  <<E  <<"del="  <<del  <<endl;
}

void writeToDotFile(const char* filename, uint N, const uintA& E){
  ofstream fil(filename);
  fil  <<"digraph "  <<"N_"  <<N  <<"_"  <<E.d0  <<" {\n"
 <<"\trankdir=TB;\n";
  
  for(uint i=0; i<E.d0; ++i){
    fil  <<E(i, 0)  <<'-'  <<E(i, 1)  <<endl;
  }
  
  fil  <<"}\n";
  fil.close();
}
