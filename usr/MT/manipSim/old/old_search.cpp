mlr::KinematicWorld* world=NULL;

void OrsGraph2RelationalGraph(Graph& G, mlr::KinematicWorld& W){
  G.clear();

  //do this first to ensure they have the same indexing
  for(mlr::Body *b:world->bodies){
    G.newNode<mlr::Body>({"body", b->name}, b, false);
  }

  for(mlr::Body *b:world->bodies){
    G.newNode<mlr::Transformation>({"pose"}, {G(b->index)}, new mlr::Transformation(b->X), true);
//    if(b->ats["ctrlable"]) G.newNode<bool>({"controllable"}, {G(b->index)}, NULL);
    if(b->ats["canGrasp"]) G.newNode<bool>({"canGrasp"}, {G(b->index)}, NULL, false);
    if(b->ats["fixed"])    G.newNode<bool>({"fixed"}, {G(b->index)}, NULL, false);
  }

  for(mlr::Joint *j:world->joints){
    if(j->type==mlr::JT_rigid)
      G.newNode<bool>({"rigid"}, {G(j->from->index), G(j->to->index)}, NULL, false);
    if(j->type==mlr::JT_transXYPhi)
      G.newNode<bool>({"support"}, {G(j->from->index), G(j->to->index)}, NULL, false);
  }

}

uint Domain::numObjects(){
  return world->bodies.N;
}

void Domain::getInitialState(State &s){
  OrsGraph2RelationalGraph(s.G, *world);
}

//===========================================================================

void sample(){
  mlr::KinematicWorld W("model.kvg");
  world = &W;

  //-- fwd expansion
  SearchNodeL T;
  SearchNode *root=new SearchNode(T);
  SearchNode *goal=NULL;

  cout <<"initial state=\n" <<*root <<endl;

  for(uint k=0;k<10;k++){
    SearchNode *n = T(k);
    Action a = n->getRandomFeasibleAction();
    cout <<"random action=" <<a <<endl;
    SearchNode *m = new SearchNode(*n, a);
    m->state.expandReachable();
    cout <<"new state=\n" <<*m <<endl;

    mlr::wait();

//      if(checkGoalIsFeasible(s)){
//        goal = T.last();
//        break;
//      }
//    }
  }

  return;
  //backtracking
  SearchNodeL plan = backtrack<SearchNode>(T,goal);
  for(SearchNode *n:plan){
    cout <<"pre-action=" <<n->getPreAction() <<endl;
    cout <<"state=" <<n->getState() <<endl;
  }

}

//===========================================================================
