
struct ManipulationTree_Node{
  ManipulationTree_Node *parent;
  MT::Array<ManipulationTree_Node*> children;
  uint s;               ///< depth/step of this node
  double t;               ///< real time

  action;           ///< what decision (relative to the parent) does this node represent

  ors::KinematicSwitch sw;
  ors::KinematicWorld kinematics;
  ors::KinematicWorld effectiveKinematics;


  MCTS_Node(MCTS_Node *parent, MCTS_Environment::Handle decision):parent(parent), decision(decision), Qup(0.), Qme(0.), Qlo(0.), r(0.), R(0.), N(0), Q(0.), t(0), data(NULL){
    if(parent){
      t=parent->t+1;
      parent->children.append(this);
    }
  }
};
