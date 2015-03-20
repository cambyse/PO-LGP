#include <Core/array.h>
#include <Core/graph.h>

//===========================================================================

template<class D>
struct Node{
  Node *parent;
  MT::Array<Node*> children;
  D decision;           ///< what decision (relative to the parent) does this node represent

  double Qup,Qme,Qlo;   ///< upper, mean, and lower Q estimates
  double r, R;          ///< last and total immediate rewards
  uint N;               ///< # of visits (to normalize immediate rewards)
  double Q;             ///< total (on-policy) returns

  uint t;               ///< depth of this node
  void *data;           ///< dummy helper (to convert to other data structures)

  Node(Node *parent, D decision):parent(parent), decision(decision), Qup(0.), Qme(0.), Qlo(0.), r(0.), R(0.), N(0), Q(0.), t(0), data(NULL){
    if(parent){
      t=parent->t+1;
      parent->children.append(this);
    }
  }
};

//===========================================================================

template<class W, class D>
struct MCTS{
  W& world;
  Node<D> root;

  MCTS(W& world):world(world), root(NULL, D()){}

  void addRollout();                 ///< adds one more rollout to the tree
  Node<D>* treePolicy(Node<D> *n);   ///< policy to choose the child from which to do a rollout or to expand
  double Qvalue(Node<D>* n, int optimistic); ///< current value estimates at a node
  arr Qfunction(Node<D> *n=NULL, int optimistic=0); ///< the Q-function (value estimates of all children) at a node

  //only to display
  void writeToGraph(Graph& G, Node<D> *n=NULL);
  Graph getGraph(){ Graph G; writeToGraph(G); return G; }
};

//===========================================================================

template<class W, class D> void MCTS<W, D>::addRollout(){
  Node<D> *n = &root;
  world.resetToStart();

  //-- tree policy
  while(!world.terminal()){
    if(!n->children.N) break;
    n = treePolicy(n);
    n->r = world.advance(n->decision);
  }

  //-- expand: compute new decisions and add corresponding nodes
  if(!world.terminal() && n->N){ //we expand n only if it is not a freshmen
    for(D d:world.getDecisions()) new Node<D>(n, d); //this adds a bunch of freshmen for all possible decisions
    n = n->children(0);
    n->r = world.advance(n->decision);
  }

  //-- rollout
  double Return=0.;
  while(!world.terminal())  Return += world.advanceRandomly();

  //-- backup
  for(;;){
    if(!n) break;
    n->N++;
    n->R += n->r;   //total immediate reward
    Return += n->r; //add up total return from n to terminal
    n->Q += Return;
    if(n->children.N && n->N>n->children.N){ //propagate bounds
      n->Qup = max( Qfunction(n, +1) );
      n->Qme = max( Qfunction(n,  0) );
      n->Qlo = max( Qfunction(n, -1) );
    }
    n = n->parent;
  }  
}

template<class W, class D> Node<D>* MCTS<W, D>::treePolicy(Node<D>* n){
  CHECK(n->children.N, "you should have children!");
  CHECK(n->N, "you should not be a freshman!");
  if(n->N>n->children.N){ //we've visited each child at least once
    arr Q = Qfunction(n, +1);      //optimistic Qfunction
    rndUniform(Q, 0., 1e-3, true); //add noise
    return n->children( argmax( Q ) );
  }
  return n->children( n->N-1 ); //else: visit children by their order
}

template<class W, class D> arr MCTS<W, D>::Qfunction(Node<D>* n, int optimistic){
  if(!n) n=&root;
  if(!n->children.N) return arr();
  arr Q(n->children.N);
  uint i=0;
  for(Node<D> *ch:n->children){ Q(i) = Qvalue(ch, optimistic); i++; }
  return Q;
}

template<class W, class D> double MCTS<W, D>::Qvalue(Node<D>* n, int optimistic){
  if(n->children.N && n->N>n->children.N){ //the child is mature and has children itself
    if(optimistic==+1) return n->Qup;
    if(optimistic== 0) return n->Qme;
    if(optimistic==-1) return n->Qlo;
  }else{
    //the child is premature -> use its on-policy return estimates (and UCB)
    double beta = 2.*sqrt(2.*::log(n->parent?n->parent->N:n->N));
    if(optimistic==+1) return n->Q/n->N + beta/sqrt(n->N);
    if(optimistic== 0) return n->Q/n->N;
    if(optimistic==-1) return n->Q/n->N - beta/sqrt(n->N);
  }
  HALT("");
  return 0.;
}

template<class W, class D> void MCTS<W, D>::writeToGraph(Graph& G, Node<D> *n){
  ItemL par;
  if(!n) n=&root; else par.append((Item*)(n->parent->data));
  double q=-10.;  if(n->N) q=n->Q/n->N;
  n->data = new Item_typed<double>(G, {STRING("t"<<n->t <<'N' <<n->N <<'[' <<n->Qlo <<',' <<n->Qme <<',' <<n->Qup <<']')}, par, new double(q), true);
  for(Node<D> *c:n->children) writeToGraph(G, c);
}
