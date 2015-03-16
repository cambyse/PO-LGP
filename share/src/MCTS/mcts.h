#include <Core/array.h>
#include <Core/graph.h>

template<class D>
struct Node{
  Node *parent;
  MT::Array<Node*> children;
  D decision; ///< what decision (relative to the parent) does this node represent
  double Q;   ///< accumulation of all returns
  uint N;     ///< #of rollouts from this node
  uint t;     ///< depth of this node
  void *data; ///< dummy helper (to convert to other data structures)

  Node(Node *parent, D decision):parent(parent), decision(decision), Q(0.), N(0), t(0), data(NULL){
    if(parent) t=parent->t+1;
  }
};

template<class W, class D>
struct MCTS{
  W& world;
  Node<D> root;

  MCTS(W& world):world(world), root(NULL, D()){}

  void addRollout();                 ///< adds one more rollout to the tree
  Node<D>* treePolicy(Node<D> *n);   ///< policy to choose the child from which to do a rollout or to expand
  arr Qfunction(Node<D> *n=NULL);

  void writeToGraph(Graph& G, Node<D> *n=NULL);
  Graph getGraph(){ Graph G; writeToGraph(G); return G; }
};

template<class W, class D> void MCTS<W, D>::addRollout(){
  Node<D> *n = &root;
  double R=0;
  world.resetToStart();

  //-- tree policy
  while(!world.terminal()){
    if(!n->children.N) break;
    n = treePolicy(n);
    R += world.advance(n->decision);
  }

  //-- expand: compute new decisions and add corresponding nodes
  if(!world.terminal() && n->N){ //we expand n only if it is not a freshmen
    MT::Array<D> decisions = world.getDecisions();
    for(D d:decisions){
      Node<D> *newNode = new Node<D>(n, d); //this adds a bunch of freshmen for all possible decisions
      n->children.append(newNode);
    }
    n = treePolicy(n); //this plays one of the freshmen
    R += world.advance(n->decision);
  }

  //-- rollout
  while(!world.terminal()){
    R += world.advanceRandomly();
  }

  //-- backup
  for(;;){
    if(!n) break;
    n->Q += R;
    n->N++;
    n = n->parent;
  }  
}

template<class W, class D> Node<D>* MCTS<W, D>::treePolicy(Node<D>* n){
  arr preference = zeros(n->children.N);
  uint i=0, d;
  CHECK(n->N, "you should not be a freshman, if you have children!");
  //NOTE: if every child was visited once we have n->N==#children+1 (because the parent was visited once as a freshman)
  if(n->N>n->children.N){ //we've visited each child at least once
    double beta = 2.*sqrt(2.*::log(n->N));
    for(Node<D> *c:n->children){
      preference(i) = c->Q/c->N + beta/sqrt(c->N);
      i++;
    }
    d = argmax(preference);
  }else{
    d = n->N-1;
  }
  return n->children(d);
}

template<class W, class D> arr MCTS<W, D>::Qfunction(Node<D>* n){
  if(!n) n=&root;
  CHECK(n->N>n->children.N, "you should not be a freshman, if you have children!");
  arr Q = zeros(n->children.N);
  uint i=0;
  for(Node<D> *c:n->children){
    Q(i) = c->Q/c->N;
    i++;
  }
  return Q;
}

template<class W, class D> void MCTS<W, D>::writeToGraph(Graph& G, Node<D> *n){
  ItemL par;
  if(!n) n=&root; else par.append((Item*)(n->parent->data));
  double q=-10.;  if(n->N) q=n->Q/n->N;
  n->data = new Item_typed<double>(G, {STRING("t"<<n->t <<"d" <<n->decision <<'N' <<n->N <<'Q' <<n->Q)}, par, new double(q), true);
  for(Node<D> *c:n->children) writeToGraph(G, c);
}
