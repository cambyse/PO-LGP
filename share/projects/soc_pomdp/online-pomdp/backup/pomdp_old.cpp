

#include <boost/config.hpp>
#include <iostream>
#include <utility>
#include <algorithm>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graphviz.hpp>
#include "pomdp.h"
#include <vector>
#include <list>


using namespace boost;
using namespace std;


typedef struct edgepp{
   double capacity;
   //double weight;
 };

//////////////////////////////////////////

typedef property<vertex_distance_t, float,
           property<vertex_name_t, std::string> > VertexProperty;

typedef adjacency_list<listS, vecS, bidirectionalS, VertexProperty, edgepp> Graph;




template <class CapacityMap>
class edge_writer {
public:
  edge_writer(CapacityMap c) : cm(c) {}
  template <class Edge>
  void operator()(ostream &out, const Edge& e) const {
    out << "[label=\"dual = " << cm[e] << "\"]";
  }
private:  
  CapacityMap cm;
};

template <class CapacityMap>
inline edge_writer<CapacityMap> make_edge_writer(CapacityMap c) {
  return edge_writer<CapacityMap>(c);
}



template <class Graph>
void print(Graph& g) {
  typename Graph::vertex_iterator i, end;
  typename Graph::out_edge_iterator ei, edge_end;
  for(boost::tie(i,end) = vertices(g); i != end; ++i) {
    cout << *i << " --> ";
    for (boost::tie(ei,edge_end) = out_edges(*i, g); ei != edge_end; ++ei)
      cout << target(*ei, g) << "  ";
    cout << endl;
  }
}
std::size_t myrand(std::size_t N) {
  std::size_t ret = rand() % N;
  //  cout << "N = " << N << "  rand = " << ret << endl;
  return ret;
}



template <class Graph>
bool check_edge(Graph& g, std::size_t a, std::size_t b) {
  typedef typename Graph::vertex_descriptor Vertex;
  typename Graph::adjacency_iterator vi, viend, found;
  boost::tie(vi, viend) = adjacent_vertices(vertex(a,g), g);
  found = find(vi, viend, vertex(b, g));
  if ( found == viend )
    return false;
  return true;
}









int FSC::Horizon = 0;
long FSC::numNode = 0;

FSC::FSC(){
    Root = new NODE();

    FSC::numNode = 0;
}

FSC::~FSC(){

}


NODE::NODE()
{
    index = 0;
    NumChildren = 0;
    Childrens.clear();
    Obss.clear();
}

NODE::~NODE()
{

}


int NODE::findIndex(int obs)
{
    for(int i=0; Obss.size();i++)
    {
        if(obs == Obss[i])
            return i;
    }
    return -1; //there is no match
}

int NODE::findIndex(int obs) const
{
    for(int i=0; Obss.size();i++)
    {
        if(obs == Obss[i])
            return i;
    }
    return -1; //there is no match
}

void NODE::Add(int obs)
{
    int temp = this->getNumChildren();
    temp = temp+ 1;
    this->getNumChildren() = temp;


    Childrens.resize(temp);
    //cout<<NumChildren<<endl;
    Childrens[temp-1] = new NODE();
    Obss.push_back(obs);

}




void getTrajectory(arr& x, arr& y, arr& dual, ors::KinematicWorld& world, arr x0, const double& height, bool stickyness, uint horizon){
    /////////////

  //set initial state
  world.getBodyByName("table")->X.pos.z = height;
  //setInitialPose(x0,world);
  world.setJointState(x0);

  /////////////////////////////////////////////////////
  ////////////////////////////////////////////////////


  MotionProblem P(world, false);
  P.loadTransitionParameters(); // can change horizon here


  P.T = horizon;


  x = P.getInitialization();

  //-- setup the motion problem
  TaskCost *pos = P.addTask("position", new DefaultTaskMap(posTMT, world, "endeff", NoVector, "target", NoVector));
  P.setInterpolatingCosts(pos, MotionProblem::finalOnly,ARRAY(0.,0.,0.), 1e3);
//                          ARRAY(P.world.getShapeByName("target")->X.pos), 1e3);
//  P.setInterpolatingCosts(pos, MotionProblem::finalOnly, ARRAY(0.,0.,0.), 1e1);

  TaskCost *cons = P.addTask("planeConstraint", new PlaneConstraint(world, "endeff", ARR(0,0,-1,.7)));
    P.setInterpolatingCosts(cons, MotionProblem::constant, ARRAY(0.), 1.);

 //P.addTask("collisionConstraints", new CollisionConstraint());
  //TaskCost *coll = P.addTask("collisionConstraints", new CollisionConstraint());

  //P.setInterpolatingCosts(coll, MotionProblem::constant, ARRAY(0.), 1.);

    if(stickyness){
        stickyWeight = 10.;
        P.makeContactsAttractive = true;
    }else{
        stickyWeight = 0.;
        P.makeContactsAttractive = false;
    }

  MotionProblemFunction MF(P);
  Convert ConstrainedP(MF);

  UnconstrainedProblem UnConstrainedP(ConstrainedP);
  UnConstrainedP.mu = 10.;

  for(uint k=0;k<5;k++){
    optNewton(x, UnConstrainedP, OPT(verbose=0, stopIters=100, damping=1e-3, stopTolerance=1e-4, maxStep=.5));
    P.costReport(false);
//    displayTrajectory(x, 1, G, gl,"planned trajectory");
    UnConstrainedP.aulaUpdate(.9,x);

    P.dualMatrix = UnConstrainedP.lambda;
    UnConstrainedP.mu *= 2.;

  }
  //get the final optimal cost at each time slice
  P.costReport(false);

  if(&y){
    y.resize(x.d0, pos->map.dim_phi(world));
    for(uint t=0;t<x.d0;t++){
      world.setJointState(x[t]);
      pos->map.phi(y[t](), NoArr, world);
    }
  }
  if(&dual) dual = UnConstrainedP.lambda;
}



//set a specific pose to the robot
void setInitialPose(const arr x, ors::KinematicWorld& world)
{
    world.setJointState(x);
}

/// Online execution: Using POMDP policy (solve the POMDP online, using offline value functions from SOC)
void POMDPExecution(const arr& allx, const arr& ally, const arr& alldual, ors::KinematicWorld& world, int num){
  arr q, qdot;
  world.getJointState(q, qdot);

  ofstream data(STRING("data-"<<num<<".dat"));

  ors::Shape *endeff = world.getShapeByName("endeff");
  ors::Shape *true_target = world.getShapeByName("truetarget");
  ors::Body *est_target = world.getBodyByName("target");
  ors::Body *table = world.getBodyByName("table");
  double mean_table_height = table->X.pos.z;

  double sin_jitter = MT::getParameter<double>("sin_jitter", 0.);

  FeedbackMotionControl MC(world);
  MC.qitselfPD.active=false;

  //position PD task:  decayTime = 0.1, dampingRatio = 0.8
  PDtask *pd_y =  MC.addPDTask("position", .1, .8, new DefaultTaskMap(posTMT, world, "endeff", NoVector, "target"));
  pd_y->prec = 10.;

  //joint space PD task
  PDtask *pd_x = MC.addPDTask("pose", .1, .8, new DefaultTaskMap(qItselfTMT, world));
  pd_x->prec = .1;

  //plane constraint task
#define USE_DUAL
#ifdef USE_DUAL
  PlaneConstraint *plane_constraint = new PlaneConstraint(world, "endeff", ARR(0,0,-1,table->X.pos.z+0.02));
  ConstraintForceTask *pd_c =
      MC.addConstraintForceTask("planeConstraint", plane_constraint );
//      MC.addConstraintForceTask("touchTable",
//                                new PairCollisionConstraint(world, "endeff2", "table"));
#endif


  double tau = 0.01;
  arr x = allx[0];
  arr y = ally[0];
  arr dual = alldual[0];

  MT::Array<bool> particles;
  particles.resize(allx.d0);
  uint eligible_counts = allx.d0;
  uint index = 0, prev=index;
  for(int i=0;i<particles.d0;i++)
    particles(i) = true;

  //loop over time

  // remaining 100 steps is for reaching to the target.
  double observation = -10000.0;

  //loop over time

  // remaining 100 steps is for reaching to the target.
  for(uint t=0;t<x.d0 + 100;t++){
    MC.setState(q, qdot);


    // POMDP's online action selection
    #ifdef USE_DUAL
    observation = pd_c->infraRed_obs;
    //cout<<"observation "<<observation<<endl;
    prev = index;
   if(t<y.d0){
        for(uint sample = 0; sample < alldual.d0 ; sample++){
            //observation: equivalent to touch or not?

            if(particles(sample)){
                //un-touch: but lambda > 0 (desired touch), then eliminate this particle (sample)
                if((observation < -1e-2) && (alldual[sample](t) > 0)){
                    particles(sample) = false;
                    eligible_counts = eligible_counts - 1;
                }
                else if((observation > -1e-2) && (alldual[sample](t) == 0)){
                    particles(sample) = false;
                    eligible_counts = eligible_counts - 1;
                }else
                    index = sample;
            }
        }
   }
   #endif

   //adapt the PD task references following the plan

      if(prev!=index) cout<<" at "<<t<<"; using model # "<<index << " "<<endl;
      //cout<<" at "<<t<<"; size = "<<eligible_counts<<endl;
      if(t<y.d0){
        pd_y->y_ref = ally[index][t];
        pd_x->y_ref = allx[index][t];
  #ifdef USE_DUAL
        pd_c->desiredForce = alldual[index](t);
  #endif
    }

#ifdef USE_DUAL
    //recalibrate the target based on touch
    double d=0.;
    if(pd_c->desiredApproach.y.N){
      d = pd_c->desiredApproach.y(0); //d = distance measured by constraint task
      if(pd_c->desiredApproach.y_ref(0)==0. && d<1e-2){
        est_target->X.pos.z = endeff->X.pos.z + 0.1; //est_target position update
      }
    }
#endif
    //external sinus on the table height
    table->X.pos.z = mean_table_height+sin_jitter*::sin(double(t)/15);
#ifdef USE_DUAL
    plane_constraint->planeParams(3) = table->X.pos.z + 0.02;
#endif

    //operational space loop
    for(uint tt=0;tt<10;tt++){
      MC.updateConstraintControllers();
      arr a = MC.operationalSpaceControl();
      q += .1*tau*qdot;
      qdot += .1*tau*a;
    }

    //display and record video
//    world.watch(false, STRING(t));
    world.gl().update(STRING(t), true, false, true);
    //    flip_image(world.gl().captureImage);
    //    vid->addFrame(world.gl().captureImage);

    //write data
    MT::arrayBrackets="  ";
    data <<t <<' ' <<(t<dual.N?dual(t):0.) <<' '
        <<table->X.pos.z <<' '
       <<endeff->X.pos.z <<' '
      <<endeff->X.pos.z-table->X.pos.z <<' '
      <<est_target->X.pos.z <<' '
     <<true_target->X.pos.z <<' '
    <<endl;
  }
  data.close();

  FILE(STRING("data-"<<num<<"-err.dat")) << ARRAY(true_target->X.pos)- ARRAY(endeff->X.pos);
}
// a binary predicate implemented as a function:
// only applicable for the simple experiment in dual-execution paper.
inline bool same_contact_dual (double first, double second)
{ return ((first>0)&&(second>0)) || ((first<=0)&&(second<=0))  ; }


inline bool stickyCompute(const NODE* node)
{
    return true;
}


inline void extract(arr a1, arr& a2)
{
    a2.resize(a1.d0 - 1,a1.d1);
    for(int t=1;t<a1.d0;t++)
        a2[t-1]() = a1[t]();
}


inline void extract2(arr a1, arr& a2)
{
    a2.resize(a1.d0 - 1);
    for(int t=1;t<a1.d0;t++)
        a2(t-1) = a1(t);

}

inline void extract3(arr a1, arr& a2)
{
    a2.resize(a1.d0,a1.d1-1);
    for(int i=0;i<a1.d0;i++){
        for(int t=1;t < a1.d1;t++)
            a2[i](t-1) = a1[i](t);
    }

}


//check if all next observations are similar,
//then skip optimization for this next node.
//Just use the information from its parent's node( this means just continue following its parent's traj)
inline bool check_next_observation (arr sampleDual, double prev)
{
    double first = sampleDual[0](1);

    if(first > 0){
        for(int i = 1;i<sampleDual.d0; i++){
            if(sampleDual[i](1) <= 0 ) return false;
        }
    }
    if(first <= 0){

        for(int i = 1;i<sampleDual.d0; i++){
            if(sampleDual[i](1) > 0 ) return false;
        }
    }

    //different from its parent
    //cout<< sampleDual[0](0)<<endl;
    //cout<< prev<<endl;



    if((first > 0)&& (prev <= 0) )
        return false;
    if((first <= 0)&& (prev > 0) )
        return false;


    return true;

}


//CHECK current_obs
void OptimizeFSC(ors::KinematicWorld& world, NODE*& node, int horizon)
{
    int steps = 1;

    if(horizon >= FSC::Horizon - 1) return;

    FSC::numNode = FSC::numNode + 1;


    if(node){
        node->setIndex(FSC::numNode);
        double current_obs = node->Dual();
        //cout<< "  current_obs "<< current_obs<<endl;
        //if(horizon!=0)cout<< "  node->AllDual() "<< node->AllDual()(0)<<endl;


        //not root and new observation from the next node

        if((horizon==0)||(!check_next_observation(node->getSampleDual(),current_obs))){
            //this mean: only has change of observation (0->1; 1->0) then re-plan.

            arr heights = node->Heights();
            arr x0 = node->X();
            arr allX, allY, allDual; //trajectory for this node
            bool set=false;

            //for creating new nodes
            //std::list<double> newObs;
            arr x_nev;
            arr y_nev;
            double dual_nev;
            arr heights_nev;

            arr x_pos;
            arr y_pos;
            double dual_pos;
            arr heights_pos;
            ///////////////////////////////////////
            /// \brief num_obs
            ///

            //Compute

            arr sampleDual, sampleDual_nev, sampleDual_pos;
            sampleDual.resize(heights.d0, FSC::Horizon-horizon + 1);
            sampleDual_nev.resize(heights.d0, FSC::Horizon-horizon + 1);
            sampleDual_pos.resize(heights.d0, FSC::Horizon-horizon + 1);

            arr allDual_nev;
            arr allDual_pos;

            arr allX_nev;
            arr allX_pos;

            arr allY_nev;
            arr allY_pos;


            for(uint i=0;i < heights.d0;i++){
                arr x, y, dual;
                //compute the stickyness
                bool stickyness = stickyCompute(node);
                /////////
                /// \brief getTrajectory
                ///

                getTrajectory(x, y, dual, world, x0, heights(i), stickyness, FSC::Horizon-horizon);

                //cout<<" t= "<<i<<"   dual = "<<dual<<endl;


                //x,y,dual is a full traj of the model heights(i) START HERE

                if(!set){
                    allX = x;
                    allY = y;
                    allDual = dual;
                    set = true;
                }


                sampleDual[i]() = dual;

                //check observation of the next time step
                if(dual(steps)<=0){
                    x_nev = x[steps]();
                    y_nev = y[steps]();
                    dual_nev = dual(steps);
                    heights_nev.append(heights(i));
                    allDual_nev = dual;

                    allX_nev = x;
                    allY_nev = y;

                    sampleDual_nev[i]() = dual;
                }else{
                    x_pos = x[steps]();
                    y_pos = y[steps]();
                    dual_pos = dual(steps);
                    heights_pos.append(heights(i));
                    allDual_pos = dual;
                    allX_pos = x;
                    allY_pos = y;

                    sampleDual_pos[i]() = dual;
                }
            }

            sampleDual_nev.resize(heights_nev.d0, FSC::Horizon-horizon  + 1);
            sampleDual_pos.resize(heights_pos.d0, FSC::Horizon-horizon  + 1);

           // cout<<sampleDual_nev<<endl;




            // SET trajectory for this root node

            node->AllX() = allX;
            node->AllY() = allY;
            node->AllDual() = allDual;
            node->getSampleDual() = sampleDual; //dual traj from all samples

            /////////////////////////////////////

            //add Child: with obs = 0 (no force sensing)


            if (heights_nev.d0>0) {
                 cout<<"Negative = "<<heights_nev.d0<<endl;

                node->Add(0);
                NODE*temp = node->Child(0);
                if(current_obs <= 0){
                    //same observation with parent, then use the next state in its parent's traj
                    temp->X() = allX[steps]();
                    temp->Y() = allY[steps]();
                    temp->Dual() = allDual(steps);
                    temp->Heights() = heights_nev;

                }else{
                    temp->X() = x_nev;
                    temp->Y() = y_nev;
                    temp->Dual() = dual_nev;
                    temp->Heights() = heights_nev;
                    //recursive update this child node
                }
                extract(allX_nev,temp->AllX());
                extract(allY_nev,temp->AllY());

                extract2(allDual_nev,temp->AllDual());

                temp->getSampleDual() = sampleDual_nev;
                OptimizeFSC(world, temp, horizon + steps);
            }

            //add Child: with obs = 0 (no force sensing)
            if (heights_pos.d0>0) {
                cout<<"Positive = "<<heights_pos.d0<<endl;


                node->Add(1);
                NODE*temp = node->Child(1);
                if(current_obs > 0){
                    //similar observation, then use the next state in its parent's traj
                    temp->X() = allX[steps]();
                    temp->Y() = allY[steps]();
                    temp->Dual() = allDual(steps);
                    temp->Heights() = heights_pos;


                }else{
                    temp->X() = x_pos;
                    temp->Y() = y_pos;
                    temp->Dual() = dual_pos;
                    temp->Heights() = heights_pos;
                }
                extract(allX_pos,temp->AllX());
                extract(allY_pos,temp->AllY());

                extract2(allDual_pos,temp->AllDual());
                temp->getSampleDual() = sampleDual_pos;
                //recursive update this child node
                OptimizeFSC(world, temp, horizon + steps);



            }
        }else{
            arr allX = node->AllX();
            arr allY = node->AllY();
            arr dual = node->AllDual();
            arr allHeights = node->Heights();

            arr samples = node->getSampleDual();

            NODE*temp;
            //cout<< dual<<endl;

            //cout<< allHeights<<endl;

            if(dual(1) > 0){
                node->Add(1);
                temp = node->Child(1);
            }else{
                node->Add(0);
                temp = node->Child(0);
            }

            temp->X() = allX[steps]();
            temp->Y() = allY[steps]();
            temp->Dual() = dual(steps);
            temp->Heights() = allHeights;

            extract(allX,temp->AllX());
            extract(allY,temp->AllY());
            extract2(dual,temp->AllDual());

            extract3(samples,temp->getSampleDual());

            OptimizeFSC(world, temp, horizon + steps);

        }

    }



}
inline void add_node_to_graph(NODE*& root, Graph& graph)
{
    for(int i=0;i<root->Childrens.size();i++){
        if(root->Childrens[i]){
            edgepp prop;
            //prop.weight = 0.0;root->Dual();
            prop.capacity = root->Childrens[i]->Dual();
            long node1 = root->getIndex();
            long node2 = root->Childrens[i]->getIndex();
            add_edge(node1, node2, prop, graph);
        }
        add_node_to_graph(root->Childrens[i], graph);
    }
}


void write_to_graphviz(FSC fsc)
{
    NODE*& root=fsc.getRoot();
    long numN = FSC::numNode;
    std::size_t N = numN;
    Graph graph(N);

    add_node_to_graph(root, graph);



    // write the dot file

   std::ofstream dotfile ("policy.dot");
   write_graphviz (dotfile, graph,
                   boost::default_writer(),
                   make_edge_writer( boost::get(&edgepp::capacity,graph)));
}
