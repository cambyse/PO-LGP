

#include <boost/config.hpp>
#include <iostream>
#include <utility>
#include <algorithm>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graphviz.hpp>
#include "pomdp.h"
#include "execution.h"
#include <vector>
#include <list>


using namespace boost;
using namespace std;


typedef struct edgepp{
   //double capacity;
   arr x;
   arr y;
   double dual;
   double height;
 };

//////////////////////////////////////////

typedef property<vertex_distance_t, float,
           property<vertex_name_t, std::string> > VertexProperty;

typedef adjacency_list<listS, vecS, bidirectionalS, VertexProperty, edgepp> Graph;




template <class X, class Y, class Dual, class Height>
class edge_writer {
public:
    edge_writer(X x,Y y,Dual dual, Height height) : cx(x), cy(y), cdual(dual), cheight(height) {}
  template <class Edge>
  void operator()(ostream &out, const Edge& e) const {
      //out << "[label=\"dual = " << cdual[e] <<"  x= "<<cx[e] <<"  y= "<<cy[e] << "\"]";
      out << "[label=\"dual = " << cdual[e] <<"  y= "<<cy[e] << "\"]";
  }
private:
  X cx;
  Y cy;
  Dual cdual;
  Height cheight;
};

template  <class X, class Y, class Dual, class Height>
inline edge_writer<X,Y,Dual,Height> make_edge_writer(X x,Y y,Dual dual, Height height) {
  return edge_writer<X,Y,Dual,Height>(x,y,dual,height);
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
long FSC::numNode = -1;

FSC::FSC(){
    Root = new NODE();

    FSC::numNode = -1;
}

FSC::~FSC(){

}


NODE::NODE()
{
    index = 0;
    NumChildren = 0;
    Childrens.clear();
    Obss.clear();
    certainty = false;
    sticky = true; //sticky = false when  certainty(t) && certainty(t-1)
    height = 0.0;
}

NODE::~NODE()
{

}

inline bool equal(const Observation obs1,const Observation obs2)
{
    if((obs1.height == obs2.height) && (obs1.x == obs2.x) && (obs1.y == obs2.y) && (obs1.dual == obs2.dual))
        return true;
    else
        return false;
}

int NODE::findIndex(Observation obs)
{
    for(int i=0; Obss.size();i++)
    {
        if(equal(obs,Obss[i]))
            return i;
    }
    return -1; //there is no match
}

int NODE::findIndex(Observation obs) const
{
    for(int i=0; Obss.size();i++)
    {
        if(equal(obs,Obss[i]))
            return i;
    }
    return -1; //there is no match
}

void NODE::Add(Observation obs)
{
    int temp = this->getNumChildren();
    temp = temp + 1;
    this->getNumChildren() = temp;


    Childrens.resize(temp);
    //cout<<NumChildren<<endl;
    Childrens[temp-1] = new NODE();
    Obss.push_back(obs);

}




//set a specific pose to the robot
void setInitialPose(const arr x, ors::KinematicWorld& world)
{
    //Using the controller.
}
// a binary predicate implemented as a function:
// only applicable for the simple experiment in dual-execution paper.
inline bool same_contact_dual (double first, double second)
{ return ((first>0)&&(second>0)) || ((first<=0)&&(second<=0))  ; }


inline bool stickyCompute(NODE*& node)
{

    //simply compute, if there is only one particle, then it is certain
    if((node->Heights().d0 == 1) && (node->AllDual()(0)>0)){
        node->certainty = true;
        return true;
    }
    else
        return false;
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
        return false;
        for(int i = 1;i<sampleDual.d0; i++){
            if(sampleDual[i](1) <= 0 ) return false;
        }
    }
    if(first <= 0){

        for(int i = 1;i<sampleDual.d0; i++){
            if(sampleDual[i](1) > 0 ) return false;
        }
    }


    if((first > 0)&& (prev <= 0) )
        return false;
    if((first <= 0)&& (prev > 0) )
        return false;


    return true;
}

void OptimizeFSC_Test(ors::KinematicWorld& world, NODE *&node, int horizon)
{
    arr x, y, dual;

    arr heights = node->Heights();
    arr x0 = node->X();
    for(uint i=0;i < heights.d0;i++){
        getTrajectory(x, y, dual, world, x0, heights(i), true, FSC::Horizon-horizon);
        cout<<dual<<endl;
    }

}

inline bool trajectory_pass_pos (double pos, arr y, double z_target)
{
    //arr curr_target = ARR(0.45, -0.45, z_target + 0.12);

    arr end_eff = y[1];// + curr_target; //y[1] means next step; curr_target of the current followed model (in the node)

    //0.5 is the width: pos + WIDTH means the left edge

    //cout<<"pso "<<pos <<"  "<<end_eff <<endl;

    if(pos + 0.02 > end_eff(2)) //comparing table's height and z_eff
        return true; // the trajectory pass this heigth at the next step [STRICTLY]
    else return false; //not yet pass

}

inline bool trajectory_not_pass_models (arr heights, arr y, double z_target)
{
    //cout<<x_target <<z_target <<endl;
    for(int i = 0;i<heights.d0; i++){
        if(trajectory_pass_pos(heights(i),y,z_target))
            return false;
    }

    return true;
}


NODE* addNode_MultiParticle(NODE*& node,Observation obs, int steps)
{
    node->Add(obs);
    NODE*temp = node->Child(obs);

    temp->X() = node->AllX()[steps]();
    temp->Y() = node->AllY()[steps]();
    temp->Dual() = node->AllDual()(steps);
    temp->Heights() = node->Heights();
    temp->Height() = node->Height();

    temp->certainty = node->certainty;

    //cout<< " multi "<<temp->Model() <<endl;


    extract(node->AllX(),temp->AllX());
    extract(node->AllY(),temp->AllY());
    extract2(node->AllDual(),temp->AllDual());

    return temp;
}


int choose_Next_Obs(arr Heights)
{
    int best_index=0;
    double best_value = 100000; //this 1-d problem, we choose the lowest table (which give best entropy, or one-step look-ahead)
    for(int i=0;i<Heights.d0;i++){
        if(Heights(i)<best_value){
            best_value = Heights(i);
            best_index = i;
        }
    }
    return best_index;
}

//OPTIMIZING an FSC
void OptimizeFSC(ors::KinematicWorld& world, NODE*& node, int horizon)
{
    int steps = 1;

    if(!node) return;

    FSC::numNode = FSC::numNode + 1; //static variable to count the number of generated nodes
    node->setIndex(FSC::numNode);   //indexing the node

    if(horizon >= FSC::Horizon - 1) return;

    arr samples = node->Heights();

    if(((node->Heights().d0 > 1) && ((horizon==0)||(!trajectory_not_pass_models(samples,node->AllY(),node->Height())))) || ((node->Heights().d0 == 1)&&(!node->certainty))){
        //CHECK Again: if the best model is still in the node, then don't need replanning
        arr x0 = node->X();
        int best_obs = choose_Next_Obs(samples);
        arr x, y, dual;

        if(node->Heights().d0 == 1){
            node->certainty = true;
        }



        if(node->certainty)
            getTrajectory(x, y, dual, world, x0, samples(best_obs), false, FSC::Horizon-horizon);
        else
            getTrajectory(x, y, dual, world, x0, samples(best_obs), true, FSC::Horizon-horizon);


/*/
        cout<<dual<<endl;
        for(int t=0;t<x.d0;t++){
            world.setJointState(x[t]);
            world.gl().update(STRING(t), true, false, true);
        }
/*/

        //re-update this node
        node->X() = x[0];
        node->Y() = y[0];
        node->Dual() = dual(0);
        node->AllX() = x;
        node->AllY() = y;
        node->AllDual() = dual;
        node->Height() = samples(best_obs);





        arr samples_nev;

         for(uint i=0;i < samples.d0;i++){
            if(!trajectory_pass_pos(samples(i),y,samples(best_obs))){
                samples_nev.append(samples(i));
            }else{ //create new branch
                Observation obs;
                obs.x = x[steps]();
                obs.y = y[steps]();
                obs.dual = dual(steps);
                obs.height = samples(i);

                node->Add(obs);
                NODE*temp = node->Child(obs);

                extract(x,temp->AllX());
                extract(y,temp->AllY());
                extract2(dual,temp->AllDual());
                temp->Heights().resize(1);
                temp->Heights()(0) = samples(i);
                temp->Height() = samples(i);
                temp->X() = x[steps]();
                temp->Y() = y[steps]();
                temp->Dual() = dual(steps);

                temp->certainty = node->certainty;
            }
        }


        if (samples_nev.d0 > 0) {
             int best_o = choose_Next_Obs(samples_nev);

             Observation obs;
             obs.x = x[steps]();
             obs.y = y[steps]();
             obs.dual = dual(steps);
             obs.height = samples_nev(best_o);
             //obs.pos    = samples_nev[best_o](1);


            node->Add(obs);
            NODE*temp = node->Child(obs);


            extract(x,temp->AllX());
            extract(y,temp->AllY());
            extract2(dual,temp->AllDual());
            temp->Heights() = samples_nev;
            temp->Height() = samples_nev(best_o);
            temp->X() = x[steps]();
            temp->Y() = y[steps]();
            temp->Dual() = dual(steps);

            temp->certainty = node->certainty;

            //cout<<"  temp->Model() "<<temp->Model() <<endl;

        }

        for(int no=0; no < node->Childrens.size();no++){
            NODE* child = node->Child(node->Obss[no]);
            OptimizeFSC(world, child, horizon + steps);
        }

    }else{ //if there is no change in observation (then continue following the parent's traj

        Observation obs;
        obs.x = node->AllX()[steps]();
        obs.y = node->AllY()[steps]();
        obs.dual = node->AllDual()(steps);
        obs.height = node->Height();
        //obs.pos = node->Model()(1);

        NODE* temp = addNode_MultiParticle(node, obs, steps);
        OptimizeFSC(world, temp, horizon + steps);
    }
}



inline void add_node_to_graph(NODE*& root, Graph& graph)
{
    for(int i=0;i<root->Obss.size();i++){
        if(root->Childrens[i]){
            edgepp prop;
            prop.x = root->Childrens[i]->X();
            prop.y = root->Childrens[i]->Y();
            prop.dual = root->Childrens[i]->Dual();
            prop.height = root->Childrens[i]->Height();
            long node1 = root->getIndex();
            long node2 = root->Childrens[i]->getIndex();
            add_edge(node1, node2, prop, graph);
            add_node_to_graph(root->Childrens[i], graph);
        }

        /*/
        NODE *temp = root->Child(root->Obss[i]);
        if(temp){
            edgepp prop;
            prop.x = temp->X();
            prop.y = temp->Y();
            prop.dual = temp->Dual();
            long node1 = root->getIndex();
            long node2 = temp->getIndex();
            add_edge(node1, node2, prop, graph);
        }
        add_node_to_graph(temp, graph);
        /*/
    }
}


void write_to_graphviz(FSC fsc)
{
    NODE*& root=fsc.getRoot();
    long numN = FSC::numNode;
    std::size_t N = numN + 1;
    Graph graph(N);

    add_node_to_graph(root, graph);



    // write the dot file

   std::ofstream dotfile ("policy.dot");
   write_graphviz (dotfile, graph,
                   boost::default_writer(),
                   make_edge_writer(boost::get(&edgepp::x,graph),boost::get(&edgepp::y,graph), boost::get(&edgepp::dual,graph), boost::get(&edgepp::height,graph)));
}
