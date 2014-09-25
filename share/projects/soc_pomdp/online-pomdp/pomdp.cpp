

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
   //double weight;
 };

//////////////////////////////////////////

typedef property<vertex_distance_t, float,
           property<vertex_name_t, std::string> > VertexProperty;

typedef adjacency_list<listS, vecS, bidirectionalS, VertexProperty, edgepp> Graph;




template <class X, class Y, class Dual>
class edge_writer {
public:
    edge_writer(X x,Y y,Dual dual) : cx(x), cy(y), cdual(dual) {}
  template <class Edge>
  void operator()(ostream &out, const Edge& e) const {
      //out << "[label=\"dual = " << cdual[e] <<"  x= "<<cx[e] <<"  y= "<<cy[e] << "\"]";
      out << "[label=\"dual = " << cdual[e] <<"  y= "<<cy[e] << "\"]";
  }
private:  
  X cx;
  Y cy;
  Dual cdual;
};

template  <class X, class Y, class Dual>
inline edge_writer<X,Y,Dual> make_edge_writer(X x,Y y,Dual dual) {
  return edge_writer<X,Y,Dual>(x,y,dual);
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

///

NODE* addNode_SingleParticle(NODE*& node,Observation obs, int steps, double height, int  horizon)
{
    node->Add(obs);
    NODE*temp = node->Child(obs);

    temp->X() = node->AllX()[steps]();
    temp->Y() = node->AllY()[steps]();
    temp->Dual() = node->AllDual()(steps);
    temp->Heights().append(height);
    temp->Height() = height;


    extract(node->AllX(),temp->AllX());
    extract(node->AllY(),temp->AllY());
    extract2(node->AllDual(),temp->AllDual());

    arr temp_sampleDual;
    temp_sampleDual.resize(1, FSC::Horizon-horizon  + 1);
    temp_sampleDual[0]() = node->AllDual();
    extract3(temp_sampleDual,temp->getSampleDual());

    if(node->certainty)
        temp->certainty = true;
    else if(temp->Dual() > 0)
        temp->certainty = true;
    else
        temp->certainty = false;

    return temp;
}

NODE* addNode_SingleParticle(NODE*& node, arr x, arr y, arr dual, Observation obs, int steps, double height, int  horizon)
{
    node->Add(obs);
    NODE*temp = node->Child(obs);

    temp->X() = x[steps]();
    temp->Y() = y[steps]();
    temp->Dual() = dual(steps);
    temp->Heights().append(height);
    temp->Height() = height;


    extract(x,temp->AllX());
    extract(y,temp->AllY());
    extract2(dual,temp->AllDual());

    arr temp_sampleDual;
    temp_sampleDual.resize(1, FSC::Horizon-horizon  + 1);
    temp_sampleDual[0]() = dual;
    extract3(temp_sampleDual,temp->getSampleDual());

    if(node->certainty)
        temp->certainty = true;
    else if(temp->Dual() > 0)
        temp->certainty = true;
    else
        temp->certainty = false;

    return temp;
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

    if(node->certainty)
        temp->certainty = true;
    else if(temp->Dual() > 0)
        temp->certainty = true;
    else
        temp->certainty = false;

    extract(node->AllX(),temp->AllX());
    extract(node->AllY(),temp->AllY());
    extract2(node->AllDual(),temp->AllDual());

    extract3(node->getSampleDual(),temp->getSampleDual());

    return temp;
}

int choose_Next_Obs(arr Heights)
{
    int best_index;
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


    FSC::numNode = FSC::numNode + 1;
    node->setIndex(FSC::numNode);

    if(horizon >= FSC::Horizon - 1) return;

    double current_obs = node->Dual();

    arr heights = node->Heights();
    //this is certainty case: Just add all trajectory into FSC
    if(heights.d0 == 1){
        if(node->certainty){
            if(!node->sticky){
                Observation obs;
                obs.x = node->AllX()[steps]();
                obs.y = node->AllY()[steps]();
                obs.dual = node->AllDual()(steps);
                obs.height = heights(0);
                NODE* temp = addNode_SingleParticle(node, obs, steps, heights(0), horizon);
                temp->sticky = false;
                //recursively add the next node.
                OptimizeFSC(world, temp, horizon + steps);

            }else{ //wolve whithout sticky (means that already certainty, then ignore contacts with constraints)
                arr x, y, dual;
                arr x0 = node->X();
                getTrajectory(x, y, dual, world, x0, heights(0), false, FSC::Horizon-horizon);

                Observation obs;
                obs.x = x[steps]();
                obs.y = y[steps]();
                obs.dual = dual(steps);

                obs.height = heights(0);

                NODE* temp = addNode_SingleParticle(node, x,y,dual, obs, steps, heights(0), horizon);
                temp->sticky = false;

                //recursively add the next node.
                OptimizeFSC(world, temp, horizon + steps);

            }

        }else{
            //check if no change in observation: current and the next (if does not change, then don't have to solve the trajectory again: see else)
            if(!check_next_observation(node->getSampleDual(),current_obs))
            {
                arr x, y, dual;
                arr x0 = node->X();
                getTrajectory(x, y, dual, world, x0, heights(0), true, FSC::Horizon-horizon);

                Observation obs;
                obs.x = x[steps]();
                obs.y = y[steps]();
                obs.dual = dual(steps);
                obs.height = heights(0);

                NODE* temp = addNode_SingleParticle(node, x,y,dual,obs, steps, heights(0), horizon);

                //recursively add the next node.
                OptimizeFSC(world, temp, horizon + steps);

            }else{

                Observation obs;
                obs.x = node->AllX()[steps]();
                obs.y = node->AllY()[steps]();
                obs.dual = node->AllDual()(steps);
                obs.height = node->Height();

                NODE* temp = addNode_MultiParticle(node, obs, steps);
                OptimizeFSC(world, temp, horizon + steps);
            }
        }
    }else{//node has more than 1 particle
        if((horizon==0) || (!check_next_observation(node->getSampleDual(),current_obs))){
            //this mean: only has change of observation (0->1; 1->0) then re-plan.

            arr heights = node->Heights();
            arr x0 = node->X();
            arr allX, allY, allDual; //trajectory for this node

            //for creating new nodes
            //std::list<double> newObs;
            arr x_nev;
            arr y_nev;
            double height_nev;
            double dual_nev;
            arr heights_nev;

            ///////////////////////////////////////
            /// \brief num_obs
            ///

            //Compute

            arr sampleDual, sampleDual_nev, sampleDual_pos;
            sampleDual.resize(heights.d0, FSC::Horizon-horizon + 1);
            sampleDual_nev.resize(heights.d0, FSC::Horizon-horizon + 1);

            arr allTraj_X;
            arr allTraj_Y;
            arr allTraj_Dual;



            arr allDual_nev;
            arr allX_nev;
            arr allY_nev;


            int index_nev=0;
            bool set=false;
            int dimx=0;
            int dimy=0;
            int dim_dual=0;


            for(uint i=0;i < heights.d0;i++){
                arr x, y, dual;
                getTrajectory(x, y, dual, world, x0, heights(i), true, FSC::Horizon-horizon);


                if(!set){
                    allTraj_X.resize(heights.d0,x.d0,x.d1);
                    allTraj_Y.resize(heights.d0,y.d0,y.d1);
                    allTraj_Dual.resize(heights.d0,dual().d0);
                    set = true;
                    dimx = x.d0;
                    dimy = y.d0;
                    dim_dual = dual.d0;
                }
                sampleDual[i]() = dual;


                if(dual(steps)<=0){
                    heights_nev.append(heights(i));
                    sampleDual_nev[index_nev]() = dual;
                    allTraj_X[index_nev]() = x;
                    allTraj_Y[index_nev]() = y;
                    allTraj_Dual[index_nev]() = dual;
                    index_nev = index_nev + 1;
                }else{

                //if changes: add all trajectory into the FSC
                    Observation obs;
                    obs.x = x[steps]();
                    obs.y = y[steps]();
                    obs.dual = dual(steps);
                    obs.height = heights(i);

                    addNode_SingleParticle(node, x, y, dual, obs, steps, heights(i), horizon);

                }
            }

            ////////for the case in [NOTE]

            sampleDual_nev.resize(heights_nev.d0, FSC::Horizon-horizon  + 1);
            allTraj_X.resize(heights_nev.d0, dimx,7);
            allTraj_Y.resize(heights_nev.d0, dimy, 3);
            allTraj_Dual.resize(heights_nev.d0, dim_dual);


            /////////////////////////////////////

            //add Child: with obs = 0 (no force sensing)


            if (heights_nev.d0 > 0) {
                 int index = choose_Next_Obs(heights_nev);

                 Observation obs;
                 obs.x = allTraj_X[index][steps]();
                 obs.y = allTraj_Y[index][steps]();
                 obs.dual = allTraj_Dual[index](steps);
                 obs.height = heights_nev(index);


                 //trajectory of the root node
                 if(horizon==0)
                 {
                     node->AllX() = allTraj_X[index]();
                     node->AllY() = allTraj_Y[index]();
                     node->AllDual() = allTraj_Dual[index]();
                     node->getSampleDual() = sampleDual; //dual traj from all samples
                     node->Height() = heights_nev(index);
                 }

                node->Add(obs);
                NODE*temp = node->Child(obs);
                if(current_obs <= 0){
                    //same observation with parent, then use the next state in its parent's traj
                    temp->X() = node->AllX()[steps]();
                    temp->Y() = node->AllY()[steps]();
                    temp->Dual() = node->AllDual()(steps);
                    temp->Heights() = heights_nev;
                    temp->Height()  = node->Height();
                    extract(node->AllX(),temp->AllX());
                    extract(node->AllY(),temp->AllY());
                    extract2(node->AllDual(),temp->AllDual());

                }else{
                    extract(allTraj_X[index](),temp->AllX());
                    extract(allTraj_Y[index](),temp->AllY());
                    extract2(allTraj_Dual[index](),temp->AllDual());
                    temp->Heights() = heights_nev;
                    temp->Height() = heights_nev(index);
                    temp->X() = temp->AllX()[index]();
                    temp->Y() = temp->AllY()[index]();
                    temp->Dual() = temp->AllDual()(index);
                }

                extract3(sampleDual_nev,temp->getSampleDual());
            }

                     //recursively optimize all children
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

            NODE* temp = addNode_MultiParticle(node, obs, steps);
            OptimizeFSC(world, temp, horizon + steps);

        }


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
                   make_edge_writer(boost::get(&edgepp::x,graph),boost::get(&edgepp::y,graph), boost::get(&edgepp::dual,graph)));
}
