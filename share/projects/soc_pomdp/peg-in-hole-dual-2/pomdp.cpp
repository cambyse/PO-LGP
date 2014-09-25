

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
   double pos;
 };

//////////////////////////////////////////

typedef property<vertex_distance_t, float,
           property<vertex_name_t, std::string> > VertexProperty;

typedef adjacency_list<listS, vecS, bidirectionalS, VertexProperty, edgepp> Graph;




template <class X, class Y, class Dual, class Pos>
class edge_writer {
public:
    edge_writer(X x,Y y,Dual dual, Pos pos) : cx(x), cy(y), cdual(dual), cpos(pos) {}
  template <class Edge>
  void operator()(ostream &out, const Edge& e) const {
      //out << "[label=\"dual = " << cdual[e] <<"  x= "<<cx[e] <<"  y= "<<cy[e] << "\"]";
      out << "[label=\"dual = " << cdual[e] <<"  pos = " << cpos[e] <<"  y= "<<cy[e] << "\"]";
  }
private:  
  X cx;
  Y cy;
  Dual cdual;
  Pos cpos;
};

template  <class X, class Y, class Dual, class Pos>
inline edge_writer<X,Y,Dual,Pos> make_edge_writer(X x,Y y,Dual dual,Pos pos) {
  return edge_writer<X,Y,Dual,Pos>(x,y,dual,pos);
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
    model.resize(2);
}

NODE::~NODE()
{

}

inline bool equal(const Observation obs1,const Observation obs2)
{
    if((obs1.height == obs2.height) && (obs1.pos == obs2.pos) &&  (obs1.x == obs2.x) && (obs1.y == obs2.y) && (obs1.dual == obs2.dual))
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
    if((node->Samples().d0 == 1) && (node->AllDual()(0)>0)){
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
inline void extract4(arr a1, arr& a2)
{
    a2.resize(a1.d0,a1.d1-1,a1.d2);
    for(int i=0;i<a1.d0;i++){
        for(int t=1;t < a1.d1;t++)
            a2[i][t-1]() = a1[i][t];
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

inline bool check_next_observation_edge (NODE *&node, double curr_x)
{
    //mean that the current x already passed through a specific model's edge
    for(int i=0;i<node->Samples().d0;i++){
        if((curr_x<=0)&&(node->Samples()[i](1) > curr_x))
            return false;
        if((curr_x>0)&&(node->Samples()[i](1) < curr_x))
            return false;
    }
    return true;
}


void OptimizeFSC_Test(ors::KinematicWorld& world, NODE *&node, int horizon)
{
    arr x, y, dual;

    arr Samples = node->Samples();
    arr x0 = node->X();
    for(uint i=0;i < Samples.d0;i++){
        getTrajectory(x, y, dual, world, x0, Samples[i], true, FSC::Horizon-horizon);
        cout<<dual<<endl;
    }

}

NODE* addNode_MultiParticle(NODE*& node,Observation obs, int steps)
{
    node->Add(obs);
    NODE*temp = node->Child(obs);

    temp->X() = node->AllX()[steps]();
    temp->Y() = node->AllY()[steps]();
    temp->Dual() = node->AllDual()(steps);
    temp->Samples() = node->Samples();
    temp->Model() = node->Model();


    extract(node->AllX(),temp->AllX());
    extract(node->AllY(),temp->AllY());
    extract2(node->AllDual(),temp->AllDual());

    extract3(node->getSampleDual(),temp->getSampleDual());
    extract4(node->getSampleX(),temp->getSampleX());
    extract4(node->getSampleY(),temp->getSampleY());

    return temp;
}


inline bool distinct(arr distint_heights,double check)
{
    for(int i=0;i<distint_heights.d0;i++)
        if(distint_heights(i)==check) return true;

    return false;
}

int choose_Next_Obs(arr samples)
{
    int best_index;
    double best_value_1 = 100000; //this 1-d problem, we choose the lowest table (which give best entropy, or one-step look-ahead)
    double best_value_2 = 0;
    for(int i=0;i<samples.d0;i++){
        if((samples[i](0)<=best_value_1)||(fabs(samples[i](1))>=fabs(best_value_2))){
            best_value_1 = samples[i](0);
            best_value_2 = samples[i](1);
            best_index = i;
        }
    }
    return best_index;
}

int choose_Next_Obs_2(arr samples)
{
    /*/
    int best_index;
    double best_value = 0;
    for(int i=0;i<samples.d0;i++){
        if(fabs(samples[i](1))>fabs(best_value)){
            best_value = samples[i](1);
            best_index = i;
        }
    }
    return best_index;
    /*/
    return choose_Next_Obs(samples);
}

//OPTIMIZING an FSC
void OptimizeFSC(ors::KinematicWorld& world, NODE*& node, int horizon)
{
    int steps = 1;

    if(!node) return;


    FSC::numNode = FSC::numNode + 1; //static variable to count the number of generated nodes
    node->setIndex(FSC::numNode);   //indexing the node

    if(horizon >= FSC::Horizon - 1) return;

    double current_obs = node->Dual();

    if((horizon==0) || (!check_next_observation(node->getSampleDual(),current_obs))){
        //this mean: only has change of observation (0->1; 1->0) then re-plan.

        arr samples = node->Samples();
        arr x0 = node->X();

        arr samples_nev,samples_pos;
        samples_nev.resize(samples.d0,2);
        samples_pos.resize(samples.d0,2);

        arr sampleDual;//, sampleDual_nev, sampleDual_pos;        
        sampleDual.resize(samples.d0, FSC::Horizon-horizon + 1);


        //arr sampleX, sampleX_nev, sampleX_pos;
        //arr sampleY, sampleY_nev, sampleY_pos;

        //sampleX.resize(samples.d0, FSC::Horizon-horizon + 1,7);
        //sampleY.resize(samples.d0, FSC::Horizon-horizon + 1,3);

        //sampleDual_nev.resize(samples.d0, FSC::Horizon-horizon + 1);
        //sampleX_nev.resize(samples.d0, FSC::Horizon-horizon + 1,7);
        //sampleY_nev.resize(samples.d0, FSC::Horizon-horizon + 1,3);

        //sampleDual_pos.resize(samples.d0, FSC::Horizon-horizon + 1);

        arr allTraj_X_nev;
        arr allTraj_Y_nev;
        arr allTraj_Dual_nev;

        arr allTraj_X_pos;
        arr allTraj_Y_pos;
        arr allTraj_Dual_pos;


        int index_nev=0;
        int index_pos=0;
        bool set=false;
        int dimx=0;
        int dimy=0;
        int dimx1=0;
        int dimy1=0;
        int dim_dual=0;


        for(uint i=0;i < samples.d0;i++){
            arr x, y, dual;
            getTrajectory(x, y, dual, world, x0, samples[i], true, FSC::Horizon-horizon);

            if(!set){
                allTraj_X_nev.resize(samples.d0,x.d0,x.d1);
                allTraj_Y_nev.resize(samples.d0,y.d0,y.d1);
                allTraj_Dual_nev.resize(samples.d0,dual.d0);

                allTraj_X_pos.resize(samples.d0,x.d0,x.d1);
                allTraj_Y_pos.resize(samples.d0,y.d0,y.d1);
                allTraj_Dual_pos.resize(samples.d0,dual.d0);
                set = true;
                dimx = x.d0;
                dimx1 = x.d1;
                dimy = y.d0;
                dimy1 = y.d1;
                dim_dual = dual.d0;
            }
            sampleDual[i]() = dual;


            if(dual(steps)<=0){
                samples_nev[index_nev]() = samples[i];
                //sampleDual_nev[index_nev]() = dual;
                allTraj_X_nev[index_nev]() = x;
                allTraj_Y_nev[index_nev]() = y;
                allTraj_Dual_nev[index_nev]() = dual;
                index_nev = index_nev + 1;
            }else{

            //if changes: add all trajectory into the FSC
                samples_nev[index_pos]() = samples[i];
                //sampleDual_pos[index_pos]() = dual;
                allTraj_X_pos[index_pos]() = x;
                allTraj_Y_pos[index_pos]() = y;
                allTraj_Dual_pos[index_pos]() = dual;
                index_pos = index_pos + 1;
            }
        }

        //sampleDual_nev.resize(index_nev, FSC::Horizon-horizon  + 1);
        allTraj_X_nev.resize(index_nev, dimx,dimx1);
        allTraj_Y_nev.resize(index_nev, dimy, dimy1);
        allTraj_Dual_nev.resize(index_nev, dim_dual);
        samples_nev.resize(index_nev, 2);

        //sampleDual_pos.resize(index_pos, FSC::Horizon-horizon  + 1);
        allTraj_X_pos.resize(index_pos, dimx,dimx1);
        allTraj_Y_pos.resize(index_pos, dimy, dimy1);
        allTraj_Dual_pos.resize(index_pos, dim_dual);
        samples_pos.resize(index_pos, 2);


        if (samples_nev.d0 > 0) {
             int index = choose_Next_Obs(samples_nev);

             Observation obs;
             obs.x = allTraj_X_nev[index][steps]();
             obs.y = allTraj_Y_nev[index][steps]();
             obs.dual = allTraj_Dual_nev[index](steps);
             obs.height = samples_nev[index](0);
             obs.pos   = samples_nev[index](1);


             //trajectory of the root node (also always have samples_nev >0)
             if(horizon==0)
             {
                 node->AllX() = allTraj_X_nev[index];
                 node->AllY() = allTraj_Y_nev[index];
                 node->AllDual() = allTraj_Dual_nev[index];
                 node->getSampleDual() = sampleDual; //dual traj from all samples
                 node->Model() = samples_nev[index];
             }

            node->Add(obs);
            NODE*temp = node->Child(obs);

            /*/if((current_obs <= 0)&&(horizon!=0)){
                //same observation with parent, then use the next state in its parent's traj
                temp->X() = node->AllX()[steps]();
                temp->Y() = node->AllY()[steps]();
                temp->Dual() = node->AllDual()(steps);
                temp->Samples() = node->Samples();
                temp->Model()  = node->Model();
                extract(node->AllX(),temp->AllX());
                extract(node->AllY(),temp->AllY());
                extract2(node->AllDual(),temp->AllDual());

            }else{/*/
            extract(allTraj_X_nev[index](),temp->AllX());
            extract(allTraj_Y_nev[index](),temp->AllY());
            extract2(allTraj_Dual_nev[index](),temp->AllDual());
            temp->Samples() = samples_nev;
            temp->Model() = samples_nev[index];
            temp->X() = temp->AllX()[index]();
            temp->Y() = temp->AllY()[index]();
            temp->Dual() = temp->AllDual()(index);
            /*/}/*/

            //extract3(sampleDual_nev,temp->getSampleDual());
            extract3(allTraj_Dual_nev,temp->getSampleDual());
            extract4(allTraj_X_nev,temp->getSampleX());
            extract4(allTraj_Y_nev,temp->getSampleY());
        }

         if (samples_pos.d0 > 0) {
             arr distint_heights;
             for(ixznt in=0;in<samples_pos.d0;in++){
                 if(!distinct(distint_heights,samples_pos[in](0)))
                     distint_heights.append(samples_pos[in](0));
             }

             for(int in=0;in<distint_heights.d0;in++){
                 arr samples_temp;//,sampleDual_temp;
                 arr allTraj_X_temp, allTraj_Y_temp, allTraj_Dual_temp;
                 uint index_temp = 0;

                 for(int i=0;i<samples_pos.d0;i++){
                     if(distint_heights(in) == samples_pos[i](0)){
                         samples_temp.resize(index_temp+1,2);
                         //sampleDual_temp.resize(index_temp+1,FSC::Horizon-horizon  + 1);
                         allTraj_X_temp.resize(index_temp+1,dimx,dimx1);
                         allTraj_Y_temp.resize(index_temp+1,dimy,dimy1);
                         allTraj_Dual_temp.resize(index_temp+1,dim_dual);

                         samples_temp[index_temp]() = samples_pos[i];
                         //sampleDual_temp[index_temp]() = sampleDual_pos[i];
                         allTraj_X_temp[index_temp]() = allTraj_X_pos[i];
                         allTraj_Y_temp[index_temp]() = allTraj_Y_pos[i];
                         allTraj_Dual_temp[index_temp]() = allTraj_Dual_pos[i];
                         index_temp = index_temp + 1;
                     }
                 }
                 //add the cluster into FSC
                 if (samples_temp.d0 > 0) {
                      int index = choose_Next_Obs_2(samples_temp);

                      Observation obs;
                      obs.x = allTraj_X_temp[index][steps]();
                      obs.y = allTraj_Y_temp[index][steps]();
                      obs.dual = allTraj_Dual_temp[index](steps);
                      obs.height = samples_temp[index](0);
                      obs.pos    = samples_temp[index](1);

                      node->Add(obs);
                      NODE*temp = node->Child(obs);


                     extract(allTraj_X_temp[index](),temp->AllX());
                     extract(allTraj_Y_temp[index](),temp->AllY());
                     extract2(allTraj_Dual_temp[index](),temp->AllDual());
                     temp->Samples() = samples_temp;
                     temp->Model() = samples_temp[index];
                     temp->X() = temp->AllX()[index]();
                     temp->Y() = temp->AllY()[index]();
                     temp->Dual() = temp->AllDual()(index);


                     //extract3(sampleDual_temp,temp->getSampleDual());
                     extract3(allTraj_Dual_temp,temp->getSampleDual());
                     extract4(allTraj_X_temp,temp->getSampleX());
                     extract4(allTraj_Y_temp,temp->getSampleY());
                 }


             }

         }

                 //recursively optimize all children
        for(int no=0; no < node->Childrens.size();no++){
            //if(node->)
            NODE* child = node->Child(node->Obss[no]);
            if(node->Dual()<=0)
                OptimizeFSC(world, child, horizon + steps);
            else
                OptimizeFSC_Edges(world, child, horizon + steps);
        }

    }else{ //if there is no change in observation (then continue following the parent's traj

        Observation obs;
        obs.x = node->AllX()[steps]();
        obs.y = node->AllY()[steps]();
        obs.dual = node->AllDual()(steps);
        obs.height = node->Model()(0);
        obs.pos = node->Model()(1);

        NODE* temp = addNode_MultiParticle(node, obs, steps);
        if(node->Dual()<=0)
            OptimizeFSC(world, temp, horizon + steps);
        else
            OptimizeFSC_Edges(world, temp, horizon + steps);

    }
}

//OPTIMIZING an FSC
void OptimizeFSC_Edges(ors::KinematicWorld& world, NODE*& node, int horizon)
{
    int steps = 1;

    if(!node) return;


    FSC::numNode = FSC::numNode + 1;
    node->setIndex(FSC::numNode);

    if(horizon >= FSC::Horizon - 1) return;

    arr y0 = node->Y();


    if(check_next_observation_edge(node,y0(0))){

        arr samples = node->Samples();
        arr x0 = node->X();

        arr samples_nev;
        samples_nev.resize(samples.d0,2);

        arr sampleDual;//, sampleDual_nev;
        sampleDual.resize(samples.d0, FSC::Horizon-horizon + 1);
        //sampleDual_nev.resize(samples.d0, FSC::Horizon-horizon + 1);


        arr allTraj_X_nev;
        arr allTraj_Y_nev;
        arr allTraj_Dual_nev;


        int index_nev=0;

        int dimx=FSC::Horizon-horizon+1; //with terminal
        int dimy=FSC::Horizon-horizon+1;
        int dimx1=7;
        int dimy1=3;
        int dim_dual=FSC::Horizon-horizon+1;

        allTraj_X_nev.resize(samples.d0,dimx,dimx1);
        allTraj_Y_nev.resize(samples.d0,dimy,dimy1);
        allTraj_Dual_nev.resize(samples.d0,dim_dual);


        for(uint i=0;i < samples.d0;i++){
            sampleDual[i]() = node->getSampleDual()[i];
            if(samples[i](1) > fabs(node->getSampleY()[i][steps](0))){
                samples_nev[index_nev]() = samples[i];
                //sampleDual_nev[index_nev]() = dual;
                allTraj_X_nev[index_nev]() = node->getSampleX()[i];
                allTraj_Y_nev[index_nev]() = node->getSampleY()[i];
                allTraj_Dual_nev[index_nev]() = node->getSampleDual()[i];
                index_nev = index_nev + 1;
            }else{
                arr x, y, dual;
                arr x0 = node->X();
                getTrajectory(x, y, dual, world, x0, samples[i], true, FSC::Horizon-horizon);

                Observation obs;
                obs.x = x[steps]();
                obs.y = y[steps]();
                obs.dual = dual(steps);
                obs.height = samples[i](0);
                obs.pos = samples[i](1);

                node->Add(obs);
                NODE*temp = node->Child(obs);

                temp->X() = x[steps]();
                temp->Y() = y[steps]();
                temp->Dual() = dual(steps);
                temp->Samples().resize(1,2);
                temp->Samples()[0] = samples[i];
                temp->Model() = samples[i];


                extract(x,temp->AllX());
                extract(y,temp->AllY());
                extract2(dual,temp->AllDual());

                arr temp_sampleDual;
                temp_sampleDual.resize(1, FSC::Horizon-horizon  + 1);
                temp_sampleDual[0]() = dual;
                extract3(temp_sampleDual,temp->getSampleDual());

                arr temp_sampleX;
                temp_sampleX.resize(1, FSC::Horizon-horizon  + 1,7);
                temp_sampleX[0]() = x;
                extract4(temp_sampleX,temp->getSampleX());

                arr temp_sampleY;
                temp_sampleY.resize(1, FSC::Horizon-horizon  + 1,3);
                temp_sampleY[0]() = y;
                extract4(temp_sampleY,temp->getSampleY());


            }
        }

        //sampleDual_nev.resize(index_nev, FSC::Horizon-horizon  + 1);
        allTraj_X_nev.resize(index_nev, dimx,dimx1);
        allTraj_Y_nev.resize(index_nev, dimy, dimy1);
        allTraj_Dual_nev.resize(index_nev, dim_dual);
        samples_nev.resize(index_nev, 2);

        //sampleDual_pos.resize(index_pos, FSC::Horizon-horizon  + 1);
        //allTraj_X_pos.resize(index_pos, dimx,dimx1);
        //allTraj_Y_pos.resize(index_pos, dimy, dimy1);
        //allTraj_Dual_pos.resize(index_pos, dim_dual);
        //samples_pos.resize(index_pos, 2);


        if (samples_nev.d0 > 0) {
             int index = choose_Next_Obs_2(samples_nev);

             Observation obs;
             obs.x = allTraj_X_nev[index][steps]();
             obs.y = allTraj_Y_nev[index][steps]();
             obs.dual = allTraj_Dual_nev[index](steps);
             obs.height = samples_nev[index](0);
             obs.pos   = samples_nev[index](1);


            node->Add(obs);
            NODE*temp = node->Child(obs);


            extract(allTraj_X_nev[index](),temp->AllX());
            extract(allTraj_Y_nev[index](),temp->AllY());
            extract2(allTraj_Dual_nev[index](),temp->AllDual());
            temp->Samples() = samples_nev;
            temp->Model() = samples_nev[index];
            temp->X() = temp->AllX()[index]();
            temp->Y() = temp->AllY()[index]();
            temp->Dual() = temp->AllDual()(index);;


            extract3(allTraj_Dual_nev,temp->getSampleDual());
            extract4(allTraj_X_nev,temp->getSampleX());
            extract4(allTraj_Y_nev,temp->getSampleY());
        }

                 //recursively optimize all children
        for(int no=0; no < node->Childrens.size();no++){
            NODE* child = node->Child(node->Obss[no]);
            OptimizeFSC_Edges(world, child, horizon + steps);
        }

    }else{ //if there is no change in observation (then continue following the parent's traj

        Observation obs;
        obs.x = node->AllX()[steps]();
        obs.y = node->AllY()[steps]();
        obs.dual = node->AllDual()(steps);
        obs.height = node->Model()(0);
        obs.pos = node->Model()(1);

        NODE* temp = addNode_MultiParticle(node, obs, steps);
        OptimizeFSC_Edges(world, temp, horizon + steps);

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
            prop.pos = root->Childrens[i]->Model()(1);

            long node1 = root->getIndex();
            long node2 = root->Childrens[i]->getIndex();
            add_edge(node1, node2, prop, graph);
            add_node_to_graph(root->Childrens[i], graph);
        }


        //NODE *temp = root->Child(root->Obss[i]);
        //if(temp){
        //    edgepp prop;
        //    prop.x = temp->X();
        //    prop.y = temp->Y();
        //    prop.dual = temp->Dual();
        //    long node1 = root->getIndex();
        //    long node2 = temp->getIndex();
        //    add_edge(node1, node2, prop, graph);
       // }
        //add_node_to_graph(temp, graph);

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
                   make_edge_writer(boost::get(&edgepp::x,graph),boost::get(&edgepp::y,graph), boost::get(&edgepp::dual,graph), boost::get(&edgepp::pos,graph)));
}



