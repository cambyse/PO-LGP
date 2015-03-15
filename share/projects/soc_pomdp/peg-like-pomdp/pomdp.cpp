

#include <boost/config.hpp>
//#include <iostream>
#include <fstream>
#include <utility>
#include <algorithm>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graphviz.hpp>
#include "pomdp.h"
#include "execution.h"
#include <vector>
#include <list>

//width of the table (along x-axis)
#define WIDTH 0.5


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


//this return true: when the trajectory of the node has not yet passed any partile's x-axis.
//that means there have been not yet observed change in observations.
//return false otherwise
inline bool check_next_observation_edge (NODE *&node, double curr_x)
{
    //mean that the current x already passed through a specific model's edge
    // curr_x means current end-eff
    double x0 = node->AllY()[0](0); //first step
    double x1 = node->AllY()[1](0); //second step

    for(int i = 0;i < node->Samples().d0;i++){
        if(((x1-x0) <= 0)&&(node->Samples()[i](1) - WIDTH > curr_x)) //moving left
            return false;
        if(((x1-x0) >  0)&&(node->Samples()[i](1) + WIDTH < curr_x)) //moving right
            return false;
    }
    return true;
}

inline bool trajectory_pass_pos (double pos, arr y, double x_target, double z_target)
{
    //arr curr_target = ARR(x_target,0.0,z_target+0.02);

    arr end_eff = y[1];// + curr_target; //y[1] means next step; curr_target of the current followed model (in the node)


    //cout<<x_target <<z_target <<endl;
    //cout<<end_eff <<"  "<< y[1] << "  "<<curr_target  <<endl;

    //0.5 is the width: pos + WIDTH means the left edge
    if((fabs(z_target-end_eff(2))<0.1)&&(pos - WIDTH > end_eff(0))) //assuming that the pos is always negative
        return true; // the trajectory pass this heigth at the next step [STRICTLY]
    else return false; //not yet pass

}
inline bool trajectory_not_pass_models (arr models, arr y, double x_target, double z_target)
{

    for(int i = 0;i<models.d0; i++){
        if(trajectory_pass_pos(models[i](1),y,x_target,z_target))
            return false;
    }

    //cout<<"  ==="<<endl;
    return true; //not yet pass all heights
}




void OptimizeFSC_Test(ors::KinematicWorld& world, NODE *&node, int horizon)
{
    arr x, y, dual;

    arr Samples = node->Samples();
    arr x0 = node->X();
    for(uint i=0;i < Samples.d0;i++){
        getTrajectory(x, y, dual, world, x0, Samples[i], true, FSC::Horizon-horizon);
        //cout<<dual<<endl;
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


    temp->certainty = node->certainty;

    //cout<< " multi "<<temp->Model() <<endl;


    extract(node->AllX(),temp->AllX());
    extract(node->AllY(),temp->AllY());
    extract2(node->AllDual(),temp->AllDual());

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
    int best_index = 0;
    double best_value_1 = 100000; //this 1-d problem, we choose the lowest table (which give best entropy, or one-step look-ahead)


    for(int i=0;i<samples.d0;i++){
        if(samples[i](1) <= best_value_1){
            best_value_1 = samples[i](1);
            best_index = i;
        }
    }
    return best_index;
}

int choose_Next_Obs_2(arr samples)
{
    return choose_Next_Obs(samples);
}

//OPTIMIZING an FSC
void OptimizeFSC(ors::KinematicWorld& world, NODE*& node, int horizon)
{
    int steps = 1;

    if(!node) return;  

    FSC::numNode = FSC::numNode + 1; //static variable to count the number of generated nodes
    node->setIndex(FSC::numNode);   //indexing the node

    if(horizon >= FSC::Horizon ) return;

    arr samples = node->Samples();

    if((((node->Samples().d0>1)&&((horizon==0)||(!trajectory_not_pass_models(samples,node->AllY(),node->Model()(1),node->Model()(0))))))  || ((!node->certainty) && (node->Samples().d0==1))){
        //CHECK Again: if the best model is still in the node, then don't need replanning
        arr x0 = node->X();

        //cout<<" x0 "<<x0<<endl;

        int best_obs = choose_Next_Obs(samples);
        arr x, y, dual;
        long adaptive_horizon = FSC::Horizon;
        long offset = 100;

        if(node->Samples().d0 == 1)
            adaptive_horizon = FSC::Horizon;
        else
            adaptive_horizon = FSC::Horizon-offset;

        //getTrajectory(x, y, dual, world, x0, samples[best_obs], true, FSC::Horizon-horizon);
        getTrajectory(x, y, dual, world, x0, samples[best_obs], true, adaptive_horizon-horizon);


        //resize x,y,dual
        if(node->Samples().d0 >1){
            arr x_prefix;
            x_prefix.resize(7);
            x_prefix = x[x.d0-1];
            for(int i=0;i<offset;i++){
                x.append(x_prefix);
                y.append(ARR(0.0,0.0,0.0));
                dual.append(0.0);
            }
              //x.resize(x.d0+offset,x.d1);
              //y.resize(y.d0+offset,y.d1);
              //dual.resize(dual.d0+offset);
        }


        //re-update this node
        node->X() = x[0];
        node->Y() = y[0];
        node->Dual() = dual(0);
        node->AllX() = x;
        node->AllY() = y;
        node->AllDual() = dual;
        node->Model() = samples[best_obs];


       // cout<<"  node->Y() "<<node->Y()<<endl;
        //cout<<"  node->X() "<<node->X()<<endl;


        if(node->Samples().d0 == 1){
            node->certainty = true;
            //cout<< "certainty "<<endl;
        }

        arr samples_nev;
        samples_nev.resize(samples.d0,2);


        uint new_set = 0;
        for(uint i=0;i < samples.d0;i++){          

            if(!trajectory_pass_pos(samples[i](1),y,samples[best_obs](1),samples[best_obs](0))){
                samples_nev[new_set]() = samples[i];
                new_set += 1;
            }else{ //create new branch
                Observation obs;
                obs.x = x[steps]();
                obs.y = y[steps]();
                obs.dual = dual(steps);
                obs.height = samples[i](0);
                obs.pos = samples[i](1);

                node->Add(obs);
                NODE*temp = node->Child(obs);

                extract(x,temp->AllX());
                extract(y,temp->AllY());
                extract2(dual,temp->AllDual());
                temp->Samples().resize(1,2);
                temp->Samples()[0]() = samples[i];
                temp->Model() = samples[i];
                temp->X() = x[steps]();
                temp->Y() = y[steps]();
                temp->Dual() = dual(steps);

                temp->certainty = node->certainty;


                //cout<<"  temp->Y() "<<temp->Y()<<endl;
                //cout<<"  x[steps]() "<<x[steps]()<<endl;
              // cout<<"  temp->X() "<<temp->X()<<endl;
            }
        }


        samples_nev.resize(new_set,2);


       // cout<<samples_nev <<endl;

        if (samples_nev.d0 > 0) {
             int best_o = choose_Next_Obs(samples_nev);

             Observation obs;
             obs.x = x[steps]();
             obs.y = y[steps]();
             obs.dual = dual(steps);
             obs.height = samples_nev[best_o](0);
             obs.pos    = samples_nev[best_o](1);


            node->Add(obs);
            NODE*temp = node->Child(obs);


            extract(x,temp->AllX());
            extract(y,temp->AllY());
            extract2(dual,temp->AllDual());
            temp->Samples() = samples_nev;
            temp->Model() = samples_nev[best_o];
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
        obs.height = node->Model()(0);
        obs.pos = node->Model()(1);

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
void write(FSC fsc)
{
    std::ofstream dotfile ("policy.txt");
    dotfile << fsc.numNode + 1 <<endl; //number of node




    dotfile.close();

}
