

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
void setInitialPose(const arr x, mlr::KinematicWorld& world)
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
void OptimizeFSC_Test(mlr::KinematicWorld& world, NODE *&node, int horizon)
{
    arr x, y, dual;

    arr heights = node->Heights();
    arr x0 = node->X();
    for(uint i=0;i < heights.d0;i++){
        getTrajectory(x, y, dual, world, x0, heights(i), true, FSC::Horizon-horizon);
        cout<<dual<<endl;
    }

}

//OPTIMIZING an FSC
void OptimizeFSC(mlr::KinematicWorld& world, NODE*& node, int horizon)
{
    int steps = 1;


    FSC::numNode = FSC::numNode + 1;
    node->setIndex(FSC::numNode);

    if(horizon >= FSC::Horizon - 1) return;


    if(node){

        double current_obs = node->Dual();

        //not root and new observation from the next node

        if(((horizon==0) || (!check_next_observation(node->getSampleDual(),current_obs)))&&(node->Heights().d0>1)){
            //this mean: only has change of observation (0->1; 1->0) then re-plan.

            arr heights = node->Heights();
            arr x0 = node->X();
            arr allX, allY, allDual; //trajectory for this node
            bool set=false;

            //for creating new nodes
            //std::list<double> newObs;
            arr x_nev;
            arr y_nev;
            double height_nev;
            double dual_nev;
            arr heights_nev;

            arr x_pos;
            arr y_pos;
            double height_pos;
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

            int index_nev=0;
            int index_pos=0;


            for(uint i=0;i < heights.d0;i++){
                arr x, y, dual;
                //compute the stickyness               
                /////////
                /// \brief getTrajectory
                ///

                getTrajectory(x, y, dual, world, x0, heights(i), true, FSC::Horizon-horizon);

                //cout<< dual<<endl;

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
                if (((dual(0) > 0) && (dual(steps) > 0)) || ((dual(0) <= 0) && (dual(steps)<= 0))){
                    //[NOTE] if the observation does not change through 2-time steps
                    if(dual(steps)<=0){
                        x_nev = x[steps]();
                        y_nev = y[steps]();
                        dual_nev = dual(steps);
                        height_nev =  heights(i);
                        heights_nev.append(heights(i));
                        allDual_nev = dual;

                        allX_nev = x;
                        allY_nev = y;

                        sampleDual_nev[index_nev]() = dual;
                        index_nev = index_nev + 1;
                    }else{
                        x_pos = x[steps]();
                        y_pos = y[steps]();
                        dual_pos = dual(steps);
                        height_pos =  heights(i);
                        heights_pos.append(heights(i));
                        allDual_pos = dual;
                        allX_pos = x;
                        allY_pos = y;

                        sampleDual_pos[index_pos]() = dual;
                        index_pos = index_pos + 1;
                    }
                }else{
                    //if changes: add all trajectory into the FSC
                    Observation obs;
                    obs.x = x[steps]();
                    obs.y = y[steps]();
                    obs.dual = dual(steps);
                    obs.height = heights(i);

                    //cout<<" x "<< obs.x<<endl;
                    //cout<<" y "<< obs.y<<endl;
                    //cout<<" dual "<< obs.dual<<endl;

                    node->Add(obs);
                    NODE*temp = node->Child(obs);

                    temp->X() = x[steps]();
                    temp->Y() = y[steps]();
                    temp->Dual() = dual(steps);
                    temp->Heights().append(heights(i));


                    extract(x,temp->AllX());
                    extract(y,temp->AllY());

                    extract2(dual,temp->AllDual());


                    arr temp_sampleDual;
                    temp_sampleDual.resize(1, FSC::Horizon-horizon  + 1);
                    temp_sampleDual[0]() = dual;

                    extract3(temp_sampleDual,temp->getSampleDual());

                    //OptimizeFSC(world, temp, horizon + steps);

                }
            }

            ////////for the case in [NOTE]

            sampleDual_nev.resize(heights_nev.d0, FSC::Horizon-horizon  + 1);
            sampleDual_pos.resize(heights_pos.d0, FSC::Horizon-horizon  + 1);

           // cout<<sampleDual_nev<<endl;
            // SET trajectory for this root node
            node->AllX().clear();
            node->AllX() = allX;
            node->AllY().clear();
            node->AllY() = allY;
            node->AllDual().clear();
            node->AllDual() = allDual;
            node->getSampleDual().clear();
            node->getSampleDual() = sampleDual; //dual traj from all samples

            /////////////////////////////////////

            //add Child: with obs = 0 (no force sensing)


            if (heights_nev.d0>0) {
                 cout<<"Negative = "<<heights_nev.d0<<endl;

                 Observation obs;
                 obs.x = x_nev;
                 obs.y = y_nev;
                 obs.dual = dual_nev;
                 obs.height = height_nev;

                node->Add(obs);
                NODE*temp = node->Child(obs);
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

                extract3(sampleDual_nev,temp->getSampleDual());
                //OptimizeFSC(world, temp, horizon + steps);
            }

            //add Child: with obs = 0 (no force sensing)
            if (heights_pos.d0>0) {
                cout<<"Positive = "<<heights_pos.d0<<endl;


                Observation obs;
                obs.x = x_nev;
                obs.y = y_nev;
                obs.dual = dual_nev;
                obs.height = height_pos;

               node->Add(obs);
               NODE*temp = node->Child(obs);

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
                extract3(sampleDual_pos,temp->getSampleDual());
                //recursive update this child node
                //OptimizeFSC(world, temp, horizon + steps);
            }


            //recursively optimize all children
            for(int no=0; no < node->Childrens.size();no++){
                NODE* child = node->Child(node->Obss[no]);
                OptimizeFSC(world, child, horizon + steps);
            }


            allX.clear(); allY.clear(); allDual.clear();

        }else if((node->Heights().d0==1)&&(node->AllDual()(0)>0)&&(!node->certainty)){
            //this mean: only has change of observation (0->1; 1->0) then re-plan.

            arr heights = node->Heights();
            arr x0 = node->X();
            arr allX, allY, allDual; //trajectory for this node
            bool set=false;

            //for creating new nodes
            //std::list<double> newObs;
            arr x_nev;
            arr y_nev;
            double height_nev;
            double dual_nev;
            arr heights_nev;

            arr x_pos;
            arr y_pos;
            double height_pos;
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

            int index_nev=0;
            int index_pos=0;

            bool stickyWeight = stickyCompute(node);


            for(uint i=0;i < heights.d0;i++){
                arr x, y, dual;
                //compute the stickyness
                /////////
                /// \brief getTrajectory
                ///

                getTrajectory(x, y, dual, world, x0, heights(i), false, FSC::Horizon-horizon);

                //cout<<" t= "<<heights(i)<<"   dual = "<<dual<<endl;
                //cout<<" y " <<y[1]() <<endl;


                //x,y,dual is a full traj of the model heights(i) START HERE

                if(!set){
                    allX = x;
                    allY = y;
                    allDual = dual;
                    set = true;
                }

                sampleDual[i]() = dual;

                //check observation of the next time step
                if (((dual(0) > 0) && (dual(steps) > 0)) || ((dual(0) <= 0) && (dual(steps)<= 0))){
                    //[NOTE] if the observation does not change through 2-time steps
                    if(dual(steps)<=0){
                        x_nev = x[steps]();
                        y_nev = y[steps]();
                        dual_nev = dual(steps);
                        height_nev =  heights(i);
                        heights_nev.append(heights(i));
                        allDual_nev = dual;

                        allX_nev = x;
                        allY_nev = y;

                        sampleDual_nev[index_nev]() = dual;
                        index_nev = index_nev + 1;
                    }else{
                        x_pos = x[steps]();
                        y_pos = y[steps]();
                        dual_pos = dual(steps);
                        height_pos =  heights(i);
                        heights_pos.append(heights(i));
                        allDual_pos = dual;
                        allX_pos = x;
                        allY_pos = y;

                        sampleDual_pos[index_pos]() = dual;
                        index_pos = index_pos + 1;
                    }
                }else{
                    //if changes: add all trajectory into the FSC
                    Observation obs;
                    obs.x = x[steps]();
                    obs.y = y[steps]();
                    obs.dual = dual(steps);
                    obs.height = heights(i);

                    //cout<<" x "<< obs.x<<endl;
                    //cout<<" y "<< obs.y<<endl;
                    //cout<<" dual "<< obs.dual<<endl;

                    node->Add(obs);
                    NODE*temp = node->Child(obs);

                    temp->X() = x[steps]();
                    temp->Y() = y[steps]();
                    temp->Dual() = dual(steps);
                    temp->Heights().append(heights(i));

                    if(node->certainty) temp->certainty = true;


                    extract(x,temp->AllX());
                    extract(y,temp->AllY());

                    extract2(dual,temp->AllDual());


                    arr temp_sampleDual;
                    temp_sampleDual.resize(1, FSC::Horizon-horizon  + 1);
                    temp_sampleDual[0]() = dual;

                    extract3(temp_sampleDual,temp->getSampleDual());

                    //OptimizeFSC(world, temp, horizon + steps);

                }
            }

            ////////for the case in [NOTE]

            sampleDual_nev.resize(heights_nev.d0, FSC::Horizon-horizon  + 1);
            sampleDual_pos.resize(heights_pos.d0, FSC::Horizon-horizon  + 1);

           // cout<<sampleDual_nev<<endl;
            // SET trajectory for this root node
            node->AllX().clear();
            node->AllX() = allX;
            node->AllY().clear();
            node->AllY() = allY;
            node->AllDual().clear();
            node->AllDual() = allDual;
            node->getSampleDual().clear();
            node->getSampleDual() = sampleDual; //dual traj from all samples

            /////////////////////////////////////

            //add Child: with obs = 0 (no force sensing)


            if (heights_nev.d0>0) {
                 cout<<"Negative = "<<heights_nev.d0<<endl;

                 Observation obs;
                 obs.x = x_nev;
                 obs.y = y_nev;
                 obs.dual = dual_nev;
                 obs.height = height_nev;

                node->Add(obs);
                NODE*temp = node->Child(obs);
                temp->X() = x_nev;
                temp->Y() = y_nev;
                temp->Dual() = dual_nev;
                temp->Heights() = heights_nev;
                extract(allX_nev,temp->AllX());
                extract(allY_nev,temp->AllY());

                if(node->certainty) temp->certainty = true;

                extract2(allDual_nev,temp->AllDual());

                extract3(sampleDual_nev,temp->getSampleDual());
                //OptimizeFSC(world, temp, horizon + steps);
            }

            //add Child: with obs = 0 (no force sensing)
            if (heights_pos.d0>0) {
                cout<<"Positive = "<<heights_pos.d0<<endl;


                Observation obs;
                obs.x = x_nev;
                obs.y = y_nev;
                obs.dual = dual_nev;
                obs.height = height_pos;

               node->Add(obs);
               NODE*temp = node->Child(obs);


                temp->X() = x_pos;
                temp->Y() = y_pos;
                temp->Dual() = dual_pos;
                temp->Heights() = heights_pos;

                if(node->certainty) temp->certainty = true;

                extract(allX_pos,temp->AllX());
                extract(allY_pos,temp->AllY());

                extract2(allDual_pos,temp->AllDual());
                extract3(sampleDual_pos,temp->getSampleDual());
                //recursive update this child node
                //OptimizeFSC(world, temp, horizon + steps);
            }


            //recursively optimize all children
            for(int no=0; no < node->Childrens.size();no++){
                NODE* child = node->Child(node->Obss[no]);
                OptimizeFSC(world, child, horizon + steps);
            }



            allX_nev.clear();
            allY_nev.clear();
            allDual_nev.clear();
            sampleDual_nev.clear();
            heights_nev.clear();

            allX_pos.clear();
            allY_pos.clear();
            allDual_pos.clear();
            sampleDual_pos.clear();
            heights_pos.clear();

            allX.clear(); allY.clear(); allDual.clear();



        }else{ //if there is no change in observation (then continue following the parent's traj
            arr allX = node->AllX();
            arr allY = node->AllY();
            arr dual = node->AllDual();
            arr allHeights = node->Heights();

            arr samples = node->getSampleDual();

            NODE*temp;
            //cout<< dual<<endl;

            //cout<< allHeights<<endl;

            Observation obs;
            obs.x = allX[steps]();
            obs.y = allY[steps]();
            obs.dual = dual(steps);
            obs.height = allHeights(allHeights.d0-1);

            node->Add(obs);
            //temp = node->Child(obs);
            temp = node->Childrens[0];

            temp->X() = allX[steps]();
            temp->Y() = allY[steps]();
            temp->Dual() = dual(steps);
            temp->Heights() = allHeights;

            if(node->certainty) temp->certainty = true;

            extract(allX,temp->AllX());
            extract(allY,temp->AllY());
            extract2(dual,temp->AllDual());

            extract3(samples,temp->getSampleDual());

            OptimizeFSC(world, temp, horizon + steps);

            allX.clear();
            allY.clear();
            dual.clear();
            allHeights.clear();

        }

    }



}
inline void add_node_to_graph(NODE*& root, Graph& graph)
{
    for(int i=0;i<root->Obss.size();i++){
        if(root->getIndex() == 23)
            cout<< root->getIndex() <<endl;
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
