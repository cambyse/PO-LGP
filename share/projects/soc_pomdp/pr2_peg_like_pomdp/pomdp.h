#ifndef POMDP_H
#define POMDP_H

#include <Core/array.h>
#include <vector>
#include <Motion/motion.h>
#include <Motion/taskMap_default.h>
#include <Motion/taskMap_constrained.h>
#include <Motion/feedbackControl.h>
#include <Optim/optimization.h>
#include <Core/util.h>
//#include <Perception/videoEncoder.h>
#include <Gui/opengl.h>


//NODE in FSC
class NODE;

class FSC{


public:
    FSC();
    ~FSC();

    NODE*& getRoot(){return Root;}

    static int Horizon;
    static long numNode;

    //void setNumNode(long num) {numNode = num;}
    //long getNumNode(){return numNode;}

private:

    NODE* Root;
};

struct Observation{
    arr x;
    arr y;
    double height;//model sample
    double pos   ;//pos in x-axis
    double dual;
};




class NODE {

public:

    NODE*& ChildIndex(int index)       { return Childrens[index]; }
    NODE*  ChildIndex(int index) const { return Childrens[index]; }

    NODE*& Child(Observation obs)       { return Childrens[findIndex(obs)]; }
    NODE*  Child(Observation obs) const { return Childrens[findIndex(obs)]; }

    int findIndex(Observation obs);
    int findIndex(Observation obs) const;

    void Add(Observation obs);
    //int getState(){ return pState; }
    //void setState(int s) {pState = s;}


    void Initialise();
    NODE();
    ~NODE();

    //this can be considered as action node (choosing a next state)
    arr& X(){ return x;}
    arr& Y(){ return y;}
    double& Dual(){ return dual;}
    arr& Model(){ return model;}



    //this is particles of this node. (we combine FSC and search tree, because the tree is the optimal reachable tree)
    arr& AllX(){ return allX;}
    arr& AllY(){ return allY;}
    arr& AllDual(){ return allDual;}
    arr& Samples(){ return samples;}


    arr& getSampleDual(){ return sampleDual;}
    arr& getSampleX(){ return sampleX;}
    arr& getSampleY(){ return sampleY;}

    int& getNumChildren(){return NumChildren;}

    void setIndex(long num){index = num;}
    long getIndex(){return index;}

    // each is one pair of an observation and a next node (in ordering)
    std::vector<NODE*> Childrens;
    std::vector<Observation> Obss;
    bool certainty;
    bool sticky;// disable sticky, if certainty at least two consecutive steps.


private:
    int NumChildren;

    //information of the model in this node
    arr x; //immediate joint state of this node.
    arr y;
    arr model; //the model that is being  followed (robot thinks that is the best)

    double dual;

    arr allX; //full joint trajectory of the best following model
    arr allY;
    arr allDual;

    // all x,y,dual trajectories of sampled models (correspond to all model particles)
    arr sampleDual;
    arr sampleX;
    arr sampleY;

    arr samples; //contains all model particles

    long index;

};



void setInitialPose(const arr y, ors::KinematicWorld& world);


void OptimizeFSC(ors::KinematicWorld& world, NODE *&node, int horizon);
void OptimizeFSC_Edges(ors::KinematicWorld& world, NODE *&node, int horizon);
void OptimizeFSC_Test(ors::KinematicWorld& world, NODE *&node, int horizon);

NODE* addNode_MultiParticle(NODE*& node,Observation obs, int steps);
NODE* addNode_SingleParticle(NODE*& node,Observation obs, int steps, double height, int  horizon);
int choose_Next_Obs(arr Heights);

void write_to_graphviz(FSC fsc);
void write(FSC fsc);
//void add_node_to_graph(NODE*& root);
//void add_node_to_graph(NODE*& root, Graph& graph);







#endif // POMDP_H
