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





extern double stickyWeight;

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

class NODE {

public:

    NODE*& ChildIndex(int index)       { return Childrens[index]; }
    NODE*  ChildIndex(int index) const { return Childrens[index]; }

    NODE*& Child(int obs)       { return Childrens[findIndex(obs)]; }
    NODE*  Child(int obs) const { return Childrens[findIndex(obs)]; }

    int findIndex(int obs);
    int findIndex(int obs) const;

    void Add(int obs);
    //int getState(){ return pState; }
    //void setState(int s) {pState = s;}


    void Initialise();
    NODE();
    ~NODE();

    //this can be considered as action node (choosing a next state)
    arr& X(){ return x;}
    arr& Y(){ return y;}
    double& Dual(){ return dual;}


    //this is particles of this node. (we combine FSC and search tree, because the tree is the optimal reachable tree)
    arr& AllX(){ return allX;}
    arr& AllY(){ return allY;}
    arr& AllDual(){ return allDual;}
    arr& Heights(){ return heights;}

    arr& getSampleDual(){ return sampleDual;}

    int& getNumChildren(){return NumChildren;}

    void setIndex(long num){index = num;}
    long getIndex(){return index;}

    // each is one pair of an observation and a next node (in ordering)
    std::vector<NODE*> Childrens;
    std::vector<int>   Obss;


private:
    int NumChildren;
    arr x; //immediate joint state of this node.
    arr y;
    double dual;

    arr allX; //full joint trajectory of
    arr allY;
    arr allDual;

    arr sampleDual;

    arr heights;



    long index;

};





void setInitialPose(const arr y, ors::KinematicWorld& world);
void getTrajectory(arr& x, arr& y, arr& dual, ors::KinematicWorld& world, arr y0, const double& height, bool stickyness, uint horizon);
void POMDPExecution(const arr& allx, const arr& ally, const arr& alldual, ors::KinematicWorld& world, int num);

void OptimizeFSC(ors::KinematicWorld& world, NODE *&node, int horizon);

void write_to_graphviz(FSC fsc);
//void add_node_to_graph(NODE*& root);
//void add_node_to_graph(NODE*& root, Graph& graph);







#endif // POMDP_H
