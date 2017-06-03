#include <vector>
#include <string>

#include <bits/shared_ptr.h>
using std::shared_ptr;
using std::make_shared;

typedef std::vector<double> vec;
typedef std::vector<int>    vInt;
typedef std::vector<bool>   vBool;
typedef unsigned char       byte;

template<class T> typedef std::vector<std::shared_ptr<T>> list;

struct arr  : std::vector<double>{ std::vector<int> dim; };
struct aInt : std::vector<int>{ std::vector<int> dim; };

//ALL container data structures are revision R/W locked

struct Signaler{
};

struct VariableBase : Signaler {
};

struct Geom{
    GeomStore& G; //every geom is uniquely owned (listed) by a geom store
    int ID; //its ID equals its indes in the list

    arr vertices;  //(nx3)
    aInt triangles; //(Tx3): triangles; (0): point cloud; (Tx2): lines
    arr normals; //(nx3)
    arr color;     //(nx3|4): color per vertex, (3|4): color of whole geom (to allow colors per triangle you need to triple the vertices)
    //texture
    double radius; //if positive, this is a sphere-swept convex geometry, where 'verticies' describe the convex core; the true geom is larger

    Geom createColorsPerTriangle(const arr& colors); //returns a new geom with colors per triangle [asserts colors.dim(0)==triangles.dim(0)]
    Geom createSphereSweptVisual(int sphereApproxLevel=3); //asserts radius>0; returns a geom with radius==0 that is a visual of this geom
};

/** A store for geometries (maybe only a singleton, or distinguish dynamic/perceptual vs static/ad hoc known?) */
struct GeomStore : VariableBase{
    list<Geom> geoms;

    void checkConsistency(); //asserts geom IDs are one-to-one with index in this list
};

struct Frame{
    KinematicConfiguration& K; //every frame is uniquely owned by (listed in) a kinematic configuration
    int ID; //its ID equals its indes in the list

    std::string name;

    Link *parent;
    std::vector<Link*> children;

    arr trans;
    arr pose;
};

struct FrameJoint{
    KinematicConfiguration& K;
    int ID;

    Frame& F; //a joint also is uniquely associated with a frame: it 'articulates' F.trans

    arr rigidTrans;

    // joint information
    int jointDim;
    byte jointGenerator; // (7bits), h in Featherstone's code (indicates basis vectors of the Lie algebra, but including the middle quaternion w)
    arr limits;        ///< joint limits (lo, up, [maxvel, maxeffort])
    arr q0;            ///< joint null position
    double H;          ///< control cost scalar

    FrameJoint *mimic;       ///< if non-NULL, this joint's state is identical to another's
};

struct FrameMass{
    DynamicConfiguration& D;
    int ID;

    Frame& F; //a mass also is uniquely associated with a frame

    //dynamic properties
    arr centerOfMass;
    double mass;
    arr inertiaTensor;
};

struct GeomConfiguration{
    const GeomStore& store;
    vInt IDs;
    arr poses;
};

struct KinematicConfiguration{
    GeomConfiguration visuals;
    GeomConfiguration colls;
    list<Frame> frames;
    list<FrameJoint> joints; //needs to be topologically sorted

    aInt linkVisualsToFrames; //pairs of (visualID, frameID)
    aInt linkCollsToFrames;

    //proxies

    //loop constraints

    void checkConsistency(); //checks for consistency of the data structure (parent-children, topological sorting)


};

struct DynamicConfiguration {
    KinematicConfiguration K;
    list<Mass> masses;

    enum LinkType { BT_none=-1, BT_dynamic=0, BT_kinematic, BT_static };

    std::vector<LinkType> linkTypes;

    void checkConsistency(); //asserts masses.size()==K.links.size()
};

/* FUNCTIONAL TARGETS: We design the data structures specifically to be minimalistic inputs to components */

void render(const Geoms&, const arr& poses);

void forwardKinematics(arr& out_poses, const Links&, const arr& q);

struct KinematicEngine{
    KinematicConfiguration& K;

    KinematicEngine(KinematicConfiguration& K) : K(K) {}

    //computes the poses of all links
    virtual void forwardKinematics(const arr& q);
    virtual void forwardKinematics(const arr& q, const vInt& jointSelection);


};
