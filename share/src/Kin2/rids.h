#include <vector>
#include <string>

#include <bits/shared_ptr.h>
using std::shared_ptr;
using std::make_shared;

typedef std::vector<double> vec;
typedef std::vector<int>    vInt;
typedef std::vector<bool>   vBool;
typedef unsigned char       byte;

struct Transform;

template<class T> typedef std::vector<std::shared_ptr<T>> list;

struct arr  : std::vector<double>{ std::vector<int> dim; };
struct aInt : std::vector<int>{ std::vector<int> dim; };

//ALL container data structures are revision R/W locked

struct Signaler{
};

struct VariableBase : Signaler {
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
