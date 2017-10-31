#include <Core/thread.h>
#include <Kin/kin.h>
#include <Geo/geoms.h>

//===============================================================================

struct PerceptSimple;
typedef mlr::Array<PerceptSimple*> PerceptSimpleL;

struct PerceptSimple : GLDrawer{
  enum Type { PT_cluster, PT_plane, PT_box, PT_mesh, PT_alvar, PT_optitrackmarker, PT_optitrackbody, PT_end };

  mlr::GeomStore& store;
  int geomID = -1;
  int frameID = -1;
  mlr::Transformation pose;
  double precision = 1.; //1 is maximum; decays as a precision in kalman filtering
  PerceptSimple() : store(_GeomStore()), pose(0) {}
  PerceptSimple(const mlr::Geom& geom, const mlr::Transformation& pose)
    : store(_GeomStore()), geomID(geom.ID), pose(pose) {
  }
  virtual ~PerceptSimple(){}

  virtual double fuse(PerceptSimple* other);
  virtual void write(ostream& os) const;
  virtual void glDraw(OpenGL& gl);
  virtual PerceptSimple* newClone() const{ return new PerceptSimple(*this); }

};
stdOutPipe(PerceptSimple)

//===============================================================================

struct FilterSimple : Thread{
  mlr::KinematicWorld K;
  FrameL objects;
  PerceptSimpleL percepts_display;
  double time=0.;

  Access<PerceptSimpleL> percepts_input;
  Access<PerceptSimpleL> percepts_filtered;
  Access<arr> currentQ;
  Access<StringA> switches;
  Access<double> timeToGo;
  Access<mlr::Transformation> robotBase;
  Access<mlr::KinematicWorld> filterWorld;

  FilterSimple(double dt=.01);
  ~FilterSimple(){
  }

  void open(){}
  void close(){}

  void step();
};

