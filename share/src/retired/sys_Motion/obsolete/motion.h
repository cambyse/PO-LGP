//===========================================================================
//
// Viewer (will be redundant with a more generic GUI)
//

template<class T>
struct PoseViewer:Process {
  T *var;
  WorkingCopy<GeometricState> geo;
  OpenGL *gl;
  
  PoseViewer(T& v):Process("PoseViewer"), var(&v), gl(NULL) {
    geo.init("GeometricState", this);
    listenTo(var);
  }
  void open() {
    geo.pull();
//     geo->writeAccess(this);
//     ors = geo->ors.newClone();
//     geo->deAccess(this);
    gl = new OpenGL(var->name);
    gl->add(glStandardScene);
    gl->add(ors::glDrawGraph, &geo().ors);
    gl->camera.setPosition(5, -10, 10);
    gl->camera.focus(0, 0, 1);
    gl->camera.upright();
    gl->update();
  }
  void close() {
    delete gl;
    gl = NULL;
  }
  void step() {
    geo.pull();
    uint n=geo().ors.getJointStateDimension();
    arr q;
    var->readAccess(this);
    var->get_poseView(q);
    var->deAccess(this);
    if (q.nd==1) {
      if (q.N==2*n) q = q.sub(0,q.N/2-1); //check dynamic state
      if (q.N!=n){ MT_MSG("pose view on wrong dimension");  return; }
      geo().ors.setJointState(q);
      gl->text.clear() <<"pose view";
      gl->update();
    } else {
      for (uint t=0; t<q.d0; t++) {
        geo().ors.setJointState(q[t]);
        gl->text.clear() <<"pose view at step " <<t <<"/" <<q.d0-1;
        gl->update();
      }
    }
  }
};

template<class T>
struct OrsViewer:Process {
  T *var;
  WorkingCopy<GeometricState> geo;
  OpenGL *gl;
  
  OrsViewer(T& v):Process("OrsViewer"), var(&v), gl(NULL) {
    geo.init("GeometricState", this);
    listenTo(var);
  }
  void open() {
    geo.pull();
//     geo->writeAccess(this);
//     ors = geo->ors.newClone();
//     geo->deAccess(this);
    gl = new OpenGL(var->name);
    gl->add(glStandardScene);
    gl->add(ors::glDrawGraph, &geo().ors);
    gl->camera.setPosition(5, -10, 10);
    gl->camera.focus(0, 0, 1);
    gl->camera.upright();
    gl->update();
  }
  void close() {
    delete gl;
    gl = NULL;
  }
  void step() {
    arr q;
    geo.pull();
    gl->text.clear() <<"ors view of Variable " <<var->name;
    gl->update();
  }
};
