#include <climits>
#include "frame.h"
#include "kin.h"
#include "uncertainty.h"

#ifdef MLR_GL
#include <Gui/opengl.h>
#endif

//===========================================================================
//
// Frame
//

mlr::Frame::Frame(KinematicWorld& _K, const Frame* copyFrame)
  : K(_K){

  ID=K.frames.N;
  K.frames.append(this);
  if(copyFrame){
    const Frame& f = *copyFrame;
    Q = copyFrame->Q;
    name=f.name; X=f.X; ats=f.ats; active=f.active;
    //we cannot copy link! because we can't know if the frames already exist. KinematicWorld::copy copies the rel's !!
    if(copyFrame->joint) new Joint(*this, copyFrame->joint);
    if(copyFrame->shape) new Shape(*this, copyFrame->shape);
    if(copyFrame->inertia) new Inertia(*this, copyFrame->inertia);
  }
}

mlr::Frame::Frame(Frame *_parent)
  : Frame(_parent->K){
  CHECK(_parent, "");
  linkFrom(_parent);
}

mlr::Frame::~Frame() {
  if(joint) delete joint;
  if(shape) delete shape;
  if(inertia) delete inertia;
  if(parent) unLink();
  while(outLinks.N) outLinks.last()->unLink();
  K.frames.removeValue(this);
  listReindex(K.frames);
}

void mlr::Frame::read(const Graph& ats) {
  //interpret some of the attributes
  ats.get(X, "X");
  ats.get(X, "pose");
  ats.get(Q, "Q");

  if(ats["type"]) ats["type"]->keys.last() = "shape"; //compatibility with old convention: 'body { type... }' generates shape

  if(ats["joint"]){ joint = new Joint(*this); joint->read(ats); }
  if(ats["shape"]){ shape = new Shape(*this); shape->read(ats); }
  if(ats["mass"]){ inertia = new Inertia(*this); inertia->read(ats); }
}

void mlr::Frame::write(std::ostream& os) const {
  if(parent){
    if(!Q.isZero()) os <<" Q=<T " <<Q <<" > ";
  }else{
    if(!X.isZero()) os <<" X=<T " <<X <<" > ";
  }
  if(joint) joint->write(os);
  if(shape) shape->write(os);
  if(inertia) inertia->write(os);

//  if(mass) os <<"mass=" <<mass <<' ';
//  if(type!=BT_dynamic) os <<"dyntype=" <<(int)type <<' ';
//  uint i; Node *a;
//  for(Type *  a:  ats)
//      if(a->keys(0)!="X" && a->keys(0)!="pose") os <<*a <<' ';
}

mlr::Frame* mlr::Frame::insertPreLink(const mlr::Transformation &A){
  //new frame between: parent -> f -> this
  Frame *f = new Frame(K);
  if(name) f->name <<'>' <<name;

  if(parent){
    f->linkFrom(parent);
    parent->outLinks.removeValue(this);
  }
  parent=f;
  parent->outLinks.append(this);

  f->Q = A;

  return f;
}

mlr::Frame* mlr::Frame::insertPostLink(const mlr::Transformation &B){
  NIY;
  //new frame between: parent -> this -> f
  Frame *f = new Frame(K);
  if(name) f->name <<'<' <<name;

  //reconnect all outlinks from -> to
  f->outLinks = outLinks;
  for(Frame *b:outLinks) b->parent = f;
  outLinks.clear();
  f->Q = B;
  f->linkFrom(this);

  return f;
}

void mlr::Frame::unLink(){
  CHECK(parent,"");
  parent->outLinks.removeValue(this);
  parent=NULL;
  Q.setZero();
  if(joint){ delete joint; joint=NULL; }
}

void mlr::Frame::linkFrom(mlr::Frame *_parent){
  CHECK(!parent,"");
  parent=_parent;
  parent->outLinks.append(this);
}

mlr::Joint::Joint(Frame &f, Joint *copyJoint)
  : frame(f), dim(0), qIndex(UINT_MAX), q0(0.), H(1.), mimic(NULL){
  CHECK(!frame.joint, "the Link already has a Joint");
  frame.joint = this;
  frame.K.reset_q();

  if(copyJoint){
    qIndex=copyJoint->qIndex; dim=copyJoint->dim; mimic=reinterpret_cast<Joint*>(copyJoint->mimic?1l:0l); constrainToZeroVel=copyJoint->constrainToZeroVel;
    type=copyJoint->type; axis=copyJoint->axis; limits=copyJoint->limits; q0=copyJoint->q0; H=copyJoint->H;
    active=copyJoint->active;

    if(copyJoint->mimic){
      mimic = frame.K.frames(copyJoint->mimic->frame.ID)->joint;
    }

    if(copyJoint->uncertainty){
      new Uncertainty(this, copyJoint->uncertainty);
    }
  }
}

mlr::Joint::Joint(Frame &from, Frame &f, Joint *copyJoint)
  : Joint(f, copyJoint){
  frame.linkFrom(&from);
}

mlr::Joint::~Joint() {
  frame.K.reset_q();
  frame.joint = NULL;
  //if(frame.parent) frame.unLink();
}

void mlr::Joint::calc_Q_from_q(const arr &q, uint _qIndex){
  mlr::Transformation &Q = frame.Q;
    if(mimic){
        Q = mimic->frame.Q;
    }else{
        switch(type) {
        case JT_hingeX: {
            Q.rot.setRadX(q.elem(_qIndex));
        } break;

        case JT_hingeY: {
            Q.rot.setRadY(q.elem(_qIndex));
        } break;

        case JT_hingeZ: {
            Q.rot.setRadZ(q.elem(_qIndex));
        } break;

        case JT_universal:{
            mlr::Quaternion rot1, rot2;
            rot1.setRadX(q.elem(_qIndex));
            rot2.setRadY(q.elem(_qIndex+1));
            Q.rot = rot1*rot2;
        } break;

        case JT_quatBall:{
            Q.rot.set(q.p+_qIndex);
            {
                double n=Q.rot.normalization();
                if(n<.1 || n>10.) LOG(-1) <<"quat normalization is extreme: " <<n <<endl;
            }
            Q.rot.normalize();
            Q.rot.isZero=false; //WHY? (gradient check fails without!)
        } break;

        case JT_free:{
            Q.pos.set(q.p+_qIndex);
            Q.rot.set(q.p+_qIndex+3);
            {
                double n=Q.rot.normalization();
                if(n<.1 || n>10.) LOG(-1) <<"quat normalization is extreme: " <<n <<endl;
            }
            Q.rot.normalize();
            Q.rot.isZero=false;
        } break;

        case JT_XBall:{
            Q.pos.x = q.elem(_qIndex);
            Q.pos.y = 0.;
            Q.pos.z = 0.;
            Q.pos.isZero = false;
            Q.rot.set(q.p+_qIndex+1);
            {
                double n=Q.rot.normalization();
                if(n<.1 || n>10.) LOG(-1) <<"quat normalization is extreme: " <<n <<endl;
            }
            Q.rot.normalize();
            Q.rot.isZero=false;
        } break;

        case JT_transX: {
            Q.pos = q.elem(_qIndex)*Vector_x;
        } break;

        case JT_transY: {
            Q.pos = q.elem(_qIndex)*Vector_y;
        } break;

        case JT_transZ: {
            Q.pos = q.elem(_qIndex)*Vector_z;
        } break;

        case JT_transXY: {
            Q.pos.set(q.elem(_qIndex), q.elem(_qIndex+1), 0.);
        } break;

        case JT_trans3: {
            Q.pos.set(q.elem(_qIndex), q.elem(_qIndex+1), q.elem(_qIndex+2));
        } break;

        case JT_transXYPhi: {
            Q.pos.set(q.elem(_qIndex), q.elem(_qIndex+1), 0.);
            Q.rot.setRadZ(q.elem(_qIndex+2));
        } break;

        case JT_phiTransXY: {
            Q.rot.setRadZ(q.elem(_qIndex));
            Q.pos = Q.rot*Vector(q.elem(_qIndex+1), q.elem(_qIndex+2), 0.);
        } break;

        case JT_rigid:
            break;
        default: NIY;
        }
    }
    CHECK_EQ(Q.pos.x, Q.pos.x, "NAN transform");
    CHECK_EQ(Q.rot.w, Q.rot.w, "NAN transform");

//    link->link = A * Q * B; //total rel transformation
}

arr mlr::Joint::calc_q_from_Q(const mlr::Transformation &Q) const{
    arr q;
    switch(type) {
    case JT_hingeX:
    case JT_hingeY:
    case JT_hingeZ: {
        q.resize(1);
        //angle
        mlr::Vector rotv;
        Q.rot.getRad(q(0), rotv);
        if(q(0)>MLR_PI) q(0)-=MLR_2PI;
        if(type==JT_hingeX && rotv*Vector_x<0.) q(0)=-q(0);
        if(type==JT_hingeY && rotv*Vector_y<0.) q(0)=-q(0);
        if(type==JT_hingeZ && rotv*Vector_z<0.) q(0)=-q(0);
    } break;

    case JT_universal: {
        q.resize(2);
        //angle
        if(fabs(Q.rot.w)>1e-15) {
            q(0) = 2.0 * atan(Q.rot.x/Q.rot.w);
            q(1) = 2.0 * atan(Q.rot.y/Q.rot.w);
        } else {
            q(0) = MLR_PI;
            q(1) = MLR_PI;
        }
    } break;

    case JT_quatBall: {
        q.resize(4);
        q(0)=Q.rot.w;
        q(1)=Q.rot.x;
        q(2)=Q.rot.y;
        q(3)=Q.rot.z;
    } break;

    case JT_transX: {
        q.resize(1);
        q(0)=Q.pos.x;
    } break;
    case JT_transY: {
        q.resize(1);
        q(0)=Q.pos.y;
    } break;
    case JT_transZ: {
        q.resize(1);
        q(0)=Q.pos.z;
    } break;
    case JT_transXY: {
        q.resize(2);
        q(0)=Q.pos.x;
        q(1)=Q.pos.y;
    } break;
    case JT_transXYPhi: {
        q.resize(3);
        q(0)=Q.pos.x;
        q(1)=Q.pos.y;
        mlr::Vector rotv;
        Q.rot.getRad(q(2), rotv);
        if(q(2)>MLR_PI) q(2)-=MLR_2PI;
        if(rotv*Vector_z<0.) q(2)=-q(2);
    } break;
    case JT_phiTransXY: {
        q.resize(3);
        mlr::Vector rotv;
        Q.rot.getRad(q(0), rotv);
        if(q(0)>MLR_PI) q(0)-=MLR_2PI;
        if(rotv*Vector_z<0.) q(0)=-q(0);
        mlr::Vector relpos = Q.rot/Q.pos;
        q(1)=relpos.x;
        q(2)=relpos.y;
    } break;
    case JT_trans3: {
        q.resize(3);
        q(0)=Q.pos.x;
        q(1)=Q.pos.y;
        q(2)=Q.pos.z;
    } break;
    case JT_rigid:
        break;
    case JT_free:
        q.resize(7);
        q(0)=Q.pos.x;
        q(1)=Q.pos.y;
        q(2)=Q.pos.z;
        q(3)=Q.rot.w;
        q(4)=Q.rot.x;
        q(5)=Q.rot.y;
        q(6)=Q.rot.z;
        break;
    case JT_XBall:
        q.resize(5);
        q(0)=Q.pos.x;
        q(1)=Q.rot.w;
        q(2)=Q.rot.x;
        q(3)=Q.rot.y;
        q(4)=Q.rot.z;
        break;
    default: NIY;
    }
    return q;
}

uint mlr::Joint::getDimFromType() const {
  if(mimic) return 0;
  if(type>=JT_hingeX && type<=JT_transZ) return 1;
  if(type==JT_transXY) return 2;
  if(type==JT_transXYPhi) return 3;
  if(type==JT_phiTransXY) return 3;
  if(type==JT_trans3) return 3;
  if(type==JT_universal) return 2;
  if(type==JT_quatBall) return 4;
  if(type==JT_free) return 7;
  if(type==JT_rigid || type==JT_none) return 0;
  if(type==JT_XBall) return 5;
  HALT("shouldn't be here");
  return 0;
}

void mlr::Joint::makeRigid(){
  type=JT_rigid; frame.K.reset_q();
}

void mlr::Joint::read(const Graph &G){
  double d=0.;
  mlr::String str;

  mlr::Transformation A=0, B=0;

  G.get(A, "A");
  G.get(A, "from");
  if(G["BinvA"]) B.setInverse(A);
  G.get(B, "B");
  G.get(B, "to");

  //axis
  arr axis;
  if(G.get(axis, "axis")) {
    CHECK_EQ(axis.N, 3, "");
    Vector ax(axis);
    Transformation f;
    f.setZero();
    f.rot.setDiff(Vector_x, ax);
    A = A * f;
    B = -f * B;
  }

  if(!B.isZero()){
    //new frame between: from -> f -> to
    CHECK(frame.outLinks.N==1,"");
    Frame *follow = frame.outLinks.scalar();

    CHECK(follow->parent, "");
    CHECK(!follow->joint, "");
    follow->Q = B;
    B.setZero();
  }

  if(!A.isZero()){
    frame.insertPreLink(A);
    A.setZero();
  }

  G.get(frame.Q, "Q");
  G.get(H, "ctrl_H");
  if(G.get(d, "joint"))        type=(JointType)d;
  else if(G.get(str, "joint")) { str >>type; }
  else if(G.get(d, "type"))    type=(JointType)d;
  else if(G.get(str, "type"))  { str >>type; }
  else type=JT_hingeX;

  dim = getDimFromType();

  if(G.get(d, "q")){
    if(!dim){ //HACK convention
      frame.Q.rot.setRad(d, 1., 0., 0.);
    }else{
      CHECK(dim, "setting q (in config file) for 0-dim joint");
      q0 = consts<double>(d, dim);
      calc_Q_from_q(q0, 0);
    }
  }else if(G.get(q0, "q")){
    CHECK_EQ(q0.N, dim, "given q (in config file) does not match dim");
    calc_Q_from_q(q0, 0);
  }else{
//    link->Q.setZero();
    q0 = calc_q_from_Q(frame.Q);
  }

  //limit
  arr ctrl_limits;
  G.get(limits, "limits");
  if(limits.N && type!=JT_rigid && !mimic){
    CHECK(limits.N==2*qDim() || limits.N==2*qDim()+3, "parsed limits have wrong dimension: either lo-hi or lo-hi-vel-eff-acc");
  }
  G.get(ctrl_limits, "ctrl_limits");
  if(ctrl_limits.N && type!=JT_rigid){
    if(!limits.N) limits.resizeAs(ctrl_limits).setZero();
    CHECK_EQ(3,ctrl_limits.N, "parsed ctrl_limits have wrong dimension");
    limits.append(ctrl_limits);
  }

  //coupled to another joint requires post-processing by the Graph::read!!
  if(G["mimic"]){
    mimic=(Joint*)1;
    dim=0;
  }
}

void mlr::Joint::write(std::ostream& os) const {
  os <<" joint=" <<type;
  if(H) os <<" ctrl_H="<<H;
  if(limits.N) os <<" limits=[" <<limits <<"]";
  if(mimic){
    os <<" mimic=" <<mimic->frame.name;
  }

  Node *n;
  if((n=frame.ats["Q"])) os <<*n <<' ';
  if((n=frame.ats["q"])) os <<*n <<' ';
}

//===========================================================================
//
// Shape
//

mlr::Shape::Shape(Frame &f, const Shape *copyShape)
  : frame(f), store(_GeomStore())/*, type(ST_none)*/ {
//  size = {1.,1.,1.,.1};
//  mesh.C = consts<double>(.8, 3); //color[0]=color[1]=color[2]=.8; color[3]=1.;

  CHECK(!frame.shape, "this frame already has a geom attached");
  frame.shape = this;
  if(copyShape){
    const Shape& s = *copyShape;
    CHECK(&store==&s.store,"copying shapes that refer to different geom stores is not possible");
    mesh_radius=s.mesh_radius;
    cont=s.cont;
    geomID = s.geomID;

#if 0
    type=s.type;
    size=s.size;
    if(!referenceMeshOnCopy){
      mesh=s.mesh;
      sscCore=s.sscCore;
    }else{
      mesh.V.referTo(s.mesh.V);
      mesh.T.referTo(s.mesh.T);
      mesh.C.referTo(s.mesh.C);
      mesh.Vn.referTo(s.mesh.Vn);
      sscCore.V.referTo(s.sscCore.V);
      sscCore.T.referTo(s.sscCore.T);
      sscCore.C.referTo(s.sscCore.C);
      sscCore.Vn.referTo(s.sscCore.Vn);
    }
#endif
  }
}

mlr::Shape::~Shape() {
  frame.shape = NULL;
}

void mlr::Shape::read(const Graph& ats) {

  geom().read(ats);

  if(ats["contact"])           { cont=true; }

  //center the mesh:
  if(type()==mlr::ST_mesh && mesh().V.N){
    if(ats["rel_includes_mesh_center"]){
      mesh().center();
    }
//    if(c.length()>1e-8 && !ats["rel_includes_mesh_center"]){
//      frame.link->Q.addRelativeTranslation(c);
//      frame.ats.newNode<bool>({"rel_includes_mesh_center"}, {}, true);
//    }
  }

  //compute the bounding radius
  if(mesh().V.N) mesh_radius = mesh().getRadius();

  //add inertia to the body
#if 0
  if(frame) {
    Matrix I;
    double mass=-1.;
    switch(type) {
      case ST_sphere:   inertiaSphere(I.p(), mass, 1000., size(3));  break;
      case ST_box:      inertiaBox(I.p(), mass, 1000., size(0), size(1), size(2));  break;
      case ST_capsule:
      case ST_cylinder: inertiaCylinder(I.p(), mass, 1000., size(2), size(3));  break;
      case ST_none:
      default: ;
    }
    if(mass>0.){
      if(!body->inertia) body->iner
      NIY;
//      frame.mass += mass;
//      frame.inertia += I;
    }
  }
#endif
}

void mlr::Shape::write(std::ostream& os) const {
  os <<" shape=" <<geom().type;
  os <<" size=[" <<geom().size <<"]";

  Node *n;
  if((n=frame.ats["color"])) os <<*n <<' ';
  if((n=frame.ats["mesh"])) os <<*n <<' ';
  if((n=frame.ats["meshscale"])) os <<*n <<' ';
  if(cont) os <<"contact, ";
}

#ifdef MLR_GL
void mlr::Shape::glDraw(OpenGL& gl) {
  //set name (for OpenGL selection)
  glPushName((frame.ID <<2) | 1);
  if(frame.K.orsDrawColors && !frame.K.orsDrawIndexColors){
    if(mesh().C.N) glColor(mesh().C); //color[0], color[1], color[2], color[3]*world.orsDrawAlpha);
    else   glColor(.5, .5, .5);
  }
  if(frame.K.orsDrawIndexColors) glColor3b((frame.ID>>16)&0xff, (frame.ID>>8)&0xff, frame.ID&0xff);


  double GLmatrix[16];
  frame.X.getAffineMatrixGL(GLmatrix);
  glLoadMatrixd(GLmatrix);

  if(!frame.K.orsDrawShapes) {
    double scale=.33*(size(0)+size(1)+size(2) + 2.*size(3)); //some scale
    if(!scale) scale=1.;
    scale*=.3;
    glDrawAxes(scale);
    glColor(0, 0, .5);
    glDrawSphere(.1*scale);
  }
  if(frame.K.orsDrawShapes) {
    geom().glDraw(gl);
  }
  if(frame.K.orsDrawZlines) {
    glColor(0, .7, 0);
    glBegin(GL_LINES);
    glVertex3d(0., 0., 0.);
    glVertex3d(0., 0., -frame.X.pos.z);
    glEnd();
  }

  if(frame.K.orsDrawBodyNames){
    glColor(1,1,1);
    glDrawText(frame.name, 0, 0, 0);
  }

  glPopName();
}

#endif


mlr::Inertia::Inertia(Frame &f, Inertia *copyInertia) : frame(f), type(BT_dynamic) {
  CHECK(!frame.inertia, "this frame already has inertia");
  frame.inertia = this;
  if(copyInertia){
    mass = copyInertia->mass;
    matrix = copyInertia->matrix;
    type = copyInertia->type;
    com = copyInertia->com;
    force = copyInertia->force;
    torque = copyInertia->torque;
  }
}

mlr::Inertia::~Inertia(){
  frame.inertia = NULL;
}

void mlr::Inertia::write(std::ostream &os) const{
  os <<" mass=" <<mass;
}

void mlr::Inertia::read(const Graph& G){
  double d;
  if(G.get(d, "mass")) {
    mass=d;
    matrix.setId();
    matrix *= .2*d;
    type=BT_dynamic;
  }
  if(G["fixed"])       type=BT_static;
  if(G["static"])      type=BT_static;
  if(G["kinematic"])   type=BT_kinematic;
  if(G.get(d,"dyntype")) type=(BodyType)d;
}


RUN_ON_INIT_BEGIN(frame)
JointL::memMove=true;
RUN_ON_INIT_END(frame)
