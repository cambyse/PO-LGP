/*  Copyright 2009 Marc Toussaint
    email: mtoussai@cs.tu-berlin.de

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a COPYING file of the GNU General Public License
    along with this program. If not, see <http://www.gnu.org/licenses/> */

#include "ors.h"
#include "ann.h"

/* MT TODO: are prenetrations optimally handled? proper penetration depth+gradient? Would be useful also for grasp variable */


#ifdef MT_SWIFT

#ifdef MT_SINGLE
#  define SWIFT_USE_FLOAT
#endif

#include <SWIFT/SWIFT.h>
#undef min
#undef max

// internal use
void exportStateToSwift(const ors::Graph& C, SwiftInterface& swift);
void importProxiesFromSwift(ors::Graph& C, SwiftInterface& swift, bool dumpReport=false);
void swiftQueryExactDistance(SwiftInterface& swift);
ANN *global_ANN=NULL;
ors::Shape *global_ANN_shape;

SwiftInterface::~SwiftInterface() {
  close();
}

SwiftInterface* SwiftInterface::newClone(const ors::Graph& G) const {
  SwiftInterface* s=new SwiftInterface;
  s->init(G, cutoff);
  return s;
}

void SwiftInterface::close() {
  if(scene) delete scene;
  if(global_ANN) delete global_ANN;
  scene=NULL;
  cout <<" -- SwiftInterface closed" <<endl;
  isOpen=false;
}

void SwiftInterface::init(const ors::Graph& C, double _cutoff) {
  ors::Shape *s;
  uint k;
  bool r, add;
  
  cutoff=_cutoff;
  if(scene) delete scene;
  scene=new SWIFT_Scene(true, false);
  
  INDEXswift2shape.resize(C.shapes.N);  INDEXswift2shape=-1;
  INDEXshape2swift.resize(C.shapes.N);  INDEXshape2swift=-1;
  
  cout <<" -- SwiftInterface init";
  for_list(k, s, C.shapes) {
    cout <<'.' <<flush;
    add=true;
    switch(s->type) {
      case ors::noneST: HALT("shapes should have a type - somehow wrong initialization..."); break;
      case ors::boxST:
        s->mesh.setBox();
        s->mesh.scale(s->size[0], s->size[1], s->size[2]);
        break;
      case ors::sphereST:
        s->mesh.setSphere();
        s->mesh.scale(s->size[3], s->size[3], s->size[3]);
        break;
      case ors::cylinderST:
        s->mesh.setCylinder(s->size[3], s->size[2]);
        break;
      case ors::cappedCylinderST:
        s->mesh.setCappedCylinder(s->size[3], s->size[2]);
        break;
      case ors::meshST:
        //check if there is a specific swiftfile!
        MT::String *filename;
        filename=anyListGet<MT::String>(s->ats, "swiftfile", 1);
        if(!filename)
          filename=anyListGet<MT::String>(s->body->ats, "swiftfile", 1);
        if(filename) {
          r=scene->Add_General_Object(*filename, INDEXshape2swift(s->index), false);
          if(!r) HALT("--failed!");
        }
        break;
      case ors::pointCloudST:
        //for now, assume there is only ONE pointCloudObject!
        CHECK(s->mesh.V.N, "");
        global_ANN=new ANN;
        global_ANN_shape=s;
        global_ANN->setX(s->mesh.V);
        global_ANN->calculate();
        add=false;
        break;
      case ors::markerST:
        add=false; // ignore (no collisions)
        break;
      default:
        NIY;
    }
    if(add) {
      r=scene->Add_Convex_Object(
          s->mesh.V.p, (int*)s->mesh.T.p,
          s->mesh.V.d0, s->mesh.T.d0, INDEXshape2swift(s->index), false,
          DEFAULT_ORIENTATION, DEFAULT_TRANSLATION, DEFAULT_SCALE,
          DEFAULT_BOX_SETTING, DEFAULT_BOX_ENLARGE_REL, cutoff);
      if(!r) HALT("--failed!");
      
      INDEXswift2shape(INDEXshape2swift(s->index)) = s->index;
    }
  }
  
  initActivations(C);
  
  exportStateToSwift(C, *this);
  cout <<"...done" <<endl;
  isOpen=true;
}

void SwiftInterface::reinitShape(const ors::Graph& ors, const ors::Shape *s) {
  int sw = INDEXshape2swift(s->index);
  scene->Delete_Object(sw);
  INDEXswift2shape(sw) = -1;
  
  bool r=scene->Add_Convex_Object(s->mesh.V.p, (int*)s->mesh.T.p,
                                  s->mesh.V.d0, s->mesh.T.d0, sw, false,
                                  DEFAULT_ORIENTATION, DEFAULT_TRANSLATION, DEFAULT_SCALE,
                                  DEFAULT_BOX_SETTING, DEFAULT_BOX_ENLARGE_REL, cutoff);
  if(!r) HALT("--failed!");
  
  INDEXshape2swift(s->index) = sw;
  INDEXswift2shape(sw) = s->index;
  
  if(s->cont) scene->Activate(sw);
}

void SwiftInterface::initActivations(const ors::Graph& C, uint parentLevelsToDeactivate) {
  ors::Shape *s;
  ors::Body *b, *b2;
  ors::Joint *e;
  uint j, k, k2;
  
  /* deactivate some collision pairs:
    -- no `cont' -> no collisions with this object at all
    -- no collisions between shapes of same object
    -- no collisions between linked objects
    -- no collisions between objects liked via the tree via 3 links
  */
  
  //cout <<"collision active shapes: ";
  //for_list(k, s, C.shapes) if(s->cont) cout <<s->name <<' ';
  
  for_list(k, s, C.shapes) {
    if(!s->cont) {
      if(INDEXshape2swift(s->index)!=-1) scene->Deactivate(INDEXshape2swift(s->index));
    } else {
      if(INDEXshape2swift(s->index)!=-1) scene->Activate(INDEXshape2swift(s->index));
    }
  }
  //shapes within a body
  for_list(j, b, C.bodies) deactivate(b->shapes);
  //deactivate along edges...
  for_list(j, e, C.joints) {
    //cout <<"deactivating edge pair"; listWriteNames(ARRAY(e->from, e->to), cout); cout <<endl;
    deactivate(ARRAY(e->from, e->to));
  }
  //deactivate along trees...
  for_list(k, b, C.bodies) {
    MT::Array<ors::Body*> group, children;
    group.append(b);
    for(uint l=0; l<parentLevelsToDeactivate; l++) {
      //listWriteNames(group, cout);
      children.clear();
      for_list(k2, b2, group) {
        for_list(j, e, b2->outLinks) {
          children.setAppend(e->to);
          //listWriteNames(children, cout);
        }
      }
      group.setAppend(children);
    }
    deactivate(group);
  }
}

void SwiftInterface::deactivate(const MT::Array<ors::Body*>& bodies) {
  //cout <<"deactivating body group "; listWriteNames(bodies, cout); cout <<endl;
  MT::Array<ors::Shape*> shapes;
  uint i;  ors::Body *b;
  for_list(i,b,bodies) shapes.setAppend(b->shapes);
  deactivate(shapes);
}

void SwiftInterface::deactivate(const MT::Array<ors::Shape*>& shapes) {
  //cout <<"deactivating shape group "; listWriteNames(shapes, cout); cout <<endl;
  uint k1, k2;
  ors::Shape *s1, *s2;
  for_list(k1, s1, shapes) for_list(k2, s2, shapes) {
    if(k1>k2) deactivate(s1, s2);
  }
}

void SwiftInterface::deactivate(ors::Shape *s1, ors::Shape *s2) {
  if(INDEXshape2swift(s1->index)==-1 || INDEXshape2swift(s2->index)==-1) return;
  //cout <<"deactivating shape pair " <<s1->name <<'-' <<s2->name <<endl;
  scene->Deactivate(INDEXshape2swift(s1->index), INDEXshape2swift(s2->index));
}

void exportStateToSwift(const ors::Graph& C, SwiftInterface& swift) {
  ors::Shape *s;
  uint k;
  arr rot(3, 3);
  for_list(k, s, C.shapes) {
    s->X.rot.getMatrix(rot.p);
    if(swift.INDEXshape2swift(s->index)!=-1) {
      swift.scene->Set_Object_Transformation(swift.INDEXshape2swift(s->index), rot.p, s->X.pos.p);
      if(!s->cont) swift.scene->Deactivate(swift.INDEXshape2swift(s->index));
      //else         swift.scene->Activate( swift.INDEXshape2swift(s->index) );
    }
  }
}

void importProxiesFromSwift(ors::Graph& C, SwiftInterface& swift, bool dumpReport) {
  int i, j, k, np;
  int *oids, *num_contacts;
  SWIFT_Real *dists, *nearest_pts, *normals;
  
  try {
    swift.scene->Query_Contact_Determination(
      false, swift.cutoff, np,
      &oids, &num_contacts,
      &dists,
      &nearest_pts,
      &normals);
  } catch(const char *msg) {
    cout <<"... catching error '" <<msg <<"' -- SWIFT failed! .. no proxies for this posture!!..." <<endl;
    return;
  }
  
  if(dumpReport) {
    cout <<"contacts: np=" <<np <<endl;
    for(k=0, i=0; i<np; i++) {
      cout <<"* Object " <<C.shapes(oids[i <<1])->name <<" vs. Object " <<C.shapes(oids[(i <<1)+1])->name <<endl;
      cout <<"  #contacts = " <<num_contacts[i] <<endl;
      for(j=0; j<num_contacts[i]; j++, k++) {
        cout <<"  - contact " <<j <<endl;
        cout <<"    distance= " <<dists[k] <<endl;
        cout <<"    points  = " <<nearest_pts[6*k+0] <<' ' <<nearest_pts[6*k+1] <<' ' <<nearest_pts[6*k+2] <<' ' <<nearest_pts[6*k+3] <<' ' <<nearest_pts[6*k+4] <<' ' <<nearest_pts[6*k+5] <<endl;
        cout <<"    normals = " <<normals[3*k+0] <<' ' <<normals[3*k+1] <<' ' <<normals[3*k+2] <<endl;
      }
    }
  }
  
  //count total number of new proxies:
  for(k=0, i=0; i<np; i++) {
    if(num_contacts[i]>=0) k+=num_contacts[i];
    if(num_contacts[i]==-1) k++;
  }
  
  //resize old list
#if 0
  uint Nold=C.proxies.N;
  for(i=0; i<(int)Nold; i++) C.proxies(i)->age++;
  C.proxies.memMove=true;
  C.proxies.resizeCopy(Nold+k);
  for(i=0; i<k; i++) C.proxies(Nold+i) = new ors::Proxy;
#else
  uint Nold=0;
  listResize(C.proxies,k);
#endif
  
  //add contacts to list
  ors::Proxy *proxy;
  int a, b;
  for(k=0, i=0; i<np; i++) {
    a=swift.INDEXswift2shape(oids[i <<1]);
    b=swift.INDEXswift2shape(oids[(i <<1)+1]);
    //CHECK(swift.ids(a)==a && swift.ids(b)==b, "shape index does not coincide with swift index");
    //non-penetrating pair of objects
    if(num_contacts[i]>=0) for(j=0; j<num_contacts[i]; j++, k++) {
        proxy=C.proxies(Nold+k);
        proxy->a=a;
        proxy->b=b;
        proxy->age=0;
        proxy->d = dists[k];
        proxy->normal.set(&normals[3*k+0]);
        proxy->normal.normalize();
        //swift returns nearest points in the local frame -> transform them
        proxy->posA.set(&nearest_pts[6*k+0]);  proxy->posA = C.shapes(a)->X * proxy->posA;
        proxy->posB.set(&nearest_pts[6*k+3]);  proxy->posB = C.shapes(b)->X * proxy->posB;
        
        //if(a!=-1) proxy->velA=C.bodies(a).X.v + (C.bodies(a).X.w^(p+d-(C.bodies(a).X.p))); else proxy->velA.setZero();
        //if(b!=-1) proxy->velB=C.bodies(b).X.v + (C.bodies(b).X.w^(p-d-(C.bodies(b).X.p))); else proxy->velB.setZero();
        proxy->velA.setZero();
        proxy->velB.setZero();
        if(a!=-1 && b!=-1) proxy->rel.setDifference(C.shapes(a)->X, C.shapes(b)->X);
        else if(a!=-1) proxy->rel.setInverse(C.shapes(a)->X);
        else if(b!=-1) proxy->rel = C.shapes(b)->X;
        else           proxy->rel.setZero();
      }
      
    //penetrating pair of objects
    if(num_contacts[i]==-1) {
      proxy=C.proxies(Nold+k);
      proxy->a=a;
      proxy->b=b;
      proxy->age=0;
      proxy->d = -.0;
      if(C.shapes(a)->type==ors::meshST) proxy->posA = C.shapes(a)->X * C.shapes(a)->mesh.getMeanVertex();
      else proxy->posA = C.shapes(a)->X.pos;
      if(C.shapes(b)->type==ors::meshST) proxy->posB = C.shapes(b)->X * C.shapes(b)->mesh.getMeanVertex();
      else proxy->posB = C.shapes(b)->X.pos;
      proxy->normal = proxy->posA - proxy->posB; //normal always points from b to a
      proxy->normal.normalize();
      //!! IN PENETRATION we measure d as -1+(distance between object centers) - that gives a well-defined (though non-smooth) gradient!
      //proxy->d += (proxy->posA-proxy->posB).length();
      //proxy->posA -= .5*proxy->normal;
      //proxy->posB += .5*proxy->normal;
      //CHECK(fabs(fabs(proxy->d+.1)-(proxy->posA-proxy->posB).length())<1e-10, "")
      k++;
//       cout <<".";
      //MT_MSG("WARNING - swift penetration!!!");
    }
  }
  CHECK(k+Nold == C.proxies.N, "");
  
  //add pointClound stuff to list
  if(global_ANN) {
    ors::Shape *s;
    uint i, k, _i;
    arr R(3, 3), t(3);
    arr v, dists, _dists;
    intA idx, _idx;
    for_list(k, s, C.shapes) {
      if(!s->cont || s==global_ANN_shape) continue;
      
      //relative rotation and translation of shapes
      ors::Transformation rel;
      rel.setDifference(global_ANN_shape->X, s->X);
      rel.rot.getMatrix(R.p);
      t.setCarray(rel.pos.p, 3);
      
      //check for each vertex
      for(i=0; i<s->mesh.V.d0; i++) {
        v = s->mesh.V[i];
        v = R*v + t;
        global_ANN->getNN(dists, idx, v, 1);
        if(!i || dists(0)<_dists(0)) {
          _i=i;  _dists=dists;  _idx=idx;
        }
      }
      if(_dists(0)>swift.cutoff) continue;
      
      proxy = new ors::Proxy;
      C.proxies.append(proxy);
      proxy->a=global_ANN_shape->index;
      proxy->b=s->index;
      proxy->age=0;
      proxy->d = _dists(0);
      proxy->posA.set(&global_ANN_shape->mesh.V(_idx(0), 0));  proxy->posA = global_ANN_shape->X * proxy->posA;
      proxy->posB.set(&s->mesh.V(_i, 0));                      proxy->posB = s->X * proxy->posB;
      proxy->normal = proxy->posA - proxy->posB;
      proxy->normal.normalize();
      proxy->velA.setZero();
      proxy->velB.setZero();
    }
  }
  
  C.sortProxies();
}

void SwiftInterface::computeProxies(ors::Graph& C, bool dumpReport) {
  exportStateToSwift(C, *this);
  importProxiesFromSwift(C, *this, dumpReport);
}

void swiftQueryExactDistance(SwiftInterface& swift) {
  int i, np;
  int *oids;
  SWIFT_Real *dists;
  
  swift.scene->Query_Exact_Distance(false, SWIFT_INFINITY, np, &oids, &dists);
  
  cout <<"exact distances: np=" <<np <<endl;
  for(i=0; i<np; i++) {
    cout <<"    Object " <<oids[i <<1] <<" vs. Object "
         <<oids[(i <<1)+1] <<" = " <<dists[i] <<endl;
  }
}

#else
#include "util.h"
//#warning "MT_SWIFT undefined - using HALT implementations"
void SwiftInterface::init(const ors::Graph& C, double _cutoff) { MT_MSG("WARNING - creating dummy SwiftInterface"); }
void SwiftInterface::initActivations(const ors::Graph& C, uint parentLevelsToDeactivate) {}
void SwiftInterface::close() {}
void SwiftInterface::deactivate(const MT::Array<ors::Body*>& bodies) {}
void SwiftInterface::deactivate(ors::Shape *s1, ors::Shape *s2) {}
void SwiftInterface::computeProxies(ors::Graph& C, bool dumpReport) {}
SwiftInterface* SwiftInterface::newClone(const ors::Graph& G) const { return NULL; }
void swiftQueryExactDistance(SwiftInterface& swift) {}
SwiftInterface::~SwiftInterface() {}
#endif
