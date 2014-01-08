/*  ---------------------------------------------------------------------
    Copyright 2013 Marc Toussaint
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
    along with this program. If not, see <http://www.gnu.org/licenses/>
    -----------------------------------------------------------------  */




/**
 * @file
 * @ingroup group_ors
 */
/**
 * @ingroup group_ors
 * @{
 */


#include "ors_swift.h"
#include <Algo/ann.h>

#ifdef MT_extern_SWIFT

#ifdef MT_SINGLE
#  define SWIFT_USE_FLOAT
#endif

#include <extern/SWIFT/SWIFT.h>
#undef min
#undef max

ANN *global_ANN=NULL;
ors::Shape *global_ANN_shape;

SwiftInterface::~SwiftInterface() {
  if(scene) delete scene;
  if(global_ANN) delete global_ANN;
  scene=NULL;
  cout <<" -- SwiftInterface closed" <<endl;
}

SwiftInterface::SwiftInterface(ors::KinematicWorld& _world)
  : world(_world), scene(NULL), cutoff(.1) {
  ors::Shape *s;
  uint k;
  bool r, add;
  
  if(scene) delete scene;
  scene=new SWIFT_Scene(true, false);

  INDEXswift2shape.resize(world.shapes.N);  INDEXswift2shape=-1;
  INDEXshape2swift.resize(world.shapes.N);  INDEXshape2swift=-1;
  
  cout <<" -- SwiftInterface init";
  for_list(k, s, world.shapes) {
    cout <<'.' <<flush;
    add=true;
    switch(s->type) {
      case ors::noneST: HALT("shapes should have a type - somehow wrong initialization..."); break;
      case ors::meshST: {
        //check if there is a specific swiftfile!
        MT::String *filename;
        filename=s->ats.getValue<MT::String>("swiftfile");
        if(!filename)
          filename=s->body->ats.getValue<MT::String>("swiftfile");
        if(filename) {
          r=scene->Add_General_Object(*filename, INDEXshape2swift(s->index), false);
          if(!r) HALT("--failed!");
        }
      } break;
      case ors::pointCloudST: {
        //for now, assume there is only ONE pointCloudObject!
        CHECK(s->mesh.V.N, "");
        global_ANN=new ANN;
        global_ANN_shape=s;
        global_ANN->setX(s->mesh.V);
        global_ANN->calculate();
        add=false;
      } break;
      case ors::markerST:
        add=false; // ignore (no collisions)
        break;
      default:
        break;
    }
    if(add) {
      CHECK(s->mesh.V.d0,"no mesh to add to SWIFT, something was wrongly initialized");
      r=scene->Add_Convex_Object(
          s->mesh.V.p, (int*)s->mesh.T.p,
          s->mesh.V.d0, s->mesh.T.d0, INDEXshape2swift(s->index), false,
          DEFAULT_ORIENTATION, DEFAULT_TRANSLATION, DEFAULT_SCALE,
          DEFAULT_BOX_SETTING, DEFAULT_BOX_ENLARGE_REL, cutoff);
      if(!r) HALT("--failed!");
      
      INDEXswift2shape(INDEXshape2swift(s->index)) = s->index;
    }
  }
  
  initActivations();
  
  pushToSwift();
  cout <<"...done" <<endl;
}

void SwiftInterface::reinitShape(const ors::Shape *s) {
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

void SwiftInterface::initActivations(uint parentLevelsToDeactivate) {
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
  //for_list(k, s, world.shapes) if(s->cont) cout <<s->name <<' ';
  
  for_list(k, s, world.shapes) {
    if(!s->cont) {
      if(INDEXshape2swift(s->index)!=-1) scene->Deactivate(INDEXshape2swift(s->index));
    } else {
      if(INDEXshape2swift(s->index)!=-1) scene->Activate(INDEXshape2swift(s->index));
    }
  }
  //shapes within a body
  for_list(j, b, world.bodies) deactivate(b->shapes);
  //deactivate along edges...
  for_list(j, e, world.joints) {
    //cout <<"deactivating edge pair"; listWriteNames(ARRAY(e->from, e->to), cout); cout <<endl;
    deactivate(ARRAY(e->from, e->to));
  }
  //deactivate along trees...
  for_list(k, b, world.bodies) {
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

void SwiftInterface::pushToSwift() {
  CHECK(INDEXshape2swift.N==world.shapes.N,"the number of shapes has changed");
  ors::Shape *s;
  uint k;
  ors::Matrix rot;
  for_list(k, s, world.shapes) {
    rot = s->X.rot.getMatrix();
    if(INDEXshape2swift(s->index)!=-1) {
      scene->Set_Object_Transformation(INDEXshape2swift(s->index), rot.p(), s->X.pos.p());
      if(!s->cont) scene->Deactivate(INDEXshape2swift(s->index));
      //else         scene->Activate( INDEXshape2swift(s->index) );
    }
  }
}

void SwiftInterface::pullFromSwift(bool dumpReport) {
  int i, j, k, np;
  int *oids, *num_contacts;
  SWIFT_Real *dists, *nearest_pts, *normals;
  
  try {
    scene->Query_Contact_Determination(
      false, cutoff, np,
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
      cout <<"* Shape '" <<world.shapes(oids[i <<1])->name <<"' vs. Shape '" <<world.shapes(oids[(i <<1)+1])->name <<"'" <<endl;
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
  
  listResize(world.proxies, k);
  
  //add contacts to list
  ors::Proxy *proxy;
  int a, b;
  for(k=0, i=0; i<np; i++) {
    a=INDEXswift2shape(oids[i <<1]);
    b=INDEXswift2shape(oids[(i <<1)+1]);
    //CHECK(ids(a)==a && ids(b)==b, "shape index does not coincide with swift index");
    
    //non-penetrating pair of objects
    if(num_contacts[i]>=0) for(j=0; j<num_contacts[i]; j++, k++) {
        proxy=world.proxies(k);
        proxy->a=a;
        proxy->b=b;
        proxy->d = dists[k];
        proxy->normal.set(&normals[3*k+0]);
        proxy->normal.normalize();
        //swift returns nearest points in the local frame -> transform them
        proxy->posA.set(&nearest_pts[6*k+0]);  proxy->posA = world.shapes(a)->X * proxy->posA;
        proxy->posB.set(&nearest_pts[6*k+3]);  proxy->posB = world.shapes(b)->X * proxy->posB;
        proxy->cenA = world.shapes(a)->X.pos;
        proxy->cenB = world.shapes(b)->X.pos;
//        if(world.shapes(a)->type==ors::meshST) proxy->cenA = world.shapes(a)->X * world.shapes(a)->mesh.getMeanVertex(); else proxy->cenA = world.shapes(a)->X.pos;
//        if(world.shapes(b)->type==ors::meshST) proxy->cenB = world.shapes(b)->X * world.shapes(b)->mesh.getMeanVertex(); else proxy->cenB = world.shapes(b)->X.pos;
        proxy->cenN = proxy->cenA - proxy->cenB; //normal always points from b to a
        proxy->cenD = proxy->cenN.length();
        proxy->cenN /= proxy->cenD;
      }
      
    //penetrating pair of objects
    if(num_contacts[i]==-1) {
      proxy=world.proxies(k);
      proxy->a=a;
      proxy->b=b;
      proxy->d = -.0;
      if(world.shapes(a)->type==ors::meshST) proxy->cenA = world.shapes(a)->X * world.shapes(a)->mesh.getMeanVertex(); else proxy->cenA = world.shapes(a)->X.pos;
      if(world.shapes(b)->type==ors::meshST) proxy->cenB = world.shapes(b)->X * world.shapes(b)->mesh.getMeanVertex(); else proxy->cenB = world.shapes(b)->X.pos;
      proxy->cenN = proxy->cenA - proxy->cenB; //normal always points from b to a
      proxy->cenD = proxy->cenN.length();
      proxy->cenN /= proxy->cenD;
      
      //copy to pos..
      proxy->posA = proxy->cenA;
      proxy->posB = proxy->cenB;
      proxy->normal = proxy->cenN;
      
      ///! IN PENETRATION we measure d as -1+(distance between object centers) - that gives a well-defined (though non-smooth) gradient!
//      proxy->d += -1.+(proxy->posA-proxy->posB).length();
      k++;
    }
  }
  CHECK(k == (int)world.proxies.N, "");
  
  //add pointClound stuff to list
  if(global_ANN) {
    ors::Shape *s;
    uint i, k, _i;
    arr R(3, 3), t(3);
    arr v, dists, _dists;
    intA idx, _idx;
    for_list(k, s, world.shapes) {
      if(!s->cont || s==global_ANN_shape) continue;
      
      //relative rotation and translation of shapes
      ors::Transformation rel;
      rel.setDifference(global_ANN_shape->X, s->X);
      rel.rot.getMatrix(R.p);
      t = ARRAY(rel.pos);
      
      //check for each vertex
      for(i=0; i<s->mesh.V.d0; i++) {
        v = s->mesh.V[i];
        v = R*v + t;
        global_ANN->getkNN(dists, idx, v, 1);
        if(!i || dists(0)<_dists(0)) {
          _i=i;  _dists=dists;  _idx=idx;
        }
      }
      if(_dists(0)>cutoff) continue;
      
      proxy = new ors::Proxy;
      world.proxies.append(proxy);
      proxy->a=global_ANN_shape->index;
      proxy->b=s->index;
      proxy->d = _dists(0);
      proxy->posA.set(&global_ANN_shape->mesh.V(_idx(0), 0));  proxy->posA = global_ANN_shape->X * proxy->posA;
      proxy->posB.set(&s->mesh.V(_i, 0));                      proxy->posB = s->X * proxy->posB;
      proxy->normal = proxy->posA - proxy->posB;
      proxy->normal.normalize();
    }
  }
}

void SwiftInterface::step(bool dumpReport) {
  pushToSwift();
  pullFromSwift(dumpReport);
}

void SwiftInterface::swiftQueryExactDistance() {
  int i, np;
  int *oids;
  SWIFT_Real *dists;
  
  scene->Query_Exact_Distance(false, SWIFT_INFINITY, np, &oids, &dists);
  
  cout <<"exact distances: np=" <<np <<endl;
  for(i=0; i<np; i++) {
    cout <<"    Object " <<oids[i <<1] <<" vs. Object "
         <<oids[(i <<1)+1] <<" = " <<dists[i] <<endl;
  }
}

#else
#include <Core/util.h>
void setCutoff(double _cutoff){ cutoff=_cutoff; }

  void SwiftInterface::step(bool dumpReport=false){}
  void SwiftInterface::pushToSwift() {}
  void SwiftInterface::pullFromSwift(bool dumpReport) {}

  void SwiftInterface::reinitShape(const ors::Shape *s) {}
//  void close();
  void SwiftInterface::deactivate(ors::Shape *s1, ors::Shape *s2) {}
  void SwiftInterface::deactivate(const MT::Array<ors::Shape*>& shapes) {}
  void SwiftInterface::deactivate(const MT::Array<ors::Body*>& bodies) {}
  void SwiftInterface::initActivations(uint parentLevelsToDeactivate=3) {}
  void SwiftInterface::swiftQueryExactDistance() {}
#endif
/** @} */
