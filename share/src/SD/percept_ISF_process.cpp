#include "percept_ISF_process.h"

Percept_ISF_process::Percept_ISF_process():Process("percept ISF"){
  graspobj=NULL;
  perc_out=NULL;
  /* perception vs cmd line */
  obj_comes_from = MT::getParameter<uint>("obj_comes_from");
  /* shape params */
  shape = MT::getParameter<uint>("shape");
  shapeprior = MT::getParameter<uint>("shapeprior",0);
  center = MT::getParameter<arr>("center");
  radius = MT::getParameter<double>("radius");
  sigma = MT::getParameter<double>("sigma");
  height = MT::getParameter<double>("height");
  zorientation  = MT::getParameter<arr>("orientation");
  /* plot  params */
  plotObservs = MT::getParameter<uint>("plotObservs", 1);
  observs = MT::getParameter<uint>("observs", 1);
  /* random */
  seed = MT::getParameter<uint>("seed", 1);

};

void
Percept_ISF_process::get_percept_obj(GraspObject **obj) {/*get object once*/

  SD_INF("getting object from perception ...");

  MT::Array<Object> *percobjs=&perc_out->objects;
  bool found=false;
  while(!found){
    perc_out->readAccess(this);
    if (percobjs->N && percobjs->p[1].found) found=true;
    perc_out->deAccess(this);
  }
  
  perc_out->readAccess(this);

  GraspObject_Sphere *os;
  GraspObject_Cylinder1 *oc;
  switch (shape) {
    case 2: *obj = new GraspObject_Cylinder1(); 
              oc = ((GraspObject_Cylinder1*)*obj);
              oc->h = percobjs->p[0].orsShapeParams(2);
              oc->r = percobjs->p[0].orsShapeParams(3)+.03;
              oc->c = percobjs->p[0].center3d+ARR(-.02,.0,.005);
              //above additions are hack to avoid systematic error
            break;
    case 0: *obj = new GraspObject_Sphere(); 
              os = ((GraspObject_Sphere*)*obj);
              os->r = percobjs->p[0].orsShapeParams(3)+.03;
              os->c = percobjs->p[0].center3d+ARR(-.02,.0,.005);
            break;
    default:  HALT("Provide proper shape!");
  }

  perc_out->deAccess(this);

  SD_INF("Got object.");
}

void
Percept_ISF_process::get_cmd_line_obj(GraspObject **obj, GraspObject **prior){

  arr pts,grads;
  uint i;
  GraspObject_GP *o;

  /* create the correct prior (only if
   * GraspObject_GP_analytical_prior) */
  switch (shapeprior) {
    case 3: *prior = new GraspObject_InfCylinder(); break;
    case 2: *prior = new GraspObject_Cylinder1(); break;
    case 1: *prior = new GraspObject_Sphere(); break;
    default:*prior = NULL;
  }

  /* create the right object type */
  switch (shape) {
    case 1: *obj = new GraspObject_InfCylinder(center, zorientation, radius, sigma); break;
    case 2: *obj = new GraspObject_Cylinder1(center, zorientation, radius, sigma, height); break;
    case 0: *obj = new GraspObject_Sphere(center, radius, sigma); break;
    case 3: *obj = new GraspObject_GPblob(); break;
    case 4: /* analytical prior */
            if (!prior) HALT("Provide meaningful -shapeprior!");
            *obj = new GraspObject_GP_analytical_prior(*prior);
            if (plotObservs)
              plotPoints(((GraspObject_GP*)*obj)->isf_gp.gp.X);
            break;
    case 5: 
            // *obj = new GraspObject_Cylinder1();
            *obj = new GraspObject_Sphere();
            break;
    case 6: /* sample GP object */
            *obj = new GraspObject_GP();
            o = (GraspObject_GP*)*obj;
            if (!prior) HALT("Provide meaningful -shapeprior!");
            MT::rnd.seed(seed);
            get_observs_gradwalk(pts, grads, *prior,
                ARR(-2,-2,0), ARR(2,0,2),
                observs);
            FOR1D(pts, i){
              o->isf_gp.gp.appendObservation(
                  pts[i],0);
              o->isf_gp.gp.appendGradientObservation(
                  pts[i],grads[i]/(norm(grads[i])));
            }
            o->isf_gp.gp.recompute();
            if (plotObservs) plotPoints(pts);
            break;
    default:  
            HALT("You should be kidding with that shape!");
  }
}

void
Percept_ISF_process::get_grasp_obj(GraspObject **obj, GraspObject **prior){


  switch (obj_comes_from) {
    case 0: get_percept_obj(obj); break;
    case 1: get_cmd_line_obj(obj, prior); break;
    default:  
            MT_MSG("provide meaningful obj_comes_from");
            MT_MSG("0: vision (camera running), or");
            MT_MSG("1: cmd line (-center '[.0 -.9 .99 ]' -shape 0 -height .25  -sigma 10 -radius .04) ");
            HALT("need object to grasp");
  }

}
void Percept_ISF_process::open(){
}

void Percept_ISF_process::step(){

  GraspObject *obj=NULL, *prior=NULL;

  get_grasp_obj(&obj, &prior);

  graspobj->writeAccess(this);
  graspobj->o=obj;
  graspobj->prior=prior;
  graspobj->deAccess(this);

  /* // TODO move this to a place it doesn't slow execution
  SD_INF("Building mesh, patience...");
  obj->buildMesh();
  if(prior) prior->buildMesh();
  */
  
}

void Percept_ISF_process::close(){
}

