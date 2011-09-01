/*

   Demo
   ----

   Demonstrate how Gaussian process (GP) is used to learn the true shape of an
   object based on simulated  tactile observations 

 */

#define MT_IMPLEMENTATION
#include <MT/ors.h>
#include <MT/gaussianProcess.h>
#include <SD/ISF_GP.h>
#include <SD/utils.h>
#include <SD/surface_helpers.h>
#include <SD/graspObjects.h>

#define SD_DBG_C 

#define RNDM_FUNC_SCALE 0.6

#define cfgenv(type,name)    MT::getParameter< type >( name )
#define cfgenvd(type,name,defult)   MT::getParameter< type >( name , defult )

// ugly GLOBAL vars
MeshObject      *o_tru = NULL;
GraspObject_GP  *o_est = NULL;

OpenGL *gl[2]={NULL,NULL};
bool with_GL;
bool gl_auto;

void
drawBase(void*){
  glStandardLight(NULL);
  glDrawAxes(3.);
  glColor(.9,1.,.9);
}

void
show(MeshObject *o, uint win=0, const char * msg=""){
  if( with_GL && !gl[win]){
    gl[win] = new OpenGL;
    gl[win]->add(drawBase,NULL);
    gl[win]->add(glDrawMeshObject, o);
    gl[win]->add(plotDrawOpenGL,plotModule.WS);
  }

  if (!with_GL) return; // non interactive

  if ( gl_auto ) gl[win]->update(msg);	// use this to advance automatically
  else  gl[win]->watch(msg); // use this to advance manually 

}

void
plot_slice_belief(double x1){

  arr X, Y1, Y2;
  double lo, hi;
  o_tru->getEnclCube(lo,hi);
  X.setGrid(2, lo, hi, 50);
  Y1.resize(X.d0);
  Y2.resize(X.d0);
  uint i;
  FOR1D(X,i){ 
    Y1(i) = staticPhi( x1, X[i](0), X[i](1), o_tru);
    Y2(i) = staticPhi( x1, X[i](0), X[i](1), o_est);
  }
  plotClear();
  plotFunction(X, Y1);
  plotFunction(X, Y2*1.1);
  //plot(true);
  plot(false);
}

void
test_slice_belief(){

  plotGnuplot();
  arr lo,hi;
  o_tru->getEnclRect(lo,hi);
  lo -= (hi-lo)*.2; hi += (hi-lo)*.17;
  for(double x1 = lo(0); x1 < hi(0); x1+=(hi(0)-lo(0))/30)
    plot_slice_belief(x1);
  plotOpengl();
}

bool
show_iteration(uint i,uint obs){
  static uint e,u;
  static bool readconf=true;
  if (readconf){
    readconf=false;
    e=cfgenv(uint,"showeveryith");
    u=cfgenv(uint,"showuntil");
  }

  return ( (i+1)%e== 0 || i+1==obs || i<u);
}

/* ******  sampling points from surface */

void
append_observs_gradwalk(GaussianProcess& gp, PotentialField *ot , const arr &mins, const arr &maxs, uint obs_N=30){
  arr pts, grads;
  uint i;

  get_observs_gradwalk(pts,grads, ot, mins, maxs, obs_N);

  FOR1D(pts,i){
    gp.appendGradientObservation(pts[i], grads[i]);
    gp.appendObservation(pts[i], 0);
  }
}

void
append_observs_vrtx(GaussianProcess &gp, ors::Mesh &m, const uint obs_N=30){

  arr pts, grads;
  uint i;

  get_observs_vrtx(pts, grads, m, obs_N);

  FOR1D(pts,i){
    gp.appendGradientObservation(pts[i], grads[i]);
    gp.appendObservation(pts[i], 0);
  }
}

/* append obseravions by sampling rays and taling the intersection points with
 * the mesh
 */
void
append_observs_ray(GaussianProcess &gp, ors::Mesh &m, const uint obs_N=30){

  arr pts, grads;
  uint i;

  get_observs_ray(pts, grads, m, obs_N);

  FOR1D(pts,i){
    gp.appendGradientObservation(pts[i], grads[i]);
    gp.appendObservation(pts[i], 0);
  }
}

/* ********* tests ************ */

void
test_random_object_learning(uint obs=30){
  double s, gp_size=cfgenv(double,"gp_size");
  uint i;
  arr c=cfgenv(arr,"center");

  /* GP for random object generation and for learning */
  GraspObject_GP ot( c, gp_size);
  GraspObject_GP oe( c, gp_size);
  o_tru=&ot; o_est=&oe;
 
  /* generate object */
  rnd.seed(cfgenv(uint,"rnd_srfc_seed"));
  randomGP_on_random_points(ot.isf_gp.gp, c, gp_size, 20);
  ot.buildMesh();
  show(&ot,0);

  for(i=0;i<obs;++i){
    /* learn object */
    //1. variant
    /*{
      append_observs_ray(*o_est, *mp, 1);
      }*/
    //2. variant
    {/**/
      arr mins,maxs;
      o_tru->getEnclRect(mins,maxs);
      append_observs_gradwalk(oe.isf_gp.gp, &ot, mins, maxs, 1);
    }

    if (  show_iteration(i,obs)){
      oe.isf_gp.gp.recompute();
      oe.buildMesh();
      /* either */  s=ISF_common_volume(&ot,&oe);
      /* or */    //s=mesh_similar_view(ot.m,oe.m);
      SD_DBG("Observations:"<<i+1<<" similarity or loss:"<<s);
      show(&oe,1,STRING("Belief after " << i+1<< "observations;"<<
            " similarity or total norm dist: "<<s));
    }
  }

  oe.m.writeTriFile(cfgenv(MT::String,"finalbelieffile"));

  //gl_auto=false;
  show(NULL,0,STRING("True shape. Similarity or total norm loss: "<<s));
  test_slice_belief();
}

void
test_ISF_object_learning(uint obs=30){
  double s;
  uint i;

  /* GP for random object generation and for learning */
  GraspObject_Cylinder1 ot( ARR(0,0,0), ARR(0,0,1),
      cfgenv(double,"radius"),
      cfgenv(double,"sigma"),
      cfgenv(double,"height"));//c,z,r,s,h
  GraspObject_GP oe( ARR(0,0,0), cfgenv(double,"gp_size"));
  o_tru=&ot; o_est=&oe;

  /* generate object */
  ot.buildMesh();
  show(&ot,0);

  for(i=0;i<obs;++i){
    /* learn object */
    //1. variant
    /*{
      append_observs_ray(*o_est, *mp, 1);
      }*/
    //2. variant
    {/**/
      arr mins,maxs;
      o_tru->getEnclRect(mins,maxs);
      append_observs_gradwalk(oe.isf_gp.gp, &ot, mins, maxs, 1);
    }

    if (  show_iteration(i,obs) ){
      oe.isf_gp.gp.recompute();
      oe.buildMesh();
      /* either */  s=ISF_common_volume(&ot,&oe);
      /* or */    //s=mesh_similar_view(ot.m,oe.m);
      SD_DBG("Observations:"<<i+1<<" similarity or loss:"<<s);
      show(&oe,1,STRING("Belief after " << i+1<< "observations;"<<
            " similarity or total norm dist: "<<s));
    }
  }

  oe.m.writeTriFile(cfgenv(MT::String,"finalbelieffile"));

  gl_auto=false;
  show(NULL,0,STRING("True shape. Similarity or total norm loss: "<<s));
  test_slice_belief();
}

void
test_random_object_as_mesh_with_random_vertex_observations_scale(uint obs=30){

  double scale=1.*RNDM_FUNC_SCALE*2;

  GraspObject_GP ot( ARR(0,0,0), scale);
  GraspObject_GP oe( ARR(0,0,0), scale);
  o_tru=&ot; o_est=&oe;

  /* generate object */
  rnd.seed(cfgenv(uint,"rnd_srfc_seed"));
  randomGP_on_random_points(ot.isf_gp.gp, ARR(0,0,0), 1, 20);
  ot.isf_gp.scale_gp_params(RNDM_FUNC_SCALE, scXY);
  ot.isf_gp.scale_gp_input(RNDM_FUNC_SCALE, scXY);
  ot.isf_gp.gp.recompute();
  ot.buildMesh();
  show(o_tru,0);

  /* learn object */
  append_observs_vrtx(oe.isf_gp.gp, ot.m, obs);

  //gl_auto=false;
  show(NULL,0,STRING("True shape"));
  oe.isf_gp.gp.recompute();
  test_slice_belief();
}

void
test_mesh_object_learning(uint obs=30){
  uint i;
  double s=0;

  MeshObject      ot(cfgenv(MT::String,"meshfile"), cfgenv(arr,"center"), cfgenv(double,"objsize"));
  GraspObject_GP  oe( ARR(0,0,0), MT::getParameter<double>("gp_size"));
  o_tru=&ot; o_est=&oe;

  //ot.m.readFile("../../../../share/data/3dmodels/benchmarks/offs/m483.off");
  show(&ot,0);

  /* learn object */
  for(i=0;i<obs;++i){

    //append_observs_ray(oe.isf_gp.gp, ot.m, 1);
    append_observs_vrtx(oe.isf_gp.gp, ot.m, 1);

    if (  show_iteration(i,obs) ){
      oe.isf_gp.gp.recompute();
      oe.buildMesh();
      s=mesh_similar_view(ot.m,oe.m);
      SD_DBG("Observations:"<<i+1<<" similarity or loss:"<<s);
      show(&oe,1,STRING("Belief after " << i+1<< "observations;"<<
            " similarity or total norm loss: "<<s));
    }
  }

  oe.m.writeTriFile(cfgenv(MT::String,"finalbelieffile"));
  
  gl_auto=false;
  show(NULL,0,STRING("True shape. Similarity or total norm loss: "<<s));
  //test_slice_belief(); can't test, since ot is not potential
}

void
test_intersec(){

  ors::Vector a,b,c,lp,lv;
  ors::Vector pi;

  a = MT::Parameter<ors::Vector>("tri_a");
  b = MT::Parameter<ors::Vector>("tri_b");
  c = MT::Parameter<ors::Vector>("tri_c");
  lp = MT::Parameter<ors::Vector>("line_p");
  lv = MT::Parameter<ors::Vector>("line_v");

  int res = intersect3d_tri_line(pi,a,b,c,lp,lv);

  SD_INF("result is: " <<res);
  SD_INF("Intersection pt is: " <<pi);
}

void
init_gl_globals(){

  with_GL = MT::getParameter<int>("with_GL");
  gl_auto = MT::getParameter<int>("gl_auto");
  SD_INF("GL settings: with_GL="<<with_GL<<";  gl_auto="<<gl_auto);

}

void
usage(char *argv0){

SD_INF(argv0<<" -- Usage:");
SD_INF(argv0<<" [params]");
SD_INF("params are:\n\
\t  with_GL \\in {0,1} use or not GL windows. Say 0 for noninteractive(simulation).,\n\
\t  gl_auto \\in {0,1} auto advance when 1, when 0 requires user to press enter,\n\
\t  showeveryith \\in N show every n-th step, \n\
\t  showuntil \\in N show all steps until n-th,\n\
\t  center \\in R^3 [ a, b, c] where to place the object,\n\
\t  objsize \\in R variance of the unif distr around `center' where random values are sampled,\n\
\t  gp_size \\in R size of the GP used to estimate the real object,\n\
\t  demo \\in N show demo with that number (see source main(), switch(demo) ),\n\
 ");
SD_INF("NOTE: This usage message may be outdated. Consult source.");
SD_INF("NOTE: You can put the params one per line in 'param = value' format in MT.cfg in the working dir.");

}

int
main (int argc, char **argv){
  MT::initCmdLine(argc,argv);

  usage(argv[0]);

  init_gl_globals();

  //test_mesh_normal_error();
  //test_intersec();

  /* tests */
  int  demo = MT::getParameter<uint>("demo"); 
  SD_INF("DEMO: " << demo) ;
  switch(demo){
    case 1: test_random_object_learning(
                cfgenv(uint,"observations"));
            break;
    case 2: 
            test_random_object_as_mesh_with_random_vertex_observations_scale(50);
            break;
    case 3: 
            /* o_est->isf_gp.set_size(1.); test_random_object_as_mesh_with_random_vertex_observations(40);
            delete gl[0]; delete gl[1];gl[0]=NULL; gl[1]=NULL;o_tru->isf_gp.gp.clear(); o_est->isf_gp.gp.clear();
            o_est->isf_gp.set_size(1.*RNDM_FUNC_SCALE); test_random_object_as_mesh_with_random_vertex_observations_scale(50); */
            break;
    case 4: 
            test_mesh_object_learning(cfgenv(uint,"observations"));
            break;
    case 5: 
            test_ISF_object_learning(cfgenv(uint,"observations"));
            break;
    default: HALT("No such demo: " << demo)
  }

}
