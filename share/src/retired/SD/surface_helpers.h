#ifndef SD_surface_helpers_h
#define SD_surface_helpers_h

#include <Ors/ors.h>
#include <SD/utils.h>
#include "graspObjects.h"

/* These obbjects are used in the commented out code for debug
struct dbg_obj_t{ // for visual debugging of points and frames
  ors::Shape s;
  ors::Frame f;

  void siz(double a,double b,double c,double d){s.size[0]=a;s.size[1]=b;s.size[2]=c;s.size[3]=d;};
  void col(double r,double g,double b){s.color[0]=r;s.color[1]=g;s.color[2]=b;};
  void pos(double *p){f.p.set(p);};
  void ori(double *z){ ors::Vector z1(0,0,1), z2; z2.set(z); f.r.setDiff(z1,z2);};

  void set(double* sz, double *cl, double *p, double *z ){
    memcpy(s.size, sz, 4); memcpy(s.color, cl, 3);
    pos(p); ori(z);
  };
  dbg_obj_t(){siz(0,0,0,0); col(0,0,0);f.setZero();};
};

dbg_obj_t dbg_objs[4];
dbg_obj_t *pt1=dbg_objs+0,
           *pt2=dbg_objs+1,
           *ray=dbg_objs+2,
           *cub=dbg_objs+3;
void
glDrawSimpleObjs(void*){
  for(uint i=0;i<sizeof(dbg_objs)/sizeof(dbg_obj_t);++i){
    glDrawShape(&dbg_objs[i].s,dbg_objs[i].f);
  }
}

in init_gl_globals()

  // init shapes for debug drawing for  ray intersection and volume measure
  pt1->s.type=pt2->s.type=1;
  pt1->siz(.01,.01,.01,.01);
  pt1->col(.1,.1,.8);

  pt2->siz(.01,.01,.01,.01);
  pt2->col(.8,.1,.1);

  ray->s.type=0;
  ray->col(.5,.5,.5);
  ray->siz(.001,.001,10,.001);

  cub->s.type=0;
  cub->siz(.1,.1,.1,.1);
  cub->col(.1,.1,.8);

in show()
    gl[win]->add(glDrawSimpleObjs,NULL);
*/

/**
 * return
 * 1: l \cap t-plane = p && p \in t
 * 2: l \cap t-plane = p && p \not\in t
 * 0: l \cap t-plane = \empty
 * 3: l \cap t-plane = l
 * -1> bad triangle
 * if bad triangle return -1
 * If line intersects the plane of the triangle (outside or inside the
 * triangle) put the point into intersec.
 * http://www.softsurfer.com/Archive/algorithm_0105/algorithm_0105.htm#intersect_RayTriangle()
 */
int
intersect3d_tri_line(ors::Vector &intersec,
    const ors::Vector &ta, const ors::Vector &tb, const ors::Vector &tc,
    const ors::Vector &lp, const ors::Vector &lv);

/** return true if line intersects mesh (but not if l ine lies in a face);
 * false otherwise
 * return all faces which intersect with the line;
 * return al points at which the line intersects with the mesh.
 */
bool
intersect3d_mesh_line(arr &sec, uintA &tri, const ors::Mesh &mesh,
    const ors::Vector &lp, const ors::Vector &lv);

/* ******  similarity of  surfaces */

/** meaasire for similarity of two meshes.
 * N times, inside the bounding cuboid of the real shape, sample uniformly a
 * point and direction, i.e. sample a ray. Take the distance between
 * intersections with real and estimated surface. Average the distance by N.
 * Normalize by the volume of the cuboid.
 */
double mesh_similar_view(ors::Mesh &mtrue, ors::Mesh &mestm);

/** take the common volume of real and estimated object as measure of similarity.
 * Iterative naive implementation. */
double ISF_common_volume(PotentialField *ot, PotentialField *oe, const arr lo, const arr hi, const double step); 

/** recursive implementation of the volume similarity -- prunning incl.  */
double ISF_common_volume_rec(PotentialField *ot, PotentialField *oe,
    const arr lo, const arr hi, const double step);

/** calculate the common volume measure.
 * use impl 1(iterative) or 2(recursive)(default)*/
double ISF_common_volume(PotentialField *ot,PotentialField *oe, int impl=2);

/* ******  sampling points from surface */

/** generate observations by sampling a point and walking along its gradient
 * until encountering the zero
 */
void get_observs_gradwalk(arr& pts, arr& grads, PotentialField *ot , const arr &mins, const arr &maxs, uint obs_N=30);

void get_observs_vrtx(arr& pts, arr& grads, ors::Mesh &m, const uint obs_N=30);

/* append obseravions by sampling rays and taling the intersection points with
 * the mesh
 */
void get_observs_ray(arr& pts, arr& grads, ors::Mesh &m, const uint obs_N=30);


#ifdef  MT_IMPLEMENTATION
#  include "surface_helpers.cpp"
#endif

#endif // ifundef
