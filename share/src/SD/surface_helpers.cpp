#include <MT/ors.h>
#include "ISF_GP.h"
#include "utils.h"
#include "surface_helpers.h"
#include "graspObjects.h"

/**
 * if triangle and line have an intersection point return 1.
 * if triangle plane and line have an intersection point outside triangle return 2.
 * if triangle plane and line don't intersect return 0.
 * if line lies in triangle plane return 3
 * if bad triangle return -1
 * If line intersects the plane of the triangle (outside or inside the
 * triangle) put the point into intersec.
 * http://www.softsurfer.com/Archive/algorithm_0105/algorithm_0105.htm#intersect_RayTriangle()
 */
int
intersect3d_tri_line(ors::Vector &intersec, const ors::Vector &ta, const ors::Vector &tb, const ors::Vector &tc, const ors::Vector &lp, const ors::Vector &lv){

  ors::Vector vab, vac, tn;// triangle vectors and normal
  ors::Vector ta2lp, vai; // triangle to line "begin", triangle to intersection
  double ratio, pr1, pr2, s, t;

  // vectors defining the triangle plane
  vab = tb - ta;
  vac = tc - ta;

  // plane normal vector
  tn = vab ^ vac;

  if ( tn.length() == 0 ) return -1; // bad triangle

  ta2lp = lp - ta;
  pr1 = -(tn * ta2lp);
  pr2 = tn * lv;

  if (pr2 == 0){
    if (pr1 == 0 ) return 3; // \in triangle plane
    else return 0; // parallel to triangle plane
  }

  ratio = pr1/pr2;

  // intersection is done.
  intersec = lp + ratio * lv;

  // find whether inside triangle
  vai = intersec - ta;
  //(uv * wv - vv * wu) / (uv * uv - uu * vv)
  s = ( (vab * vac) * (vai * vac) - (vac * vac) * (vai * vab) )
    / ( (vab * vac) * (vab * vac) - (vab * vab) * (vac * vac) );
  if ( s < 0 || 1 < s ) return 2;
  //(uv * wu - uu * wv) / D
  t = ( (vab * vac) * (vai * vab) - (vab * vab) * (vai * vac) )
    / ( (vab * vac) * (vab * vac) - (vab * vab) * (vac * vac) );
  if ( t < 0 || 1 < t+s ) return 2;

  return 1;

}

/** return true if line intersects mesh (but not if l ine lies in a face);
 * false otherwise
 * return all faces which intersect with the line;
 * return al points at which the line intersects with the mesh.
 */
bool
intersect3d_mesh_line(arr &sec, uintA &tri, const ors::Mesh &mesh, const ors::Vector &lp, const ors::Vector &lv){

  uint i,j;
  ors::Vector intersec;
  ors::Vector t[3];
  int res;

  tri.clear();sec.clear();
  // for all faces
  FOR1D(mesh.T, i){
    // get the triangle points as Vectors
    for(j=0; j<mesh.V.d1; ++j) t[j].set(mesh.V[mesh.T(i,j)].p);
    // check intersection
    res = intersect3d_tri_line(intersec,t[0],t[1],t[2],lp,lv);
    //SD_DBG("checking triangle " << i << ": " << mesh.T[i] << "; result: " << res);
    if (res == 1){  // intersect inside
      tri.append(i);
      sec.append(intersec.p,3);
    }
  }
  if (tri.N) sec.reshape(tri.d0,3);
  return tri.N>0;
  //SD_DBG("found :" << tri <<"; total:" << tri.N);

}

/* ******  similarity of  surfaces */

/** meaasire for similarity of two meshes.
 * N times, inside the bounding cuboid of the real shape, sample uniformly a
 * point and direction, i.e. sample a ray. Take the distance between
 * intersections with real and estimated surface. Average the distance by N.
 * Normalize by the volume of the cuboid.
 */
double
mesh_similar_view(ors::Mesh &mtrue, ors::Mesh &mestm){

  // FIX hmmmm! >:o|
  if ((mestm.T.N<10 && mtrue.V.N>4) || mtrue.V.N<10){ SD_ERR("mesh is empty!"); return 100;}

  uint d = mtrue.V.d1;
  uint i,j,ray_N=300;
  arr vertx_col, var(d), mins(d), maxs(d), obs;
  double total_diff=0, diff=0;
  uintA facestrue, facesestm;
  arr intertrue, interestm;
  uint idiff = 0;
  ors::Vector lp,lv;

  // find variance of mesh
  vertx_col = ~mtrue.V;

  for(i = 0; i<d; ++i){
    mins(i) = vertx_col[i].min();
    maxs(i) = vertx_col[i].max();
  }
  var = maxs - mins;

  SD_DBG1("mins: "<<mins<<"; var: "<<var<<"; maxs: "<<maxs );
  for(i = 0; i<ray_N;++i){

    // sample a ray which intersects both meshes
    do{
      lp.set( // random point inside enclosing cube 
          mins(0) + var(0) * rnd.uni(),
          mins(1) + var(1) * rnd.uni(),
          mins(2) + var(2) * rnd.uni()
          );
      lv.set( // random direction
          -1 + 2 * rnd.uni(),
          -1 + 2 * rnd.uni(),
          -1 + 2 * rnd.uni()
          );
      SD_DBG1("point: "<<lp<<"; direction: "<<lv);
    }while(
        !intersect3d_mesh_line(intertrue,facestrue,mtrue,lp,lv) || // 
        !intersect3d_mesh_line(interestm,facesestm,mestm,lp,lv)
        );
    SD_DBG1("true faces: " << facestrue<< " estimate faces: "<< facesestm);
    // take the closest distance
    diff=norm(intertrue[0] - interestm[0]);
    idiff=0;
    FOR1D(interestm,j){ 
      if (norm(intertrue[0]-interestm[j]) < diff){
        diff = norm(intertrue[0]-interestm[j]);
        idiff = j;
      }
    }

    total_diff += diff;

    /*if(0){DEBUG ray intersections 
    pt1->pos(intertrue[0].p); pt2->pos(interestm[idiff].p);
    ray->pos(lp.v); ray->ori(lv.v);
    show(NULL,0,STRING("I="<<i));
    }*/
  }
  SD_DBG1("total intersections: "<<i<<"; total diff:"
  <<total_diff<<"; avg diff: "<<total_diff/i
  <<"; norm avg diff"<<total_diff/i/(var(0)*var(1)*var(2)));
  return total_diff/i/(var(0)*var(1)*var(2));
}

/** take the common volume of real and estimated object as measure of similarity.
 */
double
ISF_common_volume(PotentialField *ot, PotentialField *oe, const arr lo, const arr hi, const double step) {

  CHECK(lo<hi, "lo > hi");
  uint i,j,k,b;
  uintA steps(3);
  arr pt,corner;
  double vol_tru=0, vol_com=0, vol_step;
  bool in_est, in_tru;

  FOR1D(steps,i) steps(i) = ceil((hi(i)-lo(i))/step);
  vol_step = step*step*step;

  for(i=0;i<steps(0);++i) for(j=0;j<steps(1);++j) for(k=0;k<steps(2);++k){
    pt = lo + step*ARR(i,j,k);
    in_tru=in_est=false;
    for(b=0;b<8;++b){ // b goes 000,001,010,...,111 positive neighbours of a point
      corner=ARR(
          (b&1)/1, // b&1 is 1 for 001
          (b&2)/2, // b&2 is 2 for 110
          (b&4)/4 //
          );
      corner=pt + step*corner;
      in_tru = in_tru || ot->psi(NULL,NULL,corner) < 0;
      in_est = in_est || oe->psi(NULL,NULL,corner) < 0; 
      if (in_tru && in_est) break;

    }
    if (in_tru) vol_tru += vol_step;
    if (in_tru && in_est) vol_com += vol_step;
    if (in_est && !in_tru) {vol_com -= vol_step;SD_DBG1("penalty:"<<vol_step);}//TODO penalize est exceeding true. OK? 
    
    /*if(1){ DEBUG cubes 
      cub->siz(step,step,step,step);
      cub->col(.6,.6,.6);
      cub->pos((pt+.5*step).p); 
      if (in_tru && in_est) {cub->col(.1,.8,.1); }else {if (in_tru) cub->col(.8,.1,.1);} 
      show(NULL,0,STRING("[i,j,k]="<<ARR(i,j,k)<<";true, common vol:"<<ARR(vol_tru, vol_com)));
    }*/
  }

  return vol_com/vol_tru;

}

/** 
 * for the current box, check if it is
 * - entirely in both est and tru obj, then return box volume
 * - entirely outside both est and tru obj, then return zero volume
 * - height becomes smaller than step, then set volumes and return
 * else 
 * - go deeper in recursion
 */
void
ISF_common_volume_rec(double& vol_tru, double& vol_com,PotentialField *ot, PotentialField *oe,
    const arr& lo, const double hi, const double step) {

  uint i;
  arr corner;
  double loc_vol_tru=0, loc_vol_com=0;
  bool whol_in_est, whol_in_tru, part_in_est, part_in_tru, c_in_tru, c_in_est;

  vol_tru = vol_com =0;

  /* check if inside outside or partially */
  part_in_tru=part_in_est=false;
  whol_in_tru=whol_in_est=true;
  for(i=0;i<8;++i){ // i goes 000,001,010,...,111 positive neighbours of a point
    corner=ARR(
        (i&1)/1, // i&1 is 1 for 001
        (i&2)/2, // i&2 is 2 for 110
        (i&4)/4 //
        );
    corner=lo + hi*corner;
    c_in_tru = ot->psi(NULL,NULL,corner) < 0;
    c_in_est = oe->psi(NULL,NULL,corner) < 0;
    part_in_tru  = part_in_tru || c_in_tru;// one corner inside is enough(max)
    part_in_est  = part_in_est || c_in_est;
    whol_in_tru  = whol_in_tru && c_in_tru;// one corner outside is enough (min)
    whol_in_est  = whol_in_est && c_in_est; 
  }
  /*if(0){DEBUG recursive cubes
    cub->siz(hi,hi,hi,hi);
    cub->pos((corner+.5*hi).p); 
    cub->col(.6,.6,.6);
    if (whol_in_est == whol_in_tru) cub->col(.1,.8,.1);
    if (whol_in_tru != whol_in_est) cub->col(.8,.1,.1); 
    show(NULL,0,STRING("hi="<<hi));
  }*/

  if (whol_in_tru &&  whol_in_est){ vol_tru=vol_com=hi*hi*hi; return;}; // both inside
  if (whol_in_tru && !part_in_est){ vol_tru= hi*hi*hi; vol_com=0; return;}; // inside true outside est
  if (whol_in_est && !part_in_tru){ vol_com=-hi*hi*hi;SD_DBG1("penalty:"<<hi*hi*hi);return;}//TODO penalize est exceeding true. OK? 
  if (hi/step < 5 && !part_in_tru && !part_in_est){ vol_tru=vol_com=0; return;}//both entirely outside
  if (hi/2<step){ // too small grain to continue
    if( part_in_tru ) vol_tru = hi*hi*hi;
    if( part_in_tru &&  part_in_est ) vol_com = hi*hi*hi;
    if (part_in_est && !part_in_tru) {vol_com = -hi*hi*hi;SD_DBG1("penalty:"<<hi*hi*hi);}//TODO penalize est exceeding true. OK? 
    return;
  }

  /* step down */
  for(i=0;i<8;++i){
    corner=ARR( (i&1)/1, (i&2)/2, (i&4)/4);
    ISF_common_volume_rec(loc_vol_tru, loc_vol_com, ot, oe,
        lo + .5*hi*corner, .5*hi, step);
    vol_tru += loc_vol_tru;
    vol_com += loc_vol_com;
  }
}


/** recursive implementation of the volume similarity -- prunning incl.
 */
double
ISF_common_volume_rec(PotentialField *ot, PotentialField *oe,
    const arr lo, const arr hi, const double step) {
  double h,vol_tru,vol_com;

  CHECK(lo<hi, "lo > hi");

  /* come up with  hi1 so that
   * - lo hi1 contains lo hi
   * - is cube
   * - (lo-hi) is power-of-two-multipe of step*/
  
  h = step * lgep2(ceil((hi-lo).max()/step));
  //SD_DBG("lo"<<lo<<"; hi"<<hi<<"; (hi-lo).max()="<<(hi-lo).max()<<"; max/step="<<(hi-lo).max()/step<<"; ceil="<<ceil((hi-lo).max()/step)<<"; lgep2="<<lgep2(ceil((hi-lo).max()/step))<<"; h="<<h);
  
  ISF_common_volume_rec(vol_tru, vol_com, ot, oe, lo, h, step);

  return vol_com/vol_tru;
}

double
ISF_common_volume(PotentialField *ot,PotentialField *oe, int impl){
  
    double s,step=0;
    arr  mins, maxs;
    ot->getEnclRect(mins,maxs);
    if (mins!=maxs) {
      mins -= (maxs-mins)*.2; maxs += (maxs-mins)*.17;
      step = MT::getParameter<double>("gp_size")/16;
      switch(impl){
        case 1: s=ISF_common_volume(ot,oe,mins,maxs,step); break;
        case 2: s=ISF_common_volume_rec(ot, oe,mins,maxs,step); break;
      }
    }
    SD_DBG1("similarity: method="<<impl<<" s="<<s);
    return s;

}

/* ******  sampling points from surface */

/** generate observations by sampling a point and walking along its gradient
 * until encountering the zero
 */
void
get_observs_gradwalk(arr& pts, arr& grads, PotentialField *ot , const arr &mins, const arr &maxs, uint obs_N){
  uint d = 3;
  uint i, j;
  arr grad, var(d);
  arr p,o;
  uintA faces; arr inter;
  double y,eps=.01;

  pts.resize(obs_N, 3);
  grads.resizeAs(pts);

  var = maxs - mins;
  SD_DBG1("mins: "<<mins<<"; var: "<<var<<"; maxs: "<<maxs );

  for(i=0; i<obs_N; ++i){

    do{
      // sample a point inside the cube
      o = ARR( 
          mins(0) + var(0) * rnd.uni(),
          mins(1) + var(1) * rnd.uni(),
          mins(2) + var(2) * rnd.uni()
          );
      SD_DBG1("point: "<<o);
      p=o;

      j=1000;
      // walk along gradient
      do{
        y = ot->psi(&grad,NULL, o);
        o = o - (y>0?1:-1)*eps*grad/norm(grad);
        --j;
        
        /*if(1){DEBUG gradwalk
          pt1->pos(p.p); pt2->pos(o.p); 
          ray->pos(o.p); ray->ori(grad.p);
          show(NULL,0,STRING("y="<<y<<"; |grad|="<<norm(grad)));
        }*/ 
      }while(eps<y*y && 0<norm(grad) && 0<j); // big enough and not stuck
    }while(eps<y*y );

    // append first intersected face only
    pts[i] = o;
    grads[i] = grad;
  }
}

void
get_observs_vrtx(arr& pts,arr& grads, ors::Mesh &m, const uint obs_N){

  pts.resize(obs_N, 3);
  grads.resizeAs(pts);


  // generate observations from mesh vertices
  m.computeNormals();

  uintA observations(m.V.d0);
  for (uint i=0; i<observations.N; ++i) observations(i) = i;

  observations.permuteRandomly();

  for (uint i=0; i<m.V.d0 && i<obs_N; ++i){
    arr xi; xi = m.V[observations(i)];
    arr x_normal; x_normal = m.Vn[observations(i) ];
    if ( sum(xi) != sum(xi) || sum(x_normal) != sum(x_normal)){ // check for NaN
      SD_ERR( "encountered NaN in observations" <<
          observations(i) << ":\t" << xi << "\tn:" << x_normal );
      continue; 
    }
    SD_DBG1( observations(i) << ":\t" << xi << "\tn:" << x_normal );
    // the gradient is the normal
    pts[i]() = xi;
    grads[i]() = x_normal;
  }
}

/* append obseravions by sampling rays and taling the intersection points with
 * the mesh
 */
void
get_observs_ray(arr& pts,arr& grads, ors::Mesh &m, const uint obs_N){

  uint d = m.V.d1;
  uint i;
  arr vertx_col, var(d), mins(d), maxs(d), obs;
  ors::Vector lp,lv;
  uintA faces; arr inter;

  pts.resize(obs_N, 3);
  grads.resizeAs(pts);

  m.computeNormals();

  // find variance of mesh
  vertx_col = ~m.V;
  for(i = 0; i<d; ++i){
    mins(i) = vertx_col[i].min();
    maxs(i) = vertx_col[i].max();
  }
  var = maxs - mins;
  SD_DBG1("mins: "<<mins<<"; var: "<<var<<"; maxs: "<<maxs );

  for(i = 0; i<obs_N; ++i){

    // sample a ray which intersects mesh
    do{
      lp.set( // random point inside enclosing cube 
          mins(0) + var(0) * rnd.uni(),
          mins(1) + var(1) * rnd.uni(),
          mins(2) + var(2) * rnd.uni()
          );
      lv.set( // random direction
          -1 + 2 * rnd.uni(),
          -1 + 2 * rnd.uni(),
          -1 + 2 * rnd.uni()
          );
      SD_DBG1("point: "<<lp<<"; direction: "<<lv);
    } while( ! intersect3d_mesh_line(inter,faces,m,lp,lv));

    /*if(0){DEBUG ray intersections 
      pt1->pos(inter[0].p); 
      ray->pos(lp.v); ray->ori(lv.v);
      show(NULL,0,STRING("I="<<i));
    }*/

    // append first intersected face only
    pts[i]() = inter[0];
    grads[i]() = m.Tn[faces(0)];
  }
}

