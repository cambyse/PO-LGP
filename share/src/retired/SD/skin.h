#ifndef SD_skin_h
#define SD_skin_h

#include <Core/array.h>
#include <Ors/ors.h>
#include <MT/schunk.h>

struct tactile_arr_t {
  enum type {LINK=0, TIP=1};
  mlr::Body *b;
  mlr::Array<mlr::Transformation> tr;
  mlr::Array<mlr::Shape *>  sh_s,sh_n; 
  uintA mapp;
  byteA map;
  type t;

  tactile_arr_t(){};
  tactile_arr_t(mlr::KinematicWorld &G, const char *name, uintA mapparam, type t1){
    init(G,name,mapparam,t1); }
  void init(mlr::KinematicWorld &G, const char *name, uintA mapparam, type t1){
    b = G.getBodyByName(name);
    mapp = mapparam;
    t = t1;
  }
  void add_sensor_shapes(mlr::KinematicWorld &);
  void update_shapes();
  void senseskin(SchunkSkinModule *);
  void senseimg(const byteA &img);
  void sensebypass(const byteA &map);
  void settexel(const uint i, const uint j, const byte val);
  void read_sensor_transfs(const char *filename);

};

struct  fing_skin_t {
  tactile_arr_t link;
  tactile_arr_t tip;
};

struct grobi_skin_t {
  mlr::Array<fing_skin_t> fingers;
  fing_skin_t *th, *f2, *f3;
  
  grobi_skin_t(mlr::KinematicWorld &);
  void sense(SchunkSkinModule*,const byteA*const,const byteA*const);
  void update_shapes();
};

#ifdef  MLR_IMPLEMENTATION
#include "skin.cpp"
#endif

#endif// header ifdef
