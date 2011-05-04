#ifndef SD_skin_h
#define SD_skin_h


#include <MT/array.h>
#include <MT/ors.h>
#include <MT/schunk.h>

typedef MT::Array<ors::Transformation> transfA ;
typedef MT::Array<ors::Shape *> shapeL;

struct tactile_arr_t {
  enum type {LINK=0, TIP=1};
  ors::Body *b;
  transfA tr;
  shapeL sh_s,sh_n; 
  intA mapp;
  byteA map;
  type t;

  tactile_arr_t(){};
  tactile_arr_t(ors::Graph &G, const char *name, intA mapparam, type t1){
    init(G,name,mapparam,t1); }
  void init(ors::Graph &G, const char *name, intA mapparam, type t1){
    b = G.getBodyByName(name);
    mapp = mapparam;
    t = t1;
  }
  void add_sensor_shapes(ors::Graph &);
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
  MT::Array<fing_skin_t> fingers;
  fing_skin_t *th, *f2, *f3;
  
  grobi_skin_t(ors::Graph &);
  void sense(SchunkSkinModule*,const byteA*const,const byteA*const);
  void update_shapes();
};

#include "skin.cpp"

#endif// header ifdef
