#include "skin.h"
#include <MT/array.h>
#include <MT/ors.h>
#include <MT/schunk.h>

#define SD_SIZE(s, s1, s2, s3, s4)  s[0]=(s1); s[1]=(s2); s[2]=(s3); s[3]=(s4);
#define SD_COL(c, c1, c2, c3)       c[0]=(c1); c[1]=(c2); c[2]=(c3);

void
tactile_arr_t::read_sensor_transfs(const char *filename){
  std::ifstream f(filename);
  f >> tr;
  tr.reshape( t==TIP?13:14 , 6 );
}

void
tactile_arr_t::senseskin(SchunkSkinModule *skin){
  byteA img;
  skin->getImage(img);
  map = img.sub((uint)(mapp(0)), mapp(1), (uint)(mapp(2)), mapp(3)); 
}

void
tactile_arr_t::senseimg(const byteA &img){
  map = img.sub((uint)(mapp(0)), mapp(1), (uint)(mapp(2)), mapp(3)); 
}

void
tactile_arr_t::sensebypass(const byteA &map1){
  map = map1;
}

void
tactile_arr_t::settexel(const uint i, const uint j, const byte val){
  map(i,j) = val;
}

void
tactile_arr_t::update_shapes(){
  uint i,j;
  FOR2D(map,i,j) {
    sh_s( t==TIP ? (sh_s.d0-1-i) : i ,
        j)->color[0]=(double)(map(i,j))/255.; 
    sh_n( t==TIP ? (sh_n.d0-1-i) : i ,
        j)->color[1]=(double)(map(i,j))/255.;
    sh_n( t==TIP ? (sh_n.d0-1-i) : i ,
        j)->size[2]=.02*(double)( map(i,j))/255.;
  }
}

void
tactile_arr_t::add_sensor_shapes(ors::Graph &G){
  ors::Shape *s1,*s2;
  MT::String *n,*a; 
  uint i,j;
  FOR2D(tr,i,j){
    MT::String s; s<<tr(i,j);

    s1 =  new ors::Shape(G.shapes, b);
    s1->reset();
    a = new MT::String("a"); *a<<i<<" "<<j;
    s1->name = a->p;
    s1->type=ors::boxST;
    s1->rel.setText(s.p);
    SD_SIZE(s1->size, .0034, .0034, .001, .0034);
    SD_COL(s1->color, .8, .1, .1 );
    sh_s.append(s1);

    s2 =  new ors::Shape(G.shapes, b);
    s2->reset();
    n = new MT::String("n"); *a<<i<<" "<<j;
    s2->name = n->p;
    s2->type=ors::cylinderST;
    s2->rel.setText(s.p);
    SD_SIZE(s2->size, .0034, .0034, .02, .00003);
    SD_COL(s2->color, .1, .8, .1  );
    sh_n.append(s2);
  }
  sh_s.reshape( t==TIP ? 13 : 14 , 6 );
  sh_n.reshapeAs(sh_s);
}

grobi_skin_t::grobi_skin_t(ors::Graph &ors){
  uint i;

  /* 3 fingers */
  fingers.resize(3);
  fingers(0).tip.init(ors, "tip1",   TUP(1,  13, 7,  12), tactile_arr_t::TIP);
  fingers(0).link.init(ors, "fing1", TUP(15, -1, 7,  12), tactile_arr_t::LINK);
  fingers(1).tip.init(ors, "tip2",   TUP(1,  13, 14, -1), tactile_arr_t::TIP);
  fingers(1).link.init(ors, "fing2", TUP(15, -1, 14, -1), tactile_arr_t::LINK);
  fingers(2).tip.init(ors, "tip3",   TUP(1,  13, 0,  5), tactile_arr_t::TIP);
  fingers(2).link.init(ors, "fing3", TUP(15, -1, 0,  5), tactile_arr_t::LINK);

  /* read frames and add to ors */
  FOR1D(fingers,i){
    fingers(i).tip.read_sensor_transfs("tip_sensor_frames.cfg");
    fingers(i).tip.add_sensor_shapes(ors);

    fingers(i).link.read_sensor_transfs("fing_sensor_frames.cfg");
    fingers(i).link.add_sensor_shapes(ors);
  }

  /* For convenience we define more verbose pointers to the fingers: thumb,
   * finger2 and finger 3.  If you look at SDH, fingers pointing to you,
   * according to current schunk.ors numbering is:
   * ___2
   * 1___
   * ___3
   */
  th = fingers.p+0;
  f2 = fingers.p+1;
  f3 = fingers.p+2;

}

/*
 * only one parameter should be non-NULL and determines the update method used!
 */
void
grobi_skin_t::sense(SchunkSkinModule *skin,const byteA *const img, const byteA *const map){

  uint i;
  FOR1D(fingers,i)
    if (skin){
      fingers(i).tip.senseskin(skin);
      fingers(i).link.senseskin(skin);
    }else if (img){
      fingers(i).tip.senseimg(*img);
      fingers(i).link.senseimg(*img);
    }else if (map){
      NIY("Don't use this.");
      /*
      fingers(i).tip.sensebypass(*map);
      fingers(i).link.sensebypass(*map);
      */
    }else HALT("one of three should be non-NULL");
}

void
grobi_skin_t::update_shapes(){

  uint i;
  FOR1D(fingers,i){
    fingers(i).tip.update_shapes();
    fingers(i).link.update_shapes();
  }
}


