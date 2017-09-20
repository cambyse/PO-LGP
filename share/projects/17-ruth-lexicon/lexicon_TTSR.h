#ifndef LEXICON_TTSR_H
#define LEXICON_TTSR_H

#include <Roopi/roopi.h>
#include <Geo/geo.h>
#include <string>
#include <vector>
#include <math.h>
#include <HRI_state.h>

struct xyIndex {
  int x;
  int y;
};

struct lexicon {
  //dimensions of table
  double tableSize[2];
  double objectSize;
  //neighbourhood size
  double neighbourhoodSize;
  //lexicon table with (10x10)x5 concept elements
  int dltTTSR[100][5];
  void init_lexicon();
  void update_lexicon(Roopi& R, mlr::Vector pos, int word);
  void load_lexicon(mlr::String filename);
  void save_lexicon(mlr::String filename);
  int choose_object(Roopi& R, int word, std::vector<HRIObject*> objects);
  double calculate_confidence(int word, mlr::Vector pos);
  xyIndex table_position_to_index(mlr::Vector tpos);
  mlr::Vector index_to_table_position(xyIndex index);
  mlr::Vector robot_to_raw_table_position(Roopi& R, mlr::Vector rpos);
  mlr::Vector raw_table_to_robot_position(Roopi& R, mlr::Vector tpos);
  mlr::Vector raw_table_position_to_table_position(mlr::Vector rtpos);
  mlr::Vector table_position_to_raw_table_position(mlr::Vector tpos);
  mlr::Vector choose_position(Roopi& R, int word);

};


class lexicon_TTSR
{
public:
  lexicon_TTSR();
};

#endif // LEXICON_TTSR_H
