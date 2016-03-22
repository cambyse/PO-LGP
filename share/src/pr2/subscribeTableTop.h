#include "roscom.h"

struct SubscribeTableTop{
  Access_typed<visualization_msgs::MarkerArray> tableTopObjects;
  Subscriber<visualization_msgs::MarkerArray> sub;

  SubscribeTableTop()
    : tableTopObjects(this, "tableTopObjects"),
      sub("/tabletop/rs_fitted", tableTopObjects) {
  }
  ~SubscribeTableTop(){
  }

};
