#include <pr2/roscom.h>
#include <pr2/perceptionCollection.h>

Collector::Collector():
    Module("Collector", 0){}

void Collector::step()
{
  tabletop_clusters.waitForNextRevision();

  const visualization_msgs::MarkerArray msg = tabletop_clusters.get();

  FilterObjects clusters;
  for(auto & marker : msg.markers){
    clusters.append(conv_Marker2FilterObject( marker ));
  }

  perceptual_inputs.set() = clusters;
}


FilterObject conv_Marker2FilterObject(const visualization_msgs::Marker& marker)
{
  arr pts = conv_points2arr(marker.points);
  arr mean = sum(pts,0)/(double)pts.d0;
  // Put it into our list

  FilterObject new_object;
  new_object.Cluster::mean = mean;
  //std::cout << "Mean: " << mean(0) << ' ' << mean(1) << ' ' << mean(2) << std::endl;

  new_object.Cluster::points = pts;
  new_object.id = -1;
  new_object.relevance = 1;
  new_object.Cluster::frame_id = marker.header.frame_id;
  new_object.type = FilterObject::FilterObjectType::cluster;
  return new_object;
}
