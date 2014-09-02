#include "module_G4Publisher.h"
#include <time.h>

REGISTER_MODULE(G4Publisher)

#ifdef HAVE_ROS_G4
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#endif

struct sG4Publisher{
	unsigned int max_sensors;
	double time_offset;
#ifdef HAVE_ROS_G4
	tf::TransformBroadcaster br;
#endif
	sG4Publisher() : max_sensors(3 * MT::getParameter<uint>("g4_numHubs")) {
		time_offset = time(NULL);
		time_offset -= (((time_t)time_offset)%86400);
	}
};

G4Publisher::G4Publisher() : Module("G4Publisher"), s(NULL) {
}

void G4Publisher::open(){
  s = new sG4Publisher;
}

void G4Publisher::close(){
	if(s) {
		delete s;
		s = NULL;
	}
}

void G4Publisher::step(){
#ifdef HAVE_ROS_G4
  uint t = poses.readAccess();
  floatA p = poses();
  double tstamp = poses.tstamp();
  poses.deAccess();

  ros::Time timestamp = ros::Time(tstamp + s->time_offset);

  for(unsigned int i = 0, len = p.d0; i < len; ++i) {
	  tf::Transform t;
	  stringstream name;
	  name << "g4_marker_" << i;

	  t.position.x = p(i, 0);
	  t.position.y = p(i, 1);
	  t.position.z = p(i, 2);
	  t.orientation.w = p(i, 3);
	  t.orientation.x = p(i, 4);
	  t.orientation.y = p(i, 5);
	  t.orientation.z = p(i, 6);
	  s->br.sendTransform(tf::StampedTransform(t, timestamp, "g4", name.str());
  }
#endif
}
