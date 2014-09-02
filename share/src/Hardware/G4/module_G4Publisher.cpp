#include "module_G4Publisher.h"
#include <time.h>
#include <Core/util.h>

REGISTER_MODULE(G4Publisher)

#ifdef HAVE_ROS_G4
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#endif

struct sG4Publisher{
	unsigned int max_sensors;
	double time_offset;
	static bool initialized;
#ifdef HAVE_ROS_G4
	tf::TransformBroadcaster br;
#endif
	sG4Publisher() : max_sensors(3 * MT::getParameter<uint>("g4_numHubs")) {
		time_offset = time(NULL);
		time_offset -= (((time_t)time_offset)%86400);

#ifdef HAVE_ROS_G4
		if(!initialized) {
			ros::init(MT::argc, MT::argv, "g4_publisher");
			initialized = true;
		}
#endif
	}
};

bool sG4Publisher::initialized = false;

G4Publisher::G4Publisher() : Module("G4Publisher"), s(NULL) {
}

void G4Publisher::open() {
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
  uint rev = poses.readAccess();
  floatA p = poses();
  double tstamp = poses.tstamp();
  poses.deAccess();

  ros::spinOnce();

  ros::Time timestamp = ros::Time(tstamp + s->time_offset);

  for(unsigned int i = 0, len = p.d0; i < len; ++i) {
	  tf::Transform t;
	  std::stringstream name;
	  name << "g4_marker_" << i;

	  t.setOrigin(tf::Vector3(p(i, 0), p(i, 1), p(i, 2)));
	  tf::Quaternion q(p(i, 4), p(i, 5), p(i, 6), p(i, 3));
	  t.setRotation(q);

	  s->br.sendTransform(tf::StampedTransform(t, timestamp, "g4", 
		name.str()));
  }
#endif
}
