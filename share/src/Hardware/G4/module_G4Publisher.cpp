#include "module_G4Publisher.h"
#include <time.h>
#include <Core/util.h>

REGISTER_MODULE(G4Publisher)

#ifdef HAVE_ROS_G4
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#endif

struct sG4Publisher{
	double time_offset;
	static bool initialized;
	std::string frame_id;
#ifdef HAVE_ROS_G4
	tf::TransformBroadcaster *br;
#endif

	sG4Publisher()  {
		time_offset = time(NULL);
		time_offset -= (((time_t)time_offset)%86400);
		
#ifdef HAVE_ROS_G4
		frame_id = MT::getParameter<MT::String>("g4_pub_frame", MT::String("world")).p;
		if(!initialized) {
			ros::init(MT::argc, MT::argv, "g4_publisher", ros::init::NoSigIntHandler);
			initialized = true;
		}
		br = new tf::TransformBroadcaster;
#endif
	}

	~sG4Publisher() {
#ifdef HAVE_ROS_G4
	delete br;
	ros::shutdown();
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
  poses.readAccess();
  floatA p = poses();
  double tstamp = poses.tstamp();
  poses.deAccess();

  ros::Time timestamp = ros::Time(tstamp + s->time_offset);

  for(unsigned int i = 0, len = p.d0; i < len; ++i) {
	  tf::Transform t;
	  std::stringstream name;
	  name << "g4_marker_" << i;

	  t.setOrigin(tf::Vector3(p(i, 0), p(i, 1), p(i, 2)));
	  t.setRotation(tf::Quaternion(p(i, 4), p(i, 5), p(i, 6), p(i, 3)));

	  s->br->sendTransform(tf::StampedTransform(t, timestamp, s->frame_id, 
		name.str()));
  }

  ros::spinOnce();
#endif
}
