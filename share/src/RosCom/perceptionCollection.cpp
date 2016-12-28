#include <RosCom/roscom.h>
#include "perceptionCollection.h"

Collector::Collector(const bool simulate)
  : Thread("Collector", simulate ? 0.05:-1),
    tabletop_clusters(this, "tabletop_clusters", !simulate),
    ar_pose_markers(this, "ar_pose_markers", !simulate),
    opti_markers(this, "opti_markers", !simulate),
    opti_bodies(this, "opti_bodies", !simulate),
    tabletop_tableArray(this, "tabletop_tableArray", !simulate)
{
  tabletop_srcFrame.set()->setZero();
  alvar_srcFrame.set()->setZero();
  optitrack_srcFrame.set()->setZero();
  this->simulate = simulate;
}

void Collector::step()
{
  FilterObjects percepts;
  percepts.clear();

  if (!simulate)
  {

    int cluster_rev = tabletop_clusters.readAccess();
    if ( cluster_rev > tabletop_clusters_revision ){ //only if new cluster info is available
      tabletop_clusters_revision = cluster_rev;
      const visualization_msgs::MarkerArray msg = tabletop_clusters();

      if (msg.markers.size() > 0)
      {
        if (!has_tabletop_transform)
        {
          tf::TransformListener listener;
          mlr::Transformation tf;
          if (ros_getTransform("/base_footprint", msg.markers[0].header.frame_id, listener, tf))
          {
            tabletop_srcFrame.set() = tf;
            has_tabletop_transform = true;
          }
        }

#if 0
        if (!has_transform) { //get the transform from ROS
          //MT: markers and clusters have the same transformation??
          // Convert into a position relative to the base.
          tf::TransformListener listener;
          tf::StampedTransform baseTransform;
          try{
            mlr::Transformation tf = ros_getTransform("/base_footprint", msg.markers[0].header.frame_id, listener);

            //MT: really add the meter here? This seems hidden magic numbers in the code. And only for Baxter..?
            mlr::Transformation inv;
            inv.setInverse(tf);
            inv.addRelativeTranslation(0,0,-1);
            inv.setInverse(inv);
            tabletop_srcFrame.set() = inv;
            has_tabletop_transform = true;
          }
          catch (tf::TransformException &ex) {
              ROS_ERROR("%s",ex.what());
              ros::Duration(1.0).sleep();
              exit(0);
          }
        }
#endif

        for(auto & marker : msg.markers){
          Cluster* new_cluster = new Cluster(conv_ROSMarker2Cluster( marker ));
          new_cluster->frame = tabletop_srcFrame.get();
          percepts.append( new_cluster );
        }
      }
    }
    tabletop_clusters.deAccess();

    int tableArray_rev = tabletop_tableArray.readAccess();
    if ( tableArray_rev > tabletop_tableArray_revision ){ //only if new cluster info is available
      tabletop_tableArray_revision = tableArray_rev;
      const object_recognition_msgs::TableArray msg = tabletop_tableArray();

      if (msg.tables.size() > 0)
      {

        if (!has_tabletop_transform)
        {
          tf::TransformListener listener;
          mlr::Transformation tf;
          if (ros_getTransform("/base_footprint", msg.header.frame_id, listener, tf))
          {
            tabletop_srcFrame.set() = tf;
            has_tabletop_transform = true;
          }
        }

        for(auto & table : msg.tables){
          Plane* new_plane = new Plane(conv_ROSTable2Plane( table ));
          new_plane->frame = tabletop_srcFrame.get(); //tf
          percepts.append( new_plane );
        }
      }
    }
    tabletop_tableArray.deAccess();

    int ar_rev = ar_pose_markers.readAccess();
    if ( ar_rev > ar_pose_markers_revision){ //new alwar objects are available
      ar_pose_markers_revision = ar_rev;
      const ar::AlvarMarkers msg = ar_pose_markers();

      for(auto & marker : msg.markers) {
        if (!has_alvar_transform)
        {
          tf::TransformListener listener;
          mlr::Transformation tf;
          if (ros_getTransform("/base_footprint", msg.markers[0].header.frame_id, listener, tf))
          {
            alvar_srcFrame.set() = tf;
            has_alvar_transform = true;
          }
        }
#if 0
        if (!has_transform)
        {
          // Convert into a position relative to the base.
          tf::TransformListener listener;
          tf::StampedTransform baseTransform;
          try{
            listener.waitForTransform("/base", msg.markers[0].header.frame_id, ros::Time(0), ros::Duration(1.0));
            listener.lookupTransform("/base", msg.markers[0].header.frame_id, ros::Time(0), baseTransform);
            tf = conv_transform2transformation(baseTransform);
            mlr::Transformation inv;
            inv.setInverse(tf);
            inv.addRelativeTranslation(0,0,-1);
            inv.setInverse(inv);
            alvar_srcFrame.set() = inv;
            has_alvar_transform = true;
          }
          catch (tf::TransformException &ex) {
              ROS_ERROR("%s",ex.what());
              ros::Duration(1.0).sleep();
              exit(0);
          }
        }
#endif

        Alvar* new_alvar = new Alvar( conv_ROSAlvar2Alvar(marker) );
        new_alvar->frame = alvar_srcFrame.get();
        percepts.append( new_alvar );
      }
    }
    ar_pose_markers.deAccess(); //MT: make the access shorter, only around the blocks above
	
	int body_rev = opti_bodies.readAccess();
    if ( body_rev > opti_bodies_revision)
    {
      opti_bodies_revision = body_rev;
      const std::vector<geometry_msgs::TransformStamped> msgs = opti_bodies();
      for(uint i=0; i<msgs.size(); i++){
          geometry_msgs::TransformStamped msg = msgs.at(i);
#if 0
          if (!has_transform)
          {
           // Convert into a position relative to the base.
            tf::TransformListener listener;
            tf::StampedTransform baseTransform;
            try{
                listener.waitForTransform("/world", msg.header.frame_id, ros::Time(0), ros::Duration(1.0));
                listener.lookupTransform("/world", msg.header.frame_id, ros::Time(0), baseTransform);
                tf = conv_transform2transformation(baseTransform);
                mlr::Transformation inv;
                inv.setInverse(tf);
                inv.addRelativeTranslation(0,0,-1);
                inv.setInverse(inv);
                tf = inv;
                has_transform = true;
            }
            catch (tf::TransformException &ex) {
              ROS_ERROR("%s",ex.what());
              ros::Duration(1.0).sleep();
              exit(0);
            }
          }
#endif
          OptitrackBody* new_optitrack_body = new OptitrackBody(conv_tf2OptitrackBody( msg ));
          new_optitrack_body->frame = optitrack_srcFrame.get(); //tf
          percepts.append( new_optitrack_body );
        }
    }
	opti_bodies.deAccess();

	int marker_rev = opti_markers.readAccess();
    if ( marker_rev > opti_markers_revision)
    {
      opti_markers_revision = marker_rev;
      const std::vector<geometry_msgs::TransformStamped> msgs = opti_markers();

      for(uint i=0; i<msgs.size(); i++){
          geometry_msgs::TransformStamped msg = msgs.at(i);
#if 0
          if (!has_transform)
          {
           // Convert into a position relative to the base.
            tf::TransformListener listener;
            tf::StampedTransform baseTransform;
            try{
                listener.waitForTransform("/world", msg.header.frame_id, ros::Time(0), ros::Duration(1.0));
                listener.lookupTransform("/world", msg.header.frame_id, ros::Time(0), baseTransform);
                tf = conv_transform2transformation(baseTransform);
                mlr::Transformation inv;
                inv.setInverse(tf);
                inv.addRelativeTranslation(0,0,-1);
                inv.setInverse(inv);
                tf = inv;
                has_transform = true;
            }
            catch (tf::TransformException &ex) {
              ROS_ERROR("%s",ex.what());
              ros::Duration(1.0).sleep();
              exit(0);
            }
          }
#endif
          OptitrackMarker* new_optitrack_marker = new OptitrackMarker(conv_tf2OptitrackMarker( msg ));
          new_optitrack_marker->frame = optitrack_srcFrame.get(); //tf;
          percepts.append( new_optitrack_marker );
        }
    }
	opti_markers.deAccess();

  }
  else // If 'simulate', make a fake cluster and alvar
  {
    mlr::Mesh box;
    box.setBox();
    box.subDivide();
    box.subDivide();
    box.subDivide();
    rndUniform(box.V, -0.05, 0.05, true);
    box.scale(0.1);

    Cluster* fake_cluster = new Cluster( ARR(0.6, 0., 0.05),  // mean
                                         box.V,               // points
                                         "/base_footprint");  // frame
    fake_cluster->frame.setZero();
    fake_cluster->frame.addRelativeTranslation(0.6, 0., 1.05);
    mlr::Quaternion rot;

//    int tick = perceptual_inputs.readAccess();
//    perceptual_inputs.deAccess();
//    cout << "tick: " << tick << endl;
//    rot.setDeg(0.01 * tick, mlr::Vector(0.1, 0.25, 1));

    rot.setDeg(30, mlr::Vector(0.1, 0.25, 1));
    fake_cluster->frame.addRelativeRotation(rot);
    percepts.append( fake_cluster );

    Alvar* fake_alvar = new Alvar("/base_footprint");
    fake_alvar->frame.setZero();

    arr pos = { 0.9, 0.3, 1.5 };
    rndUniform(pos, -0.005, 0.005, true);
    fake_alvar->frame.addRelativeTranslation(pos(0), pos(1), pos(2));

    arr alv_rot = { MLR_PI/2, 0, -MLR_PI/2};
    rndUniform(alv_rot, -0.01, 0.01, true);
    rot.setRpy(alv_rot(0), alv_rot(1), alv_rot(2));
    fake_alvar->frame.addRelativeRotation(rot);
    fake_alvar->id = 10;
    percepts.append( fake_alvar );
    mlr::wait(0.01);
  }

  if (percepts.N > 0){
    perceptual_inputs.set() = percepts;
  }
}


Cluster conv_ROSMarker2Cluster(const visualization_msgs::Marker& marker)
{
  arr points = conv_points2arr(marker.points);
  arr mean = sum(points,0)/(double)points.d0;
  return Cluster(mean, points, marker.header.frame_id);
}

Plane conv_ROSTable2Plane(const object_recognition_msgs::Table& table){
  mlr::Transformation t = conv_pose2transformation(table.pose);
  arr hull = conv_points2arr(table.convex_hull);
  arr center = ARR(t.pos.x, t.pos.y, t.pos.z);
  mlr::Vector norm = t.rot*mlr::Vector(0,0,1);

  arr normal = ARR(norm.x, norm.y, norm.z);
  Plane toReturn = Plane(normal, center, hull, table.header.frame_id);
  toReturn.transform = t;
  return toReturn;
}

Alvar conv_ROSAlvar2Alvar(const ar::AlvarMarker& marker)
{
  Alvar new_alvar(marker.header.frame_id);
  new_alvar.id = marker.id;
  new_alvar.transform = conv_pose2transformation(marker.pose.pose);
  return new_alvar;
}

OptitrackMarker conv_tf2OptitrackMarker(const geometry_msgs::TransformStamped& msg)
{
  OptitrackMarker new_optitrackmarker(msg.header.frame_id);
//  new_optitrackmarker.id = marker.id;
  new_optitrackmarker.transform = conv_transform2transformation(msg.transform);
  return new_optitrackmarker;
}

OptitrackBody conv_tf2OptitrackBody(const geometry_msgs::TransformStamped& msg)
{
  OptitrackBody new_optitrackbody(msg.header.frame_id);
//  new_alvar.id = body.id;
  new_optitrackbody.transform = conv_transform2transformation(msg.transform);
  return new_optitrackbody;
}


