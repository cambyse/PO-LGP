BASE   = ../../..
-include $(BASE)/build/config.mk

MSGFILES = ObjId.msg MarkerArray.msg
ROSDIR = /opt/ros/$(ROS_DISTRO)
OUTPUTDIR = $(BASE)/include/ros_msg
PKGNAME = obj_id_pkg


default: $(MSGFILES)

%.msg:
	python2 $(ROSDIR)/lib/gencpp/gen_cpp.py \
    	msg/$*.msg \
        -p $(PKGNAME) \
        -e $(ROSDIR)/share/gencpp \
        -I "std_msgs:$(ROSDIR)/share/std_msgs/msg" \
        -I "geometry_msgs:$(ROSDIR)/share/geometry_msgs/msg" \
        -I "visualization_msgs:$(ROSDIR)/share/visualization_msgs/msg" \
        -o $(OUTPUTDIR)
