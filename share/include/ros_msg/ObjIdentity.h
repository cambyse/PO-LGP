// Generated by gencpp from file obj_id_pkg/ObjIdentity.msg
// DO NOT EDIT!


#ifndef OBJ_ID_PKG_MESSAGE_OBJIDENTITY_H
#define OBJ_ID_PKG_MESSAGE_OBJIDENTITY_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>

namespace obj_id_pkg
{
template <class ContainerAllocator>
struct ObjIdentity_
{
  typedef ObjIdentity_<ContainerAllocator> Type;

  ObjIdentity_()
    : header()
    , obj_id(0)  {
    }
  ObjIdentity_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , obj_id(0)  {
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef uint8_t _obj_id_type;
  _obj_id_type obj_id;




  typedef boost::shared_ptr< ::obj_id_pkg::ObjIdentity_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::obj_id_pkg::ObjIdentity_<ContainerAllocator> const> ConstPtr;

}; // struct ObjIdentity_

typedef ::obj_id_pkg::ObjIdentity_<std::allocator<void> > ObjIdentity;

typedef boost::shared_ptr< ::obj_id_pkg::ObjIdentity > ObjIdentityPtr;
typedef boost::shared_ptr< ::obj_id_pkg::ObjIdentity const> ObjIdentityConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::obj_id_pkg::ObjIdentity_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::obj_id_pkg::ObjIdentity_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace obj_id_pkg

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': True}
// {'std_msgs': ['/opt/ros/indigo/share/std_msgs/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::obj_id_pkg::ObjIdentity_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::obj_id_pkg::ObjIdentity_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::obj_id_pkg::ObjIdentity_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::obj_id_pkg::ObjIdentity_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::obj_id_pkg::ObjIdentity_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::obj_id_pkg::ObjIdentity_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::obj_id_pkg::ObjIdentity_<ContainerAllocator> >
{
  static const char* value()
  {
    return "4c857525cce71c3f010b8da5d1f74607";
  }

  static const char* value(const ::obj_id_pkg::ObjIdentity_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x4c857525cce71c3fULL;
  static const uint64_t static_value2 = 0x010b8da5d1f74607ULL;
};

template<class ContainerAllocator>
struct DataType< ::obj_id_pkg::ObjIdentity_<ContainerAllocator> >
{
  static const char* value()
  {
    return "obj_id_pkg/ObjIdentity";
  }

  static const char* value(const ::obj_id_pkg::ObjIdentity_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::obj_id_pkg::ObjIdentity_<ContainerAllocator> >
{
  static const char* value()
  {
    return "Header header\n\
uint8 obj_id\n\
\n\
================================================================================\n\
MSG: std_msgs/Header\n\
# Standard metadata for higher-level stamped data types.\n\
# This is generally used to communicate timestamped data \n\
# in a particular coordinate frame.\n\
# \n\
# sequence ID: consecutively increasing ID \n\
uint32 seq\n\
#Two-integer timestamp that is expressed as:\n\
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n\
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n\
# time-handling sugar is provided by the client library\n\
time stamp\n\
#Frame this data is associated with\n\
# 0: no frame\n\
# 1: global frame\n\
string frame_id\n\
";
  }

  static const char* value(const ::obj_id_pkg::ObjIdentity_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::obj_id_pkg::ObjIdentity_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.obj_id);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER;
  }; // struct ObjIdentity_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::obj_id_pkg::ObjIdentity_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::obj_id_pkg::ObjIdentity_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "obj_id: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.obj_id);
  }
};

} // namespace message_operations
} // namespace ros

#endif // OBJ_ID_PKG_MESSAGE_OBJIDENTITY_H
