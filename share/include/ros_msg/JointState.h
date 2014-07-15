/* Software License Agreement (BSD License)
 *
 * Copyright (c) 2011, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 *  * Neither the name of Willow Garage, Inc. nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * Auto-generated by genmsg_cpp from file /home/mtoussai/git/mlr/share/src/pr2/marc_controller_pkg/msg/JointState.msg
 *
 */


#ifndef MARC_CONTROLLER_PKG_MESSAGE_JOINTSTATE_H
#define MARC_CONTROLLER_PKG_MESSAGE_JOINTSTATE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace marc_controller_pkg
{
template <class ContainerAllocator>
struct JointState_
{
  typedef JointState_<ContainerAllocator> Type;

  JointState_()
    : q()
    , qdot()
    , fL()
    , fR()
    , u_bias()
    , Kq_gainFactor()
    , Kd_gainFactor()
    , Kf_gainFactor()  {
    }
  JointState_(const ContainerAllocator& _alloc)
    : q(_alloc)
    , qdot(_alloc)
    , fL(_alloc)
    , fR(_alloc)
    , u_bias(_alloc)
    , Kq_gainFactor(_alloc)
    , Kd_gainFactor(_alloc)
    , Kf_gainFactor(_alloc)  {
    }



   typedef std::vector<double, typename ContainerAllocator::template rebind<double>::other >  _q_type;
  _q_type q;

   typedef std::vector<double, typename ContainerAllocator::template rebind<double>::other >  _qdot_type;
  _qdot_type qdot;

   typedef std::vector<double, typename ContainerAllocator::template rebind<double>::other >  _fL_type;
  _fL_type fL;

   typedef std::vector<double, typename ContainerAllocator::template rebind<double>::other >  _fR_type;
  _fR_type fR;

   typedef std::vector<double, typename ContainerAllocator::template rebind<double>::other >  _u_bias_type;
  _u_bias_type u_bias;

   typedef std::vector<double, typename ContainerAllocator::template rebind<double>::other >  _Kq_gainFactor_type;
  _Kq_gainFactor_type Kq_gainFactor;

   typedef std::vector<double, typename ContainerAllocator::template rebind<double>::other >  _Kd_gainFactor_type;
  _Kd_gainFactor_type Kd_gainFactor;

   typedef std::vector<double, typename ContainerAllocator::template rebind<double>::other >  _Kf_gainFactor_type;
  _Kf_gainFactor_type Kf_gainFactor;




  typedef boost::shared_ptr< ::marc_controller_pkg::JointState_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::marc_controller_pkg::JointState_<ContainerAllocator> const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;

}; // struct JointState_

typedef ::marc_controller_pkg::JointState_<std::allocator<void> > JointState;

typedef boost::shared_ptr< ::marc_controller_pkg::JointState > JointStatePtr;
typedef boost::shared_ptr< ::marc_controller_pkg::JointState const> JointStateConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::marc_controller_pkg::JointState_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::marc_controller_pkg::JointState_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace marc_controller_pkg

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'geometry_msgs': ['/opt/ros/groovy/share/geometry_msgs/msg'], 'std_msgs': ['/opt/ros/groovy/share/std_msgs/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::marc_controller_pkg::JointState_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::marc_controller_pkg::JointState_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::marc_controller_pkg::JointState_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::marc_controller_pkg::JointState_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::marc_controller_pkg::JointState_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::marc_controller_pkg::JointState_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::marc_controller_pkg::JointState_<ContainerAllocator> >
{
  static const char* value()
  {
    return "d11554e6fcc1e2b2448fd4d8fdc39425";
  }

  static const char* value(const ::marc_controller_pkg::JointState_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xd11554e6fcc1e2b2ULL;
  static const uint64_t static_value2 = 0x448fd4d8fdc39425ULL;
};

template<class ContainerAllocator>
struct DataType< ::marc_controller_pkg::JointState_<ContainerAllocator> >
{
  static const char* value()
  {
    return "marc_controller_pkg/JointState";
  }

  static const char* value(const ::marc_controller_pkg::JointState_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::marc_controller_pkg::JointState_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float64[] q\n\
float64[] qdot\n\
float64[] fL\n\
float64[] fR\n\
float64[] u_bias\n\
float64[] Kq_gainFactor\n\
float64[] Kd_gainFactor\n\
float64[] Kf_gainFactor\n\
\n\
";
  }

  static const char* value(const ::marc_controller_pkg::JointState_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::marc_controller_pkg::JointState_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.q);
      stream.next(m.qdot);
      stream.next(m.fL);
      stream.next(m.fR);
      stream.next(m.u_bias);
      stream.next(m.Kq_gainFactor);
      stream.next(m.Kd_gainFactor);
      stream.next(m.Kf_gainFactor);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER;
  }; // struct JointState_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::marc_controller_pkg::JointState_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::marc_controller_pkg::JointState_<ContainerAllocator>& v)
  {
    s << indent << "q[]" << std::endl;
    for (size_t i = 0; i < v.q.size(); ++i)
    {
      s << indent << "  q[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.q[i]);
    }
    s << indent << "qdot[]" << std::endl;
    for (size_t i = 0; i < v.qdot.size(); ++i)
    {
      s << indent << "  qdot[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.qdot[i]);
    }
    s << indent << "fL[]" << std::endl;
    for (size_t i = 0; i < v.fL.size(); ++i)
    {
      s << indent << "  fL[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.fL[i]);
    }
    s << indent << "fR[]" << std::endl;
    for (size_t i = 0; i < v.fR.size(); ++i)
    {
      s << indent << "  fR[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.fR[i]);
    }
    s << indent << "u_bias[]" << std::endl;
    for (size_t i = 0; i < v.u_bias.size(); ++i)
    {
      s << indent << "  u_bias[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.u_bias[i]);
    }
    s << indent << "Kq_gainFactor[]" << std::endl;
    for (size_t i = 0; i < v.Kq_gainFactor.size(); ++i)
    {
      s << indent << "  Kq_gainFactor[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.Kq_gainFactor[i]);
    }
    s << indent << "Kd_gainFactor[]" << std::endl;
    for (size_t i = 0; i < v.Kd_gainFactor.size(); ++i)
    {
      s << indent << "  Kd_gainFactor[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.Kd_gainFactor[i]);
    }
    s << indent << "Kf_gainFactor[]" << std::endl;
    for (size_t i = 0; i < v.Kf_gainFactor.size(); ++i)
    {
      s << indent << "  Kf_gainFactor[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.Kf_gainFactor[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // MARC_CONTROLLER_PKG_MESSAGE_JOINTSTATE_H
