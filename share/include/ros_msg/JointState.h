// Generated by gencpp from file marc_controller_pkg/JointState.msg
// DO NOT EDIT!


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
    , J_ft_invL()
    , J_ft_invR()
    , Kp()
    , Kd()
    , Ki()
    , KiFTL()
    , KiFTR()
    , ft_offsetL()
    , ft_offsetR()
    , velLimitRatio(0.0)
    , effLimitRatio(0.0)
    , intLimitRatio(0.0)
    , gamma(0.0)
    , qd_filt(0.0)  {
    }
  JointState_(const ContainerAllocator& _alloc)
    : q(_alloc)
    , qdot(_alloc)
    , fL(_alloc)
    , fR(_alloc)
    , u_bias(_alloc)
    , J_ft_invL(_alloc)
    , J_ft_invR(_alloc)
    , Kp(_alloc)
    , Kd(_alloc)
    , Ki(_alloc)
    , KiFTL(_alloc)
    , KiFTR(_alloc)
    , ft_offsetL(_alloc)
    , ft_offsetR(_alloc)
    , velLimitRatio(0.0)
    , effLimitRatio(0.0)
    , intLimitRatio(0.0)
    , gamma(0.0)
    , qd_filt(0.0)  {
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

   typedef std::vector<double, typename ContainerAllocator::template rebind<double>::other >  _J_ft_invL_type;
  _J_ft_invL_type J_ft_invL;

   typedef std::vector<double, typename ContainerAllocator::template rebind<double>::other >  _J_ft_invR_type;
  _J_ft_invR_type J_ft_invR;

   typedef std::vector<double, typename ContainerAllocator::template rebind<double>::other >  _Kp_type;
  _Kp_type Kp;

   typedef std::vector<double, typename ContainerAllocator::template rebind<double>::other >  _Kd_type;
  _Kd_type Kd;

   typedef std::vector<double, typename ContainerAllocator::template rebind<double>::other >  _Ki_type;
  _Ki_type Ki;

   typedef std::vector<double, typename ContainerAllocator::template rebind<double>::other >  _KiFTL_type;
  _KiFTL_type KiFTL;

   typedef std::vector<double, typename ContainerAllocator::template rebind<double>::other >  _KiFTR_type;
  _KiFTR_type KiFTR;

   typedef std::vector<double, typename ContainerAllocator::template rebind<double>::other >  _ft_offsetL_type;
  _ft_offsetL_type ft_offsetL;

   typedef std::vector<double, typename ContainerAllocator::template rebind<double>::other >  _ft_offsetR_type;
  _ft_offsetR_type ft_offsetR;

   typedef double _velLimitRatio_type;
  _velLimitRatio_type velLimitRatio;

   typedef double _effLimitRatio_type;
  _effLimitRatio_type effLimitRatio;

   typedef double _intLimitRatio_type;
  _intLimitRatio_type intLimitRatio;

   typedef double _gamma_type;
  _gamma_type gamma;

   typedef double _qd_filt_type;
  _qd_filt_type qd_filt;




  typedef boost::shared_ptr< ::marc_controller_pkg::JointState_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::marc_controller_pkg::JointState_<ContainerAllocator> const> ConstPtr;

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
// {'geometry_msgs': ['/opt/ros/indigo/share/geometry_msgs/msg'], 'std_msgs': ['/opt/ros/indigo/share/std_msgs/msg']}

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
    return "79df038e604035d93660c71ee6024907";
  }

  static const char* value(const ::marc_controller_pkg::JointState_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x79df038e604035d9ULL;
  static const uint64_t static_value2 = 0x3660c71ee6024907ULL;
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
float64[] J_ft_invL\n\
float64[] J_ft_invR\n\
float64[] Kp\n\
float64[] Kd\n\
float64[] Ki\n\
float64[] KiFTL\n\
float64[] KiFTR\n\
float64[] ft_offsetL\n\
float64[] ft_offsetR\n\
float64 velLimitRatio\n\
float64 effLimitRatio\n\
float64 intLimitRatio\n\
float64 gamma\n\
float64 qd_filt\n\
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
      stream.next(m.J_ft_invL);
      stream.next(m.J_ft_invR);
      stream.next(m.Kp);
      stream.next(m.Kd);
      stream.next(m.Ki);
      stream.next(m.KiFTL);
      stream.next(m.KiFTR);
      stream.next(m.ft_offsetL);
      stream.next(m.ft_offsetR);
      stream.next(m.velLimitRatio);
      stream.next(m.effLimitRatio);
      stream.next(m.intLimitRatio);
      stream.next(m.gamma);
      stream.next(m.qd_filt);
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
    s << indent << "J_ft_invL[]" << std::endl;
    for (size_t i = 0; i < v.J_ft_invL.size(); ++i)
    {
      s << indent << "  J_ft_invL[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.J_ft_invL[i]);
    }
    s << indent << "J_ft_invR[]" << std::endl;
    for (size_t i = 0; i < v.J_ft_invR.size(); ++i)
    {
      s << indent << "  J_ft_invR[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.J_ft_invR[i]);
    }
    s << indent << "Kp[]" << std::endl;
    for (size_t i = 0; i < v.Kp.size(); ++i)
    {
      s << indent << "  Kp[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.Kp[i]);
    }
    s << indent << "Kd[]" << std::endl;
    for (size_t i = 0; i < v.Kd.size(); ++i)
    {
      s << indent << "  Kd[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.Kd[i]);
    }
    s << indent << "Ki[]" << std::endl;
    for (size_t i = 0; i < v.Ki.size(); ++i)
    {
      s << indent << "  Ki[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.Ki[i]);
    }
    s << indent << "KiFTL[]" << std::endl;
    for (size_t i = 0; i < v.KiFTL.size(); ++i)
    {
      s << indent << "  KiFTL[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.KiFTL[i]);
    }
    s << indent << "KiFTR[]" << std::endl;
    for (size_t i = 0; i < v.KiFTR.size(); ++i)
    {
      s << indent << "  KiFTR[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.KiFTR[i]);
    }
    s << indent << "ft_offsetL[]" << std::endl;
    for (size_t i = 0; i < v.ft_offsetL.size(); ++i)
    {
      s << indent << "  ft_offsetL[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.ft_offsetL[i]);
    }
    s << indent << "ft_offsetR[]" << std::endl;
    for (size_t i = 0; i < v.ft_offsetR.size(); ++i)
    {
      s << indent << "  ft_offsetR[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.ft_offsetR[i]);
    }
    s << indent << "velLimitRatio: ";
    Printer<double>::stream(s, indent + "  ", v.velLimitRatio);
    s << indent << "effLimitRatio: ";
    Printer<double>::stream(s, indent + "  ", v.effLimitRatio);
    s << indent << "intLimitRatio: ";
    Printer<double>::stream(s, indent + "  ", v.intLimitRatio);
    s << indent << "gamma: ";
    Printer<double>::stream(s, indent + "  ", v.gamma);
    s << indent << "qd_filt: ";
    Printer<double>::stream(s, indent + "  ", v.qd_filt);
  }
};

} // namespace message_operations
} // namespace ros

#endif // MARC_CONTROLLER_PKG_MESSAGE_JOINTSTATE_H
