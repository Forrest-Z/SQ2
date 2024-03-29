/* Auto-generated by genmsg_cpp for file /home/amsl/AMSL_ros_pkg/rwrc15/trajectory_generation/msg/RoadProperty.msg */
#ifndef TRAJECTORY_GENERATION_MESSAGE_ROADPROPERTY_H
#define TRAJECTORY_GENERATION_MESSAGE_ROADPROPERTY_H
#include <string>
#include <vector>
#include <map>
#include <ostream>
#include "ros/serialization.h"
#include "ros/builtin_message_traits.h"
#include "ros/message_operations.h"
#include "ros/time.h"

#include "ros/macros.h"

#include "ros/assert.h"

#include "std_msgs/Header.h"

namespace trajectory_generation
{
template <class ContainerAllocator>
struct RoadProperty_ {
  typedef RoadProperty_<ContainerAllocator> Type;

  RoadProperty_()
  : header()
  , y(0.0)
  , phi(0.0)
  , c0(0.0)
  , c1(0.0)
  , w(0.0)
  , curvature(0.0)
  , inter_angle(0.0)
  {
  }

  RoadProperty_(const ContainerAllocator& _alloc)
  : header(_alloc)
  , y(0.0)
  , phi(0.0)
  , c0(0.0)
  , c1(0.0)
  , w(0.0)
  , curvature(0.0)
  , inter_angle(0.0)
  {
  }

  typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
   ::std_msgs::Header_<ContainerAllocator>  header;

  typedef float _y_type;
  float y;

  typedef float _phi_type;
  float phi;

  typedef float _c0_type;
  float c0;

  typedef float _c1_type;
  float c1;

  typedef float _w_type;
  float w;

  typedef float _curvature_type;
  float curvature;

  typedef float _inter_angle_type;
  float inter_angle;


  typedef boost::shared_ptr< ::trajectory_generation::RoadProperty_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::trajectory_generation::RoadProperty_<ContainerAllocator>  const> ConstPtr;
}; // struct RoadProperty
typedef  ::trajectory_generation::RoadProperty_<std::allocator<void> > RoadProperty;

typedef boost::shared_ptr< ::trajectory_generation::RoadProperty> RoadPropertyPtr;
typedef boost::shared_ptr< ::trajectory_generation::RoadProperty const> RoadPropertyConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::trajectory_generation::RoadProperty_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::trajectory_generation::RoadProperty_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace trajectory_generation

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::trajectory_generation::RoadProperty_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::trajectory_generation::RoadProperty_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::trajectory_generation::RoadProperty_<ContainerAllocator> > {
  static const char* value() 
  {
    return "fa95af025608c91e7baa5bd33f3a2197";
  }

  static const char* value(const  ::trajectory_generation::RoadProperty_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0xfa95af025608c91eULL;
  static const uint64_t static_value2 = 0x7baa5bd33f3a2197ULL;
};

template<class ContainerAllocator>
struct DataType< ::trajectory_generation::RoadProperty_<ContainerAllocator> > {
  static const char* value() 
  {
    return "trajectory_generation/RoadProperty";
  }

  static const char* value(const  ::trajectory_generation::RoadProperty_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::trajectory_generation::RoadProperty_<ContainerAllocator> > {
  static const char* value() 
  {
    return "Header header\n\
float32 y\n\
float32 phi \n\
float32 c0 \n\
float32 c1\n\
float32 w\n\
float32 curvature\n\
float32 inter_angle\n\
#nav_msgs/Path path\n\
\n\
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
\n\
";
  }

  static const char* value(const  ::trajectory_generation::RoadProperty_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct HasHeader< ::trajectory_generation::RoadProperty_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct HasHeader< const ::trajectory_generation::RoadProperty_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::trajectory_generation::RoadProperty_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.header);
    stream.next(m.y);
    stream.next(m.phi);
    stream.next(m.c0);
    stream.next(m.c1);
    stream.next(m.w);
    stream.next(m.curvature);
    stream.next(m.inter_angle);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct RoadProperty_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::trajectory_generation::RoadProperty_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::trajectory_generation::RoadProperty_<ContainerAllocator> & v) 
  {
    s << indent << "header: ";
s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "y: ";
    Printer<float>::stream(s, indent + "  ", v.y);
    s << indent << "phi: ";
    Printer<float>::stream(s, indent + "  ", v.phi);
    s << indent << "c0: ";
    Printer<float>::stream(s, indent + "  ", v.c0);
    s << indent << "c1: ";
    Printer<float>::stream(s, indent + "  ", v.c1);
    s << indent << "w: ";
    Printer<float>::stream(s, indent + "  ", v.w);
    s << indent << "curvature: ";
    Printer<float>::stream(s, indent + "  ", v.curvature);
    s << indent << "inter_angle: ";
    Printer<float>::stream(s, indent + "  ", v.inter_angle);
  }
};


} // namespace message_operations
} // namespace ros

#endif // TRAJECTORY_GENERATION_MESSAGE_ROADPROPERTY_H

