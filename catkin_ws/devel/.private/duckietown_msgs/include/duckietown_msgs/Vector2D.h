// Generated by gencpp from file duckietown_msgs/Vector2D.msg
// DO NOT EDIT!


#ifndef DUCKIETOWN_MSGS_MESSAGE_VECTOR2D_H
#define DUCKIETOWN_MSGS_MESSAGE_VECTOR2D_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace duckietown_msgs
{
template <class ContainerAllocator>
struct Vector2D_
{
  typedef Vector2D_<ContainerAllocator> Type;

  Vector2D_()
    : x(0.0)
    , y(0.0)  {
    }
  Vector2D_(const ContainerAllocator& _alloc)
    : x(0.0)
    , y(0.0)  {
  (void)_alloc;
    }



   typedef float _x_type;
  _x_type x;

   typedef float _y_type;
  _y_type y;





  typedef boost::shared_ptr< ::duckietown_msgs::Vector2D_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::duckietown_msgs::Vector2D_<ContainerAllocator> const> ConstPtr;

}; // struct Vector2D_

typedef ::duckietown_msgs::Vector2D_<std::allocator<void> > Vector2D;

typedef boost::shared_ptr< ::duckietown_msgs::Vector2D > Vector2DPtr;
typedef boost::shared_ptr< ::duckietown_msgs::Vector2D const> Vector2DConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::duckietown_msgs::Vector2D_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::duckietown_msgs::Vector2D_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace duckietown_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'sensor_msgs': ['/opt/ros/melodic/share/sensor_msgs/cmake/../msg'], 'duckietown_msgs': ['/duckietown/catkin_ws/src/dt-ros-commons/packages/duckietown_msgs/msg'], 'std_msgs': ['/opt/ros/melodic/share/std_msgs/cmake/../msg'], 'geometry_msgs': ['/opt/ros/melodic/share/geometry_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::duckietown_msgs::Vector2D_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::duckietown_msgs::Vector2D_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::duckietown_msgs::Vector2D_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::duckietown_msgs::Vector2D_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::duckietown_msgs::Vector2D_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::duckietown_msgs::Vector2D_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::duckietown_msgs::Vector2D_<ContainerAllocator> >
{
  static const char* value()
  {
    return "ff8d7d66dd3e4b731ef14a45d38888b6";
  }

  static const char* value(const ::duckietown_msgs::Vector2D_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xff8d7d66dd3e4b73ULL;
  static const uint64_t static_value2 = 0x1ef14a45d38888b6ULL;
};

template<class ContainerAllocator>
struct DataType< ::duckietown_msgs::Vector2D_<ContainerAllocator> >
{
  static const char* value()
  {
    return "duckietown_msgs/Vector2D";
  }

  static const char* value(const ::duckietown_msgs::Vector2D_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::duckietown_msgs::Vector2D_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float32 x\n"
"float32 y\n"
;
  }

  static const char* value(const ::duckietown_msgs::Vector2D_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::duckietown_msgs::Vector2D_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.x);
      stream.next(m.y);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct Vector2D_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::duckietown_msgs::Vector2D_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::duckietown_msgs::Vector2D_<ContainerAllocator>& v)
  {
    s << indent << "x: ";
    Printer<float>::stream(s, indent + "  ", v.x);
    s << indent << "y: ";
    Printer<float>::stream(s, indent + "  ", v.y);
  }
};

} // namespace message_operations
} // namespace ros

#endif // DUCKIETOWN_MSGS_MESSAGE_VECTOR2D_H
