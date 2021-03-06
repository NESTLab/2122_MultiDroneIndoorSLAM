// Generated by gencpp from file argos_bridge/los.msg
// DO NOT EDIT!


#ifndef ARGOS_BRIDGE_MESSAGE_LOS_H
#define ARGOS_BRIDGE_MESSAGE_LOS_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace argos_bridge
{
template <class ContainerAllocator>
struct los_
{
  typedef los_<ContainerAllocator> Type;

  los_()
    : robotName()  {
    }
  los_(const ContainerAllocator& _alloc)
    : robotName(_alloc)  {
  (void)_alloc;
    }



   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _robotName_type;
  _robotName_type robotName;





  typedef boost::shared_ptr< ::argos_bridge::los_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::argos_bridge::los_<ContainerAllocator> const> ConstPtr;

}; // struct los_

typedef ::argos_bridge::los_<std::allocator<void> > los;

typedef boost::shared_ptr< ::argos_bridge::los > losPtr;
typedef boost::shared_ptr< ::argos_bridge::los const> losConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::argos_bridge::los_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::argos_bridge::los_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::argos_bridge::los_<ContainerAllocator1> & lhs, const ::argos_bridge::los_<ContainerAllocator2> & rhs)
{
  return lhs.robotName == rhs.robotName;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::argos_bridge::los_<ContainerAllocator1> & lhs, const ::argos_bridge::los_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace argos_bridge

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::argos_bridge::los_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::argos_bridge::los_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::argos_bridge::los_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::argos_bridge::los_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::argos_bridge::los_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::argos_bridge::los_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::argos_bridge::los_<ContainerAllocator> >
{
  static const char* value()
  {
    return "0e4ce7af4736710e228ed1cbe6f009e7";
  }

  static const char* value(const ::argos_bridge::los_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x0e4ce7af4736710eULL;
  static const uint64_t static_value2 = 0x228ed1cbe6f009e7ULL;
};

template<class ContainerAllocator>
struct DataType< ::argos_bridge::los_<ContainerAllocator> >
{
  static const char* value()
  {
    return "argos_bridge/los";
  }

  static const char* value(const ::argos_bridge::los_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::argos_bridge::los_<ContainerAllocator> >
{
  static const char* value()
  {
    return "string robotName\n"
;
  }

  static const char* value(const ::argos_bridge::los_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::argos_bridge::los_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.robotName);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct los_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::argos_bridge::los_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::argos_bridge::los_<ContainerAllocator>& v)
  {
    s << indent << "robotName: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.robotName);
  }
};

} // namespace message_operations
} // namespace ros

#endif // ARGOS_BRIDGE_MESSAGE_LOS_H
