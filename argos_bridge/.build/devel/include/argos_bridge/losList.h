// Generated by gencpp from file argos_bridge/losList.msg
// DO NOT EDIT!


#ifndef ARGOS_BRIDGE_MESSAGE_LOSLIST_H
#define ARGOS_BRIDGE_MESSAGE_LOSLIST_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>
#include <argos_bridge/los.h>

namespace argos_bridge
{
template <class ContainerAllocator>
struct losList_
{
  typedef losList_<ContainerAllocator> Type;

  losList_()
    : header()
    , n(0)
    , robots()  {
    }
  losList_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , n(0)
    , robots(_alloc)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef int32_t _n_type;
  _n_type n;

   typedef std::vector< ::argos_bridge::los_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::argos_bridge::los_<ContainerAllocator> >::other >  _robots_type;
  _robots_type robots;





  typedef boost::shared_ptr< ::argos_bridge::losList_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::argos_bridge::losList_<ContainerAllocator> const> ConstPtr;

}; // struct losList_

typedef ::argos_bridge::losList_<std::allocator<void> > losList;

typedef boost::shared_ptr< ::argos_bridge::losList > losListPtr;
typedef boost::shared_ptr< ::argos_bridge::losList const> losListConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::argos_bridge::losList_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::argos_bridge::losList_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::argos_bridge::losList_<ContainerAllocator1> & lhs, const ::argos_bridge::losList_<ContainerAllocator2> & rhs)
{
  return lhs.header == rhs.header &&
    lhs.n == rhs.n &&
    lhs.robots == rhs.robots;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::argos_bridge::losList_<ContainerAllocator1> & lhs, const ::argos_bridge::losList_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace argos_bridge

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::argos_bridge::losList_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::argos_bridge::losList_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::argos_bridge::losList_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::argos_bridge::losList_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::argos_bridge::losList_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::argos_bridge::losList_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::argos_bridge::losList_<ContainerAllocator> >
{
  static const char* value()
  {
    return "3c2be5cd3cebc9db2ce990dc41ef741c";
  }

  static const char* value(const ::argos_bridge::losList_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x3c2be5cd3cebc9dbULL;
  static const uint64_t static_value2 = 0x2ce990dc41ef741cULL;
};

template<class ContainerAllocator>
struct DataType< ::argos_bridge::losList_<ContainerAllocator> >
{
  static const char* value()
  {
    return "argos_bridge/losList";
  }

  static const char* value(const ::argos_bridge::losList_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::argos_bridge::losList_<ContainerAllocator> >
{
  static const char* value()
  {
    return "Header header\n"
"int32 n\n"
"los[] robots\n"
"\n"
"================================================================================\n"
"MSG: std_msgs/Header\n"
"# Standard metadata for higher-level stamped data types.\n"
"# This is generally used to communicate timestamped data \n"
"# in a particular coordinate frame.\n"
"# \n"
"# sequence ID: consecutively increasing ID \n"
"uint32 seq\n"
"#Two-integer timestamp that is expressed as:\n"
"# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n"
"# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n"
"# time-handling sugar is provided by the client library\n"
"time stamp\n"
"#Frame this data is associated with\n"
"string frame_id\n"
"\n"
"================================================================================\n"
"MSG: argos_bridge/los\n"
"string robotName\n"
;
  }

  static const char* value(const ::argos_bridge::losList_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::argos_bridge::losList_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.n);
      stream.next(m.robots);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct losList_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::argos_bridge::losList_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::argos_bridge::losList_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "n: ";
    Printer<int32_t>::stream(s, indent + "  ", v.n);
    s << indent << "robots[]" << std::endl;
    for (size_t i = 0; i < v.robots.size(); ++i)
    {
      s << indent << "  robots[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::argos_bridge::los_<ContainerAllocator> >::stream(s, indent + "    ", v.robots[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // ARGOS_BRIDGE_MESSAGE_LOSLIST_H
