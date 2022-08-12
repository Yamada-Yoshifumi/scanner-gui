// Generated by gencpp from file ros_srv/VelodyneSwitchRequest.msg
// DO NOT EDIT!


#ifndef ROS_SRV_MESSAGE_VELODYNESWITCHREQUEST_H
#define ROS_SRV_MESSAGE_VELODYNESWITCHREQUEST_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace ros_srv
{
template <class ContainerAllocator>
struct VelodyneSwitchRequest_
{
  typedef VelodyneSwitchRequest_<ContainerAllocator> Type;

  VelodyneSwitchRequest_()
    : command(0)  {
    }
  VelodyneSwitchRequest_(const ContainerAllocator& _alloc)
    : command(0)  {
  (void)_alloc;
    }



   typedef int64_t _command_type;
  _command_type command;





  typedef boost::shared_ptr< ::ros_srv::VelodyneSwitchRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::ros_srv::VelodyneSwitchRequest_<ContainerAllocator> const> ConstPtr;

}; // struct VelodyneSwitchRequest_

typedef ::ros_srv::VelodyneSwitchRequest_<std::allocator<void> > VelodyneSwitchRequest;

typedef boost::shared_ptr< ::ros_srv::VelodyneSwitchRequest > VelodyneSwitchRequestPtr;
typedef boost::shared_ptr< ::ros_srv::VelodyneSwitchRequest const> VelodyneSwitchRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::ros_srv::VelodyneSwitchRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::ros_srv::VelodyneSwitchRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::ros_srv::VelodyneSwitchRequest_<ContainerAllocator1> & lhs, const ::ros_srv::VelodyneSwitchRequest_<ContainerAllocator2> & rhs)
{
  return lhs.command == rhs.command;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::ros_srv::VelodyneSwitchRequest_<ContainerAllocator1> & lhs, const ::ros_srv::VelodyneSwitchRequest_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace ros_srv

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::ros_srv::VelodyneSwitchRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ros_srv::VelodyneSwitchRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::ros_srv::VelodyneSwitchRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::ros_srv::VelodyneSwitchRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ros_srv::VelodyneSwitchRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ros_srv::VelodyneSwitchRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::ros_srv::VelodyneSwitchRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "856b30ed20a04e555abc88aa6a08af67";
  }

  static const char* value(const ::ros_srv::VelodyneSwitchRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x856b30ed20a04e55ULL;
  static const uint64_t static_value2 = 0x5abc88aa6a08af67ULL;
};

template<class ContainerAllocator>
struct DataType< ::ros_srv::VelodyneSwitchRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "ros_srv/VelodyneSwitchRequest";
  }

  static const char* value(const ::ros_srv::VelodyneSwitchRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::ros_srv::VelodyneSwitchRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "int64 command\n"
;
  }

  static const char* value(const ::ros_srv::VelodyneSwitchRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::ros_srv::VelodyneSwitchRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.command);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct VelodyneSwitchRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::ros_srv::VelodyneSwitchRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::ros_srv::VelodyneSwitchRequest_<ContainerAllocator>& v)
  {
    s << indent << "command: ";
    Printer<int64_t>::stream(s, indent + "  ", v.command);
  }
};

} // namespace message_operations
} // namespace ros

#endif // ROS_SRV_MESSAGE_VELODYNESWITCHREQUEST_H
