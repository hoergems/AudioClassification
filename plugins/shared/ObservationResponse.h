// Generated by gencpp from file ObservationService/ObservationResponse.msg
// DO NOT EDIT!


#ifndef OBSERVATIONSERVICE_MESSAGE_OBSERVATIONRESPONSE_H
#define OBSERVATIONSERVICE_MESSAGE_OBSERVATIONRESPONSE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace ObservationService
{
template <class ContainerAllocator>
struct ObservationResponse_
{
  typedef ObservationResponse_<ContainerAllocator> Type;

  ObservationResponse_()
    : observation(0)  {
    }
  ObservationResponse_(const ContainerAllocator& _alloc)
    : observation(0)  {
  (void)_alloc;
    }



   typedef int64_t _observation_type;
  _observation_type observation;





  typedef boost::shared_ptr< ::ObservationService::ObservationResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::ObservationService::ObservationResponse_<ContainerAllocator> const> ConstPtr;

}; // struct ObservationResponse_

typedef ::ObservationService::ObservationResponse_<std::allocator<void> > ObservationResponse;

typedef boost::shared_ptr< ::ObservationService::ObservationResponse > ObservationResponsePtr;
typedef boost::shared_ptr< ::ObservationService::ObservationResponse const> ObservationResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::ObservationService::ObservationResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::ObservationService::ObservationResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace ObservationService

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::ObservationService::ObservationResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::ObservationService::ObservationResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ObservationService::ObservationResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ObservationService::ObservationResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ObservationService::ObservationResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ObservationService::ObservationResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::ObservationService::ObservationResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "b1cd82a8c939565f6765e4d3a2dfd02f";
  }

  static const char* value(const ::ObservationService::ObservationResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xb1cd82a8c939565fULL;
  static const uint64_t static_value2 = 0x6765e4d3a2dfd02fULL;
};

template<class ContainerAllocator>
struct DataType< ::ObservationService::ObservationResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "ObservationService/ObservationResponse";
  }

  static const char* value(const ::ObservationService::ObservationResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::ObservationService::ObservationResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "int64 observation\n\
\n\
";
  }

  static const char* value(const ::ObservationService::ObservationResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::ObservationService::ObservationResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.observation);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct ObservationResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::ObservationService::ObservationResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::ObservationService::ObservationResponse_<ContainerAllocator>& v)
  {
    s << indent << "observation: ";
    Printer<int64_t>::stream(s, indent + "  ", v.observation);
  }
};

} // namespace message_operations
} // namespace ros

#endif // OBSERVATIONSERVICE_MESSAGE_OBSERVATIONRESPONSE_H
