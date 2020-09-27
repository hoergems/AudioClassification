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
    : centroid(0.0)
    , rms(0.0)  {
    }
  ObservationResponse_(const ContainerAllocator& _alloc)
    : centroid(0.0)
    , rms(0.0)  {
  (void)_alloc;
    }



   typedef double _centroid_type;
  _centroid_type centroid;

   typedef double _rms_type;
  _rms_type rms;





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


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::ObservationService::ObservationResponse_<ContainerAllocator1> & lhs, const ::ObservationService::ObservationResponse_<ContainerAllocator2> & rhs)
{
  return lhs.centroid == rhs.centroid &&
    lhs.rms == rhs.rms;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::ObservationService::ObservationResponse_<ContainerAllocator1> & lhs, const ::ObservationService::ObservationResponse_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace ObservationService

namespace ros
{
namespace message_traits
{





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
    return "2c482b3318ee938958236df814de1721";
  }

  static const char* value(const ::ObservationService::ObservationResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x2c482b3318ee9389ULL;
  static const uint64_t static_value2 = 0x58236df814de1721ULL;
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
    return "float64 centroid\n"
"float64 rms\n"
;
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
      stream.next(m.centroid);
      stream.next(m.rms);
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
    s << indent << "centroid: ";
    Printer<double>::stream(s, indent + "  ", v.centroid);
    s << indent << "rms: ";
    Printer<double>::stream(s, indent + "  ", v.rms);
  }
};

} // namespace message_operations
} // namespace ros

#endif // OBSERVATIONSERVICE_MESSAGE_OBSERVATIONRESPONSE_H
