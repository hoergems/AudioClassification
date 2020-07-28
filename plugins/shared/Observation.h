// Generated by gencpp from file ObservationService/Observation.msg
// DO NOT EDIT!


#ifndef OBSERVATIONSERVICE_MESSAGE_OBSERVATION_H
#define OBSERVATIONSERVICE_MESSAGE_OBSERVATION_H

#include <ros/service_traits.h>


// #include <ObservationService/ObservationRequest.h>
// #include <ObservationService/ObservationResponse.h>
#include "/home/jihirshu/workspaces/ObjectDetection_ws/devel/include/ObservationService/ObservationRequest.h"
#include "/home/jihirshu/workspaces/ObjectDetection_ws/devel/include/ObservationService/ObservationResponse.h"

namespace ObservationService
{

struct Observation
{

typedef ObservationRequest Request;
typedef ObservationResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct Observation
} // namespace ObservationService


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::ObservationService::Observation > {
  static const char* value()
  {
    return "27afc928d81a66a9d1064a580ce4b6b8";
  }

  static const char* value(const ::ObservationService::Observation&) { return value(); }
};

template<>
struct DataType< ::ObservationService::Observation > {
  static const char* value()
  {
    return "ObservationService/Observation";
  }

  static const char* value(const ::ObservationService::Observation&) { return value(); }
};


// service_traits::MD5Sum< ::ObservationService::ObservationRequest> should match 
// service_traits::MD5Sum< ::ObservationService::Observation > 
template<>
struct MD5Sum< ::ObservationService::ObservationRequest>
{
  static const char* value()
  {
    return MD5Sum< ::ObservationService::Observation >::value();
  }
  static const char* value(const ::ObservationService::ObservationRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::ObservationService::ObservationRequest> should match 
// service_traits::DataType< ::ObservationService::Observation > 
template<>
struct DataType< ::ObservationService::ObservationRequest>
{
  static const char* value()
  {
    return DataType< ::ObservationService::Observation >::value();
  }
  static const char* value(const ::ObservationService::ObservationRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::ObservationService::ObservationResponse> should match 
// service_traits::MD5Sum< ::ObservationService::Observation > 
template<>
struct MD5Sum< ::ObservationService::ObservationResponse>
{
  static const char* value()
  {
    return MD5Sum< ::ObservationService::Observation >::value();
  }
  static const char* value(const ::ObservationService::ObservationResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::ObservationService::ObservationResponse> should match 
// service_traits::DataType< ::ObservationService::Observation > 
template<>
struct DataType< ::ObservationService::ObservationResponse>
{
  static const char* value()
  {
    return DataType< ::ObservationService::Observation >::value();
  }
  static const char* value(const ::ObservationService::ObservationResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // OBSERVATIONSERVICE_MESSAGE_OBSERVATION_H
