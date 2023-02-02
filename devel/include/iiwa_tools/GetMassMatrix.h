// Generated by gencpp from file iiwa_tools/GetMassMatrix.msg
// DO NOT EDIT!


#ifndef IIWA_TOOLS_MESSAGE_GETMASSMATRIX_H
#define IIWA_TOOLS_MESSAGE_GETMASSMATRIX_H

#include <ros/service_traits.h>


#include <iiwa_tools/GetMassMatrixRequest.h>
#include <iiwa_tools/GetMassMatrixResponse.h>


namespace iiwa_tools
{

struct GetMassMatrix
{

typedef GetMassMatrixRequest Request;
typedef GetMassMatrixResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct GetMassMatrix
} // namespace iiwa_tools


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::iiwa_tools::GetMassMatrix > {
  static const char* value()
  {
    return "c2241a2536933b0e13a23c6758fa7bfe";
  }

  static const char* value(const ::iiwa_tools::GetMassMatrix&) { return value(); }
};

template<>
struct DataType< ::iiwa_tools::GetMassMatrix > {
  static const char* value()
  {
    return "iiwa_tools/GetMassMatrix";
  }

  static const char* value(const ::iiwa_tools::GetMassMatrix&) { return value(); }
};


// service_traits::MD5Sum< ::iiwa_tools::GetMassMatrixRequest> should match
// service_traits::MD5Sum< ::iiwa_tools::GetMassMatrix >
template<>
struct MD5Sum< ::iiwa_tools::GetMassMatrixRequest>
{
  static const char* value()
  {
    return MD5Sum< ::iiwa_tools::GetMassMatrix >::value();
  }
  static const char* value(const ::iiwa_tools::GetMassMatrixRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::iiwa_tools::GetMassMatrixRequest> should match
// service_traits::DataType< ::iiwa_tools::GetMassMatrix >
template<>
struct DataType< ::iiwa_tools::GetMassMatrixRequest>
{
  static const char* value()
  {
    return DataType< ::iiwa_tools::GetMassMatrix >::value();
  }
  static const char* value(const ::iiwa_tools::GetMassMatrixRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::iiwa_tools::GetMassMatrixResponse> should match
// service_traits::MD5Sum< ::iiwa_tools::GetMassMatrix >
template<>
struct MD5Sum< ::iiwa_tools::GetMassMatrixResponse>
{
  static const char* value()
  {
    return MD5Sum< ::iiwa_tools::GetMassMatrix >::value();
  }
  static const char* value(const ::iiwa_tools::GetMassMatrixResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::iiwa_tools::GetMassMatrixResponse> should match
// service_traits::DataType< ::iiwa_tools::GetMassMatrix >
template<>
struct DataType< ::iiwa_tools::GetMassMatrixResponse>
{
  static const char* value()
  {
    return DataType< ::iiwa_tools::GetMassMatrix >::value();
  }
  static const char* value(const ::iiwa_tools::GetMassMatrixResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // IIWA_TOOLS_MESSAGE_GETMASSMATRIX_H
