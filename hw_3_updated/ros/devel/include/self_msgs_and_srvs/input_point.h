// Generated by gencpp from file self_msgs_and_srvs/input_point.msg
// DO NOT EDIT!


#ifndef SELF_MSGS_AND_SRVS_MESSAGE_INPUT_POINT_H
#define SELF_MSGS_AND_SRVS_MESSAGE_INPUT_POINT_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Point.h>

namespace self_msgs_and_srvs
{
template <class ContainerAllocator>
struct input_point_
{
  typedef input_point_<ContainerAllocator> Type;

  input_point_()
    : cur_pos()
    , pred_pos()
    , succ_pos()
    , target_pos()
    , target_vel()
    , target_acc()  {
    }
  input_point_(const ContainerAllocator& _alloc)
    : cur_pos(_alloc)
    , pred_pos(_alloc)
    , succ_pos(_alloc)
    , target_pos(_alloc)
    , target_vel(_alloc)
    , target_acc(_alloc)  {
  (void)_alloc;
    }



   typedef  ::geometry_msgs::Point_<ContainerAllocator>  _cur_pos_type;
  _cur_pos_type cur_pos;

   typedef  ::geometry_msgs::Point_<ContainerAllocator>  _pred_pos_type;
  _pred_pos_type pred_pos;

   typedef  ::geometry_msgs::Point_<ContainerAllocator>  _succ_pos_type;
  _succ_pos_type succ_pos;

   typedef  ::geometry_msgs::Point_<ContainerAllocator>  _target_pos_type;
  _target_pos_type target_pos;

   typedef  ::geometry_msgs::Point_<ContainerAllocator>  _target_vel_type;
  _target_vel_type target_vel;

   typedef  ::geometry_msgs::Point_<ContainerAllocator>  _target_acc_type;
  _target_acc_type target_acc;





  typedef boost::shared_ptr< ::self_msgs_and_srvs::input_point_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::self_msgs_and_srvs::input_point_<ContainerAllocator> const> ConstPtr;

}; // struct input_point_

typedef ::self_msgs_and_srvs::input_point_<std::allocator<void> > input_point;

typedef boost::shared_ptr< ::self_msgs_and_srvs::input_point > input_pointPtr;
typedef boost::shared_ptr< ::self_msgs_and_srvs::input_point const> input_pointConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::self_msgs_and_srvs::input_point_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::self_msgs_and_srvs::input_point_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::self_msgs_and_srvs::input_point_<ContainerAllocator1> & lhs, const ::self_msgs_and_srvs::input_point_<ContainerAllocator2> & rhs)
{
  return lhs.cur_pos == rhs.cur_pos &&
    lhs.pred_pos == rhs.pred_pos &&
    lhs.succ_pos == rhs.succ_pos &&
    lhs.target_pos == rhs.target_pos &&
    lhs.target_vel == rhs.target_vel &&
    lhs.target_acc == rhs.target_acc;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::self_msgs_and_srvs::input_point_<ContainerAllocator1> & lhs, const ::self_msgs_and_srvs::input_point_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace self_msgs_and_srvs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::self_msgs_and_srvs::input_point_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::self_msgs_and_srvs::input_point_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::self_msgs_and_srvs::input_point_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::self_msgs_and_srvs::input_point_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::self_msgs_and_srvs::input_point_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::self_msgs_and_srvs::input_point_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::self_msgs_and_srvs::input_point_<ContainerAllocator> >
{
  static const char* value()
  {
    return "4b446b61e7be949357360ca65dc198b5";
  }

  static const char* value(const ::self_msgs_and_srvs::input_point_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x4b446b61e7be9493ULL;
  static const uint64_t static_value2 = 0x57360ca65dc198b5ULL;
};

template<class ContainerAllocator>
struct DataType< ::self_msgs_and_srvs::input_point_<ContainerAllocator> >
{
  static const char* value()
  {
    return "self_msgs_and_srvs/input_point";
  }

  static const char* value(const ::self_msgs_and_srvs::input_point_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::self_msgs_and_srvs::input_point_<ContainerAllocator> >
{
  static const char* value()
  {
    return "geometry_msgs/Point cur_pos\n"
"geometry_msgs/Point pred_pos\n"
"geometry_msgs/Point succ_pos\n"
"\n"
"geometry_msgs/Point target_pos\n"
"geometry_msgs/Point target_vel\n"
"geometry_msgs/Point target_acc\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Point\n"
"# This contains the position of a point in free space\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
;
  }

  static const char* value(const ::self_msgs_and_srvs::input_point_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::self_msgs_and_srvs::input_point_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.cur_pos);
      stream.next(m.pred_pos);
      stream.next(m.succ_pos);
      stream.next(m.target_pos);
      stream.next(m.target_vel);
      stream.next(m.target_acc);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct input_point_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::self_msgs_and_srvs::input_point_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::self_msgs_and_srvs::input_point_<ContainerAllocator>& v)
  {
    s << indent << "cur_pos: ";
    s << std::endl;
    Printer< ::geometry_msgs::Point_<ContainerAllocator> >::stream(s, indent + "  ", v.cur_pos);
    s << indent << "pred_pos: ";
    s << std::endl;
    Printer< ::geometry_msgs::Point_<ContainerAllocator> >::stream(s, indent + "  ", v.pred_pos);
    s << indent << "succ_pos: ";
    s << std::endl;
    Printer< ::geometry_msgs::Point_<ContainerAllocator> >::stream(s, indent + "  ", v.succ_pos);
    s << indent << "target_pos: ";
    s << std::endl;
    Printer< ::geometry_msgs::Point_<ContainerAllocator> >::stream(s, indent + "  ", v.target_pos);
    s << indent << "target_vel: ";
    s << std::endl;
    Printer< ::geometry_msgs::Point_<ContainerAllocator> >::stream(s, indent + "  ", v.target_vel);
    s << indent << "target_acc: ";
    s << std::endl;
    Printer< ::geometry_msgs::Point_<ContainerAllocator> >::stream(s, indent + "  ", v.target_acc);
  }
};

} // namespace message_operations
} // namespace ros

#endif // SELF_MSGS_AND_SRVS_MESSAGE_INPUT_POINT_H
