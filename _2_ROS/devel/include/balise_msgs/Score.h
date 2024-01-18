// Generated by gencpp from file balise_msgs/Score.msg
// DO NOT EDIT!


#ifndef BALISE_MSGS_MESSAGE_SCORE_H
#define BALISE_MSGS_MESSAGE_SCORE_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace balise_msgs
{
template <class ContainerAllocator>
struct Score_
{
  typedef Score_<ContainerAllocator> Type;

  Score_()
    : score(0)  {
    }
  Score_(const ContainerAllocator& _alloc)
    : score(0)  {
  (void)_alloc;
    }



   typedef int32_t _score_type;
  _score_type score;





  typedef boost::shared_ptr< ::balise_msgs::Score_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::balise_msgs::Score_<ContainerAllocator> const> ConstPtr;

}; // struct Score_

typedef ::balise_msgs::Score_<std::allocator<void> > Score;

typedef boost::shared_ptr< ::balise_msgs::Score > ScorePtr;
typedef boost::shared_ptr< ::balise_msgs::Score const> ScoreConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::balise_msgs::Score_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::balise_msgs::Score_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::balise_msgs::Score_<ContainerAllocator1> & lhs, const ::balise_msgs::Score_<ContainerAllocator2> & rhs)
{
  return lhs.score == rhs.score;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::balise_msgs::Score_<ContainerAllocator1> & lhs, const ::balise_msgs::Score_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace balise_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::balise_msgs::Score_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::balise_msgs::Score_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::balise_msgs::Score_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::balise_msgs::Score_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::balise_msgs::Score_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::balise_msgs::Score_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::balise_msgs::Score_<ContainerAllocator> >
{
  static const char* value()
  {
    return "2ed963831938da1fc8151b77dec7741f";
  }

  static const char* value(const ::balise_msgs::Score_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x2ed963831938da1fULL;
  static const uint64_t static_value2 = 0xc8151b77dec7741fULL;
};

template<class ContainerAllocator>
struct DataType< ::balise_msgs::Score_<ContainerAllocator> >
{
  static const char* value()
  {
    return "balise_msgs/Score";
  }

  static const char* value(const ::balise_msgs::Score_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::balise_msgs::Score_<ContainerAllocator> >
{
  static const char* value()
  {
    return "int32 score\n"
;
  }

  static const char* value(const ::balise_msgs::Score_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::balise_msgs::Score_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.score);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct Score_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::balise_msgs::Score_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::balise_msgs::Score_<ContainerAllocator>& v)
  {
    s << indent << "score: ";
    Printer<int32_t>::stream(s, indent + "  ", v.score);
  }
};

} // namespace message_operations
} // namespace ros

#endif // BALISE_MSGS_MESSAGE_SCORE_H
