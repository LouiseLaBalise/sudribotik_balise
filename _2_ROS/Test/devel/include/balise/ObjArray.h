// Generated by gencpp from file balise/ObjArray.msg
// DO NOT EDIT!


#ifndef BALISE_MESSAGE_OBJARRAY_H
#define BALISE_MESSAGE_OBJARRAY_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <balise/Obj.h>

namespace balise
{
template <class ContainerAllocator>
struct ObjArray_
{
  typedef ObjArray_<ContainerAllocator> Type;

  ObjArray_()
    : elements()  {
    }
  ObjArray_(const ContainerAllocator& _alloc)
    : elements(_alloc)  {
  (void)_alloc;
    }



   typedef std::vector< ::balise::Obj_<ContainerAllocator> , typename std::allocator_traits<ContainerAllocator>::template rebind_alloc< ::balise::Obj_<ContainerAllocator> >> _elements_type;
  _elements_type elements;





  typedef boost::shared_ptr< ::balise::ObjArray_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::balise::ObjArray_<ContainerAllocator> const> ConstPtr;

}; // struct ObjArray_

typedef ::balise::ObjArray_<std::allocator<void> > ObjArray;

typedef boost::shared_ptr< ::balise::ObjArray > ObjArrayPtr;
typedef boost::shared_ptr< ::balise::ObjArray const> ObjArrayConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::balise::ObjArray_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::balise::ObjArray_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::balise::ObjArray_<ContainerAllocator1> & lhs, const ::balise::ObjArray_<ContainerAllocator2> & rhs)
{
  return lhs.elements == rhs.elements;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::balise::ObjArray_<ContainerAllocator1> & lhs, const ::balise::ObjArray_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace balise

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::balise::ObjArray_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::balise::ObjArray_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::balise::ObjArray_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::balise::ObjArray_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::balise::ObjArray_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::balise::ObjArray_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::balise::ObjArray_<ContainerAllocator> >
{
  static const char* value()
  {
    return "bd1a346423c1712118b76ab7523541bc";
  }

  static const char* value(const ::balise::ObjArray_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xbd1a346423c17121ULL;
  static const uint64_t static_value2 = 0x18b76ab7523541bcULL;
};

template<class ContainerAllocator>
struct DataType< ::balise::ObjArray_<ContainerAllocator> >
{
  static const char* value()
  {
    return "balise/ObjArray";
  }

  static const char* value(const ::balise::ObjArray_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::balise::ObjArray_<ContainerAllocator> >
{
  static const char* value()
  {
    return "Obj[] elements\n"
"================================================================================\n"
"MSG: balise/Obj\n"
"string dscript \n"
"uint32 ident\n"
"geometry_msgs/Vector3 position\n"
"float64 theta\n"
"================================================================================\n"
"MSG: geometry_msgs/Vector3\n"
"# This represents a vector in free space. \n"
"# It is only meant to represent a direction. Therefore, it does not\n"
"# make sense to apply a translation to it (e.g., when applying a \n"
"# generic rigid transformation to a Vector3, tf2 will only apply the\n"
"# rotation). If you want your data to be translatable too, use the\n"
"# geometry_msgs/Point message instead.\n"
"\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
;
  }

  static const char* value(const ::balise::ObjArray_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::balise::ObjArray_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.elements);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct ObjArray_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::balise::ObjArray_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::balise::ObjArray_<ContainerAllocator>& v)
  {
    s << indent << "elements[]" << std::endl;
    for (size_t i = 0; i < v.elements.size(); ++i)
    {
      s << indent << "  elements[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::balise::Obj_<ContainerAllocator> >::stream(s, indent + "    ", v.elements[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // BALISE_MESSAGE_OBJARRAY_H