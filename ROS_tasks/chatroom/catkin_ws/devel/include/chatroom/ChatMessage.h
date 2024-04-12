// Generated by gencpp from file chatroom/ChatMessage.msg
// DO NOT EDIT!


#ifndef CHATROOM_MESSAGE_CHATMESSAGE_H
#define CHATROOM_MESSAGE_CHATMESSAGE_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace chatroom
{
template <class ContainerAllocator>
struct ChatMessage_
{
  typedef ChatMessage_<ContainerAllocator> Type;

  ChatMessage_()
    : sender()
    , message()  {
    }
  ChatMessage_(const ContainerAllocator& _alloc)
    : sender(_alloc)
    , message(_alloc)  {
  (void)_alloc;
    }



   typedef std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> _sender_type;
  _sender_type sender;

   typedef std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> _message_type;
  _message_type message;





  typedef boost::shared_ptr< ::chatroom::ChatMessage_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::chatroom::ChatMessage_<ContainerAllocator> const> ConstPtr;

}; // struct ChatMessage_

typedef ::chatroom::ChatMessage_<std::allocator<void> > ChatMessage;

typedef boost::shared_ptr< ::chatroom::ChatMessage > ChatMessagePtr;
typedef boost::shared_ptr< ::chatroom::ChatMessage const> ChatMessageConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::chatroom::ChatMessage_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::chatroom::ChatMessage_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::chatroom::ChatMessage_<ContainerAllocator1> & lhs, const ::chatroom::ChatMessage_<ContainerAllocator2> & rhs)
{
  return lhs.sender == rhs.sender &&
    lhs.message == rhs.message;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::chatroom::ChatMessage_<ContainerAllocator1> & lhs, const ::chatroom::ChatMessage_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace chatroom

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::chatroom::ChatMessage_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::chatroom::ChatMessage_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::chatroom::ChatMessage_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::chatroom::ChatMessage_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::chatroom::ChatMessage_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::chatroom::ChatMessage_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::chatroom::ChatMessage_<ContainerAllocator> >
{
  static const char* value()
  {
    return "985ef42f02a992c58d5de728edeb121e";
  }

  static const char* value(const ::chatroom::ChatMessage_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x985ef42f02a992c5ULL;
  static const uint64_t static_value2 = 0x8d5de728edeb121eULL;
};

template<class ContainerAllocator>
struct DataType< ::chatroom::ChatMessage_<ContainerAllocator> >
{
  static const char* value()
  {
    return "chatroom/ChatMessage";
  }

  static const char* value(const ::chatroom::ChatMessage_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::chatroom::ChatMessage_<ContainerAllocator> >
{
  static const char* value()
  {
    return "string sender\n"
"string message\n"
;
  }

  static const char* value(const ::chatroom::ChatMessage_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::chatroom::ChatMessage_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.sender);
      stream.next(m.message);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct ChatMessage_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::chatroom::ChatMessage_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::chatroom::ChatMessage_<ContainerAllocator>& v)
  {
    s << indent << "sender: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>::stream(s, indent + "  ", v.sender);
    s << indent << "message: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>::stream(s, indent + "  ", v.message);
  }
};

} // namespace message_operations
} // namespace ros

#endif // CHATROOM_MESSAGE_CHATMESSAGE_H