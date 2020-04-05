#include "behaviortree_cpp_v3/bt_factory.h"
#include <string_publisher.hpp>

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<StringPublisherBT>("StringPublisher");
}