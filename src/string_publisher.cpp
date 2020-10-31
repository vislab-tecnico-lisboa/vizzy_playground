/*
  -----------------------------------------------------
  A template to create behavior tree actions to publish to ROS topics.
  Don't forget to replace PUBLISHER_TEMPLATE_BT with the name of your
  action!
*/

#include <string_publisher.hpp>
#include <behaviortree_cpp_v3/bt_factory.h>


std::map<std::string, ros::Publisher> StringPublisherBT::_publishers;

StringPublisherBT::StringPublisherBT(const std::string& name, const NodeConfiguration& config)
    : SyncActionNode(name, config), nh_()
{


}

BT::NodeStatus StringPublisherBT::tick()
{

    BT::Optional<std::string> topic = getInput<std::string>("topic");
    BT::Optional<std::string> message = getInput<std::string>("message");


    /*Check if inputs are set*/
    if(!topic)
    {
        throw BT::RuntimeError("missing required inputs [topic]: ",
                                   topic.error() );
    }
    
    if (!message)
    {
        throw BT::RuntimeError("missing required inputs [message]: ", 					message.error() );
    }


    auto publisher_pair = _publishers.find(topic.value());

    if(publisher_pair == _publishers.end())
    {
        _publishers[topic.value()] = nh_.advertise<std_msgs::String>(topic.value(), 1, true);
        ROS_INFO_STREAM("Created publisher to topic: " << topic.value());
    }

    publisher_pair = _publishers.find(topic.value());
    ros::Publisher &pub = publisher_pair->second;


    std_msgs::String command;
    command.data = message.value();

    pub.publish(command);

    if(pub.getNumSubscribers() < 1)
    {
        return BT::NodeStatus::FAILURE;
    }else{
        return BT::NodeStatus::SUCCESS;
    }

}
