#include "orientationsubscriber.hpp"
#include <cmath>
#include "behaviortree_cpp_v3/bt_factory.h"


void OrientationSubscriberBT::callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
{

    this->last_msg = *msg;
}


OrientationSubscriberBT::OrientationSubscriberBT(const std::string& name, const NodeConfiguration& config)
    : SyncActionNode(name, config)
    {
        ros::NodeHandle nh_;
        BT::Optional<std::string> topic = TreeNode::getInput<std::string>("topic");


        if(!topic){
            throw BT::RuntimeError("missing required inputs [topic]: ",
                                    topic.error()); 
        }
 

        ros::SubscribeOptions ops = ros::SubscribeOptions::create<geometry_msgs::PoseWithCovarianceStamped>
            (topic.value(), 1, boost::bind(&OrientationSubscriberBT::callback, this, _1), ros::VoidPtr(), &queue_);

        sub_ = nh_.subscribe(ops);
    }

BT::NodeStatus OrientationSubscriberBT::tick()
{
    if(sub_.getNumPublishers() < 1)
    {
        return BT::NodeStatus::FAILURE;
    }

    BT::Optional<std::string> field = TreeNode::getInput<std::string>("field");
     if(!field)
    {

        throw BT::RuntimeError("missing required inputs [field]: ",
                                field.error());
    }


    setStatus(BT::NodeStatus::RUNNING);

 
    queue_.callOne();
    
    geometry_msgs::PoseWithCovarianceStamped amcl;
    amcl = this->last_msg;
    
    double value;

    if(field.value() == "x")
    {
        value = amcl.pose.pose.orientation.x;
    }else if(field.value() == "y")
    {
        value = amcl.pose.pose.orientation.y;
    }else if(field.value() == "z")
    {
        value = amcl.pose.pose.orientation.z;
    }else if(field.value() == "w")
    {
        value = amcl.pose.pose.orientation.w;
    }else
        return BT::NodeStatus::FAILURE;

    auto result = setOutput("value", value);

    if(!result)
    {
        std::cout << result.error() << std::endl;
        return BT::NodeStatus::FAILURE;
    }

   return BT::NodeStatus::SUCCESS;
}
