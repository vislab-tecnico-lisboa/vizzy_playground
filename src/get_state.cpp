#include "get_state.hpp"
#include <cmath>
#include "behaviortree_cpp_v3/bt_factory.h"
#include <tf/transform_datatypes.h>


void GetStateBT::callback(const std_msgs::Int16::ConstPtr &msg) //Subscribe state node
{

    this->last_msg = *msg;
}

GetStateBT::GetStateBT(const std::string& name, const NodeConfiguration& config)
    : SyncActionNode(name, config)
    {
        ros::NodeHandle nh_;
        BT::Optional<std::string> topic = TreeNode::getInput<std::string>("topic");


        if(!topic){
            throw BT::RuntimeError("missing required inputs [topic]: ",
                                    topic.error()); 
        }

 

        ros::SubscribeOptions ops = ros::SubscribeOptions::create<std_msgs::Int16>
            (topic.value(), 1, boost::bind(&GetStateBT::callback, this, _1), ros::VoidPtr(), &queue_);

        sub_ = nh_.subscribe(ops);
	
	
    }

BT::NodeStatus GetStateBT::tick()
{
    if(sub_.getNumPublishers() < 1)
    {
        return BT::NodeStatus::FAILURE;
    }



    setStatus(BT::NodeStatus::RUNNING);

 
    queue_.callOne();
    
    std_msgs::Int16 state;
    state = this->last_msg;

    std::ostringstream ss;
    ss << state.data;
    std::string str_state = ss.str(); 

    auto result = setOutput("state", str_state);

    if(!result)
    {
        std::cout << result.error() << std::endl;
        return BT::NodeStatus::FAILURE;
    }

   return BT::NodeStatus::SUCCESS;
}
