#ifndef GET_STATE_HPP_
#define GET_STATE_HPP_


#include <behaviortree_cpp_v3/behavior_tree.h>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <std_msgs/Int16.h>

using namespace BT;


class GetStateBT : public SyncActionNode
{
  
    public:

        ros::Subscriber sub_;
        ros::CallbackQueue queue_;
        std_msgs::Int16 last_msg;

    GetStateBT(const std::string& name, const NodeConfiguration& config);
    static PortsList providedPorts()
    {
        return { InputPort<std::string>("topic"),
                 OutputPort<std::string>("state")};
    }

    NodeStatus tick() override;
    void callback(const std_msgs::Int16::ConstPtr &msg);
    

};


#endif
