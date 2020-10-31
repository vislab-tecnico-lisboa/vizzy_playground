#ifndef ORIENTATION_SUBSCRIBER_HPP_
#define ORIENTATION_SUBSCRIBER_HPP_


#include <behaviortree_cpp_v3/behavior_tree.h>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

using namespace BT;


class OrientationSubscriberBT : public SyncActionNode
{
  
    public:

        geometry_msgs::PoseWithCovarianceStamped last_msg;
        ros::Subscriber sub_;
        ros::CallbackQueue queue_;

    OrientationSubscriberBT(const std::string& name, const NodeConfiguration& config);
    static PortsList providedPorts()
    {
        return { InputPort<std::string>("topic"),
		 InputPort<std::string>("field"),
                 OutputPort<double>("value")};
    }

    NodeStatus tick() override;
    void callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg);

};


#endif
