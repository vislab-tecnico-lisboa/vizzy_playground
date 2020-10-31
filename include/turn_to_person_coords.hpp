#ifndef TURN_TO_PERSON_COORDS_HPP_
#define TURN_TO_PERSON_COORDS_HPP_


#include <behaviortree_cpp_v3/behavior_tree.h>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <std_msgs/String.h>

using namespace BT;


class TurnToPersonCoordsBT : public SyncActionNode
{
  
    public:

        ros::Subscriber sub_;
        ros::CallbackQueue queue_;
        geometry_msgs::PoseWithCovarianceStamped last_msg;

    TurnToPersonCoordsBT(const std::string& name, const NodeConfiguration& config);
    static PortsList providedPorts()
    {
        return { InputPort<geometry_msgs::PoseStamped>("face_pose"),
		 InputPort<std::string>("topic"),
                 OutputPort<geometry_msgs::PoseStamped>("final_pose")};
    }

    NodeStatus tick() override;
    void callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg);
    

};


#endif
