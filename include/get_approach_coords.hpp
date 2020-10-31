#ifndef GET_APPROACH_COORDS_HPP_
#define GET APPROACH_COORDS_HPP_


#include <behaviortree_cpp_v3/behavior_tree.h>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

using namespace BT;


class GetApproachCoordsBT : public SyncActionNode
{
  
    public:

        geometry_msgs::PoseWithCovarianceStamped last_msg;
        ros::Subscriber sub_;
        ros::CallbackQueue queue_;

    GetApproachCoordsBT(const std::string& name, const NodeConfiguration& config);
    static PortsList providedPorts()
    {
        return { InputPort<std::string>("topic"),
		 InputPort<geometry_msgs::PoseStamped>("face_pose"),
		 InputPort<double>("dist_to_person"),
                 OutputPort<geometry_msgs::PoseStamped>("final_pose")};
    }

    NodeStatus tick() override;

    void callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg);
    

};


#endif
