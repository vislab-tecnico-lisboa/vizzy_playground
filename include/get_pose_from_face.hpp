#ifndef GET_POSE_FROM_FACE_HPP_
#define GET_POSE_FROM_FACE_HPP_


#include <behaviortree_cpp_v3/behavior_tree.h>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <geometry_msgs/PoseStamped.h>
#include <vizzy_playground/FaceExtraction.h>
#include <std_msgs/String.h>

using namespace BT;


class GetPoseFromFaceBT : public SyncActionNode
{
  
    public:

        ros::Subscriber sub_;
        ros::CallbackQueue queue_;
        vizzy_playground::FaceExtraction last_msg;

    GetPoseFromFaceBT(const std::string& name, const NodeConfiguration& config);
    static PortsList providedPorts()
    {
        return { InputPort<std::string>("topic"),
                 OutputPort<geometry_msgs::PoseStamped>("final_pose")};
    }

    NodeStatus tick() override;
    void callback(const vizzy_playground::FaceExtraction::ConstPtr &msg);
    

};


#endif
