#ifndef GAZE_DOWN_COORDS_HPP_
#define GAZE_DOWN_COORDS_HPP_


#include <behaviortree_cpp_v3/behavior_tree.h>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <geometry_msgs/PoseStamped.h>

using namespace BT;


class GazeDownCoordsBT : public SyncActionNode
{
  
    public:

        ros::CallbackQueue queue_;

    GazeDownCoordsBT(const std::string& name, const NodeConfiguration& config);
    static PortsList providedPorts()
    {
        return { InputPort<geometry_msgs::PoseStamped>("face_pose"),
                 OutputPort<geometry_msgs::PoseStamped>("final_pose")};
    }

    NodeStatus tick() override;
    

};


#endif
