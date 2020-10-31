#include "gaze_down_coords.hpp"
#include <cmath>
#include "behaviortree_cpp_v3/bt_factory.h"
#include <tf/transform_datatypes.h>


GazeDownCoordsBT::GazeDownCoordsBT(const std::string& name, const NodeConfiguration& config)
    : SyncActionNode(name, config)
{


}

BT::NodeStatus GazeDownCoordsBT::tick()
{

    BT::Optional<geometry_msgs::PoseStamped> face_pose = TreeNode::getInput<geometry_msgs::PoseStamped>("face_pose"); //pose of face identified
     if(!face_pose)
    {

        throw BT::RuntimeError("missing required inputs [face_pose]: ",
                                face_pose.error());
    }

    setStatus(BT::NodeStatus::RUNNING);

 
    queue_.callOne();
    
    geometry_msgs::PoseStamped pose_val = face_pose.value();

    geometry_msgs::PoseStamped final_pose;
    final_pose = pose_val;
    final_pose.pose.position.z = final_pose.pose.position.z/2;


    auto result = setOutput("final_pose", final_pose);

    if(!result)
    {
        std::cout << result.error() << std::endl;
        return BT::NodeStatus::FAILURE;
    }

   return BT::NodeStatus::SUCCESS;
}
