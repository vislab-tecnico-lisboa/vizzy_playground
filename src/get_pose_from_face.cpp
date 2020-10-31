#include "get_pose_from_face.hpp"
#include <cmath>
#include "behaviortree_cpp_v3/bt_factory.h"
#include <tf/transform_datatypes.h>

void GetPoseFromFaceBT::callback(const vizzy_playground::FaceExtraction::ConstPtr &msg) //Subscribe faces topic
{

    this->last_msg = *msg;
}

GetPoseFromFaceBT::GetPoseFromFaceBT(const std::string& name, const NodeConfiguration& config)
    : SyncActionNode(name, config)
    {
        ros::NodeHandle nh_;
        BT::Optional<std::string> topic = TreeNode::getInput<std::string>("topic");


        if(!topic){
            throw BT::RuntimeError("missing required inputs [topic]: ",
                                    topic.error()); 
        }

 

        ros::SubscribeOptions ops = ros::SubscribeOptions::create<vizzy_playground::FaceExtraction>
            (topic.value(), 1, boost::bind(&GetPoseFromFaceBT::callback, this, _1), ros::VoidPtr(), &queue_);

        sub_ = nh_.subscribe(ops);
	
	
    }
BT::NodeStatus GetPoseFromFaceBT::tick()
{
    if(sub_.getNumPublishers() < 1)
    {
        return BT::NodeStatus::FAILURE;
    }

    setStatus(BT::NodeStatus::RUNNING);

 
    queue_.callOne();
    
    vizzy_playground::FaceExtraction face_extract;
    face_extract = this->last_msg;

    double t_x,t_y,t_z,pitch,yaw,roll;
    t_x = face_extract.poses_T[0]*0.001;
    t_y = face_extract.poses_T[1]*0.001;
    t_z = face_extract.poses_T[2]*0.001;
    pitch = face_extract.poses_R[0];
    yaw = face_extract.poses_R[1];
    roll = face_extract.poses_R[2];

    geometry_msgs::PoseStamped final_pose;

    final_pose.pose.orientation.x = sin(roll/2) * cos(pitch/2) * cos(yaw/2) - cos(roll/2) * sin(pitch/2) * sin(yaw/2);
    final_pose.pose.orientation.y = cos(roll/2) * sin(pitch/2) * cos(yaw/2) + sin(roll/2) * cos(pitch/2) * sin(yaw/2);
    final_pose.pose.orientation.z = cos(roll/2) * cos(pitch/2) * sin(yaw/2) - sin(roll/2) * sin(pitch/2) * cos(yaw/2);
    final_pose.pose.orientation.w = cos(roll/2) * cos(pitch/2) * cos(yaw/2) + sin(roll/2) * sin(pitch/2) * sin(yaw/2);
    final_pose.pose.position.x = t_z;
    final_pose.pose.position.y = t_x;
    final_pose.pose.position.z = t_y;


    auto result = setOutput("final_pose", final_pose);

    if(!result)
    {
        std::cout << result.error() << std::endl;
        return BT::NodeStatus::FAILURE;
    }

   return BT::NodeStatus::SUCCESS;
}
