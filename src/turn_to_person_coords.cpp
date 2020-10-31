#include "turn_to_person_coords.hpp"
#include <cmath>
#include "behaviortree_cpp_v3/bt_factory.h"
#include <tf/transform_datatypes.h>

void TurnToPersonCoordsBT::callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg) //Subscribe amcl_pose node
{

    this->last_msg = *msg;
}

TurnToPersonCoordsBT::TurnToPersonCoordsBT(const std::string& name, const NodeConfiguration& config)
    : SyncActionNode(name, config)
    {
        ros::NodeHandle nh_;
        BT::Optional<std::string> topic = TreeNode::getInput<std::string>("topic");


        if(!topic){
            throw BT::RuntimeError("missing required inputs [topic]: ",
                                    topic.error()); 
        }

 

        ros::SubscribeOptions ops = ros::SubscribeOptions::create<geometry_msgs::PoseWithCovarianceStamped>
            (topic.value(), 1, boost::bind(&TurnToPersonCoordsBT::callback, this, _1), ros::VoidPtr(), &queue_);

        sub_ = nh_.subscribe(ops);
	
	
    }
BT::NodeStatus TurnToPersonCoordsBT::tick()
{
    if(sub_.getNumPublishers() < 1)
    {
        return BT::NodeStatus::FAILURE;
    }


    BT::Optional<geometry_msgs::PoseStamped> face_pose = TreeNode::getInput<geometry_msgs::PoseStamped>("face_pose"); //pose of face identified
     if(!face_pose)
    {

        throw BT::RuntimeError("missing required inputs [face_pose]: ",
                                face_pose.error());
    }

    setStatus(BT::NodeStatus::RUNNING);

 
    queue_.callOne();
    
    geometry_msgs::PoseStamped pose_val = face_pose.value();
    geometry_msgs::PoseWithCovarianceStamped amcl;
    amcl = this->last_msg;

    double x_o1,y_o1,z_o1,w_o1,x_o2,y_o2,z_o2,w_o2;
    x_o1 = pose_val.pose.orientation.x;
    y_o1 = pose_val.pose.orientation.y;
    z_o1 = pose_val.pose.orientation.z;
    w_o1 = pose_val.pose.orientation.w;
    x_o2 = amcl.pose.pose.orientation.x;
    y_o2 = amcl.pose.pose.orientation.y;
    z_o2 = amcl.pose.pose.orientation.z;
    w_o2 = amcl.pose.pose.orientation.w;


    tf::Quaternion q1(x_o1,y_o1,z_o1,w_o1), q2(x_o2,y_o2,z_o2,w_o2);
    tf::Matrix3x3 m1(q1), m2(q2);

    double roll_1,roll_2,pitch_1,pitch_2,yaw_1,yaw_2; 
    //Calculating the orientations in angle units -> RPY1 is person's orientation(robot coordinates) and RPY2 is robot's orientation (world    	   coordinates):
    m1.getRPY(roll_1, pitch_1, yaw_1); 
    m2.getRPY(roll_2, pitch_2, yaw_2);

    double yaw_person; 
    //Calculating person's orientation in world coordinates: (We ignore roll and pitch because the robot only rotates around its Z axis)
    yaw_person = yaw_1 + yaw_2;
    
    double x_final, x_person, y_final, y_person, x_robot, y_robot;
    x_person = pose_val.pose.position.x; //position of the person (x,y) in the robot frame-> Z is irrelevant
    y_person = pose_val.pose.position.y;
    x_robot = amcl.pose.pose.position.x; //position of robot (x,y) in world frame
    y_robot = amcl.pose.pose.position.y;
    
    double x_person_world, y_person_world;
    //Now we have to calculate person's position at the world frame, using a rotation in Z of the robot's orientation (yaw_2):
    // R = [cos(yaw_2)  -sin(yaw_2)]
    //     [sin(yaw_2)  cos(yaw_2) ]
    // And the translation which is robot's position in the world: T= [x_robot y_robot]'
    x_person_world = cos(yaw_2)*x_person - sin(yaw_2)*y_person + x_robot; 
    y_person_world = sin(yaw_2)*x_person + cos(yaw_2)*y_person + y_robot;
    geometry_msgs::PoseStamped final_pose;
    
    double desired_angle;
    desired_angle = atan2((y_person_world - y_robot),(x_person_world - x_robot));

    final_pose.pose.orientation.x = 0;
    final_pose.pose.orientation.y = 0;
    final_pose.pose.orientation.z = sin(desired_angle/2);
    final_pose.pose.orientation.w = cos(desired_angle/2);
    final_pose.pose.position = amcl.pose.pose.position;


    auto result = setOutput("final_pose", final_pose);

    if(!result)
    {
        std::cout << result.error() << std::endl;
        return BT::NodeStatus::FAILURE;
    }

   return BT::NodeStatus::SUCCESS;
}
