/*
  -----------------------------------------------------
  A template to create behavior tree actions to publish to ROS topics.
  Don't forget to replace StringPublisherBT with the name of your
  action!
*/

#ifndef STRING_PUBLISHER_ACTIONS_HPP_
#define STRING_PUBLISHER_ACTIONS_HPP_

#include <behaviortree_cpp_v3/behavior_tree.h>
#include <vizzy_behavior_trees/util.hpp>
#include <ros/ros.h>
#include <vizzy_behavior_trees/rosbt_blackboard.hpp>
#include <std_msgs/String.h>

using namespace BT;

class StringPublisherBT : public BT::SyncActionNode
{
    public:
        StringPublisherBT(const std::string& name, const BT::NodeConfiguration& config);

        static BT::PortsList providedPorts()
        {
            return{BT::InputPort<std::string>("topic"),
                   BT::InputPort<std::string>("message")
                   };
        }

        ros::NodeHandle nh_;

        BT::NodeStatus tick() override;

    private:
        static std::map<std::string, ros::Publisher> _publishers;
};


#endif
