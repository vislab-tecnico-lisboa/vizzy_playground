#include "behaviortree_cpp_v3/bt_factory.h"
#include <string_publisher.hpp>
#include <get_approach_coords.hpp>
#include <gaze_down_coords.hpp>
#include <turn_to_person_coords.hpp>
#include <get_pose_from_face.hpp>
#include <get_state.hpp>
/* All action includes that you created. Example:
#include <vizzy_behavior_trees/actions/speech_actions.hpp>
#include <vizzy_behavior_trees/actions/move_base_actions.hpp>
#include <vizzy_behavior_trees/actions/general.hpp>
#include <vizzy_behavior_trees/actions/charging_actions.hpp>
#include <vizzy_behavior_trees/actions/gaze_actions.hpp>
#include <vizzy_behavior_trees/actions/arm_cartesian_actions.hpp>
#include <vizzy_behavior_trees/actions/arm_routines.hpp>
#include <vizzy_behavior_trees/actions/ros_msgs/get_geometry_msgs.hpp>
#include <vizzy_behavior_trees/actions/ros_msgs/get_std_msgs.hpp>
#include <vizzy_behavior_trees/conditions/general.hpp>
#include <vizzy_behavior_trees/actions/torso_actions.hpp>
*/

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<StringPublisherBT>("StringPublisher");
  factory.registerNodeType<GetApproachCoordsBT>("GetApproachCoords");
  factory.registerNodeType<GazeDownCoordsBT>("GazeDownCoords");
  factory.registerNodeType<TurnToPersonCoordsBT>("TurnToPersonCoords");
  factory.registerNodeType<GetPoseFromFaceBT>("GetPoseFromFace");
  factory.registerNodeType<GetStateBT>("GetState");
  /* Register your nodes here. Example:

  factory.registerNodeType<ArmRoutineBT>("ArmRoutines");
  factory.registerNodeType<CartesianActionBT>("ArmCartesian");
  factory.registerNodeType<GazeActionBT>("GazeAtTarget");
  factory.registerNodeType<WaitForXSeconds>("WaitForXSeconds");
  factory.registerNodeType<TimerAction>("TimerAction");
  factory.registerNodeType<DebugAction>("DebugAction");
  factory.registerNodeType<GeneralActionBT>("GeneralActionlib");
  factory.registerNodeType<ChargeActionBT>("Charge");
  factory.registerNodeType<CheckChargingBT>("CheckCharging");
  factory.registerNodeType<CheckBatteryBT>("CheckBattery");
  factory.registerNodeType<MoveBaseActionBT>("MoveBase");
  factory.registerNodeType<GetPoseArrayBT>("GetPoseArray");
  factory.registerNodeType<SelectPose>("SelectPose");
  factory.registerNodeType<SelectFieldFromPoseStamped>("SelectFieldFromPoseStamped");
  factory.registerNodeType<GetInt16BT>("GetInt16");
  factory.registerNodeType<GetFloat64BT>("GetFloat64");
  factory.registerNodeType<SpeechActionBT>("Speak");
  factory.registerNodeType<TorsoRoutineBT>("MoveTorso");
  */
}
