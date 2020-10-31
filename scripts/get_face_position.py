#!/usr/bin/env python 

#Joao Avelino, 2020.
#ISR-Lisboa / IST

#An example action server where the robot is instructed to navigate X meters forward
#This uses the General purpose action from vizzy_behavior_trees. This mimicks a complex
#action that uses a lot of information and algorithms that a BT should not be processing
#by itself!

#ROS imports
import rospy
import actionlib

#General action files
from vizzy_behavior_trees.msg import GeneralAction, GeneralFeedback, GeneralResult

#Laser scan messages
from sensor_msgs.msg import LaserScan

#Direct wheel command messages
from geometry_msgs.msg import Twist

#Odometry messages: wheel information from encoders (unreliable in real scenarios)
from nav_msgs.msg import Odometry

from locale import atof


class MoveForwardServer(object):
    # create messages that are used to publish feedback/result
    _feedback = GeneralFeedback()
    _result = GeneralResult()

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, GeneralAction, execute_cb=self.execute_cb, auto_start = False)
        self._velocity_publisher = rospy.Publisher('/vizzy/cmd_vel', Twist, queue_size=10)
        self._as.start()
      
    def execute_cb(self, goal):

        #5Hz control loop
        dt = 1.0/5.0
        r = rospy.Rate(1.0/dt)
        success = True
        
        self._feedback.percentage = 0

        #Use general action's command as reference of how much to travel forward
        desired_meters = atof(goal.constants[0])

        #Use passed variables to set robot velocity and minimum obstacle distance
        desired_velocity = atof(goal.variables[0])
        desired_distance = atof(goal.variables[1])

        #Traveled distance accumulator
        travelled = 0
        
        # publish info to the console for the user
        rospy.loginfo('%s: Executing  move forward %f meters.' % (self._action_name, desired_meters))
        
        vel_x = 0

        # start executing the action
        while (not rospy.is_shutdown()) and (self._feedback.percentage < 100):

            #Check if the action was preempted!
            if self._as.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self._action_name)
                self._as.set_preempted()
                success = False
                break

            #Check if there is an obstacle closer than desired_distance m detected by the laser:
            scan = rospy.wait_for_message("/scan_filtered", LaserScan, timeout=0.15)

            if not all(i >= desired_distance for i in scan.ranges):
                print("Obstaculo perto!")
                success = False
                break

            print("Say what?")

            #Command the wheels to move forward at desired_velocity m/s!
            vel_msg = Twist()
            vel_msg.linear.x = desired_velocity
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = 0

            self._velocity_publisher.publish(vel_msg)

            #This chuck of code gets executed approx every dt seconds. We assume that velocity was kept
            #constant between calls.

            travelled += vel_x*dt
            odometry = rospy.wait_for_message("/odom", Odometry, timeout=0.1)
            vel_x = odometry.twist.twist.linear.x
            
            self._feedback.percentage = travelled/desired_meters*100

            print(self._feedback)

            # publish the feedback
            self._as.publish_feedback(self._feedback)

            r.sleep()
        
        #Stop the wheels!
        vel_msg = Twist()
        vel_msg.linear.x = 0
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = 0

        self._velocity_publisher.publish(vel_msg)
          
        if success:
            self._result.result = "Andei " + str(desired_meters) + " metros em frente!"
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(self._result)
        
        else:
            self._as.set_aborted()
            self._result.result = "Nao consigo andar. Um obstaculo!"
            rospy.loginfo('%s: Failed' % self._action_name)
        
if __name__ == '__main__':
    rospy.init_node('move_forward')
    server = MoveForwardServer(rospy.get_name())
    rospy.spin()
