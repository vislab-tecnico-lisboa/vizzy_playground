#!/usr/bin/env python 

#Just "detects" and sends out a PoseArray of people's faces in gazebo

import rospy
import random
import math
import numpy as np
from gazebo_msgs.msg import LinkStates
from geometry_msgs.msg import Pose, PoseArray
import tf_conversions.posemath as pm
import PyKDL


rospy.init_node("faces_detector")
message_pub = rospy.Publisher("faces", PoseArray, queue_size=10)


def callback(data):

    #Get the robot current position. We want to publish people detections relative to the /base_footprint

    index = [i for i,x in enumerate(data.name) if 'vizzy::base_footprint' in x]

    robot_pose = Pose()

    for i in index:
        robot_pose.position.x = float(data.pose[i].position.x)
        robot_pose.position.y = float(data.pose[i].position.y)
        robot_pose.position.z = float(data.pose[i].position.z)
        robot_pose.orientation.x = float(data.pose[i].orientation.x)
        robot_pose.orientation.y = float(data.pose[i].orientation.y)
        robot_pose.orientation.z = float(data.pose[i].orientation.z)
        robot_pose.orientation.w = float(data.pose[i].orientation.w)

    map_to_robot_tf = pm.fromMsg(robot_pose)

    
    

    # Get people's positions
    idx = [i for i,x in enumerate(data.name) if ('pessoa' in x and 'base' in x) or ('person' in x and 'link' in x)]

    people = PoseArray()
    people.header.frame_id = "/base_footprint"
    people.header.stamp = rospy.Time.now()

    #Publish detections 
    for i in idx:
        

        person_pose = pm.fromMsg(data.pose[i])
        rot = person_pose.M
        pos = person_pose.p


        [R, P, Y] = rot.GetRPY()

        Y -= math.pi/2.0

        if Y < 0:
            Y = Y+2.0*math.pi

        rotated = PyKDL.Frame(PyKDL.Rotation.RPY(R,P,Y),
            PyKDL.Vector(pos.x(),pos.y(),pos.z()))


        person = pm.toMsg(map_to_robot_tf.Inverse()*rotated)


        #Add height to simulate that the face was detected

        person.position.z += 1.7

        #If people are too far away or behind the robot, we can't detect them
        if (math.sqrt(person.position.x**2+person.position.y**2) > 4.0) or (person.position.x < 0):
            continue

        people.poses.append(person)
    

    message_pub.publish(people)
    


if __name__ == '__main__':
    rospy.Subscriber("/gazebo/link_states", LinkStates, callback)
    rospy.spin()
