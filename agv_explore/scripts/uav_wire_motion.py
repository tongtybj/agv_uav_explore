#!/usr/bin/env python

import sys
import rospy
import numpy as np
import time
from nav_msgs.msg import Odometry
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState

class UavWireMotion:
    def __init__(self):

        rospy.init_node("uav_wire_motion")


        self.uav_pos = None
        self.uav_init_pos = None

        self.servo_name = rospy.get_param("~servo_name", 'wire_servo')
        self.servo_init_angle = None
        self.servo_angle = None

        self.tension_mode = False

        self.wire_bias = rospy.get_param("~wire_bias", 0.02)
        self.normal_torque = rospy.get_param("~normal_torque", 20)
        self.wind_torque = rospy.get_param("~wind_torque", 5)

        # wire (regression) model
        # self.a_vec = rospy.get_param("~wire_model", [0.0])


        # wire simple mode
        self.wire_scale = rospy.get_param("~wire_scale", -16.734)

        # pub/sub
        self.servo_angle_cmd_pub = rospy.Publisher("dynamixel_workbench/joint_trajectory", JointTrajectory, queue_size=1)
        self.servo_torque_cmd_pub = rospy.Publisher("dynamixel_workbench/cmd_current", JointState, queue_size=1)

        self.uav_odom_sub = rospy.Subscriber("uav/cog/odom", Odometry, self.odom_callback)
        self.servo_state_sub = rospy.Subscriber("dynamixel_workbench/joint_states", JointState, self.servo_state_callback)

        time.sleep(0.5)

        # reset winding
        self.reset_wind();

        # timer
        rospy.Timer(rospy.Duration(0.05), self.timer_callback) # 20 Hz

        rospy.spin()

    def odom_callback(self, msg):

        self.uav_pos = msg.pose.pose.position
        self.uav_pos = np.array([self.uav_pos.x, self.uav_pos.y, self.uav_pos.z])

        if self.uav_init_pos is None:
            self.uav_init_pos = self.uav_pos

    def servo_state_callback(self, msg):

        if msg.name[0] != self.servo_name:
            rospy.logerr_throttle(1.0, "the servo name does not match: {} VS {}".format(msg.name, self.servo_name))
            return

        self.servo_angle = msg.position[0]

        if self.servo_init_angle is None:
            self.servo_init_angle = self.servo_angle

    def reset_wind(self):

        while True:

            if self.servo_angle is not None:
                break

            time.sleep(0.1) # 10Hz

        # impose the winding torque 
        cur_cmd = JointState()
        cur_cmd.name.append(self.servo_name)
        cur_cmd.position.append(0)
        cur_cmd.velocity.append(0)
        cur_cmd.effort.append(self.wind_torque)
        self.servo_torque_cmd_pub.publish(cur_cmd)

        # set the reset angle
        ang_cmd = JointTrajectory()
        ang_cmd.joint_names.append(self.servo_name)
        ang_cmd.points.append(JointTrajectoryPoint())
        ang_cmd.points[0].positions.append(1000) # hard-coding
        ang_cmd.points[0].velocities.append(0)
        ang_cmd.points[0].accelerations.append(0)
        ang_cmd.points[0].effort.append(0)
        self.servo_angle_cmd_pub.publish(ang_cmd)


        # check whether the winding is complete
        thresh = 0.1 # rad
        prev_ang = self.servo_angle
        time.sleep(0.5)

        while True:

            delta = self.servo_angle - prev_ang
            rospy.loginfo("prev ang: {}; cur ang: {}".format(prev_ang, self.servo_angle))

            if np.abs(delta) < 0.05:

                self.servo_init_angle = self.servo_angle
                rospy.loginfo("complete winding reset, init angle is {}".format(self.servo_init_angle));

                # set the reset angle
                ang_cmd = JointTrajectory()
                ang_cmd.joint_names.append(self.servo_name)
                ang_cmd.points.append(JointTrajectoryPoint())
                ang_cmd.points[0].positions.append(self.servo_init_angle)
                ang_cmd.points[0].velocities.append(0)
                ang_cmd.points[0].accelerations.append(0)
                ang_cmd.points[0].effort.append(0)
                self.servo_angle_cmd_pub.publish(ang_cmd)


                # impose the normal
                cur_cmd = JointState()
                cur_cmd.name.append(self.servo_name)
                cur_cmd.position.append(0)
                cur_cmd.velocity.append(0)
                cur_cmd.effort.append(self.normal_torque)
                self.servo_torque_cmd_pub.publish(cur_cmd)

                break

            prev_ang = self.servo_angle

            time.sleep(1.0)


    def timer_callback(self, event):

        if self.uav_init_pos is None:
            return

        # straight distance from takeoff
        d = np.linalg.norm(self.uav_pos - self.uav_init_pos) + self.wire_bias

        # simple model
        ang = self.wire_scale * d + self.servo_init_angle

        # regression model desire angle
        # ang = 0
        # for i, a in enumerate(self.a_vec)):
        #     ang += a * pow(d, i)

        rospy.loginfo_throttle(1.0, "distance: {}; angle: {}".format(d, ang))

        ang_cmd = JointTrajectory()
        ang_cmd.joint_names.append(self.servo_name)
        ang_cmd.points.append(JointTrajectoryPoint())
        ang_cmd.points[0].positions.append(ang) # hard-coding
        ang_cmd.points[0].velocities.append(0)
        ang_cmd.points[0].accelerations.append(0)
        ang_cmd.points[0].effort.append(0)
        self.servo_angle_cmd_pub.publish(ang_cmd)

if __name__=="__main__":
    rospy.init_node("uav_wire_motion")

    node = UavWireMotion()
