#!/usr/bin/env python

import rospy
import smach
import smach_ros
import numpy as np
import tf.transformations as tft
import tf2_ros
import ros_numpy as ros_np
import copy

from std_msgs.msg import Empty, UInt8
from geometry_msgs.msg import PoseStamped, Quaternion, Twist
from move_base_msgs.msg import MoveBaseActionResult
from actionlib_msgs.msg import GoalStatus
from nav_msgs.msg import Odometry
from move_base_msgs.msg import MoveBaseActionGoal

class Start(smach.State):
    def __init__(self):

        smach.State.__init__(self,
                           outcomes=['succeeded', 'preempted'],
                           io_keys=['init_position'])

        self.task_start = False
        self.task_start_sub = rospy.Subscriber('task_start', Empty, self.taskStartCallback)
        self.base_frame = rospy.get_param("~base_frame", "base_footprint")
        self.map_frame = rospy.get_param("~map_frame", "map")
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

    def taskStartCallback(self, msg):
        self.task_start = True

    def execute(self, userdata):

        while not self.task_start:
            rospy.sleep(0.1)
            rospy.logdebug_throttle(1.0, "wait to start task")

            if rospy.is_shutdown():
                return 'preempted'

            pass

        while not rospy.is_shutdown():

            try:
                trans = self.tf_buffer.lookup_transform(self.map_frame, self.base_frame, rospy.Time.now(), rospy.Duration(1.0))
                trans = ros_np.numpify(trans.transform)

                break
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                continue


        # get the initial position of AGV
        userdata.init_position = tft.translation_from_matrix(trans)

        return 'succeeded'


class RoughApproach(smach.State):
    def __init__(self):

        smach.State.__init__(self,
                           outcomes=['precise_approach', 'idle', 'failed'],
                           io_keys=['waypoint_info', 'cnt'])

        self.goal_pub = rospy.Publisher("/move_base/goal", MoveBaseActionGoal, queue_size = 1)
        self.move_base_result_sub = rospy.Subscriber("/move_base/result", MoveBaseActionResult, self.waypointCallback)

        self.reach = None


    def waypointCallback(self, msg):

        rospy.logdebug("move base action status: %s", msg.status.text)

        if msg.status.status == GoalStatus.SUCCEEDED:
            self.reach = True
        else:
            self.reach = False

    def execute(self, userdata):

        # send the target position to AGV
        waypoint = userdata.waypoint_info[userdata.cnt]

        if not (len(waypoint) == 2 or len(waypoint) == 3):
            # invalid waypoint
            rospy.logerr("the waypoint{} is invalid: {}".format(userdata.cnt, waypoint))
            return 'failed'

        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.seq = userdata.cnt + 1
        goal.header.stamp = rospy.Time.now()
        goal.pose.position.x = waypoint[0]
        goal.pose.position.y = waypoint[1]
        goal.pose.orientation.w = 1
        action_goal = MoveBaseActionGoal()
        action_goal.header = goal.header
        action_goal.goal.target_pose = goal
        self.goal_pub.publish(action_goal)
        self.reach = None

        # check the convergence

        while not rospy.is_shutdown():

            if self.reach is not None:

                if self.reach:

                    if len(waypoint) == 2:
                        # only rough position
                        self.reach = None
                        return 'idle'
                    else:
                        # precise position and yaw
                        self.reach = None
                        return 'precise_approach'
                else:
                    self.reach = None
                    return 'failed'

                break

            rospy.loginfo_throttle(1.0, "[Rough Approach] move to waypoint{}: [{}, {}]".format(userdata.cnt + 1, waypoint[0], waypoint[1]))
            rospy.sleep(0.1)


class PreciseApproach(smach.State):
    def __init__(self):

        smach.State.__init__(self,
                             outcomes=['idle', 'failed'],
                             io_keys=['waypoint_info', 'cnt'])

        self.agv_frame = rospy.get_param("~agv/frame", "base_footprint")
        self.map_frame = rospy.get_param("~map_frame", "map")
        self.pos_k_gain = rospy.get_param("~pos_k_gain", 0.0)
        self.yaw_k_gain = rospy.get_param("~yaw_k_gain", 0.0)
        self.max_linear_vel = rospy.get_param("~max_linear_vel", 1.0)
        self.max_angular_vel = rospy.get_param("~max_angular_vel", 1.0)

        self.time_thresh = rospy.get_param("~time_thresh", -1)
        self.pos_thresh = rospy.get_param("~pos_thresh", 0.01)
        self.yaw_thresh = rospy.get_param("~yaw_thresh", 0.01)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)


    def transform(self, pos, yaw):
        pose = tft.concatenate_matrices(tft.euler_matrix(0, 0, yaw), \
                                        tft.translation_matrix(np.append(pos, 0)))
        return tft.translation_from_matrix(pose)[:2]

    def execute(self, userdata):

        # send the target position to AGV
        waypoint = userdata.waypoint_info[userdata.cnt]

        start_t = rospy.Time.now()

        if not len(waypoint) == 3:
            rospy.logwarn("the waypoint {} is not suitable for precise appraoch".format(waypoint))
            return 'idle'

        target_pos = np.array(waypoint[:2])
        target_yaw = waypoint[2]

        while not rospy.is_shutdown():

            try:
                trans = self.tf_buffer.lookup_transform(self.map_frame, self.agv_frame, rospy.Time.now(), rospy.Duration(0.2))
                trans = ros_np.numpify(trans.transform)

            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                continue

            # get the AGV position
            curr_pos = tft.translation_from_matrix(trans)[:2]
            curr_yaw = tft.euler_from_matrix(trans)[2]

            diff_pos = target_pos - curr_pos
            # transform by yaw
            diff_pos = self.transform(diff_pos, -curr_yaw)

            diff_yaw = target_yaw - curr_yaw

            rospy.loginfo("[Precise Approach] curr pos: {}; curr yaw: {}, target pos: {}, target yaw: {}".format(curr_pos, curr_yaw, target_pos, target_yaw))

            if np.linalg.norm(diff_pos) < self.pos_thresh and np.fabs(diff_yaw) < self.yaw_thresh:
                rospy.loginfo("[Precise Approach] reach the goal")

                # send a idle vel cmd
                msg = Twist()
                self.pub.publish(msg)

                return 'idle'

            t = rospy.Time.now()
            if self.time_thresh > 0 and t - start_t > rospy.Duration(self.time_thresh):

                rospy.loginfo("[Precise Approach] time out")

                # send a idle vel cmd
                msg = Twist()
                self.pub.publish(msg)
                return 'failed'

            trans_vel = self.pos_k_gain * diff_pos
            rot_vel   = self.yaw_k_gain * diff_yaw

            vel_norm = np.linalg.norm(trans_vel)
            if vel_norm > self.max_linear_vel:
                trans_vel = (trans_vel / vel_norm * self.max_linear_vel)

            if rot_vel > self.max_angular_vel:
                rot_vel = self.max_angular_vel
            if rot_vel < -self.max_angular_vel:
                rot_vel = -self.max_angular_vel

            msg = Twist()
            msg.linear.x = trans_vel[0]
            msg.linear.y = trans_vel[1]
            msg.angular.z = rot_vel

            rospy.loginfo("[Precise Approach] target vel: {}, target omega: {}".format(trans_vel, rot_vel))
            self.pub.publish(msg)


class Idle(smach.State):
    def __init__(self):

        smach.State.__init__(self,
                           outcomes=['continue', 'preempted'],
                           io_keys=['waypoint_info', 'cnt'])

        self.idle_duration = rospy.get_param('~idle_duration', 2.0)

    def execute(self, userdata):

       rospy.sleep(self.idle_duration)

       userdata.cnt += 1

       if userdata.cnt == len(userdata.waypoint_info):
           return 'preempted'
       else:
           return 'continue'
