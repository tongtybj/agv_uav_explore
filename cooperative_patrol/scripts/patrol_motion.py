#!/usr/bin/env python

import rospy
import smach
import smach_ros
import ros_numpy
import copy
from patrol_state import *


def main():
    rospy.init_node('patrol_motion')
    sm_top = smach.StateMachine(outcomes=['preempted'])

    sm_top.userdata.waypoint_info = rospy.get_param('~waypoint_info', [])
    sm_top.userdata.cnt = 0
    sm_top.userdata.init_position = None

    if len(sm_top.userdata.waypoint_info) == 0:
        rospy.logerr('no valid waypoint info. exit')
        return


    with sm_top:
        smach.StateMachine.add('Start', Start(),
                               transitions = {'succeeded':'RoughApproach'},
                               remapping = {'init_position': 'init_position'})


        smach.StateMachine.add('RoughApproach', RoughApproach(),
                               transitions = {'idle':'Idle',
                                              'failed':'preempted'},
                               remapping = {'waypoint_info': 'waypoint_info',
                                            'cnt': 'cnt'})

        smach.StateMachine.add('Idle', Idle(),
                               transitions={'continue':'RoughApproach'},
                               remapping = {'waypoint_info': 'waypoint_info',
                                            'cnt': 'cnt'})


        sis = smach_ros.IntrospectionServer('patrol_smach_server', sm_top, '/SM_ROOT')
        sis.start()

    outcome = sm_top.execute()

    rospy.spin()
    sis.stop()

if __name__ == '__main__':
    main()
