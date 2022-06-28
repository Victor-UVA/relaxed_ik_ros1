#! /usr/bin/env python3
import rospy
import smach
import smach_ros
from std_msgs.msg import Bool

class DetectState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['detected', 'detecting'])
        pass

    def execute(self, ud):
        if rospy.wait_for_message('/transition', Bool).data:
            return 'detected'
        else:
            return 'undetected'


class PlanState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['planned', 'planning'])
        pass

    def execute(self, ud):
        if rospy.wait_for_message('/transition', Bool).data:
            return 'planned'
        else:
            return 'planning'


class ApproachState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['approached', 'approaching'])
        pass

    def execute(self, ud):
        if rospy.wait_for_message('/transition', Bool).data:
            return 'approached'
        else:
            return 'approaching'


class UnlatchState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['unlatched', 'unlatching'])
        pass

    def execute(self, ud):
        if rospy.wait_for_message('/transition', Bool).data:
            return 'unlatched'
        else:
            return 'unlatching'


class PullState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['pulled', 'pulling'])
        pass

    def execute(self, ud):
        if rospy.wait_for_message('/transition', Bool).data:
            return 'pulled'
        else:
            return 'pulling'


class DisengageState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['disengaged', 'disengaging'])
        pass

    def execute(self, ud):
        if rospy.wait_for_message('/transition', Bool).data:
            return 'disengaged'
        else:
            return 'disengaging'


class PushState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['pushed', 'pushing'])
        pass

    def execute(self, ud):
        if rospy.wait_for_message('/transition', Bool).data:
            return 'pushed'
        else:
            return 'pushing'


class EnterState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['entered', 'entering'])
        pass

    def execute(self, ud):
        if rospy.wait_for_message('/transition', Bool).data:
            return 'entered'
        else:
            return 'entering'


# ["Detect", "Plan", "Approach",
#  "Unlatch", "Pull", "Disengage", "Push", "Enter", "Done"]


def main():
    rospy.init_node("deception_state_machine")

    sm = smach.StateMachine(outcomes=["Done"])

    with sm:
        smach.StateMachine.add('Detect', DetectState(), transitions={
            'detected': 'Plan', 'detecting': 'Detect'})
        smach.StateMachine.add('Plan', PlanState(), transitions={
            'planned': 'Approach', 'planning': 'Plan'})
        smach.StateMachine.add('Approach', ApproachState(), transitions={
            'approached': 'Unlatch', 'approaching': 'Approach'})
        smach.StateMachine.add('Unlatch', UnlatchState(), transitions={
            'unlatched': 'Pull', 'unlatching': 'Unlatch'})
        smach.StateMachine.add('Pull', PullState(), transitions={
            'pulled': 'Disengage', 'pulling': 'Pull'})
        smach.StateMachine.add('Disengage', DisengageState(), transitions={
            'disengaged': 'Push', 'disengaging': 'Disengage'})
        smach.StateMachine.add('Push', PushState(), transitions={
            'pushed': 'Enter', 'pushing': 'Push'})
        smach.StateMachine.add('Enter', EnterState(), transitions={
            'entered': 'Done', 'entering': 'Enter'})

    sis = smach_ros.IntrospectionServer('server', sm, '/SM_ROOT')
    sis.start()

    outcome = sm.execute()
    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()
