#! /usr/bin/env python3
from click import getchar
import rospy
import smach
import smach_ros


class PlanState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['planned', 'planning'])

    def execute(self, ud):
        getchar()
        return 'planned'


class PullState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['pulled', 'pulling'])

    def execute(self, ud):
        getchar()
        return 'pulled'


class PushState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['pushed', 'pushing'])

    def execute(self, ud):
        getchar()
        return 'pushed'


class EnterState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['entered', 'entering'])

    def execute(self, ud):
        getchar()
        return 'entered'


class PatrolState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['received task', 'patrolling'])

    def execute(self, ud):
        getchar()
        return 'received task'


class NavigateState(smach.State):
    def __init__(self):
        smach.State.__init__(
            self, outcomes=['disinfection', 'manipulation', 'inspection'])

    def execute(self, ud):
        getchar()
        return 'manipulation'


class CompletedState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['completed'])

    def execute(self, ud):
        getchar()
        return 'completed'


def main():
    rospy.init_node("deception_state_machine")

    sm = smach.StateMachine(outcomes=[])
    disinfection_sm = smach.StateMachine(outcomes=['completed'])
    inspection_sm = smach.StateMachine(outcomes=['completed'])
    manipulation_sm = smach.StateMachine(outcomes=['completed'])
    welding_sm = smach.StateMachine(outcomes=['completed'])
    open_door_sm = smach.StateMachine(outcomes=['completed'])

    with manipulation_sm:
        smach.StateMachine.add('Plan', PlanState(), transitions={
            'planned': 'Pull', 'planning': 'Plan'})
        smach.StateMachine.add('Pull', PullState(), transitions={
            'pulled': 'Push', 'pulling': 'Pull'})
        smach.StateMachine.add('Push', PushState(), transitions={
            'pushed': 'Enter', 'pushing': 'Push'})
        smach.StateMachine.add('Enter', EnterState(), transitions={
            'entered': 'completed', 'entering': 'Enter'})

    with sm:
        smach.StateMachine.add('Patrol', PatrolState(), transitions={
            'received task': 'Navigate', 'patrolling': 'Patrol'})
        smach.StateMachine.add('Navigate', NavigateState(), transitions={
            'disinfection': 'Disinfection', 'manipulation': 'Manipulation', 'inspection': 'Inspection'})
        smach.StateMachine.add('Disinfection', disinfection_sm,
                               transitions={'completed': 'Patrol'})
        smach.StateMachine.add('Manipulation', manipulation_sm,
                               transitions={'completed': 'Patrol'})
        smach.StateMachine.add('Inspection', inspection_sm,
                               transitions={'completed': 'Patrol'})

    sis = smach_ros.IntrospectionServer('server', sm, '/SM_ROOT')
    sis.start()

    outcome = sm.execute()
    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()
