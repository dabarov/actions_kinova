#!/usr/bin/env python

import rospy
import smach
from .constants import outcomes, states, transitions


class HomePosition(smach.State):
    def __init__(self):
        smach.State.__init__(
            self, outcomes=[outcomes.OUTCOME_SUCCESS, outcomes.OUTCOME_FAILURE]
        )

    def execute(self, userdata):
        return outcomes.OUTCOME_SUCCESS


class Explore(smach.State):
    def __init__(self):
        smach.State.__init__(
            self, outcomes=[outcomes.OUTCOME_SUCCESS, outcomes.OUTCOME_FAILURE]
        )

    def execute(self, userdata):
        return outcomes.OUTCOME_SUCCESS


class Approach(smach.State):
    def __init__(self):
        smach.State.__init__(
            self, outcomes=[outcomes.OUTCOME_SUCCESS, outcomes.OUTCOME_FAILURE]
        )

    def execute(self, userdata):
        return outcomes.OUTCOME_SUCCESS


class Grab(smach.State):
    def __init__(self):
        smach.State.__init__(
            self, outcomes=[outcomes.OUTCOME_SUCCESS, outcomes.OUTCOME_FAILURE]
        )

    def execute(self, userdata):
        return outcomes.OUTCOME_SUCCESS


def main():
    rospy.init_node("actions_kinova_smach")

    # Create a SMACH state machine
    sm = smach.StateMachine(
        outcomes=[outcomes.OUTCOME_FINISHED_SUCCESS, outcomes.OUTCOME_FINISHED_FAILURE]
    )

    # Open the container
    with sm:
        # Add states to the container

        smach.StateMachine.add(
            label=states.STATE_HOME_POSITION,
            state=HomePosition(),
            transitions=transitions.TRANSITIONS_HOME_POSITION,
        )

        smach.StateMachine.add(
            label=states.STATE_EXPLORE,
            state=Explore(),
            transitions=transitions.TRANSITIONS_EXPLORE,
        )

        smach.StateMachine.add(
            label=states.STATE_APPROACH,
            state=Approach(),
            transitions=transitions.TRANSITIONS_APPROACH,
        )

        smach.StateMachine.add(
            label=states.STATE_GRAB,
            state=Grab(),
            transitions=transitions.TRANSITIONS_GRAB,
        )

    outcome = sm.execute()
    if outcome == outcomes.OUTCOME_FINISHED_SUCCESS:
        rospy.loginfo("The task was successfully finished")
    elif outcome == outcomes.OUTCOME_FINISHED_FAILURE:
        rospy.loginfo("The task failed")


if __name__ == "__main__":
    main()
