#!/usr/bin/env python

import rospy
import smach

STATE_HOME_POSITION = "HOME_POSITION"
STATE_EXPLORE = "EXPLORE"
STATE_APPROACH = "APPROACH"
STATE_GRAB = "GRAB"

OUTCOME_FAILURE = "FAILURE"
OUTCOME_SUCCESS = "SUCCESS"
OUTCOME_FINISHED_SUCCESS = "FINISHED_SUCCESS"
OUTCOME_FINISHED_FAILURE = "FINISHED_FAILURE"

TRANSITIONS_HOME_POSITION = {
    OUTCOME_SUCCESS: STATE_EXPLORE,
    OUTCOME_FAILURE: STATE_HOME_POSITION,
}

TRANSITIONS_EXPLORE = {
    OUTCOME_SUCCESS: STATE_APPROACH,
    OUTCOME_FAILURE: STATE_HOME_POSITION,
}

TRANSITIONS_APPROACH = {
    OUTCOME_SUCCESS: STATE_GRAB,
    OUTCOME_FAILURE: STATE_HOME_POSITION,
}

TRANSITIONS_GRAB = {
    OUTCOME_SUCCESS: OUTCOME_FINISHED_SUCCESS,
    OUTCOME_FAILURE: OUTCOME_FINISHED_FAILURE,
}


class HomePosition(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=[OUTCOME_SUCCESS, OUTCOME_FAILURE])

    def execute(self, userdata):
        return OUTCOME_SUCCESS


class Explore(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=[OUTCOME_SUCCESS, OUTCOME_FAILURE])

    def execute(self, userdata):
        return OUTCOME_SUCCESS


class Approach(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=[OUTCOME_SUCCESS, OUTCOME_FAILURE])

    def execute(self, userdata):
        return OUTCOME_SUCCESS


class Grab(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=[OUTCOME_SUCCESS, OUTCOME_FAILURE])

    def execute(self, userdata):
        return OUTCOME_SUCCESS


def main():
    rospy.init_node("actions_kinova_smach")

    # Create a SMACH state machine
    sm = smach.StateMachine(
        outcomes=[OUTCOME_FINISHED_SUCCESS, OUTCOME_FINISHED_FAILURE]
    )

    # Open the container
    with sm:
        # Add states to the container

        smach.StateMachine.add(
            label=STATE_HOME_POSITION,
            state=HomePosition(),
            transitions=TRANSITIONS_HOME_POSITION,
        )

        smach.StateMachine.add(
            label=STATE_EXPLORE,
            state=Explore(),
            transitions=TRANSITIONS_EXPLORE,
        )

        smach.StateMachine.add(
            label=STATE_APPROACH,
            state=Approach(),
            transitions=TRANSITIONS_APPROACH,
        )

        smach.StateMachine.add(
            label=STATE_GRAB,
            state=Grab(),
            transitions=TRANSITIONS_GRAB,
        )

    outcome = sm.execute()
    if outcome == OUTCOME_FINISHED_SUCCESS:
        rospy.loginfo("The task was successfully finished")
    elif outcome == OUTCOME_FINISHED_FAILURE:
        rospy.loginfo("The task failed")


if __name__ == "__main__":
    main()
