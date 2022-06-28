from .outcomes import (
    OUTCOME_SUCCESS,
    OUTCOME_FAILURE,
    OUTCOME_FINISHED_SUCCESS,
    OUTCOME_FINISHED_FAILURE,
)
from .states import STATE_APPROACH, STATE_EXPLORE, STATE_GRAB, STATE_HOME_POSITION

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
