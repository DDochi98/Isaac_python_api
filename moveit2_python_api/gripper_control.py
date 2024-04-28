#!/usr/bin/env python3
"""
A script to outline the fundamentals of the moveit_py motion planning API.
"""

import time

# generic ros libraries
import rclpy
from rclpy.logging import get_logger

# moveit python library
from moveit.core.robot_state import RobotState
from moveit.planning import (
    MoveItPy,
    MultiPipelinePlanRequestParameters,
)


def plan_and_execute(
    robot,
    planning_component,
    logger,
    single_plan_parameters=None,
    multi_plan_parameters=None,
    sleep_time=0.0,
):
    """Helper function to plan and execute a motion."""
    # plan to goal
    logger.info("Planning trajectory")
    if multi_plan_parameters is not None:
        plan_result = planning_component.plan(
            multi_plan_parameters=multi_plan_parameters
        )
    elif single_plan_parameters is not None:
        plan_result = planning_component.plan(
            single_plan_parameters=single_plan_parameters
        )
    else:
        plan_result = planning_component.plan()

    # execute the plan
    if plan_result:
        logger.info("Executing plan")
        robot_trajectory = plan_result.trajectory
        robot.execute(robot_trajectory, blocking=True, controllers=[])
    else:
        logger.error("Planning failed")

    time.sleep(sleep_time)


def main():

    ###################################################################
    # MoveItPy Setup
    ###################################################################
    rclpy.init()
    logger = get_logger("moveit_py.pose_goal")

    # instantiate MoveItPy instance and get planning component
    panda = MoveItPy(node_name="moveit_py")
    panda_arm = panda.get_planning_component("panda_arm")
    panda_arm_hand = panda.get_planning_component("panda_arm_hand")
    panda_hand = panda.get_planning_component("hand")

    logger.info("MoveItPy instance created")

    ###########################################################################
    # Plan 1 - set states with predefined string
    ###########################################################################
    time.sleep(1)
    logger.info("------------Ready------------")

    # set plan start state using predefined state
    # panda_arm (o), panda_arm_hand (x)
    panda_arm.set_start_state(configuration_name="ready")

    # set pose goal using predefined state
    panda_arm.set_goal_state(configuration_name="extended")

    # plan to goal
    plan_and_execute(panda, panda_arm, logger, sleep_time=3.0)

    ###########################################################################
    # Plan 4 - set goal state with constraints
    ###########################################################################
    time.sleep(1)
    logger.info("------------Open------------")

    # set plan start state to current state
    panda_hand.set_start_state_to_current_state()

    # set constraints message
    from moveit.core.kinematic_constraints import construct_joint_constraint

    joint_values = {
        "panda_finger_joint1": 0.04,
        "panda_finger_joint2": 0.04,
    }

    robot_model = panda.get_robot_model()
    robot_state = RobotState(robot_model)

    robot_state.joint_positions = joint_values
    joint_constraint = construct_joint_constraint(
        robot_state=robot_state,
        joint_model_group=panda.get_robot_model().get_joint_model_group("hand"),
    )
    panda_hand.set_goal_state(motion_plan_constraints=[joint_constraint])

    # plan to goal
    plan_and_execute(panda, panda_hand, logger, sleep_time=3.0)


if __name__ == "__main__":
    main()

