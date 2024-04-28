#!/usr/bin/env python3
"""
A script to outline the fundamentals of the moveit_py motion planning API.
"""

import time

# generic ros libraries
import rclpy
from rclpy.logging import get_logger
from rclpy.node import Node

# set pose goal with PoseStamped message
from geometry_msgs.msg import PoseStamped

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

pose_goal = PoseStamped()

def pick_pose(pose_msg):
    global pose_goal
    pose_goal = pose_msg
    
    pose_goal.header.frame_id = "panda_link0"
    
    pose_goal.pose.orientation.x = 0.922604 
    pose_goal.pose.orientation.y = -0.385712 
    pose_goal.pose.orientation.z = -0.00372501 
    pose_goal.pose.orientation.w = 0.0037637 

    #x = pose_msg.pose.position.x
    #y = pose_msg.pose.position.y
    #z = pose_msg.pose.position.z
    #print(f"Planning and executing motion to position: x={x}, y={y}, z={z}")

def main(args=None):
    
    ###################################################################
    # MoveItPy Setup
    ###################################################################
    rclpy.init(args=args)
    logger = get_logger("moveit_py.pose_goal")
    node = Node('subscriber_node')

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
    # panda_arm.set_goal_state(configuration_name="extended")
    panda_arm.set_goal_state(configuration_name="ready")    

    # plan to goal
    plan_and_execute(panda, panda_arm, logger, sleep_time=3.0)
    ###########################################################################

    def pose_callback(msg):
        pick_pose(msg)

    subscriber = node.create_subscription(PoseStamped, 'pose_topic', pose_callback, 10)

    try:
        while rclpy.ok():
            rclpy.spin_once(node)

            time.sleep(2)
            logger.info("2-1------------Move(Forward)------------")
            panda_arm.set_start_state_to_current_state()
            panda_arm.set_goal_state(pose_stamped_msg=pose_goal, pose_link="panda_link8")
            plan_and_execute(panda, panda_arm, logger, sleep_time=3.0)

    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()
