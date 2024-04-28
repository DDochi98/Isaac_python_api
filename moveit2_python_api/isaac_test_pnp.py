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

    def g_open():
        #logger.info("------------Open------------")

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
    
    def g_close():
        #logger.info("------------Open------------")

        # set plan start state to current state
        panda_hand.set_start_state_to_current_state()

        # set constraints message
        from moveit.core.kinematic_constraints import construct_joint_constraint

        joint_values = {
            "panda_finger_joint1": -0.02,
            "panda_finger_joint2": -0.02,
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

    def pose_callback(msg):
        pick_pose(msg)
        
        time.sleep(1)
        logger.info("2-1------------Move(Forward)------------")
        panda_arm.set_start_state_to_current_state()
        
        link8_gripper_end = 0.1

        pose_goal.pose.position.z = pose_goal.pose.position.z + link8_gripper_end + 0.1
        panda_arm.set_goal_state(pose_stamped_msg=pose_goal, pose_link="panda_link8")
        plan_and_execute(panda, panda_arm, logger, sleep_time=3.0)

        g_open()
        
        #cube size 0.03 x 0.03 x 0.03
        #cube_size = 0.03
        logger.info("2-2------------Move(Down)------------")
        panda_arm.set_start_state_to_current_state()
        pose_goal.pose.position.z -= 0.1
        panda_arm.set_goal_state(pose_stamped_msg=pose_goal, pose_link="panda_link8")
        plan_and_execute(panda, panda_arm, logger, sleep_time=3.0)
        
        g_close()

        logger.info("2-3------------Move(Up)------------")
        panda_arm.set_start_state_to_current_state()

        pose_goal.pose.position.z += 0.2
        panda_arm.set_goal_state(pose_stamped_msg=pose_goal, pose_link="panda_link8")
        plan_and_execute(panda, panda_arm, logger, sleep_time=3.0)

        
    subscriber = node.create_subscription(PoseStamped, 'pose_topic', pose_callback, 10)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()
