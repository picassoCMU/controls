import sys
sys.path.append("/home/ros_ws")

import time
from geometry_msgs.msg import Pose
from collections import OrderedDict
from src.devel_packages.manipulation.src.moveit_class import MoveItPlanner


RECTANGLE_POSES = OrderedDict(
    br=dict(
        tra=[0.4, 0.0, 0.05],
        quat=[0.0, 1.0,  0.0,  0.0],
    ),
    bl=dict(
        tra=[0.4, 0.2, 0.05],
        quat=[0.0, 1.0,  0.0,  0.0],
    ),
    tl=dict(
        tra=[0.52, 0.2, 0.05],
        quat=[0.0, 1.0,  0.0,  0.0],
    ),
    tr=dict(
        tra=[0.52, 0.0, 0.05],
        quat=[0.0, 1.0,  0.0,  0.0],
    ),
)

def to_pose(tra, quat):
    """Packs tra and quat lists into Pose message."""
    pose = Pose()
    pose.position.x = tra[0]
    pose.position.y = tra[1]
    pose.position.z = tra[2]
    pose.orientation.w = quat[0]
    pose.orientation.x = quat[1]
    pose.orientation.y = quat[2] 
    pose.orientation.z = quat[3]
    return pose


def move_robot(pose_goal, robot: MoveItPlanner):
    """Moves the robot to given pose"""
    # Convert pose goal to the panda_hand frame (the frame that moveit uses)
    pose_goal_moveit = robot.get_moveit_pose_given_frankapy_pose(pose_goal)

    # Plan a straight line motion to the goal
    plan = robot.get_straight_plan_given_pose(pose_goal_moveit)

    # Execute the plan
    robot.execute_plan(plan)

    # Temporary patch for 'Can't perform multiple skills bug'
    robot.fa.stop_skill()



if __name__ == '__main__':

    # Create a MoveItPlanner object and start the moveit node
    franka_moveit = MoveItPlanner()

    for pose_name, pose_data in RECTANGLE_POSES.items():
        print(f'Going to: {pose_name}')
        goal_pose = to_pose(**pose_data)
        move_robot(goal_pose, franka_moveit)

    print('Done! :)')