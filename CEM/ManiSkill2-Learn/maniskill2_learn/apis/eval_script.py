import os
import sys
sys.path.append("/home/thu-xulab/CEM/ManiSkill2/mani_skill2")
from utils.eigengrasp import EigenGrasp
import h5py
import numpy as np
import rospy
import time
import moveit_commander
from geometry_msgs.msg import Pose
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
#sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), 'external')))

from external.xMate3_interface import Joint_control_interface, DEFAULT_JOINT_IMPEDANCE
from external.allegro_hand_interface import Allegro_hand_interface




from numpy.typing import NDArray
from typing import Literal, List, Dict, Any

def recover_action(action, limit):
    action = (action + 1) / 2 * (limit[:, 1] - limit[:, 0]) + limit[:, 0]
    return action

velocity_limit = np.array([1] * 7 + [np.pi] * 16)

VELOCITY_LIMIT = np.stack([-velocity_limit, velocity_limit], axis=1)
MAX_STEPS = 30
TRAJ_NUM = "20231211_194838"
EIGEN_DIM = 7
CONTROL_FREQ = 20

JOINT_POS_MIN: [-0.47,-0.196,-0.174,-0.227,
                      -0.47,-0.196,-0.174,-0.227,
                      -0.47,-0.196,-0.174,-0.227,
                      0.263,-0.105,-0.189,-0.162]
JOINT_POS_MAX: [0.47,1.61,1.709,1.618,
                      0.47,1.61,1.709,1.618,
                      0.47,1.61,1.709,1.618,
                      1.396,1.163,1.644,1.719]

#INITIAL_QPOS = np.array(list(np.array([0,10,0,91,0,35,0])*np.pi/180) + [0.0]*16 )
#INITIAL_QPOS = np.array([-0.488665, 0.340129, -1.14341, 1.18789, 0.346452, 1.73497, -1] + [0.0]*16 )
#[-27.9984421   19.48795619 -65.51256725  68.06108353  19.8502374 99.40645858 -57.29577951]
INITIAL_QPOS = np.array(list(np.array([0,0,0,40,0,35,0])*np.pi/180) + [0.0]*16 )
current_pth = os.getcwd()
h5_pth = current_pth + f'/work_dirs/CEM-allegro-v0/{TRAJ_NUM}/trajectory.h5'
print(h5_pth)
with h5py.File(h5_pth, 'r') as f:
    trajectory = np.array(f["traj_0"]["dict_str_env_states"])

if __name__ == "__main__":
    #initialize controlling port
    rospy.init_node("CEM_evaluation")
    #arm_interface = Joint_control_interface(DEFAULT_JOINT_IMPEDANCE)
    
    #set initial position
    #arm_interface.exec_arm(INITIAL_QPOS[:7])
    
    robot = moveit_commander.RobotCommander()
    moveit_commander.roscpp_initialize(sys.argv)
    arm = moveit_commander.MoveGroupCommander('xmate_arm')
    
    #velocity limit
    arm.set_max_velocity_scaling_factor(0.1)
    arm.set_max_acceleration_scaling_factor(0.1)

    end_effector_link = arm.get_end_effector_link()
    
    hand_interface = Allegro_hand_interface()
    # #initialize scene
    # scene = moveit_commander.PlanningSceneInterface()
    # # set camera into the scene
    # camera_size = [1, 0.4, 1.4]
    # camera_pose = Pose()
    # camera_pose.header.frame_id = "base_link"
    # camera_pose.pose.position.x = 0.5
    # camera_pose.pose.position.y = -0.6
    # camera_pose.pose.position.z = 0.7
    # camera_pose.pose.orientation.w = 1.0
    # scene.add_box('camera', camera_pose, camera_size)
    # #set table into the scene
    # table_size = [0.2, 1.2, 1]
    # table_pose = Pose()
    # table_pose.header.frame_id = "base_link"
    # table_pose.pose.position.x = -0.3
    # table_pose.pose.position.y = -0.2
    # table_pose.pose.position.z = 0.5
    # table_pose.pose.orientation.w = 1.0
    # scene.add_box("table", table_pose, table_size)
    # #set ground into the scene
    # ground_size = [2, 2, 0.2]
    # ground_pose = Pose()
    # ground_pose.header.frame_id = "base_link"
    # ground_pose.pose.position.x = 0.45
    # ground_pose.pose.position.y = -0.45
    # ground_pose.pose.position.z = -0.12
    # ground_pose.pose.orientation.w = 1.0
    # scene.add_box("ground", ground_pose, ground_size)

    # set touch initial position
    
    arm.set_joint_value_target(np.array([0,0,0,40,0,35,0])*np.pi/180)
    arm.go()
    arm.set_start_state_to_current_state()
    
    #hand_interface.set_position(INITIAL_QPOS[7:])
    print("set initial position is done")
    time.sleep(1)
    # grasp_model_path = "/home/thu-xulab/EigenGrasp/grasp_model.pkl"
    # eigengrasp_model = EigenGrasp(16,7).load_from_file(grasp_model_path)
    
    

    

    

    step: int = 1
    hand_qpos = INITIAL_QPOS[7:]

    arm_qpos = INITIAL_QPOS[:7]
    while step < 20:
        traj = trajectory[step]
        # # hand_velocity = eigengrasp_model.compute_grasp(traj[7:])
        # # hand_velocity = recover_action(hand_velocity, VELOCITY_LIMIT[7:])
        # # hand_action = hand_velocity/CONTROL_FREQ
        # # hand_action = eigengrasp_model.compute_grasp(traj[7:])


        # arm_velocity = traj[:7]
        # arm_velocity = recover_action(arm_velocity, VELOCITY_LIMIT[:7])
        # arm_action = arm_velocity/CONTROL_FREQ

        # arm_qpos = list(np.array(arm_qpos) + np.array(arm_action))
        # # hand_qpos = list(np.array(hand_qpos) + np.array(hand_action))
        # hand_qpos = hand_action
        # print("step:",step)
        # print("arm_qpos:", list(np.array(arm_qpos)*180/np.pi))
        # print("hand_qpos:", list(np.array(hand_qpos)*180/np.pi))
        # #input()
        # # arm_interface.exec_arm(traj[:7])``

        arm_qpos = traj[65:72]
        hand_qpos = traj[72:88]
        hand_qpos = np.clip(hand_qpos,JOINT_POS_MIN,JOINT_POS_MAX)
        print("step:",step)
        print("arm_qpos:", list(np.array(arm_qpos)*180/np.pi))
        print("hand_qpos:", list(np.array(hand_qpos)*180/np.pi))
        input()
        
        arm.set_joint_value_target(np.array(arm_qpos).astype(np.float64))
        arm.go()
        
        hand_interface.set_position(np.array(hand_qpos).astype(np.float64))
        step+=1

    # rospy.init_node("test")
    # arm_interface = Joint_control_interface(DEFAULT_JOINT_IMPEDANCE)
    # arm_interface.exec_arm([0,0,0,0,0,0,0])
