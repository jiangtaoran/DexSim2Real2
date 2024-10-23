import os
import sys
sys.path.append("/home/rvsa/CEM/ManiSkill2/mani_skill2")

import h5py
import numpy as np
import rospy
import time
import moveit_commander
from geometry_msgs.msg import Pose
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
#sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), 'external')))

TRAJ_NUM = "task5_16_2"
current_pth = os.getcwd()
h5_pth = current_pth + f'/work_dirs/CEM-allegro-v0/{TRAJ_NUM}/trajectory.h5'
print(h5_pth)
with h5py.File(h5_pth, 'r') as f:
    trajectory = np.array(f["traj_0"]["dict_str_env_states"])
'''
step=20
traj = trajectory[step]
traj_next = trajectory[step+1]
# print(traj)
# print(np.size(np.array(traj)))
arm_qpos = traj[65:72]
hand_qpos = traj[72:88]

# print("arm_qpos",arm_qpos)
# print("hand_qpos",hand_qpos)
arm_qvel = traj[88:95]
hand_qvel = traj[95:111]
hand_qvel_next = traj_next[95:111]
hand_delta_qvel  = (hand_qvel_next - hand_qvel)
print("jiajiasudu",hand_delta_qvel)
'''
hand_3d_total = np.array(list([0.0]*16))

for step in range(30):
    hand_qpos_0  = trajectory[step][72:88]
    hand_qpos_1 = trajectory[step+1][72:88]
    hand_qpos_2 = trajectory[step+2][72:88]
    hand_qpos_3 = trajectory[step+3][72:88]
    hand_qvel_0 = hand_qpos_1 - hand_qpos_0
    hand_qvel_1 = hand_qpos_2-hand_qpos_1
    hand_qvel_2 = hand_qpos_3 - hand_qpos_2
    hand_qacc_0 = hand_qvel_1 - hand_qvel_0
    hand_qacc_1 = hand_qvel_2 - hand_qvel_1
    hand_qjerk_0 = hand_qacc_1 - hand_qacc_0
    hand_qjerk = np.array(list(map(abs,hand_qjerk_0)))
    #print(hand_delta_qvel_abs)
    hand_3d_total +=hand_qjerk

'''
for step in range(30):
    hand_qvel_0  = trajectory[step][95:111]
    hand_qvel_1 = trajectory[step+1][95:111]
    hand_qvel_2 = trajectory[step+2][95:111]
    hand_qacc_0 = hand_qvel_1 - hand_qvel_0
    hand_qacc_1 = hand_qvel_2-hand_qvel_1
    
    hand_qjerk_0 = hand_qacc_1 - hand_qacc_0
    hand_qjerk = np.array(list(map(abs,hand_qjerk_0)))
    if (step==20):
        print(hand_qjerk*400)
    # print("hand_qjerk_0",hand_qjerk_0)
    # print("hand_qjerk",hand_qjerk)
    #print(hand_delta_qvel_abs)
    hand_3d_total +=hand_qjerk
'''
print(TRAJ_NUM)
three_finger_first = hand_3d_total[0]+hand_3d_total[4]+hand_3d_total[8]
three_finger_second = hand_3d_total[1]+hand_3d_total[5]+hand_3d_total[9]
three_finger_third = hand_3d_total[2]+hand_3d_total[6]+hand_3d_total[10]
three_finger_fourth = hand_3d_total[3]+hand_3d_total[7]+hand_3d_total[11]
thumb = hand_3d_total[12]+hand_3d_total[13]+hand_3d_total[14]+hand_3d_total[15]
index = hand_3d_total[0]+hand_3d_total[1]+hand_3d_total[2]+hand_3d_total[3]
middle = hand_3d_total[4]+hand_3d_total[5]+hand_3d_total[6]+hand_3d_total[7]
ring = hand_3d_total[8]+hand_3d_total[9]+hand_3d_total[10]+hand_3d_total[11]
hand_3d_output = [thumb/4, three_finger_first/3,three_finger_second/3,three_finger_third/3,three_finger_fourth/3]
hand_3d_output2 = np.array([thumb,index,middle,ring])*400/30
print(list(hand_3d_output2))