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
rospy.init_node("CEM_evaluation")
hand_interface = Allegro_hand_interface()
hand_qpos = [0]*16
hand_interface.set_position(np.array(hand_qpos).astype(np.float64))
hand_qpos_2 = [0,-0.328,-0.209,0.154,0,-0.3,-0.2,0.1,0,-0.24,-0.2,0,0,0.47,-0.46,-0.3]
hand_interface.set_position((-3)*np.array(hand_qpos_2).astype(np.float64))