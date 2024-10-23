import h5py
import os
import numpy as np
import sys
sys.path.append("/home/thu-xulab/CEM/ManiSkill2/mani_skill2")
from utils.eigengrasp import EigenGrasp
current_pth = os.getcwd()
h5_pth = current_pth + '/work_dirs/CEM-allegro-v0/20231207_201000/trajectory.h5'
with h5py.File(h5_pth, 'r') as f:
    for group in f.keys():
         print(group)
         print(np.array(f[group]))
    #print(np.array(f["traj_0"]["dict_str_actions"][0]))
    print(np.array(f["traj_0"]["dict_str_env_states"][0]))
    print(np.shape(np.array(f["traj_0"]["dict_str_env_states"])))
    qpos = np.array(f["traj_0"]["dict_str_actions"][0])
    print(qpos)
grasp_model_path = "/home/thu-xulab/EigenGrasp/grasp_model.pkl"
eigengrasp_model = EigenGrasp(16,7).load_from_file(grasp_model_path)
#print(eigengrasp_model.compute_grasp(qpos[7:]))
#print(np.array(list(np.array([-16,42,0,53,0,75,0])*np.pi/180) + [0.0]*7))
