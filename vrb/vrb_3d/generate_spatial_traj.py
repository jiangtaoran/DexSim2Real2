import sys
sys.path.append("..")
import numpy as np
import cv2
import argparse
import os
import random
import torch
import sapien.core as sapien
from sapien.utils.viewer import Viewer
from camera import RgbAndMesh
from networks.model import VRBModel
from networks.traj import TrajAffCVAE
from inference import run_inference_2, run_inference
from PIL import Image
from generate_img import GenerateImg,pos_to_mat44


def generate_2dtraj(args,image:str, strr):
    torch.cuda.manual_seed_all(args.manual_seed)
    torch.manual_seed(args.manual_seed)
    np.random.seed(args.manual_seed)
    random.seed(args.manual_seed)

    hand_head = TrajAffCVAE(in_dim=2 * args.traj_len, hidden_dim=args.hidden_dim,
                        latent_dim=args.hand_latent_dim, condition_dim=args.cond_dim,
                        coord_dim=args.coord_dim, traj_len=args.traj_len)

    # resnet output
    if args.resnet_type == 'resnet50':
        src_in_features = 2048
    else:
        src_in_features = 512

    net = VRBModel(src_in_features=src_in_features,
               num_patches=1,
               hidden_dim=args.hidden_dim,
               hand_head=hand_head,
               encoder_time_embed_type=args.encoder_time_embed_type,
               num_frames_input=10,
               resnet_type=args.resnet_type,
               embed_dim=args.cond_dim, coord_dim=args.coord_dim,
               num_heads=args.num_heads, enc_depth=args.enc_depth,
               attn_kp=args.attn_kp, attn_kp_fc=args.attn_kp_fc, n_maps=5)

    dt = torch.load(args.model_path, map_location='cpu')
    net.load_state_dict(dt)
    net = net.cpu()
    image_pil = Image.open(image).convert("RGB")
    im_out, x_start, y_start, x_len, y_len, max_id = run_inference(net, image_pil,strr)
    im_out.save(f"{image}_out.png")
    
    object_parm = [x_start, y_start, x_len, y_len, max_id]
    

    return object_parm


def pixel_to_world(point_in_pixel,depth_intrin, real_camera = True):
    "use the intrinsic of the realsense, get the points_in_world from pixels"
    x_camera = (point_in_pixel[0]-depth_intrin.ppx)/depth_intrin.fx
    y_camera = (point_in_pixel[1]-depth_intrin.ppy)/depth_intrin.fy
    point_in_camera = [x_camera, 0 , y_camera]
    point_in_camera.append(1)
    #print(point_in_camera)
    fake_camera_pos = [0.191, 0, 0.718, 0, 1.0, -np.pi/2]
    if real_camera == True:
        world_to_camera = np.array([[0.74504764, -0.53747931, 0.39499367, 0.16860667],
                                    [-0.66646109, -0.62391296, 0.40812036, -0.38366486],
                                    [0.02708542, -0.56731703, -0.82305393, 0.70189088],
                                    [ 0, 0, 0, 1]])
    else:
        cam_pos = fake_camera_pos
        world_to_camera= pos_to_mat44(cam_pos[:3], cam_pos[3], cam_pos[4], cam_pos[5])
    mat_transform = np.array([[0, 1, 0, 0], [-1, 0, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
    point_in_world = world_to_camera @ mat_transform @ point_in_camera
    return point_in_world

parser = argparse.ArgumentParser()
parser.add_argument('--num_heads', type=int, default=8, help='num of heads in transformer')
parser.add_argument('--enc_depth', type=int, default=6, help='transformer encoder depth')
parser.add_argument('--hidden_dim', type=int, default=192, help="hidden feature dimension")
parser.add_argument('--hand_latent_dim', type=int, default=4, help="Latent dimension for trajectory CVAE")
parser.add_argument('--cond_dim', type=int, default=256, help="downprojection dimension for transformer encoder")
parser.add_argument('--coord_dim', type=int, default=64, help='Contact coordinate feature dimension')
parser.add_argument('--resnet_type', type=str, default='resnet18')
parser.add_argument('--attn_kp', type=int, default=1)
parser.add_argument('--attn_kp_fc', type=int, default=1)
parser.add_argument('--traj_len', type=int, default=5)
parser.add_argument("--encoder_time_embed_type", default="sin",  choices=["sin", "param"], help="transformer encoder time position embedding")
parser.add_argument("--manual_seed", default=0, type=int, help="manual seed")
parser.add_argument('--model_path', type=str, default='../models/model_checkpoint_1249.pth.tar')

args = parser.parse_args()

def generate_spatial_trajectory(args,rgba_img,points_world,depth_intrin):
    gi = GenerateImg(rgba_img,points_world, depth_intrin)
    gi.camera_matrix_transform()
    gi.generate_img_2()
   
    object1_parm = generate_2dtraj(args,"object1.png", "1")
    object1_dst_parm = generate_2dtraj(args, "object1_img.png","2")
    # object1_parm = [x_start_1, y_start_1, x_len_1, y_len_1, max_id1]
    # object1_dst_parm = [x_start_2, y_start_2, x_len_2, y_len_2, max_id2]
    
    real_start = [object1_parm[0] , object1_parm[1] ]
    fake_start = [object1_dst_parm[0], object1_dst_parm[1]]
    real_end = [object1_parm[0] +object1_parm[3], object1_parm[1]-object1_parm[2]]
    fake_end = [object1_dst_parm[0]+object1_dst_parm[3], object1_dst_parm[1]-object1_dst_parm[2]]
    real_start_world = pixel_to_world(real_start,depth_intrin,real_camera=True)[:3]
    fake_start_world = pixel_to_world(fake_start,depth_intrin,real_camera=False)[:3]
    real_end_world = pixel_to_world(real_end ,depth_intrin,real_camera=True)[:3]
    fake_end_world = pixel_to_world(fake_end,depth_intrin,real_camera=False)[:3]
    real_nvec = np.array(real_end_world - real_start_world)
    #the force angle is nearly the same
    #fake_nvec = np.array(fake_end_world - fake_start_world)

    #two angles are quite opposite
    fake_nvec = np.array(fake_start_world - fake_end_world)
    spatial_traj = np.cross(real_nvec,fake_nvec)
    print("spatial_traj:", spatial_traj)
    #spatial_start = gi.points_world[(800-object1_parm[1]-1)*800 + object1_parm[0]+1]
    spatial_start = gi.points_world[object1_parm[4]]
    print("spatial_start:", spatial_start)
    return spatial_traj, spatial_start
    
if __name__ =="__main__":
    generate_spatial_trajectory(args)







