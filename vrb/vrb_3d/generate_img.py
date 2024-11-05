from camera import RgbAndMesh
import numpy as np
import sapien.core as sapien
from sapien.utils.viewer import Viewer
from PIL import Image

def pos_to_mat44(pos,r_x,b_y,a_z):
    '''
    turn the xyzrpy to mat44
    '''
    left = np.array([-np.cos(a_z) * np.cos(r_x) - np.sin(a_z) * np.sin(b_y) * np.sin(r_x), \
                     -np.sin(a_z) * np.cos(r_x) + np.cos(a_z) * np.sin(b_y) * np.sin(r_x), \
                     np.sin(r_x) * np.cos(b_y)])
    forward = np.array([-np.sin(a_z) * np.cos(b_y), \
                        +np.cos(a_z) * np.cos(b_y), \
                        -np.sin(b_y)])
    up = np.cross(forward, left)

    mat44 = np.eye(4)
    mat44[:3, :3] = np.vstack([forward, left, up]).T
    mat44[:3, 3] = pos  # mat44 is cam2world
    return mat44

class GenerateImg(RgbAndMesh):
    def __init__(self,rgba_img, points_world, depth_intrin):
        self.rm = RgbAndMesh(use_gui=False)
        self.rgba_img = rgba_img[:,:,:4]  
        self.points_world = points_world
        self.fx = depth_intrin.fx
        self.fy = depth_intrin.fy
        self.ppx = depth_intrin.ppx
        self.ppy = depth_intrin.ppy
       

    def camera_matrix_transform(self):
        '''
        transform mesh point from camera1 matrix to fake camera matrix
        mesh_to_fake = mesh_to_world*fake_to_world.reverse
        '''
        points_in_world = np.append(self.points_world,np.ones([self.rm.height*self.rm.width,1]), axis = 1)
        # world_to_camera = self.rm.camera.get_model_matrix()
        
        fake_camera_pos = [0.191, 0, 0.718, 0, 1.0, -np.pi/2]
        world_to_fake = pos_to_mat44(fake_camera_pos[:3],fake_camera_pos[3],fake_camera_pos[4],fake_camera_pos[5])
        mat_transform = [[0,1,0,0],[-1,0,0,0],[0,0,1,0],[0,0,0,1]]
        self.points_in_fake = np.linalg.inv(world_to_fake@mat_transform)@points_in_world.T
        self.points_in_fake = self.points_in_fake.T

    

    def generate_img_2(self):
        '''
        generate image with intrinx
        '''
        # fx = fy = 908.384
        # ppx = 643.318
        # ppy = 362.901
        # 1280*720
        # ppx = ppy = 402

        fake_points = np.array(self.points_in_fake)

        x_points = fake_points.take([0],axis=1)*self.fx/fake_points.take([1],axis=1)+self.ppx
        y_points = fake_points.take([2],axis=1)*self.fy/fake_points.take([1],axis=1)+self.ppy
        z_points = fake_points.take([1],axis=1)
    
        x_pixel = np.clip(np.round(x_points),0,640)
        y_pixel = np.clip(np.round(y_points),0,360)

        img_rgb = np.zeros([self.rm.height, self.rm.width, 5])

        for i in range(fake_points.shape[0]):
            h_rgb = i // self.rm.width + 1
            w_rgb = i - (i // self.rm.width) * self.rm.width
            rgb_i = self.rgba_img[h_rgb - 1][w_rgb - 1]
            
            if img_rgb[self.rm.height - int(y_pixel[i][0]) - 1][int(x_pixel[i][0]) - 1][3] == 0:
                img_rgb[self.rm.height - int(y_pixel[i][0]) - 1][int(x_pixel[i][0]) - 1][:4] = rgb_i
                img_rgb[self.rm.height - int(y_pixel[i][0]) - 1][int(x_pixel[i][0]) - 1][4] = z_points[i]
            else:
                if img_rgb[self.rm.height - int(y_pixel[i][0]) - 1][int(x_pixel[i][0]) - 1][4] > z_points[i]:
                    img_rgb[self.rm.height - int(y_pixel[i][0]) - 1][int(x_pixel[i][0]) - 1][:4] = rgb_i
                    img_rgb[self.rm.height - int(y_pixel[i][0]) - 1][int(x_pixel[i][0]) - 1][4] = z_points[i]
        img_rgb_1 = np.zeros([self.rm.height, self.rm.width, 4])

        img_mask = np.zeros([self.rm.height, self.rm.width, 4])

        for j in range(img_rgb.shape[0]):
            for k in range(img_rgb.shape[1]):
                if img_rgb[j][k][3] == 0:
                    img_rgb[j][k][:4] = [255,255,255,255]
                    img_mask[j][k] = [255,255,255,255]     #white
                else:
                    img_mask[j][k] = [0,0,0,255]   #black
                img_rgb_1[j][k] = img_rgb[j][k][:4]

        rgba_pil = Image.fromarray(np.uint8(img_rgb_1[:,:,:3]))
        rgba_pil.save("object1_img.png")
        print("generating the imagine picture is done")

        rgba_pil = Image.fromarray(np.uint8(img_mask[:,:,:3]))
        rgba_pil.save("object1_mask.png")
        print("generating the mask of imagine picture is done")


        return

if __name__ =="__main__":
    mat44 = pos_to_mat44([0.191, 0, 0.718] ,0, 1.0, -np.pi/2)
    print(mat44)
