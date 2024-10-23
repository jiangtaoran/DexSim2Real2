import sapien.core as sapien
from sapien.utils.viewer import Viewer
import numpy as np
from PIL import Image
from pathlib import Path

class RgbAndMesh(object):
    '''
    get rgb and its 3d mesh in virtual environment,and generate image
    use in simulation environment
    '''
    def __init__(self,use_gui = False):
        self.engine = sapien.Engine()
        self.renderer = sapien.SapienRenderer()
        self.engine.set_renderer(self.renderer)

        self.balance_passive_force = True
        self.fix_root_link = True
        self.scene_config = sapien.SceneConfig()
        self.scene = self.engine.create_scene(self.scene_config)
        self.scene = self.engine.create_scene()
        self.scene.set_timestep(1/240.0)
        self.scene.add_ground(0)
        self.width = 640
        self.height = 360

        self.current_dir = Path(__file__).resolve().parent

        self.use_gui = use_gui
        if self.use_gui == True:
            self.viewer = Viewer(self.renderer)
            self.viewer.set_scene(self.scene)
            self.viewer.set_camera_xyz(x=-0.75, y=0.5, z=1)
            self.viewer.set_camera_rpy(r=0, p=-0.3, y=0)

        self.add_default_scene_light()
    def add_default_scene_light(self):
        # If the light is already set, then we just skip the function.
        if len(self.scene.get_all_lights()) >= 3:
            return
        self.scene.add_directional_light(np.array([-1, -1, -1]), np.array([0.5, 0.5, 0.5]), shadow=True)
        self.scene.add_directional_light([0, 0, -1], [0.9, 0.8, 0.8], shadow=True)

        intensity = 20
        self.scene.add_spot_light(np.array([0, 0, 3]), direction=np.array([0, 0, -1]), inner_fov=0.3, outer_fov=1.0,
                             color=np.array([0.5, 0.5, 0.5]) * intensity, shadow=True)
        self.scene.add_spot_light(np.array([1, 0, 2]), direction=np.array([-1, 0, -1]), inner_fov=0.3, outer_fov=1.0,
                             color=np.array([0.5, 0.5, 0.5]) * intensity, shadow=True)
        visual_material = self.renderer.create_material()
        visual_material.set_base_color(np.array([0.5, 0.5, 0.5, 1]))
        visual_material.set_roughness(0.7)
        visual_material.set_metallic(1)
        visual_material.set_specular(0.04)

    def load_object(self,index):
        loader: sapien.URDFLoader = self.scene.create_urdf_loader()
        loader.fix_root_link = self.fix_root_link
        loader.load_multiple_collisions_from_file = True

        urdf_pth = str(self.current_dir.parent/"assets"/"sapien"/str(index)/"mobility.urdf")
        object: sapien.Articulation = loader.load(urdf_pth)
        object.set_root_pose(sapien.Pose([1.5,0,0.4],[1,0,0,0]))
        #num_qpos = np.size(object.get_qpos())
        #self.init_qpos = np.zeros(num_qpos)
        self.init_qpos = [-np.pi/3]
        object.set_qpos(self.init_qpos)
    
    def set_camera_intrinx(self, fov_x = 70.0529, fov_y = 42.921):
        near,far = 0.1,100
        
        self.camera_mount_actor = self.scene.create_actor_builder().build_kinematic()
        self.camera = self.scene.add_mounted_camera(
            name = "camera1",
            width = self.width,
            height = self.height,
            fovy = np.deg2rad(fov_y),
            fovx = np.deg2rad(fov_x),
            near = near,
            far = far,
            actor = self.camera_mount_actor,
            pose = sapien.Pose(),
        )

    
    def set_camera_extrinx(self,r_x = 0, b_y = 1.3, a_z = -1.2, pos=[0,-0.5,4]):
 
        #self, r_x = -0.0374, b_y = 0.9776, a_z = -1.2197, pos = [0.2192, -0.43277, 0.7225]
        left = np.array([-np.cos(a_z)*np.cos(r_x)-np.sin(a_z)*np.sin(b_y)*np.sin(r_x), \
                         -np.sin(a_z)*np.cos(r_x)+np.cos(a_z)*np.sin(b_y)*np.sin(r_x), \
                          np.sin(r_x)*np.cos(b_y)])
        forward = np.array([-np.sin(a_z)*np.cos(b_y), \
                            +np.cos(a_z)*np.cos(b_y), \
                            -np.sin(b_y)])
        up = np.cross(forward, left)

        mat44 = np.eye(4)
        mat44[:3, :3] = np.vstack([forward, left, up]).T
        mat44[:3, 3] = pos # mat44 is cam2world
        self.camera_mount_actor.set_pose(sapien.Pose.from_transformation_matrix(mat44))
        #print(mat44)
        #print(self.camera.get_model_matrix())
    def save_rgb(self):
        self.scene.step()
        self.scene.update_render()
        self.camera.take_picture()
        self.rgba = self.camera.get_color_rgba()
        rgba_img = (self.rgba*255).clip(0, 255).astype("uint8")
        rgba_pil = Image.fromarray(rgba_img)
        rgba_pil.save("object1.png")

        return rgba_img
    
    def save_3dmesh(self):
        position = self.camera.get_float_texture('Position')
        points_opengl = position[..., :3][position[..., 3] < 1]

        model_matrix = self.camera.get_model_matrix()
        #print(self.camera.get_model_matrix())
        points_world = points_opengl @ model_matrix[:3, :3].T + model_matrix[:3, 3]
        return position, points_world #position[H,W,4]，points_world[H*W,3]

    def open_viewer(self):
        if self.use_gui == True:

            while not self.viewer.closed:
                self.scene.step()
                self.scene.update_render()
                self.viewer.render()

    def demo(self,index):
        self.load_object(index)
        self.set_camera_intrinx()
        self.set_camera_extrinx()
        self.open_viewer()
        rgba_img = self.save_rgb()
        position, points_world = self.save_3dmesh()
        print("taking the picture of the object is done")
        return rgba_img, position, points_world

    def visualize_3dtraj(self):
        self.load_object(10098)
        self.open_viewer()

if __name__ == "__main__":
    rm = RgbAndMesh(use_gui = True)
    rgba_img, position, points_world=rm.demo(10098)



