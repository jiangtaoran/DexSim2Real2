import numpy as np
from scipy.spatial.transform import Rotation as R
from pyquaternion import Quaternion
import sapien.core as sapien

from camera import RgbAndMesh
from generate_spatial_traj import generate_spatial_trajectory, args

def create_capsule(
        scene: sapien.Scene,
        pose: sapien.Pose,
        radius,
        half_length,
        color=None,
        name='',
) -> sapien.Actor:
    """Create a capsule (x-axis <-> half_length). See create_box."""
    builder = scene.create_actor_builder()
    #builder.add_capsule_collision(radius=radius, half_length=half_length)
    builder.add_capsule_visual(radius=radius, half_length=half_length, color=color)
    capsule = builder.build(name=name)
    capsule.set_pose(pose)
    return capsule

def transform_q(traj):
    traj = traj / np.linalg.norm(traj)
    rot = R.from_rotvec(np.pi / 2 * traj)
    matrix = np.eye(4)
    matrix = rot.as_matrix()
    q = Quaternion(matrix = matrix)
    q = q.elements.tolist()
    return q


rm = RgbAndMesh(use_gui = True)

spatial_traj = [ 0.00102287, 0.00053677, -0.00586645]
spatial_start = [ 0.9760355,  -0.20923296,  0.22143626]


q = np.array(transform_q(spatial_traj))
print(q)
capsule = create_capsule(
        rm.scene,
        sapien.Pose(p=spatial_start, q=q),
        radius=0.05,
        half_length=0.5,
        color=[0., 0., 1.],
        name='trajectory',
    )
rm.visualize_3dtraj()