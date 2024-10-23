from collections import OrderedDict

import numpy as np
import sapien.core as sapien
from sapien.core import Pose

from mani_skill2.agents.base_agent import BaseAgent
from mani_skill2.utils.common import compute_angle_between
from mani_skill2.utils.geometry import transform_points
from mani_skill2.utils.sapien_utils import (
    check_joint_stuck,
    get_actor_by_name,
    get_entity_by_name,
    get_pairwise_contact_impulse,
)


class FixedXmate3Allegro(BaseAgent):
    def __init__(self, *args, **kwargs):
        super(FixedXmate3Allegro, self).__init__(*args, **kwargs)
        self.finger1_link: sapien.LinkBase = get_actor_by_name(
            self._robot.get_links(), "link_3.0_tip"
        )
        self.finger2_link: sapien.LinkBase = get_actor_by_name(
            self._robot.get_links(), "link_7.0_tip"
        )
        self.finger3_link: sapien.LinkBase = get_actor_by_name(
            self._robot.get_links(), "link_11.0_tip"
        )
        self.finger4_link: sapien.LinkBase = get_actor_by_name(
            self._robot.get_links(), "link_15.0_tip"
        )
        self.finger_links = [self.finger1_link,self.finger2_link,self.finger3_link,self.finger4_link]
        self.finger_size = (0.012)  # values from URDF, sphere collision with radius=0.012
        self.grasp_site: sapien.Link = get_entity_by_name(
            self._robot.get_links(), "palm_link"
        )
        self.hand_wrist: sapien.Link = get_entity_by_name(
            self._robot.get_links(), "hand_to_arm.SLDPRT"
        )

    def get_proprioception(self):
        state_dict = OrderedDict()
        qpos = self._robot.get_qpos()
        qvel = self._robot.get_qvel()

        state_dict["qpos"] = qpos
        state_dict["qvel"] = qvel

        return state_dict

    def get_tcp_wrench(self):
        joint_tau = self.get_generalized_external_forces()[:7]
        controller = self._combined_controllers[self._control_mode]._controllers[0]
        assert controller.control_joint_names == [
            "joint1",
            "joint2",
            "joint3",
            "joint4",
            "joint5",
            "joint6",
            "joint7",
        ]
        controller._sync_articulation()
        J_full = controller.controller_articulation.compute_world_cartesian_jacobian()[
            -6:
        ]
        J = np.linalg.pinv(J_full.T)

        TCP_wrench = J @ joint_tau
        return TCP_wrench

    @staticmethod
    def build_grasp_pose(forward, flat, center):
        extra = np.cross(flat, forward)
        ans = np.eye(4)
        ans[:3, :3] = np.array([forward, flat, -extra]).T
        ans[:3, 3] = center
        return Pose.from_transformation_matrix(ans)
