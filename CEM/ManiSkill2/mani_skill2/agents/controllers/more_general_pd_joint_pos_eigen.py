import numpy as np
import sapien.core as sapien

from mani_skill2.agents.controllers.base_controller import BaseController
from mani_skill2.agents.controllers.more_general_pd_joint_pos import MoreGeneralPDJointPosController
from mani_skill2.utils.eigengrasp import EigenGrasp


class MoreGeneralPDJointPosController_Eigen(MoreGeneralPDJointPosController):
    """
    More General PD joint position controller, allowing different limits, stiffness and damping
    Mainly used for allegro hand
    Eigengrasp is used to reduce the action space
    """
    def __init__(
        self, controller_config: dict, robot: sapien.Articulation, control_freq: int
    ):
        super(MoreGeneralPDJointPosController_Eigen, self).__init__(
            controller_config, robot, control_freq
        )
        self.action_dimension = controller_config['eigen_dim']
        self.grasp_model = EigenGrasp(
            self.num_control_joints, self.action_dimension
        ).load_from_file(controller_config['grasp_model_path'])

    """
    A N*2 matrix shows the border of the action space qpos. 
    Every row [min, max] stands for a joint, min is the lower bound and max is the upper bound.
    """
    @property
    def action_range(self) -> np.ndarray:
        ranges = np.stack(
            [self.joint_delta_pos_min+self.grasp_model._pca.mean_, self.joint_delta_pos_max+self.grasp_model._pca.mean_], axis=0
        ) if self.use_delta else np.stack(
            [self.joint_pos_min, self.joint_pos_max], axis=0
        )
        
        ranges = self.grasp_model.reduce_original_dim(ranges).transpose()
        #print("ranges",ranges)
        #ranges = np.stack([[-100]*self.grasp_model._M,[100]*self.grasp_model._M],axis=0)
        
        return ranges

    """
    Set the action of the agent
    """
    def set_action(self, action: np.ndarray):
        
        assert action.shape[0] == self.action_dimension
        action = self.grasp_model.compute_grasp(action)
        mean = self.grasp_model._pca.mean_
       # print("mean",mean)
        #print("hand_action",action)
        self.curr_step = 0
        self.start_joint_pos = self._get_curr_joint_pos()
        if self.use_delta:
        #使用了delta    
            #self.final_target_joint_pos = action + self.start_joint_pos
            self.final_target_joint_pos = action-mean+self.start_joint_pos
            #print("过程中手部姿势",self.final_target_joint_pos)
        else:
            self.final_target_joint_pos = action
            
