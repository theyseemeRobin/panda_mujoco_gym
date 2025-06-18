import os
import numpy as np
from copy import copy

from panda_mujoco_gym.envs.panda_env import FrankaEnv

MODEL_XML_PATH = os.path.join(os.path.dirname(__file__), "../assets/", "reach.xml")


class FrankaReachEnv(FrankaEnv):
    def __init__(
        self,
        reward_type,
        **kwargs,
    ):
        super().__init__(
            model_path=MODEL_XML_PATH,
            n_substeps=25,
            reward_type=reward_type,
            block_gripper=True,
            distance_threshold=0.05,
            goal_xy_range=0.3,
            obj_xy_range=0.3,
            goal_x_offset=0.0,
            goal_z_range=0.0,
            **kwargs,
        )

    def _get_obs(self) -> dict:
        # robot
        ee_position = self._utils.get_site_xpos(self.model, self.data, "ee_center_site").copy()

        ee_velocity = self._utils.get_site_xvelp(self.model, self.data, "ee_center_site").copy() * self.dt

        if not self.block_gripper:
            fingers_width = self.get_fingers_width().copy()

        if not self.block_gripper:
            ee = np.concatenate([ee_position, ee_velocity, fingers_width])
        else:
            ee = np.concatenate([ee_position, ee_velocity])

        obs = {
            "observation": ee.copy(),
            "achieved_goal": ee_position.copy(),
            "desired_goal": self.goal.copy(),
        }

        return obs

    def _sample_goal(self) -> np.ndarray:
        joint_angles = np.array([0, -0.78, 0, -2.35, 0, 1.57, 0.78, 0.001, 0.001])
        return forward_kinematics(self, joint_angles)

    def _render_callback(self) -> None:
        sites_offset = (self.data.site_xpos - self.model.site_pos).copy()
        site_id = self._model_names.site_name2id["target"]
        self.model.site_pos[site_id] = self.goal - sites_offset[site_id]
        self._mujoco.mj_forward(self.model, self.data)


def print_joints(env, model, data):
    print("Joints: ", end='')
    for name in env.arm_joint_names:
        print(f"{env._utils.get_joint_qpos(model, data, name)[0]:.3f}", end=' ')
    print("")

def print_ee_pos(env, model, data):
    ee_pos = env._utils.get_site_xpos(model, data, "ee_center_site")
    print(f"EE: {ee_pos}")

def forward_kinematics(env, joint_values):
    data = copy(env.data)
    model = copy(env.model)
    for name, value in zip(env.arm_joint_names, joint_values):
        env._utils.set_joint_qpos(model, data, name, value)
    env._mujoco.mj_kinematics(model, data)
    ee_position = env._utils.get_site_xpos(model, data, "ee_center_site")
    return ee_position