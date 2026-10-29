"""Train a phase-conditioned, human-style Humanoid-v5 gait policy.

Unlike simple speed tracking, this policy receives a gait-cycle clock and is
rewarded for alternating hip motion, knee flexion during swing, counter-swinging
arms, and a steady torso. The reference cycle is intentionally simple and is
not a substitute for measured motion-capture data.
"""

import argparse
from pathlib import Path

import gymnasium as gym
import mujoco
import numpy as np
from stable_baselines3 import PPO
from stable_baselines3.common.monitor import Monitor


class HumanGaitReward(gym.Wrapper):
    """Add a walking/running reference cycle to Humanoid-v5."""

    TORSO_JOINTS = ("abdomen_z", "abdomen_y", "abdomen_x")
    ARM_JOINTS = (
        "right_shoulder1", "right_shoulder2", "right_elbow",
        "left_shoulder1", "left_shoulder2", "left_elbow",
    )

    def __init__(self, env, gait):
        super().__init__(env)
        self.gait = gait
        self.settings = {
            "walk": {
                "target_speed": 0.8,
                "cycle_frequency": 1.0,
                "stance_fraction": 0.62,
                "hip_swing": 0.30,
                "knee_swing": 0.55,
                "arm_swing": 0.18,
            },
            "run": {
                "target_speed": 2.0,
                "cycle_frequency": 1.6,
                "stance_fraction": 0.40,
                "hip_swing": 0.42,
                "knee_swing": 0.80,
                "arm_swing": 0.28,
            },
        }[gait]

        self.model = self.env.unwrapped.model
        self.data = self.env.unwrapped.data
        self.dt = float(self.model.opt.timestep * self.env.unwrapped.frame_skip)
        self.phase = 0.0

        self.hip_qpos, self.hip_dofs = self._addresses(
            ("right_hip_y", "left_hip_y")
        )
        self.knee_qpos, self.knee_dofs = self._addresses(
            ("right_knee", "left_knee")
        )
        self.torso_qpos, self.torso_dofs = self._addresses(self.TORSO_JOINTS)
        self.arm_qpos, self.arm_dofs = self._addresses(self.ARM_JOINTS)

        root_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_JOINT, "root"
        )
        if root_id < 0:
            raise ValueError("Could not find the Humanoid free joint named 'root'.")
        root_dof = int(self.model.jnt_dofadr[root_id])
        self.root_rotation_dofs = np.arange(root_dof + 3, root_dof + 6)

        # Expose gait phase to the policy so it can learn a repeatable cycle.
        base_space = self.env.observation_space
        self.observation_space = gym.spaces.Box(
            low=np.concatenate((base_space.low, np.array([-1.0, -1.0]))),
            high=np.concatenate((base_space.high, np.array([1.0, 1.0]))),
            dtype=base_space.dtype,
        )

        self.reference_qpos = self.data.qpos.copy()
        self.previous_action = np.zeros(self.action_space.shape, dtype=np.float64)

    def _addresses(self, names):
        qpos_addresses = []
        dof_addresses = []
        for name in names:
            joint_id = mujoco.mj_name2id(
                self.model, mujoco.mjtObj.mjOBJ_JOINT, name
            )
            if joint_id < 0:
                raise ValueError(f"Could not find Humanoid joint {name!r}.")
            qpos_addresses.append(int(self.model.jnt_qposadr[joint_id]))
            dof_addresses.append(int(self.model.jnt_dofadr[joint_id]))
        return np.asarray(qpos_addresses), np.asarray(dof_addresses)

    def _add_phase(self, observation):
        phase_features = np.array(
            [np.sin(2.0 * np.pi * self.phase), np.cos(2.0 * np.pi * self.phase)],
            dtype=self.observation_space.dtype,
        )
        return np.concatenate((observation, phase_features)).astype(
            self.observation_space.dtype, copy=False
        )

    def reset(self, **kwargs):
        observation, info = self.env.reset(**kwargs)
        self.phase = 0.0
        self.reference_qpos = self.data.qpos.copy()
        self.previous_action = np.zeros(self.action_space.shape, dtype=np.float64)
        return self._add_phase(observation), info

    def _leg_reference(self, phase):
        """Return hip and knee offsets for one leg over its normalized cycle."""
        phase = phase % 1.0
        stance = self.settings["stance_fraction"]

        # Hip moves forward in swing and backward through stance.
        hip_offset = -self.settings["hip_swing"] * np.cos(2.0 * np.pi * phase)

        # Keep the knee nearly extended in stance, then flex it during swing.
        if phase < stance:
            knee_bend = 0.05
        else:
            swing_phase = (phase - stance) / (1.0 - stance)
            knee_bend = np.sin(np.pi * swing_phase)
        knee_offset = -0.06 - self.settings["knee_swing"] * knee_bend
        return hip_offset, knee_offset

    @staticmethod
    def _angle_error(current, target):
        difference = current - target
        return np.arctan2(np.sin(difference), np.cos(difference))

    def step(self, action):
        observation, _, terminated, truncated, info = self.env.step(action)
        self.phase = (
            self.phase + self.settings["cycle_frequency"] * self.dt
        ) % 1.0

        right_hip, right_knee = self._leg_reference(self.phase)
        left_hip, left_knee = self._leg_reference(self.phase + 0.5)

        target_hips = self.reference_qpos[self.hip_qpos] + np.array(
            [right_hip, left_hip]
        )
        target_knees = self.reference_qpos[self.knee_qpos] + np.array(
            [right_knee, left_knee]
        )

        # Swing the arms opposite the same-side leg; keep other arm axes near
        # their starting pose to reduce flailing.
        arm_offsets = np.zeros(6)
        arm_offsets[0] = self.settings["arm_swing"] * np.sin(
            2.0 * np.pi * (self.phase + 0.5)
        )
        arm_offsets[3] = self.settings["arm_swing"] * np.sin(
            2.0 * np.pi * self.phase
        )
        target_arms = self.reference_qpos[self.arm_qpos] + arm_offsets
        target_torso = self.reference_qpos[self.torso_qpos]

        hip_error = self._angle_error(self.data.qpos[self.hip_qpos], target_hips)
        knee_error = self._angle_error(self.data.qpos[self.knee_qpos], target_knees)
        arm_error = self._angle_error(self.data.qpos[self.arm_qpos], target_arms)
        torso_error = self._angle_error(self.data.qpos[self.torso_qpos], target_torso)

        # Penalize torso rocking and arm flailing while rewarding the reference
        # leg cycle and opposite arm swing.
        gait_cost = (
            5.0 * np.sum(hip_error**2)
            + 4.0 * np.sum(knee_error**2)
            + 2.0 * np.sum(arm_error**2)
            + 0.25 * np.sum(torso_error**2)
            + 0.025 * np.sum(self.data.qvel[self.torso_dofs] ** 2)
            + 0.020 * np.sum(self.data.qvel[self.root_rotation_dofs] ** 2)
            + 0.001 * np.sum(self.data.qvel[self.arm_dofs] ** 2)
        )
        smoothness_cost = 0.01 * np.sum((np.asarray(action) - self.previous_action) ** 2)

        speed_error = float(info["x_velocity"]) - self.settings["target_speed"]
        reward = (
            float(info.get("reward_survive", 0.0))
            - 2.0 * speed_error**2
            + float(info.get("reward_ctrl", 0.0))
            + float(info.get("reward_contact", 0.0))
            - gait_cost
            - smoothness_cost
        )

        self.previous_action = np.asarray(action).copy()
        info["gait_phase"] = self.phase
        info["gait_tracking_cost"] = -float(gait_cost)
        return self._add_phase(observation), float(reward), terminated, truncated, info


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--gait", choices=("walk", "run"), required=True)
    parser.add_argument("--steps", type=int, default=2_000_000)
    args = parser.parse_args()

    env = Monitor(HumanGaitReward(gym.make("Humanoid-v5"), args.gait))
    model = PPO(
        "MlpPolicy",
        env,
        verbose=1,
        device="cpu",
        seed=7,
        n_steps=2048,
        batch_size=64,
    )

    print(
        f"Training a new {args.gait} gait policy for {args.steps:,} steps. "
        "This is a fresh policy because the gait-phase observation is new."
    )
    model.learn(total_timesteps=args.steps)

    output_dir = Path("models")
    output_dir.mkdir(exist_ok=True)
    model_path = output_dir / f"humanoid_{args.gait}"
    model.save(str(model_path))
    env.close()
    print(f"Saved gait policy to {model_path}.zip")


if __name__ == "__main__":
    main()
