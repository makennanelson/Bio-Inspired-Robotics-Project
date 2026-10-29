"""View a gait-phase-conditioned Humanoid-v5 walking/running policy."""

import argparse
import time
from pathlib import Path

import gymnasium as gym
import mujoco
import mujoco.viewer
from stable_baselines3 import PPO

from train_humanoid import HumanGaitReward


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--gait", choices=("walk", "run"), required=True)
    args = parser.parse_args()

    model_path = Path("models") / f"humanoid_{args.gait}.zip"
    if not model_path.exists():
        raise FileNotFoundError(
            f"Could not find {model_path}. Train the {args.gait} policy first."
        )

    policy = PPO.load(str(model_path), device="cpu")

    # This wrapper appends sin(phase) and cos(phase), matching the 350-value
    # observation used during training (348 Humanoid values + 2 phase values).
    env = HumanGaitReward(gym.make("Humanoid-v5"), args.gait)

    try:
        observation, _ = env.reset(seed=7)
        model = env.unwrapped.model
        data = env.unwrapped.data
        torso_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_BODY, "torso"
        )

        print(f"Running the {args.gait} policy. Close the MuJoCo window to stop.")

        with mujoco.viewer.launch_passive(model, data) as viewer:
            viewer.cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
            viewer.cam.trackbodyid = torso_id
            viewer.cam.distance = 3.2
            viewer.cam.azimuth = 90
            viewer.cam.elevation = -10

            while viewer.is_running():
                action, _ = policy.predict(observation, deterministic=True)
                observation, _, terminated, truncated, _ = env.step(action)

                if terminated or truncated:
                    observation, _ = env.reset()

                viewer.sync()
                time.sleep(env.dt)

    finally:
        env.close()


if __name__ == "__main__":
    main()
