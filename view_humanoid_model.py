"""Open Gymnasium's Humanoid-v5 model in the MuJoCo viewer."""

import time

import gymnasium as gym
import mujoco.viewer


def main():
    # Load the human-shaped model and its default standing pose.
    env = gym.make("Humanoid-v5")

    try:
        env.reset(seed=7)
        model = env.unwrapped.model
        data = env.unwrapped.data

        print("Humanoid model loaded. Close the MuJoCo window to exit.")

        with mujoco.viewer.launch_passive(model, data) as viewer:
            # Adjust the camera to show the full robot.
            viewer.cam.lookat[:] = [0.0, 0.0, 0.9]
            viewer.cam.distance = 3.2
            viewer.cam.azimuth = 90
            viewer.cam.elevation = -10

            while viewer.is_running():
                viewer.sync()
                time.sleep(0.02)

    finally:
        env.close()


if __name__ == "__main__":
    main()