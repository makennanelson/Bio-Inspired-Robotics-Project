"""Open Gymnasium's built-in human-shaped MuJoCo model in the viewer."""
import time

import gymnasium as gym
import mujoco.viewer


def main():
    # Humanoid-v5 supplies the model and its initial standing configuration.
    env = gym.make("Humanoid-v5")
    try:
        env.reset(seed=7)
        model = env.unwrapped.model
        data = env.unwrapped.data

        print("Humanoid model loaded. Close the MuJoCo window to exit.")
        with mujoco.viewer.launch_passive(model, data) as sim_viewer:
            sim_viewer.cam.lookat[:] = [0.0, 0.0, 0.85]
            sim_viewer.cam.distance = 3.2
            while sim_viewer.is_running():
                sim_viewer.sync()
                time.sleep(0.02)
    finally:
        env.close()


if __name__ == "__main__":
    main()
