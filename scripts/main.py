import os

import gymnasium as gym
from stable_baselines3 import SAC, DDPG
from stable_baselines3.common.monitor import Monitor

# for original
# import panda_mujoco_gym_og
# for modified
import panda_mujoco_gym


N_STEPS = 1000000
log_dir = "./logs/"
os.makedirs(log_dir, exist_ok=True)
env = gym.make("FrankaPushDense-v0", render_mode="human")
env = Monitor(env, filename=os.path.join(log_dir, "monitor.csv"))
model = SAC(policy="MultiInputPolicy", env=env, verbose=1, tensorboard_log=log_dir)

try:
    model.learn(total_timesteps=N_STEPS)
except KeyboardInterrupt:
    pass
print("Saving model...")
model.save(f"SAC_panda_reach_{N_STEPS}")