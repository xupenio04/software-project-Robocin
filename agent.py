import gymnasium as gym
import rsoccer_gym
from gymnasium.envs.registration import register


register(
    id="VSS-Project",
    entry_point="vssenv:ExampleEnv"
)

register(
    id="SSL-Project",
    entry_point="sslenv:SSLExampleEnv"
)

env = gym.make("SSL-Project")

env.reset()
# Run for 1 episode and print reward at the end
for i in range(1):
    terminated = False
    truncated = False
    while not (terminated or truncated):
        # Step using random actions
        action = env.action_space.sample()
        next_state, reward, terminated, _, _ = env.step(action)
    print(reward)