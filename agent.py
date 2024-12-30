import gymnasium as gym
import rsoccer_gym
from gymnasium.envs.registration import register


register(
    id="Software-Project",
    entry_point="env:ExampleEnv"
)

env = gym.make("Software-Project", render_mode="human")

env.reset()
# Run for 1 episode and print reward at the end
for i in range(1):
    terminated = False
    truncated = False
    while not (terminated or truncated):
        # Step using random actions
        action = env.action_space.sample()
        next_state, reward, terminated, truncated, _ = env.step(action)
    print(reward)