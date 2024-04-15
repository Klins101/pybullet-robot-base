import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim
from robot import *
from goal import *
import matplotlib.pyplot as plt

class DRLAgent:
    def __init__(self, state_dim, action_dim):
        self.state_dim = state_dim
        self.action_dim = action_dim
        # Initialize nn
        self.initialize_network()

        self.total_rewards = []
        self.steps_to_completion = []

    def initialize_network(self):
        self.model = nn.Sequential(
            nn.Linear(self.state_dim, 64),
            nn.ReLU(),
            nn.Linear(64, self.action_dim),
            nn.Softmax(dim=-1)
        )
        self.optimizer = optim.Adam(self.model.parameters(), lr=0.001)

    def get_action(self, state):
        # epsilon-greedy policy
        epsilon = 0.1  
        if np.random.rand() < epsilon:
            return np.random.randint(0, self.action_dim)
        else:
            state_tensor = torch.FloatTensor(state)
            action_probs = self.model(state_tensor)
            return torch.argmax(action_probs).item()

    def update_policy(self, trajectory):
        states, actions, rewards, _, _ = zip(*trajectory)
        states = torch.FloatTensor(states)
        actions = torch.LongTensor(actions)
        rewards = torch.FloatTensor(rewards)

        action_probs = self.model(states)
        selected_action_probs = action_probs.gather(1, actions.unsqueeze(1)).squeeze()

        loss = -torch.mean(torch.log(selected_action_probs) * rewards)

        self.optimizer.zero_grad()
        loss.backward()
        self.optimizer.step()

    def plot_training_progress(self):
        plt.figure(figsize=(12, 6))
        plt.subplot(1, 2, 1)
        plt.plot(self.total_rewards)
        plt.title('Total Reward per Episode')
        plt.xlabel('Episode')
        plt.ylabel('Total Reward')

        plt.subplot(1, 2, 2)
        plt.plot(self.steps_to_completion)
        plt.title('Steps to Completion per Episode')
        plt.xlabel('Episode')
        plt.ylabel('Steps to Completion')
        plt.show()

robot = Robot(with_gui=True)
goal = Goal()
home_conf = robot.home_conf
# Initializing DRL agent
agent = DRLAgent(state_dim=7, action_dim=7)

num_episodes = 1000
steps_per_episode = 100

# Training loop
for episode in range(num_episodes):
    robot.reset_joint_pos(home_conf)
    # Get state
    state = robot.get_joint_pos()
    # Initialize trajectory
    trajectory = []
    total_reward = 0
    steps = 0

    for step in range(steps_per_episode):
        # Get action
        action = agent.get_action(state)
        # Apply action
        new_state = robot.reset_joint_pos(action)
        # Check collision
        collision = robot.in_collision()
        # Compute reward
        distance = goal.distance(new_state)
        reward = -distance if not collision else -1e3

        # Check if the goal has been reached
        done = goal.reached(new_state)

        # Store the transition in the trajectory
        trajectory.append((state, action, reward, new_state, done))

        # Update the state
        state = new_state

        total_reward += reward
        steps += 1

        # If the goal has been reached or a collision has occurred, end the episode
        if done or collision:
            break

    # Update the agent's policy
    agent.update_policy(trajectory)

    # Append metrics for plotting
    agent.total_rewards.append(total_reward)
    agent.steps_to_completion.append(steps)

    if episode % 10 == 0:
        print(f"Episode {episode}, Total Reward: {total_reward}, Steps to Completion: {steps}")

# Plot training progress
agent.plot_training_progress()
