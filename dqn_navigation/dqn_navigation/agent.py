"""Small replay-based DQN agent. PyTorch is installed separately from ROS."""
from collections import deque
import os
import random

import numpy as np
import torch
from torch import nn


class QNetwork(nn.Module):
    def __init__(self, state_size: int, action_count: int):
        super().__init__()
        self.layers = nn.Sequential(
            nn.Linear(state_size, 128), nn.ReLU(),
            nn.Linear(128, 128), nn.ReLU(),
            nn.Linear(128, action_count),
        )

    def forward(self, x):
        return self.layers(x)


class DQNAgent:
    def __init__(self, state_size: int, action_count: int, checkpoint: str,
                 learning: bool = True, device: str = 'cpu'):
        self.state_size = state_size
        self.action_count = action_count
        self.checkpoint = checkpoint
        self.learning = learning
        self.device = torch.device(device)
        self.online = QNetwork(state_size, action_count).to(self.device)
        self.target = QNetwork(state_size, action_count).to(self.device)
        self.target.load_state_dict(self.online.state_dict())
        self.optimizer = torch.optim.Adam(self.online.parameters(), lr=1e-3)
        self.memory = deque(maxlen=20_000)
        self.gamma = 0.98
        self.batch_size = 64
        self.epsilon = 1.0 if learning else 0.0
        self.epsilon_min = 0.05
        self.epsilon_decay = 0.9995
        self.steps = 0
        try:
            saved = torch.load(checkpoint, map_location=self.device, weights_only=True)
            self.online.load_state_dict(saved['online'])
            self.target.load_state_dict(saved.get('target', saved['online']))
            self.epsilon = saved.get('epsilon', self.epsilon) if learning else 0.0
            self.steps = saved.get('steps', 0)
        except (FileNotFoundError, KeyError, RuntimeError, OSError, EOFError):
            pass

    def act(self, state: np.ndarray) -> int:
        if self.learning and random.random() < self.epsilon:
            return random.randrange(self.action_count)
        with torch.no_grad():
            x = torch.as_tensor(state, dtype=torch.float32, device=self.device).unsqueeze(0)
            return int(self.online(x).argmax(dim=1).item())

    def remember(self, state, action, reward, next_state, done):
        if self.learning:
            self.memory.append((state.copy(), action, reward, next_state.copy(), done))

    def train_step(self):
        if not self.learning or len(self.memory) < self.batch_size:
            return
        batch = random.sample(self.memory, self.batch_size)
        states, actions, rewards, next_states, dones = zip(*batch)
        states = torch.as_tensor(np.asarray(states), dtype=torch.float32, device=self.device)
        actions = torch.as_tensor(actions, dtype=torch.int64, device=self.device).unsqueeze(1)
        rewards = torch.as_tensor(rewards, dtype=torch.float32, device=self.device)
        next_states = torch.as_tensor(np.asarray(next_states), dtype=torch.float32, device=self.device)
        dones = torch.as_tensor(dones, dtype=torch.float32, device=self.device)
        current = self.online(states).gather(1, actions).squeeze(1)
        with torch.no_grad():
            future = self.target(next_states).max(dim=1).values
            expected = rewards + self.gamma * future * (1.0 - dones)
        loss = nn.functional.smooth_l1_loss(current, expected)
        self.optimizer.zero_grad()
        loss.backward()
        nn.utils.clip_grad_norm_(self.online.parameters(), 5.0)
        self.optimizer.step()
        self.steps += 1
        self.epsilon = max(self.epsilon_min, self.epsilon * self.epsilon_decay)
        if self.steps % 250 == 0:
            self.target.load_state_dict(self.online.state_dict())
        if self.steps % 100 == 0:
            self.save()

    def save(self):
        temporary_checkpoint = f'{self.checkpoint}.tmp'
        torch.save({'online': self.online.state_dict(), 'target': self.target.state_dict(),
                    'epsilon': self.epsilon, 'steps': self.steps}, temporary_checkpoint)
        os.replace(temporary_checkpoint, self.checkpoint)
