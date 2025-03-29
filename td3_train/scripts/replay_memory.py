import random
import numpy as np
import os
import pickle
from collections import deque

class ReplayMemory:
    def __init__(self, capacity, seed):
        random.seed(seed)
        self.capacity = capacity
        self.buffer = deque(maxlen=capacity)
        self.position = 0
    
    def push(self, state, action, reward, next_state, done):
        state = np.array(state, dtype=np.float32)
        action = np.array(action, dtype=np.float32).reshape(-1)
        reward = np.array(reward, dtype=np.float32).reshape(-1)
        next_state = np.array(next_state, dtype=np.float32)
        done = np.array(done, dtype=np.float32).reshape(-1)
        experience = (state, action, reward, next_state, done)
        self.buffer.append(experience)

    def sample(self, batch_size):
        batch = random.sample(self.buffer, batch_size)
        try:
            state, action, reward, next_state, done = map(np.stack, zip(*batch))
        except ValueError as e:
            print("ValueError 발생: 모든 입력 배열의 크기가 같아야 합니다.")
            for i, data in enumerate(batch):
                print(f"Sample {i}: State shape {np.shape(data[0])}, Action shape {np.shape(data[1])}, "
                    f"Reward shape {np.shape(data[2])}, Next State shape {np.shape(data[3])}, Done shape {np.shape(data[4])}")
            raise e
        return state, action, reward, next_state, done

    def __len__(self):
        return len(self.buffer)
    
    def clear(self):
        self.buffer.clear()

    def save_buffer(self, env_name, suffix="", save_path=None):
        if not os.path.exists('checkpoints/'):
            os.makedirs('checkpoints/')

        if save_path is None:
            save_path = "checkpoints/sac_buffer_{}_{}".format(env_name, suffix)
        print('Saving buffer to {}'.format(save_path))

        with open(save_path, 'wb') as f:
            pickle.dump(self.buffer, f)

    def load_buffer(self, save_path):
        print('Loading buffer from {}'.format(save_path))

        with open(save_path, "rb") as f:
            self.buffer = pickle.load(f)
            self.position = len(self.buffer) % self.capacity
