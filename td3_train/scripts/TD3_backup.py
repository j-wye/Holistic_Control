import os
import torch
import torch.nn as nn
import torch.nn.functional as F
from torch.optim import Adam
from utils import soft_update, hard_update
from model import GaussianPolicy, QNetwork, DeterministicPolicy
from torch.utils.tensorboard import SummaryWriter
import math
import numpy as np


class Actor(nn.Module):
    def __init__(self, state_dim, action_dim, hidden_dim):
        super(Actor, self).__init__()

        self.layer_1 = nn.Linear(state_dim, hidden_dim)
        self.layer_2 = nn.Linear(hidden_dim, hidden_dim)
        self.layer_3 = nn.Linear(hidden_dim, 1)
        self.tanh = nn.Tanh()

    def forward(self, s):
        s = F.relu(self.layer_1(s))
        s = F.relu(self.layer_2(s))
        a = self.tanh(self.layer_3(s))
        return a


class Critic(nn.Module):
    def __init__(self, state_dim, action_dim, hidden_dim):
        super(Critic, self).__init__()

        self.layer_1 = nn.Linear(state_dim, hidden_dim)
        self.layer_2_s = nn.Linear(hidden_dim, hidden_dim)
        self.layer_2_a = nn.Linear(1, hidden_dim)
        self.layer_3 = nn.Linear(hidden_dim, 1)

        self.layer_4 = nn.Linear(state_dim, hidden_dim)
        self.layer_5_s = nn.Linear(hidden_dim, hidden_dim)
        self.layer_5_a = nn.Linear(1, hidden_dim)
        self.layer_6 = nn.Linear(hidden_dim, 1)

    def forward(self, s, a):
        s1 = F.relu(self.layer_1(s))
        self.layer_2_s(s1)
        self.layer_2_a(a)
        s11 = torch.mm(s1, self.layer_2_s.weight.data.t())
        s12 = torch.mm(a, self.layer_2_a.weight.data.t())
        s1 = F.relu(s11 + s12 + self.layer_2_a.bias.data)
        q1 = self.layer_3(s1)

        s2 = F.relu(self.layer_4(s))
        self.layer_5_s(s2)
        self.layer_5_a(a)
        s21 = torch.mm(s2, self.layer_5_s.weight.data.t())
        s22 = torch.mm(a, self.layer_5_a.weight.data.t())
        s2 = F.relu(s21 + s22 + self.layer_5_a.bias.data)
        q2 = self.layer_6(s2)
        return q1, q2


class TD3(object):
    def __init__(self, num_inputs, action_space, args):

        self.gamma = args.gamma
        self.tau = args.tau
        self.alpha = args.alpha
        self.hidden_size=args.hidden_size
        self.policy_freq = args.policy_freq

        self.policy_type = args.policy

        #self.device = torch.device("cuda" if args.cuda else "cpu")
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        
        #initialize the actor network
        self.actor = Actor(num_inputs, action_space, self.hidden_size).to(self.device)
        self.actor_target = Actor(num_inputs, action_space, self.hidden_size).to(self.device)
        self.actor_target.load_state_dict(self.actor.state_dict())
        self.actor_optimizer = torch.optim.Adam(self.actor.parameters())

        #initialize the critic network
        self.critic = Critic(num_inputs, action_space, self.hidden_size).to(self.device)
        self.critic_target = Critic(num_inputs, action_space, self.hidden_size).to(self.device)
        self.critic_target.load_state_dict(self.critic.state_dict())
        self.critic_optimizer = torch.optim.Adam(self.critic.parameters())

        # if self.policy_type == "Gaussian":
        #     # Target Entropy = −dim(A) (e.g. , -6 for HalfCheetah-v2) as given in the paper
        #     if self.automatic_entropy_tuning is True:
        #         self.target_entropy = -torch.prod(torch.Tensor(action_space.shape).to(self.device)).item()
        #         self.log_alpha = torch.zeros(1, requires_grad=True, device=self.device)
        #         self.alpha_optim = Adam([self.log_alpha], lr=args.lr)

        #     self.policy = GaussianPolicy(num_inputs, action_space.shape[0], args.hidden_size, action_space).to(self.device)
        #     self.policy_optim = Adam(self.policy.parameters(), lr=args.lr)

        # else:
        #     self.alpha = 0
        #     self.automatic_entropy_tuning = False
        #     self.policy = DeterministicPolicy(num_inputs, action_space.shape[0], args.hidden_size, action_space).to(self.device)
        #     self.policy_optim = Adam(self.policy.parameters(), lr=args.lr)

    def select_action(self, state):#, evaluate=False):
        state = torch.FloatTensor(state).to(self.device)
        return self.actor(state).cpu().data.numpy().flatten()
        # if evaluate is False:
        #     action, _, _ = self.policy.sample(state)
        # else:
        #     _, _, action = self.policy.sample(state)
        # return action.detach().cpu().numpy()[0]

    def update_parameters(self, memory, batch_size,i):
        max_Q=float('-inf')
        av_Q=0
        av_loss=0
        # Sample a batch from memory
        state_batch, action_batch, reward_batch, next_state_batch, mask_batch = memory.sample(batch_size=batch_size)

        state_batch = torch.FloatTensor(state_batch).to(self.device)
        next_state_batch = torch.FloatTensor(next_state_batch).to(self.device)
        action_batch = torch.FloatTensor(action_batch).to(self.device).view(-1, 1)
        reward_batch = torch.FloatTensor(reward_batch).to(self.device).view(-1, 1)
        mask_batch = torch.FloatTensor(mask_batch).to(self.device).view(-1, 1)

        next_action = self.actor_target(next_state_batch)

        noise = torch.Tensor(action_batch).data.normal_(0,0.2).to(self.device)
        noise = noise.clamp(-0.5,0.5)
        
        next_action = next_action+noise

        target_Q1,target_Q2 = self.critic_target(next_state_batch,next_action)

        target_Q = torch.min(target_Q1,target_Q2)
        av_Q += torch.mean(target_Q)
        max_Q = torch.max(target_Q)
        target_Q = reward_batch + ((torch.ones_like(mask_batch)-mask_batch)*0.99*target_Q).detach() # here I should change 1 to same size of mask_batch

        current_Q1,current_Q2=self.critic(state_batch, action_batch)
        loss = F.mse_loss(current_Q1, target_Q) + F.mse_loss(current_Q2, target_Q)

        self.critic_optimizer.zero_grad()
        loss.backward()
        self.critic_optimizer.step()

        if i%self.policy_freq == 0:
            actor_grad,_ = self.critic(state_batch,self.actor(state_batch))
            actor_grad = -actor_grad.mean()
            self.actor_optimizer.zero_grad()
            actor_grad.backward()
            self.actor_optimizer.step()

            for param, target_param in zip(self.actor.parameters(),self.actor_target.parameters()):
                target_param.data.copy_(
                    self.tau * param.data + (1 - self.tau) * target_param.data
                )
            for param, target_param in zip(
                    self.critic.parameters(), self.critic_target.parameters()
                ):
                    target_param.data.copy_(
                         self.tau * param.data + (1 - self.tau) * target_param.data
                    )

        av_loss+=loss
        av_loss=av_loss/(i+1)
        return av_loss, av_Q, max_Q 

    # Save model parameters
    def save_checkpoint(self, env_name, suffix="", ckpt_path=None):
        if not os.path.exists('checkpoints/'):
            os.makedirs('checkpoints/')
        if ckpt_path is None:
            ckpt_path = "checkpoints/td3_checkpoint_{}_{}".format(env_name, suffix)
        print('Saving models to {}'.format(ckpt_path))
        torch.save({'actor_state_dict': self.actor.state_dict(),
                    'critic_state_dict': self.critic.state_dict()},ckpt_path)

    # Load model parameters
    def load_checkpoint(self, ckpt_path, evaluate=False):
        print('Loading models from {}'.format(ckpt_path))
        if ckpt_path is not None:
            checkpoint = torch.load(ckpt_path)
            self.actor.load_state_dict(checkpoint['actor_state_dict'])
            self.critic.load_state_dict(checkpoint['critic_state_dict'])