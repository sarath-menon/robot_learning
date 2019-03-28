Vanilla Policy Gradient
============

::

  import argparse
  import numpy as np
  from itertools import count

  import torch
  import torch.nn as nn
  import torch.nn.functional as F
  import torch.optim as optim
  from torch.distributions import Categorical

  import rospy
  from geometry_msgs.msg import Twist
  from reinf_env import mobile_robot_reinforce

  parser = argparse.ArgumentParser(description='PyTorch REINFORCE example')
  parser.add_argument('--gamma', type=float, default=0.99, metavar='G',
                      help='discount factor (default: 0.99)')
  parser.add_argument('--seed', type=int, default=543, metavar='N',
                      help='random seed (default: 543)')
  parser.add_argument('--render', action='store_true',
                      help='render the environment')
  parser.add_argument('--log-interval', type=int, default=10, metavar='N',
                      help='interval between training status logs (default: 10)')
  parser.add_argument('--resume', type=str, default=False, metavar='N',
                      help='To load previously saved model (default: False)')
  args = parser.parse_args()

  robot1 = mobile_robot_reinforce()
  torch.manual_seed(args.seed)
  # resume = False

  class Policy(nn.Module):
      def __init__(self):
          super(Policy, self).__init__()
          self.affine1 = nn.Linear(5, 128)
          self.affine2 = nn.Linear(128, 3)

          self.saved_log_probs = []
          self.rewards = []

      def forward(self, x):
          x = F.relu(self.affine1(x))
          action_scores = self.affine2(x)
          return F.softmax(action_scores, dim=1)

  if args.resume=='False': policy = Policy()
  elif args.resume=='True' :
      policy = torch.load("/home/adept/Documents/saved_model_2.pth")
      print(policy.parameters())
      # exit()
      policy.eval()
  optimizer = optim.Adam(policy.parameters(), lr=1e-2)
  eps = np.finfo(np.float32).eps.item()


  def select_action(state):
      state = torch.from_numpy(state).float().unsqueeze(0)
      state = state/state.mean()
      state = state/state.std()
      probs = policy(state)
      m = Categorical(probs)
      action = m.sample()
      print('probs:',probs,'action:',action)
      policy.saved_log_probs.append(m.log_prob(action))
      return action.item()


  def finish_episode():
      R = 0
      policy_loss = []
      rewards = []
      for r in policy.rewards[::-1]:
          R = r + args.gamma * R
          rewards.insert(0, R)
      rewards = torch.tensor(rewards)
      rewards = (rewards - rewards.mean()) / (rewards.std() + eps)
      for log_prob, reward in zip(policy.saved_log_probs, rewards):
          policy_loss.append(-log_prob * reward)
      optimizer.zero_grad()
      policy_loss = torch.cat(policy_loss).sum()
      policy_loss.backward()
      optimizer.step()
      del policy.rewards[:]
      del policy.saved_log_probs[:]


  def main():
      running_reward = 10
      for i_episode in count(1):
          print("i_episode:" ,i_episode)
          state = robot1.reset()
          for t in range(10000):  # Don't infinite loop while learning
              action = select_action(state)
              state ,reward ,done = robot1.step(action)
              # print(reward)
              if args.render:
                  env.render()
              policy.rewards.append(reward)
              if done:
                  print('reward:',reward)
                  break

          running_reward = running_reward * 0.99 + t * 0.01
          finish_episode()
          if i_episode % args.log_interval == 0:
              print('Episode {}\tLast length: {:5d}\tAverage length: {:.2f}'.format(
                  i_episode, t, running_reward))
              torch.save(policy, "/home/adept/Documents/saved_model_2.pth")

          if running_reward > 10000:
              print("Solved! Running reward is now {} and "
                    "the last episode runs to {} time steps!".format(running_reward, t))
              break


  if __name__ == '__main__':
      main()
