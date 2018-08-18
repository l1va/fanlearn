#!/usr/bin/env python
from fl_learning.srv import *
import rospy
import numpy as np
import random
import math

# Set environment parameters
env_width = 10
env_height = 4
num_actions = 4

# Initialize Q-table
Q_table = np.zeros([env_width, env_height, env_width, env_height, env_width, env_height, num_actions])

def learning_service():
    rospy.init_node('learning')
    rospy.Service('determine_action', DetermineAction, learning)
    rospy.spin()

def execute_action(action):
    rospy.wait_for_service('execute_action')
    try:
        execute_action = rospy.ServiceProxy('execute_action', ExecuteAction)
        return execute_action(action)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def learning(obs):
    global Q_table

    # Parameters
    epsilon = 0.4
    learning_rate = 1.0
    discount_factor = 0.95

    # Select action
    if random.random() < epsilon:
        # Random exploratory action
        action = random.randint(0, 3)
    else:
        # Greedy action
        action = np.argmax(Q_table[obs.end_effector_x, obs.end_effector_y, obs.block_x, obs.block_y, obs.goal_x, obs.goal_y, :])

    # Execute action and get observation of next state
    obs_next = execute_action(action)

    # Calculate reward
    reward = calculate_reward(obs)

    # Update Q-table
    Q_table = update_Q_table(Q_table, learning_rate, discount_factor, obs, obs_next, action, reward)

    return DetermineActionResponse(4)

def update_Q_table(Q_table, learning_rate, discount_factor, obs, obs_next, action, reward):
    Q_table[obs.end_effector_x, obs.end_effector_y, obs.block_x, obs.block_y, obs.goal_x, obs.goal_y, action] = Q_table[obs.end_effector_x, obs.end_effector_y, obs.block_x, obs.block_y, obs.goal_x, obs.goal_y, action] + learning_rate * (reward + discount_factor * np.max(Q_table[obs_next.end_effector_x, obs_next.end_effector_y, obs_next.block_x, obs_next.block_y, obs_next.goal_x, obs_next.goal_y, :]) - Q_table[obs.end_effector_x, obs.end_effector_y, obs.block_x, obs.block_y, obs.goal_x, obs.goal_y, action])
    return Q_table

def calculate_reward(obs):
    step_reward = -0.2
    reward = step_reward - calculate_euclidean_distance(obs.block_x - obs.end_effector_x, obs.block_y - obs.end_effector_y) - calculate_euclidean_distance(obs.goal_x - obs.block_x, obs.goal_y - obs.block_y)
    if obs.block_x == obs.goal_x and obs.block_y == obs.block_x:
        reward = 1.0
    return reward

def calculate_euclidean_distance(x, y):
    return math.sqrt(math.pow(x, 2) + math.pow(y, 2))
                     
if __name__ == '__main__':
    learning_service()
