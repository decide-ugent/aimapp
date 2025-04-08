#!/usr/bin/env python3
import random
import numpy as np

class MCTS_Model:
    def __init__(self, model, n=1000):
        self.model = model
        self.n = n
        self.num_actions = 2
        
        self.free_energy_cache = {}
        self.belief_transition_cache = {}
        self.particle_filter_cache = {}
        self.expected_observations_cache = {}
        self.feasible_action_cache = {}
        self.map_observations = {}  # key: observation hash, value: observation object
        
        

    def create_node_children(self, node):
    # def create_childs(self):
        '''
        We create one children for each possible action of the game, 
        then we apply such action to a copy of the current node enviroment 
        and create such child node with proper information returned from the action executed
        '''
                   
        possible_actions = self.model.get_possible_actions()
        odom = self.model.PoseMemory.id_to_pose(node.pose_id)
        if node.next_possible_actions is None:
            node.next_possible_actions =[]
            node.childs = {}
        #we save as the current node child each new node created taking an action from current pose 
        for action in possible_actions.keys():
            #=== check if new  ===#
            if action not in node.next_possible_actions:
                next_pose_id = self.define_next_node_position(odom, action)
                if next_pose_id < 0:
                    continue
                node.next_possible_actions.append(action)
            
            #=== action leading to a node, saving believed qs and qo ===#
            next_state_qs = self.model.get_next_state_given_action(qs=node.state_qs, action=action)
            qo_pi = self.model.get_state_observations(next_state_qs)
            #python should erase unreferenced classes. But let's systematise it
            if action in node.childs:
                del node.childs[action]
            node.childs[action] = Node(state_qs = next_state_qs, pose_id=next_pose_id, parent=node, observation=qo_pi, action_index=action, next_possible_actions=None)   
            print('from', node.id, 'creating node', node.childs[action].id, 'with action' , action )                     
            # if child[action].id == self.id:
            #     child[action].T = -50 #BAD
        return node
    
    
    def define_next_node_position(self, odom:list, action:int)-> int:
        
        next_pose = self.model.PoseMemory.pose_transition_from_action(action, odom = odom) # next pose 
        next_pose_id = self.model.PoseMemory.pose_to_id(next_pose, save_in_memory=False) # next pose to id (considering existing points and euclidian distance)
        next_pose = self.model.PoseMemory.id_to_pose(next_pose_id) # returning next pose corresponding to id (might differ from actual transition)
        pose_in_action_range = self.model.PoseMemory.pose_in_action_range(action, next_pose, odom= odom) #if we don't reach that pose with that action, we pass
        if not pose_in_action_range:
            return -1
        return next_pose_id

    def define_child_transition_G(self,child):
        G = 0
        if self.model.use_utility:
            G += self.model.get_utility_term(child.observation)
        if self.model.use_states_info_gain:
            G+= self.model.get_info_gain_term([child.state_qs])

        child.reward = G
        return child
    
    def define_children_rewards(self, node, depth):
        for child in node.childs.values():
            child = self.define_child_transition_G(child)
            child.N += 1
            child.T = child.reward + self.rollout(child, depth)
            parent = child
            while parent.parent:
                parent = parent.parent
                parent.N += 1
                parent.T = parent.T + node.T
        return node 


    def infer_best_action_given_actions(self, G, actions):
        return self.model.infer_best_action_given_actions(G, actions)
        
    def rollout(self,node, max_depth):
        step = 0
        current = node
        cumulated_G = 0
        while current.childs is not None and step < max_depth: 
            action = random.choice(current.next_possible_actions)
            child = current.childs[action]
            if child.reward == 0:
                G = self.define_child_transition_G(child)
                current.childs[action].reward  = G
            current = child
            cumulated_G +=G
            step+=1
        return cumulated_G
import numpy as np
from copy import deepcopy
import random

import matplotlib.pyplot as plt

#NODE OF THE TREE
class Node:
    
    '''
    The Node class represents a node of the MCTS tree. 
    It contains the information needed for the algorithm to run its search.
    '''
    '''
    child: is a dictionary of child nodes which, for every possible action from that state, will tell us what the next state of the game is taking that action
    T: represents the sum of the value of the rollouts that have been started from this node
    N: represents the visit count of the node, i.e. how many times we have visited the node
    game: is the game environment in the current state represented by the node. Its a copy of the original game that we are currently playing, so that we can use it to simulate the search. It represents the current state of the game that will result in applying all the actions from the root node to the node itself
    observation: is the current state of the game in that node, in our example, it will be the 4 values as described above
    done: will tell us if the game at this point is ended, so as to stop the search
    parent: a connection to the parent node, for backpropagation
    action_index: is the index of the action that lead to this node (i.e. is the action that the parent of the node has taken to get into this node)
    
    '''
   
    def __init__(self, state_qs, pose_id, parent, observation, action_index, next_possible_actions=None, reward=None):
        #Will clean this later
        # child nodes
        self.childs = None #(i.e. the next states of the game)
        self.pose_id = pose_id
        self.id = pose_id
        # total rewards from MCTS exploration
        self.T = 0
        
        # visit count
        self.N = 0        
        
        # observation of the environment
        self.observation = observation
        

        # link to parent node
        self.parent = parent
        
        # action index that leads to this node
        self.action_index = action_index
        self.reward = reward
        self.next_possible_actions = next_possible_actions
        self.state_qs = state_qs
        self.state_reward = reward

    def get_averaged_efe_score(self):
        return self.T/ self.N
    
    def getUCBscore(self):
        '''
        This is the upper confidence bound formula that gives a value to the node. 
        It balances exploitation (i.e. pick the best known action) and exploration (i.e. explore new actions):
        The MCTS will pick the nodes with the highest value.        

        The exploitation term is the current estimated value of the node (self.T / self.N).
        The exploration term is inversely proportional to the number of times the node has been visited, with respect to the number of visits of its parent node (sqrt(log(top_node.N) / self.N)).

        There is also a tunable constant c that will give more or less value to the second term.
        '''
        c=1
        # Unexplored nodes have maximum values so we favour exploration
        if self.N == 0:
            return float('inf')
        
        # We need the parent node of the current node 
        top_node = self
        if top_node.parent:
            top_node = top_node.parent
            
        # We use one of the possible MCTS formula for calculating the node value
        UCB_score = (self.T / self.N) + c * np.sqrt(np.log(top_node.N) / self.N) 
        return  UCB_score + self.state_reward
    

    def detach_parent(self):
        # free memory detaching nodes
        del self.parent
        self.parent = None
    
    def has_children_nodes(self):
        if self.childs is None:
            return False
        return len(self.childs) > 0
