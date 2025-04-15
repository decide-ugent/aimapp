#!/usr/bin/env python3
import numpy as np
import random

from model_mcts_test import MCTS_Model, Node
from map_dm_nav.model.V5 import Ours_V5_RW
import pickle
from pathlib import Path


import matplotlib.pyplot as plt
import networkx as nx

# THIS FILE PROVIDES SKELETON CODE FOR PLANNING USING MCTS - 
# A mix between https://github.com/erwinwalraven/active_inference based on 'Information Gathering in POMDPs using Active Inference'
# and https://github.com/ciamic/MCTS based on https://medium.com/@_michelangelo_/monte-carlo-tree-search-mcts-algorithm-for-dummies-74b2bae53bfa

def plot_mcts_tree(root_node):
    """Visualises the Monte Carlo Tree Search (MCTS) tree."""
    G = nx.DiGraph()  # Directed Graph

    # Recursively extract tree structure
    def add_nodes_edges(node, parent=None, action=None):
        # print('add_nodes_edges',node.id)
        node_label = f"ID: {node.id}\nR: {round(node.get_averaged_efe_score(), 2)}"
        G.add_node(node.id, label=node_label, reward=node.T)

        if parent is not None:
            G.add_edge(parent.id, node.id, action=int(action))  # Add edge with action label
            #G.add_edge(node.id, parent.id, action=rev_action(action))  
        # print('node has children?', node.has_children_nodes())
        if node.has_children_nodes():
            for action, child_node in node.childs.items():
                # print('child id', child_node.id)
                add_nodes_edges(child_node, node, action)
    add_nodes_edges(root_node)

    pos = nx.kamada_kawai_layout(G)
    # Scale positions to increase spacing
    pos = {k: (x * 1.5, y * 1.5) for k, (x, y) in pos.items()}

    # Node colors based on reward
    rewards = [G.nodes[n]['reward'] for n in G.nodes]
    nx.draw(G, pos, with_labels=True, labels=nx.get_node_attributes(G, 'label'),
            node_color=rewards, cmap=plt.cm.cool, node_size=1500, font_size=9, edgecolors="black")

    # Draw edges and action labels
    edge_labels = nx.get_edge_attributes(G, 'action')
    nx.draw_networkx_edge_labels(G, pos, edge_labels=edge_labels, font_size=9, label_pos=0.3)


    plt.title("Monte Carlo Tree Search (MCTS) Visualization")
    plt.show()


def get_best_action_final(node):
    assert isinstance(node, Node)
    all_averaged_efe = [c.get_averaged_efe_score() for c in node.childs.values()]
    print('all_averaged_efe', all_averaged_efe, 'actions', list(node.childs.keys()))
    q_pi, best_action_id = model.infer_best_action_given_actions(all_averaged_efe, list(node.childs.keys()))
    print('q_pi', q_pi.round(2), 'get_best_action_final', best_action_id)
    return best_action_id
    

def simulate(model, node, depth, max_depth):
    assert isinstance(node, Node)
    
    # return 0 if max depth was reached
    if depth == max_depth:
        return 0.0
    
    # if node does not have childs for the actions, we create them first and we do a rollout
    if not node.has_children_nodes():
        node = model.create_node_children(node)
        node = model.define_children_rewards(node, depth)

    current = node
    while current.has_children_nodes():
        # print('here current', current.id)
        childs = current.childs
        #we get all actions leading to max U score and pick one at random as next child
        all_averaged_efe = [c.get_averaged_efe_score() for c in childs.values()]
        print(current.id,'child score', [('id',c.id, 'N',round(c.N,2), 'T',round(c.T,2)) for a,c in childs.items()] )
        print('child efe_av', [('id',c.id,'a',a,'efe_av', round(c.get_averaged_efe_score(),2)) for a,c in childs.items()] )
        q_pi, action = model.infer_best_action_given_actions(all_averaged_efe, list(childs.keys()))
        # print('q_pi', q_pi.round(2), 'selected next action', action)
        current = childs[action]
        print(current.id)
  

    if not current.has_children_nodes():
        current = model.create_node_children(current)
        current = model.define_children_rewards(current, depth)
    #if any children, we pick one at random.
    #next simulate
    else:
        current = random.choice(current.childs)
    current.T = current.T + model.rollout(current, depth)
    current.N += 1      
            
    # update statistics and backpropagate
    parent = current
    #Add the current children child T to parent nodes up to grandgrandma
    while parent.parent:
        parent = parent.parent
        parent.N += 1
        parent.T = parent.T + current.T
    return 1.0


def plan(model, node, depth, max_depth, num_simulations):
    assert isinstance(node, Node)
    for i in range(num_simulations):
        simulate(model, node, depth, max_depth)
    return get_best_action_final(node)
        
def pickle_load_model(store_path:str=None)-> None:
    store_path = Path(store_path)
    with open(store_path, 'rb') as f:
        loaded_model= pickle.load(f)
    return loaded_model

if __name__ == "__main__":
    import datetime
    
    # config parameters
    num_simulations = 20       # this is the number of simulations for planning
    num_steps = 1
        
    # initialise your Model object here
    ours = pickle_load_model('/home/idlab332/workspace/ros_ws/src/map_dm_nav/map_dm_nav/test1_noob_model.pkl')
    model = MCTS_Model(ours)
    map_action_names = ours.get_possible_actions()

    
    print('===== INITIAL BELIEF FOR PLANNING =====')
    # belief_prior = model.get_prior()
    # model.print_belief(belief_prior)
    
    # define true state here
    #state, parent, observation, action_index, next_possible_actions, reward=None
    state_qs = ours.get_belief_over_states()
    actual_state = Node(state_qs = state_qs, pose_id=0, parent=0, observation=ours.get_expected_observation(state_qs), action_index=0, next_possible_actions=None)
    
    print()
    print('Actual state', actual_state.id)
    
    start_time = datetime.datetime.now()
    for i in range(num_steps):

        max_depth = num_steps + 1     # at this depth value=0 is used in the search tree
        a = plan(model,actual_state, i, max_depth, num_simulations)
        execute_a = a
        
        print()
        print('===== PLAN ACTION =====')
        print('Depth:', i)
        print('Selected action:', execute_a, map_action_names[execute_a])
        print('Action values:')
        for action_id in actual_state.childs:
            print('', action_id, map_action_names[action_id], actual_state.childs[action_id].T)
        print('Action visit counts:', [actual_state.childs[action_id].N for action_id in actual_state.childs])
        
        # execute the action in actual_state
        next_state = actual_state.childs[execute_a]
        # plot_mcts_tree(actual_state)
        next_state.detach_parent()
        
        print()
        print('===== EXECUTE IT =====')
        print('Next state', next_state.id)
        print()
        print('Visual Observation:', next_state.observation[0][0].round(2), '\nPose Observation', next_state.observation[0][1].round(2))
        
        # prepare for next iteration
        actual_state = next_state


    print('execution time', datetime.datetime.now() - start_time)

        