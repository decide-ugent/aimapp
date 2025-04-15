#!/usr/bin/env python3
import numpy as np
import random
import pickle
import logging
from map_dm_nav.model.V5 import Ours_V5_RW
import math
import datetime
from pathlib import Path
import matplotlib.pyplot as plt
import networkx as nx

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

# --- Node Class ---
class Node:
    """
    Represents a node in the MCTS tree.
    Stores state information, MCTS statistics, and tree structure links.
    """
    def __init__(self, state_qs, pose_id, parent, action_index, observation=None, initial_reward=0.0):
        self.pose_id = pose_id
        self.id = pose_id  # Using pose_id as a unique identifier for the node

        # State Representation
        self.state_qs = state_qs  # Belief over states (e.g., particle filter weights)
        self.observation = observation # Observation associated with reaching this state (qo_pi)

        # MCTS Statistics
        self.total_reward = 0 #initial_reward # Sum of rewards accumulated through this node (Formerly T)
        self.N = 0 # Visit count

        # Tree Structure
        self.parent = parent # Reference to the parent node
        self.childs = {} # Dictionary mapping action -> child Node
        self.action_index = action_index # Action taken by the parent to reach this node

        # Action Space
        self.possible_actions = None # List of possible actions from this node's state (computed during expansion)
        self.untried_actions = None # Actions not yet explored from this node

        # Intrinsic Reward (EFE components calculated when node is evaluated)
        self.state_reward = initial_reward # The immediate EFE/G calculated for reaching this state

    def get_averaged_reward(self):
        """Calculates the average reward accumulated through this node."""
        if self.N == 0:
            return 0.0 # Avoid division by zero for unvisited nodes
        return self.total_reward / self.N

    def get_ucb1_score(self, c_param=1.41):
        """
        Calculates the UCB1 score for this node.
        Balances exploitation (average reward) and exploration (visit count).

        UCB1 ensures that the search doesn't prematurely focus only on the initially best-looking option but also invests simulations in exploring other potentially promising, but less certain, branches. This leads to more robust and accurate value estimates for the actions at the root node over many simulations.
        """
        if self.N == 0:
            return float('inf') # Prioritize exploring unvisited nodes

        if self.parent is None:
             # Should not happen during selection if root is handled correctly, but added for safety
             parent_visits = self.N
        else:
             parent_visits = self.parent.N

        if parent_visits == 0: # Avoid log(0) or division by zero if parent somehow has 0 visits
            parent_visits = 1

        exploitation_term = self.get_averaged_reward()
        exploration_term = c_param * math.sqrt(math.log(parent_visits) / self.N)

        # logging.debug(f"Node {self.id}: AvgReward={exploitation_term:.3f}, ExploitTerm={exploration_term:.3f}, ParentN={parent_visits}, SelfN={self.N}")

        # Add intrinsic state reward to bias selection towards immediately rewarding states
        # Note: Depending on the scale of state_reward vs rollout_reward, this might need tuning.
        return exploitation_term + exploration_term #+ self.state_reward

    def is_fully_expanded(self):
        """Checks if we verified possible actions from this node leading to a child node."""
        return self.possible_actions is not None

    def select_best_child_UCB(self, c_param:float=1.41)->object:
        """Selects the child with the highest UCB1 score."""
        best_score = -float('inf')
        best_child = None
        for action, child in self.childs.items():
            score = child.get_ucb1_score(c_param)
            logging.debug(f"  Child {child.id} (Action {action}) UCB1: {score:.2f}")
            
            if score > best_score:
                best_score = score
                best_child = child
        #logging.debug(f"Node {self.id}: Selected child {best_child.id if best_child else 'None'} with score {best_score:.2f}")
        return best_child
    
    def all_children_AIF(self)->list:
        """Selects the child with AIF."""
        all_averaged_efe = [c.get_averaged_reward() for c in self.childs.values()]
        # q_pi, best_action_id = self.infer_policy_over_actions(all_averaged_efe, self.possible_actions)
        # logging.debug(f"  Child {child.id} (Action {action}) average_EFE: {score:.2f}")
            
    
        #logging.debug(f"Node {self.id}: Selected child {best_child.id if best_child else 'None'} with score {best_score:.2f}")
        return all_averaged_efe

    def detach_parent(self):
        """Removes the reference to the parent node to allow garbage collection."""
        logging.debug(f"Detaching parent from node {self.id}")
        del self.parent
        self.parent = None

    def has_children_nodes(self):
        """Checks if the node has any child nodes."""
        return len(self.childs) > 0

# --- Model Interface Class ---
class MCTS_Model_Interface:
    """
    Acts as a wrapper or interface to the underlying Active Inference model.
    Provides methods to query the model for transitions, observations, rewards, etc.
    """
    def __init__(self, underlying_model):
        self.model = underlying_model # The actual model object (e.g., Ours_V5_RW instance)
        # Caches can be added here if needed for expensive model calls
        # self.transition_cache = {}
        # self.observation_cache = {}
        # self.reward_cache = {}
        logging.info(f"MCTS_Model_Interface initialized with model type: {type(underlying_model)}")

    def get_possible_actions(self)->dict:
        """Returns a dictionary of possible actions {action_id: action_range} from the current pose."""
        return self.model.get_possible_actions() 

    def id_to_pose(self, pose_id:int)->list:
        return self.model.PoseMemory.id_to_pose(pose_id)
    
    def get_next_node_pose_id(self, current_pose_id:int, action:int)->int:
        """Calculates the next pose ID resulting from taking an action."""   
        odom = self.model.PoseMemory.id_to_pose(current_pose_id)
        next_pose = self.model.PoseMemory.pose_transition_from_action(action, odom=odom)
        next_pose_id = self.model.PoseMemory.pose_to_id(next_pose, save_in_memory=False)
        next_pose = self.model.PoseMemory.id_to_pose(next_pose_id)
        pose_in_action_range = self.model.PoseMemory.pose_in_action_range(action, next_pose, odom= odom) #if we don't reach that pose with that action, we pass
        
        if not pose_in_action_range:
            #logging.warning(f"Action {action} from pose {current_pose_id} leads to unreachable pose {next_pose_id}. Invalid transition.")
            return -1 # Indicate invalid transition
        #logging.info(f"Action {action} from pose {odom}, {current_pose_id} leads to pose {next_pose}, {next_pose_id}. Invalid transition.")
        return next_pose_id

    def get_next_state_belief(self, current_belief_qs:np.ndarray, action:int)->np.ndarray:
        """Predicts the next belief state (qs) given the current belief and action."""
        # This corresponds to the belief state transition model p(qs'|qs, a)
        return self.model.get_next_state_given_action(qs=current_belief_qs, action=action)

    def get_expected_observation(self, next_belief_qs:np.ndarray)->np.ndarray:
        """Calculates the expected observation (qo_pi) given a belief state."""
        # This corresponds to p(o|qs')
        return self.model.get_expected_observation(next_belief_qs)

    def calculate_expected_free_energy(self, next_belief_qs:np.ndarray, expected_observation_qo_pi:np.ndarray)->float:
        """
        Calculates the Expected Free Energy (G) for a potential next state.
        G = Utility + Information Gain
        """
        G = 0.0
        if self.model.use_states_info_gain:
            info_gain = self.model.get_info_gain_term([next_belief_qs]) # Assuming takes a list
            G += info_gain
            # logging.debug(f"  Info Gain Term: {info_gain:.4f}")
        if self.model.use_utility:
            utility = self.model.get_utility_term(expected_observation_qo_pi)
            G += utility
            # logging.debug(f"  Utility Term: {utility:.4f}")
        # logging.debug(f"  Calculated G: {G:.4f}")
        return G

    def infer_policy_over_actions(self, action_values:list, available_actions:list, action_selection:str=None, alpha:float=None):
        """Infers a probability distribution (policy) over actions based on their values (e.g., EFE)."""
        # This likely involves a softmax function as in the original code's example
        q_pi, best_action_id = self.model.infer_best_action_given_actions(action_values, available_actions,action_selection, alpha)
        return q_pi, best_action_id

# --- MCTS Algorithm Class ---
class MCTS:
    """
    Implements the Monte Carlo Tree Search algorithm using an Active Inference model.
    """
    def __init__(self, model_interface:object, c_param:float=1.41):
        self.model_interface = model_interface
        self.c_param = c_param # Exploration parameter for UCB1
        logging.info(f"MCTS initialized with exploration parameter c={c_param}")

    def _select_node(self, root_node:object)->object:
        """Phase 1: Selection - Traverse the tree using UCB1 until a leaf node is reached."""
        current = root_node
        # logging.debug(f"--- Selection Phase Start (Root: {root_node.id}) ---")
        while current.is_fully_expanded() and current.has_children_nodes():
            logging.info(f"  Selected Node {current.id}")
            # logging.debug(f"Selecting from Node {current.id} (N={current.N}, TR={current.total_reward:.3f})")
            #USING UCB
            current = current.select_best_child_UCB(self.c_param)
            #USING AIF
            # children_G = current.all_children_AIF()
            # q_pi, best_action = self.model_interface.infer_policy_over_actions(children_G, current.possible_actions, action_selection='stochastic', alpha=1.0)
            # current = current.childs[best_action]

        logging.info(f"--- Selection Phase End (Selected Node: {current.id}) ---")
        return current

    def _expand_node_in_all_possible_direction(self, node:object)->object:
        """Phase 2: Expansion - Add a new child node for an untried action."""
        
        node.possible_actions =[]
        node.childs = {}
            
        all_possible_actions = list(self.model_interface.get_possible_actions().keys())
        #we save as the current node child each new node created taking an action from current pose 
        for action in all_possible_actions:
            #=== check if new  ===#
            if action not in node.possible_actions:
                next_pose_id = self.model_interface.get_next_node_pose_id(node.pose_id, action)
                if next_pose_id < 0: #no known or valid next node
                    continue
                node.possible_actions.append(action)
            
            #=== action leading to a node, saving believed qs and qo ===#
            # print('node.state_qs', node.state_qs[0].round(4), action)
            next_state_qs = self.model_interface.get_next_state_belief(node.state_qs, action=action)[0]
            # print('next_state_qs', next_state_qs[0][0].round(3))
            qo_pi = self.model_interface.get_expected_observation(next_state_qs)
            #python should erase unreferenced classes. But let's systematise it
            if action in node.childs:
                del node.childs[action]

            # Calculate the immediate reward (Expected Free Energy) for this transition
            # Note: This G is associated with *reaching* the new state.
            child_reward_G = self.model_interface.calculate_expected_free_energy(next_state_qs, qo_pi)

            # Create the new child node
            child_node = Node(
                state_qs=next_state_qs,
                pose_id=next_pose_id,
                parent=node,
                action_index=action,
                observation=qo_pi,
                initial_reward= child_reward_G 
                # Rollout will determine total_reward
                # possible_actions will be determined when the child is expanded later
            )

            #To get a headstart (not necessary)
            # child_node.N +=1
            # child_node.total_reward = child_node.state_reward 
            # parent = child_node
            # while parent.parent:
            #     parent = parent.parent
            #     parent.N += 1
            #     parent.total_reward = parent.total_reward + child_reward_G

            node.childs[action] = child_node
            logging.info(f"from node {node.id} -> Child Node {child_node.id}, expanding with action {action}(Initial G={child_reward_G:.3f})")
            # logging.debug(f"--- Expansion Phase End (Expanded Node: {child_node.id}) ---")
        return node # Return the newly expanded node


    # def _rollout(self, start_node:object, max_depth:int)->float:
        """
        Phase 3: Simulation (Rollout) - Simulate a trajectory from the start_node
        using a default policy (e.g., random actions) and return the cumulative reward (G).
        """
        logging.debug(f"--- Rollout Phase Start (Node: {start_node.id}, Max Depth: {max_depth}) ---")
        
        cumulative_G = 0.0
        depth = 0

        current_node = start_node
        while depth < max_depth:
            # 1. Check possible actions from the *current simulated pose*
            if not current_node.is_fully_expanded() or len(current_node.possible_actions) == 0:
                #current_node = self._expand_node_in_all_possible_direction(current_node)
                # logging.debug(f"  Rollout Depth {depth}: No actions possible from pose {current_sim_pose_id}. Stopping.")
                break # Dead end in simulation

            # 2. Choose an action using the default policy (random)
            action = random.choice(current_node.possible_actions)

            child = current_node.childs[action]
            # 3. Simulate the transition (expected state and observation)
            # next_sim_qs = child.state_qs
            # sim_qo_pi = child.observation

            # 4. Calculate reward (G) for this simulated step
            step_G = child.state_reward
           
            cumulative_G += step_G
            logging.info(f"  Rollout Depth {depth}: Action {action}, Next node{child.id}, StepG={step_G:.3f}, CumulG={cumulative_G:.3f}")

            # 5. Update simulated state
            current_node = child
            depth += 1

        # logging.debug(f"--- Rollout Phase End (Node: {start_node.id}, Total Rollout G: {cumulative_G:.3f}) ---")
        return cumulative_G

    def _rollout(self, start_node, max_depth):
        """
        Phase 3: Simulation (Rollout) - Simulate a trajectory from the start_node
        using a default policy (e.g., random actions) and return the cumulative reward (G).
        """
        # logging.debug(f"--- Rollout Phase Start (Node: {start_node.id}, Max Depth: {max_depth}) ---")
        current_sim_qs = start_node.state_qs
        current_sim_pose_id = start_node.pose_id
        cumulative_G = 0.0
        depth = 1

        current_node = start_node

        while depth < max_depth:
            # 1. Get possible actions from the *current simulated pose*
            if current_node and current_node.is_fully_expanded():
                all_possible_actions = current_node.possible_actions
            else:
                all_possible_actions = list(self.model_interface.get_possible_actions().keys())
            
            if len(all_possible_actions)==0:
                # logging.debug(f"  Rollout Depth {depth}: No actions possible from pose {current_sim_pose_id}. Stopping.")
                break # Dead end in simulation

            # 2. Choose an action using the default policy (random)
            action = random.choice(all_possible_actions)

            # 3. Simulate the transition
            next_sim_pose_id = self.model_interface.get_next_node_pose_id(current_sim_pose_id, action)
            if next_sim_pose_id < 0:
                 # logging.debug(f"  Rollout Depth {depth}: Action {action} from pose {current_sim_pose_id} leads to invalid state. Stopping.")
                 break # Invalid move in simulation

            next_sim_qs = self.model_interface.get_next_state_belief(current_sim_qs, action)[0]
            sim_qo_pi = self.model_interface.get_expected_observation(next_sim_qs)

            # 4. Calculate reward (G) for this simulated step
            step_G = self.model_interface.calculate_expected_free_energy(next_sim_qs, sim_qo_pi)
            cumulative_G += step_G
            logging.info(f"  Rollout Depth {depth}: Action {action}, NextPose {next_sim_pose_id}, StepG={step_G:.3f}, CumulG={cumulative_G:.3f}")

            # 5. Update simulated state
            current_sim_qs = next_sim_qs
            current_sim_pose_id = next_sim_pose_id
            depth += 1

            # 6. If a node exist for that action, retrieve it to get appropriate next actions
            if current_node and current_node.has_children_nodes():
                current_node = current_node.childs.get(action,None)
            else:
                current_node = None

        # logging.debug(f"--- Rollout Phase End (Node: {start_node.id}, Total Rollout G: {cumulative_G:.3f}) ---")
        return cumulative_G / depth
    
    def _backpropagate(self, node, reward):
        """Phase 4: Backpropagation - Update visit counts and total rewards up the tree."""
        # logging.debug(f"--- Backpropagation Start (Node: {node.id}, Reward: {reward:.3f}) ---")
        current = node
        while current is not None:
            current.N += 1
            current.total_reward += reward
            # logging.debug(f"  Updating Node {current.id}: N={current.N}, TR={current.total_reward:.3f}")
            current = current.parent
        # logging.debug(f"--- Backpropagation End ---")


    def run_simulation(self, root_node, max_rollout_depth):
        """Runs a single iteration of the MCTS algorithm (Select, Expand, Simulate, Backpropagate)."""
        # logging.debug(f"=== Starting MCTS Simulation ===")

        # Phase 1: Selection
        selected_node = self._select_node(root_node)

        # Phase 2: Expansion
        # If the selected node is not terminal and not fully expanded, expand it.
        # Check if the node is terminal (add domain-specific logic if needed, e.g., goal reached)
        # is_terminal = False # Placeholder - add condition if applicable
        # if not is_terminal:
        if not selected_node.is_fully_expanded():
            selected_node = self._expand_node_in_all_possible_direction(selected_node)
        else:
            # If fully expanded, the rollout starts from the selected node itself
            # This can happen if selection leads to an already expanded node
            logging.debug(f"Selected node {selected_node.id} is fully expanded, starting rollout from here.")
            #pass


        # Phase 3: Simulation (Rollout)
        # Start rollout from the newly expanded node (or the selected node if expansion wasn't possible/needed)
        reward = self._rollout(selected_node, max_rollout_depth)
        # Add the immediate state reward (G) of the node where the rollout started
        # This connects the immediate EFE gain with the future expected gains from the rollout
        reward += selected_node.state_reward

        # Phase 4: Backpropagation
        self._backpropagate(selected_node, reward)
        
        children_info = [('child id',c.id,'N',c.N,'T', round(c.total_reward,2),'efe_av', round(c.get_averaged_reward(),2)) for a,c in root_node.childs.items()]
        logging.info(f"Root node children stats: {children_info}")
        # logging.debug(f"=== Finished MCTS Simulation ===")


    def plan(self, root_node, num_simulations, max_rollout_depth):
        """Runs the MCTS planning process for a given number of simulations."""
        logging.info(f"Starting MCTS planning from root node {root_node.id} for {num_simulations} simulations.")
        for i in range(num_simulations):
            print()
            logging.info(f"--- Simulation {i+1}/{num_simulations} ---")
            self.run_simulation(root_node, max_rollout_depth)

        # After simulations, determine the best action from the root
        best_action = self.get_best_action(root_node)
        logging.info(f"MCTS planning finished. Best action: {best_action}")
        return best_action

    def get_best_action(self, root_node):
        """Selects the best action from the root node after simulations."""
        if not root_node.childs:
            logging.warning("Root node has no children after simulations. Cannot determine best action.")
            return None # Or a default action

        # Option 1: Choose the most visited child (robust)
        # best_action = max(root_node.childs.keys(), key=lambda action: root_node.childs[action].N)

        # Option 2: Choose the child with the highest average reward (can be greedy)
        # best_action = max(root_node.childs.keys(), key=lambda action: root_node.childs[action].get_averaged_reward())

        # Option 3: Use the model's policy inference based on average rewards (AIF scheme)
        action_values = []
        available_actions = []
        child_info = []
        for action, child in root_node.childs.items():
            avg_reward = child.get_averaged_reward()
            action_values.append(avg_reward)
            available_actions.append(action)
            child_info.append(f"Action {action}: AvgR={avg_reward:.3f}, N={child.N}")
        logging.info(f"Root node children stats: {'; '.join(child_info)}")

        if len(available_actions)==0:
             logging.warning("No valid actions available from root node children.")
             return None

        q_pi, best_action_id = self.model_interface.infer_policy_over_actions(action_values, available_actions)
        logging.info(f"action average G: {action_values}")
        logging.info(f"softmax policies: {q_pi.round(2)}")
        logging.info(f"Selected best action based on policy: {best_action_id}")
        
        # Ensure the selected action is actually one of the children
        if best_action_id not in root_node.childs:
             logging.error(f"Policy selected action {best_action_id} which is not a child of the root node. Available: {list(root_node.childs.keys())}. Falling back to most visited.")
             # Fallback to most visited
             if available_actions:
                best_action_id = max(root_node.childs.keys(), key=lambda action: root_node.childs.get(action).N if root_node.childs.get(action) else -1)
             else:
                 return None # No valid children

        return best_action_id


# --- Utility Functions ---
def plot_mcts_tree(root_node):
    """Visualises the Monte Carlo Tree Search (MCTS) tree."""
    G = nx.DiGraph()  # Directed Graph

    # Recursively extract tree structure
    def add_nodes_edges(node, parent=None, action=None):
        # print('add_nodes_edges',node.id)
        node_label = f"ID: {node.id}\nR: {round(node.get_averaged_reward(), 2)}"
        G.add_node(node.id, label=node_label, reward=node.total_reward)

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
    min_reward = min(rewards) if rewards else 0
    max_reward = max(rewards) if rewards else 1
    node_colors = [(r - min_reward) / (max_reward - min_reward + 1e-6) for r in rewards] # Normalize 0-1

    # Node sizes based on visit count (log scale might be better)
    # sizes = [G.nodes[n]['N'] for n in G.nodes]
    # #node_sizes = [200 + s * 10 for s in sizes] # Linear scaling
    # node_sizes = [200 + 200 * math.log(s + 1) for s in sizes] # Log scaling


    plt.figure(figsize=(12, 8))
    nx.draw(G, pos, with_labels=True, labels=nx.get_node_attributes(G, 'label'),
            node_color=node_colors, cmap=plt.cm.cool,node_size=1500,
            font_size=8, font_weight='bold', edgecolors="black", alpha=0.9)

    # Draw edge labels (actions)
    edge_labels = nx.get_edge_attributes(G, 'action')
    nx.draw_networkx_edge_labels(G, pos, edge_labels=edge_labels, font_size=7, label_pos=0.7)

    plt.title("Monte Carlo Tree Search (MCTS) Visualization")
    plt.show()

def pickle_load_model(store_path: str = None):
    """Loads a pickled model from the specified path."""
    store_path = Path(store_path)
    if not store_path.exists():
        logging.error(f"Model file not found at: {store_path}")
        return None
    try:
        with open(store_path, 'rb') as f:
            loaded_model = pickle.load(f)
            logging.info(f"Model successfully loaded from: {store_path}")
            return loaded_model
    except Exception as e:
        logging.error(f"Failed to load model from {store_path}: {e}")
        return None

# --- Main Execution ---
if __name__ == "__main__":
    # --- Configuration ---
    NUM_SIMULATIONS = 100  # Number of MCTS simulations per planning step
    NUM_STEPS = 1         # Number of actions to take in the environment
    MAX_ROLLOUT_DEPTH = 10 # Maximum depth for the simulation (rollout) phase
    C_PARAM = 5
    MODEL_PATH = '/home/idlab332/workspace/ros_ws/src/map_dm_nav/map_dm_nav/test1_noob_model.pkl' # Path to your pickled model
    PLOT_TREE = False      # Whether to plot the MCTS tree after each planning step

    # --- Initialization ---
    # Load the underlying Active Inference model
    underlying_model = pickle_load_model(MODEL_PATH)
    if underlying_model is None:
        exit() # Stop if model loading failed

    # Create the interface for the MCTS algorithm
    model_interface = MCTS_Model_Interface(underlying_model)

    # Create the MCTS algorithm instance
    mcts = MCTS(model_interface, c_param=C_PARAM) # Adjust c_param if needed

    # Get action names for logging
    map_action_names = model_interface.get_possible_actions() # Assuming pose 0 exists

    # Define the initial state
    initial_pose_id = 0 # Or get from your model/environment
    initial_belief_qs = underlying_model.get_belief_over_states() # Get initial belief
    initial_observation = model_interface.get_expected_observation(initial_belief_qs)
    # Root node has no parent and no action leading to it
    root_node = Node(state_qs=initial_belief_qs,
                     pose_id=initial_pose_id,
                     parent=None,
                     action_index=None,
                     observation=initial_observation)

    logging.info(f"===== Initial Root Node ID: {root_node.id} =====")

    # --- Simulation Loop ---
    current_node = root_node
    start_time = datetime.datetime.now()

    for i in range(NUM_STEPS):
        logging.info(f"\n===== Planning Step {i+1}/{NUM_STEPS} =====")
        logging.info(f"Current State (Node ID): {current_node.id}")

        # Plan the next action using MCTS
        # The root of the search is the current state node
        best_action = mcts.plan(current_node, NUM_SIMULATIONS, MAX_ROLLOUT_DEPTH)

        if best_action is None:
            logging.error("MCTS failed to find a best action. Stopping simulation.")
            break

        action_name = map_action_names.get(best_action, "Unknown")
        logging.info(f"Selected Action: {best_action} ({action_name})")

        # Display action values/visits from the root node
        if current_node.childs:
            child_info_list = []
            for action_id, child in current_node.childs.items():
                 a_name = map_action_names.get(action_id, "?")
                 child_info_list.append(f"  Action {action_id} ({a_name}): AvgR={child.get_averaged_reward():.3f}, N={child.N}")
            logging.info("Root Node Children Details:\n" + "\n".join(child_info_list))
            print('Action visit counts:', [current_node.childs[action_id].N for action_id in current_node.childs])
        else:
             logging.info("Root node has no children explored.")

        # Visualize the tree if enabled
        if PLOT_TREE:
            plot_mcts_tree(current_node)

        # --- Execute the selected action ---
        # In a real robot, this would involve sending the command and getting sensor feedback.
        # Here, we transition to the corresponding child node in the tree.
        if best_action in current_node.childs:
            next_node = current_node.childs[best_action]
            logging.info(f"Executing action {best_action} -> Transitioning to Node {next_node.id}")

            # IMPORTANT: Detach the chosen next state from its parent (the previous state).
            # This makes the chosen next state the new root for the *next* planning step
            # and allows the old parts of the tree to be garbage collected.
            next_node.detach_parent()
            current_node = next_node # Update the current state

            # Log information about the new state (optional)
            # logging.info(f"New State Observation (Visual): {current_node.observation[0][0].round(2)}")
            # logging.info(f"New State Observation (Pose): {current_node.observation[0][1].round(2)}")

        else:
            logging.error(f"Consistency Error: Best action {best_action} not found in children of node {current_node.id}. Stopping.")
            break # Stop if the tree is inconsistent

    end_time = datetime.datetime.now()
    logging.info(f"\n===== Simulation Finished =====")
    logging.info(f"Completed {i+1 if 'i' in locals() else 0} steps.")
    logging.info(f"Final State (Node ID): {current_node.id}")
    print("Total execution time:", end_time - start_time)