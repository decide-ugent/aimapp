import numpy as np
import igraph
import sys
import seaborn as sns
import os
import pickle
import csv
from matplotlib import pyplot as plt
from pathlib import Path
from matplotlib import colors
import matplotlib.cm as cm

from PIL import ImageGrab
import pandas
import networkx as nx

from map_dm_nav.model.modules import from_degree_to_point
import matplotlib
matplotlib.use('Agg')  # Non-interactive backend, works headless

import cv2 #cv2 called before matplotlib can results in errors
def create_custom_cmap(custom_colors) -> colors.ListedColormap:
    return colors.ListedColormap(custom_colors[:]) #,  alpha=None)

def get_cmap() -> colors.ListedColormap:
    custom_colors = (
            np.array(
                [
                    [255, 255, 255],#white 1
                    [255, 0, 0],#red 2
                    [0, 255, 0], #green 3
                    [50,50, 255], #bluish 4
                    [112, 39, 195], #purple5
                    [255, 255, 0], #yellow6
                    [100, 100, 100], #grey7
                    [115, 60, 60], #brown8
                    [255, 0, 255], #flash pink9
                    [80, 145,80], #kaki10
                    [201,132,226], #pink11
                    [75,182,220], #turquoise12
                    [255,153,51], #orange13
                    [255,204,229], #light pink14
                    [153,153,0], #ugly kaki 15
                    [229,255,204], #light green16
                    [204,204,255],#light purple17
                    [0, 153,153], #dark turquoise18
                    [138, 108, 106], #light brown19
                    [108, 115, 92],#ugly green20
                    [149, 199, 152],#pale green21
                    [89, 235, 210], #flashy light blue22
                    [37, 105, 122], #dark blue23
                    [22, 25, 92], #dark purple-blue24
                    [131, 24, 219], #flashy purple25
                    [109, 11, 120], #purple-pink26
                    [196, 145, 191], #pale pink27
                    [148, 89, 130], #dark pink28
                    [201, 75, 119], #pink-red29
                    [189, 89, 92], #light red30




                ]
            )
            / 256
        )

    n_colors = len(custom_colors)
    return create_custom_cmap(custom_colors[:n_colors])

def plot_image(img:np.ndarray, figsize_in_inches:tuple=(25,25), title:str = None, show:bool=True):
    fig, ax = plt.subplots(figsize=figsize_in_inches)
    ax.imshow(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
    ax.axis('off')
    if title is not None:
        plt.savefig(title + '.png', bbox_inches='tight', pad_inches=0)
    if show:
        plt.show()
    return ax
    
def plot_images(img_list:list):
    for i, img in enumerate(img_list):
        plot_image(img, title='test_img_pic'+str(i))

def remove_white_border(image):
    """ panorama are saved with white borders, if any, let's remove them"""
    # Convert the image to grayscale
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    # Threshold the image to create a binary mask of non-white regions
    _, binary = cv2.threshold(gray, 240, 255, cv2.THRESH_BINARY_INV)
    # Find contours of the non-white areas
    contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    # If no contours are found, the image might be all white
    if not contours:
        print("No significant content found in the image.")
        return image
    # Combine all contours into a single bounding box
    x, y, w, h = cv2.boundingRect(np.vstack(contours))
    # Crop the image using the bounding box
    cropped_image = image[y:y+h, x:x+w]
    return cropped_image

def get_efe_frame(action_marginals: np.ndarray) -> np.ndarray:
    """ 
    Plot the EFE of each policy in a matrix-like image with X reversed and Y axis inverted.
    """
    # Reverse the X values in action_marginals and transpose to swap X and Y axes
    reversed_action_marginals = action_marginals.T
    reversed_action_marginals = np.flip(reversed_action_marginals, axis=1).T
    #reversed_action_marginals = action_marginals
    center_x, center_y = reversed_action_marginals.shape[0] // 2, reversed_action_marginals.shape[1] // 2

    fig, ax = plt.subplots()

    # Plot the transposed and reversed action_marginals
    cmap = plt.cm.get_cmap('binary')
    norm = colors.Normalize(vmin=0, vmax=18)
    cax = ax.imshow(reversed_action_marginals, alpha=1, cmap=cmap, norm=norm, zorder=1)

    # Invert the Y-axis to match the correct orientation
    ax.invert_yaxis()

    # Set ticks and labels to be symmetric around the center
    tick_labels_x = np.arange(-center_x, center_x + 1, 1)
    tick_labels_y = - np.arange(-center_y, center_y + 1, 1)
    ax.set_xticks(np.arange(reversed_action_marginals.shape[1]))
    ax.set_xticklabels(tick_labels_x)
    ax.set_yticks(np.arange(reversed_action_marginals.shape[0]))
    ax.set_yticklabels(tick_labels_y)  # No need to negate tick_labels_y

    plt.xlabel('Y', fontsize=15)
    plt.ylabel('X', fontsize=15)
    # plt.title('EFE')

    return fig

def plot_beliefs(Qs:np.ndarray, title:str="current_state_probability")-> np.ndarray:
    """ plot the distribution probabilities of the state """
    fig = plt.figure()
    plt.grid(zorder=0, axis='y')
    plt.bar(range(Qs.shape[0]), Qs, color='r', zorder=3)
    plt.xticks(range(Qs.shape[0]))
    plt.ylabel("proba")
    plt.xlabel("states")
    plt.title(title)
    return fig

def plot_likelihood(A:np.ndarray, state_mapping=None, tittle_add:str='')-> np.ndarray:
    """ plot the proba of an observation given a state"""
    state_labels = [i for i in range(A.shape[1])]
    if state_mapping:
        sorted_state_mapping = dict(sorted(state_mapping.items(), key=lambda x: x[1]['state']))
        state_labels = [next((key, value['state']) for i in range(A.shape[1]) if value['state'] == i) for key, value in sorted_state_mapping.items() ]
    else:
        state_labels = [i for i in range(A.shape[0])]
    fig = plt.figure()
    ax = sns.heatmap(A, cmap = "OrRd", xticklabels = state_labels, cbar = False) #xticklabels =state_labels 
    plt.tight_layout()
    plt.ylabel(tittle_add+" ID")
    plt.xlabel("(pose, state)")
    plt.title(tittle_add + " likelihood distribution (A)")
    return fig
    
import matplotlib.pyplot as plt
import seaborn as sns
import numpy as np

def plot_transitions(B: np.ndarray, state_map: dict, actions: dict) -> np.ndarray:
    """Plot Transitions matrix showing the probability of a transition between two states given a certain action."""
    
    sorted_state_map = dict(sorted(state_map.items(), key=lambda item: item[1]['state']))
    labels = [f"{key} ({value['state']})" for key, value in sorted_state_map.items()]

    n_actions = len(actions)
    l = int(np.ceil(np.sqrt(n_actions)))
    L = int(np.ceil(n_actions / l))
    
    fig, axes = plt.subplots(L, l, figsize=(L*3 + max(10, 2.5*len(state_map)), 
                                             l*2 + max(10, 1.5*len(state_map))))
    
    axes = np.atleast_2d(axes)  # Ensure axes is always a 2D array
    count = 0

    for i in range(L):
        for j in range(l):
            if count >= n_actions:
                fig.delaxes(axes[i][j])
                continue
            
            if count not in actions:
                continue

            action_str = str(actions[count])  # Convert action name to string

            # Plot the heatmap
            g = sns.heatmap(B[:len(labels), :len(labels), count], cmap="OrRd", linewidth=3, 
                            cbar=False, ax=axes[i, j], xticklabels=labels, yticklabels=labels)

            g.tick_params(axis='both', which='major', labelsize=14)  # Adjust label font size
            g.set_title(action_str, fontsize=20)
            g.set_xlabel('Prev State', fontsize=16)
            g.set_ylabel('Next State', fontsize=16)

            # Rotate labels for better visibility
            g.set_xticklabels(labels, rotation=45, ha="right", fontsize=12)
            g.set_yticklabels(labels, rotation=0, fontsize=12)
            
            count += 1

    plt.subplots_adjust(left=0.2, bottom=0.2)  # Add margin space
    plt.tight_layout()

    return fig


def plot_state_in_map_wt_gt(model:object, gt_odom:list, odom:list=None) -> np.ndarray: 
    dim = max(25, int(model.get_n_states()/2))
    fig, ax = plt.subplots(figsize=(dim,dim))  
    circle=plt.Circle((gt_odom[1], gt_odom[0]),radius=0.21,fill=True, color='0.5') #linestyle='dotted'
    plt.gca().add_patch(circle)
    if odom is not None:
        circle2=plt.Circle((odom[1], odom[0]),radius=0.15,fill=True, color='0.2') #linestyle='dotted'
        plt.gca().add_patch(circle2)
    plt.plot()
    fig = plot_state_in_map(model.get_B(),model.get_agent_state_mapping(), fig_ax=[fig,ax])
    return fig

def plot_state_in_map(B: np.ndarray, state_mapping: dict,fig_ax=[None, None]) -> np.ndarray:
    """
    Plot states as dots positioned based on `state_mapping` keys.
    Draw transitions between states based on transition probabilities in `B`.

    Parameters:
    - B (np.ndarray): Transition matrix of shape (num_states, num_states, num_actions).
    - state_mapping (dict): Mapping of (x, y) positions to state properties.
    - possible_actions (dict): Dictionary of action indices to angle ranges.
    - pose_dist (float): Distance associated with each move action.

    Returns:
    - fig (matplotlib Figure): The generated figure.
    """
    if fig_ax[0] is None:
        fig, ax = plt.subplots(figsize=(25, 25))
    else:
        fig = fig_ax[0]
        ax = fig_ax[1]


    # Get unique observation values for color mapping
    unique_obs = np.sort(list({v['ob'] for v in state_mapping.values()}))
    color_map = get_cmap() #get_cmap('viridis', len(unique_obs))
    ob_to_color = {ob: color_map.colors[i] for i, ob in enumerate(unique_obs)}

    # Draw transitions between states
    num_states, _, num_actions = B.shape
    for prev_state in range(num_states):
        for next_state in range(num_states):
            for action in range(num_actions):
                prob = B[next_state, prev_state, action]
                if prob > 0.1:  # Only plot meaningful transitions
                    # Find corresponding positions in `state_mapping`
                    prev_pos = next((pos for pos, data in state_mapping.items() if data['state'] == prev_state), None)
                    next_pos = next((pos for pos, data in state_mapping.items() if data['state'] == next_state), None)
                    
                    if prev_pos and next_pos:
                        ax.plot([prev_pos[1], next_pos[1]], [prev_pos[0], next_pos[0]], 
                                'k-', linewidth=prob * 10)  # Scale linewidth with probability

    # Plot states as dots
    for (x, y), data in state_mapping.items():
        state = data['state']
        ob = data.get('ob', 0)
        color = ob_to_color[ob]

        ax.plot(y, x, 'o', color=color, markersize=20)  # Position state as (y, x)
        ax.text(y - 0.05, x + 0.05, str(state), fontsize=25, ha='right', c='r')  # Label state number

    # Formatting
    # ax.invert_yaxis()
    ax.invert_xaxis()
    ax.set_aspect('equal')
    ax.tick_params(axis='both', which='major', labelsize=26)
    plt.ylabel('X', fontsize=30)
    plt.xlabel('Y', fontsize=30)
    plt.title('State Transitions', fontsize=35)
    plt.grid(False)

def plot_state_in_map_wt_gt_model_transition(model1:object, model2:object, gt_odom:list) -> np.ndarray: 
    """ 
    This alternative function highligts the difference in transitions 
    probabilities between two models of the agent
    getting stronger: green
    geting weaker: red
    """
    dim = max(25, int(model2.get_n_states()/2))
    fig, ax = plt.subplots(figsize=(dim,dim))  
    circle=plt.Circle((gt_odom[1], gt_odom[0]),radius=0.21,fill=True, color='0.5') #linestyle='dotted'
    plt.gca().add_patch(circle)
    plt.plot()
    fig = plot_state_in_map_model_transition(model1, model2, fig_ax=[fig,ax])
    return fig

def plot_state_in_map_model_transition(model1, model2, fig_ax=[None, None]) -> np.ndarray:
    """
    B: transition matrix
    state_mapping: pose:{state, ob}
    possibles actions

    Plot the states at the pose coordinates with dot of colours depending on observation id.
    Blank: no observation
    Adjacent states with an existing transition have a line connecting them (width considering B weight). 

    Return plot
    """

    B1 = model1.get_B()
    state_mapping1 = model1.get_agent_state_mapping()
    possible_actions = model1.possible_directions
    pose_dist = model1.get_pose_dist()

    B2 = model2.get_B()
    state_mapping2 = model2.get_agent_state_mapping()


    direction_mapping = {}
    if fig_ax[0] is None:
        fig, ax = plt.subplots(figsize=(25, 25))
    else:
        fig = fig_ax[0]
        ax = fig_ax[1]
        
    for angle in possible_actions.keys():
        if angle == 'STAY':
            direction_mapping[angle] = (0, 0)
            continue
        motion = from_degree_to_point(float(angle), pose_dist=pose_dist)
        direction_mapping[angle] = motion

    actions = {}
    # Transform the directions into a motion mapping for the plot
    for k, v in possible_actions.items():
        if k in direction_mapping:
            actions[v] = direction_mapping[k]

    # Draw transitions between adjacent states
    for action, (dx, dy) in actions.items():
        for (x, y), data in state_mapping2.items():
            
            state = data['state']
            next_x, next_y = x + dx, y + dy
            if (next_x, next_y) in state_mapping2:
                colour='k'
                show_anyway=False
                next_state = state_mapping2[(next_x, next_y)]['state']
                Trans = B2[next_state, state, action]
                if (x, y) in state_mapping1 and (next_x, next_y) in state_mapping1:
                    Trans1 = B1[next_state, state, action]
                    if Trans1 < Trans:
                        colour='g'
                    elif Trans1 > Trans:
                        colour='r'
                        # print('previous Trans', round(Trans1, 4))
                        # print('current Trans', round(Trans, 4))
                        show_anyway = True
                elif not (next_x, next_y) in state_mapping1 or (x,y) not in state_mapping1:
                    colour = 'g'
                # Print worthwhile transitions with line width considering B weight
                if Trans > 0.005 or show_anyway: 
                    if show_anyway and Trans< 0.005:
                        Trans= 0.05
                    ax.plot([y, next_y], [x, next_x], ls= '-', color=colour, linewidth=Trans * 10)  # Swapping x and y

    # Get unique ob values and assign colors
    unique_obs = np.sort(list({v['ob'] for v in state_mapping2.values()}))
    color_map = get_cmap()
    ob_to_color = {ob: color_map(i) for i, ob in enumerate(unique_obs)}


    ax.invert_yaxis()

    # Plot the states as dots with colors different for each RGB ob
    for (x, y), data in state_mapping2.items():
        state = data['state']
        colour = 'black'
        #If the state is new, then we print it in green
        if (x,y) in state_mapping1:
            prev_model_state = state_mapping2[(x, y)]['state']
            if prev_model_state != state:
                colour = 'green'

        ob = data.get('ob', 0)  # Default to 0 if 'ob' is not present
        color = ob_to_color[ob]
        ax.plot(y, x, 'o', color=color, markersize=20)  # Swapping x and y for the plot
        ax.text(y - 0.05, x + 0.08, str(state), color = colour, fontsize=25, ha='right')  # Annotate state number

    # Only show integer values
    ax.xaxis.set_major_locator(plt.MaxNLocator(integer=True))
    ax.yaxis.set_major_locator(plt.MaxNLocator(integer=True))

    ax.set_aspect('equal')
    ax.tick_params(axis='both', which='major', labelsize=26)
    plt.ylabel('X', fontsize=30)  # X-axis is now vertical, so it's the ylabel
    plt.xlabel('Y', fontsize=30)  # Y-axis is now horizontal, so it's the xlabel
    plt.title('State Transitions', fontsize=35)
    plt.grid(False)
    return fig

#===== IGRAPH PLOT ======#
def plot_mcts_tree(root_node):
    """Visualises the Monte Carlo Tree Search (MCTS) tree."""
    G = nx.DiGraph()  # Directed Graph

    # Recursively extract tree structure
    def add_nodes_edges(node, parent=None, action=None):
        node_label = f"ID: {node.id}\nR: {round(node.state_reward, 2)}"
        G.add_node(node.id, label=node_label, reward=node.state_reward)

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


def plot_graph_as_cscg(A:np.ndarray, agent_state_mapping:dict, \
                       cmap:colors.ListedColormap,store_path:Path, edge_threshold:float= 1):
    """ plot states connections, 
    edge_threshold allows to purge low proba links deepending on its value.
    colours are purely indicative
    """
    
    v = [value['state'] for value in agent_state_mapping.values()]
    obs = [value['ob'] for value in agent_state_mapping.values()]
    #print(pd.DataFrame(A, index=list(range(0,A.shape[0])), columns=list(range(0,A.shape[0])), dtype=float))

    g = igraph.Graph.Adjacency((A > 0).tolist())
    # edge_widths = [np.log(A[i, j]+1)*5 for i, j in g.get_edgelist()]
    # edge_widths = [np.log(A[i, j]+1)*5 for i, j in g.get_edgelist()]
    # edge_widths = [x if x>=edge_threshold else 0 for x in edge_widths]
    # print(edge_widths)
    colors = [cmap(nl)[:3] for nl in obs]
    # plot_name = 'figures/'+ specific_str + 'graph_0'
    # count = 0
    # while os.path.exists(plot_name+'.png'):
    #     count+=1
    #     plot_name = plot_name.replace('graph_'+str(count-1), 'graph_'+str(count))
    file = 'connection_graph_edge_Th_'+str(edge_threshold)+'.png'
    
    out = igraph.plot(
        g,
        store_path / file,
        layout=g.layout("kamada_kawai"),
        vertex_color=colors,
        vertex_label=v,
        vertex_size=30,
        # edge_width=edge_widths,  
        margin=50,
    )
    return out

#===== PLOT ALL PANORAMAS ====#

def plot_panorama_memories_and_odom(hlnav_model:object) -> None:
    """ plot the RGB panorama and ros log the model odometry"""
    hlnav_model.get_logger().info('OdomMemory: %s' % str(hlnav_model.model.PoseMemory.get_odom(as_tuple=True)))
    for i in range(len(hlnav_model.Views.views)):
        print('image id:',hlnav_model.Views.views[i].id)
        # print(type(model.Views.views[i].full_ob))
        print('shape:',hlnav_model.Views.views[i].full_ob.shape)
        plot_image(hlnav_model.Views.views[i].full_ob, title='test_first_ob')

#====== SAVE DATA ======#
def create_save_data_dir(start_folder:str=None) -> Path:
    ''' generate the directory where all the data will be saved'''
    test_id = 0
    if start_folder is None:
        start_folder = Path.cwd()
    else:
        start_folder = Path(start_folder)
    while os.path.isdir( start_folder / 'tests' / str(test_id)):
        test_id+=1
    store_path = start_folder / 'tests' / str(test_id)
    store_path.mkdir(exist_ok=True, parents=True)
    return store_path

def get_save_data_dir(start_folder:str=None) -> Path:
    """ get latest created test directory """
    test_id = 0
    if start_folder is None:
        start_folder = Path.cwd()
    else:
        start_folder = Path(start_folder)
    while os.path.isdir( start_folder / 'tests' / str(test_id)):
        test_id+=1
    store_path = start_folder / 'tests' / str(test_id-1)
    return store_path

def generate_step_save_dir(n_steps: int, store_path:Path=None):
    ''' generate the sub directory where the step relative data will be stored'''
    step = 'step_'+str(n_steps)
    if store_path is None:
        store_path = Path.cwd() / 'tests' /  step
    else:
        store_path = store_path / step
    store_path.mkdir(exist_ok=True, parents=True)
    return store_path

def generate_failed_step_save_dir(n_steps: int, store_path:Path=None):
    ''' generate the sub directory where the step relative data will be stored'''
    step = 'step_'+str(n_steps)+'*'
    if store_path is None:
        store_path = Path(Path.cwd() / 'tests' /  'step*')
    else:
        store_path = store_path / step
    store_path.mkdir(exist_ok=True, parents=True)
    return store_path

def save_step_data(model:object,ob_id:int, ob:np.ndarray, ob_match_score:list, scan_dist:list, gt_odom:list, action_success:bool, elapsed_time:int,
                 store_path:Path=None, action_select_data:dict=None)-> None:
    
    next_possible_actions = model.define_next_possible_actions(scan_dist)
    n_steps = save_data_to_excel(model, ob_id, ob, ob_match_score, next_possible_actions,
                       scan_dist,gt_odom, action_success, elapsed_time, store_path,action_select_data)
    
    store_path = generate_step_save_dir(n_steps, store_path)

    pickle_dump_model(model, store_path)
    save_panorama(ob, ob_id, store_path)
    #save_plot_state_graph(model, n_steps, store_path)
    save_plot_state_in_map(model, gt_odom, store_path)
    #save_B_transitions(model, n_steps,store_path )
    save_plot_beliefs(model.get_belief_over_states()[0], store_path)
    # save_overview_image(overview, store_path)
    #save_screenshot(store_path)
    A = model.get_A()
    save_plot_likelihood(A[0], model.get_agent_state_mapping(), \
                        'observations', store_path)
    save_plot_likelihood(A[1], model.get_agent_state_mapping(), \
                        'poses', store_path)


def save_pose_data(model, ob, ob_id, obstacle_dist_per_actions, gt_odom=None, store_path=None, logs=None):
    '''tempo just to get data to run without env'''
    pose_id = model.current_pose
    visit= 1
    if store_path is None:
        store_path = Path.cwd() / 'tests' / 'poses' 

    pose_visit = str(pose_id) + '_' + str(visit)
    store_path = store_path / pose_visit
    store_path = str(store_path)
    while os.path.exists(store_path):
        visit+=1
        store_path = store_path.replace('_'+str(visit-1), '_'+str(visit))
    store_path = Path(store_path)
    store_path.mkdir(exist_ok=True, parents=True)

    pickle_dump_model(model, store_path)
    save_panorama(ob, ob_id, store_path)
    save_plot_state_in_map(model, model.current_pose, store_path)
    save_plot_beliefs(model.get_belief_over_states()[0], store_path)
    A = model.get_A()
    save_plot_likelihood(A[0], model.get_agent_state_mapping(), \
                        'observations', store_path)
    save_plot_likelihood(A[1], model.get_agent_state_mapping(), \
                        'poses', store_path)
    with open("obstacle_dist_per_actions.txt", "w") as file:
        file.write(str(obstacle_dist_per_actions))

def save_failed_step_data(model:object,ob_id:int, ob:np.ndarray, ob_match_score:list, poss_next_a:list, \
                 scan_dist:list, gt_odom:list, action_success:bool,elapsed_time:int,
                 store_path:Path=None, action_select_data:dict=None):
    n_steps = save_data_to_excel(model, ob_id, ob, ob_match_score, poss_next_a, \
                       scan_dist,gt_odom, action_success, elapsed_time, store_path,action_select_data)
    
    store_path = generate_failed_step_save_dir(n_steps, store_path)
    pickle_dump_model(model, store_path)
    save_plot_state_in_map(model, gt_odom, store_path)
    if action_select_data is not None and 'poses_efe' in action_select_data:
        save_efe_plot(action_select_data['poses_efe'],n_steps,store_path)
    
    # save_overview_image(overview, store_path)

def save_overview_image(overview:np.ndarray, store_path:Path=None):
    if overview is not None :
        cv2.imwrite(str(store_path)+ "/overview.png", overview)

def save_screenshot(store_path:Path=None):
    """ take a screenshot of the full screen"""
    # Capture the entire screen
    screenshot = ImageGrab.grab()
    screenshot.save(str(store_path)+ "/screenshot.png")
    screenshot.close()

def save_B_transitions(model:object, store_path:Path=None) -> None:
    fig = plot_transitions(model.get_B(),model.get_agent_state_mapping(), model.possible_directions)
    plt.savefig(str(store_path)+ "/state_Transitions.png")
    plt.close(fig)

def save_plot_state_in_map(model:object, gt_odom:list, store_path:Path=None) -> None:  
    fig = plot_state_in_map_wt_gt(model,gt_odom)
    plt.savefig(str(store_path)+ "/state_B_plot.png")
    plt.close(fig)
    
def save_efe_plot(action_marginals:np.ndarray,n_steps:int, store_path:Path=None) -> None:
    store_path = generate_step_save_dir(n_steps, store_path)
    fig = get_efe_frame(action_marginals)
    plt.savefig(str(store_path)+ "/EFE_plot.png")
    plt.close(fig)

def save_plot_beliefs(Qs, store_path:Path=None):
    fig = plot_beliefs(Qs, title="current_state_probabilities")
    plt.savefig(str(store_path)+ "/state_proba_dist.png")
    plt.close(fig)

def save_plot_likelihood(A, state_mapping=None, tittle_add='', store_path=None ):
    fig = plot_likelihood(A, state_mapping, tittle_add=tittle_add)
    title = tittle_add + "_likelihood_heatmap.png"
    plt.savefig(str(store_path) + "/" + str(title))
    plt.close(fig)

def save_panorama(ob:np.ndarray, img_id:int, store_path:Path=None) -> None:
    ax = plot_image(ob,title=None,show=False)
    plt.savefig(str(store_path)+'/img_'+str(img_id)+'.png')
    plt.close()

def save_plot_state_graph(model:object, store_path:Path=None) -> None:
    ''' save the state graph plot at each step, 
    poses is only usefull to keep track of the number of steps'''
    cmap = get_cmap()
    agent_state_mapping = model.get_agent_state_mapping()
    Transition_matrix = model.get_B()
    v = [value['state'] for value in agent_state_mapping.values()]
    # obs = [value['ob'] for value in agent_state_mapping.values()]
    T = Transition_matrix[v,:][:,v,:]
    A = T.sum(2).round(1)
    div = A.sum(1, keepdims=True)
    A /= (div + 0.0001)
    e_th = 0.1
    
    # print('store_path for igraph', store_path)
    while e_th < 0.3:
        edge_threshold = e_th
        e_th*=1.1
        A[A < edge_threshold] = 0
        plot_graph_as_cscg(A, agent_state_mapping, cmap, store_path, edge_threshold= edge_threshold)

#====================== CSV process =================================#

def save_data_to_excel(model, ob_id, ob, ob_match_score, poss_next_a, \
                       scan_dist,gt_odom, action_success, elapsed_time, store_path,action_select_data:dict=None):
    n_steps = model.get_current_timestep()
    data = process_data(model, ob_id, ob, ob_match_score, poss_next_a, n_steps, \
                 scan_dist,gt_odom,action_success, elapsed_time, action_select_data)
    save_step_data_to_file(data, store_path)
    return n_steps

def process_data(model:object, ob_id:int, ob:np.ndarray, ob_match_score:list, poss_next_a:list, n_steps:int, \
                 scan_dist:list,gt_odom:list,action_success:bool, elapsed_time:int, action_select_data:dict=None)->dict:
    np.set_printoptions(threshold=sys.maxsize)
    data={'step': n_steps, 'time': elapsed_time , 'action_success': action_success, 'ob_id': ob_id, \
          'ob_match_score':ob_match_score, 'possible_next_actions':poss_next_a, \
          }
    qs = model.get_belief_over_states()[0]

    A = model.get_A()
    data['state_proba'] = qs
    data['highest_state_proba'] = np.argmax(qs)
    data['pose_id'] = model.PoseMemory.pose_to_id(save_in_memory=False)
    data['odom'] = model.PoseMemory.get_odom(as_tuple=False)
    data['gt_odom'] = list(gt_odom)
    if model.action is None:
        action = -1
    else:
        action = model.action[0]
    data['applied_action'] = action
    # data['state_mapping'] = model.get_agent_state_mapping()
    data['scan_dist'] = list(scan_dist)
    data['ob_shape'] = ob.shape
    # data['B matrix'] = str(model.get_B()) #else if B too big i lose info
    # data['A ob matrix'] = A[0]
    # data['A pose matrix'] = A[1]
    if action_select_data is not None:
    #     for k,v in action_select_data.items():
    #         data[k] = v
        # data["bayesian_surprise"]= action_select_data["bayesian_surprise"]
        
        data["qpi"] = action_select_data["qpi"]
        data["efe"] = action_select_data["efe"]
        data["info_gain"] = action_select_data["info_gain"]
        data['utility'] = action_select_data['utility']
        if 'poses_efe' in data:
            data['poses_efe'] = action_select_data["poses_efe"]
    else:
        data["qpi"] = None
        data["efe"] = None
        data["info_gain"] = None
        data['utility'] = None
        if 'poses_efe' in data:
            data['poses_efe'] = action_select_data["poses_efe"]

    return data

def save_step_data_to_file(data:dict, store_path:Path=None) -> None:
    if store_path is None:
        store_path = Path.cwd() / 'tests' 
   
    csv_name = 'steps_data.csv'
    
    #if the csv file exist, we re-write it each time (in case we have more headers)
    if os.path.exists(store_path / csv_name):
        rewrite_file = True
        with open(store_path / csv_name, 'r', encoding="UTF8") as file:
            reader = csv.reader(file)
            rows = list(reader)
        rows[0] = list(data.keys())
    else:
        rewrite_file = False


    with open(store_path / csv_name, 'w', encoding="UTF8", newline='') as file:
        writer = csv.writer(file)
        if rewrite_file:
            writer.writerows(rows)
        else:
            writer.writerow(list(data.keys()))
        writer.writerow(list(data.values()))

    # with open(store_path / csv_name, 'a+', encoding="UTF8") as file:
    #     writer = csv.writer(file)
    #     if no_header:
    #         writer.writerow(list(data.keys()))
    #     writer.writerow(list(data.values()))

#======================= PICKLE MODEL =================================#
def pickle_dump_model(model:object, store_path:Path=None)-> None:
    # Serialize (pickle) the model instance to a file
    if store_path is None:
        store_path = get_save_data_dir()
    with open(store_path / 'model.pkl', 'wb') as f:
        pickle.dump(model, f)

def pickle_load_model(store_path:str=None)-> None:
    if store_path is None:
        store_path = get_save_data_dir()
    else:
        store_path = Path(store_path)
    with open(store_path/ 'model.pkl', 'rb') as f:
        loaded_model= pickle.load(f)
    return loaded_model

#======================= PLOT PATH FROM CSV ===========================#

def plot_odometry_path(csv_file:str, noise_level:float=0.065)-> np.ndarray:
    """ 
    Plot odometry path with color trail
    The noise level of the path can be adjusted to distinguish 
    the back and forths of the agent
    """
    # Read the CSV file
    df = pandas.read_csv(csv_file)
    
    # Check if 'odom' column exists in the CSV
    if 'odom' not in df.columns:
        raise ValueError("CSV file must contain an 'odom' column")
    
    # Extract odometry data
    odom_data = df['odom'].dropna().values
    x_data = np.array([eval(coord)[0] for coord in odom_data])
    y_data = np.array([eval(coord)[1] for coord in odom_data])
    
    if len(x_data) != len(y_data):
        raise ValueError("'x' and 'y' values must have the same length")
    
    # Add random noise to x and y coordinates
    x_data += np.random.uniform(-noise_level, noise_level, size=x_data.shape)
    y_data += np.random.uniform(-noise_level, noise_level, size=y_data.shape)
    
    # Generate indices for plotting
    indices = np.arange(len(odom_data))
    
    # Create a color map
    norm = plt.Normalize(indices.min(), indices.max())
    colors = cm.viridis(norm(indices))
    
    # Plot with color trail
    fig = plt.figure(figsize=(10, 6))
    for i in range(len(indices) - 1):
        plt.plot(x_data[i:i+2], y_data[i:i+2], color=colors[i], lw=2)
    
    plt.title('Agent believed Path')
    plt.xlabel('X')
    plt.ylabel('Y')
      # Set grid at round values
    x_ticks = np.arange(np.floor(x_data.min()), np.ceil(x_data.max()) + 1, 1)
    y_ticks = np.arange(np.floor(y_data.min()), np.ceil(y_data.max()) + 1, 1)
    plt.xticks(x_ticks)
    plt.yticks(y_ticks)
    
    # plt.set_xticklabels(x_ticks)
    # ax.set_yticklabels(y_ticks)
    plt.grid(True, which='both', linestyle='--', linewidth=0.5)
    cbar = plt.colorbar(plt.cm.ScalarMappable(norm=norm, cmap='viridis'), label='Sequence of steps')

    # Set the color bar ticks to round numbers
    tick_values = np.linspace(indices.min(), indices.max(), num=6)
    cbar.set_ticks(tick_values)
    cbar.set_ticklabels([f"{int(round(tick))}" for tick in tick_values])
    plt.show()

    return fig

