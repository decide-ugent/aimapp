import numpy as np
from copy import deepcopy


from .pymdp import utils
from .pymdp.utils import bayesian_surprise
from .pymdp import control, inference
from .pymdp.learning import update_obs_likelihood_dirichlet
from .pymdp.agent import Agent
from .modules import *
from .odometry import HotEncodedActionOdometry, PoseOdometry
from .pymdp.maths import spm_dot

#==== INIT AGENT ====#
class Ours_V4_RW1(Agent):
    def __init__(self, num_obs=2, num_states=2, dim=2, observations=[0,(0,0)], lookahead=4, \
                 learning_rate_pB=3.0, actions= {'Left':0}, \
                 set_stationary_B=True, inference_algo= 'VANILLA',
                 pose_dist_th:float=1.0) -> None:
        self.agent_state_mapping = {} #testing purposes

        self.PoseMemory = PoseOdometry(pose_dist_th) #HotEncodedActionOdometry(pose_dist_th)
        self.ViewMemory = []
        # self.pose_mapping = []
        self.possible_directions = actions 


        self.preferred_ob = [-1,-1]
        self.set_stationary_B = set_stationary_B
        # self.step_possible_actions = list(self.possible_directions.values())
        self.lookahead_distance = False #lookahead in number of consecutive steps
        self.simple_paths = True # less computationally expensive path than full coverage paths 
                                 #(consider only a few direction and never go back on path)

        observations, agent_params = self.create_agent_params(num_obs=num_obs, num_states=num_states, observations=observations, \
                            learning_rate_pB=learning_rate_pB, dim=dim, lookahead=lookahead, inference_algo = inference_algo)
        super().__init__(**agent_params)
        self.initialisation(observations=observations)
        
    def create_agent_params(self,num_obs:int=2, num_states:int=2, observations:list=[0,(0,0)], 
                       learning_rate_pB:float=3.0, dim:int=2, lookahead:int=4,inference_algo:str='VANILLA'):
        ob = observations[0]
        p_idx = -1
        if dim > 1:
            #start pose in map
            if len(observations) < 2:
                observations.append([0.0,0.0])
            # self.current_pose = observations[1]
            # self.pose_mapping.append(observations[1])
            #p_idx = self.pose_mapping.index(observations[1])
            self.PoseMemory.reset_odom(observations[1])
            p_idx = self.PoseMemory.pose_to_id()
            observations[1] = p_idx
            
        else:
            p_idx = self.PoseMemory.pose_to_id()
        
        self.current_pose = self.PoseMemory.get_odom(as_tuple=True)
        #INITIALISE AGENT PARAMS
        B_agent = create_B_matrix(num_states,len(self.possible_directions))
        if 'STAY' in self.possible_directions and self.set_stationary_B:
            B_agent = set_stationary(B_agent,self.possible_directions['STAY'])
        pB = utils.to_obj_array(B_agent)

        obs_dim = [np.max([num_obs, ob + 1])] + ([np.max([num_obs, p_idx + 1])] if dim > 1 else [])
        A_agent = create_A_matrix(obs_dim,[num_states]*dim,dim)
        pA = utils.dirichlet_like(A_agent, scale = 1)

        return observations, {
            'A': A_agent,
            'B': B_agent,
            'pA': pA,
            'pB': pB,
            'policy_len': lookahead,
            'inference_horizon': lookahead,
            'lr_pB': learning_rate_pB,
            'lr_pA': 3,
            'inference_algo': inference_algo,
            'save_belief_hist': True,
            'action_selection': "stochastic", 
            'use_param_info_gain': False
        }

    def initialisation(self,observations:list=[0,[0,0]], linear_policies:bool=True, E=None):
        """ Initialise agent with first current observation and verify that all parameters 
        are adapted for continuous navigation.
        linear_policies(bool): 
        if False: we try all combinaison of actions (exponential n_action^policy_len  -wth policy_len==lookahead-) )
        if True: We create linear path reaching at a lookahead DISTANCE (not number of consecutive actions) or NUM STEPS
        we make it linear if no 'STAY' actions, else it's polynomial.
        It's not linear because of the STAY action that is irregular and set only at the end of a policy.

        NOTE:linear_policies=True is only tailored for num_factor==1 and len(num_control)==1 
        """
      
        if linear_policies:
            self.init_policies(E)
        self.reset(start_pose=self.PoseMemory.get_poses_from_memory()[0])
        if self.edge_handling_params["use_BMA"] and hasattr(self, "q_pi_hist"): #This is not compatible with our way of moving
            del self.q_pi_hist
            
        self.inference_params_dict = {'MMP':
                    {'num_iter': 6, 'grad_descent': True, 'tau': 0.25},
                    'VANILLA':
                    {'num_iter': 3, 'dF': 1.0, 'dF_tol': 0.001}}
        self.switch_inference_algo(algo_type=self.inference_algo)
        
        #Not necessary step, but cleaner
        self.A[1][:,:] = 0.1 #reset A for cleaner plot and more fair state inference
        self.update_A_with_data(observations,0)
        self.update_agent_state_mapping(self.current_pose, observations, 0)
        self.infer_states(observation = observations, distr_obs=False, partial_ob=None)
        return 
    
    def initialise_current_pose(self, observations:list, z_score:float=2, min_z_score:float=2):
        ''' define our position p '''
        
        
        # if self.current_pose is None:
        #     z_score = 2
        
        p_idx = self.get_current_most_likely_pose(z_score, min_z_score, observations = observations)
        #if we have a pose, replace current inferred pose by the most likely one.
        if p_idx >= 0:
            self.current_pose = self.PoseMemory.id_to_pose(p_idx)
            self.PoseMemory.reset_odom(self.current_pose)
            print('updating believed pose given certitude on state:', self.current_pose)
        elif p_idx < -1:
            self.current_pose = None
        return p_idx
        
    def init_policies(self, E=None):
        policies = create_policies(self.policy_len, self.possible_directions, lookahead_distance=self.lookahead_distance,simple_paths= self.simple_paths)
        self.policies = policies
        assert all([len(self.num_controls) == policy.shape[1] for policy in self.policies]), "Number of control states is not consistent with policy dimensionalities"
        
        all_policies = np.vstack(self.policies)

        assert all([n_c >= max_action for (n_c, max_action) in zip(self.num_controls, list(np.max(all_policies, axis =0)+1))]), "Maximum number of actions is not consistent with `num_controls`"
        # Construct prior over policies (uniform if not specified) 
        if E is not None:
            if not isinstance(E, np.ndarray):
                raise TypeError(
                    'E vector must be a numpy array'
                )
            self.E = E
            assert len(self.E) == len(self.policies), f"Check E vector: length of E must be equal to number of policies: {len(self.policies)}"
        else:
            self.E = self._construct_E_prior()

    #==== GET METHODS ====#
    def set_memory_views(self, views):
        self.ViewMemory = views
    def get_memory_views(self):
        return self.ViewMemory
    
    def get_agent_state_mapping(self, x=None,a=None, agent_pose=None)->dict:
        return self.agent_state_mapping
    
    def get_B(self):
        return self.B[0]
    
    def get_A(self):
        return self.A

    def get_n_states(self):
        return len(self.agent_state_mapping)
        
    def get_belief_over_states(self, Qs=None, n_step_past=0, verbose=False):
        """ 
        Extract a mean qs over policies if qs is in format qs[policy][timestep]
        It choses the current_qs by default unless a n_step_past is given. 
        (only valid when we have a number of steps < self.inference horizon, 
        else it's always qs_step).
        
        """
        if Qs is None:
            Qs = self.qs
            
        if len(Qs) == 1:
            my_qs = Qs
            
        else:
            
            qs_copy = [q[:self.policy_len + 1] for q in Qs] #In case we have policies of various length.
            current_qs_idx = self.qs_step if len(self.prev_obs) > self.inference_horizon \
                                        else np.max([self.qs_step - n_step_past,0])
            qs_mean = np.mean(qs_copy, axis=0)
            my_qs = qs_mean[current_qs_idx]

            if verbose:
                print('get_belief_over_states', current_qs_idx, qs_mean)
                print('get_belief_over_states QS:', my_qs)
                    
        return my_qs

    def get_current_timestep(self):
        return self.curr_timestep
    
    def get_possible_directions(self):
        return self.possible_directions
    
    def get_pose_dist(self):
        return self.PoseMemory.dist_th
    
    def get_current_most_likely_pose(self, z_score:float, min_z_score:float=2, observations:list=[])->int:
        """
        Given a z_scores (usually around 2), is the agent certain about the state. If it is, to which pose does it correspond?
        Return pose -1 if < threhsold, else return pose id.
        If no state stands out at all, we don't know where we are and return -2
        """
        qs = self.get_belief_over_states()[0]
        p_idx = -1
        mean = np.mean(qs)
        std_dev = np.std(qs)
        print('qs mean and std_dev', mean, std_dev)
        # Calculate Z-scores
        z_scores = (qs - mean) / std_dev
        # Get indices of values with Z-score above 2
        outlier_indices = np.where(np.abs(z_scores) > z_score)[0]
        min_outlier_indices = np.where(np.abs(z_scores) > min_z_score)[0]
        
        
        print("Indices of outliers (Z-score >",z_score,"):" , outlier_indices)
        #If we are sure of a state (independent of number of states), we don't have pose as ob and A allows for pose
        if len(outlier_indices) >= 0 and len(observations) < 2 and len(self.A) > 1:
            #If 1 state stands out
            if len(outlier_indices) == 1:
                p_idx = outlier_indices[0]
        #If min_z_scores length is 0, it means no proba is standing out! We don't know where we are   
        elif len(min_outlier_indices) == 0 and len(observations) < 2 and len(self.A) > 1:
            p_idx = -2
        return p_idx

    def get_observation_most_likely_states(self, z_score:float, observations:list=[])->int:
        """
        Given a z_scores (usually around 2), is the agent certain about the state. If it is, to which pose does it correspond?
        Return pose -1 if < threhsold, else return pose id.
        If no state stands out at all, we don't know where we are and return -2
        """
        likelihood = self.get_A()[0][observations[0],:]
        p_idx = -1
        mean = np.mean(likelihood)
        std_dev = np.std(likelihood)
        print('likelihood mean and std_dev', mean, std_dev)
        # Calculate Z-scores
        z_scores = (likelihood - mean) / std_dev
        # Get indices of values with Z-score above 2
        outlier_indices = np.where(np.abs(z_scores) > z_score)[0]       
        
        print("likelihood Indices of outliers (Z-score >",z_score,"):" , outlier_indices)
        #If we are sure of a state (independent of number of states), we don't have pose as ob and A allows for pose
    
        return outlier_indices
    #==== NAVIGATION SETTINGS ====#

    def explo_oriented_navigation(self, inference_algo:str='VANILLA'):
        self.switch_inference_algo(inference_algo)
        self.use_param_info_gain = False
        self.use_states_info_gain = True #Should we
        self.use_utility = False

    def goal_oriented_navigation(self, obs=None, **kwargs):
        inf_algo = kwargs.get('inf_algo', 'VANILLA')
        pref_weight = kwargs.get('pref_weight', 1.0)
        self.switch_inference_algo(inf_algo)
        self.update_preference(obs, pref_weight)
        self.use_param_info_gain = False
        self.use_states_info_gain = False #This make it FULLY Goal oriented
        #NOTE: if we want it to prefere this C but still explore a bit once certain about state 
        #(keep exploration/exploitation balanced) keep info gain
        self.use_utility = True
        # self.inference_horizon = 4 

    def update_preference(self, obs:list, pref_weight:float=1.0):
        """given a list of observations we fill C with thos as preference. 
        If we have a partial preference over several observations, 
        then the given observation should be an integer < 0, the preference will be a null array 
        """
        if isinstance(obs, list):
            self.update_A_dim_given_obs_3(obs, null_proba=[False]*len(obs))

            C = self._construct_C_prior()

            for modality, ob in enumerate(obs):
                if ob >= 0:
                    self.preferred_ob[modality] = ob
                    ob_processed = utils.process_observation(ob, 1, [self.num_obs[modality]])
                    ob = utils.to_obj_array(ob_processed)
                else:
                    ob = utils.obj_array_zeros([self.num_obs[modality]])
                C[modality] = np.array(ob[0]) * pref_weight

            if not isinstance(C, np.ndarray):
                raise TypeError(
                    'C vector must be a numpy array'
                )
            self.C = utils.to_obj_array(C)

            assert len(self.C) == self.num_modalities, f"Check C vector: number of sub-arrays must be equal to number of observation modalities: {agent.num_modalities}"

            for modality, c_m in enumerate(self.C):
                assert c_m.shape[0] == self.num_obs[modality], f"Check C vector: number of rows of C vector for modality {modality} should be equal to {agent.num_obs[modality]}"
        else:
            self.preferred_ob = [-1,-1]
            self.C = self._construct_C_prior()

    def update_agent_state_mapping(self, pose:tuple, ob:list, state_belief:list=None)-> dict:
        """ Dictionnary to keep track of believes and associated obs, usefull for testing purposes"""
        if state_belief is None:
            state = -1
        else:
            state = np.argmax(state_belief)
        #If we already have an ob, let's not squish it with ghost nodes updates
        if pose in self.agent_state_mapping.keys() and self.agent_state_mapping[pose]['ob'] != -1:
            ob[0] = self.agent_state_mapping[pose]['ob']
        self.agent_state_mapping[pose] = {'state' : state , 'ob': ob[0]}
        if len(ob) > 1:
           self.agent_state_mapping[pose]['ob2'] =  ob[1] 
      
        return self.agent_state_mapping
    
    def switch_inference_algo(self, algo_type:str=None):
        if isinstance(algo_type, str):
            self.inference_algo = algo_type

        elif self.inference_algo == "VANILLA":
            self.inference_algo = "MMP" 
        
        else:
            self.inference_algo = "VANILLA" 
        self.inference_params = self.inference_params_dict[self.inference_algo]
    
    def determine_next_state_distance(self, orientation:float):
        ''' 
        expect orientation in radian and return the 
        distance of the corresponding closest point 
        (considering the desired distance between 2 states)
        '''
        pose_dist = self.get_pose_dist()
        deg_orientation = round(np.rad2deg(orientation))
        pose = from_degree_to_point(deg_orientation, pose_dist=pose_dist)
        pose_dist = euclidian_distance([0,0],pose)
        return pose_dist
    
    #==== NAVIGATION METHODS ====#
   
    def infer_pose(self, action:list)->list:
        '''
        Here we consider that action consider the actual action sensed by agent (thus consider no motion)
        and we consider PoseMemory adapted to treat that perception
        '''
        self.PoseMemory.update_odom_given_pose(action)
        if self.current_pose !=None:
            self.current_pose = self.PoseMemory.get_odom(as_tuple=True)[:2]
        return self.current_pose

    def infer_action(self, **kwargs):
        """
        return the best action to take
        possible params (as a dict):
        observations (List): (only if state not been inferred before)
        next_possible_actions (List): constraint the action to take to be among a list of given choices. 
        
        Returns
        ----------
        return action as int and info as dict
        """
        observations = kwargs.get('observations', None)
        next_possible_actions = kwargs.get('next_possible_actions', list(self.possible_directions.values()))
        # qs_hist = self.qs_hist[-2:]
        #prior = np.pad(qs_hist[-2][0], (0, max(len(qs_hist[-2][0]), len(qs_hist[-1][0])) - len(qs_hist[-2][0])), mode='constant')

        prior = self.get_belief_over_states()
        
        #If we are not inferring state at each state during the model update, we do it here
        if observations is not None and self.current_pose is None:
        #If self.current_pose is not None then we have step_update that infer state
            
            #NB: Only give obs if state not been inferred before 
            if len(observations) < len(self.A):
                partial_ob = 0
                            
            elif len(observations) == len(self.A):
                partial_ob = None
                if self.current_pose == None:
                    self.current_pose = observations[1]
                observations[1] = self.PoseMemory.pose_to_id(observations[1])
            
            _, posterior = self.infer_states(observation = observations, distr_obs=False, partial_ob=partial_ob, save_hist=True)
            # print('infer action: self.current_pose', self.current_pose, posterior[0].round(3))
        
        # self.use_param_info_gain = False

        #In case we don't have observations.
        posterior = self.get_belief_over_states()
        print('infer action: inferred prior state', posterior[0].round(3))
        q_pi, efe, info_gain, utility = self.infer_policies(posterior)
        
        poses_efe = None
        # action = self.sample_action(next_possible_actions)
        #TODO: switch back to sample_action once tests done
        action, poses_efe = self.sample_action_test(next_possible_actions)
        
        #NOTE: What we would expect given prev prior and B transition 
        prior = spm_dot(self.B[0][:, :, int(action)], prior)
        
        data = { "qs": posterior[0],
            "qpi": q_pi,
            "efe": efe,
            "info_gain": info_gain,
            "utility": utility,
            "bayesian_surprise": bayesian_surprise(posterior[0].copy(), prior),}
        if poses_efe is not None:
            data['poses_efe'] = poses_efe
        return int(action), data
    
    def infer_position_given_ob(self, ob:int, z_score:float=2, min_z_score:float=2):
        """ 
        Given an observation, disreguarding the pose but considering history nonetheless. 
        Where are we likely to be?"""
        if self.current_pose is None:
            #Just because A growth expects all observations
            p_idx = 0
        else:
            p_idx = self.PoseMemory.pose_to_id(self.current_pose)
        # prev_state_size = agent.num_states[0]
        #3. UPDATE A AND B DIM WITH THOSE DATA
        self.update_A_dim_given_obs_3([ob,p_idx], null_proba=[False,False])
        self.update_B_dim_given_A()
        self.update_C_dim()
        
        _, Qs = self.infer_states([ob], distr_obs=False, save_hist=False, partial_ob=0)
        self.qs = Qs #NOTE: THIS MIGHT WORK ONLY FOR VANILLA
        p_idx = self.initialise_current_pose(observations= [ob], z_score=z_score, min_z_score=min_z_score)
        return p_idx
        
    #==== Update A and B ====#
    def update_A_with_data(self,obs:list, state:int)->np.ndarray:
        """Given obs and state, update A entry """
        A = self.A
        
        for dim in range(self.num_modalities ):
            A[dim][:,state] = 0
            A[dim][obs[dim],state] = 1
        self.A = A
        return A

    def update_A_dim_given_obs_3(self, obs:list,null_proba:list=[True]) -> np.ndarray:
        ''' 
        Verify if the observations are new and fit into the current A shape.
        If not, increase A shape in observation (n row) only.
        '''
        A = self.A
        num_obs, num_states, num_modalities, num_factors = utils.get_model_dimensions(A=A)
        
        # Calculate the maximum dimension increase needed across all modalities
        dim_add = [int(max(0,obs[m] + 1 - num_obs[m])) for m in range(num_modalities)]

        # Update matrices size
        for m in range(num_modalities):
            A[m] = update_A_matrix_size(A[m], add_ob=dim_add[m], null_proba=null_proba[m])
            if self.pA is not None:
                self.pA[m] = utils.dirichlet_like(utils.to_obj_array(A[m]), scale=1)[0]
        num_obs, num_states, num_modalities, num_factors = utils.get_model_dimensions(A=A)
        self.num_obs = num_obs
        self.A = A
        return A

    def update_B_dim_given_A(self)-> np.ndarray:
        """ knowing A dimension, update B state dimension to match"""
        B = self.B
        add_dim = self.A[0].shape[1]-B[0].shape[1]
        if add_dim > 0: 
            #increase B dim
            B = update_B_matrix_size(B, add= add_dim)
            self.pB = update_B_matrix_size(self.pB, add= add_dim, alter_weights=False)
            if len(self.qs) > 1:
                for seq in self.qs:
                    for subseq in seq:
                        subseq[0] = np.append(subseq[0], [0] * add_dim)
            else:
            
                self.qs[0] = np.append(self.qs[0],[0]*add_dim)
        
        self.num_states = [B[0].shape[0]]
        self.B = B
        return B
    
    def update_A_dim_given_pose(self, pose_idx:int,null_proba:bool=True) -> np.ndarray:
        ''' 
        Verify if the observations are new and fit into the current A shape.
        If not, increase A shape and associate those observations with the newest state generated.
        '''
        A = self.A
        num_obs, num_states, num_modalities, num_factors = utils.get_model_dimensions(A=A)
        
        # Calculate the maximum dimension increase needed across all modalities
        dim_add = int(max(0,pose_idx + 1 - num_obs[num_modalities-1]))
        # Update matrices size
        #and associate new observations with the newest state generated
        if dim_add > 0:
            A[0] = update_A_matrix_size(A[0], add_state=dim_add, null_proba=null_proba)
            if num_modalities > 1:
                A[1] = update_A_matrix_size(A[1], add_ob=dim_add, add_state=dim_add, null_proba=null_proba)
                #we search the first fully null or normed column (thus no link between state -> ob) #THIS IS MAINLY FOR SECURITY
                columns_wthout_data = np.sort(np.append(np.where(np.all(A[1] == 1/A[1].shape[0], axis=0))[0], np.where(np.all(A[1] == 0, axis=0))[0]))
                A[1][:, columns_wthout_data[0]] = 0
                A[1][pose_idx, columns_wthout_data[0]] = 1
                self.num_obs[1] = A[1].shape[0]

            if self.pA is not None:
                self.pA = utils.dirichlet_like(utils.to_obj_array(A), scale=1)
                    
        self.num_states = [A[0].shape[1]]
        self.A = A
        return A
    
    def update_C_dim(self):
        if self.C is not None:
            num_obs, num_states, num_modalities, num_factors = utils.get_model_dimensions(A=self.A) 
            for m in range(num_modalities):
                if self.C[m].shape[0] < num_obs[m]:
                    self.C[m] = np.append(self.C[m], [0]*(num_obs[m]- self.C[m].shape[0]))
    
    #==== update Believes ====#
    def update_A_belief(self,obs:list)->None:
        #UPDATE A given all observations
        #IDENTIFY WHERE THE AGENT BELIEVES TO BE
        _, Qs = self.infer_states(obs, distr_obs=False) 
        self.update_A(obs, Qs)
        self.update_A(obs, Qs) #twice to increase effect (not mandatory)

    def update_believes_v2(self, Qs:list, action:int, obs:list)-> None:
        #UPDATE B
        if len(self.qs_hist) > 0:#secutity check
            qs_hist = self.get_belief_over_states(self.qs_hist[-1])
            qs_hist[0] = np.append(qs_hist[0],[0]*\
                                   (len(Qs[0])-len(qs_hist[0])))
            self.update_B(Qs, qs_hist, action, lr_pB = 10) 
            #2 WAYS TRANSITION UPDATE (only if T to diff state)
            if np.argmax(qs_hist[0]) != np.argmax(Qs[0]):
                a_inv = reverse_action(self.possible_directions, action)
                self.update_B(qs_hist, Qs, a_inv, lr_pB = 7)
        self.update_A_belief(obs)
    
    def add_ghost_node_v3(self, qs:np.ndarray, possible_next_actions:list)-> None:
        ''' 
        For each new pose observation, add a ghost state and update the estimated transition and observation for that ghost state.
        '''
        print('Ghost nodes process:')
        # pose = self.PoseMemory.id_to_pose(p_idx)
        print('current position', self.PoseMemory.get_odom(), 'next possible directions', possible_next_actions,'pd',self.possible_directions.values())
        ghost_node = []
        possible_next_actions = np.array(possible_next_actions)
        for action in self.possible_directions.values():
            possible_action = np.where(possible_next_actions[:, 0] == action)[0]
            if len(possible_action) == 0 or possible_next_actions[possible_action[0]][1]<=0: #this mean this action is not deemed possible
            #if action not in possible_next_actions[:,0]: #this mean this action is not deemed possible
                print('enforcing motion:',action,' leads to current state')
                self.update_B(qs, qs, action, lr_pB = 10)
                #ADDING NEGATIVE LR To existing transition with that action
                cur_pose = self.PoseMemory.get_odom().copy()
                n_pose = self.PoseMemory.get_sequential_actions(next = action, possible_actions = self.possible_directions, odom= cur_pose)[0]
                known_poses = self.PoseMemory.get_poses_from_memory().copy()
                if n_pose in known_poses:
                    print('enforcing motion:',action," DOESN'T lead to next existing state at pose: ", n_pose ," (negative lr)")
                    p_idx = known_poses.index(n_pose)
                    # state = [v['state'] for k,v in self.agent_state_mapping.items() if k == n_pose]
                    _, hypo_qs = self.infer_states([p_idx], np.array([action]), partial_ob=1, save_hist=False)
                    self.update_B(hypo_qs, qs, action, lr_pB = -10) 
                    a_inv = reverse_action(self.possible_directions, action)
                    self.update_B(qs, hypo_qs, a_inv, lr_pB = -7)
            else:
                # we extract the action corresponding to desired action and in case there are several, 
                # keep the one predicting the shortest distance range
                action_and_n_states = [sub_list for sub_list in possible_next_actions if sub_list[0] == action]
                action_and_n_states = min(action_and_n_states, key=lambda x: x[1])
                cur_pose = self.PoseMemory.get_odom().copy()
                for dist in range(int(action_and_n_states[1])):
                    next_pose = self.PoseMemory.get_sequential_actions(next = action, possible_actions = self.possible_directions, odom= cur_pose)[0]
                    known_poses = self.PoseMemory.get_poses_from_memory().copy()
                    n_pose_id = self.PoseMemory.pose_to_id(next_pose) 
                    # print('ghost pose', n_pose, n_pose_id,  'known poses', known_poses, 'seq_actions', seq_actions)
                    if n_pose_id >= len(known_poses) :
                        print('a',action,'adding new pose ', next_pose, n_pose_id, ' to known poses')
                        self.update_A_dim_given_pose(n_pose_id, null_proba=False) #we only update pose ob and assign a state to this ob
                        self.update_B_dim_given_A()
                        self.update_C_dim()
                        ghost_node.append((next_pose,n_pose_id,action))
                    cur_pose = next_pose

        prev_action = -1
        qs = self.get_belief_over_states()
        #Once the dimensions added in all directions, we update B, done in two steps so all probabilities are equivalent
        #since proba dim == for all state inferring.
        for (next_pose,n_pose_id,action) in ghost_node:
            print('we update B now: a',action,'n pose', next_pose, n_pose_id)
            #If we are not iterating in same dierction
            #reinitialise qs to default
            if action != prev_action:
                prev_qs = qs.copy()
            
            _, hypo_qs = self.infer_states([n_pose_id], np.array([action]), partial_ob=1, save_hist=False, qs=prev_qs)
            #Failsafe
            # if len(qs[0]) < len(hypo_qs[0]):
            #     qs[0] = np.append(qs[0],[0]*(len(hypo_qs[0])-len(qs[0])))
            
            self.update_B(hypo_qs, prev_qs, action, lr_pB = 5) 
            self.update_agent_state_mapping(tuple(next_pose[:2]), [-1, n_pose_id], hypo_qs[0])
            a_inv = reverse_action(self.possible_directions, action)
            self.update_B(prev_qs, hypo_qs, a_inv, lr_pB = 3)
            prev_action = action
            prev_qs = hypo_qs

    def update_imagined_transitions(self, qs, pose, possible_next_actions):   
        """ Update adjacent transitions if we believe we can reach them (only 1 step ahead). 
        There is a risk of enforcing wrong Transition and reducing other correct Transition in the process  """
        qs = self.get_belief_over_states()
        known_poses = self.PoseMemory.get_poses_from_memory().copy()
        possible_next_actions = np.array(possible_next_actions)
        for action in self.possible_directions.values():
            n_pose = self.PoseMemory.get_sequential_actions(next = action, possible_actions = self.possible_directions, odom=pose)[0]
            
            if n_pose not in known_poses: #if not known place, we pass
                continue
            p_idx = known_poses.index(n_pose)
            _, hypo_qs = self.infer_states([p_idx], np.array([action]), partial_ob=1, save_hist=False)
            possible_action = np.where(possible_next_actions[:, 0] == action)[0]
            print('action', action, 'n_pose',n_pose,'possible_action',possible_action)
            #If we can't go in this direction according to the next directions list
            if len(possible_action) == 0 or possible_next_actions[possible_action[0]][1]<=0:
                lr_pB_direct_way = -5
                lr_pB_reverse_way = -3
            else:
                lr_pB_direct_way = 5
                lr_pB_reverse_way = 3
            self.update_B(hypo_qs, qs, action, lr_pB = lr_pB_direct_way) 
            a_inv = reverse_action(self.possible_directions, action)
            self.update_B(qs, hypo_qs, a_inv, lr_pB = lr_pB_reverse_way)

    def update_ghost_nodes(self, pose, ob, p_idx,possible_next_actions):
        """ 
        update agent state mapping (tests purposes) 
        add the ghost nodes to the graph
        """
        qs = self.get_belief_over_states()
        # agent_state_mapping for TEST PURPOSES
        self.update_agent_state_mapping(pose, [ob,p_idx], qs[0])
        
        #ADD KNOWLEDGE WALL T OR GHOST NODES
        #inv_action = reverse_action(self.possible_directions, action) #just to gain some computation time
        self.add_ghost_node_v3(qs,possible_next_actions)

    #==== PYMDP modified methods ====#
    def reset(self, init_qs:np.ndarray=None, start_pose:tuple=None):
        """
        Resets the posterior beliefs about hidden states of the agent to a uniform distribution, and resets time to first timestep of the simulation's temporal horizon.
        Returns the posterior beliefs about hidden states.

        Returns
        ---------
        qs: ``numpy.ndarray`` of dtype object
           Initialized posterior over hidden states. Depending on the inference algorithm chosen and other parameters (such as the parameters stored within ``edge_handling_paramss),
           the resulting ``qs`` variable will have additional sub-structure to reflect whether beliefs are additionally conditioned on timepoint and policy.
            For example, in case the ``self.inference_algo == 'MMP' `, the indexing structure of ``qs`` is policy->timepoint-->factor, so that 
            ``qs[p_idx][t_idx][f_idx]`` refers to beliefs about marginal factor ``f_idx`` expected under policy ``p_idx`` 
            at timepoint ``t_idx``. In this case, the returned ``qs`` will only have entries filled out for the first timestep, i.e. for ``q[p_idx][0]``, for all 
            policy-indices ``p_idx``. Subsequent entries ``q[:][1, 2, ...]`` will be initialized to empty ``numpy.ndarray`` objects.
        """

        self.curr_timestep = 0
        self.action = None
        self.prev_actions = None
        self.prev_obs = []
        self.qs_step = 0
     
        self.current_pose = start_pose
        if init_qs is None:
            
            self.D = self._construct_D_prior()
           
            if hasattr(self, "q_pi_hist"):
                self.q_pi_hist = []

            if hasattr(self, "qs_hist"):
                self.qs_hist = []
            
            if self.inference_algo == 'VANILLA':
                self.qs = utils.obj_array_uniform(self.num_states)
            else: # in the case you're doing MMP (i.e. you have an inference_horizon > 1), we have to account for policy- and timestep-conditioned posterior beliefs
                self.qs = utils.obj_array(len(self.policies))
                for p_i, _ in enumerate(self.policies):
                
                    self.qs[p_i] = utils.obj_array_uniform(\
                        [self.num_states] * (self.inference_horizon + self.policy_len + 1)) # + 1 to include belief about current timestep
                    #self.qs[p_i][0] = utils.obj_array_uniform(self.num_states)
                
                first_belief = utils.obj_array(len(self.policies))
                for p_i, _ in enumerate(self.policies):
                    first_belief[p_i] = copy.deepcopy(self.D) 
                
                if self.edge_handling_params['policy_sep_prior']:
                    self.set_latest_beliefs(last_belief = first_belief)
                else:
                    self.set_latest_beliefs(last_belief = self.D)
        
        else:
            self.qs = init_qs

        return self.qs
    
    def update_B(self,qs:np.ndarray, qs_prev:np.ndarray, action:int, lr_pB:int=None)-> np.ndarray:
        """
        Update posterior beliefs about Dirichlet parameters that parameterise the transition likelihood 
        
        Parameters
        -----------
        qs_prev: 1D ``numpy.ndarray`` or ``numpy.ndarray`` of dtype object
            Marginal posterior beliefs over hidden states at previous timepoint.

        Returns
        -----------
        qB: ``numpy.ndarray`` of dtype object
            Posterior Dirichlet parameters over transition self (same shape as ``B``), after having updated it with state beliefs and actions.
        """
        
        if lr_pB is None:
            lr_pB = self.lr_pB

        qB = update_state_likelihood_dirichlet(
            self.pB,
            self.B,
            [action],
            qs,
            qs_prev,
            lr_pB,
            self.factors_to_learn
        )
        qB[0] = np.maximum(qB[0], 0.005) #No negative value (failsafe because of lr possibly negative)
        #no 0 values because 0 values can't variate anymore
        self.pB = qB # set new prior to posterior
        self.B = utils.norm_dist_obj_arr(qB)  # take expected value of posterior Dirichlet parameters to calculate posterior over B array
        return qB
    
    def update_B_given_unreachable_pose(self,next_pose, action):
        """ We reduce transition probability between those 2 states that do not connect"""
        if self.current_pose is not None and next_pose in self.PoseMemory.get_poses_from_memory() :
            n_pose_id = self.PoseMemory.pose_to_id(next_pose)
            qs = self.get_belief_over_states()
            _, hypo_qs = self.infer_states([n_pose_id], np.array([action]), partial_ob=1, save_hist=False)

            # print(self.B[0][np.argmax(hypo_qs[0])][np.argmax(qs[0])][action], self.B[0][np.argmax(qs[0])][np.argmax(hypo_qs[0])][action])
            # print(self.B[0][np.argmax(qs[0])][np.argmax(hypo_qs[0])][action], self.B[0][np.argmax(hypo_qs[0])][np.argmax(qs[0])][action])
            self.update_B(hypo_qs, qs,action,lr_pB=-10)
            a_inv = reverse_action(self.possible_directions, action)
            self.update_B(qs,hypo_qs,a_inv,lr_pB=-7)

    def update_A(self, obs, qs=None):
        """
        Update approximate posterior beliefs about Dirichlet parameters that parameterise the observation likelihood or ``A`` array.

        Parameters
        ----------
        observation: ``list`` or ``tuple`` of ints
            The observation input. Each entry ``observation[m]`` stores the index of the discrete
            observation for modality ``m``.

        Returns
        -----------
        qA: ``numpy.ndarray`` of dtype object
            Posterior Dirichlet parameters over observation self (same shape as ``A``), after having updated it with observations.
        """
        if qs is None:
            qs = self.qs
        qA = update_obs_likelihood_dirichlet(
            self.pA, 
            self.A, 
            obs, 
            qs, 
            self.lr_pA, 
            self.modalities_to_learn
        )

        self.pA = qA # set new prior to posterior
        self.A = utils.norm_dist_obj_arr(qA) # take expected value of posterior Dirichlet parameters to calculate posterior over A array

        return qA
    
    def infer_states(self, observation:list, action:int= None ,save_hist:bool=True, 
                     distr_obs:bool = False, partial_ob:int=None, qs:list=None):
        """
        Update approximate posterior over hidden states by solving variational inference problem, given an observation.

        Parameters
        ----------
        observation: ``list`` or ``tuple`` of ints
            The observation input. Each entry ``observation[m]`` stores the index of the discrete
            observation for modality ``m``.

        Returns
        ---------
        qs: ``numpy.ndarray`` of dtype object
            Posterior beliefs over hidden states. Depending on the inference algorithm chosen, the resulting ``qs`` variable will have additional sub-structure to reflect whether
            beliefs are additionally conditioned on timepoint and policy.
            For example, in case the ``self.inference_algo == 'MMP' `` indexing structure is policy->timepoint-->factor, so that 
            ``qs[p_idx][t_idx][f_idx]`` refers to beliefs about marginal factor ``f_idx`` expected under policy ``p_idx`` 
            at timepoint ``t_idx``.
        """
        # print('infer state',self.inference_algo, action)
        observation = tuple(observation) if not distr_obs else observation
        if save_hist:
            self.prev_obs.append(observation)
            observations_hist = self.prev_obs
        else:
            observations_hist = self.prev_obs.copy()
            observations_hist.append(observation)

        if action != None:
            if self.prev_actions != None:
                prev_actions = self.prev_actions.copy()
            else:
                prev_actions = []
            prev_actions.append(action)
        else:
            prev_actions = self.prev_actions
            action = self.action

        if len(observations_hist) > self.inference_horizon:
            latest_obs = observations_hist[-self.inference_horizon:]
            latest_actions = prev_actions[-(self.inference_horizon-1):]
        else:
            latest_obs = observations_hist
            latest_actions = prev_actions
        
        if partial_ob is None and len(latest_obs[0]) != len(latest_obs[-1]):
            self.qs_step = 0
            self.prev_actions = None
            self.prev_obs = []
            if save_hist:
                self.prev_obs = [latest_obs[-1]]
            latest_obs = [latest_obs[-1]]
            latest_actions = self.prev_actions

        if self.inference_algo == "VANILLA":
            if action is not None:
                if qs is None:
                    qs = self.get_belief_over_states() #we don't want to consider current obs to selest qs
                empirical_prior = control.get_expected_states(
                    qs, self.B, action.reshape(1, -1) #type: ignore
                )[0]
            else:
                self.D = self._construct_D_prior() #self.D
                empirical_prior = self.D
            if self.current_pose is None:
                #TODO: increase A with observation even when self.current_pose is None
                for i in range(len(self.A)):
                    if partial_ob != None:
                        i = partial_ob
                    if observation[i] >= len(self.A[i]):
                        print('ERROR IN INFER STATE: given observation not in A')
                        qs = self.get_belief_over_states()
                        mean_qs_over_policies = qs.copy()
                        return qs, mean_qs_over_policies
            qs = update_posterior_states(
            self.A,
            observation,
            empirical_prior,
            partial_ob,
            **self.inference_params
            )
            F = 0
            mean_qs_over_policies = qs.copy()
            qs_step = 0
        elif self.inference_algo == "MMP":

            if not hasattr(self, "qs"):
                self.reset()
    
            prior = self.latest_belief

            #MMP 
            if isinstance(prior[0][0], np.ndarray):  # Check if nested array
                for i in range(len(prior)):
                    if len(prior[i][0]) < self.num_states[0]:
                        prior[i][0] = np.append(prior[i][0], [0] * (self.num_states[0] - len(prior[i][0])))
                self.latest_belief = prior
                if not self.edge_handling_params['policy_sep_prior']:
                    prior = np.mean(prior, axis=0)
      
            
            elif len(prior[0]) < self.num_states[0]:
                prior[0] = np.append(prior[0], [0] * (self.num_states[0] - len(prior[0])))
                self.latest_belief = prior
                self.D = self._construct_D_prior()
     

            # print('latest_obs',latest_obs)
            # print('latest_actions',latest_actions)
            # print('partial_ob', partial_ob)
            # print('prior', self.latest_belief)
            qs, F = update_posterior_states_full(
                self.A,
                self.B,
                latest_obs,
                self.policies, 
                latest_actions, 
                prior = prior, 
                policy_sep_prior = self.edge_handling_params['policy_sep_prior'],
                partial_ob = partial_ob,
                **self.inference_params
            )
  
            selected_qs = [q[:self.policy_len + 1] for q in qs]
            mean_qs = np.mean(selected_qs, axis=0)
            qs_step = len(latest_obs)-1
            mean_qs_over_policies = mean_qs[qs_step]
            # print('current full qs mean', mean_qs)
            # print('current_qs',mean_qs_over_policies, 'qs idx', self.qs_step, 'save hist', save_hist)
        if save_hist:
            self.F = F # variational free energy of each policy  
            self.qs_step = qs_step
            if hasattr(self, "qs_hist"):
                self.qs_hist.append(qs)
            self.qs = qs

        return qs, mean_qs_over_policies

    def infer_policies(self, qs=None):
        """
        Perform policy inference by optimizing a posterior (categorical) distribution over policies.
        This distribution is computed as the softmax of ``G * gamma + lnE`` where ``G`` is the negative expected
        free energy of policies, ``gamma`` is a policy precision and ``lnE`` is the (log) prior probability of policies.
        This function returns the posterior over policies as well as the negative expected free energy of each policy.

        Returns
        ----------
        q_pi: 1D ``numpy.ndarray``
            Posterior beliefs over policies, i.e. a vector containing one posterior probability per policy.
        G: 1D ``numpy.ndarray``
            Negative expected free energies of each policy, i.e. a vector containing one negative expected free energy per policy.
        """
        if qs is None:
            qs = self.qs

        if self.inference_algo == "VANILLA":
            #If we want to increase the precision of the utility 
            # term on A, we can play with the section below.
            #Currently unused 
            if self.use_utility:
                A = copy.deepcopy(self.A)
                # for modality, array in enumerate(A[0]):
                #     # Compute normalization
                #     summed = array.sum(axis=0, keepdims=True)
                #     # print(summed)
                #     A[0][modality] = array * 10 / summed
            else:
                A = self.A
            q_pi, G, info_gain, utility = update_posterior_policies(
                qs,
                A,
                self.B,
                self.C,
                self.policies,
                self.use_utility,
                self.use_states_info_gain,
                self.use_param_info_gain,
                self.pA,
                self.pB,
                E = self.E,
                gamma = self.gamma,
                diff_policy_len = self.lookahead_distance
            )
        elif self.inference_algo == "MMP":
            # if qs is None:
            #     future_qs_seq = self.get_future_qs()
            info_gain, utility = [],[]
            q_pi, G = update_posterior_policies_full(
                qs, #future_qs_seq
                self.A,
                self.B,
                self.C,
                self.policies,
                self.use_utility,
                self.use_states_info_gain,
                self.use_param_info_gain,
                self.latest_belief,
                self.pA,
                self.pB,
                F = self.F,
                E = self.E,
                gamma = self.gamma,
                diff_policy_len = self.lookahead_distance
            )

        if hasattr(self, "q_pi_hist"):
            self.q_pi_hist.append(q_pi)
            if len(self.q_pi_hist) > self.inference_horizon:
                self.q_pi_hist = self.q_pi_hist[-(self.inference_horizon-1):]
            

        self.q_pi = q_pi
        self.G = G
        return q_pi, G, info_gain, utility
    
    def sample_action(self, possible_first_actions:list=None):
        """
        Sample or select a discrete action from the posterior over control states.
        This function both sets or cachés the action as an internal variable with the agent and returns it.
        This function also updates time variable (and thus manages consequences of updating the moving reference frame of beliefs)
        using ``self.step_time()``.

        Returns
        ----------
        action: 1D ``numpy.ndarray``
            Vector containing the indices of the actions for each control factor
        """
        if possible_first_actions != None:
            #Removing all policies leading us to uninteresting or forbiden action. //speed computation//
            policies, q_pi = zip(*[(policy, self.q_pi[p_id]) for p_id, policy \
                                   in enumerate(self.policies) if policy[0] in possible_first_actions])
        else:
            policies =  self.policies
            q_pi = self.q_pi
        if self.sampling_mode == "marginal":
            action = control.sample_action(
                q_pi, policies, self.num_controls, action_selection = self.action_selection, alpha = self.alpha
            )
        elif self.sampling_mode == "full":
            action = control.sample_policy(q_pi, policies, self.num_controls,
                                           action_selection=self.action_selection, alpha=self.alpha)
        self.action = action
        self.step_time()

        return action
    
    #==== BELIEVES MAIN PROCESS UPDATE ====#
    def agent_step_update(self, action, observations = [0,(0,0)], possible_next_actions:list=[[0,1],[1,1],[2,1],[3,1]]):
        
        #We only update A and B if we have inferred a current pose
        #Thus until doubt over current loc is not solved, we don't update internal self
        if self.current_pose != None:
            print('Updating model given observations', observations)
            ob = observations[0]
            if len(observations) > 1:
                self.current_pose = observations[1]
        
            pose = self.current_pose

            p_idx = self.PoseMemory.pose_to_id(pose)
            # prev_state_size = agent.num_states[0]
            #3. UPDATE A AND B DIM WITH THOSE DATA
            self.update_A_dim_given_obs_3([ob,p_idx], null_proba=[False,False])
            self.update_B_dim_given_A()
            self.update_C_dim()
            # new_state_size = agent.num_states[0]
            #4. UPDATE BELIEVES GIVEN OBS
            _, Qs = self.infer_states([ob,p_idx], distr_obs=False, save_hist=False)
            print('prior on believed state; action', action, 'colour_ob:', ob, 'inf pose:',pose,'belief:', Qs[0].round(3))
            
            

            if action < 0:
                return 
            #4.5 UPDATE A AND B WITH THOSE BELIEVES
            self.update_believes_v2(Qs, action, [ob,p_idx])
            self.update_imagined_transitions(Qs, pose,possible_next_actions)
            #5. ADD Ghost nodes
            self.update_ghost_nodes(pose, ob, p_idx,possible_next_actions)

            #This is not mandatory, just a gain of time
            if 'STAY' in self.possible_directions:
                self.B[0] = set_stationary(self.B[0], self.possible_directions['STAY'])

    #==== TEMPO TEST MODULE ====#

    def sample_action_test(self, possible_first_actions:list=None): 
        """
        Sample or select a discrete action from the posterior over control states.
        This function both sets or cachés the action as an internal variable with the agent and returns it.
        This function also updates time variable (and thus manages consequences of updating the moving reference frame of beliefs)
        using ``self.step_time()``.

        Returns
        ----------
        action: 1D ``numpy.ndarray``
            Vector containing the indices of the actions for each control factor
        """
        if possible_first_actions != None:
            #Removing all policies leading us to uninteresting or forbiden action. //speed computation//
            policies, q_pi = zip(*[(policy, self.q_pi[p_id]) for p_id, policy \
                                   in enumerate(self.policies) if policy[0] in possible_first_actions])
        else:
            policies =  self.policies
            q_pi = self.q_pi


        poses_efe = self.actions_EFE(q_pi,policies)
        
        if self.sampling_mode == "marginal":
            action = control.sample_action(
                q_pi, policies, self.num_controls, action_selection = self.action_selection, alpha = self.alpha
            )
        elif self.sampling_mode == "full":
            action = control.sample_policy(q_pi, policies, self.num_controls,
                                           action_selection=self.action_selection, alpha=self.alpha)
        self.action = action
        self.step_time()

        return action, poses_efe
    
    def actions_EFE(self,q_pi:list, policies:np.ndarray)->np.ndarray:
        
        """
        Computes the marginal posterior over actions and then samples an action from it, one action per control factor.
        Internal testing version that returns the marginal posterior over actions, and also has a seed argument for reproducibility.

        Parameters
        ----------
        
        policies: ``list`` of 2D ``numpy.ndarray``
            ``list`` that stores each policy as a 2D array in ``policies[p_idx]``. Shape of ``policies[p_idx]`` 
            is ``(num_timesteps, num_factors)`` where ``num_timesteps`` is the temporal
            depth of the policy and ``num_factors`` is the number of control factors.

        Returns
        ----------
        
        p_actions: ``numpy.ndarray`` of dtype object
            Marginal posteriors over actions, after softmaxing and scaling with action precision. This distribution will be used to sample actions,
            if``action_selection`` argument is "stochastic"
        """
    
        policy_length = len(policies[0]) #assuming all same lengths
        # print('agent lookahead sample action_test', policy_length)
        max_motion = [0,0]
        step_dist = self.get_pose_dist()
        for angle_deg in self.possible_directions.keys():
            if angle_deg == 'STAY':
                continue
            motion = from_degree_to_point(float(angle_deg))
            if sum(motion) > sum(max_motion):
                max_motion = motion

        furthest_distance = round(max(np.array(max_motion)*policy_length*step_dist))
        #print('furthest_distance', furthest_distance)
        action_marginals = np.zeros(((furthest_distance)*2+1, (furthest_distance)*2+1))
        for pol_idx, policy in enumerate(policies):
            pose = [furthest_distance, furthest_distance,0]

            if self.action is None:
                prev_action = 0
            else:
                prev_action = self.action[0]
            for action_idx, action in enumerate(policy):

                seq_actions = self.PoseMemory.get_sequential_actions(float(prev_action), float(action), self.possible_directions, pose)
                prev_action = action
                pose = seq_actions[-1]
                action_marginals[int(pose[0])][int(pose[1])] += q_pi[pol_idx]

        
        action_marginals = action_marginals * 15
        return action_marginals



