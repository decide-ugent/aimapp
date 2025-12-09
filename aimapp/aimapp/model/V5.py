from aimapp.model.MCTS import MCTS
from aimapp.model.pymdp.agent import Agent
from aimapp.model.odometry import PoseOdometry
from aimapp.model.pymdp import utils
from aimapp.model.pymdp.control import get_expected_obs, get_expected_states, calc_states_info_gain, calc_expected_utility, calc_pA_info_gain, calc_pB_info_gain
from aimapp.model.pymdp.maths import softmax, spm_log_single
from aimapp.model.modules import *
from aimapp.model.pymdp.learning import update_obs_likelihood_dirichlet
# from .pymdp.maths import spm_dot


class Ours_V5_RW(Agent):
    #====== NECESSARY TO SETUP MODEL ======#
    def __init__(self, num_obs=2, num_states=2, dim=2, observations=[0,(0,0)], lookahead_policy=4, \
                n_actions= 6, influence_radius:float=0.5, robot_dim:float=0.25, \
                lookahead_node_creation=3) -> None:
        
        #MCTS PARAMETERS
        self.num_simulations = 30  # Number of MCTS simulations per planning step
        self.lookahead_policy = lookahead_policy # Maximum depth for the simulation (rollout) phase
        self.c_param = 5
            
        self.use_inductive_inference = False
        self.agent_state_mapping = {} #testing purposes
        self.influence_radius = influence_radius
        self.robot_dim = robot_dim 
        self.possible_actions = self.generate_actions(n_actions) 
        self.PoseMemory = PoseOdometry(self.possible_actions, influence_radius, robot_dim)

        self.preferred_ob = [-1,-1]

        self.lookahead_node_creation = lookahead_node_creation
        observations, agent_params = self.create_agent_params(num_obs=num_obs, num_states=num_states, observations=observations, \
                            dim=dim)
        super().__init__(**agent_params)
        self.initialisation(observations=observations)
    
    def create_agent_params(self,num_obs:int=2, num_states:int=2, observations:list=[0,(0,0)], 
                dim:int=2):
        ob = observations[0]
        p_idx = -1
        if dim > 1:
            #start pose in map
            if len(observations) < 2:
                observations.append([0.0,0.0])
            self.PoseMemory.reset_odom(observations[1])
            p_idx = self.PoseMemory.pose_to_id()
            observations[1] = p_idx
            
        else:
            p_idx = self.PoseMemory.pose_to_id()
        
        self.current_pose = self.PoseMemory.get_odom(as_tuple=True)
        #INITIALISE AGENT PARAMS
        B_agent = create_B_matrix(num_states,len(self.possible_actions))
        if 'STAY' in self.possible_actions and self.set_stationary_B:
            B_agent = set_stationary(B_agent,self.possible_actions['STAY'])
        pB = utils.to_obj_array(B_agent)

        obs_dim = [np.max([num_obs, ob + 1])] + ([np.max([num_obs, p_idx + 1])] if dim > 1 else [])
        A_agent = create_A_matrix(obs_dim,[num_states]*dim,dim)
        pA = utils.dirichlet_like(A_agent, scale = 1)

        return observations, {
            'A': A_agent,
            'B': B_agent,
            'pA': pA,
            'pB': pB,
            'lr_pA': 5,
            'save_belief_hist': True,
            'action_selection': "stochastic", 
            'use_param_info_gain': False
        }

    def initialisation(self,observations:list=[0,[0,0]]):
        """
        Initialises the agent with the first observation and ensures all parameters 
        are suitable for continuous navigation.

        Parameters:
            observations (list, optional): Initial observation. Defaults to [0, [0, 0]].
            linear_policies (bool, optional): 
                - If **False**: Explores all combinations of actions (exponential complexity: `n_action^policy_len` with `policy_len == lookahead`).
                - If **True**: Generates a linear path reaching a **lookahead distance** or **num steps**.
                - The path remains linear if no "STAY" actions are included.
                - If "STAY" actions exist, the path follows a polynomial pattern.
                - "STAY" actions are irregular and appear only at the end of a policy.
            E (optional): Additional environment-specific parameters (default: None).

        Note:
            - `linear_policies=True` is optimized for cases where `num_factor == 1` 
            and `len(num_control) == 1`.

        Returns:
            None
        """
      
        self.reset(start_pose=self.PoseMemory.get_poses_from_memory()[0])
        if self.edge_handling_params["use_BMA"] and hasattr(self, "q_pi_hist"): #This is not compatible with our way of moving
            del self.q_pi_hist
            
        self.inference_params = {'num_iter': 3, 'dF': 1.0, 'dF_tol': 0.001}
        #Not necessary, but cleaner
        for i in range(len(self.A)):
            self.A[i][:,:] = 0.01 #reset A for cleaner plot and more fair state inference
        self.update_A_with_data(observations,0)
        self.update_agent_state_mapping(self.current_pose, observations, 0)
        self.infer_states(observation = observations, partial_ob=None)
        if 'STAY' in self.possible_actions.values():
            stay_action = [key for key, value in self.possible_actions.items() if value == 'STAY'][0]
            self.B[0] = set_stationary(self.B[0], stay_action)
        return 

    def reset(self, init_qs:np.ndarray=None, start_pose:tuple=None):
        """
        Resets the agent's posterior beliefs about hidden states to a uniform distribution 
        and resets the simulation time to the initial timestep.

        This function initializes or resets key agent parameters, including past actions, 
        observations, and beliefs, ensuring proper inference and navigation behavior.

        Parameters
        ----------
        init_qs : numpy.ndarray, optional
            A predefined posterior over hidden states. If provided, the agent's beliefs 
            will be initialized using `init_qs` instead of a uniform prior.
        
        start_pose : tuple, optional
            The initial position (pose) of the agent. If provided, it sets `self.current_pose`.

        Returns
        -------
        qs : numpy.ndarray
            The initialized posterior over hidden states. The structure of `qs` depends on 
            the inference algorithm selected:

            - If `self.inference_algo == 'VANILLA'`:  
            `qs` is a simple uniform distribution over hidden states.

        Notes
        -----
        - If `self.edge_handling_params['policy_sep_prior']` is enabled, 
        the latest beliefs are initialized separately for each policy.
        - If `init_qs` is provided, it is directly assigned to `self.qs`, 
        bypassing uniform initialization.
        """

        self.curr_timestep = 0
        self.action = None
        self.prev_actions = None
        self.prev_obs = []
        self.qs_step = 0
     
        self.current_pose = start_pose
        self.PoseMemory.reset_odom(start_pose)
        if init_qs is None:
            
            self.D = self._construct_D_prior()
           
            if hasattr(self, "q_pi_hist"):
                self.q_pi_hist = []

            if hasattr(self, "qs_hist"):
                self.qs_hist = []
            
            if self.inference_algo == 'VANILLA':
                self.qs = utils.obj_array_uniform(self.num_states)
            else:
                print('MMP INFERENCE NOT IMPLEMENTED')
        
        else:
            self.qs = init_qs

        return self.qs
    
    def generate_actions(self,n_actions:int)->dict:
        """
        Divides the 360-degree orientation into discrete action zones and 
        returns a dictionary mapping each action to its corresponding range.

        Parameters:
            n_actions (int): The number of discrete actions to divide the 
                            360-degree space into.

        Returns:
            dict: A dictionary where keys are action indices (int), and values 
                are lists containing the start and end zone (in degrees):
                `{action_index: [start_zone, end_zone]}`.

        Note:
            - The action zones are evenly spaced across 360 degrees.
            - The function include a "STAY" action.
            - The start and end values are rounded to the nearest integer.
        """
        stay = False
        if n_actions% 2 != 0:
            n_actions = n_actions-1
            stay = True
        zone_range_deg = round(360/n_actions,1)
        n_actions_keys = np.arange(0, n_actions, 1)
        zone_spacing_deg = np.arange(0, 361, zone_range_deg)
        possible_actions = {}
        for action_key in n_actions_keys:
            possible_actions[action_key] = [round(zone_spacing_deg[action_key]), round(zone_spacing_deg[action_key+1]),]
        if stay:
            possible_actions[len(possible_actions)] = "STAY"

        return possible_actions

    #==== TEST&VISU PURPOSES ONLY ====#
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
    
    #==== SET METHODS ====#
    def explo_oriented_navigation(self):
        self.use_param_info_gain = False #if true, do not use with the other terms
        self.use_states_info_gain = True 
        self.use_utility = False
        self.use_inductive_inference = False

    def goal_oriented_navigation(self, obs=None, **kwargs):
        pref_weight = kwargs.get('pref_weight', 1.0)
        self.update_preference(obs, pref_weight)
        self.use_param_info_gain = False #if true, do not use with the other terms
        self.use_states_info_gain = False #This make it FULLY Goal oriented
        #NOTE: if we want it to prefere this C but still explore a bit once certain about state 
        #(keep exploration/exploitation balanced) keep info gain
        self.use_utility = True
        self.use_inductive_inference = True

    def set_action_step(self, action):
        ''' only to do if we don't nfer action'''
        self.action = np.array([action])
        self.step_time()
    
    def set_current_pose(self, pose_id):
        self.current_pose = self.PoseMemory.id_to_pose(pose_id)
        self.PoseMemory.reset_odom(self.current_pose)


    #==== GET METHODS
    def get_current_pose_id(self):
        ''' we do not want a negative pose od'''
        if self.current_pose is None:
            current_pose = self.PoseMemory.get_odom()[:2]
        else:
            current_pose = self.current_pose
        return self.PoseMemory.pose_to_id(current_pose)
    
    def get_belief_over_states(self):
        """
        get the belief distribution over states

        Returns:
            np.ndarray: The extracted belief distribution over states.
        """
        return self.qs
    
    def get_current_timestep(self):
        return self.curr_timestep
    def get_possible_actions(self):
        return self.possible_actions
    def set_memory_views(self, views):
        self.ViewMemory = views
    def get_memory_views(self):
        return self.ViewMemory
    
    def get_n_states(self):
        return len(self.agent_state_mapping)
    
    def get_agent_state_mapping(self)->dict:
        return self.agent_state_mapping
    
    def get_B(self):
        return self.B[0]
    
    def get_A(self):
        return self.A
    def get_current_most_likely_pose(self, z_score:float, min_z_score:float=2, qs=None,  observations:list=[])->int:
        """
        Given a z_scores (usually around 2), is the agent certain about the state. If it is, to which pose does it correspond?
        Return pose -1 if < threhsold, else return pose id.
        If no state stands out at all, we don't know where we are and return -2
        """
        if qs is None:
            qs = self.get_belief_over_states()[0]
        p_idx = -1
        mean = np.mean(qs)
        std_dev = np.std(qs)
        #print('qs mean and std_dev', mean, std_dev)
        # Calculate Z-scores
        z_scores = (qs - mean) / std_dev
        # Get indices of values with Z-score above 2
        outlier_indices = np.where(np.abs(z_scores) > z_score)[0]
        min_outlier_indices = np.where(np.abs(z_scores) > min_z_score)[0]
        
        
        #print("Indices of outliers (Z-score >",z_score,"):" , outlier_indices)
        #If we are sure of a state (independent of number of states), we don't have pose as ob and A allows for pose
        if len(outlier_indices) >= 0 and len(observations) < 2 and len(self.A) > 1:
            #If 1 state stands out
            if len(outlier_indices) == 1:
                p_idx = outlier_indices[0]
        #If min_z_scores length is 0, it means no proba is standing out! We don't know where we are   
        elif len(min_outlier_indices) == 0 and len(observations) < 2 and len(self.A) > 1:
            p_idx = -2
        return p_idx

    def get_observation_most_likely_states(self, observations: list, per_threshold: float = 0.5) -> list:
        """
        standout detector using desired min percentage + fallback for sparse/multi-peak distributions in case (I think it should be removed)
        """
        likely_states = {}

        for modality, ob in enumerate(observations):
            if ob < 0:
                    continue
            standout_indices = []
            qo = np.array(self.get_A()[modality][ob])
            # print('qo', qo.round(3))
            # threshold for standout values
            standout_indices = np.where(qo >= per_threshold)[0]

            # Special case: only one clear maximum, much larger than rest
            max_val = np.max(qo)
            second_max = np.partition(qo.flatten(), -2)[-2]
            # print('max_val', max_val,'second_max',second_max)
            if max_val < 4 * second_max and second_max not in standout_indices:
                np.append(standout_indices,second_max)
            # print('standout_indices',standout_indices)
            for idx in standout_indices:
                likely_states[idx] = likely_states.get(idx, 0) + 1

        if not likely_states:
            return []
        # Return most recurrent standout indices across modalities
        most_recurrent = max(likely_states.values())
        standout_final = [state for state, count in likely_states.items() if count == most_recurrent]

        return standout_final
    
    def get_expected_observation(self, qs=np.ndarray, A:np.ndarray=None)-> np.ndarray:
        """ get observation belief for state qs"""
        if A is None:
            A = self.A
        qo_pi = get_expected_obs(qs, A)
        return qo_pi

    def get_next_state_given_action(self, qs= np.array, action=int, B=None)->np.ndarray:
        ''' expect only 1 qs, return only 1 qs with the same shape (1,) == np.ndarray([np.ndarray([])])'''
        if B is None:
            B = self.B

        # print('B check', B[0][:,np.argmax(qs), action])
        if isinstance(action, (int,np.int64)):
            action = np.array([[action]])
            
        qs_pi = get_expected_states(qs, B, action)
        return qs_pi
    
    
    #==== MCTS_CALL ====#
    def define_actions_from_MCTS_run(self,num_steps=1, logging=None,  **kwargs)->list: #,dict
        """ 
        MCTS RUN, UNDER TEST (SHOULD NOT BE RE-CREATED EACH RUN)
        TODO: adapt for when we want a full policy

        """
        
        mcts = MCTS(self, self.c_param, self.num_simulations, self.lookahead_policy)

        observations = kwargs.get('observations', None)
        next_possible_actions = kwargs.get('next_possible_actions', None)
        plot_MCTS_tree = kwargs.get('plot_MCTS_tree', False)
        save_action_memory = kwargs.get('save_action_memory', True)

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
            
            self.infer_states(observation = observations, partial_ob=partial_ob, save_hist=True)

        initial_pose_id = self.get_current_pose_id()
        initial_belief_qs = self.get_belief_over_states() # Get initial belief
        initial_observation = self.get_expected_observation(initial_belief_qs)

        best_actions, data = mcts.start_mcts(state_qs=initial_belief_qs,
                     pose_id=initial_pose_id, observation=initial_observation, \
                     next_possible_actions=next_possible_actions, num_steps=num_steps, logging=logging, plot=plot_MCTS_tree)
        
        #NOTE: THIS CONSIDERONLY FIRST ACTION OF POLICY. MIGHT LEADS TO ISSUE DEPENDING ON HOW WE USE THAT
        self.q_pi = data['qpi'][0]
        self.G = data['efe'][0]
        if save_action_memory:
        #NOTE: THIS CONSIDER THAT WE APPLY FIRST ACTION OF POLICY. MIGHT LEADS TO ISSUE DEPENDING ON HOW WE USE THAT
            self.action = np.array([best_actions[0]])
            self.step_time()
            
        return best_actions[:num_steps], data
    
    #==== INFERENCE ====#

    def infer_states(self, observation:list, action:np.ndarray= None ,save_hist:bool=True, partial_ob:int=None, qs:list=None):
        """
        Performs variational inference to update posterior beliefs over hidden states given an observation.

        This method updates the agent's belief state (`qs`) by incorporating new observations 
        and optionally considering the previous action. The update process depends on the 
        selected inference algorithm (`VANILLA`).

        Parameters
        ----------
        observation : list or tuple 
            The observed state indices for each observation modality.
    
        action : np.ndarray, optional
            The most recent action taken by the agent. If provided, it helps refine posterior beliefs.

        save_hist : bool, default=True
            If True, stores the latest observation and updates historical data for future inference.


        partial_ob : int, optional
            Specifies a particular observation modality to update the belief state for, rather than all modalities.

        qs : list, optional
            A predefined posterior belief state. If provided, this will be used instead of computing from scratch.

        Returns
        -------
        qs : numpy.ndarray of dtype object
            Updated posterior beliefs over hidden states. The structure depends on the inference algorithm:
            - For `VANILLA`, `qs` represents a single posterior belief over hidden states.


        Notes
        -----
        - If `self.inference_algo == "VANILLA"`, posterior updates consider an empirical prior derived from 
        the transition model (`B`) or from a uniform prior (`D`).
        - The method also updates `self.qs_hist` and `self.qs_step` when `save_hist=True`, 
        enabling tracking of belief evolution over time.
        """

        if save_hist:
            self.prev_obs.append(observation)
            observations_hist = self.prev_obs
        else:
            observations_hist = self.prev_obs.copy()
            observations_hist.append(observation)

        if action == None:
            action = self.action

        if self.inference_algo == "VANILLA":
            if action is not None:
                if qs is None:
                    ref_qs = self.get_belief_over_states() #we don't yet want to consider current obs to selest qs
                else: #safety to avoid any risk
                    ref_qs = qs[:]
                empirical_prior = control.get_expected_states(
                    ref_qs, self.B, action.reshape(1, -1) #type: ignore
                )[0]
            #unused, but kept as a memor
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
                        return qs
            qs = update_posterior_states(
            self.A,
            observation,
            empirical_prior,
            partial_ob,
            **self.inference_params
            )
            F = 0
            qs_step = 0

        if save_hist:
            self.F = F # variational free energy of each policy  
            self.qs_step = qs_step
            if hasattr(self, "qs_hist"):
                self.qs_hist.append(qs)
            self.qs = qs

        return qs
    
    def infer_pose(self, pose)->list:
        '''
        Parameters:
            pose (int or list): The index of the pose or the pose itself
        Here we consider that action consider the actual action sensed by agent (thus consider no motion)
        and we consider PoseMemory adapted to treat that perception
        '''
        if isinstance(pose,int):
            pose = self.PoseMemory.id_to_pose(pose)
        self.PoseMemory.update_odom_given_pose(pose)
        if self.current_pose !=None:
            self.current_pose = self.PoseMemory.get_odom(as_tuple=True)[:2]
        return self.current_pose

    def infer_action(self, logs=None, **kwargs):
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
        next_possible_actions = kwargs.get('next_possible_actions', list(self.possible_actions.keys()))
        if logs is not None:
            logs.info('observations'+ str(observations)+ 'next_possible_actions'+ str(next_possible_actions))
        # prior = self.get_belief_over_states()
        
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
            
            posterior = self.infer_states(observation = observations, partial_ob=partial_ob, save_hist=True)
            # print('infer action: self.current_pose', self.current_pose, posterior[0].round(3))
        if logs is not None:
            logs.info('still there')
        #In case we don't have observations.
        posterior = self.get_belief_over_states()
        #print('infer action: inferred prior state', posterior[0].round(3))
        q_pi, efe, info_gain, utility = self.infer_policies(posterior, logs=logs)
        if logs is not None:
            logs.info('catching up here')
        poses_efe = None
        action = self.sample_action(q_pi, next_possible_actions)

        if logs is not None:
            logs.info('no problem here')
        #TODO: switch back to sample_action once tests done
        # action, poses_efe = self.sample_action_test(next_possible_actions)
        
        #What we would expect given prev prior and B transition 
        # prior = spm_dot(self.B[0][:, :, int(action)], prior)
        
        self.q_pi = q_pi
        self.G = efe

        data = { "qs": posterior[0],
            "qpi": q_pi,
            "efe": efe,
            "info_gain": info_gain,
            "utility": utility,
            #"bayesian_surprise": utils.bayesian_surprise(posterior[0].copy(), prior),
            }
        if poses_efe is not None:
            data['poses_efe'] = poses_efe
        return int(action), data
    
    def infer_policies(self, qs=None, logs=None):
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
            diff_policy_len = False, #TODO: erase in a refactoring
            logs= logs
        )
        if hasattr(self, "q_pi_hist"):
            self.q_pi_hist.append(q_pi)
            if len(self.q_pi_hist) > self.inference_horizon:
                self.q_pi_hist = self.q_pi_hist[-(self.inference_horizon-1):]
            
        return q_pi, G, info_gain, utility
    
    def sample_action(self, q_pi:np.ndarray, possible_first_actions:list=None)->int:
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
            policies, q_pi = zip(*[(policy, q_pi[p_id]) for p_id, policy \
                                   in enumerate(self.policies) if policy[0] in possible_first_actions])
        else:
            policies =  self.policies

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
    
    def infer_current_most_likely_pose(self, observations:list, z_score:float=2, min_z_score:float=2):
        ''' define our position p '''
        
        # if self.current_pose is None:
        #     z_score = 2
        
        p_idx = self.get_current_most_likely_pose(z_score, min_z_score, observations = observations)
        #if we have a pose, replace current inferred pose by the most likely one.
        if p_idx >= 0:
            self.current_pose = self.PoseMemory.id_to_pose(p_idx)
            self.PoseMemory.reset_odom(self.current_pose)
            #print('updating believed pose given certitude on state:', self.current_pose)
        elif p_idx < -1:
            self.current_pose = None
        return p_idx
    
    def infer_best_action_given_actions(self, G:list, actions:list, action_selection:str=None, alpha:float=None):
        if isinstance(actions[0], (int, np.int64)):
            actions = np.array([[[a]] for a in actions])

        if action_selection is None:
            action_selection = self.action_selection
        if alpha is None:
            alpha = self.alpha
        G = np.array(G)
        lnE = spm_log_single(np.ones(G.shape) / len(G))

        q_pi = softmax(G * self.gamma + lnE) 
        if self.sampling_mode == "marginal":
            best_action = control.sample_action(q_pi, policies = actions, num_controls = self.num_controls, action_selection = action_selection, alpha= alpha)
        elif self.sampling_mode == "full":
            best_action = control.sample_policy(q_pi, actions, self.num_controls,
                                           action_selection=action_selection, alpha=alpha)
        return q_pi, int(best_action[0])
    
    def infer_utility_term(self, qo_pi:np.ndarray, C=None)->float:
        """ given the observation belief of a state, what is the utility term"""
        if C is None:
            C = self.C
        return calc_expected_utility(qo_pi, C)
    
    def infer_info_gain_term(self, qs_pi:np.ndarray, A=None)->float:
        """ given the belief of a state, what is the info gain term (Note the method expects several qs, thus qs must be in 3 layers of np.ndarray)"""
        if A is None:
            A = self.A
        return calc_states_info_gain(A, qs_pi)
    
    def infer_param_info_gain(self, qs_pi:np.ndarray, qo_pi:np.ndarray, qs:np.ndarray, action:int):
        """Infer param info gain for an action (but can also be a policy)"""
        G = 0
        if self.pA is not None:
            param_info_gain = calc_pA_info_gain(self.pA, qo_pi, qs_pi)
            G +=  param_info_gain
        if self.pB is not None:
            if isinstance(action, (int,np.int64)):
                action = np.array([[action]])
            param_info_gain = calc_pB_info_gain(self.pB, qs_pi, qs, action)
            G +=  param_info_gain

        return G
    
    def infer_inductive_preference(self, current_qs:np.ndarray, qs_pi:np.ndarray, C=None, logging=None)->float:
        """ given the observation belief of a state, what is the utility term of the state, 
        we propagate interest in states leading toward the goal. Based on the paper "Active Inference and Intentional Behaviour"
        NOTE: we can only see the goal if it is under self.policy_len from our position (else 0 influence)
        NOTE: THIS WORKS FOR VANILLA MODEL ONLY (NOT MMP) AS WE CONSIDER QS to have only 1 step 
        """
        if C is None:
            C = self.C
        current_qs = current_qs[0]
        qs_arg_max = np.argmax(current_qs)  #NOTE: not sure this is ideal...
        model_B = self.get_B()[:,qs_arg_max,:]
        median = np.median(model_B)
        B = model_B[model_B > median] 
        certitude_threshold = max(np.mean(B), 0.15)
        I = [copy.copy(self.Cs)]
        #Keep only certain Transitions
        B_certain_trans = (self.get_B() > certitude_threshold).astype(float)

        found_path = False
        # print('START with preferred state', np.argmax(I), I)
        # print('we are in starting state', qs_arg_max, 'prob',current_qs[qs_arg_max])
        # print('qs_pi',qs_pi)
        #from preferred states, which states lead to it then we repeat until we are in qs
        for n in range(self.lookahead_policy):
            #TODO: ADD WHEN WE STOP FOR LOOP (WHEN WE ARE ON CURRENT STATE)
            I_next = ((B_certain_trans.T.dot(I[-1])) > 0).astype(float) # New reachable states (as bool -> float)
            I_next = np.max(I_next, axis=0) #We consider all states regardless of action
            # print('backward step', len(I)-1)
            I.append(I_next)
            #logging.info(f'States to inflate H {np.argwhere(I[n] > np.amin((I[n] >0).astype(float))).flatten()}')
            if I[-1][qs_arg_max] >= current_qs[qs_arg_max]:
                # print('we end process induction in ',n+1,' steps,current state', qs_arg_max)
                n-=1
                found_path = True
                break
        n+=1 #to consider that I[0] is goal
        if found_path:
            if logging is not None:
                logging.info(f'Final States to inflate H {np.argwhere(I[n] > np.amin((I[n] >0).astype(float))).flatten()}')
            H = np.log(certitude_threshold)*I[n].dot(qs_pi[0])
        else:
            H = 0.0

        return H
    
    #==== OTHER METHODS ====#

    def determine_next_pose(self, action_id:int, pose:list=None,  min_dist_to_next_node:float=None):
        next_pose = self.PoseMemory.pose_transition_from_action(action =action_id, odom= pose, ideal_dist=min_dist_to_next_node)
        next_pose = [round(elem, 2) for elem in next_pose]
        next_pose_id = self.PoseMemory.pose_to_id(next_pose, save_in_memory=False)
        #print('action, next pose and id', action_id, next_pose, next_pose_id)
        return next_pose, next_pose_id

    def determine_action_given_angle_deg(self, angle):
        """
        Find the key in possible_actions corresponding to the given angle.

        Args:
            angle (float): The angle to check in DEGREES

        Returns:
            int: The corresponding action key, or None if no match is found.
        """
        actions = self.possible_actions.copy()
        if "STAY" in actions.values():
            actions.popitem()
        action_key = [k for k,v in actions.items() if v[0] <= angle and v[1] > angle]
        return action_key[0]
        
        #same thing
        # for key, value in self.possible_actions.items():
        #     if value == "STAY":
        #         continue
        #     if value[0] <= angle < value[1]:  # Check if angle falls within range
        #         return key
        # return None


    def calculate_min_dist_to_next_node(self, state_step:int=1):
        return self.influence_radius * state_step + self.robot_dim/2#/3 to consider -a little- robot_dim when adding nodes.as_integer_ratio
    
    def define_next_possible_actions(self, obstacle_dist_per_actions:list, restrictive:bool=False, logs=None):
        min_dist = self.calculate_min_dist_to_next_node()
        
        n_actions = len(self.possible_actions) - ("STAY" in self.possible_actions.values())
        possible_actions = [i for i in range(n_actions) if obstacle_dist_per_actions[i] >= min_dist]
        if restrictive:
            possible_actions_2 = possible_actions[:]
            for action in possible_actions_2:
                next_pose, next_pose_id = self.determine_next_pose(action, min_dist_to_next_node=min_dist)
                registered_pose = self.PoseMemory.id_to_pose(next_pose_id)
                if logs:
                    logs.info(f'next pose{next_pose}{next_pose_id}, with action{action}, but registered_pose{registered_pose}')
                if registered_pose[0] != next_pose[0] or registered_pose[1] != next_pose[1] :
                    possible_actions.remove(action)
                    
        if "STAY" in self.possible_actions.values():
            possible_actions.append(n_actions)

        return possible_actions

    #==== Update A, B, C ====#
    def update_A_with_data(self,obs:list, state:int)->np.ndarray:
        """Given obs and state, update A entry """
        A = self.A
        
        for dim in range(self.num_modalities ):
            A[dim][:,state] = 0
            A[dim][obs[dim],state] = 1
        self.A = A
        return A
    
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
    
    def update_B(self,qs:np.ndarray, qs_prev:np.ndarray, action:int, lr_pB:int=None)-> np.ndarray:
        """
        Updates the posterior Dirichlet parameters (`pB`) that parameterize the transition likelihood (`B`).

        This function refines the transition model by incorporating new posterior beliefs about states (`qs`),
        previous state beliefs (`qs_prev`), and the most recent action taken. The update is performed using 
        a Dirichlet-multinomial approach, ensuring a smooth adaptation of transition probabilities.

        Parameters
        ----------
        qs : numpy.ndarray
            Marginal posterior beliefs over hidden states at the current time step.

        qs_prev : numpy.ndarray
            Marginal posterior beliefs over hidden states at the previous time step.

        action : int
            The most recent action taken by the agent, which affects transition updates.

        lr_pB : int, optional
            Learning rate for updating `pB`. If not specified, defaults to `self.lr_pB`.

        Returns
        -------
        qB : numpy.ndarray
            Updated posterior Dirichlet parameters over transition probabilities (`B`). 
            This has the same shape as `B` but now incorporates learned state-action transitions.

        Notes
        -----
        - The update is computed using the `update_state_likelihood_dirichlet` function, 
        which adjusts `pB` based on the observed transitions.
        - The function ensures that `qB` does not contain negative values by applying a failsafe correction.
        - Transition probabilities (`B`) are normalized after updating `pB` to maintain a valid probability distribution.
        - If `lr_pB` is negative, a failsafe mechanism prevents `qB` from dropping below a minimum threshold (0.005).
        - The updated `qB` is stored as `self.pB`, and `B` is re-normalized for future inference.

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

    def update_A_dim_given_obs(self, obs:list,null_proba:list=[True]) -> np.ndarray:
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
    
    def update_A_dim_given_pose(self, pose_idx:int,null_proba:bool=True) -> np.ndarray:
        ''' 
        Verify if the observations are new and fit into the current A shape.
        If not, increase A shape and associate those observations with the newest state generated.
        If yes, search for the first empty column available and fill it with new inferred position (pose_idx)
        '''
        A = self.A
        num_obs, num_states, num_modalities, num_factors = utils.get_model_dimensions(A=A)
        if pose_idx >= max(num_states):
            # Calculate the maximum dimension increase needed across all modalities
            dim_add = int(max(0,pose_idx + 1 - num_obs[num_modalities-1]))
            # Update matrices size
            #and associate new observations with the newest state generated
            if dim_add > 0:
                A[0] = update_A_matrix_size(A[0], add_state=dim_add, null_proba=null_proba)
                if num_modalities > 1:
                    A[1] = update_A_matrix_size(A[1], add_ob=dim_add, add_state=dim_add, null_proba=null_proba)
                    self.num_obs[1] = A[1].shape[0]
        if num_modalities > 1:
            #columns_wthout_data = np.sort(np.append(np.where(np.all(A[1] == 1/A[1].shape[0], axis=0))[0], np.where(np.all(A[1] == 0, axis=0))[0]))
            A[1][:, pose_idx] = 0
            A[1][pose_idx, pose_idx] = 1
            

        if self.pA is not None:
            self.pA = utils.dirichlet_like(utils.to_obj_array(A), scale=1)
                        
        self.num_states = [A[0].shape[1]]
        self.A = A
        return A
    
    def update_B_dim_given_A(self)-> np.ndarray:
        """ knowing A dimension, update B state dimension to match"""
        B = self.B
        add_dim = self.A[0].shape[1]-B[0].shape[1]
        if add_dim > 0: 
            #increase B dim
            B = update_B_matrix_size(B, add= add_dim)
            self.pB = update_B_matrix_size(self.pB, add= add_dim, alter_weights=True)
            if len(self.qs) > 1:
                for seq in self.qs:
                    for subseq in seq:
                        subseq[0] = np.append(subseq[0], [0] * add_dim)
            else:
            
                self.qs[0] = np.append(self.qs[0],[0]*add_dim)
        
        self.num_states = [B[0].shape[0]]
        self.B = B
        return B
    
    def update_believes_with_obs(self, Qs:list, action:int, obs:list)-> None:
        """
        Updates the model's transition (`B`) and observation (`A`) matrices using the given 
        posterior beliefs over states (`Qs`), action taken, and new observation.

        Parameters
        ----------
        Qs : list
            The updated posterior beliefs over hidden states.

        action : int
            The action taken at the current step.

        obs : list
            The observed sensory input at the current step.

        Returns
        -------
        None
            Updates the transition (`B`) and observation (`A`) matrices in-place.

        Notes
        -----
        - If `qs_hist` is available, it retrieves the previous belief state and ensures consistency 
        in dimensionality before updating `B` using `Qs`.
        - If the transition resulted in a change of state, the function also updates `B` for 
        the reverse transition using the inverse action.
        - After updating `B`, the function re-infers the new state (`Qs`) based on `obs` and 
        updates `A` accordingly.
        """
        if len(self.qs_hist) > 0:#secutity check
            qs_hist = self.qs_hist[-1]
            qs_hist[0] = np.append(qs_hist[0],[0]*\
                                   (len(Qs[0])-len(qs_hist[0])))
            self.update_B(Qs, qs_hist, action, lr_pB = 10) 
            #2 WAYS TRANSITION UPDATE (only if T to diff state)
            if np.argmax(qs_hist[0]) != np.argmax(Qs[0]):
                a_inv = reverse_action(self.possible_actions, action)
                self.update_B(qs_hist, Qs, a_inv, lr_pB = 7)
        Qs = self.infer_states(obs) 
        self.update_A(obs, Qs)

    def update_B_given_unreachable_pose(self,next_pose:list, action:int)-> None:
        """ We reduce transition probability between those 2 states that do not connect"""
        if self.current_pose is not None and next_pose in self.PoseMemory.get_poses_from_memory() :
            n_pose_id = self.PoseMemory.pose_to_id(next_pose)
            qs = self.get_belief_over_states()
            hypo_qs = self.infer_states([n_pose_id], np.array([action]), partial_ob=1, save_hist=False)

            # print(self.B[0][np.argmax(hypo_qs[0])][np.argmax(qs[0])][action], self.B[0][np.argmax(qs[0])][np.argmax(hypo_qs[0])][action])
            # print(self.B[0][np.argmax(qs[0])][np.argmax(hypo_qs[0])][action], self.B[0][np.argmax(hypo_qs[0])][np.argmax(qs[0])][action])
            self.update_B(hypo_qs, qs,action,lr_pB=-10)
            a_inv = reverse_action(self.possible_actions, action)
            self.update_B(qs,hypo_qs,a_inv,lr_pB=-7)

    def update_qs_dim(self, qs:np.array=None, update_qs_memory:bool=True)->np.ndarray:
        if qs is None:
            qs = self.qs[:]
        if len(qs[0]) < self.B[0].shape[0]:
            qs[0] = np.append(qs[0],[0]*(self.B[0].shape[0]-len(qs[0])))

        if update_qs_memory:       
            self.qs = qs
            for p_step, past_qs in enumerate(self.qs_hist):
                if len(past_qs[0]) < self.B[0].shape[0]:
                    past_qs[0] = np.append(past_qs[0],[0]*(self.B[0].shape[0]-len(past_qs[0])))
                    self.qs_hist[p_step]= past_qs
        return qs
    
    def update_C_dim(self):
        if self.C is not None:
            preferences = []
            for m in range(self.num_modalities):
                if self.C[m].shape[0] < self.num_obs[m]:
                    self.C[m] = np.append(self.C[m], [0]*(self.num_obs[m]- self.C[m].shape[0]))
                    preference = np.argwhere(self.C[m] > np.amin((self.C[m]>0).astype(float))).flatten()
                    if len(preference) > 0:
                        preferences.extend(list(preference)) 
                    else:
                        preferences.append(-1)
            if len(preferences)>0: #We check if those preferences fit new states
                desired_states = self.get_observation_most_likely_states(observations=preferences, per_threshold=0.45)
                self.Cs = np.zeros(self.num_states)
                for state in desired_states:
                    self.Cs[state] = 1.0
    
    def update_preference(self, obs:list=[-1,-1], pref_weight:float=1.0):
        """given a list of observations (must fill all expected observation. If no preference enters -1) we fill C with thos as preference. 
        If we have a partial preference over several observations, 
        then the given observation should be an integer < 0, the preference will be a null array 
        """
        if isinstance(obs, list):
            self.update_A_dim_given_obs(obs, null_proba=[False]*len(obs))

            C = self._construct_C_prior()
            Cs = np.zeros(self.num_states)

            for modality, ob in enumerate(obs):
                if ob >= 0:
                    self.preferred_ob[modality] = ob
                    ob_processed = utils.process_observation(ob, 1, [self.num_obs[modality]])
                    ob = utils.to_obj_array(ob_processed)
                else:
                    ob = utils.obj_array_zeros([self.num_obs[modality]])
                C[modality] = np.array(ob[0])

            if not isinstance(C, np.ndarray):
                raise TypeError(
                    'C vector must be a numpy array'
                )
            C = C * pref_weight
            self.C = utils.to_obj_array(C)


            assert len(self.C) == self.num_modalities, f"Check C vector: number of sub-arrays must be equal to number of observation modalities: {self.num_modalities}"

            for modality, c_m in enumerate(self.C):
                assert c_m.shape[0] == self.num_obs[modality], f"Check C vector: number of rows of C vector for modality {modality} should be equal to {self.num_obs[modality]}"
        else:
            self.preferred_ob = [-1,-1]
            self.C = self._construct_C_prior()

        desired_states = self.get_observation_most_likely_states(observations=obs, per_threshold=0.45)
        for state in desired_states:
            Cs[state] = 1.0
        self.Cs = Cs
        

    #====== UPDATE MODEL ======#
    def update_transitions_both_ways(self,qs:np.ndarray, next_qs:np.ndarray, action_id:int, \
                                     direct_lr_pB:int, reverse_lr_pB:int)-> None:
        
        a_inv = reverse_action(self.possible_actions, action_id)
        self.update_B(next_qs, qs, action_id, lr_pB = direct_lr_pB) 
        self.update_B(qs, next_qs, a_inv, lr_pB = reverse_lr_pB)
   
    def update_imagined_translation(self, qs:np.ndarray, action_jump:int, n_actions:int, action_id:int, cur_pose:list, \
                                    min_dist_to_next_node:int, obstacle_dist_per_actions:int):
        """
        Updates the model's transition probabilities (`B` matrix) from current pose to imagined poses and between imagined poses up to 'action_jump' actions away
        It reinforces direct and indirect motion transitions while considering obstacles. 

        Parameters
        ----------
        qs : np.ndarray
            The posterior belief over states before taking the action.

        action_jump : int
            The range of adjacent actions to consider when updating transitions.

        n_actions : int
            The total number of possible actions.

        action_id : int
            The current action being taken.

        cur_pose : list
            The current position in the environment.

        min_dist_to_next_node : int
            The minimum distance required to move to the next node.

        obstacle_dist_per_actions : int
            The distance of obstacles from the current state for each possible action.

        hypo_qs : list or None
            The hypothetical belief state over hidden states, used when reinforcing indirect motion. 
            If `None`, direct motion updates are applied.

        Returns
        -------
        None
            Updates the transition probabilities (`B` matrix) in-place.

        Notes
        -----

        """
        #1) We get current physical pose info + the next imagined pose info given action_id
        _, next_pose_id = self.determine_next_pose(action_id, cur_pose, min_dist_to_next_node) #from odom to next imagined pose
        next_pose = self.PoseMemory.id_to_pose(next_pose_id) #get memorised pose (not the approximated one)
        cur_pose_id = self.PoseMemory.pose_to_id(cur_pose) #get odom pose id
        next_state_ob_dist = obstacle_dist_per_actions[action_id] #to check if ob between physical pose to next imagined pose
        
        prev_action = -1
        #print('__')
        for offset in range(action_jump, -action_jump - 1, -1):
            action_adjacent = action_id + offset
            #print('action_adjacent, offset', action_adjacent, offset)
            #restraingning action between possible actions numbers
            if action_adjacent < 0 :
                action_adjacent = n_actions +action_adjacent
            else:
                action_adjacent %= (n_actions)
            #from physical pose, get next adjacent pose given offset action 
            next_adjacent_pose, next_adjacent_pose_id = self.determine_next_pose(action_adjacent, cur_pose, min_dist_to_next_node)
            #if no adjacent pose, nothing to do
            #print('next_adjacent_pose_id', next_adjacent_pose_id,'next_pose_id', next_pose_id, 'cur_pose_id',cur_pose_id)
            if next_adjacent_pose_id < 0 or next_adjacent_pose_id==cur_pose_id:
                continue
            #if adjacent pose exists, then we get it's state (obtained from Transitioning from physical state to this adjacent state)
            adjacent_qs = self.infer_states([next_adjacent_pose_id], np.array([action_adjacent]), save_hist=False, partial_ob=1, qs=qs)
            adjacent_state_dist_to_ob = obstacle_dist_per_actions[action_adjacent] #get if ob at this adjacent pose
            #print('offset', offset, 'action_adjacent', action_adjacent, 'adjacent_state_dist_to_ob', round(adjacent_state_dist_to_ob,2))
            #We correct to pose ID pose, to be sure it matches
            next_adjacent_pose = self.PoseMemory.id_to_pose(next_adjacent_pose_id) #get memorised pose (not the approximated one)
            
            #If known state and this is the direct transition from physical to an already existing state 
            if offset == 0 : #if direct motion from current state to another state  
                reference_qs = qs
                action = action_adjacent
                direct_lr_pB = 5
                reverse_lr_pB = 3
                pose_in_action_range = self.PoseMemory.pose_in_action_range(action, next_pose, odom= cur_pose)
                #print('direct transition from new current odom', cur_pose_id, 'to', next_pose_id)
                
            #If this transition is a lateral transition 
            elif offset != 0 and next_adjacent_pose_id != next_pose_id: #if indirect motion, we don't want to reinforce 'stay' motion with wrong action
                reference_qs = self.infer_states([next_pose_id], np.array([action_id]), save_hist=False, partial_ob=1, qs=qs) #next direct imagined pose qs
                angle = angle_turn_from_pose_to_p(pose = next_pose, goal_pose= next_adjacent_pose, in_deg=True)
                action = self.determine_action_given_angle_deg(angle)
                direct_lr_pB = 1
                reverse_lr_pB = 1
                pose_in_action_range = self.PoseMemory.pose_in_action_range(action, next_adjacent_pose, odom= next_pose)
                #Just to avoid reinforcing same link several times (can happens if we check pose to id only considering distance)
                if prev_action == action:
                    #print('already updated that transition')
                    continue
                prev_action = action
                #print('lateral transition from imagined pose',next_pose_id, 'to', next_adjacent_pose_id)
            else:
                continue
            #print('pose_in_action_range', pose_in_action_range, 'action', action,'next_pose', next_adjacent_pose, 'odom', next_pose)
            #If the pose is not in this action range, we don't enforce it + we can't have the poses being unreachable.
            if pose_in_action_range and adjacent_state_dist_to_ob > min_dist_to_next_node and next_state_ob_dist > min_dist_to_next_node :
                # Positive LR
                self.update_transitions_both_ways(reference_qs, adjacent_qs, action, direct_lr_pB=direct_lr_pB, reverse_lr_pB=reverse_lr_pB)
            elif pose_in_action_range:
                #print('negative reinforcement')
                # Negative LR
                self.update_transitions_both_ways(reference_qs, adjacent_qs, action, direct_lr_pB=-direct_lr_pB, reverse_lr_pB=-reverse_lr_pB)

    def update_transition_nodes(self, obstacle_dist_per_actions:list, logs=None)-> None:
        ''' 
        For each new pose observation, add a ghost state and update the estimated transition and observation for that ghost state.
        '''
        #print('Ghost nodes process:')
        action_jump = int((len(self.possible_actions)-1) / 6)
        sure_about_data_until_this_state = 1
        ''' 
            TODO LIST: 
            1) Check if transition possible 
            IF impossible motion:
                2) increase transition from current state to current state for this action (both ways)
                3) if obstacle, check if there is a transition existing for this action and decrease state transition (both ways)
            go to next action
            Else: 
                4) infer new pose in that direction 
                5) check if a pose exist in that direction (margin of zone of influence)
                6) if yes, increase transition prob to existing pose with that action
                7) if no, 
                    7') increase all matrices IF NEED BE
                    7'')create new node 
                8) check if previous and next action have obstacle. If no, link previous/next pose node to current pose node
                9) from this node, check if action still possible further with increased zone of influence + margin (thus until we reach an obstacle) 
            skip next action (as we want 6 nodes around if no obstacle anywhere)
            '''
        
        n_actions = len(self.possible_actions) - ("STAY" in self.possible_actions.values())
        qs = self.get_belief_over_states()
        
        min_dist_to_next_node = self.calculate_min_dist_to_next_node()
        
        for action_id in range(n_actions):
            #print('______________________________')
            hypo_qs = None
            state_step = 1
            prev_step_qs = qs[:]
            no_obstacle = True
            cur_pose = self.PoseMemory.get_odom().copy()
            cur_ref_pose = cur_pose.copy()
            pose_in_action_range = True
            
            #9)
            #The second element is just to avoid any risk of infinity loop
            while no_obstacle and state_step<=self.lookahead_node_creation:
                next_state_min_dist_to_next_node = self.calculate_min_dist_to_next_node(state_step)
                #1)
                #Is obstacle too close?
                #print('for action', action_id, 'obstacle', obstacle_dist_per_actions[action_id], 'min_dist for new state', next_state_min_dist_to_next_node)
                if obstacle_dist_per_actions[action_id] <=  next_state_min_dist_to_next_node :
                    no_obstacle = False 
                    #Only enforce the loop back to current pose if it's a direct motion
                    if state_step <= sure_about_data_until_this_state:
                        #2)
                        #print('enforcing motion:',action_id,' leads to current state')
                        self.update_B(qs, qs, action_id, lr_pB = 10)   
                else:
                    #4)
                    next_pose, next_pose_id = self.determine_next_pose(action_id, cur_ref_pose, min_dist_to_next_node)
                    #print('next_pose', next_pose, self.PoseMemory.get_poses_from_memory().copy())
                    #5) ->7)  
                    if next_pose_id < 0 and pose_in_action_range:
                        next_pose_id = self.PoseMemory.pose_to_id(next_pose) 
                        if logs:
                            logs.info(f'creating new node in position {next_pose}, {next_pose_id}')
                        #print('creating new node in position', next_pose_id)
                        #7')
                        self.update_A_dim_given_pose(next_pose_id,null_proba=True)
                        self.update_B_dim_given_A()
                        self.update_C_dim()
                        prev_step_qs = self.update_qs_dim(prev_step_qs,update_qs_memory=False)
                        #7'')
                        hypo_qs = self.infer_states([next_pose_id], np.array([action_id]), partial_ob=1, save_hist=False, qs=prev_step_qs)
                        self.update_agent_state_mapping(tuple(next_pose[:2]), [-1, next_pose_id], hypo_qs[0])
                        #We don't want lateral state transition updated when we are extrapolating further than "sure_about_data_until_this_state"
                        #plus we only want the action continuing in a straight line from physical current pose. 
                        #becquse the beam rqnge grows bigger as the vectors are longer.                            
                    else:
                        #print('pose existing nearby as', next_pose_id,'not creating new node')
                        hypo_qs = self.infer_states([next_pose_id], np.array([action_id]), partial_ob=1, save_hist=False, qs=prev_step_qs)

                    if state_step > sure_about_data_until_this_state and pose_in_action_range:
                            #print('DIRECT MOTION AT STATE STEP',state_step)
                            self.update_imagined_translation(prev_step_qs[:], 0, n_actions, action_id, cur_ref_pose, \
                                        min_dist_to_next_node, obstacle_dist_per_actions)
                    prev_step_qs = hypo_qs[:]
                    cur_ref_pose = self.PoseMemory.id_to_pose(next_pose_id)
                    pose_in_action_range = self.PoseMemory.pose_in_action_range(action_id, cur_ref_pose, odom= cur_pose, influence_radius=next_state_min_dist_to_next_node)#doesn't work
                    #print('cur_ref_pose', cur_ref_pose, 'can be reached from ', cur_pose, 'with action', action_id,'?:', pose_in_action_range)
                    
                state_step +=1
            #3), 5)->6)with offset 0 and 8)
            qs = self.update_qs_dim(qs)
            self.update_imagined_translation(qs[:], action_jump, n_actions, action_id, cur_pose, \
                                   min_dist_to_next_node, obstacle_dist_per_actions)
               
    def agent_step_update(self, action_id:int, obs:list, logs=None)->None:
        """
        Updates the agent's belief state, transition probabilities, and learned environment 
        model based on the given action and observations.

        This method performs the following steps:
        1. **Infer new pose**: Updates the agent's estimated position.
        2. **Update observation model (A) and control model (C)**: Adjusts probability distributions 
        to incorporate new observations.
        3. **Update transition model (B)**: Modifies transition probabilities based on inferred states.
        4. **Update belief states**: Updates internal beliefs given the latest observations.
        5. **Update state mapping for visualization**: Stores the agent's inferred position and 
        corresponding belief state.
        6. **Update transition nodes**: Modifies state transitions based on perceived obstacles.
        7. **Ensure stationary transitions**: If a 'STAY' action exists, enforces it in the transition model.

        Parameters:
            action_id (int): The index of the action taken by the agent.
            obs (list): The list of observations, expected to contain:
                - primary_ob (int): The primary observed feature (e.g., color).
                - pose_id (int, optional): The ID of the inferred pose. 
                - obstacles_dist_per_action_range (list): Distances to obstacles for each action.

        Returns:
            None
        """
        #we could get action_id from ours.action instead?

        #We only update A and B if we have inferred a current pose
        #Thus until doubt over current loc is not solved, we don't update internal self
        if self.current_pose != None:
            #Just for memory sake
            primary_ob = obs[0]
            if isinstance(obs[1],int):
                pose_id = obs[1]
            else:
                pose_id = self.PoseMemory.pose_to_id(self.current_pose)

            obstacles_dist_per_action_range = obs[-1]
            
            #1. INFER NEW POSE (should be after motion and before update)
            # pose = self.PoseMemory.id_to_pose(pose_id)
            # self.current_pose = self.infer_pose(pose) #Not sure it shouold be here. in case i want to give whatever...
            
            observations = [primary_ob,pose_id]
            if logs:
                logs.info('observations pose %f, action %f, ob_id %f, obstacles %s' % (pose_id, action_id, primary_ob, str(obstacles_dist_per_action_range)))
            #2. UPDATE A C DIM IF NEW OB
            self.update_A_dim_given_obs(observations, null_proba=[True,False])
            self.update_C_dim()
            #updating B in case pose_id new
            self.update_B_dim_given_A()
            #3. UPDATE BELIEVES GIVEN OBS
            prior = self.infer_states(observations, save_hist=False)

            #print('prior on believed state; action', self.action, action_id, \
            #    'colour_ob:', primary_ob , 'inf pose:',self.current_pose,'prior belief:', prior[0].round(3))
                
            self.update_believes_with_obs(prior,action=action_id, obs=observations)

            posterior = self.infer_states(observations, save_hist=True)
            ## agent_state_mapping for TEST PURPOSES and visualisation
            self.update_agent_state_mapping(tuple(self.current_pose[:2]), observations, posterior[0])
            #4. update all nodes
            self.update_transition_nodes(obstacle_dist_per_actions=obstacles_dist_per_action_range, logs=None)
            #This is not mandatory, just a gain of time
            if 'STAY' in self.possible_actions.values():
                stay_action = [key for key, value in self.possible_actions.items() if value == 'STAY'][0]
                self.B[0] = set_stationary(self.B[0], stay_action)
