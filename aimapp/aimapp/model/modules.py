import numpy as np
import copy
from aimapp.model.pymdp import maths, utils, inference, control
from aimapp.model.pymdp.algos import run_mmp
from collections import deque

def min_delta(d1, d2, max_):
    delta = np.min([np.abs(d1 - d2), max_ - np.abs(d1 - d2)])
    return delta

def euclidian_distance(d1:list, d2:list)->int:
    sum = 0
    for i in range(len(d1)):
        sum += min_delta(d1[i],d2[i], np.inf)**2
    delta_dist = np.sqrt(sum)
    return delta_dist

def clip_rad_180(angle):
    while angle > np.pi:
        angle -= 2 * np.pi
    while angle <= -np.pi:
        angle += 2 * np.pi
    return angle

def clip_rad_360(angle):
    while angle < 0:
        angle += 2 * np.pi
    while angle >= 2 * np.pi:
        angle -= 2 * np.pi
    return angle

def signed_delta_rad(angle1, angle2):
    dir = clip_rad_180(angle2 - angle1)

    delta_angle = abs(clip_rad_360(angle1) - clip_rad_360(angle2))

    if (delta_angle < (2 * np.pi - delta_angle)):
        if (dir > 0):
            angle = delta_angle
        else:
            angle = -delta_angle
    else:
        if (dir > 0):
            angle = 2 * np.pi - delta_angle
        else:
            angle = -(2 * np.pi - delta_angle)
    return angle

def clip_deg_360(angle):
    return angle % 360



#==== GEOMETRY =====#

def quadrilater_points(odom:list, zone_influence:list, influence_radius:float):
    quadri_poses = [odom[:2]]
 
    max_dist_angle = np.mean(zone_influence)
    # not pretty but because Copy [:] etc is not really changing pointer...
    zone_influence = [zone_influence[0], max_dist_angle, zone_influence[1]]
    for angle_deg in zone_influence:
        angle_rad = np.deg2rad(angle_deg)
        x =2*influence_radius * np.cos(angle_rad) + odom[0]
        y= 2*influence_radius * np.sin(angle_rad) + odom[1]
        quadri_poses.append([x,y])
        
    return quadri_poses

def triangle_points(odom:list, zone_influence:list, influence_radius:float)->list:
    quadri_poses = [odom[:2]]
     
    for angle_deg in zone_influence:
        angle_rad = np.deg2rad(angle_deg)
        x =2*influence_radius * np.cos(angle_rad) + odom[0]
        y= 2*influence_radius * np.sin(angle_rad) + odom[1]
        quadri_poses.append([x,y])

    return quadri_poses
def sign(p1, p2, p3):
    return (p1[0]- p3[0]) * (p2[1] - p3[1]) - (p2[0] - p3[0]) * (p1[1] - p3[1])

def point_in_triangle(pt:list, triangle_poses:list)->bool:
    '''
    checking if point in triangle
    '''
    p1,p2,p3 = triangle_poses[0], triangle_poses[1], triangle_poses[2]
    d1 = sign(pt, p1, p2)
    d2 = sign(pt, p2, p3)
    d3 = sign(pt, p3, p1)
    has_neg = (d1 < 0) or (d2 < 0) or (d3 < 0)
    has_pos = (d1 > 0) or (d2 > 0) or (d3 > 0) 
    return not (has_neg and has_pos)

def point_in_polygon(pt, polygon):
    '''
    Checking if point in polygon
    '''
    num_vertices = len(polygon)
    inside = False
 
    # Store the first point in the polygon and initialize the second point
    p1 = polygon[0]
 
    # Loop through each edge in the polygon
    for i in range(1, num_vertices + 1):
        # Get the next point in the polygon
        p2 = polygon[i % num_vertices]
 
        # Check if the point is above the minimum y coordinate of the edge
        if pt[1] > min(p1[1], p2[1]):
            # Check if the point is below the maximum y coordinate of the edge
            if pt[1] <= max(p1[1], p2[1]):
                # Check if the point is to the left of the maximum x coordinate of the edge
                if pt[0] <= max(p1[0], p2[0]):
                    # Calculate the x-intersection of the line connecting the point to the edge
                    x_intersection = (pt[1] - p1[1]) * (p2[0] - p1[0]) / (p2[1] - p1[1]) + p1[0]
 
                    # Check if the point is on the same line as the edge or to the left of the x-intersection
                    if p1[0] == p2[0] or pt[0] <= x_intersection:
                        # Flip the inside flag
                        inside = not inside
 
        # Store the current point as the first point for the next iteration
        p1 = p2
 
    # Return the value of the inside flag
    return inside

def angle_turn_from_pose_to_p(pose:list, goal_pose:list, in_deg:bool=False):
        """
        Compute the angle required to turn from `pose` to `goal_pose`, 
        treating `pose` as the reference.

        Args:
            pose (list): The reference position [x, y].
            goal_pose (list): The target position [x, y].
            in_deg (bool, optional): If True, returns the angle in degrees; 
            otherwise, in radians.

        Returns:
            float: The angle to turn from `pose` to `p`, clipped to [0, 360) 
            degrees if in degrees, or [0, 2π) radians otherwise.
        """
        angle_pose_to_p = clip_rad_360(np.arctan2(goal_pose[1]- pose[1], goal_pose[0]- pose[0]))
        if in_deg:
            angle_pose_to_p = np.rad2deg(angle_pose_to_p)
        return angle_pose_to_p

def pt_between_vectors(pt:list, odom:list, p1:list, p2:list)->bool:
    '''
    We expect p2 clockwise vector from p1
    '''
    angle1 = clip_rad_360(pt_vector_angle_to_ref_pt(p1, odom))
    angle2 = clip_rad_360(pt_vector_angle_to_ref_pt(p2, odom))
    angle = clip_rad_360(pt_vector_angle_to_ref_pt(pt, odom))
    if angle2 == 0.0:
        angle2 = 2*np.pi
    #print('pt_between_vectors', round(np.rad2deg(angle1),2) ,'<=', round(np.rad2deg(angle),2), '<=',round(np.rad2deg(angle2),2))
    return angle1 <= angle <= angle2
  
def pt_vector_angle_to_ref_pt(pt:list, odom:list):
    return clip_rad_360(np.arctan2(pt[1]-odom[1], pt[0]-odom[0]))

def is_clokwise_from_p1_to_p2(p1:list, p2:list)->bool:
    ''' 
    Determines whether `p2` is in the **clockwise direction** from `p1`.

    This is done by projecting `p2` onto the **perpendicular normal** of `p1`. 
    If the projection is **negative**, `p2` is clockwise from `p1`, otherwise counterclockwise.

    Parameters:
        p1 (list or tuple): The first 2D point `[x1, y1]`
        p2 (list or tuple): The second 2D point `[x2, y2]`

    Returns:
        bool: `True` if `p2` is **clockwise** from `p1`, otherwise `False`
    '''
    #p1_n= [-p1[1],p1[0]] #counter clockwise normal
    pt_proj = -p1[0]*p2[1] + p1[1]*p2[0]
    return pt_proj < 0

def is_within_radius_range(pt:list, arc_center:list, arc_radius:float)->bool:
    '''
    Checks if a point `pt` lies within a circular region **centered at the origin** 
    with the given `arc_radius`.

    Parameters:
        pt (list or tuple): The 2D point `[x, y]` we evaluate
        arc_center (list or tuple): the center of the arc (usually odom)
        arc_radius (float): The radius of the circular region

    Returns:
        bool: `True` if `pt` is **inside or on** the circle, otherwise `False`
    '''
    pt_dist_to_arc_center = euclidian_distance(arc_center, pt)
    return pt_dist_to_arc_center <= arc_radius

def point_in_triangle_with_arc(pt:list, polygon:list)-> bool:
    '''
    Determines if a point `pt` lies **inside a triangular region with one curved edge**.

    The **polygon** is expected to have **4 points**:
    - `[arc_center, start_vector, point_on_arc, end_vector]`
    - The **arc** is formed between `start_vector` and `end_vector`, centered at `point_on_arc`.

    **Steps:**
    1. Check if the point is inside the regular **triangle** formed by `arc_center, start_vector, end_vector`.
    2. If not, check if it's inside the **curved region**:
        - The point must be within the **circular range**.
        - The point must be **between** `start_vector` and `end_vector` in a clockwise sense.

    Parameters:
        pt (list or tuple): The 2D point `[x, y]` to check.
        polygon (list of lists): A list of **4 points** `[arc_center, start_vector, point_on_arc, end_vector]`.

    Returns:
        bool: `True` if `pt` is inside the **triangle-with-arc**, otherwise `False`.
    
    '''
    # point_on_arc = polygon[2]
    arc_center, p1, p2 = polygon[0], polygon[1], polygon[3]
    arc_radius = euclidian_distance(arc_center, polygon[2]) #should be influence_radius*2
    triangle = [arc_center, p1, p2]
    # Check if inside the triangle
    if point_in_triangle(pt, triangle):
        # print(pt,'In triangle')
        return True 

    
    # If p2 vector is clocwise from pt and pt is clocwise from p1
    #  and in the circular region, then it's in the zone
    if is_within_radius_range(pt, arc_center, arc_radius) and \
        pt_between_vectors(pt, arc_center, p1,p2):
       #print(pt,'point_in_triangle_with_arc: In arc')
       return True

    return False 

#==== pymdp modified methods ====#
def run_partial_ob_vanilla_fpi(A, obs, num_obs, num_states, partial_ob=None, prior=None, num_iter=10, dF=1.0, dF_tol=0.001):
    """
    Update marginal posterior beliefs over hidden states using mean-field variational inference, via
    fixed point iteration. 

    Parameters
    ----------
    A: ``numpy.ndarray`` of dtype object
        Sensory likelihood mapping or 'observation model', mapping from hidden states to observations. Each element ``A[m]`` of
        stores an ``np.ndarray`` multidimensional array for observation modality ``m``, whose entries ``A[m][i, j, k, ...]`` store 
        the probability of observation level ``i`` given hidden state levels ``j, k, ...``
    obs: numpy 1D array or numpy ndarray of dtype object
        The observation (generated by the environment). If single modality, this should be a 1D ``np.ndarray``
        (one-hot vector representation). If multi-modality, this should be ``np.ndarray`` of dtype object whose entries are 1D one-hot vectors.
    num_obs: list of ints
        List of dimensionalities of each observation modality
    num_states: list of ints
        List of dimensionalities of each observation modality
    prior: numpy ndarray of dtype object, default None
        Prior over hidden states. If absent, prior is set to be the log uniform distribution over hidden states (identical to the 
        initialisation of the posterior)
    num_iter: int, default 10
        Number of variational fixed-point iterations to run until convergence.
    dF: float, default 1.0
        Initial free energy gradient (dF/dt) before updating in the course of gradient descent.
    dF_tol: float, default 0.001
        Threshold value of the time derivative of the variational free energy (dF/dt), to be checked at 
        each iteration. If dF <= dF_tol, the iterations are halted pre-emptively and the final 
        marginal posterior belief(s) is(are) returned
  
    Returns
    ----------
    qs: numpy 1D array, numpy ndarray of dtype object, optional
        Marginal posterior beliefs over hidden states at current timepoint
    """
    if partial_ob != None:
        A = np.array(A[partial_ob])

    # get model dimensions
    n_modalities = len(num_obs)#unused, else would be an issue with partial ob
    n_factors = len(num_states)
    # print('num_states', num_states, A.shape, partial_ob)
    """
    =========== Step 1 ===========
        Loop over the observation modalities and use assumption of independence 
        among observation modalitiesto multiply each modality-specific likelihood 
        onto a single joint likelihood over hidden factors [size num_states]
    """
  
    likelihood = maths.get_joint_likelihood(A, obs, num_states)

    likelihood = maths.spm_log_single(likelihood)

    

    """
    =========== Step 2 ===========
        Create a flat posterior (and prior if necessary)
    """

    qs = np.empty(n_factors, dtype=object)
    for factor in range(n_factors):
        qs[factor] = np.ones(num_states[factor]) / num_states[factor]

    """
    If prior is not provided, initialise prior to be identical to posterior 
    (namely, a flat categorical distribution). Take the logarithm of it (required for 
    FPI algorithm below).
    """
    if prior is None:
        prior = utils.obj_array_uniform(num_states)
        
    prior = maths.spm_log_obj_array(prior) # log the prior


    """
    =========== Step 3 ===========
        Initialize initial free energy
    """
    prev_vfe = maths.calc_free_energy(qs, prior, n_factors)

    """
    =========== Step 4 ===========
        If we have a single factor, we can just add prior and likelihood because there is a unique FE minimum that can reached instantaneously,
        otherwise we run fixed point iteration
    """
    
    if n_factors == 1:

        qL = maths.spm_dot(likelihood, qs, [0])
        qs = utils.to_obj_array(maths.softmax(qL + prior[0]))
        # print('likelihood, qL, qS',likelihood, qL, qs)
        
        return qs

    else:
        """
        =========== Step 5 ===========
        Run the FPI scheme
        """

        curr_iter = 0
        while curr_iter < num_iter and dF >= dF_tol:
            # Initialise variational free energy
            vfe = 0

            # arg_list = [likelihood, list(range(n_factors))]
            # arg_list = arg_list + list(chain(*([qs_i,[i]] for i, qs_i in enumerate(qs)))) + [list(range(n_factors))]
            # LL_tensor = np.einsum(*arg_list)

            qs_all = qs[0]
            for factor in range(n_factors-1):
                qs_all = qs_all[...,None]*qs[factor+1]
            LL_tensor = likelihood * qs_all

            for factor, qs_i in enumerate(qs):
                # qL = np.einsum(LL_tensor, list(range(n_factors)), 1.0/qs_i, [factor], [factor])
                qL = np.einsum(LL_tensor, list(range(n_factors)), [factor])/qs_i
                qs[factor] = maths.softmax(qL + prior[factor])

            # List of orders in which marginal posteriors are sequentially multiplied into the joint likelihood:
            # First order loops over factors starting at index = 0, second order goes in reverse
            # factor_orders = [range(n_factors), range((n_factors - 1), -1, -1)]

            # iteratively marginalize out each posterior marginal from the joint log-likelihood
            # except for the one associated with a given factor
            # for factor_order in factor_orders:
            #     for factor in factor_order:
            #         qL = spm_dot(likelihood, qs, [factor])
            #         qs[factor] = softmax(qL + prior[factor])

            # calculate new free energy
            vfe = maths.calc_free_energy(qs, prior, n_factors, likelihood)

            # stopping condition - time derivative of free energy
            dF = np.abs(prev_vfe - vfe)
            prev_vfe = vfe

            curr_iter += 1

        return qs
    
def update_posterior_states(A, obs, prior=None, partial_ob=None, **kwargs):
    """
    Update marginal posterior over hidden states using mean-field fixed point iteration 
    FPI or Fixed point iteration. 

    See the following links for details:
    http://www.cs.cmu.edu/~guestrin/Class/10708/recitations/r9/VI-view.pdf, slides 13- 18, and http://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.137.221&rep=rep1&type=pdf, slides 24 - 38.
    
    Parameters
    ----------
    A: ``numpy.ndarray`` of dtype object
        Sensory likelihood mapping or 'observation model', mapping from hidden states to observations. Each element ``A[m]`` of
        stores an ``np.ndarray`` multidimensional array for observation modality ``m``, whose entries ``A[m][i, j, k, ...]`` store 
        the probability of observation level ``i`` given hidden state levels ``j, k, ...``
    obs: 1D ``numpy.ndarray``, ``numpy.ndarray`` of dtype object, int or tuple
        The observation (generated by the environment). If single modality, this can be a 1D ``np.ndarray``
        (one-hot vector representation) or an ``int`` (observation index)
        If multi-modality, this can be ``np.ndarray`` of dtype object whose entries are 1D one-hot vectors,
        or a tuple (of ``int``)
    prior: 1D ``numpy.ndarray`` or ``numpy.ndarray`` of dtype object, default None
        Prior beliefs about hidden states, to be integrated with the marginal likelihood to obtain
        a posterior distribution. If not provided, prior is set to be equal to a flat categorical distribution (at the level of
        the individual inference functions).
    **kwargs: keyword arguments 
        List of keyword/parameter arguments corresponding to parameter values for the fixed-point iteration
        algorithm ``algos.fpi.run_vanilla_fpi.py``

    Returns
    ----------
    qs: 1D ``numpy.ndarray`` or ``numpy.ndarray`` of dtype object
        Marginal posterior beliefs over hidden states at current timepoint
    """

    num_obs, num_states, num_modalities, num_factors = utils.get_model_dimensions(A = A)

    if prior is not None:
        prior = utils.to_obj_array(prior)

    if partial_ob is None:
        obs = utils.process_observation(obs, num_modalities, num_obs)
    else:
        obs = utils.process_observation(obs, 1, [num_obs[partial_ob]])
        
    qs = run_partial_ob_vanilla_fpi(A, obs, num_obs, num_states, partial_ob, prior, **kwargs)
    return qs

def update_posterior_states_full(
    A,
    B,
    prev_obs,
    policies,
    prev_actions=None,
    prior=None,
    policy_sep_prior = True,
    partial_ob = None,
    **kwargs,
):

    num_obs, num_states, num_modalities, num_factors = utils.get_model_dimensions(A, B)
    if partial_ob != None:
        A = np.array(A[partial_ob])
        num_obs = [num_obs[partial_ob]]
        num_modalities = 1
    
    proc_obs_seq = utils.obj_array(len(prev_obs))
    for t, obs_t in enumerate(prev_obs):
        if len(obs_t) > 1 and partial_ob != None:
            obs_t = obs_t[partial_ob]
        proc_obs_seq[t] = utils.process_observation(obs_t, num_modalities, num_obs)
    prev_obs = proc_obs_seq
    
    lh_seq = inference.get_joint_likelihood_seq(A, prev_obs, num_states)

    if prev_actions is not None:
        prev_actions = np.stack(prev_actions,0)

    qs_seq_pi = utils.obj_array(len(policies))
    F = np.zeros(len(policies)) # variational free energy of policies

    for p_idx, policy in enumerate(policies):
        # get sequence and the free energy for policy
        qs_seq_pi[p_idx], F[p_idx] = run_mmp(
            lh_seq,
            B,
            policy,
            prev_actions=prev_actions,
            prior= prior[p_idx] if policy_sep_prior else prior, 
            **kwargs
        )
    # print('qs_seq_pi', qs_seq_pi[0].shape)
    return qs_seq_pi, F

def update_state_likelihood_dirichlet(
    pB, B, actions, qs, qs_prev, lr=1.0, factors="all"
):
    """
    Update Dirichlet parameters of the transition distribution. 

    Parameters
    -----------
    pB: ``numpy.ndarray`` of dtype object
        Prior Dirichlet parameters over transition model (same shape as ``B``)
    B: ``numpy.ndarray`` of dtype object
        Dynamics likelihood mapping or 'transition model', mapping from hidden states at ``t`` to hidden states at ``t+1``, given some control state ``u``.
        Each element ``B[f]`` of this object array stores a 3-D tensor for hidden state factor ``f``, whose entries ``B[f][s, v, u]`` store the probability
        of hidden state level ``s`` at the current time, given hidden state level ``v`` and action ``u`` at the previous time.
    actions: 1D ``numpy.ndarray``
        A vector with length equal to the number of control factors, where each element contains the index of the action (for that control factor) performed at 
        a given timestep.
    qs: 1D ``numpy.ndarray`` or ``numpy.ndarray`` of dtype object
        Marginal posterior beliefs over hidden states at current timepoint.
    qs_prev: 1D ``numpy.ndarray`` or ``numpy.ndarray`` of dtype object
        Marginal posterior beliefs over hidden states at previous timepoint.
    lr: float, default ``1.0``
        Learning rate, scale of the Dirichlet pseudo-count update.
    factors: ``list``, default "all"
        Indices (ranging from 0 to ``n_factors - 1``) of the hidden state factors to include 
        in learning. Defaults to "all", meaning that factor-specific sub-arrays of ``pB``
        are all updated using the corresponding hidden state distributions and actions.

    Returns
    -----------
    qB: ``numpy.ndarray`` of dtype object
        Posterior Dirichlet parameters over transition model (same shape as ``B``), after having updated it with state beliefs and actions.
    """

    num_factors = len(pB)

    qB = copy.deepcopy(pB)
    
    if factors == "all":
        factors = list(range(num_factors))

    for factor in factors:
        dfdb = maths.spm_cross(qs[factor], qs_prev[factor])
        print('update_B: a', actions[factor],'qs[factor]',qs[factor].round(3), 'qs_prev[factor]',qs_prev[factor].round(3))
        # print('dfdb',dfdb)
        dfdb *= (B[factor][:, :, int(actions[factor])] > 0).astype("float")
        qB[factor][:,:,int(actions[factor])] += (lr*dfdb)

    return qB

def update_posterior_policies(
    qs,
    A,
    B,
    C,
    policies,
    use_utility=True,
    use_states_info_gain=True,
    use_param_info_gain=False,
    pA=None,
    pB=None,
    E = None,
    gamma=16.0,
    diff_policy_len = False,
    logs= None
):
    """
    Update posterior beliefs about policies by computing expected free energy of each policy and integrating that
    with the prior over policies ``E``. This is intended to be used in conjunction
    with the ``update_posterior_states`` method of the ``inference`` module, since only the posterior about the hidden states at the current timestep
    ``qs`` is assumed to be provided, unconditional on policies. The predictive posterior over hidden states under all policies Q(s, pi) is computed 
    using the starting posterior about states at the current timestep ``qs`` and the generative model (e.g. ``A``, ``B``, ``C``)

    Parameters
    ----------
    qs: ``numpy.ndarray`` of dtype object
        Marginal posterior beliefs over hidden states at current timepoint (unconditioned on policies)
    A: ``numpy.ndarray`` of dtype object
        Sensory likelihood mapping or 'observation model', mapping from hidden states to observations. Each element ``A[m]`` of
        stores an ``numpy.ndarray`` multidimensional array for observation modality ``m``, whose entries ``A[m][i, j, k, ...]`` store 
        the probability of observation level ``i`` given hidden state levels ``j, k, ...``
    B: ``numpy.ndarray`` of dtype object
        Dynamics likelihood mapping or 'transition model', mapping from hidden states at ``t`` to hidden states at ``t+1``, given some control state ``u``.
        Each element ``B[f]`` of this object array stores a 3-D tensor for hidden state factor ``f``, whose entries ``B[f][s, v, u]`` store the probability
        of hidden state level ``s`` at the current time, given hidden state level ``v`` and action ``u`` at the previous time.
    C: ``numpy.ndarray`` of dtype object
       Prior over observations or 'prior preferences', storing the "value" of each outcome in terms of relative log probabilities. 
       This is softmaxed to form a proper probability distribution before being used to compute the expected utility term of the expected free energy.
    policies: ``list`` of 2D ``numpy.ndarray``
        ``list`` that stores each policy in ``policies[p_idx]``. Shape of ``policies[p_idx]`` is ``(num_timesteps, num_factors)`` where `num_timesteps` is the temporal
        depth of the policy and ``num_factors`` is the number of control factors.
    use_utility: ``Bool``, default ``True``
        Boolean flag that determines whether expected utility should be incorporated into computation of EFE.
    use_states_info_gain: ``Bool``, default ``True``
        Boolean flag that determines whether state epistemic value (info gain about hidden states) should be incorporated into computation of EFE.
    use_param_info_gain: ``Bool``, default ``False`` 
        Boolean flag that determines whether parameter epistemic value (info gain about generative model parameters) should be incorporated into computation of EFE.
    pA: ``numpy.ndarray`` of dtype object, optional
        Dirichlet parameters over observation model (same shape as ``A``)
    pB: ``numpy.ndarray`` of dtype object, optional
        Dirichlet parameters over transition model (same shape as ``B``)
    E: 1D ``numpy.ndarray``, optional
        Vector of prior probabilities of each policy (what's referred to in the active inference literature as "habits")
    gamma: float, default 16.0
        Prior precision over policies, scales the contribution of the expected free energy to the posterior over policies

    Returns
    ----------
    q_pi: 1D ``numpy.ndarray``
        Posterior beliefs over policies, i.e. a vector containing one posterior probability per policy.
    G: 1D ``numpy.ndarray``
        Negative expected free energies of each policy, i.e. a vector containing one negative expected free energy per policy.
    """

    n_policies = len(policies)
    G = np.zeros(n_policies)
    q_pi = np.zeros((n_policies, 1))
    info_gain_list = np.zeros(n_policies)
    utility_term_list = np.zeros(n_policies)

    if E is None:
        lnE = maths.spm_log_single(np.ones(n_policies) / n_policies)
    else:
        lnE =  maths.spm_log_single(E) 
    for idx, policy in enumerate(policies):
        qs_pi =  control.get_expected_states(qs, B, policy)
        qo_pi =  control.get_expected_obs(qs_pi, A)
        policy_length = len(policy)

        if use_utility:
            utility_term = control.calc_expected_utility(qo_pi, C)
            if diff_policy_len : 
                utility_term = utility_term/ policy_length
            utility_term_list[idx] = utility_term
            G[idx] +=  utility_term

        if use_states_info_gain:
            info_gain =  control.calc_states_info_gain(A, qs_pi)

            if diff_policy_len : 
                info_gain = info_gain/ policy_length
            info_gain_list[idx] = info_gain
            G[idx] += info_gain

        if use_param_info_gain:
            if pA is not None:
                param_info_gain = control.calc_pA_info_gain(pA, qo_pi, qs_pi)
                if diff_policy_len : 
                    param_info_gain = info_gain/ policy_length
                G[idx] +=  param_info_gain
            if pB is not None:
                param_info_gain = control.calc_pB_info_gain(pB, qs_pi, qs, policy)
                if diff_policy_len : 
                    param_info_gain = info_gain/ policy_length
                G[idx] +=  param_info_gain
    q_pi =  maths.softmax(G * gamma + lnE)    

    return q_pi, G, info_gain_list, utility_term_list

def update_posterior_policies_full(
    qs_seq_pi,
    A,
    B,
    C,
    policies,
    use_utility=True,
    use_states_info_gain=True,
    use_param_info_gain=False,
    prior=None,
    pA=None,
    pB=None,
    F = None,
    E = None,
    gamma=16.0,
    diff_policy_len = False
):  
    """
    Update posterior beliefs about policies by computing expected free energy of each policy and integrating that
    with the variational free energy of policies ``F`` and prior over policies ``E``. This is intended to be used in conjunction
    with the ``update_posterior_states_full`` method of ``inference.py``, since the full posterior over future timesteps, under all policies, is
    assumed to be provided in the input array ``qs_seq_pi``.

    Parameters
    ----------
    qs_seq_pi: ``numpy.ndarray`` of dtype object
        Posterior beliefs over hidden states for each policy. Nesting structure is policies, timepoints, factors,
        where e.g. ``qs_seq_pi[p][t][f]`` stores the marginal belief about factor ``f`` at timepoint ``t`` under policy ``p``.
    A: ``numpy.ndarray`` of dtype object
        Sensory likelihood mapping or 'observation model', mapping from hidden states to observations. Each element ``A[m]`` of
        stores an ``numpy.ndarray`` multidimensional array for observation modality ``m``, whose entries ``A[m][i, j, k, ...]`` store 
        the probability of observation level ``i`` given hidden state levels ``j, k, ...``
    B: ``numpy.ndarray`` of dtype object
        Dynamics likelihood mapping or 'transition model', mapping from hidden states at ``t`` to hidden states at ``t+1``, given some control state ``u``.
        Each element ``B[f]`` of this object array stores a 3-D tensor for hidden state factor ``f``, whose entries ``B[f][s, v, u]`` store the probability
        of hidden state level ``s`` at the current time, given hidden state level ``v`` and action ``u`` at the previous time.
    C: ``numpy.ndarray`` of dtype object
       Prior over observations or 'prior preferences', storing the "value" of each outcome in terms of relative log probabilities. 
       This is softmaxed to form a proper probability distribution before being used to compute the expected utility term of the expected free energy.
    policies: ``list`` of 2D ``numpy.ndarray``
        ``list`` that stores each policy in ``policies[p_idx]``. Shape of ``policies[p_idx]`` is ``(num_timesteps, num_factors)`` where `num_timesteps` is the temporal
        depth of the policy and ``num_factors`` is the number of control factors.
    use_utility: ``Bool``, default ``True``
        Boolean flag that determines whether expected utility should be incorporated into computation of EFE.
    use_states_info_gain: ``Bool``, default ``True``
        Boolean flag that determines whether state epistemic value (info gain about hidden states) should be incorporated into computation of EFE.
    use_param_info_gain: ``Bool``, default ``False`` 
        Boolean flag that determines whether parameter epistemic value (info gain about generative model parameters) should be incorporated into computation of EFE. 
    prior: ``numpy.ndarray`` of dtype object, default ``None``
        If provided, this is a ``numpy`` object array with one sub-array per hidden state factor, that stores the prior beliefs about initial states. 
        If ``None``, this defaults to a flat (uninformative) prior over hidden states.
    pA: ``numpy.ndarray`` of dtype object, default ``None``
        Dirichlet parameters over observation model (same shape as ``A``)
    pB: ``numpy.ndarray`` of dtype object, default ``None``
        Dirichlet parameters over transition model (same shape as ``B``)
    F: 1D ``numpy.ndarray``, default ``None``
        Vector of variational free energies for each policy
    E: 1D ``numpy.ndarray``, default ``None``
        Vector of prior probabilities of each policy (what's referred to in the active inference literature as "habits"). If ``None``, this defaults to a flat (uninformative) prior over policies.
    gamma: ``float``, default 16.0
        Prior precision over policies, scales the contribution of the expected free energy to the posterior over policies

    Returns
    ----------
    q_pi: 1D ``numpy.ndarray``
        Posterior beliefs over policies, i.e. a vector containing one posterior probability per policy.
    G: 1D ``numpy.ndarray``
        Negative expected free energies of each policy, i.e. a vector containing one negative expected free energy per policy.
    """

    # num_obs, num_states, num_modalities, num_factors = utils.get_model_dimensions(A, B)
    
    num_policies = len(policies)

   

    # horizon = len(qs_seq_pi[0])
    # num_policies = len(qs_seq_pi)
    # qo_seq = utils.obj_array(horizon)
    # for t in range(horizon):
    #     qo_seq[t] = utils.obj_array_zeros(num_obs)

    # initialise expected observations
    # qo_seq_pi = utils.obj_array(num_policies)

    # initialize (negative) expected free energies for all policies
    G = np.zeros(num_policies)

    if F is None:
        F =  maths.spm_log_single(np.ones(num_policies) / num_policies)

    if E is None:
        lnE =  maths.spm_log_single(np.ones(num_policies) / num_policies)
    else:
        lnE = maths.spm_log_single(E) 


    for p_idx, policy in enumerate(policies):
        policy_length = len(policy)

        qs_pi = control.get_expected_states(qs_seq_pi, B, policy)
        qo_pi = control.get_expected_obs(qs_pi, A)
        
        if use_utility:
            utility_term = control.calc_expected_utility(qo_pi, C)
            if diff_policy_len : 
                utility_term = utility_term/ policy_length

            G[p_idx] +=  utility_term
            
        if use_states_info_gain:
            info_gain = control.calc_states_info_gain(A, qs_pi)
            if diff_policy_len : 
                info_gain = info_gain/ policy_length
            G[p_idx] += info_gain
        if use_param_info_gain:
            if pA is not None:
                param_info_gain = control.calc_pA_info_gain(pA, qo_pi, qs_pi)
                if diff_policy_len : 
                    param_info_gain = param_info_gain/ policy_length
                G[p_idx] += param_info_gain

            if pB is not None:
                param_info_gain = control.calc_pB_info_gain(pB, qs_pi, qs_seq_pi, policy)
                if diff_policy_len : 
                    param_info_gain = param_info_gain/ policy_length
                G[p_idx] += param_info_gain

    q_pi = maths.softmax(G * gamma - F + lnE)
    
    return q_pi, G

#==== Update A and B ====#

def set_stationary(mat, idx=-1):
    mat[:,:,idx] = np.eye(mat.shape[0])
    return mat

def create_B_matrix(num_states:int, num_actions:int)->np.ndarray:
    """ 
    generate a Transmition matrix B of shape 
    [n_states, n_states, n_actions] with normalised content
    """
    #create matrix of dim: (st, st-1, action)
    B = np.ones([num_states, num_states, num_actions])
    B /= num_states
    return B

def create_A_matrix(num_ob:list, num_states:list, dim:int=1)->np.ndarray:
    """ 
    generate in one Emission matrix A all the observation/states relationship
    each P(o|s) are of shape 
    [num_ob, n_states] with normalised content
    dim: the number of observations we will feed A with.
    """
    A = utils.obj_array_ones([[num_ob[d]] + [num_states[d]] for d in range(dim)])
    A = A/ num_ob
    return A

def update_B_matrix_size(B:np.ndarray, add:int=1, alter_weights:bool = True)->np.ndarray:
    """
    Expands the transition matrix B by adding new states and adjusts transition probabilities accordingly.

    Args:
        B (np.ndarray): The original transition matrix of shape (num_states, num_states, num_actions).
        add (int, optional): The number of new states to add. Defaults to 1.
        alter_weights (bool, optional): If True, modifies the transition probabilities to make 
                                        newly added states less probable than existing ones. Defaults to True.

    Returns:
        np.ndarray: The updated transition matrix with new states added.

    Notes:
        - The method initialises a new, larger transition matrix and copies the original matrix values into it.
        - Transition probabilities are renormalised after expansion.
        - If `alter_weights` is enabled, newly introduced states have their probabilities reduced to 0.05.
    """
    old_B_mean_value = 1/B[0].shape[0]
    num_states = B[0].shape[0] + add
    new_B = create_B_matrix(num_states, B[0].shape[-1])
    new_B_mean_value = 1/new_B[0].shape[0]
    #new_B -= 0.9/num_states
    
    slices = tuple([slice(dim) for dim in B[0].shape])
    new_B[slices] = B[0]
    if alter_weights:
        new_B[new_B == old_B_mean_value] = 0.05
        new_B[new_B == new_B_mean_value] = 0.05
    #If there are values that have not been explored yet, 
    # their initial values is vastly reduced to an arbitrarely number
        
    B[0] = new_B
    return B

def update_A_matrix_size(A, add_ob=0, add_state=0, null_proba = True):
    ''' increase the square matrix A by the values adds,
    add_ob: row add
    add_state: col add
    feed the content of original matrix A in the newly generated one
    '''
    num_ob = A.shape[0] + add_ob
    num_states = A.shape[1] + add_state
    
    new_A = create_A_matrix([num_ob], [num_states], 1)[0]
    if null_proba:
        new_A[:] = 0.001 #to avoid div 0
    else:
        prev_A_mean_value = 1/A.shape[0]
        new_A[new_A == prev_A_mean_value] = 1/num_ob 
        new_A[new_A == 1/(A.shape[0]*2)] = 1/(num_ob*2)  #We also reduce those proba 
        new_A[:, :A.shape[1]] = 1/(num_ob*2) 
        
    slices = tuple([slice(d) for d in A.shape])
    new_A[slices] = A
    A = new_A
    return A

#==== Motion inverse ====#

def reverse_action(actions:dict, action:int)->int:
    actions = actions.copy()
    if actions[action] == 'STAY':
        return action
    
    if "STAY" in actions.values():
        actions.popitem()

    mid_orientation = (actions[action][0] + actions[action][1]) /2
    mid_orientation += 180
    reverse_action_mid_orientation= int(clip_deg_360(mid_orientation))
    reverse_action_key = [k for k,v in actions.items() if v[0] < reverse_action_mid_orientation and v[1] > reverse_action_mid_orientation]    

    return reverse_action_key[0]

#==== POLICY GENERATION ====#

def BFS_path_gen(dx:int, dy:int, moves:list):
    """ 
    based on the Breadth-First Search (BFS) we define the position motion considering all the possible moves.
    Given position x and y, the path ends when we reach either x or y with the moves, generating paths going in every directions     
    """
   # Initialize the queue with the starting point
    queue = deque([[(0, 0)]])
    
    valid_paths = []
    
    while queue:
        path = queue.popleft()
        x, y = path[-1]
        
        # Check all possible moves from the current position
        for move in moves:
            new_x, new_y = x + move[0], y + move[1]
            
            # Create a new path by extending the current path
            new_path = path + [(new_x, new_y)]
            
            # Check if the new position reaches the desired boundary
            if (new_x == dx or new_y == dy):
                valid_paths.append(new_path)
            # Only add the new path to the queue if it hasn't reached the boundary yet
            # Ensure this works for both positive and negative dx and dy
            elif (dx > 0 and new_x <= dx or dx < 0 and new_x >= dx) and (dy > 0 and new_y <= dy or dy < 0 and new_y >= dy):
                queue.append(new_path)
    
    return valid_paths

def visited_pose(position, path):
    return position in path


