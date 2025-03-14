import numpy as np
from .modules import euclidian_distance, signed_delta_rad, clip_rad_360,\
                    from_degree_to_point

class PoseMemory(object):
    """
    Memorise consecutive poses consdering a minimum euclidian distance
    return the matching score of given pose to memorised poses  
    """
    def __init__(self,dist_th:float=0.5) -> None:
        """
        param: dist_th(float) = the threshold under which a 
        pose is considered the same as an already registered pose
        """
        self.poses = []
        self.dist_th = dist_th
        self.odometry = [0., 0., 0.]
    
    def reset_odom(self, odometry:list=None):
        if odometry is None:
            self.odometry = [0., 0., 0.]
        else:
            self.odometry = odometry
        
        if len(self.odometry) < 3:
            self.odometry = list(self.odometry)
            self.odometry.append(0.)
    
    def get_odom(self, as_tuple:bool=False) ->list:
        if as_tuple:
            return tuple(self.odometry)
        return self.odometry
    
    def get_poses_from_memory(self)->list:
        return self.poses
    
    def id_to_pose(self, id)->list:
        try:
            return self.poses[id]
        except IndexError:
            return None
    
    def pose_to_id(self, pose:list=None, save_in_memory:bool = True) -> int:
        """ 
        get new pose, already known poses and dist threshold in m, 
        check if pose is near an existing pose or not
        return pose id and known poses """
        if pose is None:
            pose = self.odometry[:2]
        else:
            pose = pose[:2]
        
        dist_p_to_pose = []
        for p in self.poses:
            dist_p_to_pose.append(euclidian_distance(pose, p))
        
        p_idx = -1
        #current pose id will be the closest pose to current
        if len(dist_p_to_pose) > 0 :
            closest_pose_id = np.argmin(dist_p_to_pose)
            if dist_p_to_pose[closest_pose_id] < self.dist_th:
                p_idx = closest_pose_id
        #No identified pose match current pose
        if p_idx < 0 and save_in_memory:
            pose = tuple(pose)
            self.poses.append(pose)
            p_idx = self.poses.index(pose)
        
        return p_idx
        
class HotEncodedActionOdometry(PoseMemory):
    ''' Use HotEncoded Action for Odometry, gridworld adapted'''

    def __init__(self,dist_th:float=0.5):
        super(HotEncodedActionOdometry, self).__init__(dist_th)
        
    def motion_without_memory(self, action:list, pose:list=None):
        if pose is None:
            odometry = self.odometry.copy()
        else:
            odometry = pose
        vtrans = action[0] 
        vrot = 0
        if action[1] != 0: #right
            vrot = np.pi / 2 
        elif action[2] != 0: #left
            vrot = -np.pi / 2 
        
        odometry[2] += vrot
        odometry[2] =  clip_rad_360(odometry[2])
        odometry[0] = odometry[0] + vtrans * round(np.cos(odometry[2]),4) 
        odometry[1] = odometry[1] + vtrans * round(np.sin(odometry[2]),4)

        return odometry
    
    def __call__(self, action:list):
        #action = observations["HEaction"] #hot encoded action : [F,R,L]
        # print('action', action)
        vtrans = action[0] 
        vrot = 0
        if action[1] != 0: #right
            vrot = np.pi / 2 
        elif action[2] != 0: #left
            vrot = -np.pi / 2 

        self.odometry[2] += vrot
        self.odometry[2] = clip_rad_360(self.odometry[2])
        self.odometry[0] += vtrans * round(np.cos(self.odometry[2]),4) 
        self.odometry[1] += vtrans * round(np.sin(self.odometry[2]),4) # the round is there to correct the error on pi that would accumulate sin(2pi) != sin(0)
        #print('in action, vtrans, vrot, odom:', vtrans, vrot, self.odometry)
        return vtrans, vrot
    
    def get_sequential_actions(self,last_action:int, next_action:int, possible_action:dict, odom:list=None) -> list:
        """ 
            Given the current orientation we give back the sequence of actions 
            to execute to move the agent toward new direction
            V1 : with up/right/down/left, we convert this direction in seq of action
            V2 : same idea but with orientations as actions dict instead  (should work for any number of degree equidistant on 360)
        """
        if last_action is None:
            last_action = 0
        possible_action =  {k.upper(): v for k, v in possible_action.items()}
        last_action_key = list(filter(lambda x: possible_action[x] == last_action, possible_action))[0] 
        next_action_key = list(filter(lambda x: possible_action[x] == next_action, possible_action))[0] 

        if next_action_key == 'STAY' :
            return [[0,0,0]] #no action
        
        #MINIGRID LIKE ENV (simple R/L/U/D motions)
        if 'DOWN' in possible_action.keys():
            alt_actions = {'UP':0, 'RIGHT':1, 'DOWN':2, 'LEFT':3}

            l_action = alt_actions[last_action_key]
            n_action = alt_actions[next_action_key]
            
            seq_action = []
            #WE STAY IN SAME DIRECTION
            if l_action == n_action: 
                seq_action.append([1,0,0])
                return seq_action #F
            #WE WANT TO TURN 180DEG
            elif (l_action+n_action)%2 == 0:
                seq_action.append([0,1,0]) #we consider 90deg a time
                seq_action.append([0,1,0])
                seq_action.append([1,0,0])
                return seq_action #RRF
            #WE WANT TO TURN RIGHT
            elif n_action == (l_action+1)%4:
                seq_action.append([0,1,0])
                seq_action.append([1,0,0])
                return seq_action #R
            else:
                seq_action.append([0,0,1])
                seq_action.append([1,0,0])
                return seq_action #L
            
        #RW IN DEGREE
        if list(possible_action.keys())[0].isdigit():
            seq_actions = self.orientation_to_actions(float(last_action_key), float(next_action_key), possible_action)
            seq_actions.append([1,0,0])
            return seq_actions
        return seq_action.append([0,0,0])
    
    def orientation_to_actions(self,prev_angle:float, next_angle:float, possible_action:dict) -> list:
        ''' 
        given 2 orientations and the list of possible orientation
        define the hotencoded actions required to reach this orientation and take 1 step forward 
        '''
        seq_actions = []
        #DEFINE HOW ARE THE ORIENTATION SEQUENCED ON A CIRCLE
        n_actions = len(possible_action)
        if 'STAY' in possible_action:
            n_actions-=1
        angle_per_action = 360 / n_actions

        angle_diff = signed_delta_rad(np.deg2rad(prev_angle), np.deg2rad(next_angle))
        angle_diff = np.rad2deg(angle_diff)

        # Determine how many turns to reach orientation
        n_turns = int(angle_diff / angle_per_action)
        # print(n_turns)
        
        if n_turns < 0:
            turns = abs(n_turns)*[[0,0,1]]
            seq_actions.extend(turns)
        elif n_turns > 0:
            turns = abs(n_turns)*[[0,1,0]]
            seq_actions.extend(turns)
        
        return seq_actions
    
class PoseOdometry(PoseMemory):
    ''' Use pose for Odometry '''

    def __init__(self,dist_th:float=0.5):
        super(PoseOdometry, self).__init__(dist_th)
        ''' TODO: CONSIDER DT'''
        self.latest = None 

    def motion_without_memory(self, pose:list, odometry:list=None):
        if odometry is None:
            odometry = self.odometry.copy()
        
        if len(pose) == 2: #(xy)
            diff = [pose[0] - odometry[0], pose[1]- odometry[1]]
            vrot = clip_rad_360(np.arctan2(diff[1], diff[0]))
            # vtrans = np.sqrt(pow(diff[0],2) + pow(diff[1],2))
             
            odometry[2] = vrot #we will keep this angular direction at pose
            odometry[0] = float(pose[0])
            odometry[1] = float(pose[1])
            return odometry
        
        else:
            print('motion_without_memory:NOT IMPLEMENTED ODOM FOR POSE LEN >2')
        
    def update_odom_given_pose(self, pose):
        p = pose
        if len(p) == 2: #(xy)
            diff = [p[0] - self.odometry[0], p[1]- self.odometry[1]]
            vrot = clip_rad_360(np.arctan2(diff[1], diff[0]))
            vtrans = np.sqrt(pow(diff[0],2) + pow(diff[1],2))
             
            self.odometry[2] = vrot #we will keep this angular direction at pose
            self.odometry[0] = float(p[0])
            self.odometry[1] = float(p[1])
            print('in pose odom, vtrans:',vtrans,' vrot:', vrot,' odom:', self.odometry)
            return vtrans , vrot
        
        print('ERROR in PoseOdometry', pose)
    
    def get_sequential_actions(self,last=None, next=None, possible_actions:dict=None, odom:list=None) -> list:
        """ 
            if we move from pose to pose, we don't have this sequential_actions issue  
            if we give an action, then we need to convert it to pose  
        """
        
        if odom is None:
            odom = self.odometry.copy()
        #did we receive an action? Convert it to pose
        if isinstance(next, (float,int, np.number)):
            
            next_dir = [angle for angle,action in possible_actions.items() if action == int(next)][0]
            if next_dir.upper() == 'STAY':
                  return [odom[:2]]
            motion = from_degree_to_point(float(next_dir), pose_dist=self.dist_th)
            next = (odom[0]+motion[0],odom[1]+motion[1])

        
        return [next]
    
    def from_pose_to_action(self, next:list,possible_actions:dict) -> int:
        """ 
        Given a pose, we transform it into an orientation by considering current odom
        Then we search for the action closest from that orientation.
        """
        diff = [next[0] - self.odometry[0], next[1]- self.odometry[1]]
        if diff[0] + diff[1] == 0:
            if 'STAY' in possible_actions:
                return possible_actions['STAY']  
            else:
                raise ValueError('from_pose_to_action: next pose and current pose are \
                                 the same but we cannot apply action STAY (not existing)')   
        vrot = clip_rad_360(np.arctan2(diff[1], diff[0]))
        vrot_deg = np.rad2deg(vrot)
        # Find the closest angle key
        possible_angles = list(possible_actions.keys())
        if 'STAY' in possible_angles:
            possible_angles.remove('STAY')
        closest_angle = min(possible_angles, key=lambda k: abs(float(k) - vrot_deg))
        action = possible_actions[closest_angle]
        return action
        
