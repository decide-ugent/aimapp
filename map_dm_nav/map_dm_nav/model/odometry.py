import numpy as np
from .modules import euclidian_distance, clip_rad_360,\
                    from_degree_to_point, quadrilater_points, point_in_triangle_with_arc

class PoseMemory(object):
    """
    Memorise consecutive poses consdering a minimum euclidian distance
    return the matching score of given pose to memorised poses  
    """
    def __init__(self, possible_actions:dict, influence_radius:float=0.5, robot_dim:float=0.3) -> None:
        """
        param: influence_radius(float) = the threshold under which a 
        pose is considered the same as an already registered pose
        """
        self.poses = []
        self.influence_radius = influence_radius
        self.possible_actions = possible_actions
        self.robot_dim = robot_dim
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
    
    def pose_to_id(self, pose:list=None, odom:list=None, save_in_memory:bool = True) -> int:
        """
        return the position id from memory. 
        Evaluate if the given pose is in action range of other memorised poses 
        considering it's orientation difference and distance (<zone of influence)

        Parameters:
            pose (list): the pose we want to evaluate from memory
            odom(list,optional): the point we use as reference to consider id.
            save_in_memory (bool, optional): Do we want to save this pose in memory? 

        Returns:
            int: The pose id, -1 if none in memory and we don't want to remember this new pose
        """
        #TODO: CONSIDR ANGLE, ELSE WE Can'T HAVE 12 NODES IN SAME ZONE OF INF
        #TODO: THIS CLOSEST_DIST REF IS RANDOM, THINK IT OVER
        if pose is None:
            pose = self.odometry[:2]
        else:
            pose = pose[:2]

        if odom is None:
            odom = self.odometry[:2]
        else:
             odom = odom[:2]
        
        zone_action = self.possible_actions[0]
        p_idx = -1
        
        ref_closest_dist = self.influence_radius 
        angle_pose_to_curr_pose = clip_rad_360(np.arctan2(pose[1]- odom[1], pose[0]- odom[0]))
        for ref_p_idx, p in enumerate(self.poses):
            # print('_____')
            # angle_p_to_curr_pose= clip_rad_360(np.arctan2(p[1]- odom[1], p[0]- odom[0])) 
            # diff_angle =  abs(angle_pose_to_curr_pose -angle_p_to_curr_pose)
            # angle_ref_pose_to_origin = clip_rad_360(angle_ref_pose_to_origin)
            
            dist_p_to_pose = euclidian_distance(pose, p)
            # print('ref idx and pose', ref_p_idx,p)
            #print('angle pose to curr_pose', round(np.rad2deg(angle_pose_to_curr_pose),2), 'angle_p_to_curr_pose',round(np.rad2deg(angle_p_to_curr_pose),2),'dist', dist_p_to_pose)
            #If pose in the same action and influence zone
            #if abs(np.rad2deg(angle_p_to_pose)) <= np.mean(zone_action) and dist_p_to_pose < ref_closest_dist:
            # if np.rad2deg(diff_angle) <= np.mean(zone_action) and dist_p_to_pose < ref_closest_dist:
            if dist_p_to_pose < ref_closest_dist:
                p_idx = ref_p_idx
                ref_closest_dist = dist_p_to_pose
                
        #No identified pose match current pose
        if p_idx < 0 and save_in_memory:
            pose = tuple(pose)
            self.poses.append(pose)
            p_idx = self.poses.index(pose)
        
        return p_idx
    
    
        
class PoseOdometry(PoseMemory):
    ''' Use pose for Odometry '''

    def __init__(self,possible_actions:dict, influence_radius:float=0.5, robot_dim:float=0.3):
        super(PoseOdometry, self).__init__(possible_actions, influence_radius)
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
    
    def get_sequential_actions(self,last=None, next=None, odom:list=None) -> list:
        """ 
            if we move from pose to pose, we don't have this sequential_actions issue  
            if we give an action, then we need to convert it to pose  
        """
        raise "MODIFY FOR ZONE OF ACTIOn"
        if odom is None:
            odom = self.odometry.copy()
        #did we receive an action? Convert it to pose
        if isinstance(next, (float,int, np.number)):
            
            next_dir = [angle for angle,action in self.possible_actions.items() if action == int(next)][0]
            if next_dir.upper() == 'STAY':
                  return [odom[:2]]
            motion = from_degree_to_point(float(next_dir), pose_dist=self.influence_radius)
            next = (odom[0]+motion[0],odom[1]+motion[1])

        
        return [next]
    
    def pose_in_action_range(self, action:int, pose:list, odom:list=None):
        
        if odom is None:
            odom = self.odometry.copy()
        if self.possible_actions[action] == 'STAY':
            return odom
        
        zone_influence = self.possible_actions[action][:]
        quadri = quadrilater_points(odom=odom, zone_influence=zone_influence,\
                                     influence_radius= self.influence_radius)
        
        return point_in_triangle_with_arc(pose[:2], quadri)

    def pose_transition_from_action(self,action:int, odom:list=None, ideal_dist:float=None):
        """
        Check if we have a pose in twice the area of the agen influence radius. 
        If we do, return this pose
        If we don't, we create a new node at a given ideal distance (if none given it's the influence radius + small value)
        """
        if odom is None:
            odom = self.odometry.copy()
        if self.possible_actions[action] == 'STAY':
            return odom
        
        zone_influence = self.possible_actions[action][:]
        quadri = quadrilater_points(odom=odom, zone_influence=zone_influence,\
                                     influence_radius= self.influence_radius)
        
        #check if pose in zone of action already exist in memory
        for pose in self.poses:
            pt_in_tr = point_in_triangle_with_arc(pose[:2], quadri)
            if pt_in_tr and not list(pose[:2]) == list(odom[:2]):
                return pose[:] #return corresponding pose
        if ideal_dist is None:
            ideal_dist = self.influence_radius + self.influence_radius/5 #just not be exactly on the radius
        average_angle = np.deg2rad(np.mean(zone_influence))
        x= ideal_dist * np.cos(average_angle) + odom[0]
        y= ideal_dist * np.sin(average_angle) + odom[1]
        return [x,y]

    def from_pose_to_action(self, ref_pose:list, next_pose:list) -> int:
        """ 
        Given a pose, we transform it into an orientation by considering current odom
        Then we search for the action closest from that orientation.
        TODO: MODIFY WITH ZONE OF ACTIONS
        """

        raise "MODIFY FOR ZONE OF ACTIOn"
     
        diff = [next[0] - self.odometry[0], next[1]- self.odometry[1]]
        if diff[0] + diff[1] == 0:
            if 'STAY' in self.possible_actions:
                return self.possible_actions['STAY']  
            else:
                raise ValueError('from_pose_to_action: next pose and current pose are \
                                 the same but we cannot apply action STAY (not existing)')   
        vrot = clip_rad_360(np.arctan2(diff[1], diff[0]))
        vrot_deg = np.rad2deg(vrot)
        # Find the closest angle key
        possible_angles = list(self.possible_actions.keys())
        if 'STAY' in possible_angles:
            possible_angles.remove('STAY')
        closest_angle = min(possible_angles, key=lambda k: abs(float(k) - vrot_deg))
        action = self.possible_actions[closest_angle]
        return action
        
