# map_dm_nav


## Usage 

## Standard usage

#### Setup
```
source install/setup.bash (in ROS2 Workspace)
colcon build --packages-select map_dm_nav
```
#### launching the world:
```
ros2 launch map_dm_nav warehouse_launch.py
```
#### Launching the turtlebot3:
```
ros2 launch map_dm_nav spawn_turtle_launch.py x:=0.0 y:=0.0
```
with x,y the spawn pose in gazebo (odom will be 0,0,orientation)

#### launching the agent model:
```
ros2 launch map_dm_nav agent_launch.py
```

The data will be saved in the starting repository under `tests/test_id`

Another alternative to run everything is to open 3 terminals and run :
```
bash setup.bash 1  --> will run world
bash setup.bash 2  --> will run agent
bash setup.bash 3  --> will run model
```

if nothing happens, try re-starting either the model or the agent. 
Ros topic connection can be loose(?)
If the model seems to be slow after a full rotation, it has to do with data saving. Some can be removed either in main.py or in visualisation_tools.py


### Docker usage

First treminal

```
docker build -t high_nav_rw .
./launch_docker.sh 
bash src/map_dm_nav/setup_env.bash 1
```

and in terminal 2/3
```
docker exec -it high_nav_rw bash
bash src/map_dm_nav/setup_env.bash 2 (in T2) 3 (inT3)
```
if nothing happens, try re-starting either the model or the agent. 
Ros topic connection can be loose(?)

NOTE: You might have to modify agent_launch.py to remove rosbag from it, as the camera filming the agent from above is not provided in the default gazebo environment. 
If you are interested in adding it manually read the section modifying related packages.



#### Parameters to tune

You can tune a few parameters: 

**The possible directions** the agent can take (they are expected to be in degree), it is strongly advised to keep the action 'STAY' to avoid any unpredicted errors (never tried without in the last runs). Whatever the angle, it is expected as a string containing an integer.
Example: `possible_actions = {'0':0, '22': 1, '45':2, '67':3,'90':4, '112':5, '135':6, '157':7,'180':8, '202':9,'225':10, '247':11,'270':12, '292':13, '315':14, '337':15, 'STAY':16}`

This dictionary will have to be manually written in the main.py file, before initialisation of the model.

**Policy**, which policy the agent should follow as a list in the main.py. If None the agent will determine itself where to go, else it will reach for the given pose. Poses are expected reachable with only 1 action, please don't jump over several poses at once (e.g don't go from `(0,0)` to `(2,0)`.)

Policy examples: `[None,None, None,None, None]` -->the agent will decide where to move over 5 steps,

`[(0,1), (1,1), (2,2), (4,3)]` --> the agent will move to those poses, this particular sequence suppose that you have the possible_actions defined above. 

**Odometry**, While i recommend to use `PoseOdometry`, you can choose to use `HotEncodedActionOdometry`, however it might fail with the latest modifications -not tested-. You have to make the change in the init of the model `V4_2.py`. You use hotencoded actions instead of poses to reach. Not been tested for more than cardinal directions motions.

**Step distance**, how far in m is one step, `self.dist_th` in the init of the class in `main.py` has to be modified. 
Be careful as this code use a potential field to move, and the agent may not be able to reach a given pose even if it's accessible yet to close from an obstacle. With self.`dist_th=1`m we are able to reach every point (as attraction strong), in order to be safe, to be considered accessible, lidar must say that 0.5m behind that position is also free. That value is statically implemented in `add_mandatory_free_dist` -> `main.py` and you could reduce it if you reduce the dist_th value. You have to play around. If you don't want the Potential field, it can be replaced by a simple `go_straight` action saved in the same folder. 

You can play with the potential field independently by spawning the robot and use:
```
ros2 run map_dm_nav potential_field_action.py --ros-args -r cmd_vel:=/gazebo/cmd_vel -r scan:=/gazebo/scan
ros2 run map_dm_nav potential_field_client.py --x 1 --y -1
```

**lookahead**, how far will the agent predict actions consequences. The larger the number the longer the computation. There are other factors you could play with in the model (determining how many policies will be generated), currently it is set on the minimum number of policies with simple paths always going in one general direction only. No going back on path or changing general direction. While the lookahead can be modified in `main.py initialisation()`, the policy generation type is to be modified IN the model `v4_2.py init_policies()`

You can also play around with the speed of the robot by modifying (`cmd_linear_max` and `cmd_angular_max` in both actions)
If desired, you can also play with the starting number of state and observations, or how many dimensions you want to have by modifying `num_obs=2, num_states=2, dim=2` in `main.py`. I wouldn't recommend it though.

Finally, if the process is too slow, i would recommend commenting the data saving part `save_data_process` as it is the longest part to compute. 


## Requirements

ros2 foxy 
world: [aws-robomaker-small-warehouse-world](https://github.com/aws-robotics/aws-robomaker-small-warehouse-world/tree/foxy-devel) 

robot: [turtlebot3](https://github.com/ROBOTIS-GIT/turtlebot3_simulations/tree/foxy-devel) 

Modified turtle3_gazebo/launch/spawn_turtlebot3.launch.py to have namespace as parameters:
```
# Launch configuration variables specific to simulation
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')
    namespace = LaunchConfiguration('namespace', default='')

    start_gazebo_ros_spawner_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', TURTLEBOT3_MODEL,
            '-file', urdf_path,
            '-x', x_pose,
            '-y', y_pose,
            '-z', '0.01',
            '-robot_namespace', namespace,
        ],
        output='screen',
    )
```
Thus in order to be able to have a reset odometry where the agent spawn in env -(0,0) is where agent spawns, not the gazebo world (0,0).-


## Content
Nodes:
- potential field
- get_panorama
- HighLevelNav_model

Actions: 
- potential field action 
- the panorama action.


The two actions are so in order to be able to receive data from the subscriber as it run (services seem to pause them).
In Humble it would be possible to stop and re-start nodes, but that is for another iteration. 

In map_dm_nav main.py you have the ros2 model starting all sub_processes.

you have the ros2 model in main.py, the model in the folder model, the observation treatments in the folder obs_transf and motion actions in motion folder. 

You can see below a rapid overview of the repo showing the ros2 nodes, what their input/outputs are and where is the code.

![overall_repo_view](git_fig/repo_overall_view.png)



## Modifying related packages

If you want to add a top view of the map and the agent, you can add a fisheye to `aws-robomaker-small-warehouse-world`
the fisheye folder is available in the folder `turtlebot3_modification` of our repository.

copy paste this fisheye folder in the `models` folder of `aws-robomaker-small-warehouse-world`.
Then open the .world you want to run and paste:

```
<model name="fisheye">
    <include>
        <uri>model://fisheye</uri>
    </include>
    <pose frame="">0 0 19 0 1.593185 0</pose>
</model>
```
modify the position as you see fit.
With this, you will be able to rosbag the top view image of the robot moving. Be wary that it takes ~5MiB/sec of memory.

<!-- #### Resetting odometry for tests purposes -->
<!-- 
Real robot preparation:

To have that some modif to turtlebot3_node were made in odometry.cpp/hpp
see:
https://github.com/paolorugg/my_turtlebot3_node/tree/main/tbt3_node/turtlebot3_node


``` -->
