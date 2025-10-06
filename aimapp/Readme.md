# MAP DM NAV modifications

This folder contains our model and subsystems

in `aimapp`
we have the `motion` foler which contains all the motions we have been using (moving straight, using potential field and using NAV2). You can add or replace any elements there by your own

we have the `obs_transf` folder which contains all the observations related actions (single camera, forward Lidar, 3 cameras, 360' camera, 360' Lidar). You can add or reaplce any elements there by your own. 

Our `main.py` get the data from the sensor action and use `observation_match.py` to form memory and compare observation (currently using a SSIM). 



# Topics 

To modify the topics to your own check the `agent_launch.py` file. 

in the remapping we have, on the left side, the topic name expected in the node (because our agent has a namespace "agent"), on the right side the topic name has emited by the sensors.

```bash
('/agent/cmd_vel','/cmd_vel'),
('/agent/odom','agent/odom'),
('/agent/scan','/scan'),
```

The `/agent/odom` should not correspond to the sensor odometry, as the model generates its owm odometry.

# Model

## Model set parameters
Only the `V5.py` of our model is usable with our project. The previous version exists for archives reason.

We presented the parameters of our model in the main Readme.

Here we will present the general structure of teh code to help make it your own.

First of all, we define in the `main` the number of action our agent is going to use. 

This outputs:
```python
possible_actions = 13 # circle / 12 + STAY action
```

will generate  action:[orientation range]
```python
{0: [0, 30], 1: [30, 60], 2: [60, 90], 
3: [90, 120], 4: [120, 150], 5: [150, 180], 
6: [180, 210], 7: [210, 240], 8: [240, 270], 
9: [270, 300], 10: [300, 330], 11: [330, 360], 12: 'STAY'}
```

We can define the max number of steps and the actions we want to follow.
In our code we have:

``` python
policy = [None] * 200
```

`None` means that we do not impose actions, we will infer the best action to apply for 200 steps. After which our system will stop.


The `get_n_pictures()` method is a bit hard-coded, if you modify the observations setting, this should be looked at, as it defines how many turns the agent will do to capture pictures and form a 360' panorama. 

## Model 

We will list the details of our model process to help comprehension. 

### initialisation 
We first initialise the model --> we create our model and form the first adjacent states to move.

### Define next action
Then, we repeat for each action in our initialised policy.

```python
action, action_data = highlevelnav.define_next_objective(action, ob_id, obstacle_dist_per_actions)
```

We run the MCTS given our model possible actions to obtain the best action to apply. 

We then apply that action, if we reach, all good. If we fail to reach, we go back to previous pose (if we also fail that, we continue to next selected action, hoping the robot is not stuck. After too many tries, the model admits defeat). 
If we fail to reach a position, Transition link between those poses is weakened. 

### Update model

```python
obstacle_dist_per_actions, ob_id, ob_match_score = highlevelnav.model_step_process(action)
```

This is where we collect new observations and update the model. We grow the map if required, enforce experimented transitions etc.

### align /agent/odom to model believed odometry

Then the next bit is about aligning odometry to the model believed odometry. 


```python
p_idx = highlevelnav.model.infer_current_most_likely_pose(observations= [ob_id], z_score=10)
```

here we verify where our models believes to be. If no state outvalue the rest we have `p_idx = -1` .
The `z_score` modulates the certainty we need to have over our state to update the odometry. In this situation `10` is extremely high, it's unlikely we will ever have a state with enough certainty to shift the odometry significantly. 



