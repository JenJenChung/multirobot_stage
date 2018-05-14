# multirobot_stage
Top level package for simulating multiple robots in stage

<h2>Package dependencies:</h2>

Available through apt-get:
<ul>
<li> <a href="http://wiki.ros.org/gmapping?distro=indigo">gmapping</a> </li>
<li> <a href="http://wiki.ros.org/move_base">move_base</a> </li>
<li> <a href="http://wiki.ros.org/stage_ros">stage_ros</a> </li>
<li> <a href="http://wiki.ros.org/rviz">rviz</a> </li>
<li> <a href="http://wiki.ros.org/amcl">amcl</a> </li>
<li> <a href="http://wiki.ros.org/map_server">map_server</a> </li>
<li> <a href="http://wiki.ros.org/costmap_2d">costmap_2d</a> </li>
</ul>

Available on github:
<ul>
<li> <a href="https://github.com/JenJenChung/stagebot_2dnav">stagebot_2dnav</a> </li>
<li> <a href="https://github.com/JenJenChung/nav_bundle">nav_bundle</a> </li>
<li> <a href="https://github.com/JenJenChung/simple_navigation_goals">simple_navigation_goals</a> </li>
</ul>

<h2>Usage:</h2>

From the command line:

Clone the package into your catkin_ws/src folder, then run:

<pre><code>catkin_make</code></pre>

To run the default simulation with two robots, run:

<pre><code>rosrun multirobot_stage run-multi-robot-sim</code></pre>

Command waypoints to the robots by sending Twist messages to /robot_0/map_goal and /robot_1/map_goal, e.g.

<pre><code>rostopic pub /robot_1/map_goal geometry_msgs/Twist '[20, 20, 0]' '[0, 0, 0]' -1</pre></code>

# multirobot_learning
Multi-agent reinforcement learning (MARL) for exploration and mapping based on multirobot_stage. Developed as part of the course Perception and Learning for Robotics at ETH Zürich in Spring 2018, by Benjamin Hahn and Julian von der Goltz, supervised by Jen Jen Chung.

<h2>Package dependencies:</h2>
multirobot_map_merge [Github](https://github.com/hrnr/m-explore) and as a [ROS Package](http://wiki.ros.org/multirobot_map_merge)

<h2>Nodes:</h2>
In addition to the nodes provided by multirobot_stage there are:

* multirobot_learning:
    The node which manages the learning part of the package. It starts and stops episodes and executes the learning algorithm, in this case neuroevolution.
    
* comms_node:
    The nodes manages the communication of maps and odometries between the robots. It can be used to simulate limited communications range.  
    
* rewards_node:
    continuosly calculates the reward (area of the explored map) for the learning
    
* action_node (one per robot):
    takes a state (projection of frontiers and obstacles and position of other robots) and a policy (a neural net) and calculates the new waypoint goal which is passed to move_base. 
    
* multirobot_map_merg (one per robot)
    merges the internal map of each robot with the maps of the other robots. 
    
<h2>Commands:</h2>
To run the exploration instance, run:
<pre><code>rosrun multirobot_stage run-multi-robot-exploration *nRob* *headless*</code></pre>

To run the learning instance, run:
<pre><code>roslaunch multirobot_stage multirobot-learning.launch -nRob:=*nRob* -headless:=*true/false*</code></pre>
