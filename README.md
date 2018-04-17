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
