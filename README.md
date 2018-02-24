# model_predictive_control

## Dependencies
### OS/ROS
Ubuntu 16.04 with [ROS Kinetic Kame](http://wiki.ros.org/kinetic/Installation/Ubuntu)

Ubuntu 14.04 with [ROS Indigo Igloo](http://wiki.ros.org/indigo/Installation/Ubuntu)

### Python (2.7.6 or greater)
1. numpy (1.8.2 or greater)
2. matplotlib (1.3.1 or greater)
3. scipy (0.19.0 or greater

In your terminal, go to the folder named 'scripts'

Python Script main.py Additional Parameters:
* plot (default="none", "all") [shorthand -p]
	* none 	= no plots
	* all	= plots end-effector position
* traj (default="pnp","circle") [shorthand -t]
	* pnp		= follows a minimum jerk pick and place trajectory
	* circle	= follows a planar circular trajectory

Example Call: python main.py --plot all -t circle