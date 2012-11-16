README
**********
1. Put this folder (tortuninja) in a path to your ROS_PACKAGE_PATH (i.e. in your fuerte_workspace)
2. Launch the Gazebo simulation:
		roslaunch tortuninja startSimulation.launch
3. Launch the code to make the turtlebot run in circles
		roslaunch tortuninja runCode.launch

And in case you want to run the teleop and no the circle...
4. Go to runCode.launch and... :
     4.1. Uncomment the turtlebot_teleop node
     4.2. Comment the turtlebot_wander node
      4.3. Launch tortuninja runCode.launch
