First, clone the repository into the src directory of ROS2 workspace
$ git clone https://github.com/meozkan/MARVer-DT_ScenarioExecution.git

Second, compile the packages
$ colcon build --packages-select rotary_single_rod_description
$ colcon build --packages-select fmu_controller_interfaces
$ colcon build --packages-select rotary_single_rod_description

Third, run the each node
Run Gazebo simulation
$ ros2 launch rotary_single_rod_description sim.launch.py 
Run action server
$ ros2 launch fmu_controller fmu_controller.launch.py 
Call action for test in the command-line
$ ros2 action send_goal /move_to_position fmu_controller_interfaces/action/MoveToPosition "{target_position: 1.0}" --feedback
Now, you have to see that the rod turns and stop at approximately 1.0 rad
