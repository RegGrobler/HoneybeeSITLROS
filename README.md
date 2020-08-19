# HoneybeeSITLROS
This repo contains the src file for a ROS catkin_workspace. This is the src file located on my PC. Not the one located on the jetson nano used for practical flight tests.


# How to use:
Follow this tutorial on setting up your ROS environment. http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment
This will create your workspace and src (source folder).
I recommend you install ROS Kinetic as this is well supported by PX4 and contents of this repo are based on a kinetic system.

Once you have a src folder, clone the contents of this repo into the src folder.
in the root directory of your catkin_ws folder run the command: catkin build
This will begin to build all the packages located in the src folder. 
Take note that not all the packages in this repo are required for your application. You can delete packages not needed in for your application. By only keeping the required packages for your system you should have alot less dependancy errors. You can google a package to see what it does. 
If any dependancies are missing, google is your friend. You shouldnt have too much difficulty installing missing dependancies. 

Basic Packages that are required: px4_nav_cmd, px4_mc_step_input, mavros, mavlink.
Once you have sucessfully built your environment:

Start a SITL simulation with your UAV or use a default one like "iris". Open a new terminal in the root directory of PX4 and run: make px4_sitl none_iris
This builds px4 using the "iris" airframe and specifies the simulator to use as "none". This can be replaced with "gazebo" but we are going to launch gazebo manually using ROS. 

To launch gazebo, in a new terminal also in the root directory of PX4 first run: source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
This sets up environment variables for that terminal. Next, in the same terminal run: roslaunch gazebo_ros empty_world.launch world_name:=$(pwd)/Tools/sitl_gazebo/worlds/iris.world
This should open gazebo with a model of iris. QGroundControl should autonamically connect to the sim. From here the UAV can be controlled to a degree.

To define how you want the UAV to fly we can use the ros node "waypoint_scheduler" located in the "px4_nav_cmd" package. To do this:
Start mavros in a seperate terminal using: roslaunch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557"

In another terminal you can start "waypoint_scheduler" using: rosrun px4_nav_cmd waypoint_scheduler.py
The UAV will wait until being put into offboard mode before accepting commands from ROS.
This can be done from QGroundControl.


