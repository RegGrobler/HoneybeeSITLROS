# HoneybeeSITLROS
This repo contains the src file for a ROS catkin_workspace. This is the src file located on my PC. Not the one located on the jetson nano used for practical flight tests.


# How to use:
Follow this tutorial on setting up your ROS environment. http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment
This will create your workspace and src (source folder).
I recommend you install ROS Kinetic as this is well supported by PX4 and contents of this repo are based on a kinetic system.

One you have a src folder clone the contents of this repo into the src folder.
in the root directory of your catkin_ws folder run the command: catkin build
This will begin to build all the packages located in the src folder. 
Take note that not all the packages in this repo are required for your application. You can delete packages not needed in your system. By only keeping the required packages for your system you should have alot less dependancy errors. You can google a package to see what it does. 
If any dependancies are missing, google is your friend. You shouldnt have too much difficulty installing missing dependancies. 

Packages that are required: px4_nav_cmd, px4_mc_step_input, mavros, mavlink.

Once you have sucessfully built your environment:

Start a SITL simulation with your UAV or use a default one like iris using: 


Start mavros in a seperate terminal using: 

