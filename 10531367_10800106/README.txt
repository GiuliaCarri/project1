Robotics - project 1 - 2020-2021

-Team memebrs:
    Alessandro Boriani 10531367
    Giulia Maria Carri 10800106


-Description:
    This archive contains:
    -This present document for presentation and instructions
    -The package "robotics_hw1" provided, for message MotorSpeed
    -An image of the TF tree
    -The package "project1" containing all needed to start the project, except for the rosbag. It contains:
        -the cfg folder with the integration.cfg file, for dynamic reconfiguration of parameter integration
        -the launch folder with the initialization.launch file. It sets the static parameters and start the two nodes "estimate" and "odometry"
        -the msg folder with the CustomOdom message definition
        -the srv folder with the two services "RiCentra.srv", to set the pose to (0,0,0), and "RiPosiziona.srv", to set the pose to arbitrary pose (x,y,th)
        -the src folder with the two nodes:
            -estimate.cpp: node "estimate" that calculates the linear and angular velocity from the rpm of the motors
            -odometry.cpp: node "odometry" that, from an initial pose, linear and angular velocity (passed by estimate), calculates the odometry of the robot (both with Euler and Runge-Kutta integration). It uses dynamic reconfiguration, services and publish the odometry as tf, nav_msgs/Odometry and custom message.
        -the file CMakeLists.txt with the setting up, imports, dependencies, and executables
        -the file package.xml with the setting up and dependencies


-ROS parameters' name and meaning:
    -Static: we split the parameter for the initial pose of the robot as follows:
        /Initial_x
        /Initial_y
        /Initial_th
    These parameters are initialized in the launch file and is used to set the initial pose of the robot in the class Odometry's constructor.
    -Dynamic: to set dynamically the odometry integration type between Euler and Runge-Kutta, we set the integration.cfg file, so we can use an enum. We keep track of it through the private attribute "integration_type" of class Odometry.
        /odometry/integration



-TF tree's structure:
    The structure is odom --> base_link, where base_link is the frame attached to the robot chassis and odom is a fixed frame representing odometry.
    For further details, please see the attached image.


-Custom message's structure:
    Our custom message is "CustomOdom" and its structure is:

    nav_msgs/Odometry odom
    std_msgs/String method

    as requested


-How to start/use the nodes:
    -To start all the project you could just do: roslaunch project1 initialization.launch
    -To start the nodes separately:
        -To start only the node "estimate" you can do: rosrun project1 estimate
        -To start only the node "odometry" you can do: rosrun project1 odometry
    (Please keep in mind that the "odometry" node needs the "estimate" node to perform correctly)


-More info:
    -The estimate of the baseline was done ......
    -The estimate of the gear ratio was done ......
    -The Optitrak messages and ground truth pose have not been used to estimate the odometry.
