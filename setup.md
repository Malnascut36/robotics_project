1. Ubuntu Version 20.04
2. Install Anaconda for Python3 - https://docs.anaconda.com/anaconda/install/linux/
3. Create a Python3.8 environment with anaconda and activate it - https://docs.conda.io/projects/conda/en/latest/user-guide/tasks/manage-environments.html
4. Install the following python packages: https://git.tu-berlin.de/-/snippets/429
5. Install ROS Noetic (ros-noetic-desktop-full) - http://wiki.ros.org/noetic/Installation/Ubuntu
6. Setup a catkin_ws - http://wiki.ros.org/catkin/Tutorials/create_a_workspace
7. Clone the following repositories into `catkin_ws/src/`:  
    7.1 Franka Panda Description - https://git.tu-berlin.de/rbo/robotics/franka_panda_description  
    7.2 Franka Panda Analytical Inverse Kinematics - https://git.tu-berlin.de/rbo/robotics/franka-panda-analytical-inverse-kinematics  
    7.3 RH3 Description - https://git.tu-berlin.de/rbo/robotics/rh3-description  
    7.4 RH3 Kinematics - https://git.tu-berlin.de/rbo/robotics/rh3-kinematics
    7.5 RH3 PyBullet Simulation - https://git.tu-berlin.de/rbo/robotics/rh3-pybullet-simulation
    7.6 Hrl-Kdl - https://git.tu-berlin.de/rbo/robotics/hrl-kdl
8. Run `catkin build` from `/catkin_ws`.
9. `source catkin_ws/devel/setup.bash`
10. Install PyKDL and Kinematic/Dynamic Utils functions: https://git.tu-berlin.de/-/snippets/430
11. Launch all required nodes and services via `roslaunch rbohand3_kinematics start_ik_fk_services_rviz.launch load_panda_arm:=1 panda_ik_service:=1`
12. `cd catkin_ws/src/rbohand3_kinematics/notebooks/Manipulability` and run `jupyter notebook`
13. Open `PyKDL Kinematics, Jacobian and Transformation Interface.ipynb` and get started.
