Desk software: https://robot.franka.de

Both robots have the same login credentials:

username	password
Franka2	frankalincoln
Franka ROS is installed from source from the following link Franka ROS Installation Or follow the instructions below

sudo apt remove "*libfranka*"
sudo apt install build-essential cmake git libpoco-dev libeigen3-dev
git clone --recursive https://github.com/frankaemika/libfranka
cd libfranka
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
cmake --build .
Install ROS Kinetic. Create catkin folder

a). cd /path/to/desired/folder

b). mkdir -p catkin_ws/src

c). cd catkin_ws

d). source /opt/ros/kinetic/setup.sh

e). catkin_init_workspace src

Clone the franka-ros repository from github


*  /..../catkin_ws

*  $ git clone --recursive https://github.com/frankaemika/franka_ros src/franka_ros

*  rosdep install --from-paths src --ignore-src --rosdistro kinetic -y --skip-keys libfranka

*  catkin_make -DCMAKE_BUILD_TYPE=Release -DFranka_DIR:PATH=/path/to/libfranka/build

*  source devel/setup.sh
Note: A generic linux kernel is not sufficient to run the real robot. A real-time kernel is to be set up for which the instructions can be found here

Franka Collision Detection and Limits
The Franka force and torque limits can be set in the Franka_controller using the function setCollisionBehavior. Documentation for this function can be found here

to update the limits, use the following command in a terminal:

    rosed franka_control franka_control_node.cpp
The following values define the robots force and torque limits. By default, both the upper and lower limits are set to be the same, making the contact and collision levels the same.

...
```
    robot.setCollisionBehavior(
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},  // lower_torque_thresholds_acceleration //Sariah : first 4 are to check joint contact
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},  // upper_torque_thresholds_acceleration
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},  // lower_torque_thresholds_nominal
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},  // upper_torque_thresholds_nominal
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},        // lower_force_thresholds_acceleration  //Sariah: last 4 are to check cartesian contact 
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},        // upper_force_thresholds_acceleration
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},        // lower_force_thresholds_nominal
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});       // upper_force_thresholds_nominal
``` 


...
There are two thresholds defined by this function:

contact
This is detected when the force or torque applied is between the lower and upper limits. The robot is still able to move and operate while within this limit.
When the franka_controller is running, contact can be seen, as an array of Boolean's, in the franka_msgs/FrankaState message under joint_contact and cartesian_contact, published by default on the topic /franka_state_controller/franka_states. No other errors are thrown.
collision:
A reflex exception is thrown when the force or torque applied is above the upper limits.
When the franka_controller is running, a collision can be seen, as an array of Boolean's, in the franka_msgs/FrankaState message under joint_collision and cartesian_collision, published by default on the topic /franka_state_controller/franka_states.
In addition, an error is displayed in the above message under current_errors(franka_msgs/Errors) as a joint_reflex and cartesian_reflex
To view this message in more detail, you can run the following from a terminal:

    rosmsg show franka_msgs/FrankaState 
Franka Joint State Update Rate
In the file ~/ws_moveit/src/franka_ros/franka_control/config





**Moveit**
Ash Babu edited this page on Dec 18, 2018 · 5 revisions


MoveIt Installation
MoveIt is installed from source. This would install some other planners like CHOMP along with the sampling based planners provided in OMPL

MoveIt can be installed from here Or follow the instructions for ROS Kinetic below

```
rosdep update
sudo apt-get update
sudo apt-get dist-upgrade
sudo apt-get install python-wstool python-catkin-tools clang-format-3.8
Inside your catkin folder

wstool init src
wstool merge -t src https://raw.githubusercontent.com/ros-planning/moveit/kinetic-devel/moveit.rosinstall
wstool update -t src
rosdep install -y --from-paths src --ignore-src --rosdistro kinetic
catkin config --extend /opt/ros/kinetic --cmake-args -DCMAKE_BUILD_TYPE=Release
catkin_make
```

A lot of tutorials on how to use franka robot with MoveIt can be found here The README.md file here describes how to move the robot to a position from the clicked point on an image obtained from Kinect. To move the franka robot using pose-goal, joint values, cartesian path etc can be found here To learn the above tutorial, follow the steps below

roslaunch panda_moveit_config demo.launch for simulated robot or roslaunch franka_control franka_control.launch robot_ip:=172.16.0.2 load_gripper:=True/False for real robot
python move_group_python_interface_tutorial.py and follow the instructions on the screen  # Sariah note: inside "ws_moveit/src/moveit_tutorials/doc/"
Very Imp Note: The real robot has obstacles (aluminium frame) which are not defined in the simulations. Hence, there is a chance that the robot comes in collision with it. Always have the emergency stop ready to be operated

Converting point-clouds to Octomap for obstacle avoidance
The detailed tutorial is available here Only the highlights are reproduced below:

Create a file named “sensors_kinect_pointcloud.yaml” and save it in the config folder in the panda_moveit_config folder (/catkin_ws/src/franka_ros/panda_moveit_config/config) with the following lines
sensors:
```
  - sensor_plugin: occupancy_map_monitor/PointCloudOctomapUpdater   (#Sariah: find the plugin in moveit-ros-perception package under /opt directory, where PointCloudOctomapUpdater is a class of namespace occupancy_map_monitor)
    point_cloud_topic: /ceiling_camera/qhd/points
    max_range: 1.5  # This value sets the max range the camera can see
    point_subsample: 1
    padding_offset: 0.1
    padding_scale: 1.0
    max_update_rate: 1.0
    filtered_cloud_topic: filtered_cloud
Update the panda_moveit_sensor_manager.launch.xml file in the “launch” directory of your panda_moveit_config directory with the following sensor information.
<rosparam command="load" file="$(find panda_moveit_config)/config/sensors_kinect_pointcloud.yaml" />
```


Also configure the Octomap by adding the following lines into the sensor_manager.launch.xml under panda_moveit_config/launch:
```
<param name="octomap_frame" type="string" value="odom_combined" />
<param name="octomap_resolution" type="double" value="0.05" />
<param name="max_range" type="double" value="5.0" />
```





**Probabilistic Movement Primitive Library**

A Probabilistic Movement Primitive (ProMP) is a probabilistic generative model used to model movement, it is typically used in the robotics community to learn movements from a human demonstrator (or teacher) and replicate those movements in a robotic platform.

This repository contains the implementation in Python and C++ of the Probabilistic Movement Primitive framework as described in this paper. Typically, the operations we want from a ProMP are:

Learning a ProMP from several human demonstrations. Typically, we consider we learn from trajectories in joint space.
Conditioning in joint space. For example, force the movement to start in the current position of the robot.
Conditioning in task space. For example, conditioning a table tennis strike movement to hit the ball in a certain position in Cartesian coordinates.
Controlling a robot to track a ProMP
We provide code for the first three operations. We assume that the learning is done in Python, and only implemented the adaptation operators in C++ (They are also provided in Python). We also provide code for the following operations:

Compute the likelihood of a given trajectory for a given ProMP
Sample trajectories from a ProMP
Save and load ProMPs
Code Examples
Please refer to the example folder in the repository promp on github (simple_example.py). We just show a simple example that illustrates how to use the API in general.

A very simple example
The following example loads a dataset of strike trajectories from a file called "strike_mov.npz" and trains a ProMP with the given trajectories. The example file is provided as part of the repository. Subsequently, the script draws samples from the learned ProMP and a ProMP conditioned to start at a particular location.

```
import robpy.full_promp as promp
import robpy.utils as utils
import numpy as np
from matplotlib import pyplot as plt



#1) Take the first 10 striking movements from a file with recorded demonstrations
with open('strike_mov.npz','r') as f:
    data = np.load(f)
    time = data['time'][0:10]
    Q = data['Q'][0:10]

#2) Create a ProMP with basis functions: 3 RBFs with scale 0.25 and 
#   centers 0.25, 0.5 and 0.75. Use also polynomial basis functions of 
#   degree one (constant and linear term)
full_basis = {
        'conf': [
                {"type": "sqexp", "nparams": 4, "conf": {"dim": 3}},
                {"type": "poly", "nparams": 0, "conf": {"order": 1}}
            ],
        'params': [np.log(0.25),0.25,0.5,0.75]
        }
robot_promp = promp.FullProMP(basis=full_basis)

#3) Train ProMP with NIW prior on the covariance matrix (as described in the paper)

dof = 7
dim_basis_fun = 5
inv_whis_mean = lambda v, Sigma: utils.make_block_diag(Sigma, dof)
prior_Sigma_w = {'v':dim_basis_fun*dof, 'mean_cov_mle': inv_whis_mean}
train_summary = robot_promp.train(time, q=Q, max_iter=10, prior_Sigma_w=prior_Sigma_w,
        print_inner_lb=True)


#4) Plot some samples from the learned ProMP and conditioned ProMP

n_samples = 5 # Number of samples to draw
plot_dof = 3 # Degree of freedom to plot
sample_time = [np.linspace(0,1,200) for i in range(n_samples)]

#4.1) Make some samples from the unconditioned ProMP
promp_samples = robot_promp.sample(sample_time)

#4.2) Condition the ProMP to start at q_cond_init and draw samples from it
q_cond_init = [1.54, 0.44, 0.15, 1.65, 0.01, -0.09, -1.23]
robot_promp.condition(t=0, T=1, q=q_cond_init, ignore_Sy=False)
cond_samples = robot_promp.sample(sample_time)
Using task space conditioning
In order to use task space conditioning you need to implement the forward kinematics of your own robot. You simply need to implement a class with a method called "position_and_jac(q)" that given a joint space configuration q produces a tuple (x, jac, ori) representing respectively the cartesian position, the jacobian and the orientation. We provide an example of the implementation of the kinematics of a Barrett WAM arm. The following example shows how to condition a ProMP in task space using the Barrett forward kinematics implementation.

import robpy.kinematics.forward as fwd

# Compute the prior distribution in joint space at the desired time
time_cartesian = 0.9
mean_marg_w, cov_marg_w = robot_promp.marginal_w(np.array([0.0,time_cartesian,1.0]), q=True)
prior_mu_q = mean_marg_w[1]
prior_Sigma_q = cov_marg_w[1]

# Compute the posterior distribution in joint space after conditioning in task space
fwd_kin = fwd.BarrettKinematics()
prob_inv_kin = promp.ProbInvKinematics(fwd_kin)

mu_cartesian = np.array([-0.62, -0.44, -0.34])
Sigma_cartesian = 0.02**2*np.eye(3) 

mu_q, Sigma_q = prob_inv_kin.inv_kin(mu_theta=prior_mu_q, sig_theta=prior_Sigma_q,
        mu_x = mu_cartesian, sig_x = Sigma_cartesian)


# Finally, condition in joint space using the posterior joint space distribution

robot_promp.condition(t=time_cartesian, T=1.0, q=mu_q, Sigma_q=Sigma_q, ignore_Sy=False)
task_cond_samples = robot_promp.sample(sample_time)
```
Publications
The implementations provided in this repository are based on the following publications:


*  Adaptation and Robust Learning of Probabilistic Movement Primitives IEEE TRO https://arxiv.org/pdf/1808.10648.pdf

*  Using probabilistic movement primitives for striking movements, IEEE RAS International Conference on Humanoid Robots, 2016

Please refer to these papers to understand our implementation, get general information about probabilistic movement primitives and see the evaluation of the implemented methods in real robotic platforms.
