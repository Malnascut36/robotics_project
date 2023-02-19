#!/usr/bin/env python
# coding: utf-8

# In[5]:


from urdf_parser_py.urdf import URDF #unified robot description format
from pykdl_utils.kdl_parser import kdl_tree_from_urdf_model
import PyKDL as _kdl #kinematics and dynamics library
import rospy as _rospy #quick interface for ROS topics, services and parameters
import numpy as np
import tf2_ros 
import rospy

rospy.init_node("manipulability") #initialize the ROS node for the process. Only one node per process


# ## Panda Kinematics and Jacobian Class

# In[6]:


class PandaURDFModel():

    def __init__(self):
        self.baseLinkName = 'panda_link0'
        self.eeLinkName = 'panda_link8'
        try:
            robot = URDF.from_parameter_server()
        except ConnectionRefusedError:
            print('Load Robot into parameter server first before accessing it!')
            raise(Exception)
            
        self.kdltree = kdl_tree_from_urdf_model(robot)
        self.ee_chain = self.kdltree.getChain(self.baseLinkName, self.eeLinkName) 
        self.fk_ee = _kdl.ChainFkSolverPos_recursive(self.ee_chain)
        self.jointposition = _kdl.JntArray(7)
        self.eeFrame = _kdl.Frame()
        self.jac_ee = _kdl.ChainJntToJacSolver(self.ee_chain)
        self.jacobian = _kdl.Jacobian(7) # A jacobian instance is constructed 
        #see .cpp : http://docs.ros.org/en/indigo/api/orocos_kdl/html/jacobian_8cpp_source.html#l00033

        #dynamics: (needs masses added to urdf!)
        self.grav_vector = _kdl.Vector(0., 0., -9.81)  
        self.dynParam = _kdl.ChainDynParam(self.ee_chain, self.grav_vector)
        self.inertiaMatrix = _kdl.JntSpaceInertiaMatrix(7)

        
    def setJointPosition(self, jointPosition):
        for i in range(7):
            self.jointposition[i] = jointPosition[i]
#        self.jointposition[7] = jointPosition[7] # I don't get what use the gripper would have here

        
    def getEELocation(self):
        self.fk_ee.JntToCart(self.jointposition, self.eeFrame)
        print(self.eeFrame.p)
        print(self.eeFrame.M)
        return np.array(self.eeFrame.p), np.array(self.eeFrame.M)
    
    def getEEJacobian(self):
        self.jac_ee.JntToJac(self.jointposition, self.jacobian)
        
        # numpy array constructor does not work for kdl stuff.
        # There is likely to be a smarter way of doing this

        np_jac = np.zeros([6,7])
        for row in range(6):
            for col in range(7):
                np_jac[row][col] = self.jacobian[row,col]
        return np_jac
        
    def getInertiaMatrix(self):
        self.dynParam.JntToMass(self.jointposition, self.inertiaMatrix)
        return self.inertiaMatrix


# In[7]:


panda_kinematics = PandaURDFModel() #plain instance creation
# Set initial joint position
joint_pos_initial = [0, 0 ,0 ,0 ,0 ,0 ,0 ] #radians
panda_kinematics.setJointPosition(joint_pos_initial)


# In[8]:


print("Position vector (x,y,z) of origin of EEframe & \n")
print("Orientation matrix R of origin of EEframe : \n")
panda_kinematics.getEELocation()


# ## Transformation Interface

# In[9]:


tf_buffer = tf2_ros.Buffer()
tf_listener = tf2_ros.TransformListener(tf_buffer)


# In[10]:


from scipy.spatial.transform import Rotation as R

def get_transformation(from_frame, to_frame):
    if tf_buffer._frameExists(from_frame) and tf_buffer._frameExists(to_frame):
        transform_base_EE = tf_buffer.lookup_transform(from_frame, to_frame, rospy.Time(0))
        # Convert quaternion into roation matrix
        quat_base_EE = transform_base_EE.transform.rotation
        r = R.from_quat([quat_base_EE.x, quat_base_EE.y, quat_base_EE.z, quat_base_EE.w])
        rot_matrix = r.as_matrix()
        # Create 4x4 transformation matrix
        trans_matrix = np.zeros((4,4))
        trans_matrix[:3, :3] = rot_matrix
        trans_matrix[0, 3] = transform_base_EE.transform.translation.x
        trans_matrix[1, 3] = transform_base_EE.transform.translation.y
        trans_matrix[2, 3] = transform_base_EE.transform.translation.z
        trans_matrix[3, 3]=1
        # Return transformation matrix
        return trans_matrix
    else:
        raise Exception('Frame(s) are not available! - Check TF tree.')
        
# Get transformation from Base to EE-Frame
t_base_EE = get_transformation("panda_link0", "panda_link8")
print("Homogeneous Transform from Base to EE-Frame : \n")
print(t_base_EE) 
print("\n")
# Get transformation from EE-Frame to Palm-Frame
t_EE_palm = get_transformation("panda_link8", "palm")
print("Homogeneous Transform from EE-Frame to to Palm-Frame : \n")
print(t_EE_palm)


# ## Franka Inverse Kinematics Service

# In[11]:


from franka_analytical_ik.srv import ik_request
rospy.wait_for_service('franka_ik_service')
ik_franka = rospy.ServiceProxy('franka_ik_service', ik_request)


# In[12]:


q_actual = [-2, -0.044 + 0.3, 0.031, -2.427 + 0.4, -2.047, 0.845, -0.71]
#ik_solver offering up to 4 IK solutions 
ik_solution = ik_franka(
    desired_transform = t_base_EE.transpose().reshape(16).tolist(),
    q7=-0.83,
    q_actual=q_actual
)


# In[13]:


# Evaluate IK solution in terms of accuracy
panda_kinematics.setJointPosition(ik_solution.solution_2)
panda_kinematics.getEELocation()
print('Desired Pose:')
print(t_base_EE)


# ## RH3/Panda Forward Kinematics Service

# In[14]:


from rbohand3_kinematics.srv import fk2

# Connect to forward kinematics service
rospy.wait_for_service('forward_kinematics')
fk_rbo_hand = rospy.ServiceProxy('forward_kinematics', fk2)


# In[15]:


# Evaluate IK results visually
poses_fk = fk_rbo_hand(index_airmass = [0,0],
                        middle_airmass = [0,0],
                           ring_airmass = [0,0],
                           little_airmass = [0,0],
                           thumb_airmass = [0,0.0,0.0,0.0],
                           palm_airmass = [0.],
                           in_palm_frame = 1,
                           scaled_masses = 1,
                           panda_joint_angles=ik_solution.solution_4)
print("solution 1 : " , ik_solution.solution_1)
print("solution 2 : " , ik_solution.solution_2)
print("solution 3 : " , ik_solution.solution_3)
print("solution 4 : " , ik_solution.solution_4)


# In order to change the robot pose in the simulator:
# 
# ***Change t_base_EE (this expresses the desired pose of the end-effector)***
# 
# ***Change the panda_joint_angles which is the configuration vector as a result of the IK_solver***

# # Unconstrained Optimization with Nullspace projection, Sampling Based Optimization

# # Free Optimization

# In[16]:


from scipy.linalg import null_space
from differentiable_robot_model.robot_model import DifferentiableRobotModel
from urdf_parser_py.urdf import URDF
from pykdl_utils.kdl_kinematics import KDLKinematics
import torch
import math
import random as rnd
#IMPORTANT
#select the path were the "modified_panda_arm.urdf" is located in yor computer
urdf_path = "/home/malnascut/catkin_ws/src/franka_panda_description/robots/custom_panda_arm/modified_panda_arm.urdf"
#modify the above path with the proper  path
robot = DifferentiableRobotModel(urdf_path)