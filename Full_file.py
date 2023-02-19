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


# In[17]:


#Global Optimization with Direct method
#by running this free optimization, maximum manipulability can be reached
#beware there's no initial position set

from scipy.optimize import direct, Bounds

#Objective function buildup
def manipulability(q):
    #"Manipulability function"
    panda_kinematics.setJointPosition(q)
    J = panda_kinematics.getEEJacobian()
    JT = np.transpose(J)
    JJT = np.matmul(J,JT)
    '''poses_fk = fk_rbo_hand(index_airmass = [0,0],
                        middle_airmass = [0,0],
                           ring_airmass = [0,0],
                           little_airmass = [0,0],
                           thumb_airmass = [0,0.0,0.0,0.0],
                           palm_airmass = [0.],
                           in_palm_frame = 1,
                           scaled_masses = 1,
                           panda_joint_angles=q)'''
    return -math.sqrt(np.linalg.det(JJT))
#The bounds parameter constrains q values to stay inside joint limits
bounds=Bounds([-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973], [2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973])
q_solution=direct(manipulability, bounds, maxiter=1000, locally_biased=True)
print('w=', q_solution.fun) #DIRECT algorithm returns q_solution.fun, which is the optimal result from the objective function
print(q_solution.x) #DIRECT algorithm returns q_solution.x, which is the q configuration of the optimal result from the objective function

poses_fk = fk_rbo_hand(index_airmass = [0,0],
                        middle_airmass = [0,0],
                           ring_airmass = [0,0],
                           little_airmass = [0,0],
                           thumb_airmass = [0,0.0,0.0,0.0],
                           palm_airmass = [0.],
                           in_palm_frame = 1,
                           scaled_masses = 1,
                           panda_joint_angles=q_solution.x)


# # Nullspace projection implementation

# In[18]:


#Random positions initializator
#This code will give back a random joint configuration position
#with a constrained position for the palm, equal to the one obtained by ik_solution.solution_4

def random_init():
    bounds = []
    import scipy
    from scipy.optimize import direct, Bounds
    from random import randrange, uniform
    from franka_analytical_ik.srv import ik_request

    robot = URDF.from_parameter_server()
    kdl_kin = KDLKinematics(robot, "panda_link0", "panda_link8")

    panda_kinematics.setJointPosition(ik_solution.solution_4)
    q_init = ik_solution.solution_4
    #Obtaining pose from the initial ik_solution.solution_4 joint configuration
    pose_init = kdl_kin.forward(q_init) # forward kinematics (returns homogeneous 4x4 numpy.mat)
    
    joint_limits=True
    while joint_limits==True: #we ensure that the random position generated is inside the joint limits, if not, repeat

        limit_counter=0
        #generation of random q values inside the joint limits
        q_rand =[[0]*1]*7
        overall_bounds=Bounds([-2.89, -1.76, -2.89, -3.07, -2.89, -0.01, -2.89], [2.89, 1.76, 2.89, -0.06, 2.89, 3.75, 2.89])

        for h in range(0,7):
            q_rand[h]=uniform(overall_bounds.lb[h], overall_bounds.ub[h]) #pick a random value inside joint limits

        pose_rand = kdl_kin.forward(q_rand) # get this random pose

        pose_new=pose_init #new pose has same value as pose from ik_solution.solution_4 joint

        for i in range(0, 3):
            pose_new[0:3,3]=pose_rand[0:3,3] #set position values from the rand t the new pose, keeping initial rotation values

        q_new=kdl_kin.inverse(pose_new) #with inverse kinematics get new q configuration from new pose

        try:
            q_new.item
            print("Success")
            print("\n")

            for h in range(0,7): #check if new q config is inside limits
                if (q_new[h] >= overall_bounds.lb[h]) and (q_new[h] <= overall_bounds.ub[h]):
                    limit_counter=limit_counter+1

            if limit_counter==7: #if so, exit the loop
                joint_limits=False

            else:
                print("Joints positions out of boundaries")
                print("\n")

        except AttributeError:
            print("Error")
            print("\n")

    q_new=kdl_kin.inverse(pose_new)

    poses_fk = fk_rbo_hand(index_airmass = [0,0],
                            middle_airmass = [0,0],
                               ring_airmass = [0,0],
                               little_airmass = [0,0],
                               thumb_airmass = [0,0.0,0.0,0.0],
                               palm_airmass = [0.],
                               in_palm_frame = 1,
                               scaled_masses = 1,   
                           panda_joint_angles=q_new) #send new q config to simulator
    return q_new

print(random_init()) #print new q config


# In[78]:


#Rotation 90 or 180 degrees
#Run if you want to rotate the hand position, sometimes it doesn't work properly
#Requires more work to fix it

robot = URDF.from_parameter_server()
kdl_kin = KDLKinematics(robot, "panda_link0", "panda_link8")

print("pose_new: ")
print(pose_new)
print("\n")

rot_mat_x_90=np.matrix([[1, 0, 0, 0],[0, 0, -1, 0],[0, 1, 0, 0],[0, 0, 0, 1]]) #rotation matrix around x axis 90º
rot_mat_y_90=np.matrix([[0, 0, 1, 0],[0, 1, 0, 0],[-1, 0, 0, 0],[0, 0, 0, 1]]) #rotation matrix around y axis 90º
rot_mat_z_90=np.matrix([[0, -1, 0, 0],[1, 0, 0, 0],[0, 0, 1, 0],[0, 0, 0, 1]]) #rotation matrix around z axis 90º

rot_mat_x_180=np.matrix([[1, 0, 0, 0],[0, -1, 0, 0],[0, 0, -1, 0],[0, 0, 0, 1]]) #rotation matrix around x axis 180º
rot_mat_y_180=np.matrix([[-1, 0, 0, 0],[0, 1, 0, 0],[0, 0, -1, 0],[0, 0, 0, 1]]) #rotation matrix around y axis 180º
rot_mat_z_180=np.matrix([[-1, 0, 0, 0],[0, -1, 0, 0],[0, 0, 1, 0],[0, 0, 0, 1]]) #rotation matrix around z axis 180º

#Select rot_mat_? variable to input in the following np.matmul to select rotation axis
rotation=np.matmul(pose_new, rot_mat_z_180)
print("rot_mat: ")
print(rotation)
print("\n")

q_rot=kdl_kin.inverse(rotation) #compute new q config
print("q rot: ")
print(q_rot)
print("\n")

if q_rot is not None:
    poses_fk = fk_rbo_hand(index_airmass = [0,0],
                            middle_airmass = [0,0],
                               ring_airmass = [0,0],
                               little_airmass = [0,0],
                               thumb_airmass = [0,0.0,0.0,0.0],
                               palm_airmass = [0.],
                               in_palm_frame = 1,
                               scaled_masses = 1,   
                           panda_joint_angles=q_new) #send new configuration to the simulator
    
else: #if the rotation has not been properly computed, due to any error, print message
    print("Error, this rotation can't be computed in this configuration. Try another rotation axis or re-run the random position initializator")


# In[19]:


#Global Optimization with Direct method with null space constraing
#implemented joint limits penalization
#values required as input: q_new = initial q config
#q_rate = value for the increment of q in the optimization iteration (usually 0.1)
#k = joint limit penalty k factor
#conv = convergence value, condition to end the optimization loop


def manipulabilityNullspace(q_new, q_rate, k, conv): 
    import scipy
    from scipy.optimize import direct, Bounds
    
    robot = URDF.from_parameter_server()
    kdl_kin = KDLKinematics(robot, "panda_link0", "panda_link8")

    q_init = q_new #activate if you want to optimize with palm facing upwards
    #q_init = q_rot #activate if you rotated the random initialized position, modify the input then also
    q1 = q_init

    J = panda_kinematics.getEEJacobian()[3:5] # choose dimensions to fix ([0:3] translation, [3:6] rotation)
    Jpinv = np.linalg.pinv(J)
    JpinvJ = np.matmul(Jpinv,J)
    Np = (np.identity(7)-JpinvJ) #computing Null Space from initial position

    q =[[0]*1]*7
    
    #storing joint limits
    overall_bounds=Bounds([-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973], [2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973], keep_feasible=True)
    
    w_last = -0.01 #value to check the covergence
    
    for h in range(0,200): #times the optimizer will run until convergence achieved of iterations complete

        J = panda_kinematics.getEEJacobian()[3:5] #chose the same dimensions to fix as above
        Jpinv = np.linalg.pinv(J)
        JpinvJ = np.matmul(Jpinv,J)
        Np = (np.identity(7)-JpinvJ)

        #Objective function buildup
        def manipulability(qDelta):
            #"Manipulability function"
            q=q_init+np.matmul(Np,qDelta)
            panda_kinematics.setJointPosition(q)
            J = panda_kinematics.getEEJacobian()
            JT = np.transpose(J)
            JJT = np.matmul(J,JT)

            #Joints limits penalization
            product=1
            for j in range(0,7):
                numerator=(q[j]-overall_bounds.lb[j])*(overall_bounds.ub[j]-q[j])
                denominator=(overall_bounds.ub[j]-overall_bounds.lb[j])**2
                current_product=numerator/denominator
                product=product*current_product

            Penal=1-np.exp(-k*product) #computes joint limit penalization (P)

            '''poses_fk = fk_rbo_hand(index_airmass = [0,0],
                                middle_airmass = [0,0],
                                   ring_airmass = [0,0],
                                   little_airmass = [0,0],
                                   thumb_airmass = [0,0.0,0.0,0.0],
                                   palm_airmass = [0.],
                                   in_palm_frame = 1,
                                   scaled_masses = 1,
                                   panda_joint_angles=q)'''
            return -math.sqrt(np.linalg.det(JJT))*Penal #returns maniulability (M = w * P)
        
        #this bounds are now the above introduced selected q_rate, the optimizer will run maxiter times
        #diferent q configurations where increment range is +-q_rate
        bounds=Bounds([-q_rate, -q_rate, -q_rate, -q_rate, -q_rate, -q_rate, -q_rate], [q_rate, q_rate, q_rate, q_rate, q_rate, q_rate, q_rate], keep_feasible=True)
        #hyperparameters for DIRECT algorithm (function to otimize, rectangle upper and lower limts, maximum iterations,...)
        q_solution=direct(manipulability, bounds, maxiter=10, locally_biased=True, f_min=-0.13167)
        qDelta=q_solution.x #once the best solution inside the range of q_rate is found, store it as qDelta
        q=q_init+np.matmul(Np,qDelta) #compute new q config updating the previous q config with the new qDelta
        #qDelta is projected into the null space to fulfill the dimension constraints
        q_init=q #update initial q and resulting q, to go into the next iteration with the updated q config
        
        w_secondlast = w_last #update second last manip value
        w_last = q_solution.fun #update last manip value
        
        if (-w_last - w_secondlast < -conv): #check if convergence is fulfilled or not
            break
            
    poses_fk = fk_rbo_hand(index_airmass = [0,0],
                            middle_airmass = [0,0],
                               ring_airmass = [0,0],
                               little_airmass = [0,0],
                               thumb_airmass = [0,0.0,0.0,0.0],
                               palm_airmass = [0.],
                               in_palm_frame = 1,
                               scaled_masses = 1,   
                           panda_joint_angles=q)
    
    return q_solution.fun, kdl_kin.forward(q) #returned values will be the optimized manip value and the pose for the q config

print(manipulabilityNullspace(random_init(), q_rate = 0.1, k=1000000, conv = 0.0001))
    


# In[21]:


#This file is has together in it the random position initializator and the null space constraint
#This code was used for printing some plots, but it's not as developed as the ones where this functions are split
#therefore, to make it run, several values have to be hardcoded, some parameters can't be sent here
#It's recomended to check the files where the functions are split, to understand some parameters / functions

#This file will start by providing a random initial position, with the palm facing upwards
#Then it will proceed to apply the optimization algorithm with DIRECT

#Random positions initializator
bounds = []
import scipy
from scipy.optimize import direct, Bounds
from random import randrange, uniform
from franka_analytical_ik.srv import ik_request

w_list=[]

for k in range(4): #number of times the whole process will run, number of random positions initialized, each one will be optimized
    #if this value is change, you'll have to adapt the ploting code in accordance to the number of rand inits that you want to have
    robot = URDF.from_parameter_server()
    kdl_kin = KDLKinematics(robot, "panda_link0", "panda_link8")

    panda_kinematics.setJointPosition(ik_solution.solution_4)
    q_init = ik_solution.solution_4
    pose_init = kdl_kin.forward(q_init) # forward kinematics (returns homogeneous 4x4 numpy.mat)

    joint_limits=True
    while joint_limits==True:

        limit_counter=0
        #generation of random q values inside the joint limits
        q_rand =[[0]*1]*7
        overall_bounds=Bounds([-2.89, -1.76, -2.89, -3.07, -2.89, -0.01, -2.89], [2.89, 1.76, 2.89, -0.06, 2.89, 3.75, 2.89])

        for h in range(0,7):
            q_rand[h]=uniform(overall_bounds.lb[h], overall_bounds.ub[h])

        pose_rand = kdl_kin.forward(q_rand) # forward kinematics (returns homogeneous 4x4 numpy.mat)

        pose_new=pose_init

        for i in range(0, 3):
            pose_new[0:3,3]=pose_rand[0:3,3]

        q_new=kdl_kin.inverse(pose_new)

        try:
            q_new.item
            print("Success")
            print("\n")

            for h in range(0,7):
                if (q_new[h] >= overall_bounds.lb[h]) and (q_new[h] <= overall_bounds.ub[h]):
                    limit_counter=limit_counter+1

            if limit_counter==7:
                joint_limits=False

            else:
                print("Joints positions out of boundaries")
                print("\n")

        except AttributeError:
            print("Error")
            print("\n")

    q_new=kdl_kin.inverse(pose_new)

    poses_fk = fk_rbo_hand(index_airmass = [0,0],
                            middle_airmass = [0,0],
                               ring_airmass = [0,0],
                               little_airmass = [0,0],
                               thumb_airmass = [0,0.0,0.0,0.0],
                               palm_airmass = [0.],
                               in_palm_frame = 1,
                               scaled_masses = 1,   
                           panda_joint_angles=q_new)

    #Global Optimization with Direct method
    #implemented joint limits penalization
    
    w_print = []
    
    #IMPORTANT!!! The following loop, the value among which the loop will iterate, are the ones that have to be
    #selected in accordance to what one desires to see the different runs for in the plots
    for i in (0.1, 0.05, 0.01, 0.001): #insert the desired values in the loop
        #Value to optimize
        q_init = q_new #activate if you want to optimize with palm facing upwards
        #q_init = q_rot #activate if you rotated the random initialized position
        q1 = q_init

        J = panda_kinematics.getEEJacobian()[3:6] #Fix rotation
        #J = panda_kinematics.getEEJacobian()[0:3] #Fix translation
        Jpinv = np.linalg.pinv(J)
        JpinvJ = np.matmul(Jpinv,J)
        Np = (np.identity(7)-JpinvJ)

        q =[[0]*1]*7

        k=100000000 #IMPORTANT : in case you want to iterate through the k values, set k=i and set the desired values in the loop range
        overall_bounds=Bounds([-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973], [2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973], keep_feasible=True)
        w_print_nested = []
        for z in range(0,200): 
        
            J = panda_kinematics.getEEJacobian()[3:6] #Fix rotation
            #J = panda_kinematics.getEEJacobian()[0:3] #Fix translation
            Jpinv = np.linalg.pinv(J)
            JpinvJ = np.matmul(Jpinv,J)
            Np = (np.identity(7)-JpinvJ)

            #Objective function buildup
            def manipulability(qDelta):
                #"Manipulability function"
                q=q_init+np.matmul(Np,qDelta)
                panda_kinematics.setJointPosition(q)
                J = panda_kinematics.getEEJacobian()
                JT = np.transpose(J)
                JJT = np.matmul(J,JT)

                #Joints limits penalization
                product=1
                for j in range(0,7):
                    numerator=(q[j]-overall_bounds.lb[j])*(overall_bounds.ub[j]-q[j])
                    denominator=(overall_bounds.ub[j]-overall_bounds.lb[j])**2
                    current_product=numerator/denominator
                    product=product*current_product

                Penal=1-np.exp(-k*product)

                '''poses_fk = fk_rbo_hand(index_airmass = [0,0],
                                    middle_airmass = [0,0],
                                       ring_airmass = [0,0],
                                       little_airmass = [0,0],
                                       thumb_airmass = [0,0.0,0.0,0.0],
                                       palm_airmass = [0.],panda_kinematics.setJointPosition(q)
                                       in_palm_frame = 1,
                                       scaled_masses = 1,
                                       panda_joint_angles=q)'''
                return -math.sqrt(np.linalg.det(JJT))*Penal
            #the bounds are set as i, as in this case these are the values that will be modified for the plots
            bounds=Bounds([-i, -i, -i, -i, -i, -i, -i], [i, i, i, i, i, i, i], keep_feasible=True)
            #if you want to plot for several values of maxiter, set maxiter=i and set the desired values in the loop range
            q_solution=direct(manipulability, bounds, maxiter=10, locally_biased=True)
            qDelta=q_solution.x
            q=q_init+np.matmul(Np,qDelta)
            q_init=q
            w_print_nested.append(-q_solution.fun)
        w_print.append(w_print_nested)
        poses_fk = fk_rbo_hand(index_airmass = [0,0],
                                middle_airmass = [0,0],
                                   ring_airmass = [0,0],
                                   little_airmass = [0,0],
                                   thumb_airmass = [0,0.0,0.0,0.0],
                                   palm_airmass = [0.],
                                   in_palm_frame = 1,
                                   scaled_masses = 1,
                               #panda_joint_angles=q_solution.x)    
                               panda_joint_angles=q)
    w_list.append(w_print)
print("\n")        
print("w_list")        
print(w_list)


# # Evaluation

# In[114]:


#This block will generate and array of q configs from the random position initializator
#with the size set in te for loop

rand_inits = [] 
for i in range(0,50):
    rand_inits.append(random_init())
    print(rand_inits)
    
print(rand_inits)


# In[13]:


#list of 50 rand inits used for the benchmark

bench_q_list = [([-0.66579611, -0.69103315,  2.08672152, -1.95843764, -1.40066813,
        0.45165682, -1.54464071]), ([-0.17749154,  1.37591613, -0.49027116, -1.81571959, -1.78481024,
        1.22836656,  0.41170082]), ([ 0.31976259, -0.22844208,  0.5563295 , -2.70533152, -1.51569737,
        0.57915717, -0.77816797]), ([-0.01545136,  0.67159119,  0.06054268, -2.35163255, -1.69418883,
        0.73834096,  0.06163387]), ([ 0.36272048,  0.86591445,  0.47763373, -1.32017233, -2.66587963,
        0.60918723, -1.57205449]), ([-2.67475968,  1.4564258 ,  0.2880491 , -0.81126216,  2.21712196,
        1.48796303,  0.88667423]), ([ 0.79934447,  0.78169628,  0.42949618, -1.38430286, -2.86879581,
        0.3674066 , -2.0955234 ]), ([-0.26253388,  0.22890602, -0.07043997, -2.23005819, -2.33368897,
        1.16302687, -0.45577531]), ([-0.29290993,  0.41000231, -0.2278887 , -2.84772653, -1.966086  ,
        0.80257395,  0.43056837]), ([ 2.5405737 ,  0.92834202, -2.71444691, -2.03241698, -1.93160033,
        1.99484302, -1.47425825]), ([ 0.45439571, -0.17430196, -0.99887813, -2.15614046, -2.64285093,
        1.52215483, -0.66275443]), ([ 2.84802613,  1.20036492,  2.78548089, -1.00860999, -0.67511501,
        2.53078706,  2.84759646]), ([-2.15960278, -1.13812222,  2.10424103, -0.61158393, -1.53512065,
        1.32972248, -1.71074917]), ([ 0.90905772,  0.5676241 ,  0.03052073, -2.26531808, -0.93938308,
        0.53915868, -0.10678625]), ([ 0.70590751,  0.97772088,  0.55645931, -1.59509076, -2.33548641,
        0.01974743, -1.39632919]), ([-0.06655134, -1.63659092,  0.69205434, -2.64346155, -1.61265196,
        1.71882906, -1.81387732]), ([-0.54005557, -0.26235932,  1.94173304, -2.50814365, -0.87347322,
        0.39518357, -0.63956126]), ([ 1.03026913,  1.59397019, -2.65690107, -2.21262241,  0.10802565,
        3.1346188 , -2.27584502]), ([-2.86529668, -0.26267406, -2.84488514, -1.19071353, -2.54821981,
        1.31096581, -1.81508569]), ([ 0.09934692, -0.43398966, -0.01256874, -2.44908225, -2.31283043,
        1.21243872, -1.0553826 ]), ([ 0.14613208, -0.5310847 ,  0.46615969, -2.7182151 , -1.86455767,
        0.895851  , -1.04959396]), ([ 0.27032353,  0.88252702,  0.32760284, -1.85720281, -1.74172473,
        0.44628582, -0.44447909]), ([ 1.17072191, -1.521212  ,  2.6809015 , -2.45667801, -0.31226422,
        0.15978866, -2.58191468]), ([-0.03452897, -0.58982763,  1.09578764, -2.60057191, -1.48070772,
        0.78345864, -1.18411466]), ([-2.26047379, -0.27311975,  0.05417318, -1.20023271, -1.02928153,
        3.64922106, -1.41523294]), ([ 2.8733891 ,  0.24870227, -0.07408589, -1.33728694,  2.42193012,
        1.30981974,  2.06675681]), ([ 0.60865776, -1.61045803,  2.43041849, -2.03352965, -0.37735326,
        0.10672662, -2.1631851 ]), ([ 0.92458539,  1.09701083, -0.3059522 , -2.2767055 , -0.86008648,
        1.11135596,  0.20893289]), ([ 2.85116887, -0.88517515, -2.66075158, -0.31011113, -2.70279215,
        1.96558838, -1.51547484]), ([-0.2313452 ,  0.83514899,  0.44174693, -2.30856258, -1.58752305,
        0.4456775 ,  0.16540773]), ([ 0.45520614,  1.38600475,  0.58825568, -1.4004542 , -1.71296939,
        0.1052409 , -0.36296822]), ([-2.62570971,  1.05838679, -2.86866927, -1.29444965, -2.21486678,
        2.03331835, -2.82645328]), ([-2.04173204,  0.90534706, -1.2659903 , -2.49770207, -2.84312233,
        0.27392095,  2.1131232 ]), ([-1.03265081,  0.15533867, -0.06337047, -2.91775097, -2.7093328 ,
        0.83552329,  0.21024544]), ([-0.55871752, -1.08826881, -0.30892092, -2.93356083, -2.82397311,
        1.77419322, -0.65843874]), ([ 1.10281654e+00,  2.96671884e-01, -2.79418952e-04, -1.43491830e+00,
       -2.64822881e+00,  7.66769476e-01, -2.16576696e+00]), ([-1.39724392, -0.78635033, -0.02310238, -1.44409901, -0.65489257,
        3.0084881 , -2.35968892]), ([ 1.02782489, -0.44052788, -0.42979588, -2.33660165, -2.52846151,
        0.8515918 , -1.57476431]), ([ 0.12463027,  0.63678701,  0.12449477, -1.7169233 , -2.19145145,
        0.87918182, -0.86207215]), ([-0.29774471,  1.15992333,  0.49044055, -1.96216373, -1.99337255,
        0.35051787, -0.10194112]), ([-0.25379561, -0.76857532, -0.29959491, -2.10524319, -2.5944675 ,
        2.09153725, -1.07702681]), ([-1.01258121, -0.48552366,  0.98266113, -1.94368357, -1.98169201,
        1.61328327, -1.17062656]), ([-0.2308123 ,  0.71019553,  0.09356837, -2.77338971, -1.3650847 ,
        0.71095402,  0.74656252]), ([-0.08570323, -0.61812674, -0.59600411, -1.89242156, -2.7621685 ,
        2.18036547, -0.94727771]), ([-0.28045455, -1.05797152, -0.00704139, -2.38664961, -2.35051423,
        1.95809223, -1.28279563]), ([-0.38331246,  0.42188444, -0.30643385, -1.91461684, -2.46497556,
        1.4543371 , -0.30368305]), ([ 0.00604438,  0.40810116,  1.20219557, -2.29704646, -1.48415863,
       -0.00494078, -0.68997927]), ([-0.84410464, -0.72537393, -0.2443885 , -1.12827014, -0.74898421,
        2.80500095, -2.67775349]), ([-0.60520207,  1.11754829, -1.07250839, -0.97437243, -2.11676127,
        2.22370725,  0.08612538]), ([ 0.09231936, -1.61499901,  0.8135256 , -2.42600274, -1.57389323,
        1.55243752, -2.12766381])]

print(bench_q_list[0])


# In[84]:


#Benchmark code

import time
from scipy.spatial.transform import Rotation as R

timeSpans =[] #shape = for loop iterations
final_manip_values = [] #shape = for loop iterations
rotZ = [] #shape = for loop iterations

for h in range(0,50):
    start_time = time.time()
    r = R.from_matrix(kdl_kin.forward(bench_q_list[h])[:3,:3])
    rotvec = r.as_rotvec()
    run = (manipulabilityNullspace(q_new = bench_q_list[h], q_rate = 0.05, k = 1000000, conv = 0.0001)) 
    final_manip_values.append(run[0])
    pose_mat=run[1]
    rr = R.from_matrix(pose_mat[:3,:3])
    newRotVec = rr.as_rotvec()
    timeSpans.append(time.time() - start_time)
    rotZ.append(abs(rotvec-newRotVec))
    
average_time = sum(timeSpans)/ len(timeSpans)
average_manip_obtained = sum(final_manip_values)/ len(final_manip_values)
average_deviation = sum(rotZ)/len(rotZ)  #in radians

print('HPs : q_rate = 0.05, k=1000000, conv=0.0001')
print("---------------------------------------------------------------------")
print("Average Time : ",average_time, "sec")
print("Average Max Manipulability : ", -average_manip_obtained, "( ", round((average_manip_obtained/min(final_manip_values))*100,2), " % of max. manip)")
print("Average Deviation around x (in radians) : ",average_deviation[0])
print("Average Deviation around y (in radians) : ",average_deviation[1])


# In[24]:


#Plotting file
#Requires using the file were random position initializator and null space implementation are located
#It will require to hardcode the parameters that one desires to change in the above mentioned code

import matplotlib.pyplot as plt
from matplotlib import cm

x = np.arange(0,200)
#Constraints tto be changed and plotted acording to
deltaQ = [0.1, 0.05, 0.01, 0.001] 
#Kvalues = [100000000, 1000000, 10000, 100]
#Iterations = [10000, 1000, 100, 10]
titles = [r"Random $q_{init_1}$", r"Random $q_{init_2}$", r"Random $q_{init_3}$", r"Random $q_{init_4}$"]
fig, axs = plt.subplots(2, 2, figsize=(14, 8), constrained_layout=True)
for ax, i in zip (axs.flat, range(4)):
    for j, val in enumerate(cm.jet(np.linspace(0,1,4))):
        ax.plot(x,w_list[i][j], color=val, label=f'{deltaQ[j]}')
    
    #ax.set_ylim([0.11, 0.128]) #activate for zooming in
    ax.legend()
    ax.set_title('%s' % titles[i], fontsize=16)
    ax.set_ylabel('Manipulability (w)')
    ax.set_xlabel('# Iterations')
    ax.grid()
    fig.savefig("Plot_1.png", dpi=500)

