import Initilization
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

