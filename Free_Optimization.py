import Initilization

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
