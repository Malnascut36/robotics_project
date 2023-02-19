# robotics_project
Robotics Project TUB
1. Start by following the steup.md instructions
2. Once completed, you should be able to run the RViz simulator and the jupyter notebook with the original `PyKDL Kinematics, Jacobian and Transformation Interface.ipynb` file
3. In the same directory where you have he notebook file, which should be `cd catkin_ws/src/rbohand3_kinematics/notebooks/Manipulability`, locatethe Pythn files .py located in thsi repository
4. Now you should be able to open the files from jupyter notebook as well.

# Python files
There are several files for this project

1. `Full_file.py` is the file with all the parts
If you want to run this one, you'll have to go running cell by cell
The first cells will import all the necessary libraries ad files.
Once you arrive to the Free Optimization text cell, run the first cell bellow it.

From here on, the cells are commented detailing the parameters and with some necessary carifications.
It's highly recommended to read the comments, there are some important details explained there.

    1.1 Global Optimization with Direct method cell: runs a free optimization algorithm.
    By running this free optimization, maximum manipulability can be reached, beware there's no initial position set

    Then you get to the Nullspace projection implementation text cell
    1.2 Random positions initializator cell: gives back a random joint configuration position with a constrained position for the palm, equal to the one obtained by ik_solution.solution_4

    1.3 Rotation 90 or 180 degrees cell: Run if you want to rotate the hand position, sometimes it doesn't work properly

    1.4 Global Optimization with Direct method with null space constraing cell: proper implementation of the null space constraint in the DIRECT algorithm

    1.5 Random Position Initializator and Global Optimization with Direct method with null space constraing cell: this cell combnes two of the cells previously mentioned, but it lacks with some modifications that increase the modularity.
    This cell was used and can be used to obtain the plots reflected in the report.
    Some parameters have to be hardcoded, please read the comments in order to make work as you wish.

    Now the Evaluation section starts.
    1.6 Generate and array of q configs from the random position initializator cell

    1.7 Here is stored the list of 50 rand inits used for the benchmark that can be find in the report

    1.8 Benchmark code cell: used to evaluate the performance, you can use it and modify it to plot the required parameters

    1.9 Plotting cell: this cell uses the 1.5 cell to plot the graphics.

Lets check the individual files

2.0 `Initialization.py` this file runs all the required initializations for the algorithm to work.

3.0 `Free_Optimization.py` runs a free optimization algorithm, as cell 1.1.

4.0 `Benchmark.py` integrates the cells 1.2, 1.3, 1.4, 1.6, 1.7 and 1.8, basically will allow you to replicate the benchmark results shown in the report, and the ones you want by modifying some parameters.

5.0 `Plotting_file.py` includes the cells 1.5 and 1.9, allows you to plot the same graphs shown in the report, the parameters have to by modified in more than one location, read carefully the comments on the code.

Thanks for your interest in this project.
Enjoy your implementation.

Glad to help if needed for the implementation and even more to recieve collaboration and ideas for further improvement.
