# ENPM661: Project-3 Phase-2
**Part-1**: Implementation and visualization of **A-Star Algorithm** for Turtlebot Action Sets.  
**Part-2**: Gazebo Simulation of a Turtlebot in a maze. Trajectory planned though A-star.

## 1. Team:

|     UID     |  Directory ID  |            Name            |
|    :---:    | :------------: |           :----:           |
|  120425554  |     rraj27     |         Rishie Raj         |
|  120305085  |      uthu      |  Uthappa Suresh Madettira  |

## 2. Description:
The goal of the project is to implement A-Star Algorithm using the action sets of a Turtlebot and finding the optimal path between two points in a maze.
## 3. Contents

 - `Part01` : This folder contains the source code for the first part which involves the implementation of A-Star based on user-input parameters and its visualization for a Turtlebot.

 - `Part02` : This folder contains the ROS package for Gazebo simulation in the *competition* environment. The trajectory planning is performed using A-Star.

 - `README.md` : markdown file containing instructions and details.

## 4. How to Use:

To start, please download the .zip file and extract it to retrive the files or clone the GitHub repo: https://github.com/rishieraj/enpm661-weighted_astar-turtlebot.git .

**Part-1**:  
Please follow the following steps to implement the code in a local system:

   - In the `Part01` folder, the .py file can be executed directly. Upon execution, the user will be prompted to enter the following:

      - Clearance between robot and obstacles.
      - Start point and orientation $(X,\;Y, \;\theta)$
      - Goal point $(X,\;Y)$
      - 2 RPMs for the robot $(RPM1,\;RPM2)$
   
   **Note:** The coordinates need to be added as per the origin described in the project documentation.

   - Once the start and goal points are added, the program will explore the nodes based on the Total Cost = Cost2Come + Cost2Goal.

   - Once the goal is reached, the program prints *Goal Reached!* along with the computation time. It then goes on to perform the visualization of the node exploration and optimal path between start and goal.

**Part-2**:  
Please follow the following steps to implement the simulation in a local system:

   - In the `Part02` folder, you will find the ROS package that contains the *competition_world* launch files along with the python script `astar.py` to implement the trajectory planning and turtlebot maneuvers in Gazebo. The following steps are to be followed:

      - Create a workspace and add the package retrieved from the above folder to the `src` folder in the workspace.
      - Now, open the workspace in a terminal and build it using the following code:

         > colcon build

      - Then, source the environment using:

         > source install/setup.bash

      - Launch the competition environment in Gazebo:

         > ros2 launch turtlebot3_project3 competition_world.launch.py 

      - Now open a new window and source the environment once again

         > source install/setup.bash

      - Run the python node for A-Star implementation

         > ros2 run turtlebot3_project3 astar.py

      - Upon running the above code, the user will be asked to enter the goal position coordinates.
   
   **Note:** The coordinates need to be added as per the origin described in the project documentation.

   - Once the goal points are added, the program will explore the nodes based on the Total Cost = Cost2Come + Cost2Goal.

   - Once the goal is reached, the program prints *Goal Reached!* along with the computation time. It then starts to publish the command velocities to the `cmd_vel` topic to run the turtlebot along the trajectory.

## 5. Simulation Video Examples/Links:
The links to the simulations along with the inputs for these examples have been given below:

 - **Part-1**: https://youtu.be/fQ7WWwSlZ_8  
   The inputs for the above visualization are as follows:
      
      - Clearance: 10
      - Start Coordinates and Orientation: 0 0 0
      - Goal Coordinates: 5250 0
      - Wheel RPMs: 40 80

 - **Part-2**: https://youtu.be/uaofchW3SHw  
   The inputs for the above visualization are as follows:

      - Goal Coordinates: 5250 0

## 6. Libraries/Dependencies Used:
The following libraries have been used in the program:

 - Numpy ( `numpy` ): used for array operations and mathematical calculations.

 - Queue ( `queue.PriorityQueue` ): used for handling nodes in Dijkstra Algorithm due to easy popping of lowest order element.

 - Time ( `time` ): used to calculate computation time.

 - OpenCV ( `opencv` ): used for display and visualization purposes.

 - Math ( `math` ): used for mathematical operations like euclidian distance calculation.

 - ROS2 Library for Python ( `rclpy` ): used to create nodes and publishers in ROS2 using python scripts.

 - Geomertry Messages ( `geometry_msgs.msg` ): used for creating and publishing `Twist` messages.