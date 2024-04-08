#!/usr/bin/env python3

# GitHub Repository: https://github.com/rishieraj/enpm661-weighted_astar-turtlebot.git

############################################ Import the required libraries ##########################################################
import numpy as np
from queue import PriorityQueue
import time
import cv2
import math as m
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

################################ Define the lists required for computation of the optimal path #######################################
open_list = PriorityQueue()
close_list = set()

RED = (0, 0, 255)
GREEN = (0, 255, 0)
BLUE = (255, 0, 0)
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)

SCALE_FACTOR = 10
WHEEL_RADIUS = 33
WHEEL_BASE = 287
RADIUS = 220

##################################################### ROS2 publisher class #########################################################
class MinimalPublisher(Node):

    def __init__(self, speed_comands):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        timer_period = 0.5  # seconds
        self.i = 0
        self.command = speed_comands
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        if self.i < len(self.command):
            lin_speed, ang_speed = self.command[self.i]
            msg = Twist()
            msg.linear.x = lin_speed
            msg.angular.z = ang_speed
            self.publisher_.publish(msg)
            self.get_logger().info('Publishing Speed Command: Linear: %s, Angular: %s' % (lin_speed, ang_speed))
            self.i += 1

        else:
            msg = Twist()
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.publisher_.publish(msg)
            self.get_logger().info('Stopping the Turtlebot.')
            self.destroy_node()
            

############################################# Define the Configuration Space ########################################################
def config_space():

    # declaring Configuration Space as an array
    c_space = 255 * np.ones((round(2000 / SCALE_FACTOR), round(6000 / SCALE_FACTOR), 3))

    # Boundary + Clearance
    c_space[1:round((clearance + RADIUS + 1) / SCALE_FACTOR), :, :] = BLUE
    c_space[0, :, :] = BLACK

    c_space[round((2000 - clearance - RADIUS - 1) / SCALE_FACTOR):round(1999 / SCALE_FACTOR) - 1, :, :] = BLUE
    c_space[round(1999 / SCALE_FACTOR) - 1, :, :] = BLACK

    c_space[1:round(1999 / SCALE_FACTOR) - 2, 1:round((clearance + RADIUS + 1) / SCALE_FACTOR), :] = BLUE
    c_space[:, 0, :] = BLACK

    c_space[1:round(1999 / SCALE_FACTOR) - 2, round((6000 - clearance - RADIUS - 1) / SCALE_FACTOR):round(5999 / SCALE_FACTOR) - 1, :] = BLUE
    c_space[:, round(5999 / SCALE_FACTOR) - 1, :] = BLACK

    # Rectangles + Clearance
    for i in range(1, round(6000 / SCALE_FACTOR) - 1):
        for j in range(1, round(2000 / SCALE_FACTOR) - 1):
            # Clearance
            a1 = i - round((1500 - clearance - RADIUS - 1) / SCALE_FACTOR)
            a2 = j - round(2000 / SCALE_FACTOR)
            a3 = i - round((1750 + clearance + RADIUS + 1) / SCALE_FACTOR)
            a4 = j - round((1000 - clearance - RADIUS - 1) / SCALE_FACTOR)

            b1 = i - round((2500 - clearance - RADIUS - 1) / SCALE_FACTOR)
            b2 = j - round((1000 + clearance + RADIUS + 1) / SCALE_FACTOR)
            b3 = i - round((2750 + clearance + RADIUS + 1) / SCALE_FACTOR)
            b4 = j

            # initializing pixel values for clearances
            if (a1 > 0) and (a2 < 0) and (a3 < 0) and (a4 > 0):
                c_space[j, i] = BLUE

            if (b1 > 0) and (b2 < 0) and (b3 < 0) and (b4 > 0):
                c_space[j, i] = BLUE

            # Rectangles
            c1 = i - round(1500 / SCALE_FACTOR)
            c2 = j - round(2000 / SCALE_FACTOR)
            c3 = i - round(1750 / SCALE_FACTOR)
            c4 = j - round(1000 / SCALE_FACTOR)

            d1 = i - round(2500 / SCALE_FACTOR)
            d2 = j - round(1000 / SCALE_FACTOR)
            d3 = i - round(2750 / SCALE_FACTOR)
            d4 = j

            # initializing pixel values for obstacles
            if (c1 > 0) and (c2 < 0) and (c3 < 0) and (c4 > 0):
                c_space[j, i] = BLACK

            if (d1 > 0) and (d2 < 0) and (d3 < 0) and (d4 > 0):
                c_space[j, i] = BLACK
            
            # Circle + Clearance           
            # Clearance
            k1 = (i - round(4200 / SCALE_FACTOR))**2 + (j - round(1200 / SCALE_FACTOR))**2 - round((600 + clearance + RADIUS) / SCALE_FACTOR)**2

            # Circle
            l1 = (i - round(4200 / SCALE_FACTOR))**2 + (j - round(1200 / SCALE_FACTOR))**2 - round(600 / SCALE_FACTOR)**2

            # initializing pixel values for clearances
            if (k1 < 0):
                c_space[j, i] = BLUE

            # initializing pixel values for obstacles
            if (l1 < 0):
                c_space[j, i] = BLACK

    # c_space = cv2.resize(c_space, (600, 200)).astype(np.uint8)
    c_space = cv2.flip(c_space, 0).astype(np.uint8)

    return c_space

#################################### Calculating destination and C2C based on RPMs ##############################################
def path_generator(current_node, rpm1, rpm2):
    t = 0
    dt = 0.01
    x = current_node[2][0]
    y = current_node[2][1]
    theta = current_node[2][2]

    while t < 0.5:
        x += ((WHEEL_RADIUS / 2) * (2 * np.pi) * (rpm1 + rpm2) * np.cos(np.radians(theta)) * dt) / SCALE_FACTOR
        y += ((WHEEL_RADIUS / 2) * (2 * np.pi) * (rpm1 + rpm2) * np.sin(np.radians(theta)) * dt) / SCALE_FACTOR
        theta += ((WHEEL_RADIUS / WHEEL_BASE) * (2 * np.pi) * (rpm1 - rpm2) * dt) * (180 / np.pi)

        if np.array_equal(obs_space[round(y), round(x)], BLUE) or np.array_equal(obs_space[round(y), round(x)], BLACK):
            return None, None, None

        t += dt
    
    return round(x), round(y), (theta % 360)

#################################################### Defining Proximity Check function ##################################################
def is_close(node, close_list):
    for closed_node in close_list:
        if np.linalg.norm(np.array(node[:2]) - np.array(closed_node[:2])) < 4:
            return True
    return False

################################################# Implementation of the A star algorithm ################################################
def astar(start, goal, rpm):
    # structure of node: (cost_to_come, (x cordinate, y cordinate))
    start_cost = round(np.sqrt((start[0] - goal[0])**2 + (start[1] - goal[1])**2))
    start_node = (start_cost, 0, start)
    open_list.put(start_node)
    # creating a dict to track parent child relations for backtracking
    parent_map = {}
    rpm_map = {}
    parent_map[start] = None

    RPM1 = rpm[0]
    RPM2 = rpm[1]

    print("Searching for the goal...")

    while not open_list.empty():
        current_node = open_list.get()

        if current_node[2] in close_list:
            continue

        close_list.add(current_node[2])

        if m.dist((current_node[2][0], current_node[2][1]), (goal[0], goal[1])) <= (15 / SCALE_FACTOR):
            print('Goal Reached!')
            # calling backtracking function after goal node is reached
            path, wheel_rpms = back_tracking(parent_map, rpm_map, start, current_node[2])
            return path, wheel_rpms, parent_map
        
        next_nodes = []

        for rpms in [[0, RPM1], [RPM1, 0], [RPM1, RPM1], [0, RPM2], [RPM2, 0], [RPM2, RPM2], [RPM1, RPM2], [RPM2, RPM1]]:
            x, y, theta = path_generator(current_node, rpms[0], rpms[1])

            # checking validity of action
            if x is None:
                continue
            
            # updating C2C and C2G
            c2g = 1.2 * m.dist((x, y), (goal[0], goal[1]))
            c2c = current_node[1] + m.dist((current_node[2][0], current_node[2][1]), (x, y))
            cost =  c2g + c2c
            new_node = [cost, c2c, (x, y, theta), rpms[0], rpms[1]]

            next_nodes.append(new_node)

        for next_node in next_nodes:
            if not is_close(next_node[2], close_list):
                if next_node[2] not in [x[2] for x in open_list.queue]:
                    parent_map[next_node[2]] = current_node[2]
                    rpm_map[next_node[2]] = (next_node[3], next_node[4])
                    open_list.put(tuple(next_node))
                
                else:
                    for node in open_list.queue:
                        if node[2] == next_node[2] and node[0] > next_node[0]:
                            open_list.queue.remove(node)
                            parent_map[next_node[2]] = current_node[2]
                            open_list.put(tuple(next_node))
    
    else:
        print("Goal could not be reached!")
        exit()

########################################## Backtracking function based on parent map ##################################################
def back_tracking(parent_map, rpm_map, start, goal):
    path = []
    wheel_rpms = []

    current_node = goal
    current_rpms = (0, 0)

    while current_node != start:
        path.append(current_node)
        wheel_rpms.append(current_rpms)

        current_rpms = rpm_map[current_node]
        current_node = parent_map[current_node]


    path.append(start)

    path.reverse()
    wheel_rpms.reverse()

    return path, wheel_rpms

##################################
def convert_speed(rpms_list):
    speed_list = []
    for rpms in rpms_list:
        linear_speed = (WHEEL_RADIUS / 2) * (2 * np.pi) * (rpms[0] + rpms[1]) * 0.001
        angular_speed = (WHEEL_RADIUS / WHEEL_BASE) * (2 * np.pi) * (rpms[1] - rpms[0])

        speed_list.append((linear_speed, angular_speed))
    
    return speed_list

################################################## Defining function to navigate Turtlebot #########################################
def turtlebot(speeds, args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher(speeds)

    rclpy.spin(minimal_publisher)

    # minimal_publisher.destroy_node()
    rclpy.shutdown()

########################################## Function for asking user input of start and goal nodes #######################################
def user_input(obs_space):
    while True:
        user_input_goal = input("Enter the coordinates of the  goal node as (X, Y): ")

        try:
            goal_points = list(map(lambda x: round(int(x) / SCALE_FACTOR), user_input_goal.strip().split()))
            goal_state = (goal_points[0] + round(500 / SCALE_FACTOR), goal_points[1] + round(1000 / SCALE_FACTOR))
            
            if (goal_state[0] not in range(0, obs_space.shape[0])) and (goal_state[1] not in range(0, obs_space.shape[1])):
                raise ValueError("Invalid goal node.")
            
            if np.array_equal(obs_space[goal_state[1], goal_state[0], :], BLUE) or np.array_equal(obs_space[goal_state[1], goal_state[0], :], BLACK):
                raise ValueError("Invalid goal node.")
            
            return goal_state
        
        except ValueError as e:
            print(e)
            continue

if __name__ == '__main__':
	
    start_point = (50, 100, 0)
    clearance = 2
    rpm = (round(40 / 60), round(80 / 60))

    # creating configuration space
    obs_space = config_space()
    # taking input for start and goal
    goal_point= user_input(obs_space)

    # timer object to measure computation time
    timer_start = time.time()

    # implementing dijkstra
    optimal_path, wheel_rpms, visit_map = astar(start_point, goal_point, rpm)

    timer_stop = time.time()
    c_time = timer_stop - timer_start
    print("Total Runtime: ", c_time)

    # convverting rpms to speeds for publishing to turtlebot
    speeds = convert_speed(wheel_rpms)
    # Turtlebot drive
    turtlebot(speeds)





