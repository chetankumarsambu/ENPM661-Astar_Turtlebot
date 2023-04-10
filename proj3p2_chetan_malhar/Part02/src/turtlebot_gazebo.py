import sys
import rospy
import cv2
from queue import PriorityQueue
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np
import math
from tf import transformations

w_r = 3.3
r_r = 15
w_d = 35.4

lst_open = PriorityQueue()
lst_close = np.zeros((400, 1200), np.uint8)
lst_visual = []
tree = []
node_cost = {}
node_prnt = {}

def input_coordinates(obs_map):
    goal_x = int(input("Enter goal x-coordinate: "))
    goal_y = int(input("Enter goal y-coordinate: "))
    g = (goal_x, obs_map.shape[0] - goal_y - 1)
    return g

def verify_input(start_node, goal, obs_map, r_r, clear):
    clearnce_total = int(round(r_r + clear))
    if(obs_map[start_node[1]][start_node[0]][0]==255 or obs_map[goal[1]][goal[0]][0]==255 or 
      (start_node[0] - clearnce_total) < 0 or (start_node[0] + clearnce_total) > 599 or (start_node[1] - clearnce_total) < 0 or (start_node[1] + clearnce_total) > 199
       or sum(obs_map[goal[1]][goal[0]])==765 or sum(obs_map[start_node[1]][start_node[0]])==765
      ):
        rospy.logerr("Start or end point is invalid. \n")
        return False
    else:
        rospy.loginfo("The start and goal points are valid")
        return True

def mapping(canva, clear):
    canva_ = canva.copy()
    for j in range(canva.shape[0]):
        for i in range(canva.shape[1]):
            if(i>=150 - clear and i < 165 + clear and j<=125 + clear):
                canva_[j,i] = [255, 0 ,0]
            if(i>=250 - clear and i < 265 + clear and j>=75 - clear):
                canva_[j,i] = [255, 0 ,0]
            if(i>=150 and i < 165 and j<=125):
                canva_[j,i] = [255, 255 ,255]
            if(i>=250 and i < 265 and j>=75):
                canva_[j,i] = [255, 255 ,255]
            if((i - 400)**2 + (j - 90)**2 <= (50 + clear)**2):
                canva_[j,i] = [255, 0 ,0]   
            if((i - 400)**2 + (j - 90)**2 <= (50)**2):
                canva_[j,i] = [255, 255 ,255]  
            if(i<clear or j<clear or i>canva.shape[1] - 1 - clear or j>canva.shape[0] - 1 - clear):
                canva_[j, i] = [255, 0 ,0]
    return canva_

def funct_cost(x_i, y_i, theta_i, u_l, u_r, clear, obs_map):
    t = 0
    dt = 0.1
    x_n = x_i
    y_n = y_i
    theta_n = math.pi * theta_i / 180
    D = 0
    
    while t < 0.5:
        t += dt
        delta_x_n = 0.5 * w_r * (u_l + u_r) * math.cos(theta_n) * dt
        delta_y_n = 0.5 * w_r * (u_l + u_r) * math.sin(theta_n) * dt
        theta_n += (w_r / w_d) * (u_r - u_l) * dt
        D += math.sqrt(delta_x_n ** 2 + delta_y_n ** 2)
        x_n += delta_x_n
        y_n += delta_y_n
        x_n = round(x_n)
        y_n = round(y_n)
        pt_next = (x_n, y_n)
        
        if not valid_pt(pt_next, obs_map, r_r, clear):
            return None, (None, None, None)
        
        tree.append(((x_i, y_i), (x_n, y_n)))
        x_i = x_n
        y_i = y_n
        
    theta_n = 180 * (theta_n) / math.pi
    
    if theta_n < 0:
        theta_n += 360
        
    theta_n %= 360
    theta_n = int(round(theta_n))
    
    return D, (x_n, y_n, theta_n)

def valid_pt(neighbor, obs_map, r_r, clear):
    clearnce_total = r_r + clear
    clearnce_total = int(round(clearnce_total))
    if(obs_map[neighbor[1]][neighbor[0]][0]==255 or  
      (neighbor[0] - clearnce_total) < 0 or (neighbor[0] + clearnce_total) > 599 or (neighbor[1] - clearnce_total) < 0 or (neighbor[1] + clearnce_total) > 199
       or  sum(obs_map[neighbor[1]][neighbor[0]])==765):
        return False
    else:
        return True

def next_step(current_node, obs_map, actions, clear):
    valid_neighbors = []
    for action in actions:
        neighbor = funct_cost(current_node[0], current_node[1], current_node[2], action[0], action[1], clear, obs_map)
        if(neighbor[1][0] and neighbor[1][1]):
            valid_neighbors.append(neighbor)
    return valid_neighbors

def astar_algorithm(obs_map, start_node, goal, rpm_1, rpm_2, clear):
    actions = [(0, rpm_1), (rpm_1, 0), (rpm_1, rpm_1), (0, rpm_2), (rpm_2, 0), (rpm_2, rpm_2), (rpm_1, rpm_2), (rpm_2, rpm_1)]
    while lst_open:
        current_node = lst_open.get()[1]
        lst_close[int(round(current_node[1] * 2)), int(round(current_node[0] * 2))] = 2
        lst_visual.append(current_node)
        goal_reached = False
        if(math.sqrt((current_node[0] - goal[0])**2 + (current_node[1] - goal[1])**2) < 0.5):
            goal_reached = True
        else:
            goal_reached = False

        if (goal_reached):
            rospy.loginfo("Goal Reached!!")
            found_path = backtracking(current_node)
            return found_path
        else:
            valid_neighbors = next_step(current_node, obs_map, actions, clear)
            for next_node in valid_neighbors:
                closed = lst_close[int(round(next_node[1][1] * 2)), int(round(next_node[1][0] * 2))]
                if(closed == 2):
                    continue
                else:
                    cost_to_next_node = next_node[0]
                    total_cost_to_node = node_cost[current_node] + cost_to_next_node
                    if(next_node[1] not in node_cost or total_cost_to_node < node_cost[next_node[1]]):
                        node_cost[next_node[1]] = total_cost_to_node
                        euc_dist = math.sqrt((next_node[1][0] - goal[0])**2 + (next_node[1][1] - goal[1])**2)
                        cost_with_heuristic = total_cost_to_node + euc_dist       
                        lst_open.put((cost_with_heuristic, next_node[1]))
                        node_prnt[next_node[1]] = current_node

def backtracking(current_node):
    found_path = []
    final_node = current_node
    while final_node != None:
        found_path.append(final_node)
        final_node = node_prnt[final_node]
    found_path.reverse()
    # rospy.loginfo("The path traversed:\n")
    for node in found_path:
        rospy.loginfo(str((node[0], node[1])))
    return found_path

def pos_now(msg):
    global x
    global y
    global theta

    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y

    orientation = msg.pose.pose.orientation
    (m, n, theta) = transformations.euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])

def turtlebot3(publisher, planned_path, goal):
    vel = Twist()
    rate = rospy.Rate(4)
    for idx, node in enumerate(planned_path):
        next_xpos = node[0]
        next_ypos = node[1]
        while True:
            if(math.sqrt((next_xpos - x)**2 + (next_ypos - y)**2) < 0.1):
                vel.linear.x = 0.0
                vel.angular.z = 0.0
                publisher.publish(vel)
                rospy.loginfo("Reached the point!")
                break
            else:
                if abs(math.atan2(next_ypos - y, next_xpos - x) - theta) > 0.2:
                    vel.linear.x = 0.0
                    vel.angular.z = (math.atan2(next_ypos - y, next_xpos - x) - theta) * 0.3
                else:
                    vel.angular.z = 0.0
                    vel.linear.x = math.sqrt((next_xpos - x)**2 + (next_ypos - y)**2) * 0.3
            publisher.publish(vel)
            rate.sleep()
            if ((idx == len(planned_path) - 1) and math.sqrt((next_xpos - goal[0])**2 + (next_ypos - goal[1])**2) < 0.1):
                vel.linear.x = 0
                vel.angular.z = 0
                publisher.publish(vel)
                rospy.loginfo("Goal reached")
                exit()

def func_main(args):
    # initialize node
    rospy.init_node('tb_sim')

    # creating publisher
    publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)

    # create empty canva
    canva = np.zeros((200, 600, 3), dtype='uint8')
    clear = 15
    obs_map = mapping(canva, clear)

    odom_sub = rospy.Subscriber('/odom', Odometry, pos_now)  # Odometry subscriber
    x_i, y_i, orientation_s = 50, 100, 0  # start_nodeing point

    g = input_coordinates(obs_map)
    i = (x_i, obs_map.shape[0] - y_i - 1, orientation_s)

    while (not verify_input(i, g, obs_map, r_r, clear)):
        g = input_coordinates(obs_map)

    if (verify_input(i, g, obs_map, r_r, clear)):
        lst_open.put((0, i))
        node_prnt[i] = None  # start_node node has no parent
        node_cost[i] = 0  # start_node node has no cost

    rospy.loginfo("Goal coordinates: " + str(g))
    # fixed rpms
    rpm1, rpm2 = 10, 10  # RPM's
    found_path = astar_algorithm(obs_map, i, g, rpm1, rpm2, clear)

    transformed_path = []
    for node in found_path:
        node_x = node[0] / 100.0 - 0.5
        node_y = (200 - node[1]) / 100.0 - 1
        node_tup = (node_x, node_y)
        transformed_path.append(node_tup)

    transformed_goal = (g[0] / 100.0 - 0.5, (200 - g[1]) / 100 - 1)

    while (not rospy.is_shutdown()):
        try:
            turtlebot3(publisher, transformed_path, transformed_goal)
        except KeyboardInterrupt:
            print("Shutting down")

if __name__ == '__main__':
    func_main(sys.argv)



