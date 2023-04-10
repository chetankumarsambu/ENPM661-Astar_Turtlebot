
import cv2
import matplotlib.pyplot as plt
import numpy as np
import math
from queue import PriorityQueue

# Constants
radius_wheel = 3.3
radius_robot = 10.5
distance_wheel = 3.54

# Global variables
lst_open = PriorityQueue()
lst_close = np.zeros((400, 1200), np.uint8)
lst_visual = []
tree = []
node_cost = {}
node_prnt = {}


def input_coordinates(obs_map):
    input_x = int(input("Enter initial x-coordinate"))
    input_y = int(input("Enter initial y-coordinate"))
    goal_x = int(input("Enter goal x-coordinate"))
    goal_y = int(input("Enter goal y-coordinate"))
    orient = int(input("Enter start orientation "))
    rpm_1 = int(input("Enter RPM 1: "))
    rpm_2 = int(input("Enter RPM 2: "))
    rpm = (rpm_1, rpm_2)
    i = (input_x, obs_map.shape[0] - input_y - 1, orient)
    g = (goal_x, obs_map.shape[0] - goal_y - 1)
    return i, g, rpm



def verify_input(start_node, goal, obs_map, radius_robot, clear):
    clearnce_total = radius_robot + clear
    clearnce_total = int(round(clearnce_total))
    
    if(obs_map[start_node[1]][start_node[0]][0]==255 or obs_map[goal[1]][goal[0]][0]==255 or 
      (start_node[0] - clearnce_total) < 0 or (start_node[0] + clearnce_total) > 599 or (start_node[1] - clearnce_total) < 0 or (start_node[1] + clearnce_total) > 199
       or sum(obs_map[goal[1]][goal[0]])==765 or sum(obs_map[start_node[1]][start_node[0]])==765
      ):
        print("The start or end point is invalid.\n")
        return False
    else:
        print("The start and goal points are valid")
        return True

  
def map_generate(canva, clear):
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

def funct_cost(input_x,input_y,theta_i,u_l,u_r, clear, obs_map):
    t = 0
    dt = 0.1
    x_n = input_x
    y_n = input_y
    theta_n = 3.14 * theta_i / 180
    D = 0
    while t < 0.5:
        t = t + dt
        delta_x_n = 0.5 * radius_wheel * (u_l + u_r) * math.cos(theta_n) * dt
        delta_y_n = 0.5 * radius_wheel * (u_l + u_r) * math.sin(theta_n) * dt
        theta_n += (radius_wheel / distance_wheel) * (u_r - u_l) * dt
        D = D + math.sqrt((0.5 * radius_wheel * (u_l + u_r) * math.cos(theta_n) * dt)**2 + (0.5 * radius_wheel * (u_l + u_r) * math.sin(theta_n) * dt)**2)
        x_n+=delta_x_n
        y_n+=delta_y_n
        x_n = round(x_n)
        y_n = round(y_n)
        next_point = (x_n, y_n)
        if(not valid_pt(next_point, obs_map, radius_robot, clear)):
            return None, (None, None, None)
        tree.append(((input_x,input_y),(x_n,y_n)))
        input_x = x_n
        input_y = y_n
    theta_n = 180 * (theta_n) / 3.14
    if(theta_n < 0):
        theta_n+=360
    theta_n%=360
    theta_n = int(round(theta_n))
    return D, (x_n, y_n, theta_n)



def valid_pt(next_node, obs_map, radius_robot, clear):
    clearnce_total = radius_robot + clear
    clearnce_total = int(round(clearnce_total))
    if(obs_map[next_node[1]][next_node[0]][0]==255 or  
      (next_node[0] - clearnce_total) < 0 or (next_node[0] + clearnce_total) > 599 or (next_node[1] - clearnce_total) < 0 or (next_node[1] + clearnce_total) > 199
       or  sum(obs_map[next_node[1]][next_node[0]])==765):
        return False
    else:
        return True
    

def next_step(current_node, obs_map, action_set):
    next_nodes = []
    for action in action_set:
        next_node = funct_cost(current_node[0], current_node[1], current_node[2], action[0], action[1], clear, obs_map)
        if next_node[1][0] and next_node[1][1]:
            next_nodes.append(next_node)
    return next_nodes

 


def astar_algorithm(obs_map, rpm_1, rpm_2):
    action_set = [(0, rpm_1),(rpm_1, 0),(rpm_1, rpm_1),(0, rpm_2),(rpm_2, 0),(rpm_2, rpm_2),(rpm_1, rpm_2),(rpm_2, rpm_1)]
    while lst_open:
        current_node = lst_open.get()[1]
        lst_close[int(round(current_node[1] * 2)),int(round(current_node[0] * 2))] = 2
        lst_visual.append(current_node)
        goal_reached = False
        if(math.sqrt((current_node[0] - goal[0])**2 + (current_node[1] - goal[1])**2) < 0.5):
            goal_reached = True
        else:
            goal_reached = False
            
        if (goal_reached):
            print("Goal Reached")
            path_found = backtracking(current_node)
            return path_found
        else:
            next_nodes = next_step(current_node, obs_map, action_set)  # Rename this variable to match the changes in the next_step function
            for next_node in next_nodes:
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
    path_found = []
    node_final = current_node
    while node_final != None:
        path_found.append(node_final)
        node_final = node_prnt[node_final]
        print(node_final)
    path_found.reverse()
    return path_found


canva = np.zeros((200, 600, 3), dtype='uint8')

clear = int(input("Enter clearance required: "))


obs_map = map_generate(canva, clear)
cv2.imshow("Obstacle Map", obs_map)
cv2.waitKey(2000)


start_node, goal, rpm = input_coordinates(obs_map)

while(not verify_input(start_node, goal, obs_map, radius_robot, clear)):
    start_node, goal, rpm = input_coordinates(obs_map)

if(verify_input(start_node, goal, obs_map, radius_robot, clear)):
    lst_open.put((0, start_node))
    node_prnt[start_node] = None 
    node_cost[start_node] = 0 
   
   
path_found = astar_algorithm(obs_map, rpm[0], rpm[1])


fourcc = cv2.VideoWriter_fourcc(*'mp4v') 
video1 = cv2.VideoWriter('exploration.avi', fourcc, 1000, (600, 200))
video2 = cv2.VideoWriter('path.avi', fourcc, 30, (600, 200))
video3 = cv2.VideoWriter('trees_explored.avi', fourcc, 1000, (600, 200))

for x, y, ori in lst_visual:
    obs_map[y, x, 1] = 255
    video1.write(obs_map)

obs_map = map_generate(canva,clear)

for x, y, ori in path_found:
    obs_map[y, x, 1] = 255
    cv2.circle(obs_map, (x, y), 5, (30, 50, 60))
    video2.write(obs_map)

obs_map = map_generate(canva,clear)

for tr in tree:
    cv2.line(obs_map, (tr[0][0],tr[0][1]),(tr[1][0],tr[1][1]), (30, 50, 60), 1)
    video3.write(obs_map)






