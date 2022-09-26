#   RBE550 - Motion Planning HMWK 5 
#   Author: <NATHANAEL> <VICKERY> 
#   Date: Nov. 17, 2021

#   ----------------------------------------------
#   Theft - RRT Planner
#   Note: *Collision Checker on line 172
#         *RRT Path Planner on line 320 (and 300)
#   ----------------------------------------------

## Imported Libraries ----------------------------
from mpl_toolkits import mplot3d
import numpy as np
from scipy.spatial.transform import Rotation as R
import math
import matplotlib.pyplot as plt
import random

## Parameters ------------------------------------
#Can paremeters
can_d = 52 # [mm] can diameter
can_h = 122 # [mm] can height
can_r = math.hypot(52/2, 122/2) # [mm] Sphere-Collision Radius of Can
##print('Movement Steps must be smaller than ',round(can_r),'mm')
#Chamber Parameters
main_cham = 1000 # [mm] side dimensions of main cube chamber
second_cham = 250 # [mm] length of 2nd chamber
third_cham = 750 # [mm] lenghth of "3rd chamber" or Outside Vending Machine
length = main_cham + second_cham + third_cham
width_abs = round(main_cham/2)
#Window Parameters
first_window = 800 # [mm] height of window's center point
second_window = 200 # [mm] height of window's center point
window_w = 150 # [mm] width of square windows
fst_win_bot = first_window - round(window_w/2)
fst_win_top = first_window + round(window_w/2)
snd_win_bot = second_window - round(window_w/2)
snd_win_top = second_window + round(window_w/2)
#Initial and Final Positions
p_o = [200, 0, can_h/2, 0, 0, 0] # [x,y,z] [mm] can's initial position and orientation
p_f = [main_cham+second_cham+500, 0, can_h/2, 0, 0, 0] # [x,y,z] [mm] can's final position and orientation

#RRT Parameters -------------------------------------
dq = 60 # [mm] step size
knum = 1500 #2000 #4000 #1000 #500 # base limit of # of nodes in graph
samples = 200 # base amount of random samples
bias = 0.2 # [% < 0.99] goal-biased additional percent added to the sample pot
goal_radius = 60 # [mm] radius around goal were path is considered successful
random.seed(4325)#3526) # random module seed

## Obstacle Space -----------------------------------
ox, oy, oz = [], [], []
barr_density = 50 # Barrier Density
snd_barr_density = 25 # Barrier Density

barr1x, barr1y, barr1z = [],[],[]
barr2x, barr2y, barr2z = [],[],[]

a = np.arange(0,main_cham - 49,barr_density) #height [mm] by "barr_density" mm sections
b = np.arange(0,length + 1,barr_density) #length [mm] by "barr_density" mm sections
c = np.arange(-width_abs, width_abs + 1,barr_density) #width [mm] by "barr_density" mm sections
d = np.arange(-width_abs + 50, width_abs -49,barr_density) #width [mm] by "barr_density" mm sections
e = np.arange(0,fst_win_bot + 1,snd_barr_density) #1st window bottom height [mm] by "barr_density" mm sections
f = np.arange(fst_win_top, main_cham - 49, snd_barr_density) #1st window top height [mm] by "barr_density" mm sections
g = np.arange(0,snd_win_bot + 1, snd_barr_density) #2nd window bottom height [mm] by "barr_density" mm sections
h = np.arange(snd_win_top,main_cham - 49, snd_barr_density) #2nd window top height [mm] by "barr_density" mm sections
i = np.arange(-width_abs + 50, -round(window_w/2) + 1, snd_barr_density) #width [mm] by "barr_density" mm sections
j = np.arange(round(window_w/2), width_abs - 49, snd_barr_density) #width [mm] by "barr_density" mm sections
k = np.arange(-round(window_w/2) + 25, round(window_w/2) - 24, snd_barr_density) #width [mm] by "barr_density" mm sections

#Floor and Ceiling
for xelem in b:         # Floor and Ceiling
    for yelem in c:
        ox.append(xelem)
        oy.append(yelem)
        oz.append(-50)
        ox.append(xelem)
        oy.append(yelem)
        oz.append(1000)
# Side Walls
for xelem in b:         # right wall and left wall
    for zelem in a:
        ox.append(xelem)
        oy.append(500)
        oz.append(zelem)
        ox.append(xelem)
        oy.append(-500)
        oz.append(zelem)
# Barriers
for yelem in d:         # Back Wall
    for zelem in a:
        ox.append(0)
        oy.append(yelem)
        oz.append(zelem)
for yelem in i:         # Barriers left side
    for zelem in a:
        ox.append(main_cham)
        oy.append(yelem)
        oz.append(zelem)
        ox.append(main_cham + second_cham)
        oy.append(yelem)
        oz.append(zelem)
        barr1x.append(main_cham)
        barr1y.append(yelem)
        barr1z.append(zelem)
        barr2x.append(main_cham + second_cham)
        barr2y.append(yelem)
        barr2z.append(zelem)
for yelem in j:         # Barriers right side
    for zelem in a:
        ox.append(main_cham)
        oy.append(yelem)
        oz.append(zelem)
        ox.append(main_cham + second_cham)
        oy.append(yelem)
        oz.append(zelem)
        barr1x.append(main_cham)
        barr1y.append(yelem)
        barr1z.append(zelem)
        barr2x.append(main_cham + second_cham)
        barr2y.append(yelem)
        barr2z.append(zelem)
for yelem in k:         # Barriers Above 1st Window 
    for zelem in f:
        ox.append(main_cham)
        oy.append(yelem)
        oz.append(zelem)
        barr1x.append(main_cham)
        barr1y.append(yelem)
        barr1z.append(zelem)
for yelem in k:         # Barriers Below 1st Window 
    for zelem in e:
        ox.append(main_cham)
        oy.append(yelem)
        oz.append(zelem)
        barr1x.append(main_cham)
        barr1y.append(yelem)
        barr1z.append(zelem)
for yelem in k:         # Barriers Above 2nd Window 
    for zelem in h:
        ox.append(main_cham + second_cham)
        oy.append(yelem)
        oz.append(zelem)
        barr2x.append(main_cham + second_cham)
        barr2y.append(yelem)
        barr2z.append(zelem)
for yelem in k:         # Barriers Below 2nd Window 
    for zelem in g:
        ox.append(main_cham + second_cham)
        oy.append(yelem)
        oz.append(zelem)
        barr2x.append(main_cham + second_cham)
        barr2y.append(yelem)
        barr2z.append(zelem)
print('Obstacle Space Established')

## Random Configuration Sample Pot ------------------
def sample_generator(goal):
    sample_pot = []
    for i in range(samples):
        x_rand = random.randint(200, 2000) 
        y_rand = random.randint(-400, 400) 
        z_rand = random.randint(0, 1000)
        sample_point = [x_rand, y_rand, z_rand]
        sample_pot.append(sample_point)
    bias_amount = round(len(sample_pot)*bias/(1-bias))
    for i in range(bias_amount):
        sample_pot.append(goal.position)
    print("Sample Pot Generated")
    #print(sample_pot)
    return sample_pot

## Collision Checking -------------------------------
# Uses Sphere Based Collision (radius of the outermost point on the can)
# Returns True if object is in collision with Collision Space
# Returns Fale if object is in Free Space
def collision_check(pos):
    collision = False
    change = None
    for elem in range(len(ox)):
        dx = abs(ox[elem] - pos[0])
        dy = abs(oy[elem] - pos[1])
        dz = abs(oz[elem] - pos[2])
        if np.linalg.norm([dx, dy, dz]) <= can_r:
            if not window_check(pos):
                collision = True
                return collision, None
            print('Window Entry')
            ang_opt = [[0, 0, 0],[0, 90, 0],[0, 90, 90]] #expand options to 45 degrees (+4 options)? not now
            for ang in ang_opt:
                edge_points = transform_edges(ang, pos)
                x_ps, y_ps, z_ps = [], [], []
                for coor in edge_points:
                    x_ps.append(coor[0])
                    y_ps.append(coor[1])
                    z_ps.append(coor[2])
                min_x, max_x = min(x_ps), max(x_ps)
                min_y, max_y = min(y_ps), max(y_ps)
                min_z, max_z = min(z_ps), max(z_ps)
                if not window_check([pos[0], min_y, min_z]) or \
                   not window_check([pos[0], max_y, max_z]) or \
                   not window_check([pos[0], min_y, max_z]) or \
                   not window_check([pos[0], max_y, min_z]):
                    collision = True
                    #print('Window Collision')
                    continue
                collision = False
                change = ang
                print('Window Cleared')
                break           
    return collision, change

def window_check(pos):
    #check if within Window 1
    if 900 <= pos[0] <= 1100 and \
       -75 <= pos[1] <= 75 and \
       725 <= pos[2] <= 875:
        #print('Window Entry')
        return True
    #check if within Window 2
    if 1150 <= pos[0] <= 1350 and \
       -75 <= pos[1] <= 75 and \
       125 <= pos[2] <= 275:
        #print('Window Entry')
        return True
    return False

## Edge Point Transformer for Prism-Collision ------
def transform_edges(n_ang, pos):
##    edges_init = [[pos[0]+26, pos[1], pos[2]+61],[pos[0]-26, pos[1], pos[2]+61],\
##                  [pos[0], pos[1]+26, pos[2]+61],[pos[0], pos[1]-26, pos[2]+61],\
##                  [pos[0]+26, pos[1], pos[2]-61],[pos[0]-26, pos[1], pos[2]-61],\
##                  [pos[0], pos[1]+26, pos[2]-61],[pos[0], pos[1]-26, pos[2]-61]]
    edges_init = [[26, 0, 61],[-26, 0, 61],\
                  [0, 26, 61],[0, -26, 61],\
                  [26, 0, -61],[-26, 0, -61],\
                  [0, 26, -61],[0, -26, -61]]
    #print('Initial Edges: ', edges_init)
    rot_matrix = rotation_matrix(n_ang)
    new_edges = []
    for point in edges_init:
         new_edges.append(np.matmul(rot_matrix, point)) #transpose point if necessary
    for point in new_edges:
        point[0] += pos[0]
        point[1] += pos[1]
        point[2] += pos[2]
    #print('Edge points: ', new_edges)
    return new_edges

# Rotation Matrix for Edges ------------------------
def rotation_matrix(zyz_ang):
    #phi, theta, psi = np.deg2rad(zyz_ang[0]), np.deg2rad(zyz_ang[1]), np.deg2rad(zyz_ang[2])
    #phi, theta, psi = np.radians(zyz_ang[0]), np.radians(zyz_ang[1]), np.radians(zyz_ang[2])
    rot_matrix = R.from_euler('zyz', zyz_ang, degrees=True)
    rot_mat = rot_matrix.as_matrix()
##    rot_mat = [[(np.cos(phi)*np.cos(theta)*np.cos(psi)) - (np.sin(phi)*np.sin(psi)), \
##                (-np.cos(phi)*np.cos(theta)*np.sin(psi)) - (np.sin(phi)*np.cos(psi)), np.cos(phi)*np.sin(theta)], \
##               [(np.sin(phi)*np.cos(theta)*np.cos(psi)) + (np.cos(phi)*np.sin(psi)), \
##                (-np.sin(phi)*np.cos(theta)*np.sin(psi)) + (np.cos(phi)*np.cos(psi)), np.sin(phi)*np.sin(theta)], \
##               [-np.sin(theta)*np.cos(psi), np.sin(theta)*np.sin(psi), np.cos(theta)]] 
    return rot_mat

## Distance Check between 2 Points -----------------
def distance(point1, point2):
    dx = point2[0] - point1[0]
    dy = point2[1] - point1[1]
    dz = point2[2] - point1[2]
    magnitude = math.sqrt(dx*dx + dy*dy + dz*dz)
    if magnitude < 0.001:
        unit_vector = [0, 0, 0]
    else:
        unit_vector = [dx/magnitude, dy/magnitude, dz/magnitude]
    return magnitude, unit_vector

## Nearest Neighbor Search ------------------------
def find_nearest_neighbor(Graph, q_rand):
    q_nearest = None
    min_dist = None
    min_unit_vector = None
    for node in Graph:
        q_current = node.position
        dist, unit_vector = distance(q_current, q_rand)
        if min_dist == None or dist < min_dist:
            min_dist = dist
            min_unit_vector = unit_vector
            q_nearest = node
    return q_nearest, min_unit_vector, min_dist

## Goal Check -------------------------------------
def goal_check(q_new_position, goal_radius, goal):
    success = False
    dx = abs(goal.position[0] - q_new_position[0])
    dy = abs(goal.position[1] - q_new_position[1])
    dz = abs(goal.position[2] - q_new_position[2])
    if np.linalg.norm([dx, dy, dz]) <= goal_radius:
        success = True
        return success
    return success

## Graph Expansion --------------------------------
def expand_tree(Graph, q_nearest, dq, direction, dist, goal, goal_radius):
    delta_x = round(q_nearest.position[0] + dq*direction[0])
    delta_y = round(q_nearest.position[1] + dq*direction[1])
    delta_z = round(q_nearest.position[2] + dq*direction[2])
    q_new_position = [delta_x, delta_y, delta_z, 0, 0, 0]
    collision, change = collision_check(q_new_position)
    if collision: 
        return Graph, False
    if change != None:
        q_new_position = [delta_x, delta_y, delta_z, change[0], change[1], change[2]] # add new orientaion
    edge_path = (q_nearest.position, q_new_position)
    q_new = node(q_new_position, q_nearest, edge_path)
    Graph.append(q_new)
    if goal_check(q_new_position, goal_radius, goal):
        end = node(goal.position, q_new, (q_new_position, goal.position))
        Graph.append(end)
        return Graph, True
    return Graph, False

## RRT Algorithm ----------------------------------
def RRT(initial, goal, goal_radius, sample_pot):
    Graph = [initial]
    for i in range(knum):
        q_rand = random.choice(sample_pot)
        q_nearest, direction, dist = find_nearest_neighbor(Graph, q_rand)
        Graph, complete = expand_tree(Graph, q_nearest, dq, direction, dist, goal, goal_radius)
        if complete:
            print('Number of Nodes in Tree: ', len(Graph))
            print('Number of Iterations: ', i)
            return Graph, True
    return Graph, False

## Node Class for use in building RRT -------------
class node:
    def __init__(self, position, parent, edge):
        self.position = position
        self.parent = parent
        self.edge = edge

## Build Path from Graph --------------------------
def build_path(graph, pos_o, pos_f):
    pathway = []
    pathway.append(graph[-1].position)
    child = graph[-1]
    while pathway[0] != pos_o:
        pathway.insert(0, child.parent.position)
        child = child.parent
    return pathway

## Can Drawing in Matplotlib ------------------------------
def draw_can(pos):
    edges_init = [[26, 0, 61],[-26, 0, 61], [0, 26, 61],[0, -26, 61],\
                  [26, 0, -61],[-26, 0, -61],[0, 26, -61],[0, -26, -61]]
    rot_matrix = rotation_matrix([pos[3],pos[4],pos[5]])
    new_edges = []
    for point in edges_init:
         new_edges.append(np.matmul(rot_matrix, point)) #transpose point if necessary
    for point in new_edges:
        point[0] += pos[0]
        point[1] += pos[1]
        point[2] += pos[2]
    x_top, y_top, z_top = [new_edges[0][0]],[new_edges[0][1]],[new_edges[0][2]]
    for elem in new_edges[2:4]:
        x_top.append(elem[0])
        y_top.append(elem[1])
        z_top.append(elem[2])
    x_top.append(new_edges[1][0])
    y_top.append(new_edges[1][1])
    z_top.append(new_edges[1][2])
    x_top.append(new_edges[0][0])
    y_top.append(new_edges[0][1])
    z_top.append(new_edges[0][2])
    x_bot, y_bot, z_bot = [new_edges[4][0]],[new_edges[4][1]],[new_edges[4][2]]
    for elem in new_edges[6:-1]:
        x_bot.append(elem[0])
        y_bot.append(elem[1])
        z_bot.append(elem[2])
    x_bot.append(new_edges[5][0])
    y_bot.append(new_edges[5][1])
    z_bot.append(new_edges[5][2])
    x_bot.append(new_edges[4][0])
    y_bot.append(new_edges[4][1])
    z_bot.append(new_edges[4][2])
    side = [new_edges[2],new_edges[3],new_edges[6],new_edges[7],new_edges[2]]
    forw = [new_edges[0],new_edges[1],new_edges[4],new_edges[5],new_edges[1]]
    x_side, y_side, z_side = [],[],[]
    x_for, y_for, z_for = [],[],[]
    for elem in side:
        x_side.append(elem[0])
        y_side.append(elem[1])
        z_side.append(elem[2])
    for elem in forw:
        x_for.append(elem[0])
        y_for.append(elem[1])
        z_for.append(elem[2])
    ax.plot(x_side, y_side, z_side, c='#800000')
    ax.plot(x_for, y_for, z_for, c='#800000')
    ax.plot(x_top, y_top, z_top, c='#800000')
    ax.plot(x_bot, y_bot, z_bot, c='#800000')
    
        
## Main code --------------------------------------
start = node(p_o, None, None)
#start = node(p_f, None, None)
#goal = node([980, 0, 800, 0, 0, 0], None, None)
goal = node(p_f, None, None) # Psuedo-node, only used to give solution in RRT

##if collision_check(goal.position):
##    print('Goal in Collision Zone')

sample_pot = sample_generator(goal)
graph, success = RRT(start, goal, goal_radius, sample_pot)

if success:
    print('Path found!')
    path = build_path(graph, p_o, p_f)
    print(path)
    #final_path = smooth_path(path)
else:
    print('Path not found.')


## Plotting in 3D Example -------------------------
fig1 = plt.figure(figsize=(6.5,6.5))
ax = plt.axes(projection="3d")

# Barrier Plot
##ax.scatter3D(barr1x, barr1y, barr1z, c='#AAAAAA', marker="o", s=15)
##ax.scatter3D(barr2x, barr2y, barr2z, c='#AAAAAA', marker="o", s=15)

# Windows Plot
win1 = [[1000, 1000, 1000, 1000, 1000],[75,-75,-75,75,75],[875, 875, 725, 725, 875]]
win2 = [[1250, 1250, 1250, 1250, 1250],[75,-75,-75,75,75],[275, 275, 125, 125, 275]]
ax.plot(win1[0],win1[1],win1[2], c='#000000')
ax.plot(win2[0],win2[1],win2[2], c='#000000')

# Samples Plot
##for elem in sample_pot:
##    ax.scatter(elem[0], elem[1], elem[2], c='#FFA500', marker="*")

# RRT Plot
for elem in graph[1:]:
    xset, yset, zset = [],[],[]
    for point in elem.edge:
        xset.append(point[0])
        yset.append(point[1])
        zset.append(point[2])
    ax.plot(xset, yset, zset, c='#800080')

# Final Path Plot
if success:
    x_path, y_path, z_path = [],[],[]
    for elem in path:
        x_path.append(elem[0])
        y_path.append(elem[1])
        z_path.append(elem[2])
        ax.plot(x_path, y_path, z_path, c='#002A85')

ax.scatter(start.position[0], start.position[1], start.position[2], c='#008000', marker="o")
ax.scatter(goal.position[0], goal.position[1], goal.position[2], c='#FF0000', marker="o")

# Obstacles Plot
##ax.scatter3D(ox, oy, oz, c=oz, cmap='hsv')

ax.set_xlim(0,length)
ax.set_ylim(-width_abs, width_abs)
ax.set_zlim(0, main_cham)
ax.set_xlabel('x')
ax.set_ylabel('y')
ax.set_zlabel('z')
ax.set_title('Theft - RRT Planner')
plt.show()

## Can Movement Animation
fig2 = plt.figure(figsize=(6.5,6.5))
ax = plt.axes(projection="3d")

for step in path:
    plt.cla()
    ax.plot(win1[0],win1[1],win1[2], c='#000000')
    ax.plot(win2[0],win2[1],win2[2], c='#000000')
    ax.plot(x_path, y_path, z_path, c='#002A85')
    ax.scatter(start.position[0], start.position[1], start.position[2], c='#008000', marker="o")
    ax.scatter(goal.position[0], goal.position[1], goal.position[2], c='#FF0000', marker="o")
    draw_can(step)
    plt.title("RRT Path for Can")
    ax.set_xlim(0,length)
    ax.set_ylim(-width_abs, width_abs)
    ax.set_zlim(0, main_cham)
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')
    plt.pause(0.0001)

plt.show()

print('Theft Homework Complete')


