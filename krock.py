import matplotlib.pyplot as plt
import json
import random
import numpy as np

# imports the task data given in GitHub and stores it as a dictionary
f = open('task.json')
data = json.load(f)

# graph class to manage linking of sampled nodes
class Graph:
    def __init__(self, start=None, values = None, directed=False):
        self._adjlist = {}
        if values is None:
            values = {}
        self._valuelist = values
        self._isdirected = directed

        # creates an adjacency list
        if start != None:
            for i in range(len(start)):
                if start[i][0] not in self._adjlist.keys():
                    self._adjlist[start[i][0]] = []
                if start[i][1] not in self._adjlist.keys():
                    self._adjlist[start[i][1]] = []

            for key in self._adjlist:
                self._valuelist[key] = None
                for i in range(len(start)):
                    if key in start[i]:
                        if key == start[i][0]:
                            self._adjlist[key].append(start[i][1])
                        if key == start[i][1]:
                            self._adjlist[key].append(start[i][0])

    def vertices(self):
        return list(self._adjlist.keys())

    def edges(self):
        # loops through all neighbors for all nodes and lists every unique pair of node and neighbor        currentEdges = []
        for key in self._adjlist.keys():
            for i in range(len(self._adjlist[key])):
                if (key, self._adjlist[key][i]) not in currentEdges and (self._adjlist[key][i], key) not in currentEdges:
                    currentEdges.append((key, self._adjlist[key][i]))
        return currentEdges

    def neighbours(self,v):
        return self._adjlist[v]

# takes the cube center positions and returns 8 vertices for every
# cube center that represents the cube corners
def cube_corners(cube_centers, half_size):
    cubes = {}
    for i, center in enumerate(cube_centers):
        cubes[i] = [[center[0]+half_size, center[1]+half_size, center[2]+half_size], 
                    [center[0]-half_size, center[1]+half_size, center[2]+half_size], 
                    [center[0]+half_size, center[1]-half_size, center[2]+half_size], 
                    [center[0]-half_size, center[1]-half_size, center[2]+half_size], 
                    [center[0]+half_size, center[1]+half_size, center[2]-half_size], 
                    [center[0]-half_size, center[1]+half_size, center[2]-half_size], 
                    [center[0]+half_size, center[1]-half_size, center[2]-half_size], 
                    [center[0]-half_size, center[1]-half_size, center[2]-half_size]]

    return cubes

# takes all the cube corner points and the points of each node in the path
def visualize(cubes, path, dist):
    from mpl_toolkits.mplot3d import Axes3D
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection
    from mpl_toolkits.mplot3d.art3d import Line3DCollection
    fig = plt.figure()
    ax = Axes3D(fig, auto_add_to_figure=False)
    fig.add_axes(ax)

    # plot every cube separately
    for i, cube in enumerate(cubes.values()):
        # list x, y and z positions of every corner of the cube
        x = [corner[0] for corner in cube]
        y = [corner[1] for corner in cube]
        z = [corner[2] for corner in cube]

        # make a list that dictates which corners are needed to project each side of the cube
        vertices = [[0, 1, 3, 2], [4, 5, 7, 6], [0, 1, 5, 4], [3, 2, 6, 7], [0, 2, 6, 4], [3, 1, 5, 7]]
        tupleList = list(zip(x, y, z))

        # create the specific argument that is accepted by 3DCollection library
        poly3d = [[tupleList[vertices[ix][iy]] for iy in range(len(vertices[0]))] for ix in range(len(vertices))]
        
        # start cube is green and goal cube is red
        if i == 0:
            ax.add_collection3d(Poly3DCollection(poly3d, edgecolors='k', facecolors='green', linewidths=1, alpha=0.4))
        elif i == len(cubes.values()) - 1:
            ax.add_collection3d(Poly3DCollection(poly3d, edgecolors='k', facecolors='red', linewidths=1, alpha=0.4))
        else:
            ax.add_collection3d(Poly3DCollection(poly3d, edgecolors='k', facecolors='grey', linewidths=1))

    # lists all the x, y, and z points of all nodes in the path from start to goal
    x = path[0]
    y = path[1]
    z = path[2]

    # make the same kind of vertex matrix as in the cube case
    vertices = []
    for i in range(len(x)-1):
        vertices.append([i, i+1])

    # and prepare the data for plotting
    tupleList = list(zip(x, y, z))
    poly3d = [[tupleList[vertices[ix][iy]] for iy in range(len(vertices[0]))] for ix in range(len(vertices))]
    ax.add_collection3d(Line3DCollection(poly3d, colors='k', linewidths=3,))

    ax.plot(x, y, z, label='parametric curve')
    ax.grid(False)
    ax.set_xticks([])
    ax.set_yticks([])
    ax.set_zticks([])
    ax.xaxis.pane.fill = False
    ax.yaxis.pane.fill = False
    ax.zaxis.pane.fill = False
    ax.xaxis.pane.set_edgecolor('w')
    ax.yaxis.pane.set_edgecolor('w')
    ax.zaxis.pane.set_edgecolor('w')
    ax.text2D(0.3, 0.75, "Total path length: " + str(round(dist, 1)), transform=ax.transAxes)
    ax.set_axis_off()
    ax.set(xlim=(-3,3), ylim=(-3,3), zlim=(-2.3,2.3))
    plt.show()

# tells if a line between two point intersects an arbitrary cube in space    
def collision(cube, point1, point2):
    # the method work by the following system of equations
    # x = A_x + (B_x - A_x)t
    # y = A_y + (B_y - A_y)t
    # z = A_z + (B_z - A_z)t
    # condition for intersection: if any of the cube sides intersects the 
    # line at a point inside the adjacent 4 sides, the line and cube intersects

    # x
    low_x = cube[1][0]
    high_x= cube[0][0]

    if (point2[0] - point1[0]) != 0:
        low_t = (low_x - point1[0])/(point2[0] - point1[0])
        high_t= (high_x- point1[0])/(point2[0] - point1[0])
    else:
        low_t = 1e10
        high_t = 1e10

    low_y = point1[1] + (point2[1] - point1[1]) * low_t
    low_z = point1[2] + (point2[2] - point1[2]) * low_t
    if cube[2][1] <= low_y <= cube[0][1] and cube[4][2] <= low_z <= cube[0][2] and (point2[0] <= low_x <= point1[0] or point2[0] >= low_x >= point1[0]):
        return True

    high_y = point1[1] + (point2[1] - point1[1]) * high_t
    high_z = point1[2] + (point2[2] - point1[2]) * high_t
    if cube[2][1] <= high_y <= cube[0][1] and cube[4][2] <= high_z <= cube[0][2] and (point2[0] <= high_x <= point1[0] or point2[0] >= high_x >= point1[0]):
        return True

    
    # y
    low_y = cube[2][1]
    high_y= cube[0][1]

    if (point2[1] - point1[1]) != 0:
        low_t = (low_y - point1[1])/(point2[1] - point1[1])
        high_t= (high_y- point1[1])/(point2[1] - point1[1])
    else:
        low_t = 1e10
        high_t = 1e10

    low_x = point1[0] + (point2[0] - point1[0]) * low_t
    low_z = point1[2] + (point2[2] - point1[2]) * low_t
    if cube[1][0] <= low_x <= cube[0][0] and cube[4][2] <= low_z <= cube[0][2] and (point2[1] <= low_y <= point1[1] or point2[1] >= low_y >= point1[1]):
        return True

    high_x = point1[0] + (point2[0] - point1[0]) * high_t
    high_z = point1[2] + (point2[2] - point1[2]) * high_t
    if cube[1][0] <= high_x <= cube[0][0] and cube[4][2] <= high_z <= cube[0][2] and (point2[1] <= high_y <= point1[1] or point2[1] >= high_y >= point1[1]):
        return True


    # z
    low_z = cube[4][2]
    high_z= cube[0][2]

    if (point2[2] - point1[2]) != 0:
        low_t = (low_z - point1[2])/(point2[2] - point1[2])
        high_t= (high_z- point1[2])/(point2[2] - point1[2])
    else:
        low_t = 1e10
        high_t = 1e10

    low_x = point1[0] + (point2[0] - point1[0]) * low_t
    low_y = point1[1] + (point2[1] - point1[1]) * low_t
    if cube[1][0] <= low_x <= cube[0][0] and cube[2][1] <= low_y <= cube[0][1] and (point2[2] <= low_z <= point1[1] or point2[2] >= low_z >= point1[2]):
        return True

    high_x = point1[0] + (point2[0] - point1[0]) * high_t
    high_y = point1[1] + (point2[1] - point1[1]) * high_t
    if cube[1][0] <= high_x <= cube[0][0] and cube[2][1] <= high_y <= cube[0][1] and (point2[2] <= high_z <= point1[1] or point2[2] >= high_z >= point1[2]):
        return True

    return False

# dijkstra's algorithm to find the shortest path in a weighted graph
def dijkstra(graph, source, cost=lambda u,v: 1):
    
    dijkdict = {}
    # all paths are empty in the beginning
    for key in graph.vertices():
        dijkdict[key] = {'path': [], 'cost': 0}

    # the first place in source is source
    dijkdict[source]['path'].append(source)

    # start counting the visited nodes
    visited = {}
    # and initiate a list that stores distances to each stop
    distances = {vertex: None for vertex in graph.vertices()}

    # every node has the value of infinity except source that has the value 0 from start
    for value in distances.keys():
        if value != source:
            distances[value] = 1e12
        else:
            distances[value] = 0
    
    vertex = source
    # every path is updated as long as there are targets whose paths are not optimized
    while len(distances.keys()) != 0:
        
        # finds which node is closest
        least_index = 1e12
        for key in distances:
            if distances[key] < least_index:
                least_index = distances[key]
        
        # initiate that node with an arbitrary name
        vertex = 0
        # loop through all nodes that will be visited
        for key in distances:
            # remove selected node from unvisited and select it as current node
            if distances[key] == least_index:
                vertex = key
                visited[key] = distances[key]
                distances.pop(key)
                break
        
        # loop through all neighbors and update path if there exists a shorter path
        for neighbor in graph._adjlist[vertex]:
            if neighbor not in visited.keys():

                # security construction 
                if type(cost(vertex,neighbor)) != type(None):

                    # new path
                    alt = visited[vertex] + cost(vertex,neighbor)

                    # if shorter path is found, insert it and remove the longer one
                    if alt <= distances[neighbor]:
                        dijkdict[neighbor]['path'] = []
                        dijkdict[neighbor]['path'] = dijkdict[vertex]['path'].copy()
                        dijkdict[neighbor]['path'].append(neighbor)
                        distances[neighbor] = alt
    
    # construct a dictionary that holds a path and a total weight for every possible target
    dijkdict = {key: {'path': dijkdict[key]['path'], 'cost': visited[key]} for key in dijkdict.keys()}
    return dijkdict

def weight(edge1, edge2):
    # returns the distance with the distance formula s = sqrt(sum(diff_coord))
    return ((z_samp[edge2] - z_samp[edge1])**2 + (y_samp[edge2] - y_samp[edge1])**2 + (x_samp[edge2] - x_samp[edge1])**2)**(1/2)

# list of points for every cube center
cube_centers = [data['start_position']] + data['cube_positions:'] + [data['goal_position']]
# half cube width
hcw = 0.5*float(data['cube_width']) 

# one list of cubes that stores double-sized cubes
# so that a cube of normal size can follow the path
# of a point that can pass through the double-size cube maze
cubes = cube_corners(cube_centers, 2*hcw)

# normal size cubes for visualization
show_cubes = cube_corners(cube_centers, hcw)

# the cubes that should be avoided are not the start and goal cube
cubes.pop(list(cubes.keys())[-1])
cubes.pop(list(cubes.keys())[0])

# sample points
low_corner  = data['domain_lower_corner']
high_corner = data['domain_upper_corner']
x_samp = np.random.uniform(low = low_corner[0]+0.25, high = high_corner[0]-0.25, size = (500,))
y_samp = np.random.uniform(low = low_corner[1]+0.25, high = high_corner[1]-0.25, size = (500,))
z_samp = np.random.uniform(low = low_corner[2]+0.25, high = high_corner[2]-0.25, size = (500,))

# remove all points that accidently ended up inside an obstacle cube
remove_index = []
for i in range(len(x_samp)):
    for cube in cubes.values():
        if cube[1][0] <= x_samp[i] <= cube[0][0] and cube[2][1] <= y_samp[i] <= cube[0][1] and cube[4][2] <= z_samp[i] <= cube[0][2]:
            remove_index.append(i)
x_samp = np.delete(x_samp, remove_index)
y_samp = np.delete(y_samp, remove_index)
z_samp = np.delete(z_samp, remove_index)

# add the center of the start and goal cube to the list of sampled points
x_samp = np.append(x_samp, -0.076138)
y_samp = np.append(y_samp, -3.258682)
z_samp = np.append(z_samp, 0.54373)
x_samp = np.append(x_samp, 0.123147)
y_samp = np.append(y_samp, 2.635865)
z_samp = np.append(z_samp, 0.321962)

# loop through all point combinations and add an edge between 
# them if the line does not collide with an obstacle cube
edges = []
for i in range(len(x_samp)):
    for j in range(len(x_samp)):
        if i != j:
            krockar = False
            for cube in cubes.values():
                if collision(cube, [x_samp[i], y_samp[i], z_samp[i]], [x_samp[j], y_samp[j], z_samp[j]]):
                    krockar = True
                    break
                    
            if not (krockar) and tuple([i,j]) not in edges and tuple([j,i]) not in edges:
                edges.append(tuple([i,j]))

# create a graph
G = Graph(start=edges)

# finds the shortest path with the weight function as cost
pathdict = dijkstra(G, len(x_samp)-2, weight)

# picks out the path
path = pathdict[len(x_samp)-1]['path']
# and the total distance
total_distance = pathdict[len(x_samp)-1]['cost']
print(path)
print(total_distance)

# lists x, y, and z points for all edge nodes
path_edges = [[x_samp[i] for i in path], [y_samp[i] for i in path], [z_samp[i] for i in path]]

# uncomment to visualize the cubes and path 
visualize(show_cubes, path_edges, total_distance)