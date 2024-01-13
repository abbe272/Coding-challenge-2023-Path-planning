import matplotlib.pyplot as plt
import random
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
fig = plt.figure()
ax = Axes3D(fig, auto_add_to_figure=False)
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from mpl_toolkits.mplot3d.art3d import Line3DCollection

import graphviz
class Graph:
    # definierar konstruktor innehållande 3 attribut, grannlista, värdeslista, och bollean för o/riktad graf
    def __init__(self, start=None, values = None, directed=False):
        self._adjlist = {}
        if values is None:
            values = {}
        self._valuelist = values
        self._isdirected = directed

        # plus some code for building a graph from a ’start’ object
        # such as a list of edges
        # here are some of the public methods to implement

        # skapa dict som är adjlist 
        # genom att loopa genom alla kanter och kolla om dess hörn redan finns i grannlistan
        if start != None:
            for i in range(len(start)):
                if start[i][0] not in self._adjlist.keys():
                    self._adjlist[start[i][0]] = []
                if start[i][1] not in self._adjlist.keys():
                    self._adjlist[start[i][1]] = []

            # sätt alla värden i värdelistan till None
            for key in self._adjlist:
                self._valuelist[key] = None
                # loopa igenom alla kanter för varje nod och gruppera noder vars kanter innehåller den sökta noden
                for i in range(len(start)):
                    if key in start[i]:
                        if key == start[i][0]:
                            self._adjlist[key].append(start[i][1])
                        if key == start[i][1]:
                            self._adjlist[key].append(start[i][0])

    def __len__(self):
        # nyckellistan i grannlistan innehåller alla noder
        return len(self._adjlist.keys())
    
    def vertices(self):
        # nyckellistan i grannlistan innehåller alla noder
        return list(self._adjlist.keys())

    def edges(self):
        # loopa genom alla grannar för alla noder - lista alla unika par av noder och granne 
        currentEdges = []
        for key in self._adjlist.keys():
            for i in range(len(self._adjlist[key])):
                if (key, self._adjlist[key][i]) not in currentEdges and (self._adjlist[key][i], key) not in currentEdges:
                    currentEdges.append((key, self._adjlist[key][i]))
        # och returnera en lista av alla dessa
        return currentEdges

    def neighbours(self,v):
        # nyckeln v i grannlistan innehåller alla dessa
        return self._adjlist[v]

    def add_edge(self,a,b):
        # med grannlista innebär detta bara att en nod 1 får en granne nod 2 och vice versa
        self._adjlist[a].append(b)
        self._adjlist[b].append(a)

    def add_vertex(self,a):
        # innebär med grannlista att en ny nyckel definieras helt utan grannar
        self._adjlist[a] = []

    def remove_vertex(self, v):
        # på liknande sätt kan den tas bort ur grannlistan på ett enkelt sätt
        self._adjlist.pop(v)

    def remove_edge(self, a, b):
        # att bort en kant blir också bara tvärt om mot att lägga till en sådan
        if a in self._adjlist[b]:
            self._adjlist[b].remove(a)
        if b in self._adjlist[a]:
            self._adjlist[a].remove(b)

    def is_directed(self):
        return self._isdirected

    def get_vertex_value(self, v):
        return self._valuelist[v]

    def set_vertex_value(self, v, x):
        self._valuelist[v] = x

dict = {
    "description": "Example planning task",
    "cube_width": "0.5",
    "start_position": [ -0.076138, -3.258682, 0.54373 ],
    "goal_position": [ 0.123147, 2.635865, 0.321962 ],
    "cube_positions:": [
        [ 0, 0, 0 ],
        [ 1.392833, -1.134785, -0.057927 ],
        [ 0.623421, -0.611667, 0.035734 ],
        [ 0.623421, -0.611667, 0.035734 ],
        [ 1.363971, 0.343793, 0.320316 ],
        [ 0.316483, 0.300595, 1.260739 ],
        [ 1.202368, 0.245398, -0.48468 ],
        [ 0.519638, -0.358962, 0.987094 ],
        [ 0.525473, -0.899843, 1.236658 ],
        [ 0.545793, -1.790587, 1.04849 ],
        [ 1.230509, -1.690394, 1.279423 ],
        [ 1.145733, -1.77694, 0.894679 ],
        [ 0.091837, -1.803752, 0.69827 ],
        [ -0.177067, -1.553129, -0.14017 ],
        [ 0.187531, 1.861265, 0.372158 ]
    ],
    "domain_lower_corner": [ -0.552, -3.929, -0.15 ],
    "domain_upper_corner": [ 1.448, 3.07, 1.349 ]
}

def cube_corners(cube_centers, half_size):
    nummer = range(len(cube_centers))
    cubes = {}
    for i, center in enumerate(cube_centers):
        cubes[i] = [[center[0]+half_size, center[1]+half_size, center[2]+half_size], [center[0]-half_size, center[1]+half_size, center[2]+half_size], [center[0]+half_size, center[1]-half_size, center[2]+half_size], [center[0]-half_size, center[1]-half_size, center[2]+half_size], [center[0]+half_size, center[1]+half_size, center[2]-half_size], [center[0]-half_size, center[1]+half_size, center[2]-half_size], [center[0]+half_size, center[1]-half_size, center[2]-half_size], [center[0]-half_size, center[1]-half_size, center[2]-half_size]]

    return cubes

def draw_cubes(cubes):
    
    
    
    fig.add_axes(ax)

    for i, cube in enumerate(cubes.values()):
        x = [corner[0] for corner in cube]#[:3]
        y = [corner[1] for corner in cube]#[:3]
        z = [corner[2] for corner in cube]#[:3]

        vertices = [[0, 1, 3, 2], [4, 5, 7, 6], [0, 1, 5, 4], [3, 2, 6, 7], [0, 2, 6, 4], [3, 1, 5, 7]]
        tupleList = list(zip(x, y, z))

        poly3d = [[tupleList[vertices[ix][iy]] for iy in range(len(vertices[0]))] for ix in range(len(vertices))]
        
        if i == 0:
            ax.add_collection3d(Poly3DCollection(poly3d, edgecolors='k', facecolors='green', linewidths=1, alpha=0.4))
        elif i == len(cubes.values()) - 1:
            ax.add_collection3d(Poly3DCollection(poly3d, edgecolors='k', facecolors='red', linewidths=1, alpha=0.4))
        else:
            ax.add_collection3d(Poly3DCollection(poly3d, edgecolors='k', facecolors='grey', linewidths=1))

    ax.set(xlim=(-3,3), ylim=(-3,3), zlim=(-2.3,2.3))
    
def collision(cube, point1, point2):
    retur = False
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
        #print('hej')
        retur = True

    high_y = point1[1] + (point2[1] - point1[1]) * high_t
    high_z = point1[2] + (point2[2] - point1[2]) * high_t
    if cube[2][1] <= high_y <= cube[0][1] and cube[4][2] <= high_z <= cube[0][2] and (point2[0] <= high_x <= point1[0] or point2[0] >= high_x >= point1[0]):
        #print('då')
        retur = True

    
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
        retur = True

    high_x = point1[0] + (point2[0] - point1[0]) * high_t
    high_z = point1[2] + (point2[2] - point1[2]) * high_t
    if cube[1][0] <= high_x <= cube[0][0] and cube[4][2] <= high_z <= cube[0][2] and (point2[1] <= high_y <= point1[1] or point2[1] >= high_y >= point1[1]):
        retur = True


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
        retur = True

    high_x = point1[0] + (point2[0] - point1[0]) * high_t
    high_y = point1[1] + (point2[1] - point1[1]) * high_t
    if cube[1][0] <= high_x <= cube[0][0] and cube[2][1] <= high_y <= cube[0][1] and (point2[2] <= high_z <= point1[1] or point2[2] >= high_z >= point1[2]):
        retur = True

    #print(retur)
    return retur

def dijkstra(graph, source, cost=lambda u,v: 1):
    # börja med att skapa ett tomt dictionary med plats för alla möjliga targets från en specifik source
    dijkdict = {}
    # alla vägar är tomma från början
    for key in graph.vertices():
        dijkdict[key] = {'path': [], 'cost': 0}

    # men vi vet att sources första plats är den själv
    dijkdict[source]['path'].append(source)

    # börja räkna vilka noder som besökts
    visited = {}
    # och initiera en lista som kan hålla koll på hur långt det är till varje stopp
    distances = {vertex: None for vertex in graph.vertices()}

    # varje nod ska ha värdet 'oändligheten' och source ska ha värdet '0' från början
    for value in distances.keys():
        if value != source:
            distances[value] = 1e12
        else:
            distances[value] = 0
    
    vertex = source
    # vi vill uppdatera alla vägar så länge det finns ett target vars väg inte är optimerad
    while len(distances.keys()) != 0:
        
        # måste först hitta till vilken nod det är kortast
        least_index = 1e12
        for key in distances:
            if distances[key] < least_index:
                least_index = distances[key]
        
        # initiera denna nod med ett namn som inte alls behöver representera en verklig nod
        vertex = 0
        # loopa genom alla noder som ska besökas
        for key in distances:
            # ta bort noden från noder som ska besökas ifall den ligger närmst möjligt och sätt denna nod som
            # den nod som just nu ska undersökas
            if distances[key] == least_index:
                vertex = key
                visited[key] = distances[key]
                distances.pop(key)
                break
        
        # loopa igenom alla grannar till den aktuella noden
        # om det nu finns ett snabbare sätt att ta sig till den noden, uppdatera vägen dit och den sammanlagda viket det tar att komma dit
        for neighbor in graph._adjlist[vertex]:
            if neighbor not in visited.keys():

                # säkerhetsanordning ifall det skulle uppstå oväntade problem
                if type(cost(vertex,neighbor)) != type(None):

                    # ny väg
                    # print(vertex, neighbor)
                    alt = visited[vertex] + cost(vertex,neighbor)

                    # om den är mindre, genom för det nämnda utbytet
                    if alt <= distances[neighbor]:
                        dijkdict[neighbor]['path'] = []
                        dijkdict[neighbor]['path'] = dijkdict[vertex]['path'].copy()
                        dijkdict[neighbor]['path'].append(neighbor)
                        distances[neighbor] = alt
    
    # formulera om dijdict så att den innehåller en väg och en sammanlagd vikt
    dijkdict = {key: {'path': dijkdict[key]['path'], 'cost': visited[key]} for key in dijkdict.keys()}
    return dijkdict

def weight(edge1, edge2):
    #print([x_samp[edge1], y_samp[edge1], z_samp[edge1]])
    #print([x_samp[edge2], y_samp[edge2], z_samp[edge2]])
    distance = ((z_samp[edge2] - z_samp[edge1])**2 + (y_samp[edge2] - y_samp[edge1])**2 + (x_samp[edge2] - x_samp[edge1])**2)**(1/2)
    return distance

#print('***')
#print(collision([[1,1,1],[-1,1,1],[1,-1,1],[-1,-1,1], [1,1,-1],[-1,1,-1],[1,-1,-1],[-1,-1,-1]], [10,0,0], [11,0,0]))
#print(collision([[1,1,1],[-1,1,1],[1,-1,1],[-1,-1,1], [1,1,-1],[-1,1,-1],[1,-1,-1],[-1,-1,-1]], [-10,0,0], [10,0,0]))
#print(collision([[1,1,1],[-1,1,1],[1,-1,1],[-1,-1,1], [1,1,-1],[-1,1,-1],[1,-1,-1],[-1,-1,-1]], [0,0,-10], [0,0,10]))


cube_centers = [dict['start_position']] + dict['cube_positions:'] + [dict['goal_position']]
hcw = 0.5*float(dict['cube_width']) # half cube width

cubes = cube_corners(cube_centers, 2*hcw)
show_cubes = cube_corners(cube_centers, hcw)
# print(cubes)
draw_cubes(show_cubes)

cubes.pop(list(cubes.keys())[-1])
cubes.pop(list(cubes.keys())[0])

# sample points
low_corner  = dict['domain_lower_corner']
high_corner = dict['domain_upper_corner']
x_samp = np.random.uniform(low = low_corner[0]+0.25, high = high_corner[0]-0.25, size = (400,))
y_samp = np.random.uniform(low = low_corner[1]+0.25, high = high_corner[1]-0.25, size = (400,))
z_samp = np.random.uniform(low = low_corner[2]+0.25, high = high_corner[2]-0.25, size = (400,))

# print(len(x_samp))

remove_index = []
for i in range(len(x_samp)):
    for cube in cubes.values():
        if cube[1][0] <= x_samp[i] <= cube[0][0] and cube[2][1] <= y_samp[i] <= cube[0][1] and cube[4][2] <= z_samp[i] <= cube[0][2]:
            remove_index.append(i)

x_samp = np.delete(x_samp, remove_index)
y_samp = np.delete(y_samp, remove_index)
z_samp = np.delete(z_samp, remove_index)

x_samp = np.append(x_samp, -0.076138)
y_samp = np.append(y_samp, -3.258682)
z_samp = np.append(z_samp, 0.54373)
x_samp = np.append(x_samp, 0.123147)
y_samp = np.append(y_samp, 2.635865)
z_samp = np.append(z_samp, 0.321962)

# print(len(x_samp))

#ax.scatter(x_samp, y_samp, z_samp)
# sample points slut

edges = []
for i in range(len(x_samp)):
    for j in range(len(x_samp)):
        if i != j:
            #print('*')
            krockar = False
            for cube in cubes.values():
                if collision(cube, [x_samp[i], y_samp[i], z_samp[i]], [x_samp[j], y_samp[j], z_samp[j]]):# and tuple([i,j]) not in edges and tuple([j,i]) not in edges:
                    krockar = True
                    break
                    
            if not (krockar) and tuple([i,j]) not in edges and tuple([j,i]) not in edges:
                edges.append(tuple([i,j]))


#print(edges)
G = Graph(start=edges)
#print(max(list(G._adjlist.keys())))
#print(len(x_samp)-2)
path = dijkstra(G, len(x_samp)-2, weight)
#print('---')
#print(path[len(x_samp)-1])
vag = path[len(x_samp)-1]['path']
print(vag)
#print(len(x_samp))
vag_kanter = [[x_samp[i] for i in vag], [y_samp[i] for i in vag], [z_samp[i] for i in vag]]

#print(vag_kanter)

# for edge in edges:
# x = [x_samp[edge[0]], x_samp[edge[1]]]
# y = [y_samp[edge[0]], y_samp[edge[1]]]
# z = [z_samp[edge[0]], z_samp[edge[1]]]

x = vag_kanter[0]
y = vag_kanter[1]
z = vag_kanter[2]

vertices = []
for i in range(len(x)-1):
    vertices.append([i, i+1])
#print(vertices)

tupleList = list(zip(x, y, z))

poly3d = [[tupleList[vertices[ix][iy]] for iy in range(len(vertices[0]))] for ix in range(len(vertices))]



ax.add_collection3d(Line3DCollection(poly3d, colors='k', linewidths=3,))

plt.show()


