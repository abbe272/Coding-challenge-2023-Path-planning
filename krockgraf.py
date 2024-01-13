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


class WeightedGraph(Graph):
    # definiera en konstruktor likt den hos Graph
    def __init__(self, start = None, values = None, directed = False):
        # ett nytt unikt attribut är listan av vikter
        self._weightlist = {}
        
        # som initieras med None-värden om inget annat anges
        if start != None:
            for i in range(len(start)):
                self._weightlist[start[i]] = None
        
        if values == None:
            values = {}
        
        # de attribut som inte är unika för en viktad graf kan ärvas rakt ner
        super(WeightedGraph, self).__init__(start=start)

    def set_weight(self, a, b, weight):
        # samma som get_weight, men här tilldelas ett värde istället för att hämtas
        if (a, b) not in self._weightlist.keys():
            self._weightlist[(a, b)] = weight
        if (b, a) not in self._weightlist.keys():
            self._weightlist[(b, a)] = weight

    def get_weight(self, a, b):
        # fråga om kanten finns, om den gör det: returnera dess värde från vikt-dictionaryt
        if (a, b) in self._weightlist.keys():
            return self._weightlist[(a, b)]
        if (b, a) in self._weightlist.keys():
            return self._weightlist[(b, a)]

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
                    print(vertex, neighbor)
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

def visualize(G, view, nodecolors):
    graph = graphviz.Graph()
    vert = G.vertices()
    edgs = G.edges()

    # loopa genom alla noder - ifall de finns i listan av de som ska färgas, ge dem den färgen (orange)
    for i in range(len(vert)):
        if vert[i] in nodecolors.keys():
            graph.node(name=str(vert[i]), fillcolor = nodecolors[vert[i]], style = 'filled')
        else:
            graph.node(name=str(vert[i]), fillcolor = 'white', style = 'filled')

    # skapa alla kanter i grafen som ska ritas
    for i in range(len(edgs)):
        graph.edge(str(edgs[i][0]), str(edgs[i][1]))

    # rita grafen och öppna den automatiskt som en pdf i det förinställda pdf-programmet
    graph.render(view=view)

def view_shortest(G, s, t, cost=lambda u,v: 1):

    # plocka fram vägen av noder från source till target
    path = dijkstra(G, s, cost=cost)[t]['path']

    # skapa en dictionary med färgen orange för alla de noder som ingår i den sökta vägen
    colormap = {v: 'orange' for v in path}

    # och anropa visualiseringsfunktionen
    visualize(G, view=True, nodecolors=colormap)