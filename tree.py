class Tree():
    def __init__(self, start, goal):
        self.start = start
        self.goal = goal
        self.vertices = []
        self.edges = {}
        self.dists = {}

        self.add_vertex(start)

    def fill_distances(self, p1, p2, dist):
        self.dists[(p1, p2)] = dist
        self.dists[(p2, p1)] = dist

    def add_edge(self, x_near, x_new):
        self.add_vertex(x_near)
        self.add_vertex(x_new)

        if x_near in self.edges.keys():
            self.edges[x_near].append(x_new)
        else:
            self.edges[x_near] = [x_new]

        if x_new in self.edges.keys():
            self.edges[x_new].append(x_near)
        else:
            self.edges[x_new] = [x_near]

    def add_vertex(self, x):
        if x not in self.vertices:
            self.vertices.append(x)
