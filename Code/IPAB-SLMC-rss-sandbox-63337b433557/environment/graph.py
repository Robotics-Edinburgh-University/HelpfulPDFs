import numpy
import numpy.linalg

import heapq

import sys


class Vertex:
    def __init__(self, node, coord):
        self.id = node
        self.adjacent = {}
        self.coord = coord
        # Set distance to infinity for all nodes
        self.distance = sys.maxint
        # Mark all nodes unvisited        
        self.visited = False
        # Predecessor
        self.previous = None

    def add_neighbor(self, neighbor, weight=0):
        self.adjacent[neighbor] = weight

    def get_connections(self):
        return self.adjacent.keys()

    def get_id(self):
        return self.id

    def get_weight(self, neighbor):
        return self.adjacent[neighbor]

    def set_distance(self, dist):
        self.distance = dist

    def get_distance(self):
        return self.distance

    def set_previous(self, prev):
        self.previous = prev

    def set_visited(self):
        self.visited = True

    def __str__(self):
        return str(self.id) + ' adjacent: ' + str([x.id for x in self.adjacent])


class Graph:
    def __init__(self):
        self.vert_dict = {}
        self.num_vertices = 0
        self.geometry_2D = geometry_2D()

    def __iter__(self):
        return iter(self.vert_dict.values())

    def add_vertex(self, node, coord):
        self.num_vertices = self.num_vertices + 1
        new_vertex = Vertex(node, coord)
        self.vert_dict[node] = new_vertex
        return new_vertex

    def get_vertex(self, n):
        if n in self.vert_dict:
            return self.vert_dict[n]
        else:
            return None

    # add an edge in the graph only and only if it does not pass through a wall	 
    def filter_edge_addition(self, frm, to, all_walls):

        ver_frm = self.vert_dict.get(frm)
        ver_to = self.vert_dict.get(to)

        # build the line segment of the potential edge
        line_segment = [ver_frm.coord, ver_to.coord]

        # check if the line intersect with any walls
        collision = False
        for wall in all_walls:
            if self.geometry_2D.intersect(line_segment, wall):
                collision = True
                break

        # if not connect nodes
        if collision == False:
            self.add_edge(frm, to)

    def add_edge(self, frm, to):
        ver_frm = self.vert_dict.get(frm)
        ver_to = self.vert_dict.get(to)

        cost = numpy.linalg.norm(numpy.array(ver_frm.coord) - numpy.array(ver_to.coord))

        # optimised cost for the current waypoints !!! 
        # NOT a general solution  for having min sufficient number of solutions
        if cost > 0.0 and cost < 130.0:
            if frm not in self.vert_dict:
                self.add_vertex(frm)
            if to not in self.vert_dict:
                self.add_vertex(to)

            self.vert_dict[frm].add_neighbor(self.vert_dict[to], cost)
            self.vert_dict[to].add_neighbor(self.vert_dict[frm], cost)

    def get_vertices(self):
        return self.vert_dict.keys()

    def set_previous(self, current):
        self.previous = current

    def get_previous(self, current):
        return self.previous

    # find shortest path to goal	    
    def shortest(self, v, path):
        ''' make shortest path from v.previous'''
        if v.previous:
            path.append(v.previous.get_id())
            self.shortest(v.previous, path)
        return

        # Path finder algorithm

    # http://www.bogotobogo.com/python/python_Dijkstras_Shortest_Path_Algorithm.php
    def dijkstra(self, start, target):

        # print '''Dijkstra's shortest path'''
        # Set the distance for the start node to zero
        start.set_distance(0)

        # Put tuple pair into the priority queue
        unvisited_queue = [(v.get_distance(), v) for v in self]
        heapq.heapify(unvisited_queue)

        while len(unvisited_queue):
            # Pops a vertex with the smallest distance
            uv = heapq.heappop(unvisited_queue)
            current = uv[1]
            current.set_visited()

            # for next in v.adjacent:
            for next in current.adjacent:
                # if visited, skip
                if next.visited:
                    continue
                new_dist = current.get_distance() + current.get_weight(next)

                if new_dist < next.get_distance():
                    next.set_distance(new_dist)
                    next.set_previous(current)
                    # print 'updated : current = %s next = %s new_dist = %s' \
                    #	    %(current.get_id(), next.get_id(), next.get_distance())
                    # else:
                    # print 'not updated : current = %s next = %s new_dist = %s' \
                    #	    %(current.get_id(), next.get_id(), next.get_distance())

            # Rebuild heap
            # 1. Pop every item
            while len(unvisited_queue):
                heapq.heappop(unvisited_queue)
            # 2. Put all vertices not visited into the queue
            unvisited_queue = [(v.get_distance(), v) for v in self if not v.visited]
            heapq.heapify(unvisited_queue)


        # Class for computation of geometrical distances and line intersection detection


class geometry_2D:
    def ccw(self, A, B, C):
        return (C[1] - A[1]) * (B[0] - A[0]) > (B[1] - A[1]) * (C[0] - A[0])

    # takes line segment [p1,p2] where p = [x,y]
    # Return true if line segments line1 and line2 intersect
    def intersect(self, line1, line2):
        line1_p1 = line1[0]
        line1_p2 = line1[1]
        line2_p1 = line2[0]
        line2_p2 = line2[1]

        return self.ccw(line1_p1, line2_p1, line2_p2) != self.ccw(line1_p2, line2_p1, line2_p2) and self.ccw(line1_p1,
                                                                                                             line1_p2,
                                                                                                             line2_p1) != self.ccw(
            line1_p1, line1_p2, line2_p2)
