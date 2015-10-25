import unittest
import matplotlib.pyplot as plt

import numpy
import map_representaion

# class map_rep_test(unittest.TestCase):

class map_rep_test():
    # plot the map with each waypoints
    def t_test_map_rep_funcs(self):

        mp = map_representaion.map_representation()
        self.map_plot(mp)

    # test the building of the walls as lines
    def t_test_map_build(self):

        mp = map_representaion.map_representation()
        mp.walls()

    # test the computation of the shortest path
    def test_shortest_path_solution(self):

        mp = map_representaion.map_representation()
        plot_path = True
        solution_path = mp.retrieve_shortest_path('B', 'D')
        # mp.map_plot(mp,plot_path,solution_path)
        trajectory = mp.path_coordinates(solution_path)
        # print trajectory
        trajectory_list = []
        for segments_index in xrange(len(trajectory) - 1):
            if (trajectory[segments_index])[0] <= (trajectory[segments_index + 1])[0]:
                xpoints = numpy.arange((trajectory[segments_index])[0], (trajectory[segments_index + 1])[0] + 1, 5)
            else:
                xpoints = numpy.arange((trajectory[segments_index])[0], (trajectory[segments_index + 1])[0] - 1, -5)
            if len(xpoints) == 0: xpoints = [(trajectory[segments_index])[0]]

            if (trajectory[segments_index])[1] < (trajectory[segments_index + 1])[1]:
                ypoints = numpy.arange((trajectory[segments_index])[1], (trajectory[segments_index + 1])[1] + 1, 5)
            else:
                ypoints = numpy.arange((trajectory[segments_index])[1], (trajectory[segments_index + 1])[1] - 1, -5)
            if len(ypoints) == 0: ypoints = [(trajectory[segments_index])[1]]

            # print xpoints
            # print ypoints
            traj = self.combine_interpolated_paths(xpoints, ypoints, 'y')
            trajectory_list += traj

        # print trajectory_list
        # raw_input("dddd")

    def combine_interpolated_paths(self, xpoints, ypoints, priority_axis):

        if priority_axis == 'x':

            ypart_x_traj = [ypoints[0]] * len(xpoints)
            first_trajectory = zip(xpoints, ypart_x_traj)
            xpart_y_traj = [xpoints[-1]] * len(ypoints)
            second_trajectory = zip(xpart_y_traj, ypoints)
            # print first_trajectory
            # print second_trajectory

        elif priority_axis == 'y':

            xpart_y_traj = [xpoints[0]] * len(ypoints)
            first_trajectory = zip(ypoints, xpart_y_traj)
            ypart_x_traj = [ypoints[-1]] * len(xpoints)
            second_trajectory = zip(xpoints, ypart_x_traj)

        else:
            print "Wrong axis was provided"
            first_trajectory = []
            second_trajectory = []

        return first_trajectory + second_trajectory

    # tesp the map
    def map_plot(self, mp, coord_path=False, plot_path=False, path=None, global_path=None):
        # robot_map = Map.MapRoom()
        # self.robot_map = robot_map
        self.mp = mp
        points = numpy.asarray(self.mp.robot_map.list_room_verteces).T

        # plot outer frame
        plt.plot(points[0, :], points[1, :], marker='o', linewidth=5, linestyle='-', color='k', label='Fence')

        # plot pilar
        if (self.mp.robot_map.obstacle != None):
            obstacle_points = numpy.asarray(self.mp.robot_map.obstacle.list_obstacle_verteces).T
            plt.plot(obstacle_points[0, :], obstacle_points[1, :], marker='+', linewidth=3, linestyle='-', color='b',
                     label='obstacle')

        # plot every room in red
        for room in self.mp.robot_map.room_list:

            room_points = numpy.asarray(room.list_room_verteces).T
            plt.plot(room_points[0, :], room_points[1, :], marker='*', linewidth=2, linestyle='-', color=room.color,
                     label=room.name)

            # plot obstacles in rooms
            if (room.obstacle != None):
                obstacle_points = numpy.asarray(room.obstacle.list_obstacle_verteces).T
                plt.plot(obstacle_points[0, :], obstacle_points[1, :], marker='+', linewidth=3, linestyle='-',
                         color='b', label='obstacle')

            # plot patches in rooms
            if (room.patch != None):
                patch_points = numpy.asarray(room.patch.list_patch_verteces).T
                plt.plot(patch_points[0, :], patch_points[1, :], marker='+', linewidth=5, linestyle='-', color='k',
                         label='patch')

            # waypoints = self.waypoint_computation()
            # waypoints_points = numpy.asarray(waypoints).T
        for waypoint in self.mp.robot_map.waypoints:
            plt.plot((waypoint['coord'])[0], (waypoint['coord'])[1], marker='D', color='c', label='waypoint')
            plt.annotate(waypoint['name'], ((waypoint['coord'])[0], (waypoint['coord'])[1]),
                         ((waypoint['coord'])[0] + 0.1, (waypoint['coord'])[1] - 0.1))


        # plot the graph of the waypoints
        plot_graph = False
        if plot_graph:
            for j in self.mp.robot_map.waypoints_as_graph.get_vertices():
                node_coord = (self.mp.robot_map.waypoints_as_graph.get_vertex(j)).coord
                adv = self.mp.robot_map.waypoints_as_graph.vert_dict[j].adjacent
                for i in adv:
                    # print "in " , i.id
                    neighboor_coord = i.coord
                    px = [node_coord[0], neighboor_coord[0]]
                    py = [node_coord[1], neighboor_coord[1]]
                    plt.plot(px, py)

        # plot connections of node
        plot_connections_node = False
        if plot_connections_node:
            j = 'Z'
            node_coord = (self.mp.robot_map.waypoints_as_graph.get_vertex(j)).coord
            adv = self.mp.robot_map.waypoints_as_graph.vert_dict[j].adjacent
            for i in adv:
                # print "in " , i.id
                neighboor_coord = i.coord
                px = [node_coord[0], neighboor_coord[0]]
                py = [node_coord[1], neighboor_coord[1]]
                plt.plot(px, py, color='r')

        # plot solution path
        if plot_path:
            linex = []
            liney = []
            for node in path:
                node_coord = (self.mp.robot_map.waypoints_as_graph.get_vertex(node)).coord
                linex.append(node_coord[0])
                liney.append(node_coord[1])
            plt.plot(linex, liney, color='r')

        # plot coord path
        if coord_path:
            linex = zip(*global_path)[0]
            liney = zip(*global_path)[1]
            plt.plot(linex, liney, color='b')

        plt.xlabel('X axis area')
        plt.ylabel('Y axis area')
        plt.title('Robot map')
        plt.xlim(-100, 600)  # 430)
        plt.ylim(-100, 600)
        # plt.legend()
        plt.show()


if __name__ == '__main__':
    unittest.main()
