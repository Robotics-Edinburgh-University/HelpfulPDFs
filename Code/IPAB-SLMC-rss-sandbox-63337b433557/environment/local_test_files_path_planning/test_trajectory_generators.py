import sys, os
import unittest
import matplotlib.pyplot as plt

import pprint
import numpy
from numpy import linalg


import copy
# Get path of to the toddler file... to use always relative paths
CURRENT_PATH = os.getcwd()


# import robot status modules
sys.path.insert(0, CURRENT_PATH + '/..')
import test_map_representation
import map_representaion


class traj:
    def __init__(self):

        self.traj = [((110, 113), numpy.array([5, 0])),
                     ((115, 113), numpy.array([5, 0])),
                     ((120, 113), numpy.array([5, 0])),
                     ((125, 113), numpy.array([5, 0])),
                     ((130, 113), numpy.array([5, 0])),
                     ((135, 113), numpy.array([5, 0])),
                     ((140, 113), numpy.array([5, 0])),
                     ((145, 113), numpy.array([5, 0])),
                     ((150, 113), numpy.array([5, 0])),
                     ((155, 113), numpy.array([5, 0])),
                     ((160, 113), numpy.array([5, 0])),
                     ((165, 113), numpy.array([5, 0])),
                     ((170, 113), numpy.array([5, 0])),
                     ((175, 113), numpy.array([5, 0])),
                     ((180, 113), numpy.array([5, 0])),
                     ((185, 113), numpy.array([5, 0])),
                     ((190, 113), numpy.array([5, 0])),
                     ((195, 113), numpy.array([5, 0])),
                     ((200, 113), numpy.array([0, 0])),
                     ((200, 113), numpy.array([0, 5])),
                     ((200, 118), numpy.array([0, 5])),
                     ((200, 123), numpy.array([0, 5])),
                     ((200, 128), numpy.array([0, 5])),
                     ((200, 133), numpy.array([0, 5])),
                     ((200, 138), numpy.array([0, 5])),
                     ((200, 143), numpy.array([2, 3])),
                     ((202, 146), numpy.array([5, 0])),
                     ((207, 146), numpy.array([5, 0])),
                     ((212, 146), numpy.array([5, 0])),
                     ((217, 146), numpy.array([5, 0])),
                     ((222, 146), numpy.array([5, 0])),
                     ((227, 146), numpy.array([5, 0])),
                     ((232, 146), numpy.array([5, 0])),
                     ((237, 146), numpy.array([5, 0])),
                     ((242, 146), numpy.array([5, 0])),
                     ((247, 146), numpy.array([5, 0])),
                     ((252, 146), numpy.array([5, 0])),
                     ((257, 146), numpy.array([5, 0])),
                     ((262, 146), numpy.array([5, 0])),
                     ((267, 146), numpy.array([5, 0])),
                     ((272, 146), numpy.array([5, 0])),
                     ((277, 146), numpy.array([5, 0])),
                     ((282, 146), numpy.array([5, 0])),
                     ((287, 146), numpy.array([0, 0]))]

    # - clear the array from duplicated sequential values
    def remove_adjacent(self, nums):
        return [a for a, b in zip(nums, nums[1:] + [not nums[-1]]) if a != b]

        # clear no perfect x or y vectors

    def remove_no_casual_values(self, all_info_path):
        for info_step in all_info_path:
            if abs(info_step[2]) != 5.0:
                all_info_path.remove(info_step)
        return all_info_path

        # compute the norm of all the step vectors

    def norms_per_step(self, relative_coord_path):
        norm_path = []
        for vector in relative_coord_path:
            norm_path.append(linalg.norm(vector))
        return norm_path

    # compute possible roation mats
    def compute_vector_rot_trans(self, step_path_list):

        rotations_list = []
        for i in xrange(len(step_path_list) - 1):
            start = step_path_list[i]
            end = step_path_list[i + 1]
            R = self.rotation_mat_between_2_vecs(start, end)
            rot_vec = numpy.dot(R, numpy.array([1., 0.]))
            rotations_list.append(rot_vec)
        return rotations_list

    # rotation matrix between 2 vectors
    def rotation_mat_between_2_vecs(self, vec1, vec2):

        vec1 = vec1 / linalg.norm(vec1)
        vec2 = vec2 / linalg.norm(vec2)

        x1 = vec1[0]
        y1 = vec1[1]
        x2 = vec2[0]
        y2 = vec2[1]

        R = numpy.array([[(x1 * x2 + y1 * y2), -(x1 * y2 - x2 * y1)], [(x1 * y2 - x2 * y1), (x1 * x2 + y1 * y2)]])

        return R


class traj_test(unittest.TestCase):
    def t_test_trajectory(self):
        tt = traj()
        global_points = list(zip(*tt.traj)[0])
        # print global_points

        clear_g_points = tt.remove_adjacent(global_points)
        # pprint.pprint(clear_g_points)

        relative_coord_path = []
        for i in xrange(len(clear_g_points) - 1):
            relative_coord_path.append(numpy.array(clear_g_points[i + 1]) - numpy.array(clear_g_points[i]))

        norm_path = tt.norms_per_step(relative_coord_path)
        all_info_path = zip(clear_g_points, relative_coord_path, norm_path)
        all_info_path = tt.remove_no_casual_values(all_info_path)

        # retrieve clear step list
        step_path_list = list(zip(*all_info_path)[1])

        # build the step commands!!! Ready
        rot_list = tt.compute_vector_rot_trans(step_path_list)
        rot_list = list(numpy.array(rot_list) * numpy.array([5., 0.001]))
        # pprint.pprint(rot_list)
        duplicated_list = []
        for index, every_command in enumerate(rot_list):
            if every_command[0] == 0.0:
                duplicated_list.append((index, every_command))
            # print duplicated_list
        for num_additions, copied in enumerate(duplicated_list):
            rot_list.insert(copied[0] + num_additions, copied[1])
        #pprint.pprint(rot_list)

        return rot_list
    # print "auto"

    def test_previw_trajectory(self):
        tt = traj()
        global_points = list(zip(*tt.traj)[0])
        mp = map_representaion.map_representation()
        tmp = test_map_representation.map_rep_test()
        coord_path = True

        #global_points = [(119.0, 94.0), (119.0, 99.0), (119.0, 104.0), (119.0, 109.0), (119.0, 114.0), (119.0, 119.0), (119.0, 124.0), (119.0, 129.0), (119.0, 134.0), (119.0, 139.0), (119.0, 144.0), (119.0, 149.0), (119.0, 154.0), (119.0, 159.0), (119.0, 164.0), (119.0, 169.0), (119.0, 174.0), (119.0, 179.0), (119.0, 179.0), (124.0, 179.0), (129.0, 179.0), (134.0, 179.0), (139.0, 179.0), (143, 179), (148, 179), (153, 179), (158, 179), (163, 179), (168, 179), (173, 179), (178, 179), (183, 179), (188, 179), (193, 179), (198, 179), (198, 179), (198, 174), (198, 169), (198, 164), (198, 159), (198, 154), (198, 149), (202, 146), (202, 146), (202, 141), (202, 136), (202, 131), (202, 126), (202, 121), (202, 116), (202, 111), (202, 106), (202, 101), (202, 96), (202, 91), (202, 86), (202, 81), (202, 76), (202, 71), (202, 66), (202, 61), (202, 56), (202, 51), (202, 46), (202, 41), (202, 36), (202, 31)]


         #exit d -- > b
        global_points = [(248, 370), (243, 370), (238, 370), (238, 370), (238, 365), (238, 360), (238, 355), (238, 350), (238, 345), (238, 340), (238, 335), (238, 330), (238, 325), (238, 320), (238, 320), (233, 320), (228, 320), (223, 320), (218, 320), (213, 320), (208, 320), (208, 320), (208, 315), (208, 310), (208, 305), (208, 300), (208, 295), (208, 290), (208, 285), (208, 280), (208, 275), (208, 270), (208, 265), (208, 260), (208, 255), (208, 250), (208, 245), (208, 240), (208, 235), (208, 230), (208, 225), (204, 224), (199, 224), (194, 224), (189, 224), (184, 224), (179, 224), (174, 224), (169, 224), (164, 224), (159, 224), (154, 224), (149, 224), (149, 224), (149, 219), (149, 214), (149, 209), (149, 204), (149, 199), (149, 194), (149, 189), (149, 184), (149, 179), (148, 179), (153, 179), (158, 179), (163, 179), (168, 179), (173, 179), (178, 179), (183, 179), (188, 179), (193, 179), (198, 179), (203, 179), (208, 179), (208, 179), (208, 174), (208, 169), (208, 164), (208, 159), (208, 154), (208, 149), (212, 146), (217, 146), (222, 146), (227, 146), (232, 146), (237, 146), (242, 146), (247, 146), (252, 146), (257, 146), (262, 146), (267, 146), (272, 146), (277, 146), (282, 146), (287, 146), (292, 146), (297, 146), (297, 146)]

        # exit e -- > e
        global_points =[(69, 343), (69, 348), (69, 353), (69, 353), (74, 353), (79, 353), (84, 353), (89, 353), (94, 353), (99, 353), (104, 353), (109, 353), (114, 353), (119, 353), (124, 353), (129, 353), (134, 353), (139, 353), (144, 353), (149, 353), (154, 353), (159, 353), (164, 353), (169, 353), (172, 356), (172, 351), (172, 346), (172, 341), (172, 336), (172, 331), (172, 326), (172, 321), (172, 316), (172, 316), (167, 316), (162, 316), (157, 316), (152, 316), (147, 316), (142, 316), (137, 316), (132, 316), (127, 316), (122, 316), (117, 316), (117, 313), (117, 308), (117, 303), (117, 298), (117, 293), (117, 288), (117, 283), (117, 278), (117, 273), (117, 268), (117, 263), (117, 258), (117, 253), (117, 248), (117, 243), (117, 238), (117, 233), (117, 228), (117, 223), (117, 218), (117, 213), (117, 213), (112, 213), (107, 213), (102, 213), (97, 213), (92, 213), (87, 213), (84, 213), (89, 213), (94, 213), (99, 213), (104, 213), (109, 213), (114, 213), (119, 213), (124, 213), (129, 213), (134, 213), (139, 213), (144, 213), (144, 213), (144, 208), (144, 203), (144, 198), (144, 193), (144, 188), (144, 183), (148, 179), (153, 179), (158, 179), (163, 179), (168, 179), (173, 179), (178, 179), (183, 179), (188, 179), (193, 179), (198, 179), (203, 179), (208, 179), (208, 179), (208, 174), (208, 169), (208, 164), (208, 159), (208, 154), (208, 149), (212, 146), (212, 146), (212, 141), (212, 136), (212, 131), (212, 126), (212, 121), (212, 116), (212, 111), (212, 106), (212, 101), (212, 96), (212, 91), (212, 86), (212, 81), (212, 76), (212, 71), (212, 66), (212, 61), (212, 56), (212, 51), (212, 46), (212, 41), (212, 36), (212, 31)]


        tmp.map_plot(mp,coord_path = coord_path ,plot_path = False, path = None,global_path = global_points)


    def t_test_preview_relative_trajectory(self):
        tt = traj()
        global_points = list(zip(*tt.traj)[0])
        mp = map_representaion.map_representation()
        tmp = test_map_representation.map_rep_test()
        coord_path = True

        # rotation mat of a position
        R = tt.rotation_mat_between_2_vecs(numpy.array([1,0,0]),numpy.array([0,1,0]))
        # rotate robot to proper pose
        print numpy.dot(numpy.linalg.inv(R),numpy.array([1.,0.]))

        relative = self.t_test_trajectory()
        rotated_list = []
        for i in relative:
            rotated_list.append(numpy.dot(R,i))

        pprint.pprint(zip(relative,rotated_list))

        tmp.map_plot(mp,coord_path = coord_path ,plot_path = False, path = None,global_path = rotated_list)


    def t_test_previw_map(self):
        tt = traj()
        mp = map_representaion.map_representation()
        tmp = test_map_representation.map_rep_test()
        tmp.map_plot(mp,coord_path = False ,plot_path = False, path = None,global_path = None)


    def t_test_command_trajectory(self):

        commnas_list =  [numpy.array([ 5.,  0.]), numpy.array([ 5.,  0.]), numpy.array([ 5.,  0.]), numpy.array([ 5.,  0.]), numpy.array([ 5.,  0.]), numpy.array([ 5.,  0.]), numpy.array([ 5.,  0.]), numpy.array([ 5.,  0.]), numpy.array([ 5.,  0.]), numpy.array([ 5.,  0.]), numpy.array([ 5.,  0.]), numpy.array([ 5.,  0.]), numpy.array([ 5.,  0.]), numpy.array([ 5.,  0.]), numpy.array([ 0.   ,  0.001]), numpy.array([ 5.,  0.]), numpy.array([ 5.,  0.]), numpy.array([ 5.,  0.]), numpy.array([ 5.,  0.]), numpy.array([ 5.,  0.]), numpy.array([ 5.,  0.]), numpy.array([ 5.,  0.]), numpy.array([ 5.,  0.]), numpy.array([ 5.,  0.]), numpy.array([ 0.   , -0.001]), numpy.array([ 5.,  0.]), numpy.array([ 5.,  0.]), numpy.array([ 5.,  0.]), numpy.array([ 5.,  0.]), numpy.array([ 5.,  0.]), numpy.array([ 5.,  0.]), numpy.array([ 5.,  0.]), numpy.array([ 5.,  0.]), numpy.array([ 5.,  0.]), numpy.array([ 5.,  0.]), numpy.array([ 0.   ,  0.001]), numpy.array([ 5.,  0.]), numpy.array([ 5.,  0.]), numpy.array([ 5.,  0.]), numpy.array([ 5.,  0.]), numpy.array([ 5.,  0.]), numpy.array([ 5.,  0.]), numpy.array([ 5.,  0.]), numpy.array([ 5.,  0.]), numpy.array([ 5.,  0.]), numpy.array([ 5.,  0.]), numpy.array([ 5.,  0.]), numpy.array([ 5.,  0.]), numpy.array([ 5.,  0.]), numpy.array([ 5.,  0.]), numpy.array([ 5.,  0.]), numpy.array([ 5.,  0.]), numpy.array([ 5.,  0.]), numpy.array([ 5.,  0.]), numpy.array([ 5.,  0.]), numpy.array([ 0.   , -0.001]), numpy.array([ 5.,  0.]), numpy.array([ 5.,  0.]), numpy.array([ 5.,  0.]), numpy.array([ 5.,  0.]), numpy.array([ 5.,  0.])]

        #pprint.pprint(commnas_list)
        indices = [i for i, x in enumerate(commnas_list) if x[0] == 0]
        indices_backup = copy.deepcopy(indices)
        indices.insert(0,0)
        indices.insert(len(indices),len(commnas_list))

        #print indices
        straight_commands = []
        for i in zip(indices, indices[1:]):
            straight_commands.append(commnas_list[i[0]+1:i[1]])
        #pprint.pprint(straight_commands)

        for line in straight_commands:
            additional_straight = int(len(line)*0.35)
            #print len(line)
            line += [numpy.array([ 5.,  0.])]*additional_straight
            #print len(line)
        #print len(straight_commands) , len(indices_backup)

        for index, i in enumerate(indices_backup):
            straight_commands.insert(index+1+index,[commnas_list[i]])
        #pprint.pprint(straight_commands)


        final_plus_commands = [item for sublist in straight_commands for item in sublist]
        pprint.pprint(final_plus_commands)

    def command_trajectory(self,commnas_list):

        commnas_list =  [numpy.array([ 5.,  0.]), numpy.array([ 5.,  0.]), numpy.array([ 5.,  0.]), numpy.array([ 5.,  0.]), numpy.array([ 5.,  0.]), numpy.array([ 5.,  0.]), numpy.array([ 5.,  0.]), numpy.array([ 5.,  0.]), numpy.array([ 5.,  0.]), numpy.array([ 5.,  0.]), numpy.array([ 5.,  0.]), numpy.array([ 5.,  0.]), numpy.array([ 5.,  0.]), numpy.array([ 5.,  0.]), numpy.array([ 0.   ,  0.001]), numpy.array([ 5.,  0.]), numpy.array([ 5.,  0.]), numpy.array([ 5.,  0.]), numpy.array([ 5.,  0.]), numpy.array([ 5.,  0.]), numpy.array([ 5.,  0.]), numpy.array([ 5.,  0.]), numpy.array([ 5.,  0.]), numpy.array([ 5.,  0.]), numpy.array([ 0.   , -0.001]), numpy.array([ 5.,  0.]), numpy.array([ 5.,  0.]), numpy.array([ 5.,  0.]), numpy.array([ 5.,  0.]), numpy.array([ 5.,  0.]), numpy.array([ 5.,  0.]), numpy.array([ 5.,  0.]), numpy.array([ 5.,  0.]), numpy.array([ 5.,  0.]), numpy.array([ 5.,  0.]), numpy.array([ 0.   ,  0.001]), numpy.array([ 5.,  0.]), numpy.array([ 5.,  0.]), numpy.array([ 5.,  0.]), numpy.array([ 5.,  0.]), numpy.array([ 5.,  0.]), numpy.array([ 5.,  0.]), numpy.array([ 5.,  0.]), numpy.array([ 5.,  0.]), numpy.array([ 5.,  0.]), numpy.array([ 5.,  0.]), numpy.array([ 5.,  0.]), numpy.array([ 5.,  0.]), numpy.array([ 5.,  0.]), numpy.array([ 5.,  0.]), numpy.array([ 5.,  0.]), numpy.array([ 5.,  0.]), numpy.array([ 5.,  0.]), numpy.array([ 5.,  0.]), numpy.array([ 5.,  0.]), numpy.array([ 0.   , -0.001]), numpy.array([ 5.,  0.]), numpy.array([ 5.,  0.]), numpy.array([ 5.,  0.]), numpy.array([ 5.,  0.]), numpy.array([ 5.,  0.])]

        # find the indices where the list hast to be cut
        indices = [i for i, x in enumerate(commnas_list) if x[0] == 0]
        indices_backup = copy.deepcopy(indices)
        indices.insert(0,0)
        indices.insert(len(indices),len(commnas_list))

        # build the sublists with the straight lines
        straight_commands = []
        for i in zip(indices, indices[1:]):
            straight_commands.append(commnas_list[i[0]+1:i[1]])

        # extend the lists appropriately * 0.3
        for line in straight_commands:
            additional_straight = int(len(line)*0.35)
            line += [numpy.array([ 5.,  0.])]*additional_straight

        # merge the staight line lists with the turns
        for index, i in enumerate(indices_backup):
            straight_commands.insert(index+1+index,[commnas_list[i]])

        # flatten the final list
        final_plus_commands = [item for sublist in straight_commands for item in sublist]

        #pprint.pprint(final_plus_commands)

        return final_plus_commands




if __name__ == '__main__':
    unittest.main()
