import sys, os	
import unittest
import matplotlib.pyplot as plt

import pprint
import numpy 
from numpy import linalg


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
    def remove_adjacent(self,nums):
     return [a for a,b in zip(nums, nums[1:]+[not nums[-1]]) if a != b]   
    
    # clear no perfect x or y vectors
    def remove_no_casual_values(self,all_info_path):
	for info_step in all_info_path:
	    if abs(info_step[2]) != 5.0:
	       all_info_path.remove(info_step)
	return all_info_path    
      
    # compute the norm of all the step vectors
    def norms_per_step(self,relative_coord_path):
        norm_path = []
        for vector in relative_coord_path:
	    norm_path.append(linalg.norm(vector))
	return norm_path
     
    # compute possible roation mats
    def compute_vector_rot_trans(self,step_path_list):
        
        rotations_list = []
        for i in xrange(len(step_path_list)-1): 
	    start = step_path_list[i]  
            end =  step_path_list[i+1]
	    R = self.rotation_mat_between_2_vecs(start,end) 
	    rot_vec = numpy.dot(R, numpy.array([1.,0.]))
	    rotations_list.append(rot_vec)
        return rotations_list
       
    # rotation matrix between 2 vectors
    def rotation_mat_between_2_vecs(self,vec1,vec2):
        
        vec1 = vec1/linalg.norm(vec1)
        vec2 = vec2/linalg.norm(vec2)
        
        x1 = vec1[0]
        y1 = vec1[1]
        x2 = vec2[0]
        y2 = vec2[1]
        
        R = numpy.array([[(x1*x2+y1*y2 ), -(x1*y2-x2*y1)],[ (x1*y2-x2*y1), (x1*x2+y1*y2)] ])
        
        return R 
        
class traj_test(unittest.TestCase):

    def test_trajectory(self):
        tt = traj()
        global_points = list(zip(*tt.traj)[0])
        #print global_points
        
        clear_g_points = tt.remove_adjacent(global_points)
        #pprint.pprint(clear_g_points)
        
        relative_coord_path = []
	for i in xrange(len(clear_g_points)-1): 
	    relative_coord_path.append(numpy.array(clear_g_points[i+1]) - numpy.array(clear_g_points[i])) 
        
        norm_path = tt.norms_per_step(relative_coord_path)
	all_info_path = zip(clear_g_points,relative_coord_path,norm_path)
	all_info_path = tt.remove_no_casual_values(all_info_path)
	 
	# retrieve clear step list 
	step_path_list = list(zip(*all_info_path)[1])
        
        # build the step commands!!! Ready
        rot_list = tt.compute_vector_rot_trans(step_path_list)
        rot_list = list(numpy.array(rot_list) * numpy.array([5.,0.001]))
        #pprint.pprint(rot_list)
        duplicated_list = []
        for index,every_command in enumerate(rot_list):
	    if every_command[0] == 0.0:
	        duplicated_list.append((index,every_command))
	#print duplicated_list       
	for num_additions,copied in enumerate(duplicated_list):
	    rot_list.insert(copied[0]+num_additions,copied[1])
        pprint.pprint(rot_list)
        #print "auto"
        
    def t_test_previw_trajectory(self):
        tt = traj()
        global_points = list(zip(*tt.traj)[0])
        mp = map_representaion.map_representation()
        tmp = test_map_representation.map_rep_test()
        coord_path = True
        #tmp.map_plot(mp,coord_path = coord_path ,plot_path = False, path = None,global_path = global_points)
        
        
if __name__ == '__main__':
        
    unittest.main() 