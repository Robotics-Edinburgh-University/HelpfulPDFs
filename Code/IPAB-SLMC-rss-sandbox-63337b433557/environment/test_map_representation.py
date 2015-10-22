

import unittest

import map_representaion

class map_rep_test(unittest.TestCase):
    
    # plot the map with each waypoints
    def t_test_map_rep_funcs(self):
        
        mp = map_representaion.map_representation()
        mp.map_plot()
    
    # test the building of the walls as lines
    def t_test_map_build(self):
        
        mp = map_representaion.map_representation()
        mp.walls()
    
    
    # test the computation of the shortest path
    def test_shortest_path_solution(self):
        
        mp = map_representaion.map_representation()
        plot_path = True
        solution_path = mp.retrieve_shortest_path('B','E')
        #mp.map_plot(plot_path,solution_path)
        print mp.waypoint_interpolation(solution_path)
      
if __name__ == '__main__':
        
    unittest.main() 