from ompl import util as ou
from ompl import base as ob
from ompl import geometric as og
import numpy as np
from pyrr import *


# Function taken from pyrr and modified to avoid NaN error
def ray_intersect_aabb(ray, aabb):
	"""Calculates the intersection point of a ray and an AABB
	:param numpy.array ray1: The ray to check.
	:param numpy.array aabb: The Axis-Aligned Bounding Box to check against.
	:rtype: numpy.array
	:return: Returns a vector if an intersection occurs.
		Returns None if no intersection occurs.
	"""
	"""
	http://gamedev.stackexchange.com/questions/18436/most-efficient-aabb-vs-ray-collision-algorithms
	"""
	# this is basically "numpy.divide( 1.0, ray[ 1 ] )"
	# except we're trying to avoid a divide by zero warning
	# so where the ray direction value is 0.0, just use infinity
	# which is what we want anyway
	direction = ray[1]
	dir_fraction = np.empty(3, dtype = ray.dtype)
	dir_fraction[direction == 0.0] = 1e16
	dir_fraction[direction != 0.0] = np.divide(1.0, direction[direction != 0.0])

	t1 = (aabb[0,0] - ray[0,0]) * dir_fraction[ 0 ]
	t2 = (aabb[1,0] - ray[0,0]) * dir_fraction[ 0 ]
	t3 = (aabb[0,1] - ray[0,1]) * dir_fraction[ 1 ]
	t4 = (aabb[1,1] - ray[0,1]) * dir_fraction[ 1 ]
	t5 = (aabb[0,2] - ray[0,2]) * dir_fraction[ 2 ]
	t6 = (aabb[1,2] - ray[0,2]) * dir_fraction[ 2 ]

	tmin = max(min(t1, t2), min(t3, t4), min(t5, t6))
	tmax = min(max(t1, t2), max(t3, t4), max(t5, t6))

	# if tmax < 0, ray (line) is intersecting AABB
	# but the whole AABB is behind the ray start
	if tmax < 0:
		return None

	# if tmin > tmax, ray doesn't intersect AABB
	if tmin > tmax:
		return None

	# t is the distance from the ray point
	# to intersection

	t = min(x for x in [tmin, tmax] if x >= 0)
	point = ray[0] + (ray[1] * t)
	return tuple(point)

def load_map(fname):
    '''
    Loads the bounady and blocks from map file fname.

    boundary = [['xmin', 'ymin', 'zmin', 'xmax', 'ymax', 'zmax','r','g','b']]

    blocks = [['xmin', 'ymin', 'zmin', 'xmax', 'ymax', 'zmax','r','g','b'],
            ...,
            ['xmin', 'ymin', 'zmin', 'xmax', 'ymax', 'zmax','r','g','b']]
    '''
    mapdata = np.loadtxt(fname,dtype={'names': ('type', 'xmin', 'ymin', 'zmin', 'xmax', 'ymax', 'zmax','r','g','b'),\
                                    'formats': ('S8','f', 'f', 'f', 'f', 'f', 'f', 'f','f','f')})
    blockIdx = mapdata['type'] == b'block'
    boundary = mapdata[~blockIdx][['xmin', 'ymin', 'zmin', 'xmax', 'ymax', 'zmax','r','g','b']].view('<f4').reshape(-1,11)[:,2:]
    blocks = mapdata[blockIdx][['xmin', 'ymin', 'zmin', 'xmax', 'ymax', 'zmax','r','g','b']].view('<f4').reshape(-1,11)[:,2:]
    return boundary, blocks

class ValidityChecker(ob.StateValidityChecker):
     # Returns whether the given state's position is
     # is collision free or not

    def isValid(self, state):

        if( state[0] < boundary[0,0] or state[0] > boundary[0,3] or \
            state[1] < boundary[0,1] or state[1] > boundary[0,4] or \
            state[2] < boundary[0,2] or state[2] > boundary[0,5] ):
            
            valid = False

        else:

            valid = True
            for k in range(blocks.shape[0]):
                if( state[0] > blocks[k,0] and state[0] < blocks[k,3] and\
                    state[1] > blocks[k,1] and state[1] < blocks[k,4] and\
                    state[2] > blocks[k,2] and state[2] < blocks[k,5] ):
                    
                    valid = False
                    break         
        return valid

class MyMotionValidator(ob.MotionValidator):
    def __init__(self, si):
        super(MyMotionValidator, self).__init__(si)
    def checkMotion(self, s1, s2):
    
        point1 = np.array([s1[0],s1[1],s1[2]])
        point2 = np.array([s2[0],s2[1],s2[2]])
        line_seg = np.array([point1,point2])
        ray_c = ray.create_from_line(line_seg)
        
        for i in range(len(blocks)):

            aabb_box = aabb.create_from_bounds(blocks[i][0:3],blocks[i][3:6])
            intersection_vect = ray_intersect_aabb(ray_c,aabb_box)

            if intersection_vect is not None:
                
                if ((min(point1[0],point2[0]) <= intersection_vect[0] <= max(point1[0],point2[0])) and \
                    (min(point1[1],point2[1]) <= intersection_vect[1] <= max(point1[1],point2[1])) and \
                    (min(point1[2],point2[2]) <= intersection_vect[2] <= max(point1[2],point2[2])) ):
                    return False

        return True

# def checkMotion_check( s1, s2):

#     point1 = np.array([s1[0],s1[1],s1[2]])
#     point2 = np.array([s2[0],s2[1],s2[2]])
#     line_seg = np.array([point1,point2])
#     ray_c = ray.create_from_line(line_seg)
    
#     for i in range(len(blocks)):

#         aabb_box = aabb.create_from_bounds(blocks[i][0:3],blocks[i][3:6])
#         intersection_vect = ray_intersect_aabb(ray_c,aabb_box)

#         if intersection_vect is not None:
            
#             print(intersection_vect)
#             is_point_on_seg = point_intersect_line_segment(intersection_vect,line_seg)
#             if is_point_on_seg is not None:
#                 print(is_point_on_seg)
#                 return False

#             if ((min(point1[0],point2[0]) <= intersection_vect[0] <= max(point1[0],point2[0])) and \
#                 (min(point1[1],point2[1]) <= intersection_vect[1] <= max(point1[1],point2[1])) and \
#                 (min(point1[2],point2[2]) <= intersection_vect[2] <= max(point1[2],point2[2])) ):
#                 return False

#     return True

def planner_ompl():

    runTime = 1.0

    # Construct the robot state space in which we're planning
    space = ob.RealVectorStateSpace(3)

    # Set the bounds of space
    bounds = ob.RealVectorBounds(3)
    bounds.setLow(0,np.double(boundary[0][0]))
    bounds.setLow(1,np.double(boundary[0][1]))
    bounds.setLow(2,np.double(boundary[0][2]))
    bounds.setHigh(0,np.double(boundary[0][3]))
    bounds.setHigh(1,np.double(boundary[0][4]))
    bounds.setHigh(2,np.double(boundary[0][5]))
    bounds.check()
    space.setBounds(bounds)

    # Construct a space information instance for this state space
    si = ob.SpaceInformation(space)
    
    # Set the object used to check which states in the space are valid
    validityChecker = ValidityChecker(si)
    si.setStateValidityChecker(validityChecker)

    # Set the object used to check which motions are valid
    mv = MyMotionValidator(si)
    si.setMotionValidator(mv)
    
    si.setup()

    # Set our robot's starting state
    start = ob.State(space)
    start[0] = start_env[0]
    start[1] = start_env[1]
    start[2] = start_env[2]

    # Set our robot's goal state
    goal = ob.State(space)
    goal[0] = goal_env[0]
    goal[1] = goal_env[1]
    goal[2] = goal_env[2]

    # Create a problem instance
    pdef = ob.ProblemDefinition(si)

    # Set the start and goal states
    pdef.setStartAndGoalStates(start, goal)

    # Create the optimization objective (Path Length Chosen here)
    pdef.setOptimizationObjective(ob.PathLengthOptimizationObjective(si))

    # Construct the optimal planner
    planner = og.RRTstar(si)
    planner.setRange(1)
    optimizingPlanner = planner

    # Set the problem instance for our planner to solve
    optimizingPlanner.setProblemDefinition(pdef)
    optimizingPlanner.setup()
    totalTime = 0

    # attempt to solve the planning problem in the given runtime
    while not pdef.hasExactSolution():
        
        solved = optimizingPlanner.solve(runTime)
        totalTime = totalTime + runTime

    fname = 'path.txt'
    with open(fname, 'w') as outFile:
        outFile.write(pdef.getSolutionPath().printAsMatrix())
    print(f'Planning time = {totalTime}')

def test_single_cube():

    global boundary,blocks,start_env,goal_env
    boundary, blocks = load_map('./maps/single_cube.txt')
    start_env = np.array([2.3, 2.3, 1.3])
    goal_env = np.array([7.0, 7.0, 5.5])
    planner_ompl()


def test_window():

    global boundary,blocks,start_env,goal_env
    boundary, blocks = load_map('./maps/window.txt')
    start_env = np.array([0.2, -4.9, 0.2])
    goal_env = np.array([6.0, 18.0, 3.0])
    planner_ompl()



def test_monza():

    global boundary,blocks,start_env,goal_env
    boundary, blocks = load_map('./maps/monza.txt')
    start_env = np.array([0.5, 1.0, 4.9])
    goal_env = np.array([3.8, 1.0, 0.1])
    planner_ompl()



def test_flappy_bird():

    global boundary,blocks,start_env,goal_env
    boundary, blocks = load_map('./maps/flappy_bird.txt')
    start_env = np.array([0.5, 2.5, 5.5])
    goal_env = np.array([19.0, 2.5, 5.5])
    planner_ompl()



def test_maze():

    global boundary,blocks,start_env,goal_env
    boundary, blocks = load_map('./maps/maze.txt')
    start_env = np.array([0.0, 0.0, 1.0])
    goal_env = np.array([12.0, 12.0, 5.0])
    planner_ompl()



def test_room():

    global boundary,blocks,start_env,goal_env
    boundary, blocks = load_map('./maps/room.txt')
    start_env = np.array([1.0, 5.0, 1.5])
    goal_env = np.array([9.0, 7.0, 1.5])
    planner_ompl()


def test_tower():
    
    global boundary,blocks,start_env,goal_env
    boundary, blocks = load_map('./maps/tower.txt')
    start_env = np.array([2.5, 4.0, 0.5])
    goal_env = np.array([4.0, 2.5, 19.5])
    planner_ompl()


if __name__=="__main__":

    # test_single_cube()
    test_flappy_bird()
    # test_monza()
    # test_window()
    # test_tower()
    # test_room()
    # test_maze()
    
