import numpy as np
import time
import matplotlib.pyplot as plt; plt.ion()
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import Planner
from pqdict import PQDict

def tic():
  return time.time()
def toc(tstart, nm=""):
  print('%s took: %s sec.\n' % (nm,(time.time() - tstart)))  

boundary = []
blocks = []
start = ()
goal = ()
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


def draw_map(boundary, blocks, start, goal):
  '''
  Visualization of a planning problem with environment boundary, obstacle blocks, and start and goal points
  '''
  fig = plt.figure()
  ax = fig.add_subplot(111, projection='3d')
  hb = draw_block_list(ax,blocks)
  hs = ax.plot(start[0:1],start[1:2],start[2:],'ro',markersize=7,markeredgecolor='k')
  hg = ax.plot(goal[0:1],goal[1:2],goal[2:],'go',markersize=7,markeredgecolor='k')  
  ax.set_xlabel('X')
  ax.set_ylabel('Y')
  ax.set_zlabel('Z')
  ax.set_xlim(boundary[0,0],boundary[0,3])
  ax.set_ylim(boundary[0,1],boundary[0,4])
  ax.set_zlim(boundary[0,2],boundary[0,5]) 
  return fig, ax, hb, hs, hg

def draw_block_list(ax,blocks):
  '''
  Subroutine used by draw_map() to display the environment blocks
  '''
  v = np.array([[0,0,0],[1,0,0],[1,1,0],[0,1,0],[0,0,1],[1,0,1],[1,1,1],[0,1,1]],dtype='float')
  f = np.array([[0,1,5,4],[1,2,6,5],[2,3,7,6],[3,0,4,7],[0,1,2,3],[4,5,6,7]])
  clr = blocks[:,6:]/255
  n = blocks.shape[0]
  d = blocks[:,3:6] - blocks[:,:3] 
  vl = np.zeros((8*n,3))
  fl = np.zeros((6*n,4),dtype='int64')
  fcl = np.zeros((6*n,3))
  for k in range(n):
    vl[k*8:(k+1)*8,:] = v * d[k] + blocks[k,:3]
    fl[k*6:(k+1)*6,:] = f + k*8
    fcl[k*6:(k+1)*6,:] = clr[k,:]
  
  if type(ax) is Poly3DCollection:
    ax.set_verts(vl[fl])
  else:
    pc = Poly3DCollection(vl[fl], alpha=0.25, linewidths=1, edgecolors='k')
    pc.set_facecolor(fcl)
    h = ax.add_collection3d(pc)
    return h


def runtest(mapfile, start, goal, verbose = True):
  '''
  This function:
   * load the provided mapfile
   * creates a motion planner
   * plans a path from start to goal
   * checks whether the path is collision free and reaches the goal
   * computes the path length as a sum of the Euclidean norm of the path segments
  '''
  global boundary
  global blocks

  # Load a map and instantiate a motion planner
  boundary, blocks = load_map(mapfile)

  # Intialize planner variables
  map_res = 0.1
  gridmap = voxel_map(boundary,blocks,map_res)
  open_pq = PQDict({})
  start_grid = get_grid_coord(start,boundary,map_res)
  goal_grid = get_grid_coord(goal,boundary,map_res)
  open_pq.additem(start_grid,0)
  g_matrix = gridmap.copy()
  g_matrix[:] = np.inf
  g_matrix[start_grid] = 0
  parent_nodes = {}
  
  # Call the planner
  t0 = tic()
  astar(open_pq,parent_nodes,g_matrix,gridmap,goal_grid)
  toc(t0,"Planning")

  min_bounds = [boundary[0][0],boundary[0][1],boundary[0][2]]
  # Get the path generated by the planner
  path = get_path(parent_nodes,goal_grid,min_bounds,map_res)
  # Display the environment
  if verbose:
    fig, ax, hb, hs, hg = draw_map(boundary, blocks, start, goal)  
  np.save('path_astar',path)
  # Plot the path
  if verbose:
    ax.plot(path[:,0],path[:,1],path[:,2],'r-')
  plt.show()
  collision = False
  goal_reached = sum((path[-1]-goal)**2) <= 0.1
  success = (not collision) and goal_reached
  pathlength = np.sum(np.sqrt(np.sum(np.diff(path,axis=0)**2,axis=1)))
  return success, pathlength

def voxel_map(boundary,blocks,map_res):

  x_range = np.ceil((boundary[0][3]-boundary[0][0])/map_res).astype(np.int)
  y_range = np.ceil((boundary[0][4]-boundary[0][1])/map_res).astype(np.int)
  z_range = np.ceil((boundary[0][5]-boundary[0][2])/map_res).astype(np.int)

  x_low = boundary[0][0]
  y_low = boundary[0][1]
  z_low = boundary[0][2]

  gridmap = np.zeros((x_range,y_range,z_range))

  for num in range(len(blocks)):

    this_block = blocks[num]
    x_min = np.ceil((this_block[0]-x_low)/map_res-1).astype(np.int)
    y_min = np.ceil((this_block[1]-y_low)/map_res-1).astype(np.int)
    z_min = np.ceil((this_block[2]-z_low)/map_res-1).astype(np.int)
    x_max = np.ceil((this_block[3]-x_low)/map_res+1).astype(np.int)
    y_max = np.ceil((this_block[4]-y_low)/map_res+1).astype(np.int)
    z_max = np.ceil((this_block[5]-z_low)/map_res+1).astype(np.int)

    x_min = max(min(x_min,x_range-1),0)
    y_min = max(min(y_min,y_range-1),0)
    z_min = max(min(z_min,z_range-1),0)
    gridmap[x_min:x_max,y_min:y_max,z_min:z_max] = np.inf

  return gridmap

def get_grid_coord(point_dec,boundary,map_res):

  x_low = boundary[0][0]
  y_low = boundary[0][1]
  z_low = boundary[0][2]

  x_idx = np.ceil((point_dec[0]-x_low)/map_res).astype(np.int)
  y_idx = np.ceil((point_dec[1]-y_low)/map_res).astype(np.int)
  z_idx = np.ceil((point_dec[2]-z_low)/map_res).astype(np.int)-1

  return (x_idx,y_idx,z_idx)

def visualize_3D(gridmap):

  x_max,y_max,z_max = gridmap.shape

  x_idx = np.arange(0,x_max)
  y_idx = np.arange(0,y_max)
  z_idx = np.arange(0,z_max)

  x_ids = np.tile(x_idx,(1,y_max*z_max)).reshape(1,-1,order='F')
  y_ids = np.tile(y_idx,(x_max,z_max)).reshape(1,-1,order='F')
  z_ids = np.tile(z_idx,(y_max*x_max,1)).reshape(1,-1,order='F')
  values = gridmap.reshape(1,-1,order='F')[0]

  fig = plt.figure()
  ax = fig.add_subplot(111, projection='3d')

  img = ax.scatter(x_ids, y_ids, z_ids, c=values, cmap=plt.hot(),alpha=0.05)
  fig.colorbar(img)
  plt.show()

def astar(open_pq,parent_nodes,g_matrix,gridmap,goal):

  max_nodes = 0
  start_time = time.time()
  while True:

    try:

      cost_c = open_pq.topitem()[1]
      current_pt = open_pq.pop()

    except KeyError:

      print('No paths found')
      break

    if (current_pt==goal):
      print('Path found successfully')
      break    

    max_nodes = neighbourhood_search(open_pq,current_pt,parent_nodes,g_matrix,gridmap,goal,max_nodes)

    # if (time.time()-start_time>1.0):

    #   visualize_tree(open_pq)
    #   start_time = time.time()
  
  print(f'Max nodes = {max_nodes}')
    
def neighbourhood_search(open_pq,current_pt,parent_nodes,g_matrix,gridmap,goal,max_nodes):

  epsilon = 1
  x_c,y_c,z_c = current_pt
  (x_max,y_max,z_max) = gridmap.shape
  if (len(open_pq)>max_nodes):
    max_nodes = len(open_pq)
  for i in range(-1,2):

    for j in range(-1,2):

      for k in range(-1,2):

        x_n,y_n,z_n = (x_c+i,y_c+j,z_c+k)
        next_pt = (x_n,y_n,z_n)

        # Check if point is within boundary

        if ((x_n>=0) and (y_n>=0) and (z_n>=0) and (x_n<x_max) and (y_n<y_max) and (z_n<z_max)): 

          # Check if this point is free
          if (gridmap[next_pt]==0):

            if (g_matrix[next_pt] > g_matrix[current_pt] + cost_f(next_pt,current_pt)):

              g_matrix[next_pt] = g_matrix[current_pt] + cost_f(next_pt,current_pt)
              parent_nodes[next_pt] = current_pt

              try:

                open_pq.updateitem(next_pt,g_matrix[next_pt]+epsilon*heuristic(next_pt,goal))

              except KeyError:

                open_pq.additem(next_pt,g_matrix[next_pt]+epsilon*heuristic(next_pt,goal))
  return max_nodes

def visualize_tree(open_pq):

  listOfCordi = [*open_pq]
  cordis = np.array(listOfCordi)
  cordis_xyz = cordis*0.2 + boundary[0][0:3]
  fig, ax, hb, hs, hg = draw_map(boundary, blocks, start, goal)
  ax.scatter(cordis_xyz[:,0],cordis_xyz[:,1],cordis_xyz[:,2],'r-')


  

def get_path(parent_nodes, goal,min_bounds,map_res):

  parent = parent_nodes[goal]
  path = []
  x_g,y_g,z_g = goal
  x_d = x_g*map_res+min_bounds[0]
  y_d = y_g*map_res+min_bounds[1]
  z_d = z_g*map_res+min_bounds[2]
  path.append((x_d,y_d,z_d))

  x_g,y_g,z_g = parent
  x_d = x_g*map_res+min_bounds[0]
  y_d = y_g*map_res+min_bounds[1]
  z_d = z_g*map_res+min_bounds[2]
  path.append((x_d,y_d,z_d)) 

  while parent in parent_nodes:

    x_g,y_g,z_g = parent_nodes[parent]
    x_d = x_g*map_res+min_bounds[0]
    y_d = y_g*map_res+min_bounds[1]
    z_d = z_g*map_res+min_bounds[2]
    path.append((x_d,y_d,z_d))
    parent = parent_nodes[parent]

  path = np.array(path)
  path = path[::-1]
  return np.array(path)


def cost_f(point1,point2):

  # 2-Norm
  x1,y1,z1 = point1
  x2,y2,z2 = point2

  return np.sqrt((x2-x1)**2+(y2-y1)**2+(z1-z2)**2)

def heuristic(point,goal):

  # 2-Norm
  x1,y1,z1 = point
  x2,y2,z2 = goal

  return np.sqrt((x2-x1)**2+(y2-y1)**2+(z1-z2)**2)




def test_single_cube(verbose = False):
  print('Running single cube test...\n') 
  start = np.array([2.3, 2.3, 1.3])
  goal = np.array([7.0, 7.0, 5.5])
  success, pathlength = runtest('./maps/single_cube.txt', start, goal, verbose)
  print('Success: %r'%success)
  print('Path length: %d'%pathlength)
  print('\n')
  
  
def test_maze(verbose = False):
  print('Running maze test...\n') 
  start = np.array([0.0, 0.0, 1.0])
  goal = np.array([12.0, 12.0, 5.0])
  success, pathlength = runtest('./maps/maze.txt', start, goal, verbose)
  print('Success: %r'%success)
  print('Path length: %d'%pathlength)
  print('\n')

    
def test_window(verbose = False):
  print('Running window test...\n') 
  start = np.array([0.2, -4.9, 0.2])
  goal = np.array([6.0, 18.0, 3.0])
  success, pathlength = runtest('./maps/window.txt', start, goal, verbose)
  print('Success: %r'%success)
  print('Path length: %d'%pathlength)
  print('\n')

  
def test_tower(verbose = False):
  print('Running tower test...\n') 
  start = np.array([2.5, 4.0, 0.5])
  goal = np.array([4.0, 2.5, 19.5])
  success, pathlength = runtest('./maps/tower.txt', start, goal, verbose)
  print('Success: %r'%success)
  print('Path length: %d'%pathlength)
  print('\n')

     
def test_flappy_bird(verbose = False):
  print('Running flappy bird test...\n') 
  start = np.array([0.5, 2.5, 5.5])
  goal = np.array([19.0, 2.5, 5.5])
  success, pathlength = runtest('./maps/flappy_bird.txt', start, goal, verbose)
  print('Success: %r'%success)
  print('Path length: %d'%pathlength) 
  print('\n')

  
def test_room(verbose = False):
  print('Running room test...\n') 
  start = np.array([1.0, 5.0, 1.5])
  goal = np.array([9.0, 7.0, 1.5])
  success, pathlength = runtest('./maps/room.txt', start, goal, verbose)
  print('Success: %r'%success)
  print('Path length: %d'%pathlength)
  print('\n')


def test_monza(verbose = False):
  global start
  global goal
  print('Running monza test...\n')
  start = np.array([0.5, 1.0, 4.9])
  goal = np.array([3.8, 1.0, 0.1])
  success, pathlength = runtest('./maps/monza.txt', start, goal, verbose)
  print('Success: %r'%success)
  print('Path length: %d'%pathlength)
  print('\n')


if __name__=="__main__":
  max_nodes = 0
  test_single_cube(True)
  # test_maze(True)
  # test_flappy_bird(True)
  # test_monza(True)
  # test_window(True)
  # test_tower(True)
  # test_room(True)








