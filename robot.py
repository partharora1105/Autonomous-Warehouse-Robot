import random

from grid import *
from helpers import *
import math

class Robot(object):
    # data members
    x = "X coordinate in world frame"
    y = "Y coordinate in world frame"
    h = "Heading angle in world frame in degree. h = 0 when robot's head (camera) points to positive X"
    wheel_r = "Radius of wheels of robot"
    wheel_dist = "Distance between wheels of robot"
    TIMESTEP = "timestep"
    path = 'rrt path'

    # functions members
    def __init__(self, x, y, heading=None, wheel_dist=0.5, wheel_r = 1):
        if heading is None:
            heading = random.uniform(0, 360)
        self.__x = x
        self.__y = y
        self.__h = heading % 360
        self.wheel_dist = wheel_dist
        self.__TIMESTEP = 1
        
        self.wheel_r = wheel_r
        self.explored_cells = {(x, y)}

        self.next_coord = None
        self.path = []
        self.markers_found_or_picked = []
        self.curr_marker = None
        # self.vr = 0
        # self.vl = 0


    def __repr__(self):
        return "(x = %f, y = %f, heading = %f deg)" % (self.__x, self.__y, self.__h)
    
    @property
    def TIMESTEP(self):
        return self.__TIMESTEP
        
    @property
    def x(self):
        return self.__x
    
    @property
    def y(self):
        return self.__y
    
    @property
    def h(self):
        return self.__h % 360
    
    @property
    def xy(self):
        return self.__x, self.__y

    @property
    def xyh(self):
        return self.__x, self.__y, self.__h
    

    def chose_random_heading(self):
        return random.uniform(0, 360)
    

    def get_cells_in_fov(self, grid, dist=2):
        """ Get list of grid cells that are in FOV of robot

            Arguments:
            grid -- grid to list cells from
            dist -- range of FOV

            Return: List of visible grid cells
        """
        block_list = []
        r_x, r_y = self.__x, self.__y
        for x in range(math.floor(r_x - dist), math.ceil(r_x + dist + 1)):
            for y in range(math.floor(r_y - dist), math.ceil(r_y + dist + 1)):
                x_dis = r_x - x - 1 if x <= r_x else r_x - x
                y_dis = r_y - y - 1 if y <= r_y else r_y - y
                if math.sqrt(x_dis**2 + y_dis**2) > dist:
                    continue
                if grid.is_in(x, y) and (x != self.__x or y != self.__y):
                    block_list.append((x, y))
                    self.explored_cells.add((x, y))
        return block_list
    

    def get_obstacles_in_fov(self, grid, dist=2):
        """ Get list of obstacles that are in FOV of robot

            Arguments:
            grid -- grid to list cells from
            dist -- range of FOV

            Return: List of visible cells occupied by obstacles
        """
        obstacle_list = []
        visible_grid_cells = self.get_cells_in_fov(grid, dist)
        for (x, y) in visible_grid_cells:
            if grid.is_in(x, y) and grid.is_occupied(x, y):
                obstacle_list.append((x, y))
        return obstacle_list
    

    def get_free_cells_in_fov(self, grid, dist=2):
        """ Get list of free grid cells that are in FOV of robot

            Arguments:
            grid -- grid to list cells from
            dist -- range of FOV

            Return: List of visible cells that are free
        """
        free_cells = []
        obstacle_list = self.get_obstacles_in_fov(grid, dist)
        visible_grid_cells = self.get_cells_in_fov(grid, dist)

        for (x, y) in visible_grid_cells:
            if (x, y) in obstacle_list:
                continue
            free_cells.append((x, y))
        return free_cells
    

    def read_marker_around(self, grid, dis=2):
        """ 
        Get list of markers in robot's FOV
        Arguments:
            grid -- grid to list cells from
            dist -- range of FOV

        Return: List of markers around
        """
        marker_list = []
        free_block = set(self.get_cells_in_fov(grid, dis))
        for marker in grid.markers:
            m_x, m_y, m_h = marker
            if (m_x, m_y) in free_block:
                marker_list.append((m_x, m_y, m_h))
        return marker_list
    
    def pickup_marker(self, grid, dis=1):
        """ 
        Get list of markers in robot's FOV
        Arguments:
            grid -- grid to list cells from
            dist -- range of FOV

        Return: List of markers around
        """
        marker_list = []
        free_block = set(self.get_cells_in_fov(grid, dis))
        for marker in grid.markers:
            m_x, m_y, m_h = parse_marker_info(marker[0], marker[1], marker[2])      
            if (m_x, m_y) in free_block:                
                if abs(float(m_h)-self.h) <= 10:
                    print("marker heading")
                    marker_list.append(marker)
        if len(self.markers_found_or_picked) != len(set(self.markers_found_or_picked).union(set(marker_list))):
            self.markers_found_or_picked = list(set(self.markers_found_or_picked).union(set(marker_list)))
            fname = grid.fname
            print(f'{fname} found {len(self.markers_found_or_picked)}/{grid.LANDMARKS_TOTAL} markers')
        return

    def marker_already_picked(self, marker):
        if marker in self.markers_found_or_picked:
            return True
        else:
            return False


    def move_diff_drive(self, grid, vl, vr, dt):
        """ Move the robot with a steering angle and diff drive forward.
            Note that the distance between the wheels is 0.5

            Arguments:
            dvl -- velocity to set of left wheel
            dvr -- velocity to set of right wheel

            No return
        """
        v = (vl+vr) * self.wheel_r/2
        w = (vr-vl) * self.wheel_r/self.wheel_dist
        self.__h += math.degrees(w)*dt
        
        h_rad = math.radians(self.__h)
        dx = v * math.cos(h_rad) * dt
        dy = v * math.sin(h_rad) * dt

        # Check if theres a collision along path
        m, c = find_line((self.__x, self.__y), (self.__x+dx, self.__y+dy))
        if m == math.inf:
            m,c = find_line((self.__x+0.01, self.__y), (self.__x+dx, self.__y+dy))

        x_range = [self.__x + .1 * i for i in range(1, math.floor(dx / .1))]
        x_range = [self.__x] + x_range + [self.__x+dx]
        for xi in x_range:
            yi = m * xi + c 
            if not grid.is_free(math.floor(xi), math.floor(yi)):
                raise Exception(f"grid ({math.floor(xi)}, {math.floor(yi)}) isn't free error")
        self.__x += dx
        self.__y += dy
        self.get_cells_in_fov(grid)

        return (self.__x, self.__y, self.__h)
    
import math

def get_wheel_velocities(robbie, coord):
    """
    Calculate wheel velocities to reach a specified coordinate.

    Parameters:
    - robbie (Robot): The robot object.
    - coord (tuple): The target coordinate (x, y).

    Returns:
    tuple: The left and right wheel velocities.
    """

    # Calculate the desired change in position
    dx_world = coord[0] - robbie.x
    dy_world = coord[1] - robbie.y
    dx_robot, dy_robot = rotate_point(dx_world, dy_world, robbie.h)
    dist_to_coord = math.sqrt(dx_robot**2 + dy_robot**2)
    
    # Turn in place first
    angle = math.atan2(dy_robot, dx_robot)
    threshold = 0.1
    if angle < -threshold:
        return -0.05, 0.05
    elif angle > threshold:
        return 0.05, -0.05
    
    # Using desired linear velocity, set left and right wheel velocity
    linear_v = 0.05 * dist_to_coord
    w = 0.3 * math.atan2(dy_robot, dx_robot)
    vl = (linear_v - robbie.wheel_dist / 2 * w) 
    vr = (linear_v + robbie.wheel_dist / 2 * w)    
    return vr, vl

def frontier_planning(robbie, grid):
    """
    Plan the next move to explore frontier cells.

    Parameters:
    - robbie (Robot): The robot object.
    - grid (Grid): The grid object representing the environment.

    Returns:
    Robot: The updated robot object.
    """

    free_cells =[cell for cell in robbie.explored_cells if grid.is_free(cell[0],cell[1])]
    frontier_cells = []

    # Frontier cell  - cell that has perp neighbor thats not in explored and is not occupied
    # and neigbor is not outside grid
    for x,y in free_cells:
        for neighbor in [(x - 1, y), (x, y - 1), (x, y + 1), (x + 1, y)]:
            if neighbor not in robbie.explored_cells and grid.is_in(neighbor[0], neighbor[1]):
                frontier_cells.append((x,y))
    frontier_cells = list(set(frontier_cells))
    frontiers = separate_adjacent_coordinates(frontier_cells, grid)
    frontier = []
    
    # Min Len, Max dist to Center. So len/dist should be min
    min_util = float('inf')
    for f in frontiers:
        x_c, y_c =  find_centroid(f)
        if grid.is_occupied(x_c, y_c):
            continue
        distance = grid_distance(robbie.x,robbie.y,x_c,y_c)
        util = (len(frontier)/ distance)
        if util < min_util:
            min_util, frontier = len(f), f
    robbie.next_coord = find_centroid(frontier)
    return robbie

def exploration_state_machine(robbie, grid):
    """
    Perform the exploration state machine.

    Parameters:
    - robbie (Robot): The robot object.
    - grid (Grid): The grid object representing the environment.

    Returns:
    Robot: The updated robot object.
    """

    if not hasattr(robbie, 'vr') and not hasattr(robbie, 'vl'):
        robbie.vr, robbie.vl = 0, 0
    if not robbie.next_coord:
        robbie.next_coord = (robbie.x, robbie.y)
    target_coord = robbie.next_coord
    if grid_distance(target_coord[0], target_coord[1], robbie.x, robbie.y) < 0.1:
        robbie = frontier_planning(robbie, grid)
        robbie.path = grid.rrt((robbie.x, robbie.y), robbie.next_coord)
    else:
        if grid_distance(robbie.path[1][0], robbie.path[1][1], robbie.x, robbie.y) < 0.1: 
            robbie.path = [robbie.path[0]] + robbie.path[2:]
        robbie.vr, robbie.vl = get_wheel_velocities(robbie, robbie.path[1])     
    return robbie

def planning(robbie, grid):
    """
    Plan the next move in phase 2 of exploration.

    Parameters:
    - robbie (Robot): The robot object.
    - grid (Grid): The grid object representing the environment.

    Returns:
    Robot: The updated robot object.
    """

    if not hasattr(robbie, 'vr') and not hasattr(robbie, 'vl'):
        robbie.vr, robbie.vl = 0, 0

    if not robbie.next_coord:
        robbie.next_coord = (robbie.x, robbie.y)
    target_coord = robbie.next_coord
    if grid_distance(target_coord[0], target_coord[1], robbie.x, robbie.y) < 0.1:
        all_markers = grid.markers
        curr_marker = 0
        for i in range(len(all_markers)):
            if not robbie.marker_already_picked(all_markers[i]):
                curr_marker = i
                break
        n_x, n_y, n_p = all_markers[curr_marker]
        robbie.next_coord = (n_x, n_y)
        print(f"Next Marker set to : {robbie.next_coord}, Curr marker idx : {curr_marker}, Marker : {all_markers[curr_marker]}")
        robbie.path = grid.rrt((robbie.x, robbie.y), robbie.next_coord)
    else:
        if grid_distance(robbie.path[1][0], robbie.path[1][1], robbie.x, robbie.y) < 0.1: 
            robbie.path = [robbie.path[0]] + robbie.path[2:]
        robbie.vr, robbie.vl = get_wheel_velocities(robbie, robbie.path[1])     
    return robbie
 