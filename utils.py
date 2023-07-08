#%%
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial import KDTree
import time
from pqdict import PQDict
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from queue import PriorityQueue

#%%
def tic():
    return time.time()

def toc(tstart, nm=""):
    print('%s took: %s sec.\n' % (nm, (time.time() - tstart)))


def load_map(fname):
    '''
    Loads the bounady and blocks from map file fname.

    boundary = [['xmin', 'ymin', 'zmin', 'xmax', 'ymax', 'zmax','r','g','b']]

    blocks = [['xmin', 'ymin', 'zmin', 'xmax', 'ymax', 'zmax','r','g','b'],
              ...,
              ['xmin', 'ymin', 'zmin', 'xmax', 'ymax', 'zmax','r','g','b']]
    '''
    mapdata = np.loadtxt(fname, dtype={'names': ('type', 'xmin', 'ymin', 'zmin', 'xmax', 'ymax', 'zmax', 'r', 'g', 'b'), \
                                       'formats': ('S8', 'f', 'f', 'f', 'f', 'f', 'f', 'f', 'f', 'f')})
    blockIdx = mapdata['type'] == b'block'
    boundary = mapdata[~blockIdx][['xmin', 'ymin', 'zmin', 'xmax', 'ymax', 'zmax', 'r', 'g', 'b']].view('<f4').reshape(
        -1, 11)[:, 2:]
    blocks = mapdata[blockIdx][['xmin', 'ymin', 'zmin', 'xmax', 'ymax', 'zmax', 'r', 'g', 'b']].view('<f4').reshape(-1,
                                                                                                                    11)[
             :, 2:]
    return boundary, blocks

def draw_map(boundary, blocks, start, goal, path):
    '''
    Visualization of a planning problem with environment boundary, obstacle blocks, and start and goal points
    '''
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    hb = draw_block_list(ax, blocks)
    ax.plot(path[:, 0], path[:, 1], path[:, 2], 'r-', label='Path')
    ax.scatter(start[0], start[1], start[2], c='green', marker='o', label='Start')
    ax.scatter(goal[0], goal[1], goal[2], c='blue', marker='o', label='Goal')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_xlim(boundary[0, 0], boundary[0, 3])
    ax.set_ylim(boundary[0, 1], boundary[0, 4])
    ax.set_zlim(boundary[0, 2], boundary[0, 5])
    ax.legend()
    plt.show()


def draw_block_list(ax, blocks):
    '''
    Subroutine used by draw_map() to display the environment blocks
    '''
    v = np.array([[0, 0, 0], [1, 0, 0], [1, 1, 0], [0, 1, 0], [0, 0, 1], [1, 0, 1], [1, 1, 1], [0, 1, 1]],
                 dtype='float')
    f = np.array([[0, 1, 5, 4], [1, 2, 6, 5], [2, 3, 7, 6], [3, 0, 4, 7], [0, 1, 2, 3], [4, 5, 6, 7]])
    clr = blocks[:, 6:] / 255
    n = blocks.shape[0]
    d = blocks[:, 3:6] - blocks[:, :3]
    vl = np.zeros((8 * n, 3))
    fl = np.zeros((6 * n, 4), dtype='int64')
    fcl = np.zeros((6 * n, 3))
    for k in range(n):
        vl[k * 8:(k + 1) * 8, :] = v * d[k] + blocks[k, :3]
        fl[k * 6:(k + 1) * 6, :] = f + k * 8
        fcl[k * 6:(k + 1) * 6, :] = clr[k, :]

    if type(ax) is Poly3DCollection:
        ax.set_verts(vl[fl])
    else:
        pc = Poly3DCollection(vl[fl], alpha=0.25, linewidths=1, edgecolors='k')
        pc.set_facecolor(fcl)
        h = ax.add_collection3d(pc)
        return h

class Node:
    def __init__(self, position):
        self.position = position
        self.parent = None
        self.g_score = float('inf')
        self.f_score = float('inf')

    def __lt__(self, other):
        return self.f_score < other.f_score

class AStarPlanner:
    def __init__(self, boundary, blocks, resolution, goal_threshold=1):
        self.boundary = boundary
        self.blocks = blocks
        self.resolution = resolution
        self.goal_threshold = goal_threshold
        self.start_node = None
        self.goal_node = None

    def plan(self, start, goal,heur):
        self.start_node = Node(start)
        self.goal_node = Node(goal)
        self.start_node.g_score = 0
        self.start_node.f_score = self.heuristic_cost_estimate(self.start_node,heur)
        open_set = PriorityQueue()
        open_set.put((self.start_node.f_score, self.start_node))

        closed_set = set()
        open_set_2 = set()
        open_set_2.add(tuple(self.start_node.position))
        while not open_set.empty():
            
            current_node = open_set.get()[1]
            if self.is_goal_reached(current_node):
                
                return self.generate_path(current_node)

            closed_set.add(tuple(current_node.position))
            
            for neighbor in self.get_neighbors(current_node):
                if (tuple(neighbor.position) in closed_set) or (tuple(neighbor.position) in open_set_2):
                    
                    continue

                tentative_g_score = current_node.g_score + self.distance(current_node, neighbor)

                if tentative_g_score < neighbor.g_score:
                    neighbor.parent = current_node
                    neighbor.g_score = tentative_g_score
                    neighbor.f_score = neighbor.g_score + self.heuristic_cost_estimate(neighbor,heur)
                    open_set.put((neighbor.f_score, neighbor))
                    open_set_2.add(tuple(neighbor.position))

        return None

    def heuristic_cost_estimate(self, node, heur="e"):
        if heur == "e":
            return np.linalg.norm(node.position - self.goal_node.position)
        elif heur == "m":
            return abs(node.position[0] - self.goal_node.position[0]) + abs(node.position[1] - self.goal_node.position[1]) + abs(node.position[2] - self.goal_node.position[2])

        else:
            dx = abs(node.position[0] - self.goal_node.position[0])
            dy = abs(node.position[1] - self.goal_node.position[1])
            dz = abs(node.position[2] - self.goal_node.position[2])
            
            # Cost for diagonal movements in 3D space
            diagonal_cost = min(dx, dy, dz)
            
            # Cost for non-diagonal movements in 3D space
            non_diagonal_cost = dx + dy + dz - 2 * diagonal_cost
            
            # Octile heuristic cost estimate
            return np.sqrt(3) * diagonal_cost + non_diagonal_cost

    def distance(self, node1, node2):
        return np.linalg.norm(node1.position - node2.position)

    def is_goal_reached(self, node):
        return np.linalg.norm(node.position - self.goal_node.position) <= self.goal_threshold

    def get_neighbors(self, node):
        neighbors = []

        for dx in [-self.resolution, 0, self.resolution]:
            for dy in [-self.resolution, 0, self.resolution]:
                for dz in [-self.resolution, 0, self.resolution]:
                    if dx == dy == dz == 0:
                        continue
                    # print(np.array([dx, dy, dz]))
                    new_position = node.position + np.array([dx, dy, dz])
                    new_node = Node(new_position)
                    for block in self.blocks:
                        if check_collision(node.position,new_position,block):
                            continue
                    # if is_path_intersecting_obstacles(node.position,new_position, self.blocks):
                    #     continue
                    
                    if(self.isValid(new_position)):
                        neighbors.append(new_node)

        return neighbors


    def isValid(self, coord):
        
    # Check if the given coordinate is valid
    # Implement the logic to check if the coordinate is within the boundary and not colliding with any blocks
        x = coord[0]
        y = coord[1]
        z = coord[2]
        
        # Check if the coordinate is within the boundary
        if x < self.boundary[0][0] or x > self.boundary[0][3]:
            return False
        if y < self.boundary[0][1] or y > self.boundary[0][4]:
            return False
        if z < self.boundary[0][2] or z > self.boundary[0][5]:
            return False
        

        for block in self.blocks:
            if x >= block[0] and x <= block[3] and \
            y >= block[1] and y <= block[4] and \
            z >= block[2] and z <= block[5]:
                return False
        
        # Coordinate is valid
        return True
        

    def generate_path(self, node):
        path = []
        current = node
        while current is not None:
            path.append(current.position)
            current = current.parent
        path = np.array(path[::-1])  # Reverse the path

        return path
# rrt

class Node_rrt:
    def __init__(self, position):
        self.position = position
        self.parent = None


class RRTPlanner:
    def __init__(self, boundary, blocks, resolution, goal_threshold=0.5, max_iterations=1000000000000):
        self.boundary = boundary
        self.blocks = blocks
        self.resolution = resolution
        self.goal_threshold = goal_threshold
        self.max_iterations = max_iterations
        self.nodes = []
        self.kdtree = None
        self.start_node = None
        self.goal_node = None

    def plan(self, start, goal):
        self.start_node = Node_rrt(start)
        self.goal_node = Node_rrt(goal)
        self.nodes = [self.start_node]
        self.kdtree = KDTree([start])

        for iteration in range(self.max_iterations):
            random_point = self.generate_random_point()
            nearest_node = self.find_nearest_node(random_point)
            new_node = self.steer(nearest_node, random_point)
            
            if self.check_collision_free(nearest_node.position, new_node.position):
                self.add_node(new_node, nearest_node)

                if self.is_goal_reached(new_node):
                    return self.generate_path(new_node)

        return None

    def generate_random_point(self):
        x_min, y_min, z_min = self.boundary[0][0:3]
        x_max, y_max, z_max = self.boundary[0][3:6]
        random_point = np.array([np.random.uniform(x_min, x_max),
                                 np.random.uniform(y_min, y_max),
                                 np.random.uniform(z_min, z_max)])
        return random_point

    def find_nearest_node(self, point):
        distances, indices = self.kdtree.query(point)
        nearest_node = self.nodes[indices]
        return nearest_node

    def steer(self, from_node, to_point):
        direction = to_point - from_node.position
        distance = np.linalg.norm(direction)
        if distance > self.resolution:
            direction = (direction / distance) * self.resolution
        new_position = from_node.position + direction
        new_node = Node_rrt(new_position)
        return new_node

    def check_collision_free(self, from_point, to_point):
        direction = to_point - from_point
        distance = np.linalg.norm(direction)
        step_size = self.resolution / distance
        for t in np.arange(0, 1,0.5):
            current_point = from_point + t * direction
            for block in self.blocks:
                if check_collision_r(from_point, current_point, block[:3], block[3:]):
                    return False
        return True

    def add_node(self, new_node, parent_node):
        new_node.parent = parent_node
        self.nodes.append(new_node)
        self.kdtree = KDTree([node.position for node in self.nodes])

    def is_goal_reached(self, node):
        return np.linalg.norm(node.position - self.goal_node.position) <= self.goal_threshold

    def generate_path(self, node):
        path = []
        current = node
        while current is not None:
            path.append(current.position)
            current = current.parent
        path = np.array(path[::-1])  # Reverse the path

        # Check if the last node is within the goal threshold
        if np.linalg.norm(node.position - self.goal_node.position) <= self.goal_threshold:
            path = np.vstack((path, self.goal_node.position))

        return path

def check_collision(line_start, line_end, block):
    
    box_min = block[0:3]
    box_max = block[3:6]
    
    # print(box_min)
    # Check if any of the start and end are inside AABB
    if (
            box_min[0] <= line_start[0] <= box_max[0] and
            box_min[1] <= line_start[1] <= box_max[1] and
            box_min[2] <= line_start[2] <= box_max[2]
    ) or (
            box_min[0] <= line_end[0] <= box_max[0] and
            box_min[1] <= line_end[1] <= box_max[1] and
            box_min[2] <= line_end[2] <= box_max[2]
    ) or (
            box_min[0] >= line_start[0] and line_end[0] >= box_max[0] and
            box_min[1] >= line_start[1] and line_end[1] >= box_max[1] and
            box_min[2] >= line_start[2] and line_end[2] >= box_max[2]
    )or (
            box_min[0] <= line_start[0] and line_end[0] <= box_max[0] and
            box_min[1] <= line_start[1] and line_end[1] <= box_max[1] and
            box_min[2] <= line_start[2] and line_end[2] <= box_max[2]
    )or (
            box_min[0] >= line_end[0] and line_start[0] >= box_max[0] and
            box_min[1] >= line_end[1] and line_start[1] >= box_max[1] and
            box_min[2] >= line_end[2] and line_start[2] >= box_max[2]
    ):
        return True
    # Check if the line segment intersects any of the AABB's faces
    for i in range(3):
        if line_start[i] < box_min[i] and line_end[i] < box_min[i]:
            continue
        if line_start[i] > box_max[i] and line_end[i] > box_max[i]:
            continue
        t = abs((box_min[i] - line_start[i]) / (line_end[i] - line_start[i] + 0.000000001))
        if 0 <= t <= 1:
            intersection_point = tuple(line_start[j] + t * (line_end[j] - line_start[j]) for j in range(3))
            if all(box_min[(i + j) % 3] <= intersection_point[(i + j) % 3] <= box_max[(i + j) % 3] for j in range(1, 3)):
                return True

    return False

def check_collision_r(line_start, line_end, box_min, box_max):
    

    # Check if any of the line segment points is inside the AABB
    if (
            box_min[0] <= line_start[0] <= box_max[0] and
            box_min[1] <= line_start[1] <= box_max[1] and
            box_min[2] <= line_start[2] <= box_max[2]
    ) or (
            box_min[0] <= line_end[0] <= box_max[0] and
            box_min[1] <= line_end[1] <= box_max[1] and
            box_min[2] <= line_end[2] <= box_max[2]
    ):
        return True

    # Check if the line segment intersects any of the AABB's faces
    for i in range(3):
        if line_start[i] < box_min[i] and line_end[i] < box_min[i]:
            continue
        if line_start[i] > box_max[i] and line_end[i] > box_max[i]:
            continue
        
        t = abs((box_min[i] - line_start[i]) / (line_end[i] - line_start[i] + 0.000000001))
        if 0 <= t <= 1:
            intersection_point = (
                line_start[0] + t * (line_end[0] - line_start[0]),
                line_start[1] + t * (line_end[1] - line_start[1]),
                line_start[2] + t * (line_end[2] - line_start[2])
            )
            if (
                    box_min[(i + 1) % 3] <= intersection_point[(i + 1) % 3] <= box_max[(i + 1) % 3] and
                    box_min[(i + 2) % 3] <= intersection_point[(i + 2) % 3] <= box_max[(i + 2) % 3]
            ):
                return True

    return False