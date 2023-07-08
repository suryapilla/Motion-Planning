from utils import *
import argparse
import yaml
import pdb

with open("./config/config.yml", 'r') as stream:
    config = yaml.safe_load(stream)

valid_envs = ["maze", "window", "tower", "single_cube", "room", "monza", "flappy_bird"]

parser = argparse.ArgumentParser()
parser.add_argument('--env', type=str, default='single_cube', choices=valid_envs)
parser.add_argument('--planner', type=str, default='Astar')
arg = parser.parse_args()
config['ENV'] = arg.env
config['PLANNER_TYPE'] = arg.planner


path_map = config['PATH_MAP']
env = config['ENV']
planner_type = config['PLANNER_TYPE']
heuristic_type = config['HEURISTIC_TYPE']["euclidean"]
# heuristic_type = config['HEURISTIC_TYPE']["manhattan"]

mapfile = path_map + env +'.txt'

# pdb.set_trace()
boundary, blocks = load_map(mapfile)
# Define the planner parameters
resolution = 0.5
goal_threshold = 0.5

t0 = tic()

# Create the planner instance
if planner_type == 'Astar':
    planner = AStarPlanner(boundary, blocks, resolution, goal_threshold)
else:
    planner = RRTPlanner(boundary, blocks, resolution, goal_threshold)
    
# Define the start and goal positions
if env == 'window':
    start = np.array([0.2, -4.9, 0.2]) 
    goal = np.array([6.0, 18.0, 3.0]) # Window
elif env == 'tower':
    start = np.array([2.5, 4.0, 0.5])
    goal = np.array([4.0, 2.5, 19.5]) # tower
elif env == 'room':
    start = np.array([1.0, 5.0, 1.5])
    goal = np.array([9.0, 7.0, 1.5]) # Room
elif env == 'flappy_bird':
    start = np.array([0.5, 2.5, 5.5])
    goal = np.array([19.0, 2.5, 5.5]) # flappy bird
elif env == 'monza':
    start = np.array([0.5, 1.0, 4.9])
    goal = np.array([3.8, 1.0, 0.1]) # Monza
elif env == 'maze':
    start = np.array([0.0, 0.0, 1.0])
    goal = np.array([12.0, 12.0, 5.0]) # Maze
elif env == 'single_cube':
    start = np.array([2.3, 2.3, 1.3])
    goal = np.array([7.0, 7.0, 5.5]) # Single cube

# Plan the path
if planner_type == 'Astar':
    path = planner.plan(start, goal,heuristic_type)
else:
    path = planner.plan(start, goal)

toc(t0, "Planning")

#%%
if path is not None:
    draw_map(boundary, blocks, start, goal, path)
    path_length = np.sum(np.sqrt(np.sum(np.diff(path, axis=0) ** 2, axis=1)))
    print('Path length: ', path_length)
else:
    print("Path not found!")
