from collections import namedtuple, deque
import matplotlib.pyplot as plt
import numpy as np
from PIL.Image import Image, new, open, fromarray
import math
import time

class Vertex:
    def __init__(self, pose, previous_pose=None, cost=0) -> None:
        self.pose = pose
        self.previous_pose = previous_pose
        self.cost = cost

# the threshold to define an obstacle in the costmap
lethal_threshold = 225
free_space = 254
# The starting position
start = (20, 60)
goal = (30, 60)
# the weight given to the potential fields around the obstacles in the costmap
alpha = 0
beta = 1.0
inflation_radius = 1
bot_radius = 3
# The map file
img = open('map.pgm')
map = np.copy(np.transpose(img))

def mark_as_visited(point, visited: set):
    for i in range(-bot_radius, bot_radius+1):
        x = point[0] + i
        for j in range(-bot_radius, bot_radius+1):
            y = point[1] + j
            visited.add((x, y))

def get_neighbors(point, use_4connected=True):
    '''Get the neighbors of the given node.'''
    if use_4connected:
        possible_neighbors = [
            (point[0]-1, point[1]),
            (point[0]+1, point[1]),
            (point[0], point[1]-1),
            (point[0], point[1]+1),
        ]
    else:
        possible_neighbors = [
            (point[0]-1, point[1]-1),
            (point[0]-1, point[1]+1),
            (point[0]-1, point[1]),
            (point[0]+1, point[1]+1),
            (point[0]+1, point[1]-1),
            (point[0]+1, point[1]),
            (point[0], point[1]-1),
            (point[0], point[1]+1),
        ]

    return [pt for pt in possible_neighbors if pt[0] >= 0 and pt[0] < map.shape[0] and pt[1] >= 0 and pt[1] < map.shape[1]]

def get_neighbors_(point, visited):
    '''Get the neighbors of the given node.'''
    possible_neighbors = []
    for i in range(bot_radius):
        x = point[0] + i
        if (x,point[1]) in visited:
            continue
        if map[x, point[1]] > free_space - inflation_radius:
            possible_neighbors.append((x, point[1]))

    for i in range(bot_radius):
        x = point[0] - i
        if (x,point[1]) in visited:
            continue
        if map[x, point[1]] > free_space - inflation_radius:
            possible_neighbors.append((x, point[1]))

    for i in range(bot_radius):
        y = point[1] + i
        if (point[0], y) in visited:
            continue
        if map[point[0], y] > free_space - inflation_radius:
            possible_neighbors.append((point[0], y))

    for i in range(bot_radius):
        y = point[1] - i
        if (point[0], y) in visited:
            continue
        if map[point[0], y] > free_space - inflation_radius:
            possible_neighbors.append((point[0], y))

    return [pt for pt in possible_neighbors if pt[0] >= 0 and pt[0] < map.shape[0] and pt[1] >= 0 and pt[1] < map.shape[1]]

def propagate_wavefront(start):
    '''Propagate a wavefront from the starting position following a bfs search.'''
    wavefront_cost = {}
    visited = set()
    wavefront_cost[start] = 0
    visited.add(start)
    queue = deque([start])
    while len(queue) > 0:
        current_point = queue.popleft()
        neighbors = get_neighbors(current_point, use_4connected=False)
        for n in neighbors:
            n_cost = map[n[0], n[1]]
            if n not in visited and n_cost > lethal_threshold:
                visited.add(n)
                wavefront_cost[n] = wavefront_cost[current_point] + 1
                queue.append(n)

    return wavefront_cost

def propogate_obsfield():
    '''Propogate potential fields around the obstacles in the map'''
    # obs_field = {}
    for i in range(map.shape[0]):
        for j in range(map.shape[1]):
            c = map[i, j]
            if c != 0:
                continue
            queue = deque()
            queue.append((i,j))
            closed = set()
            closed.add((i,j))
            # obs_field[i, j] = -(inflation_radius+1)
            while len(queue) > 0:
                current = queue.popleft()
                neighbors = [pt for pt in get_neighbors(current, use_4connected=False) if pt not in closed and map[pt[0],pt[1]] != 0]
                for neighbor in neighbors:
                    closed.add(neighbor)
                    if current[0] > i - bot_radius and current[0] < i + bot_radius and \
                        current[1] > j - bot_radius and current[1] < j + bot_radius:
                        map[neighbor[0], neighbor[1]] = 10
                        queue.append(neighbor)
                    else:
                        if map[current[0],current[1]] <= 10:
                            map[neighbor[0],neighbor[1]] = free_space - inflation_radius
                            queue.append(neighbor)
                        elif map[current[0],current[1]] >= free_space - inflation_radius:
                            map[neighbor[0],neighbor[1]] = map[current[0],current[1]] + 1
                            if map[neighbor[0],neighbor[1]] < free_space:
                                queue.append(neighbor)

                #     for neighbor in get_neighbors(current, use_4connected=False):
                #         n_c = map[neighbor[0], neighbor[1]]
                #         if neighbor not in obs_field.keys() and n_c > lethal_threshold:
                #             queue.append(neighbor)
                # else:
                #     new_cost = obs_field[current] + 1
                #     if new_cost >= 0:
                #         continue
                #     for neighbor in get_neighbors(current, use_4connected=False):
                #         n_c = map[neighbor[0], neighbor[1]]
                #         if neighbor not in obs_field.keys() and n_c > lethal_threshold:
                #             queue.append(neighbor)
                #             obs_field[neighbor] = new_cost
    return

                
def dstar_prime(start, visited, valid_pts):
    '''Execute a bfs search for the nearest unvisited node. 
        Used when the planner gets stuck to find the quickest way out.'''
    path = {start: None}
    def get_path(s):
        current = s
        while path[current] is not None:
            yield current
            current = path[current]

    closed = set()
    closed.add(start)
    queue = deque()
    queue.append(start)
    while len(queue) > 0:
        current = queue.popleft()
        for neighbor in get_neighbors(current):
            if neighbor in valid_pts and neighbor not in closed:
                closed.add(neighbor)
                path[neighbor] = current
                if neighbor not in visited:
                    return list(get_path(neighbor))
                queue.append(neighbor)
    return None



def dstar_search(start, wavefront_cost):
    '''the main search algo. Always follow the lowest cost cell in the costmap.
        Loop until the algo cannot find anymore unvisited nodes.'''
    current = start
    path = [start]
    closed = set()
    # closed.add(current)
    mark_as_visited(start, closed)

    while True:
        new_neighbors = get_neighbors_(current, closed)
        max_neighbor = None
        max_cost = None
        for neighbor in new_neighbors:
            if neighbor not in closed and neighbor in wavefront_cost.keys():
                w_cost = wavefront_cost[neighbor]
                print(f'Testing {neighbor} with w_cost {w_cost}...')
                # if neighbor in obstacle_cost.keys():
                    # obs_cost = obstacle_cost[neighbor]
                    # print(f'Neighbor has a obs_cost of {obs_cost}')
                # else:
                    # obs_cost = 0
                c = (alpha*(map[neighbor[0],neighbor[1]]-250) + beta*w_cost) / 2
                if max_cost is None or c > max_cost:
                    max_neighbor = neighbor
                    max_cost = c
        
        if max_neighbor is None:
            new_path = dstar_prime(current, closed, wavefront_cost.keys())
            if new_path is None:
                return path
            else:
                new_path.reverse()
                path.extend(new_path)
                current = new_path[-1]
                for pt in new_path:
                    mark_as_visited(pt, closed)
        else:
            print(f'Chose {max_neighbor}...')
            path.append(max_neighbor)
            current = max_neighbor
            mark_as_visited(start, closed)
            closed.add(current)

    return path


# Calculate the wavefront cost...
wavefront_cost = propagate_wavefront(goal)
# Normalize so that a weighted average can be calculated.
max_w = float(max(wavefront_cost.values()))
wavefront_cost = {x: (wavefront_cost[x] / max_w) for x in wavefront_cost }
# Calculate the obstacle field
propogate_obsfield()
# Normalize that too...
# max_o = float(max(obs_field.values()))
# obs_field = {x: (obs_field[x] / max_o) for x in obs_field }
# Find the path
path = dstar_search(start, wavefront_cost)

# Plot the results.
x = []
y = []

plt.imshow(img, cmap='gray')
plt.xlim((0, map.shape[0]))
plt.ylim((0, map.shape[1]))
axes = plt.gca()
line, = axes.plot(x, y, 'r-')
for pt in path:
    x.append(pt[0])
    y.append(pt[1])
    # foo = plt.plot(x, y, 'r')
    # foo.append()
    line.set_xdata(x)
    line.set_ydata(y)
    plt.draw()
    plt.pause(1e-17)
    time.sleep(0.005)


plt.show()