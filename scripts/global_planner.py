from collections import deque
import matplotlib.pyplot as plt
import numpy as np
from PIL.Image import open
import time

X=0 
Y=1

# The Hesitancy to add to the occupancy map around inflated obstacles
mc = 4
# The starting position
start = (200, 200)
goal = (200, 210)
# the weight given to the potential fields around the obstacles in the costmap
alpha = 0
beta = 1
bot_radius = 3
# The map file
img = open('/home/robert/catkin_ws/src/ccd_planner/maps/CustomRoom.pgm')
# A 2d array of the static map
map = np.copy(np.transpose(img))
# The occupancy map, holds the inflated obstacles
occ_map = np.zeros(map.shape, dtype='float')
# track the cleaned cells
cleaned_map = set()
# Used for generating figures for the report
heat_map = np.zeros(map.shape, dtype='int')

def paint_visited(point):
    '''Mark the visited nodes as the robot travels the map.'''
    for i in range(-bot_radius, bot_radius+1):
        x = point[0] + i
        for j in range(-bot_radius, bot_radius+1):
            y = point[1] + j
            img.putpixel((x,y), 100)
            cleaned_map.add((x,y))
            heat_map[x, y] = heat_map[x, y] + 1

def mark_as_visited(point, visited: set):
    '''Mark all of the points underneath the robot as visited'''
    for i in range(-bot_radius, bot_radius+1):
        x = point[0] + i
        for j in range(-bot_radius, bot_radius+1):
            y = point[1] + j
            visited.add((x, y))

def test_for_collision(point):
    '''test to see if a move to the given point is valid
        If not, update the occupancy map.'''
    # update the occupancy map
    collision = False
    for i in range(point[X]-bot_radius, point[X]+bot_radius):
        for j in range(point[Y]-bot_radius, point[Y]+bot_radius):
            if map[i,j] == 0:
                collision = True
                occ_map[i, j] = float('inf')
                for ii in range(i - bot_radius, i+bot_radius+1):
                    for jj in range(j - bot_radius, j+bot_radius+1):
                        occ_map[ii,jj] = float('inf')
    
    return collision

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

    # Only return points inside the grid
    return [pt for pt in possible_neighbors if pt[0] >= 0 and pt[0] < map.shape[0] and pt[1] >= 0 and pt[1] < map.shape[1]]

def find_new_neighbors(point, visited):
    '''Find new neighbors by tracing a path out from the center of 
        the robot.'''
    possible_neighbors = []
    for i in range(bot_radius):
        x = point[0] + i
        if (x,point[1]) in visited:
            continue
        if occ_map[x, point[1]] < float('inf'):
            possible_neighbors.append((x, point[1]))

    for i in range(bot_radius):
        x = point[0] - i
        if (x,point[1]) in visited:
            continue
        if occ_map[x, point[1]] < float('inf'):
            possible_neighbors.append((x, point[1]))

    for i in range(bot_radius):
        y = point[1] + i
        if (point[0], y) in visited:
            continue
        if occ_map[point[0], y] < float('inf'):
            possible_neighbors.append((point[0], y))

    for i in range(bot_radius):
        y = point[1] - i
        if (point[0], y) in visited:
            continue
        if occ_map[point[0], y] < float('inf'):
            possible_neighbors.append((point[0], y))

    # only return points inside the map
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
            n_cost = occ_map[n[0], n[1]]
            if n not in visited and n_cost < float('inf'):
                visited.add(n)
                wavefront_cost[n] = wavefront_cost[current_point] + 1
                queue.append(n)

    return wavefront_cost

def propogate_obsfield():
    '''Propogate potential fields around the obstacles in the map.
        This will also inflate obstacles to prevent collisions.'''
    # For every cell in the map
    for i in range(map.shape[0]):
        for j in range(map.shape[1]):
            # Skip if the cell is not a hard obstacle.
            c = map[i, j]
            print(f'Populating occupancy map: {(i,j)}')
            if c != 0:
                continue
            occ_map[i,j] = float('inf')
            queue = deque()
            queue.append((i,j))
            closed = set()
            closed.add((i,j))
            # while the queue is not empty, keep checking cells.
            while len(queue) > 0:
                current = queue.popleft()
                # Get all the neighbors that are not obstacles
                neighbors = [pt for pt in get_neighbors(current) 
                    if pt not in closed]
                for neighbor in neighbors:
                    closed.add(neighbor)
                    # if the robot would be in a collision when the robot's center point is here
                    # mark it as an inflated obstacle.
                    if current[0] >= i - bot_radius and current[0] <= i + bot_radius and \
                        current[1] >= j - bot_radius and current[1] <= j + bot_radius:
                        occ_map[neighbor[0], neighbor[1]] = float('inf')
                        queue.append(neighbor)
                    # otherwise assign decreasing values in the map to signify that the robot
                    # is approaching an obstacle
                    else:
                        if occ_map[current[0],current[1]] == float('inf'):
                            if occ_map[neighbor[0], neighbor[1]] < mc + 1:
                                occ_map[neighbor[0],neighbor[1]] = mc + 1
                            queue.append(neighbor)
                        elif occ_map[current[0],current[1]] < float('inf') and occ_map[current[0],current[1]] > 0:
                            if occ_map[neighbor[0], neighbor[1]] < occ_map[current[0],current[1]] - 1:
                                occ_map[neighbor[0],neighbor[1]] = occ_map[current[0],current[1]] - 1
                            if occ_map[neighbor[0],neighbor[1]] > 0:
                                queue.append(neighbor)

                
def dstar_prime(start, visited):
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
            if occ_map[neighbor[0],neighbor[1]] < float('inf') and neighbor not in closed:
                closed.add(neighbor)
                path[neighbor] = current
                if neighbor not in visited:
                    return list(get_path(neighbor))
                queue.append(neighbor)
    return None



def dstar_search(start, wavefront_cost):
    '''the main search algo. Always follow the max cost cell in the costmap.
        Loop until the algo cannot find anymore unvisited nodes.'''
    current = start
    path = [start]
    closed = set(cleaned_map)
    mark_as_visited(start, closed)

    while True:
        new_neighbors = find_new_neighbors(current, closed)
        max_neighbor = None
        max_cost = None
        for neighbor in new_neighbors:
            if neighbor not in closed and occ_map[neighbor[0],neighbor[1]] < float('inf'):
                w_cost = wavefront_cost[neighbor]
                print(f'Testing {neighbor} with w_cost {w_cost}...')
                c = beta*w_cost - alpha*occ_map[neighbor[0],neighbor[1]]
                if max_cost is None or c > max_cost:
                    max_neighbor = neighbor
                    max_cost = c
        
        # if you couldn't find a neighbor, execute the dprime search
        if max_neighbor is None:
            new_path = dstar_prime(current, closed)
            # if a path is still not found, we're done!
            if new_path is None:
                return path
            # otherwise extend the path to the new location
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

def add_object(shape):
    map[shape[0]:shape[1],shape[2]:shape[3]] = np.zeros((shape[1]-shape[0], shape[3]-shape[2]), dtype='int8')
    for i in range(shape[0],shape[1]):
        for j in range(shape[2], shape[3]):
            img.putpixel((i,j), 0)

# Calculate the wavefront cost...
wavefront_cost = propagate_wavefront(goal)
# Normalize so that a weighted average can be calculated.
max_w = float(max(wavefront_cost.values()))
wavefront_cost = {x: (wavefront_cost[x] / max_w) for x in wavefront_cost }
# Calculate the obstacle field
# propogate_obsfield()
# np.savetxt('custom_room_occ_map_v2.csv', occ_map, delimiter=',')
occ_map = np.loadtxt('custom_room_occ_map_v2.csv', delimiter=',')
# Find the path
# path = dstar_search(start, wavefront_cost)

# Plot the results.
x = []
y = []

# ax.xaxis.set_ticks([])
# ax.yaxis.set_ticks([])

update_freq = 10 # set how often the plot will be updated.

# Plot the planned path


found_collision = True
last_index = -1
path = []
initial_run = True
while found_collision:
    if initial_run:
        path = dstar_search(start, wavefront_cost)
        initial_run = False
        add_object((170,175,120,125))
    else:
        path = dstar_search(path[last_index], wavefront_cost)
        last_index = -1
    
    plt.xlim((0, map.shape[0]))
    plt.ylim((0, map.shape[1]))
    
    im_obj = plt.imshow(img, cmap='gray')
    axes = plt.gca()
    line, = axes.plot(x, y, 'o-')
    for pt in path:
        x.append(pt[0])
        y.append(pt[1])

    line2 = plt.plot(x, y, 'y:')
    x.clear()
    y.clear()

    
    for i, pt in enumerate(path):
        found_collision = test_for_collision(pt)
        if found_collision:
            x.clear()
            y.clear()
            line2[0].set_xdata(x)
            line2[0].set_ydata(y)
            line.set_xdata(x)
            line.set_ydata(y)
            break
        last_index = last_index + 1
        paint_visited(pt)
        x.append(pt[0])
        y.append(pt[1])
        line.set_xdata(x)
        line.set_ydata(y)
        if i % update_freq == 0:
            im_obj.set_data(img)
            plt.draw()
            plt.pause(1e-17)
            time.sleep(0.005)

# np.savetxt('heatmap.csv', heat_map, delimiter=',')
print(np.max(heat_map))
print(np.min(heat_map))
print(np.mean(heat_map))
plt.show()