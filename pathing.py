""" Pathfinding tools

current implementation only supplies the A* algorithm
future interest in jump-point search

neighbor search generators available:
    get_grid_neighbors
    get_euclidean__grid_neighbors
    get_orthogonal_grid_neighbors
    yet to be implemented:
        get_mesh_neighbors
        get_hex_neigbhors

distance functions available: (primarily supplied for use as heuristics)
    manhattan
    pythagorean
    euclidean
    chevyshev
    dijkstra
    hex
    hex_y_z

"""
# by agrippa kellum : april 2013

import heapq

# algorithms
#-------------------------------------------------------------------
def path(start, goal, h, n, world, tracing = 0):
    ''' generates coordinates for the shortest path from the goal to the start
        using the A* pathing algorithm

        h (the heuristic function):
            function which returns the distance from a position to the goal
            using educated guess

        n (the neighbor generator):
            generator that yields all nodes immediately accessable by the
            current node.

        world is passed to the neighbor search generator, and the structure
        necessary depends on the neighbor search used (usually you will
        use either a two dimensional array or a dictionary).

        regardless of structure, when a node in the world is referenced it
        must return the cost of travelling to that node--
        if empty or zero, the node is considered impassable

    '''
    # list of nodes whose neighbors have been added to the open
    closed = {}
    # list of nodes that are on the edge of the search
    heuristic = h(start, goal)
    open = {start : (0 + heuristic, 0, heuristic, None)}
    openHeap = [(0 + heuristic, start)]
    # format is f, g, h, p;
    # g+h, distance from start, heuristic distance from goal, parent

    while open:
        # find the member of the open with the lowest f value
        key = heapq.heappop(openHeap)[1]
        node = open.pop(key)
        # move current node to closed list
        closed[key] = node
        # check to see if this node is the goal
        if key == goal:
            if tracing:
                yield closed, open
            while key != start:
                yield key
                 # update keyinates to that of the parent
                key = closed[key][3]
            break
        # add neighbors of closed[key] node to the open list
        for nkey, movement in n(world, key):
            if nkey not in closed:
                g = node[1] + movement
                if nkey in open:
                    # if accessing this neighbor is longer through current node
                    # than the previous parent..
                    if g >= open[nkey][1]: continue
                heuristic = h(nkey, goal)
                open[nkey] = (g + heuristic, g, heuristic, key)
                heapq.heappush(openHeap, (g + heuristic, nkey))


# distance metrics
#-------------------------------------------------------------------
def  manhattan(pos, end):
    ''' return distance using taxicab geometry (no diagonal movement)'''
    h = abs(pos[0] - end[0]) + abs(pos[1] - end[1])
    return h

def pythagorean(pos, end):
    ''' return distance using distance formula '''
    h = ((pos[0] - end[0])**2 + (pos[1] - end[1])**2)**0.5
    return h

def euclidean(pos, end):
    ''' allows diagonal movement at the cost of 2**0.5 '''
    xdif = abs(pos[0] - end[0])
    ydif = abs(pos[1] - end[1])
    straight = abs(xdif - ydif)
    diag = ((xdif + ydif - straight) / 2)*(2**0.5)
    h = straight + diag
    return h

def chebyshev(pos, end):
    ''' diagonal movement costs just as much as straight '''
    h = max([abs(pos[0] - end[0]), abs(pos[1] - end[1])])
    return h

def dijkstra(pos, end):
    ''' dijkstra is A*s precedecessor which doesnt use a heuristic '''
    return 0

def hex(pos, end):
    ''' shortest distance along a grid of hexagons using offset coordinates

        how offset coordinate system is represented in an array:

        [0,2] [1,2]
           [0,1] [1,1] <-- odd row
        [0,0] [0,1]    <-- even row

        distance is calculated by converting the map coordinates into cubic form
        this is much easier to calculate
    '''

    pos[0] -= pos[1]//2
    end[0] -= end[1]//2

    return hex_y_z(pos, end)

def hex_y_z(pos, end):
    ''' shortest distance along a grid of hexagons using cubic coordinates
        (yes the name is a pun)

        how cubic coordinate system is represented in an array:

        [-1,2] [0,2] <-- y axis is slanted and straight
           [0,1] [1,1]   rather than wavey and upright
        [0,0] [0,1]

        z for each tile is extrapolated, as x + y + z always == 0

        illustration of cubic coordinate system in terms of neighbors:
        (underscore indicates no change)

              [-x, +y, _z] [_x, +y, -z]

        [-x, _y, +z] [start  pos] [+x, _y, -z]

              [_x, -y, +z] [+x, -y, _z]
    '''

    pos[2] = -(pos[0]+pos[1])
    end[2] = -(end[0]+end[1])

    h = max([abs(pos[0] - end[0]), abs(pos[1] - end[1]), abs(pos[2] - end[2])])
    return h


# utils
#-------------------------------------------------------------------
def is_orthogonal(pos1, pos2):
    ''' if the neighbor node can be reached by moving in a single orthogonal
    direction, return True

    put another way: if more than 1 coordinates between two points are
    different, the points are not orthogonal to one another, return False

    put another way: IF A ROOK, ASSUMING THERE ARE NO OBSTACLES, COULD
    TRAVEL BETWEEN THE TWO POSITIONS IN A SINGLE TURN, RETURN TRUE

    works in n dimensions, IM SURE THAT WILL BE USEFUL SOME DAY
    '''
    return sum(abs(cmp(pair)) for pair in zip(pos1, pos2)) <= 1


# neighbor searches
#-------------------------------------------------------------------
# def maptype

def get_grid_neighbors(world, coord):
    ''' generates neighbors and the cost to travel to them '''
    for nx in range(coord[0]-1, coord[0]+2):
        for ny in range(coord[1]-1, coord[1]+2):
            if (nx, ny) == coord: # if this node is the current node...
                continue
            # if this node is not impassable or out of the world...
            if 0 < nx < len(world) and 0 < ny < len(world[nx]):
                if world[nx][ny]:
                    yield (nx, ny), world[nx][ny]

def get_orthogonal_grid_neighbors(world, coord):
    ''' remove diagonal neighbors from neighbor generator '''
    for (nx, ny), movement in get_grid_neighbors(world, coord):
        if is_orthogonal((nx, ny), coord):
            yield (nx, ny), movement

def get_euclidean_grid_neighbors(world, coord):
    ''' cost to diagonal neighbors is euclidean '''
    for (nx, ny), movement in get_grid_neighbors(world, coord):
        if is_orthogonal((nx, ny), coord):
            cost_multiplier = 2**0.5
        else:
            cost_multiplier = 1
        yield (nx, ny), movement * cost_multiplier


if __name__ == "__main__":
    width = 100
    height = 100
    world = [[1 for _ in range(height)] for _ in range(width)]

    import time
    inittime = time.clock()
    for _ in range(1000):
        patho = list(path((50, 20), (75, 10), manhattan,
                          get_grid_neighbors, world))
    elapsed = time.clock() - inittime
    print(elapsed)
