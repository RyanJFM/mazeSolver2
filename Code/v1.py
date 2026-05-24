# Code v1.1 --> test mouse headings and coordinates with mms

import API
import sys

INF = 9999 # initialize infinity

def log(string):
    sys.stderr.write(f"{string}\n")  # save errors to log
    sys.stderr.flush()

def update_walls(walls, x, y, direction, width, height):
    # fetch coordinate changes into mapping
    dx = [0, 1, 0, -1]
    dy = [1, 0, -1, 0]

    # bit masks
    wall_bits = [1, 2, 4, 8]

    # add walls to current cell
    walls[x][y] |= wall_bits[direction]

    # Calculate neighbor coordinates
    nx = x + dx[direction]
    ny = y + dy[direction]

    # check bounds and prevent leaks
    if 0 <= nx < width and 0 <= ny < height:
        opposite_dir = (direction + 2) % 4
        walls[nx][ny] |= wall_bits[opposite_dir]


def floodfill(distances, walls, center_x, center_y, width, height):
    # fetch coordinate changes into mapping
    dx = [0, 1, 0, -1]
    dy = [1, 0, -1, 0]

    # bit masks
    wall_bits = [1, 2, 4, 8]

    # reset all distances to INF
    for i in range(width):
        for j in range(height):
            distances[i][j] = INF

    # BFS algorithm is used for pathfinding

    # set the center to 0 and add to Queue
    distances[center_x][center_y] = 0
    queue = [(center_x, center_y)]

    # queue operations
    while len(queue) > 0:
        cx, cy = queue.pop(0)

        for i in range(4):
            if (walls[cx][cy] & wall_bits[i]) == 0:
                nx = cx + dx[i]
                ny = cy + dy[i]

                # initialize values to neighbors
                if 0 <= nx < width and 0 <= ny < height:
                    if distances[nx][ny] == INF:
                        distances[nx][ny] = distances[cx][cy] + 1
                        queue.append((nx, ny))

                        API.setText(nx, ny, str(distances[nx][ny]))
    
    API.setText(center_x, center_y, "0")

                    
def main():
    log("Booting phase 1 & 2...")

    # Grid contraints: 5 x 5
    width, height = 5, 5
    center_x, center_y = 2, 2

    # Initialize distance map
    distances = [[0 for _ in range(height)] for _ in range(width)]

    # Initialize wall map
    walls = [[0 for _ in range(height)] for _ in range(width)]

    # fetch coordinate changes mapped to headings[N, E, S, W]
    dx = [0, 1, 0, -1]    
    dy = [1, 0, -1, 0]

    # define bitmasks --> [N = 1, E = 2, S = 4, W = 8]
    wall_bits = [1, 2, 4, 8]

    for i in range(width):
        for j in range(height):

            # Manhattan distance formular
            distances[i][j] = abs(center_x - i) + abs(center_y - j)

            # display number on mms maze tiles
            API.setText(i, j, str(distances[i][j]))

    x, y = 0, 0    # define the origin of inertial frame

    # (North = 0, East = 1, South = 2, West = 3)
    heading = 0   # default heading = North

    while True:
        API.setColor(x, y, "G") # trace mouse trail

        if x == center_x and y == center_y:
            log("Mouse reached center, Phase 4 (floodfill) successful")
            break

        wall_added = False

        # check the walls around and update wall map
        if API.wallFront():
            if (walls[x][y] & wall_bits[heading]) == 0:
                update_walls(walls, x, y, heading, width, height)
                wall_added = True
        if API.wallRight():
            direction = (heading + 1) % 4
            if (walls[x][y] & wall_bits[direction]) == 0:
                update_walls(walls, x, y, direction, width, height)
                wall_added = True
        if API.wallLeft():
            direction = (heading + 3) % 4
            if (walls[x][y] & wall_bits[direction]) == 0:
                update_walls(walls, x, y, direction, width, height)
                wall_added = True

        # dynamic reflooding
        if wall_added:
            floodfill(distances, walls, center_x, center_y, width, height)

        # 1. Set the heading
        # look at all 4 neighbors to find the lowest distance
        best_heading = heading
        min_dist = INF

        for i in range(4):

            # check neighbor walls and if detected skip
            if (walls[x][y] & wall_bits[i]) != 0:
                continue
            
            nx = x + dx[i]
            ny = y + dy[i]

            if 0 <= nx < width and 0 <= ny < height: # check for maze bounds
                if distances[nx][ny] < min_dist:
                    min_dist = distances[nx][ny]
                    best_heading = i  # save the direction had lowest number

        # 2. Rotate to face best neighbor
        turn_diff = (best_heading - heading) % 4

        if turn_diff == 1:
            API.turnRight()
        elif turn_diff == 2:
            API.turnRight()
            API.turnRight()
        elif turn_diff == 3:
            API.turnLeft()
        
        # update internal rotation state
        heading = best_heading


        # 3. move and update coordinates
        API.moveForward()
        x += dx[heading]
        y += dy[heading]

        log(f"Moved to ({x}, {y}) | heading = {heading}")

if __name__ == "__main__":
    main()