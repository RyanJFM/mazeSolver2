# Micromouse Navigation Algorithm

---

## Phase 1: Spatial Awareness
**The mouse tracks its absolute X/Y coordinates and heading.**

### Initial Setup
```python
x = 0, y = 0           # Bottom-left corner
heading = 0            # Direction: 0=North, 1=East, 2=South, 3=West
```

<img src="Maze/maze_5x5.png" alt="Maze_5x5" width="400">

### Heading Updates
After each turn command, update the heading:

| Action | Formula |
|--------|---------|
| **Turn Right** | `heading = (heading + 1) % 4` |
| **Turn Left** | `heading = (heading + 3) % 4` |

### Movement Updates
After each move forward, update X or Y based on current heading:

| Heading | Direction | Update |
|---------|-----------|--------|
| 0 | **North** | `y += 1` |
| 1 | **East** | `x += 1` |
| 2 | **South** | `y -= 1` |
| 3 | **West** | `x -= 1` |

---

## Phase 2: The Gravity Map
**Calculate Manhattan distances to determine pathfinding weights.**

### Distance Formula
```python
distances[x][y] = abs(2 - x) + abs(2 - y)
```

Each cell's distance represents its proximity to the center (2, 2). Lower values guide the mouse toward the goal.

---

## Phase 3: Wall Detection and Mapping
**The mouse senses obstacles and maintains an internal wall map.**

### Sensor Functions
- `API.wallFront()` — Detect wall ahead in current heading
- `API.wallRight()` — Detect wall to the right
- `API.wallLeft()` — Detect wall to the left

### Wall Storage
Walls are stored using **bitmask encoding**:

```python
walls[x][y]  # Each cell stores 4 bits for 4 directions
```

| Direction | Bit Value |
|-----------|-----------|
| **North** | `1` |
| **East** | `2` |
| **South** | `4` |
| **West** | `8` |

### Wall Update Logic
When a wall is detected, update **both the current cell and neighbor cell** to prevent boundary leaks:

```python
# Current cell
walls[x][y] |= wall_bits[direction]

# Neighbor cell  
walls[nx][ny] |= wall_bits[opposite_direction]
```

---

## Phase 4: Dynamic Floodfill Pathfinding
**Recalculate the distance map using BFS whenever walls are detected.**

### BFS Algorithm Steps
1. Reset all distances to `INF` (9999)
2. Set center cell distance to `0` and add to queue
3. Process queue: for each cell, examine all 4 neighbors
4. For each valid neighbor without a wall:
   - Calculate: `distances[nx][ny] = distances[cx][cy] + 1`
   - Add to queue
5. Update maze display with new distances

### Key Feature
**Dynamic Re-flooding:** When new walls are discovered, floodfill re-runs to update the optimal path.

---

## Phase 5: Optimal Movement Algorithm
**The mouse moves greedily toward the center using the updated distance map.**

### Decision Process
1. Look at all 4 neighbors from current position
2. Skip neighbors with walls: `(walls[x][y] & wall_bits[i]) != 0`
3. Find neighbor with **minimum distance value**
4. Calculate rotation needed: `turn_diff = (best_heading - heading) % 4`
5. Execute movement:
   - `turn_diff == 1` → Turn right
   - `turn_diff == 2` → Turn right twice (U-turn)
   - `turn_diff == 3` → Turn left
6. Move forward and update coordinates based on heading
7. Repeat until reaching center `(x == 2, y == 2)`

### Success Condition
**Mouse traces a green path to the center cell and completes the maze**

