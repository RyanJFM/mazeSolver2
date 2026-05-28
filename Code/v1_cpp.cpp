#include <iostream>
#include <string>
#include <queue>
#include <cmath>
#include "API.h"

const int INF = 9999;
const int WIDTH = 5;
const int HEIGHT = 5;

int distances[WIDTH][HEIGHT];
int walls[WIDTH][HEIGHT];

// Coordinate changes mapped to headings [N, E, S, W]
const int dx[4] = {0, 1, 0, -1};
const int dy[4] = {1, 0, -1, 0};

// Bitmasks
const int wall_bits[4] = {1, 2, 4, 8};

void log(const std::string& text) {
    std::cerr << text << std::endl;
}

void update_walls(int x, int y, int direction) {
    // Add wall to current cell
    walls[x][y] |= wall_bits[direction];

    // Calculate neighbor coordinates
    int nx = x + dx[direction];
    int ny = y + dy[direction];

    // Check bounds and prevent leaks
    if (0 <= nx && nx < WIDTH && 0 <= ny && ny < HEIGHT) {
        int opposite_dir = (direction + 2) % 4;
        walls[nx][ny] |= wall_bits[opposite_dir];
    }
}

void floodfill(int center_x, int center_y) {
    // Reset all distances to INF and clear the ghost text on simulator UI
    for (int i = 0; i < WIDTH; i++) {
        for (int j = 0; j < HEIGHT; j++) {
            distances[i][j] = INF;
            API::setText(i, j, ""); 
        }
    }

    // Set the center to 0 and add to Queue
    distances[center_x][center_y] = 0;
    
    // C++ uses std::queue with a std::pair to hold the (x,y) coordinates
    std::queue<std::pair<int, int>> q;
    q.push({center_x, center_y});

    // Queue operations
    while (!q.empty()) {
        std::pair<int, int> current = q.front();
        q.pop(); // Remove the item from the front of the queue
        
        int cx = current.first;
        int cy = current.second;

        for (int i = 0; i < 4; i++) {
            if ((walls[cx][cy] & wall_bits[i]) == 0) {
                int nx = cx + dx[i];
                int ny = cy + dy[i];

                // Initialize values to neighbors
                if (0 <= nx && nx < WIDTH && 0 <= ny && ny < HEIGHT) {
                    if (distances[nx][ny] == INF) {
                        distances[nx][ny] = distances[cx][cy] + 1;
                        q.push({nx, ny});
                        API::setText(nx, ny, std::to_string(distances[nx][ny]));
                    }
                }
            }
        }
    }
    API::setText(center_x, center_y, "0");
}

int main() {
    log("Booting Phase 1 & 2 (C++ Edition)...");

    int center_x = 2;
    int center_y = 2;

    // Initialize map
    for (int i = 0; i < WIDTH; i++) {
        for (int j = 0; j < HEIGHT; j++) {
            walls[i][j] = 0;
            // Manhattan distance formula
            distances[i][j] = std::abs(center_x - i) + std::abs(center_y - j);
            API::setText(i, j, std::to_string(distances[i][j]));
        }
    }

    int x = 0, y = 0;
    int heading = 0; // default heading = North

    while (true) {
        API::setColor(x, y, 'G');

        if (x == center_x && y == center_y) {
            log("Mouse reached center, Phase 4 (floodfill) successful");
            break;
        }

        bool wall_added = false;

        // Check the walls around and update wall map
        if (API::wallFront()) {
            if ((walls[x][y] & wall_bits[heading]) == 0) {
                update_walls(x, y, heading);
                wall_added = true;
            }
        }
        if (API::wallRight()) {
            int direction = (heading + 1) % 4;
            if ((walls[x][y] & wall_bits[direction]) == 0) {
                update_walls(x, y, direction);
                wall_added = true;
            }
        }
        if (API::wallLeft()) {
            int direction = (heading + 3) % 4;
            if ((walls[x][y] & wall_bits[direction]) == 0) {
                update_walls(x, y, direction);
                wall_added = true;
            }
        }

        // Dynamic reflooding
        if (wall_added) {
            floodfill(center_x, center_y);
        }

        // 1. Set the heading
        int best_heading = heading;
        int min_dist = INF;

        for (int i = 0; i < 4; i++) {
            if ((walls[x][y] & wall_bits[i]) != 0) {
                continue;
            }
            
            int nx = x + dx[i];
            int ny = y + dy[i];

            if (0 <= nx && nx < WIDTH && 0 <= ny && ny < HEIGHT) {
                if (distances[nx][ny] < min_dist) {
                    min_dist = distances[nx][ny];
                    best_heading = i;
                }
            }
        }

        // 2. Rotate to face best neighbor
        int turn_diff = (best_heading - heading) % 4;
        
        // C++ modulo fix
        if (turn_diff < 0) {
            turn_diff += 4; 
        }

        if (turn_diff == 1) {
            API::turnRight();
        } else if (turn_diff == 2) {
            API::turnRight();
            API::turnRight();
        } else if (turn_diff == 3) {
            API::turnLeft();
        }
        
        heading = best_heading;

        // 3. Move and update coordinates
        API::moveForward();
        x += dx[heading];
        y += dy[heading];

        log("Moved to (" + std::to_string(x) + ", " + std::to_string(y) + ") | heading = " + std::to_string(heading));
    }

    return 0;
}
