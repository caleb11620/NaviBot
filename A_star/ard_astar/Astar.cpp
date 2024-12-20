#include "Astar.h"
// replace reverse and find functionality
#include <algorithm>
#include <stdio.h>
//#include <iostream>
//#include <iomanip>
// replace sqrt
//#include <cmath>

// constructor
// aWidth and aHeight init 0
Astar::Astar() : aWidth(0), aHeight(0) {}

void Astar::interpretBitmap(const std::vector<std::vector<bool>>& bmp) {
    printf("AStar::Interpreting bitmap...\n");
    printf("Bitmap width: %d, height: %d\n", bmp[0].size(), bmp.size());

    // printing out the bitmap to make sure function sees it
    for (const auto& row : bmp) {
        for (bool pixel : row) {
            printf("%d", pixel);
        }
        printf("\n");
    }

    // adjusted width and height
    aWidth = bmp[0].size();
    aHeight = bmp.size();

    initializeGrid();
    for (int y{0}; y < aHeight; ++y) {
        for (int x{0}; x < aWidth; ++x) {
            grid[y][x]->isObstacle = bmp[y][x];
        }
    }
}

void Astar::initializeGrid() {
    grid.resize(aHeight, std::vector<Node*>(aWidth, nullptr));

    for (int y = 0; y < aHeight; ++y) {
        for (int x = 0; x < aWidth; ++x) {
            grid[y][x] = new Node(x, y);
        }
    }
}

void Astar::cleanupGrid() {
    for (int y = 0; y < aHeight; ++y) {
        for (int x = 0; x < aWidth; ++x) {
            delete grid[y][x];
        }
    }
}

bool Astar::isValid(int x, int y) {
    return (x >= 0 && x < aWidth && y >= 0 && y < aHeight && !grid[y][x]->isObstacle);
}

float Astar::heuristic(int x1, int y1, int x2, int y2) {
    switch (heuristicType) {
        case HeuristicType::MANHATTAN:
            // manhattan distance
            return abs(x1 - x2) + abs(y1 - y2);

        case HeuristicType::EUCLIDEAN:
            // euclidean distance
            return sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2));

        case HeuristicType::CHEBYSHEV:
            // chebyshev distance
            return std::max(abs(x1 - x2), abs(y1 - y2));

        case HeuristicType::OCTILE:
            goto default_jmp;

        default:
            default_jmp:
                // octile distance
                int dx = abs(x1 - x2);
                int dy = abs(y1 - y2);
                //return (dx + dy) + (std::sqrt(2) - 2) * std::min(dx, dy);
                // octile distance with variable diagonal cost
                float D = 1.0f; // cost of moving straight
                float D2 = 1.414214f; // cost of moving diagonally
                return D * (dx + dy) + (D2 - 2 * D) * std::min(dx, dy);
    }
}

std::vector<Node*> Astar::smoothPath(const std::vector<Node*>& originalPath) {
    if (originalPath.size() <= 2) return originalPath;
    
    std::vector<Node*> smoothedPath;
    smoothedPath.push_back(originalPath[0]);
    
    size_t current = 0;
    while (current < originalPath.size() - 1) {
        size_t furthest = current + 1;
        
        // Look ahead as far as possible while maintaining line of sight
        for (size_t test = current + 2; test < originalPath.size(); ++test) {
            if (hasLineOfSight(originalPath[current], originalPath[test])) {
                furthest = test;
            }
        }
        
        smoothedPath.push_back(originalPath[furthest]);
        current = furthest;
    }
    
    return smoothedPath;
}

bool Astar::hasLineOfSight(Node* start, Node* end) {
    int x0 = start->x;
    int y0 = start->y;
    int x1 = end->x;
    int y1 = end->y;
    
    int dx = abs(x1 - x0);
    int dy = abs(y1 - y0);
    int x = x0;
    int y = y0;
    int n = 1 + dx + dy;
    int x_inc = (x1 > x0) ? 1 : -1;
    int y_inc = (y1 > y0) ? 1 : -1;
    int error = dx - dy;
    dx *= 2;
    dy *= 2;
    
    for (; n > 0; --n) {
        if (grid[y][x]->isObstacle) return false;
        
        if (error > 0) {
            x += x_inc;
            error -= dy;
        } else {
            y += y_inc;
            error += dx;
        }
    }
    
    return true;
}

std::vector<Node*> Astar::getNeighbors(Node* node) {
    std::vector<Node*> neighbors;
    
    // Order movements: cardinal directions first, then diagonals
    int dx[] = { 0, 1, 0, -1, -1, 1, 1, -1 };
    int dy[] = { -1, 0, 1, 0, -1, 1, -1, 1 };
    
    // If using Manhattan distance, only use first 4 directions
    int numDirections = (heuristicType == HeuristicType::MANHATTAN) ? 4 : 8;
    
    for (int i = 0; i < numDirections; ++i) {
        int newX = node->x + dx[i];
        int newY = node->y + dy[i];
        if (isValid(newX, newY)) {
            neighbors.push_back(grid[newY][newX]);
        }
    }
    return neighbors;
}

Node* Astar::findLowestF(std::vector<Node*>& openSet) { Node* lowest = openSet[0];
    int lowestIndex = 0;

    for (int i = 1; i < openSet.size(); ++i) {
        if (openSet[i]->f < lowest->f) {
            lowest = openSet[i];
            lowestIndex = i;
        }
    }

    openSet.erase(openSet.begin() + lowestIndex);
    return lowest;
}

std::vector<Node*> Astar::reconstructPath(Node* current) {
    std::vector<Node*> path;
    while (current != nullptr) {
        path.push_back(current);
        current = current->parent;
    }
    std::reverse(path.begin(), path.end());
    return path;
}

std::vector<Node*> Astar::algorithm(Node* start, Node* goal) {
    std::vector<Node*> openSet;
    std::vector<Node*> closedSet;

    start->g = 0;
    start->h = heuristic(start->x, start->y, goal->x, goal->y);
    start->f = start->g + start->h;

    openSet.push_back(start);

    while (!openSet.empty()) {
        Node* current = findLowestF(openSet);

        if (current == goal) {
            return reconstructPath(current);
        }

        closedSet.push_back(current);

        for (Node* neighbor : getNeighbors(current)) {
            if (std::find(closedSet.begin(), closedSet.end(), neighbor) != closedSet.end()) {
                continue;
            }

            int tentativeG = current->g + 1;

            auto openNode = std::find(openSet.begin(), openSet.end(), neighbor);

            if (openNode == openSet.end()) {
                openSet.push_back(neighbor);
            } else if (tentativeG >= neighbor->g) {
                continue;
            }

            neighbor->parent = current;
            neighbor->g = tentativeG;
            neighbor->h = heuristic(neighbor->x, neighbor->y, goal->x, goal->y);
            neighbor->f = neighbor->g + neighbor->h;
        }
    }

    return std::vector<Node*>(); // No path found
}

// current assumption in determining the start is it is the first non-obstacle in the bottom row
Node* Astar::determineStartNode() {
    for (int x = 0; x < aWidth; ++x) {
        if (!grid[aHeight-1][x]->isObstacle) {
            grid[aHeight-1][x]->start = true;
            return grid[aHeight-1][x];
        }
    }

    // No valid start
    return nullptr;
}

// current assumption in determining the goal is it is the first non-obstacle in the top row
Node* Astar::determineGoalNode() {
    for (int x = 0; x < aWidth; ++x) {
        if (!grid[0][x]->isObstacle) {
            grid[0][x]->exit = true;
            return grid[0][x];
        }
    }
    // No valid goal found
    return nullptr;
}