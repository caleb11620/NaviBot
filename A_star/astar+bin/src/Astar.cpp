#include "../include/Astar.hpp"
#include <algorithm>
#include <stdio.h>
#include <iostream>
#include <iomanip>
#include <cmath>

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
    // manhattan distance
    //return abs(x1 - x2) + abs(y1 - y2);

    // euclidean distance
    //return sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2));

    // chebyshev distance
    //return std::max(abs(x1 - x2), abs(y1 - y2));

    // octile distance
    int dx = abs(x1 - x2);
    int dy = abs(y1 - y2);
    //return (dx + dy) + (std::sqrt(2) - 2) * std::min(dx, dy);

    // octile distance with variable diagonal cost
    float D = 1.0; // cost of moving straight
    float D2 = sqrt(2); // cost of moving diagonally
    return D * (dx + dy) + (D2 - 2 * D) * std::min(dx, dy);
}

std::vector<Node*> Astar::getNeighbors(Node* node) {
    std::vector<Node*> neighbors;
    int dx[] = { -1, 0, 1, 0, -1, 1, 1, -1 };
    int dy[] = { 0, 1, 0, -1, 1, 1, -1, -1 };

    for (int i{0}; i < 8; ++i) {
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
Node* Astar::determineStartNode(int inputX, int inputY) {
    if (inputX || inputY) {
        grid[inputY][inputX]->start = true;
        return grid[inputY][inputX];
    }
    for (int x = 0; x < aWidth; ++x) {
        if (!grid[aHeight-1][x]->isObstacle) {
            grid[aHeight-1][x]->start = true;
            return grid[aHeight-1][x];
        }
    }

    return nullptr;
    //grid[aHeight-1][(aWidth-1)/2]->start = true;
    //return grid[aHeight-1][(aWidth-1)/2];
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
    //grid[0][(aWidth-1)/2]->exit = true;
    //return grid[0][(aWidth-1)/2];
}

void Astar::printGrid(const std::vector<Node*>& path) {
    char displayGrid[aHeight][aWidth];

    for (int i{0}; i < aHeight; ++i) {
        for (int j{0}; j < aWidth; ++j) {
            displayGrid[i][j] = grid[i][j]->isObstacle ? '1' : '0';
        }
    }

    for (size_t i = 0; i < path.size(); ++i) {
        displayGrid[path[i]->y][path[i]->x] = (i == 0) ? 'S' : ((i == path.size() - 1) ? 'G' : '*');
    }

    // Print the grid
    /*
    std::cout << "  ";
    for (int i = 0; i < aWidth; ++i) {
        std::cout << std::setw(2) << i << " ";
    }
    std::cout << std::endl;
    */

    for (int i = 0; i < aHeight; ++i) {
        //std::cout << std::setw(2) << i << " ";
        for (int j = 0; j < aWidth; ++j) {
            std::cout << displayGrid[i][j];
        }
        std::cout << std::endl;
    }
}