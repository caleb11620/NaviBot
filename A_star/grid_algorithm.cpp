#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <iomanip>

const int GRID_SIZE = 10;

// A* pathfinding algorithm
// https://en.wikipedia.org/wiki/A*_search_algorithm

// g score - the cost of the path from the start node to the current node
// h score - heuristic that estimates the cost of the cheapest path from the current node to the goal node
// f score - sum of g and h. It represents the total cost of the path through the current node

struct Node {
    int x, y;
    int g, h, f;
    Node* parent;
    bool isObstacle;

    Node(int x, int y) : x(x), y(y), g(0), h(0), f(0), parent(nullptr), isObstacle(false) {}
};

Node* grid[GRID_SIZE][GRID_SIZE];

void initializeGrid() {
    for (int i = 0; i < GRID_SIZE; ++i) {
        for (int j = 0; j < GRID_SIZE; ++j) {
            grid[i][j] = new Node(j, i);
        }
    }
}

void cleanupGrid() {
    for (int i = 0; i < GRID_SIZE; ++i) {
        for (int j = 0; j < GRID_SIZE; ++j) {
            delete grid[i][j];
        }
    }
}

bool isValid(int x, int y) {
    return x >= 0 && x < GRID_SIZE && y >= 0 && y < GRID_SIZE && !grid[y][x]->isObstacle;
}

int heuristic(int x1, int y1, int x2, int y2) {
    return std::abs(x1 - x2) + std::abs(y1 - y2);
}

std::vector<Node*> getNeighbors(Node* node) {
    std::vector<Node*> neighbors;
    int dx[] = {-1, 0, 1, 0};
    int dy[] = {0, 1, 0, -1};

    for (int i = 0; i < 4; ++i) {
        int newX = node->x + dx[i];
        int newY = node->y + dy[i];

        if (isValid(newX, newY)) {
            neighbors.push_back(grid[newY][newX]);
        }
    }

    return neighbors;
}

Node* findLowestF(std::vector<Node*>& openSet) {
    Node* lowest = openSet[0];
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

std::vector<Node*> reconstructPath(Node* current) {
    std::vector<Node*> path;
    while (current != nullptr) {
        path.push_back(current);
        current = current->parent;
    }
    std::reverse(path.begin(), path.end());
    return path;
}

std::vector<Node*> astar(Node* start, Node* goal) {
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

void printGrid(const std::vector<Node*>& path) {
    char displayGrid[GRID_SIZE][GRID_SIZE];

    // Initialize grid with empty cells and obstacles
    for (int i = 0; i < GRID_SIZE; ++i) {
        for (int j = 0; j < GRID_SIZE; ++j) {
            displayGrid[i][j] = grid[i][j]->isObstacle ? '#' : '.';
        }
    }

    // Mark the path
    for (size_t i = 0; i < path.size(); ++i) {
        displayGrid[path[i]->y][path[i]->x] = (i == 0) ? 'S' : ((i == path.size() - 1) ? 'G' : '*');
    }

    // Print the grid
    std::cout << "  ";
    for (int i = 0; i < GRID_SIZE; ++i) {
        std::cout << std::setw(2) << i << " ";
    }
    std::cout << std::endl;

    for (int i = 0; i < GRID_SIZE; ++i) {
        std::cout << std::setw(2) << i << " ";
        for (int j = 0; j < GRID_SIZE; ++j) {
            std::cout << displayGrid[i][j] << "  ";
        }
        std::cout << std::endl;
    }
}

void setObstacle(int x, int y) {
    if (x >= 0 && x < GRID_SIZE && y >= 0 && y < GRID_SIZE) {
        grid[y][x]->isObstacle = true;
    }
}

void setObstacleLine(int x1, int x2, int y1, int y2) {
    if (x1 == x2) {
        for (int i = std::min(y1, y2); i <= std::max(y1, y2); ++i) {
            setObstacle(x1, i);
        }
    } else if (y1 == y2) {
        for (int i = std::min(x1, x2); i <= std::max(x1, x2); ++i) {
            setObstacle(i, y1);
        }
    }
}

int main() {
    initializeGrid();

    // Set some obstacles
    setObstacleLine(0, 7, 1, 1);
    setObstacleLine(1, 9, 4, 4);
    setObstacleLine(3, 3, 6, 9);
    setObstacleLine(5, 5, 6, 8);

    Node* start = grid[0][0];
    Node* goal = grid[9][9];

    std::vector<Node*> path = astar(start, goal);

    if (path.empty()) {
        std::cout << "No path found!" << std::endl;
    } else {
        std::cout << "Path found:" << std::endl;
        printGrid(path);
    }

    cleanupGrid();

    return 0;
}