#ifndef ASTAR_HPP
#define ASTAR_HPP

//#include <std::vector.h>
#include <Cpp_Standard_Library.h>

struct Node {
    int x, y;
    float g, h, f;
    Node* parent;
    bool isObstacle;
    bool start;
    bool exit;

    Node(int x, int y) : x(x), y(y), g(0.0), h(0.0), f(0.0), parent(nullptr), isObstacle(false), start(false), exit(false) {}
};

class Astar {
    private:
        std::vector<std::vector<Node*>> grid;
        int aWidth;
        int aHeight;

    public:
        Astar();
        void interpretBitmap(const std::vector<std::vector<bool>>& bmp);
        void initializeGrid();
        void cleanupGrid();
        bool isValid(int x, int y);
        float heuristic(int x1, int y1, int x2, int y2);
        std::vector<Node*> getNeighbors(Node* node);
        Node* findLowestF(std::vector<Node*>& openSet);
        std::vector<Node*> reconstructPath(Node* current);
        std::vector<Node*> algorithm(Node* startNode, Node* goalNode);
        Node* determineStartNode();
        Node* determineGoalNode();
        void printGrid(const std::vector<Node*>& path);
};

#endif // ASTAR_HPP