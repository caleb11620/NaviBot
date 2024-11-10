#ifndef ASTAR_HPP
#define ASTAR_HPP

#include <vector>
#include <array>

enum class Heading {
    N, NE, E, SE, S, SW, W, NW
};
enum class Turn {
    FORWARD = 'F',
    RIGHT = 'R',
    LEFT = 'L',
    INVALID = 'X'
};
enum class HeuristicType {
    MANHATTAN,
    EUCLIDEAN,
    CHEBYSHEV,
    OCTILE
};
// representation of every cm2 in map
struct Node {
    int x, y;       // grid coordinates
    float g, h, f;  // A* necessary values
    Node* parent;
    bool isObstacle;
    bool start;
    bool exit;

    // init
    Node(int x, int y) : x(x), y(y), g(0.0), h(0.0), f(0.0), parent(nullptr),
        isObstacle(false), start(false), exit(false){}
};
// step-by-step instructions for solution
struct step {
    // how to physically get to the next location
    Turn turn;  // which way to turn (or not, stay forward) relative to bot
    int angle;              // how far to turn
};
class Astar {
    private:
        HeuristicType heuristicType = HeuristicType::OCTILE;
        std::vector<std::vector<Node*>> grid;
        std::vector<step> solution;
        int aWidth;
        int aHeight;


    public:
        Astar();
        void interpretBitmap(const std::vector<std::vector<bool>>& bmp);
        void initializeGrid();
        void cleanupGrid();
        bool isValid(int x, int y);
        void setHeuristicType(HeuristicType type) {heuristicType = type;}
        float heuristic(int x1, int y1, int x2, int y2);
        std::vector<Node*> smoothPath(const std::vector<Node*>& originalPath);
        bool hasLineOfSight(Node* start, Node* end);
        std::vector<Node*> getNeighbors(Node* node);
        Node* findLowestF(std::vector<Node*>& openSet);
        std::vector<Node*> reconstructPath(Node* current);
        std::vector<Node*> algorithm(Node* startNode, Node* goalNode);
        Node* determineStartNode(int inputX, int inputY);
        Node* determineGoalNode(int exitX, int exitY);
        void printGrid(const std::vector<Node*>& path);

        static step calculateTurn(Heading from, Heading to);
        static std::tuple<Turn, int, Heading>calculateSolutionVars(int x1, int y1, int x2, int y2, Heading currHeading);
};

#endif // ASTAR_HPP