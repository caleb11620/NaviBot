#include <iostream>
#include <cmath>
#include <utility>
#include <string>
#include <map>
#include <tuple>
#include "../include/Bitmap.hpp"
#include "../include/Astar.hpp"

std::tuple<char, int, std::string> calculateDirectionAngleAndHeading(int x1, int y1, int x2, int y2, const std::string& currentHeading);

struct outcome {
    Turn turn;
    int angle;
    int distance {1};
};

int main(int argc, char **argv)
{
    std::cout << "Have " << argc << " arguments:\n";
    for (int i = 0; i < argc; ++i) {
        std::cout << argv[i] << "\n";
    }

    Bitmap bmp;
    if (bmp.read(argv[1])) {
        std::cout << "Bitmap successfully read.\n";
    }

    bmp.removeEmptyRowsAndColumns();
    if (bmp.write("cropped_output.bin")) {
        std::cout << "Bitmap successfully written.\n";
    }
    std::cout << "Adjusted bitmap width: " << bmp.getWidth() << " height: " << bmp.getHeight() << std::endl;

    Astar astar;
    auto astardata = bmp.getData();
    astar.interpretBitmap(astardata);

    // determine start and end nodes
    int inX = 0;
    int inY = 0;
    int exitX = 0;
    int exitY = 0;
    if (argc == 6) {
        inX = std::stoi(argv[2]);
        inY = std::stoi(argv[3]);
        exitX = std::stoi(argv[4]);
        exitY = std::stoi(argv[5]);
    }
    Node* start = astar.determineStartNode(inX, inY);
    Node* exit = astar.determineGoalNode(exitX, exitY);
    std::cout << "startNode x:" << start->x << " y:" << start->y << std::endl;
    std::cout << "exitNode x:" << exit->x << " y:" << exit->y << std::endl;

    // set heuristic (cost for movement/path determination) and start alg.
    astar.setHeuristicType(HeuristicType::MANHATTAN);
    std::vector<Node*> path = astar.algorithm(start, exit);
    //path = astar.smoothPath(path);
    std::vector<outcome> solution;

    // attempts to print a visualization of the path in the maze
    astar.printGrid(path);

    // printing out every coordinate-to-coordinate movement
    // along with direction, angle, and heading to make movement
    Heading newHead = Heading::N; // assuming bot starts facing north at start location in map
    Heading head = newHead;
    for (int i = 0 ; i < path.size()-1; ++i) {
        std::cout << "(" << path[i]->x << "," << path[i]->y << ")";
        std::cout << " -> " << "(" << path[i+1]->x << "," << path[i+1]->y << ") ";
        auto [direction, angle, newHeading] = astar.calculateSolutionVars(path[i]->x, path[i]->y, path[i+1]->x, path[i+1]->y, head);
        head = newHeading;
        std::cout << "Direction: " << static_cast<char>(direction) << ", Angle: " << angle << std::endl;
        outcome val = {direction, angle, 1};
        solution.push_back(val);
    }
    // when direction is unchanging combine & erase steps by adding 'distance'
    for (int i = 0; i < solution.size()-1; ++i) {
        if (solution[i].turn == Turn::FORWARD && solution[i].turn == solution[i+1].turn) {
            solution[i].distance++;
            solution.erase(solution.begin() + i+1);
            --i;
        }
    }
    // print out these steps
    for (int i = 0; i < solution.size()-1; ++i) {
        std::cout << "step " << i << ": " << "angle: " << solution[i].angle << " direction: " << static_cast<char>(solution[i].turn) << " distance: " << solution[i].distance;
        std::cout << std::endl;
    }
    // memory cleanup
    astar.cleanupGrid();
    return 0;
}