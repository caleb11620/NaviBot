#include <iostream>
#include <cmath>
#include <utility>
#include <string>
#include <map>
#include <tuple>
#include "../include/Bitmap.hpp"
#include "../include/Astar.hpp"

std::tuple<char, int, std::string> calculateDirectionAngleAndHeading(int x1, int y1, int x2, int y2, const std::string& currentHeading);

struct step {
    int angle;
    int distance {1};
    char direction;
};

int main(int argc, char **argv)
{
    printf("Hello, from bitmap!\n");

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

    Node* start = astar.determineStartNode();
    Node* exit = astar.determineGoalNode();

    std::cout << "startNode x:" << start->x << " y:" << start->y << std::endl;
    std::cout << "exitNode x:" << exit->x << " y:" << exit->y << std::endl;

    std::vector<Node*> path = astar.algorithm(start, exit);
    std::vector<step> solution;

    astar.printGrid(path);

    char direction;
    int angle;
    std::string newHeading{"N"};
    std::string heading = newHeading;
    std::cout << "Path vector: (x,y)" << std::endl;
    for (int i = 0 ; i < path.size()-1; ++i) {
        std::cout << "(" << path[i]->x << "," << path[i]->y << ")";
        std::cout << " -> " << "(" << path[i+1]->x << "," << path[i+1]->y << ")" << std::endl;
        auto [direction, angle, newHeading] = calculateDirectionAngleAndHeading(path[i]->x, path[i]->y, path[i+1]->x, path[i+1]->y, heading);
        heading = newHeading;
        std::cout << "Direction: " << direction << ", Angle: " << angle << ", New Heading: " << newHeading << std::endl;
        step val;
        val.direction = direction;
        val.angle = angle;
        val.distance = 1;
        solution.push_back(val);
    }
    for (int i = 0; i < solution.size()-1; ++i) {
        if (solution[i].direction == 'F' && solution[i].direction == solution[i+1].direction) {
            solution[i].distance++;
            solution.erase(solution.begin() + i);
        }
    }

    for (int i = 0; i < solution.size()-1; ++i) {
        std::cout << "step " << i << ": " << "angle: " << solution[i].angle << " direction: " << solution[i].direction << " distance: " << solution[i].distance;
        std::cout << std::endl;
    }
    astar.cleanupGrid();

    return 0;
}

std::tuple<char, int, std::string> calculateDirectionAngleAndHeading(int x1, int y1, int x2, int y2, const std::string& currentHeading) {
    int dx = x2 - x1;
    int dy = (y1 - y2);  // Invert y-axis if needed
    
    // Define movement types
    const std::map<std::pair<int, int>, std::string> movements = {
        {{0, 1}, "N"}, {{1, 1}, "NE"}, {{1, 0}, "E"}, {{1, -1}, "SE"},
        {{0, -1}, "S"}, {{-1, -1}, "SW"}, {{-1, 0}, "W"}, {{-1, 1}, "NW"}
    };
    
    // Find the new heading
    auto it = movements.find({dx, dy});
    if (it == movements.end()) {
        return {'X', -1, currentHeading};  // Invalid movement
    }
    std::string newHeading = it->second;
    
    // If the heading hasn't changed, it's a forward movement
    if (newHeading == currentHeading) {
        return {'F', 0, newHeading};
    }
    
    // Define the angle differences between directions
    const std::map<std::pair<std::string, std::string>, std::pair<char, int>> directionChanges = {
        {{"N", "NE"}, {'R', 45}},  {{"N", "E"}, {'R', 90}},   {{"N", "SE"}, {'R', 135}},
        {{"N", "S"}, {'R', 180}},  {{"N", "SW"}, {'L', 135}}, {{"N", "W"}, {'L', 90}},
        {{"N", "NW"}, {'L', 45}},  {{"NE", "E"}, {'R', 45}},  {{"NE", "SE"}, {'R', 90}},
        {{"NE", "S"}, {'R', 135}}, {{"NE", "SW"}, {'R', 180}},{{"NE", "W"}, {'L', 135}},
        {{"NE", "NW"}, {'L', 90}}, {{"NE", "N"}, {'L', 45}},  {{"E", "SE"}, {'R', 45}},
        {{"E", "S"}, {'R', 90}},   {{"E", "SW"}, {'R', 135}}, {{"E", "W"}, {'R', 180}},
        {{"E", "NW"}, {'L', 135}}, {{"E", "N"}, {'L', 90}},   {{"E", "NE"}, {'L', 45}},
        {{"SE", "S"}, {'R', 45}},  {{"SE", "SW"}, {'R', 90}}, {{"SE", "W"}, {'R', 135}},
        {{"SE", "NW"}, {'R', 180}},{{"SE", "N"}, {'L', 135}}, {{"SE", "NE"}, {'L', 90}},
        {{"SE", "E"}, {'L', 45}},  {{"S", "SW"}, {'R', 45}},  {{"S", "W"}, {'R', 90}},
        {{"S", "NW"}, {'R', 135}}, {{"S", "N"}, {'R', 180}},  {{"S", "NE"}, {'L', 135}},
        {{"S", "E"}, {'L', 90}},   {{"S", "SE"}, {'L', 45}},  {{"SW", "W"}, {'R', 45}},
        {{"SW", "NW"}, {'R', 90}}, {{"SW", "N"}, {'R', 135}}, {{"SW", "NE"}, {'R', 180}},
        {{"SW", "E"}, {'L', 135}}, {{"SW", "SE"}, {'L', 90}}, {{"SW", "S"}, {'L', 45}},
        {{"W", "NW"}, {'R', 45}},  {{"W", "N"}, {'R', 90}},   {{"W", "NE"}, {'R', 135}},
        {{"W", "E"}, {'R', 180}},  {{"W", "SE"}, {'L', 135}}, {{"W", "S"}, {'L', 90}},
        {{"W", "SW"}, {'L', 45}},  {{"NW", "N"}, {'R', 45}},  {{"NW", "NE"}, {'R', 90}},
        {{"NW", "E"}, {'R', 135}}, {{"NW", "SE"}, {'R', 180}},{{"NW", "S"}, {'L', 135}},
        {{"NW", "SW"}, {'L', 90}}, {{"NW", "W"}, {'L', 45}}
    };
    
    // Find the direction change
    auto change = directionChanges.find({currentHeading, newHeading});
    if (change != directionChanges.end()) {
        return {change->second.first, change->second.second, newHeading};
    }
    
    // If no change found (shouldn't happen with a complete directionChanges map)
    return {'X', -1, newHeading};
}