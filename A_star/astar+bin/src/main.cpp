#include <iostream>
#include "../include/Bitmap.hpp"
#include "../include/Astar.hpp"

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

    astar.printGrid(path);

    astar.cleanupGrid();

    return 0;
}