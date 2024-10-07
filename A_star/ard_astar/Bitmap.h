#ifndef BITMAP_HPP
#define BITMAP_HPP

#include <Cpp_Standard_Library.h>
#include <vector>
//#include <std::vector.h>
// replace fstream with sd card read/write library
//#include <fstream>
#include <stdio.h>
// replace io code with serial print or delete
//#include <iostream>
// replace remove_if functionality
//#include <algorithm>

const int MAX_WIDTH = 400;
const int MAX_HEIGHT = 400;

// Bitmap class for everything about reading/writing .BIN
class Bitmap
{
    private:
        std::vector<std::vector<bool>> data;

        int origWidth;
        int origHeight;
        int intWidth {MAX_WIDTH};
        int intHeight {MAX_HEIGHT};

    public:
        Bitmap();

        int getWidth() const { return intWidth; }
        int getHeight() const { return intHeight; }
        std::vector<std::vector<bool>> getData() const { return data; }

        int getPixel(int x, int y);
        bool read(const String &filename);
        bool write(const String &filename);
        void invertPixel(int x, int y);

        std::vector<int> findEmptyRows();
        std::vector<int> findEmptyCols();
        void removeEmptyRowsAndColumns();
};

#endif // BITMAP_HPP