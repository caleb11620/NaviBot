#ifndef BITMAP_HPP
#define BITMAP_HPP

#include <vector>
#include <SD.h>
#include <SPI.h>
#include <stdio.h>
#include <iostream>
#include <algorithm>

const int MAX_WIDTH = 400;
const int MAX_HEIGHT = 400;

// Bitmap class for everything about reading/writing .BIN
class Bitmap
{
    private:
        std::vector<std::vector<int>> data;

        int intWidth {MAX_WIDTH};
        int intHeight {MAX_HEIGHT};

    public:
        Bitmap();

        int getWidth() const { return intWidth; }
        int getHeight() const { return intHeight; }
        std::vector<std::vector<int>> getData() const { return data; }

        int getPixel(int x, int y);
        bool read(File &file);
        bool write(File &file);
        void invertPixel(int x, int y);

        std::vector<int> findEmptyRows();
        std::vector<int> findEmptyCols();
        void removeEmptyRowsAndColumns();
};

#endif // BITMAP_HPP