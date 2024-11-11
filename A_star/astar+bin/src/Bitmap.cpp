#include "../include/Bitmap.hpp"

// constructor
Bitmap::Bitmap() : data(MAX_HEIGHT, std::vector<int>(MAX_WIDTH, 0)) {}

int Bitmap::getPixel(int x, int y)
{
    return data[y][x];
}

bool Bitmap::read(const std::string &filename)
{
    std::ifstream file(filename);
    if (!file.is_open())
    {
        std::cerr << "Error opening file for reading: " << filename << std::endl;
        return false;
    }

    for (int y = 0; y < MAX_HEIGHT; ++y)
    {
        for (int x = 0; x < MAX_WIDTH; ++x)
        {
            char c;
            file >> c;
            data[y][x] = c - '0'; // convert char to int
        }
    }

    file.close();
    return true;
}

bool Bitmap::write(const std::string &filename)
{
    std::ofstream file(filename);
    if (!file.is_open())
    {
        std::cerr << "Error opening file for writing: " << filename << std::endl;
        return false;
    }

    for (int y = 0; y < intHeight; ++y)
    {
        for (int x = 0; x < intWidth; ++x)
        {
            file << (data[y][x] ? '1' : '0');
        }
        file << std::endl;
    }

    file.close();
    return true;
}

void Bitmap::invertPixel(int x, int y)
{
    if (x >= 0 && x < MAX_WIDTH && y >= 0 && y < MAX_HEIGHT)
    {
        data[y][x] = !data[y][x];
    }
}

std::vector<int> Bitmap::findEmptyRows() {
    std::vector<int> emptyRows(MAX_HEIGHT, 0);
    for (int y = 0; y < MAX_HEIGHT; ++y) {
        bool allZeros = true;
        bool allOnes = true;
        
        for (int x = 0; x < MAX_WIDTH; ++x) {
            if (data[y][x] != 0) {
                allZeros = false;
            }
            if (data[y][x] != 1) {
                allOnes = false;
            }
            if (!allZeros && !allOnes) break;
        }
        
        // Mark row for removal if it's all zeros or all ones
        if (allZeros || allOnes) {
            emptyRows[y] = 1;
        }
    }
    return emptyRows;
}

std::vector<int> Bitmap::findEmptyCols() {
    std::vector<int> emptyCols(MAX_WIDTH, 0);
    for (int x = 0; x < MAX_WIDTH; ++x) {
        bool allZeros = true;
        bool allOnes = true;
        
        for (int y = 0; y < MAX_HEIGHT; ++y) {
            if (data[y][x] != 0) {
                allZeros = false;
            }
            if (data[y][x] != 1) {
                allOnes = false;
            }
            if (!allZeros && !allOnes) break;
        }
        
        // Mark column for removal if it's all zeros or all ones
        if (allZeros || allOnes) {
            emptyCols[x] = 1;
        }
    }
    return emptyCols;
}

// Removes any row or column that is entirely 0 or entirely 1
void Bitmap::removeEmptyRowsAndColumns() {
    std::vector<int> rowsToRemove = findEmptyRows();
    std::vector<int> colsToRemove = findEmptyCols();

    // Remove marked rows
    data.erase(
        std::remove_if(data.begin(), data.end(),
            [&rowsToRemove, this](const std::vector<int>& row) {
                return rowsToRemove[&row - &data[0]] == 1;
            }),
        data.end()
    );

    // Remove marked columns
    for (auto& row : data) {
        for (int x = intWidth - 1; x >= 0; --x) {
            if (colsToRemove[x] == 1) {
                row.erase(row.begin() + x);
            }
        }
    }

    // Update dimensions
    intHeight = data.size();
    intWidth = data[0].size();
}