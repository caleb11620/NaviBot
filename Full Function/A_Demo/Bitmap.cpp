#include "Bitmap.h"
#include "Arduino.h"
#include "debug_macros.h"


// constructor
Bitmap::Bitmap() : data(MAX_HEIGHT, std::vector<int>(MAX_WIDTH, 0)) {}

int Bitmap::getPixel(int x, int y)
{
    return data[y][x];
}

bool Bitmap::read(File &file)
{
    if (!file)
    {
        return false;
    }

    int y = 0, x = 0;
    while (file.available() && y < MAX_HEIGHT) {
        char c = file.read();
        
        // need to skip newline chars or coordinates ruined
        if (c == '\n') {
            if (x == 0) {
                continue;
            }
        }

        if (c >= '0' && c <= '3') {
            data[y][x] = c - '0'; // char to int conversion
            ++x;

            // move to next row
            if (x == MAX_WIDTH) {
                x = 0;
                ++y;
            }
        }
    }

    file.close();
    // printing
    for (const auto& row : data) {
        for (bool pixel : row) {
            DEBUG_PRINTF("%d", pixel);
        }
        DEBUG_PRINTF("\n");
    }
    return true;
}

bool Bitmap::write(File &file)
{
    if (!file)
    {
        return false;
    }

    for (int y = 0; y < intHeight; ++y)
    {
        for (int x = 0; x < intWidth; ++x)
        {
            file.write(data[y][x] ? '1' : '0');
        }
        file.println();
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
    std::vector<int> rowsToRemove(MAX_HEIGHT, 0);
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
        
        if (allZeros || allOnes) {
            DEBUG_PRINTF("Found a row to remove\n");
            rowsToRemove[y] = 1;
        }
    }
    return rowsToRemove;
}

std::vector<int> Bitmap::findEmptyCols() {
    std::vector<int> colsToRemove(MAX_WIDTH, 0);
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
        
        if (allZeros || allOnes) {
            DEBUG_PRINTF("Found a column to remove\n");
            colsToRemove[x] = 1;
        }
    }
    return colsToRemove;
}

void Bitmap::removeEmptyRowsAndColumns() {
    std::vector<int> rowsToRemove = findEmptyRows();
    std::vector<int> colsToRemove = findEmptyCols();

    // Remove marked rows using index-based removal
    std::vector<std::vector<int>> newData;
    for (size_t i = 0; i < data.size(); ++i) {
        if (rowsToRemove[i] == 0) {  // Keep this row
            newData.push_back(data[i]);
        }
    }
    data = std::move(newData);

    // Remove marked columns
    for (auto& row : data) {
        std::vector<int> newRow;
        for (size_t x = 0; x < row.size(); ++x) {
            if (colsToRemove[x] == 0) {  // Keep this column
                newRow.push_back(row[x]);
            }
        }
        row = std::move(newRow);
    }

    // Update dimensions
    intHeight = data.size();
    intWidth = data.empty() ? 0 : data[0].size();
    DEBUG_PRINTF("intHeight: %d, intWidth: %d\n", intHeight, intWidth);
}
