#include "Bitmap.h"

// constructor
Bitmap::Bitmap() : origWidth(0), origHeight(0) {}

int Bitmap::getPixel(int x, int y)
{
    return data[y][x] ? 1 : 0;
}

// Read from SD card
// This will likely need to be changed depending on exact format
// of the bitmap file.
bool Bitmap::read(File &file)
{
  if (!file) {
    Serial.println(F("Error with maze bitmap: empty"));
    return false;
  } 

  for (int y = 0; y < MAX_HEIGHT; ++y)
  {
    for (int x = 0; x < MAX_WIDTH; ++x)
    {
      while (file.available() && !isdigit(file.peek())) {
        file.read(); // Skip non-digit characters
      }
      
      if (!file.available()) {
        Serial.println(F("Unexpected end of file"));
        return false;
      }
      
      char c = file.read();
      data[y][x] = (c == '1');
    }
  }
  return true;
}

// Write to SD card
bool Bitmap::write(File &file)
{
  if (!file) {
    Serial.println(F("Error opening file for writing"));
    return false;
  }

  for (int y = 0; y < intHeight; ++y)
  {
    for (int x = 0; x < intWidth; ++x)
    {
      if (!file.write(data[y][x] ? '1' : '0')) {
        Serial.println(F("Write failed"));
        return false;
      }
    }
    if (!file.println()) {
      Serial.println(F("Failed to write newline"));
      return false;
    }
  }
  return true;
}

void Bitmap::invertPixel(int x, int y)
{
    if (x >= 0 && x < MAX_WIDTH && y >= 0 && y < MAX_HEIGHT)
    {
        data[y][x] = !data[y][x];
    }
}

std::vector<int> Bitmap::findEmptyRows()
{
    std::vector<int> emptyRows(MAX_HEIGHT, 0);
    for (int y = 0; y < MAX_HEIGHT; ++y)
    {
        for (int x = 0; x < MAX_WIDTH; ++x)
        {
            if (data[y][x])
            {
                emptyRows[y] = 1;
                break;
            }
        }
    }
    return emptyRows;
} 

std::vector<int> Bitmap::findEmptyCols()
{
    std::vector<int> emptyCols(MAX_WIDTH, 0);
    for (int x = 0; x < MAX_WIDTH; ++x)
    {
        for (int y = 0; y < MAX_HEIGHT; ++y)
        {
            if (data[y][x])
            {
                emptyCols[x] = 1;
                break;
            }
        }
    }
    return emptyCols;
}

// Removes any row or column that is entirely 0
void Bitmap::removeEmptyRowsAndColumns()
{
    std::vector<int> nonEmptyRows = findEmptyRows();
    std::vector<int> nonEmptyCols = findEmptyCols();

    // Remove empty rows
    data.erase(
        std::remove_if(data.begin(), data.end(),
            [&nonEmptyRows, this](const std::vector<bool>& row) {
                return nonEmptyRows[&row - &data[0]] == 0;
            }),
        data.end()
    );

    // Remove empty columns
    for (auto& row : data) {
        for (int x = intWidth - 1; x >= 0; --x) {
            if (nonEmptyCols[x] == 0) {
                row.erase(row.begin() + x);
            }
        }
    }

    // Update dimensions
    intHeight = data.size();
    intWidth = data[0].size();
}