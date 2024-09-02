//Author: Caleb Solis
//Program: Mapping Algorithm
//Brief: This program will be accept five inputs including:
// Encoder Value, Left, Right, Center Sensor Values, Current Orientation, Enable
// The algorithm will use these values in the values of the grid.
// It will assume all cells are walls at first. It will start
// from the center of the maze and fill outward. This will allow it
// to solve mazes starting from inward. The first time a bot approaches a front wall,
// the bot will stop, measure distance and start mapping. This start distance and start point,
// will allow the bot to recalibrate on the next run. 
// Will create a 400x400 binary file.
// 0 represents wall
// 1 represents open cell



#include <cmath>
#include <iostream>
#include <fstream>
#include <sstream>

using namespace std;

fstream file;
float x, y, angle;

//Calculate file location of requested x,y
int calcLoc(int x, int y) {
    float loc = x + (401*y);
    return round(loc);
}

//Create or Zero Grid
void initializeGrid() {
    file.open("Map2.bin", ios::binary | ios::out | ios::in);
    if(file.is_open()) {
        for(int i = 0; i < 400; i++) {
            for(int j = 0; j < 400; j++) {
                file << 0;
            }
            file << endl;
        }
        file.seekp(calcLoc(199,199), ios::beg);
        file << 1;
        file.close();
    }
}

#define LEFT 1
#define CENTER 2
#define RIGHT 3

#define PI 3.14159265

void UpdateCells(int distance, float orientation) {
    float radians = (orientation / 180) * PI;
    for(int i = 0; i < distance; i++) {
        x = x + cos(radians);
        y = y + sin(radians);
        file.open("Map2.bin", ios::binary | ios::out | ios::in);
        if(file.is_open()) {
            file.seekp(calcLoc(x, y), ios::beg);
            file << 1;
            file.close();
        }
    }
}

void WallDistance(int wall_distance, int sensor) {
    int prev_x = x;
    int prev_y = y;
    switch(sensor) {
        case 1: 
        int left_angle;
        left_angle = angle - 90;
        UpdateCells(wall_distance, left_angle);
        break;

        case 2:
        UpdateCells(wall_distance, angle);
        break;

        case 3:
        int right_angle;
        right_angle = angle + 90;
        UpdateCells(wall_distance, right_angle);
        break;

        default:
        break;
    }
    x = prev_x;
    y = prev_y;
}

void MapUpdate(int distanceTraveled,int orientation, int left_sensor, int center_sensor, int right_sensor) {
            angle = orientation;
            UpdateCells(distanceTraveled, orientation);
            WallDistance(left_sensor, LEFT);
            WallDistance(center_sensor, CENTER);
            WallDistance(right_sensor, RIGHT);
}

void UserData() {
    int mapData[6];
    mapData[5] = 0;
    while (mapData[5] == 0) {
        string userArrayInput;
        int userInput;
        cout << "Data: ";
        cin >> userArrayInput;

        istringstream iss(userArrayInput);

        string item;
        int i = 0;

        while(getline(iss,item,',')) {
            mapData[i] = stoi(item, nullptr, 10);
            i = i + 1;
        }
        MapUpdate(mapData[0], mapData[1], mapData[2], mapData[3], mapData[4]);
    }

}

int main() {

    initializeGrid();

    //Starting point
    x = 199;
    y = 199;
    angle = 0;

    for(int i = 0; i < 10; i++) {
        MapUpdate(1,0,3,10-i,3);
    }
    for(int i = 0; i < 4; i++) {
        MapUpdate(1,0,15,4-i,3);
    }
    for(int i = 0; i < 15; i++) {
        MapUpdate(1,-90,4,15-i,1);
    }
    return 0;
}

