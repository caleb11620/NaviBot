#include <iostream>

using namespace std;
const int rows = 10;
const int col = 11;

int maze[rows][col] =  {{1,1,1,1,1,1,1,1,1,1,1},
                        {1,1,1,1,1,2,1,1,1,1,1},
                        {1,0,0,0,0,0,1,0,0,0,1},
                        {1,1,1,1,1,0,1,1,1,0,1},
                        {1,0,0,0,0,0,0,0,0,0,1},
                        {1,1,1,0,1,1,1,0,1,0,1},
                        {1,0,0,0,0,1,0,0,1,0,1},
                        {1,0,1,1,1,1,0,1,1,0,1},
                        {1,0,0,0,0,1,1,1,0,0,1},
                        {1,1,1,1,1,1,1,1,1,1,1}
                        }
                        ;

int x = 8;
int y = 4;
int h = 0;
int a, b, c;


void lookaround() {
    switch(h) {
        case 0:
            a = maze[x-1][y];
            b = maze[x][y+1];
            c = maze[x+1][y];
            break;

        case 1:
            a = maze[x][y-1];
            b = maze[x-1][y];
            c = maze[x][y+1];
            break;

        case 2:
            a = maze[x+1][y];
            b = maze[x][y-1];
            c = maze[x-1][y];
            break;

        case 3:
            a = maze[x][y+1];
            b = maze[x+1][y];
            c = maze[x][y-1];
            break;
    }
}

void move() {
    switch (h) {
        case 0:
            x = x;
            y = y + 1;
            break;
        case 1:
            x = x - 1;
            y = y;
            break;
        case 2:
            x = x;
            y = y - 1;
            break;
        case 3:
            x = x + 1;
            y = y;
    }
}

void lefthandsolve() {
    if (a != 1) {
        //left turn
        if (h == 3) {
            h = 0;
        } else { h = h + 1;}
        move();
    } else if (b == 1) {
        if (h == 0) {
            h = 3;
        } else { h = h - 1;}
    } else {
        move();
    }
}

int main() {
    int i = 0;
    while (i < 35 && maze[x][y] != 2) {
        cout << x << y << h << " ";
        lookaround();
        cout << a << b << c << " ";
        lefthandsolve();
        cout << x << y << h << endl;
        i = i + 1;
    }
    cout << endl << i;
}
