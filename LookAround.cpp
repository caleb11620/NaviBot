#include <iostream>
using namespace std;

int h = 0;
int a, b, c;
int x = 1;
int y = 1;

int maze[6][6] =   {{1,1,1,1,1,1},
                    {1,0,0,0,0,1},
                    {1,1,1,1,0,1},
                    {1,1,1,1,0,1},
                    {1,0,0,0,0,1},
                    {1,1,1,1,1,1}};


void lookaround(int x, int y) {
    switch (h) {
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
        default:
            break;

    }
    // cout << a << b << c << h;
}

void movement() {
    if (a == 0) {
        if (h == 3) {
            h = 0; 
        } else { h = h + 1; }
    } else if (b == 1) {
        if (h == 0) {
            h = 3;
        } else {h = h - 1; }
    } else {
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
                break;
            default:
                break;
        }
    }
    cout << x << y;
}

int main() {
    for (int i = 0; i < 12; i ++) {
        lookaround(x,y);
        movement();
        cout << endl;
    }
}