#include <iostream>

using namespace std;

const int rows = 10;
const int col = 11;

int x = 8;
int y = 4;
int h = 2;
int a, b, c;

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

//The Pledge algorithm navigates around the object until the initial heading is the final.
//To test this, we will implement a loop to be stuck and the general direction of the goal.
//So with the left follow algorithm, this bot should get stuck in the loop.

//So it's stuck in a loop. Now let us implement some factor that stops following the
//wall as long as the bearing is correct and the path is clear. Let us first develop
//a function that sets the current heading. 

int exit_x = 1;
int exit_y = 5;

int exit_h;

int hx, hy, dx, dy;

void findh() {
    dx = exit_x - x;
    dy = exit_y - y; 

    if (dx != 0) {
        hx = (exit_x - x)/abs(exit_x - x);
    } else {
        hx = 0;
    }
    if (dy != 0) {
        hy = (exit_y - y)/abs(exit_y - y);
    } else {
        hy = 0;
    }

    if (dx > 0) {
        if (hx == -1) {
            exit_h = 1;
        } else if (hx == 1) {
            exit_h = 3;
        } else if (hx == 0) {
            if (hy == -1) {
                exit_h = 2;
            } else if(hy == 1) {
                exit_h = 0;
            }
        }
    }

    if (dx = 0) {
        if (hy == -1) {
            exit_h = 2;
        } else if (hy == 1) {
            exit_h = 0;
        } else if (hy == 0) {
            if (hx == -1) {
                exit_h = 1;
            } else if(hx == 1) {
                exit_h = 3;
            }
        }
    }
}


//So now we have a function to determine the h and we are initially prioritize the x,
//this may screw us but we shall see. Now let us do a function that will only wall follow
//an object until the exit h is achieved and there is no wall in front.

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
            
        default:
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
        default:
            break;
    }
}

void lefthandsolve() {
    if (a == 0) {
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

void pledgesolve() {
    if (h == exit_h && b != 1) {
        //while we have the correct heading and the path is clear
        //the bot will move forward
        move();
    } else {
        lefthandsolve();
    }
    //So right here we need something. So far, we have an algorithm
    //that will travel to the path with x prioritized although we get stuck. 
    //An algorithm that will 


}

string direction(int w) {
    string s;
    switch(w) {
        case 0:
            s = "Right";
            break;
        case 1:
            s = "Up";
            break;
        case 2:
            s = "Left";
            break;
        case 3:
            s = "Down";
            break;
        default:
            break;
    }
    return s;
}

int main() {
    int i = 0;
    while (i < 50 && maze[x][y] != 2) {
        findh();
        cout << x << y << direction(h) << direction(exit_h) << " ";
        lookaround();
        cout << a << b << c << " ";
        pledgesolve();
        cout << x << y << direction(h) << direction(exit_h) << endl;
        i = i + 1; 
    }
    cout << endl << i;
}
