#include <iostream>
using namespace std;

int a, b, c;
void leftwallfollow() {
    if (a == 0) {
        cout << "Left Turn";
    } else if (b == 1) {
        cout << "Right Turn";
    } else {
        cout << "Moving Forward";
    }
}

int main() {
    a = 1;
    b = 1;
    c = 1;
    leftwallfollow();
}