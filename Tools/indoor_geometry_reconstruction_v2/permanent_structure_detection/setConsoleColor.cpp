//
// Created by NikoohematS on 11/17/2016.
//

#include <iostream>
#ifdef _WIN32
#include <windows.h>
#endif


using namespace std;
HANDLE hCon;

enum Color { DARKBLUE = 1, DARKGREEN, DARKTEAL, DARKRED, DARKPINK, DARKYELLOW, GRAY ,
    DARKGRAY, BLUE, GREEN, TEAL, RED, PINK, YELLOW, WHITE };

void SetColor(Color c){
    if(hCon == NULL)
        hCon = GetStdHandle(STD_OUTPUT_HANDLE);
    SetConsoleTextAttribute(hCon, c);
}

/*
int main(){
    SetColor(RED);
    cout << "InfernoDevelopment.com\n";
    SetColor(DARKRED);
    cout << "Join our forums at www.infernodevelopment.com/forum\n";
    cin.get();
    return 0;
}*/
