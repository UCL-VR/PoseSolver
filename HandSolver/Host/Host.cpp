
/*
* This executable is a convenience tool for invoking the solver library for
* development and debugging.
* It does not act as a tracker itself - to perform tracking load the library
* in a tracking system inside Unity.
*/

#include <iostream>
#include <Windows.h>

typedef void (WINAPI* Initialise)(void);

int main()
{
    std::cout << "Hello World!\n";

    HMODULE handSolver = LoadLibraryA("HandSolver.dll");

    if (!handSolver) {
        std::cout << GetLastError();
    }

    auto init = (Initialise)GetProcAddress(handSolver, "initialise");

    init();
}

// Run program: Ctrl + F5 or Debug > Start Without Debugging menu
// Debug program: F5 or Debug > Start Debugging menu

// Tips for Getting Started: 
//   1. Use the Solution Explorer window to add/manage files
//   2. Use the Team Explorer window to connect to source control
//   3. Use the Output window to see build output and other messages
//   4. Use the Error List window to view errors
//   5. Go to Project > Add New Item to create new code files, or Project > Add Existing Item to add existing code files to the project
//   6. In the future, to open this project again, go to File > Open > Project and select the .sln file
