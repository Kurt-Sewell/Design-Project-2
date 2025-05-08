#include <stdio.h>
#include <iostream>
#include <windows.h>
#include <string>
using namespace std;

// double N = 0;
// forAll(internalIDs_, i)
// {
//     N++;
//     double percent = 100 * N / internalIDs_.size();
//     // Info<< "\rProgress: " << percent << "%" << endl;
//     printf("\r[%6.4f%%]", percent);
// }

int main()
{
    string stall;
    for (int i = 0; i <= 100; ++i)
    {
        std::string line1 = "Progress: " + std::to_string(i) + "%";
        std::string line2 = "Status: Processing...";
        std::cout << "\r" << line1 << std::endl;
        std::cout << "\r" << line2 << std::endl;
        Sleep(.5);
    }
    cin >> stall;
    std::cout << std::endl;
    return 0;
}