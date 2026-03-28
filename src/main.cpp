#include "utils.hpp"
#include "solver.hpp"

int ms;

int main () {
    readAndVerifyObjFile ();
    auto startTime = chrono::high_resolution_clock::now();
    startSolve ();
    auto endTime = chrono::high_resolution_clock::now();
    auto duration = chrono::duration_cast<chrono::milliseconds> (endTime - startTime);
    ms = duration.count();
    saveObjFile ();
    printStatistics ();
}

