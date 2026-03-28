#pragma once
#include "utils.hpp"

void createCube (point A, point B, point C, point D,
                point E, point F, point G, point H);

bool isIntersect (point A, point G);

void solve (point A, point B, point C, point D,
            point E, point F, point G, point H,
            int depth);

void startSolve ();