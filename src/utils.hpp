#pragma once
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <math.h>
#include <chrono>

using namespace std;

struct point
{
    double x;
    double y;
    double z;
};

struct face
{
    int v1;
    int v2;
    int v3;
};


// variabel global
extern int OCTREE_DEPTH;
extern int ms;
extern string outputFilePath;
extern string inputFileName;

extern int freq[25];

extern vector<point> vertex_list;
extern vector<face> face_list;

extern vector<point> vertex_hasil;
extern vector<face> face_hasil;

// utils buat manipulasi point
point midPoints (point A, point B);
point cross (point A, point B);
double dot (point A, point B);
bool overlap (double min1, double max1, double min2, double max2);
bool isAxisNol (point axis);
pair<double, double> projectSegitiga (point X, point Y, point Z, point axis);
pair<double, double> projectKubus (point A, point G, point axis);
bool intersectFaceKubus (point X, point Y, point Z, point A, point G);

// utils buat read/write file dan output
vector<string> parse(string s);
pair<bool, int> verifyObjFile (ifstream &fileInput);
void readAndVerifyObjFile ();
void saveObjFile ();
void printStatistics ();