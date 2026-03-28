#include "utils.hpp"

// ifstream fileStream;
int OCTREE_DEPTH;
string outputFilePath;
string inputFileName;
vector<point> vertex_list;
vector<face> face_list;

point midPoints (point A, point B) {
    return {(A.x + B.x)/2, (A.y + B.y)/2, (A.z + B.z)/2};
}

point cross (point A, point B) {
    return {A.y*B.z - A.z*B.y, A.z*B.x - A.x*B.z, A.x*B.y - A.y*B.x};
}

double dot (point A, point B) {
    return A.x*B.x + A.y*B.y + A.z*B.z;
}

bool overlap (pair<double, double> MinMaxSegitiga, pair<double, double> MinMaxKubus) {
    return !(MinMaxSegitiga.second < MinMaxKubus.first || MinMaxKubus.second < MinMaxSegitiga.first);
}

bool isAxisNol (point axis) {
    const double toleransi = 1e-9;
    return fabs(axis.x) < toleransi && fabs(axis.y) < toleransi && fabs(axis.z) < toleransi;
}

pair<double, double> projectSegitiga (point X, point Y, point Z, point axis) {
    double p1 = dot (X, axis);
    double p2 = dot (Y, axis);
    double p3 = dot (Z, axis);

    return {min(p1, min(p2, p3)), max(p1, max(p2, p3))};
}

pair<double, double> projectKubus (point A, point G, point axis) {
    point center = {
        (A.x + G.x) / 2,
        (A.y + G.y) / 2,
        (A.z + G.z) / 2
    };

    point half = {
        (G.x - A.x) / 2,
        (G.y - A.y) / 2,
        (G.z - A.z) / 2
    };

    double c = dot(center, axis);

    double r = half.x * fabs(axis.x) + half.y * fabs(axis.y) + half.z * fabs(axis.z);

    return {c - r, c + r};

}

bool intersectFaceKubus (point X, point Y, point Z, point A, point G) {
    // axis di x
    point axis = {1, 0, 0};
    pair<double, double> MinMaxSegitiga = projectSegitiga(X, Y, Z, axis);
    pair<double, double> MinMaxKubus = projectKubus(A, G, axis);
    if (!overlap(MinMaxSegitiga, MinMaxKubus)) return false;

    // axis di y
    axis = {0, 1, 0};
    MinMaxSegitiga = projectSegitiga(X, Y, Z, axis);
    MinMaxKubus = projectKubus(A, G, axis);
    if (!overlap(MinMaxSegitiga, MinMaxKubus)) return false;

    // axis di z
    axis = {0, 0, 1};
    MinMaxSegitiga = projectSegitiga(X, Y, Z, axis);
    MinMaxKubus = projectKubus(A, G, axis);
    if (!overlap(MinMaxSegitiga, MinMaxKubus)) return false;

    // axis di titik normal segitiga 
    axis = cross({Y.x - X.x, Y.y - X.y, Y.z - X.z}, {Z.x - X.x, Z.y - X.y, Z.z - X.z});
    if (!isAxisNol(axis)) {
        MinMaxSegitiga = projectSegitiga(X, Y, Z, axis);
        MinMaxKubus = projectKubus(A, G, axis);
        if (!overlap(MinMaxSegitiga, MinMaxKubus)) return false;
    }

    // axis di edge segitiga * sumbu x, y, z

    point XY = {Y.x - X.x, Y.y - X.y, Y.z - X.z};
    point YZ = {Z.x - Y.x, Z.y - Y.y, Z.z - Y.z};
    point ZX = {X.x - Z.x, X.y - Z.y, X.z - Z.z};

    // edge XY, sumbu x
    axis = cross (XY, {1, 0, 0});
    if (!isAxisNol(axis)) {
        MinMaxSegitiga = projectSegitiga(X, Y, Z, axis);
        MinMaxKubus = projectKubus(A, G, axis);
        if (!overlap(MinMaxSegitiga, MinMaxKubus)) return false;
    }

    // edge XY, sumbu y
    axis = cross (XY, {0, 1, 0});
    if (!isAxisNol(axis)) {
        MinMaxSegitiga = projectSegitiga(X, Y, Z, axis);
        MinMaxKubus = projectKubus(A, G, axis);
        if (!overlap(MinMaxSegitiga, MinMaxKubus)) return false;
    }

    // edge XY, sumbu z
    axis = cross (XY, {0, 0, 1});
    if (!isAxisNol(axis)) {
        MinMaxSegitiga = projectSegitiga(X, Y, Z, axis);
        MinMaxKubus = projectKubus(A, G, axis);
        if (!overlap(MinMaxSegitiga, MinMaxKubus)) return false;
    }
    
    // edge YZ, sumbu x
    axis = cross (YZ, {1, 0, 0});
    if (!isAxisNol(axis)) {
        MinMaxSegitiga = projectSegitiga(X, Y, Z, axis);
        MinMaxKubus = projectKubus(A, G, axis);
        if (!overlap(MinMaxSegitiga, MinMaxKubus)) return false;
    }
    
    // edge YZ, sumbu y
    axis = cross (YZ, {0, 1, 0});
    if (!isAxisNol(axis)) {
        MinMaxSegitiga = projectSegitiga(X, Y, Z, axis);
        MinMaxKubus = projectKubus(A, G, axis);
        if (!overlap(MinMaxSegitiga, MinMaxKubus)) return false;
    }
    
    // edge YZ, sumbu z
    axis = cross (YZ, {0, 0, 1});
    if (!isAxisNol(axis)) {
        MinMaxSegitiga = projectSegitiga(X, Y, Z, axis);
        MinMaxKubus = projectKubus(A, G, axis);
        if (!overlap(MinMaxSegitiga, MinMaxKubus)) return false;
    }
    
    // edge ZX, sumbu x
    axis = cross (ZX, {1, 0, 0});
    if (!isAxisNol(axis)) {
        MinMaxSegitiga = projectSegitiga(X, Y, Z, axis);
        MinMaxKubus = projectKubus(A, G, axis);
        if (!overlap(MinMaxSegitiga, MinMaxKubus)) return false;
    }
    
    // edge ZX, sumbu y
    axis = cross (ZX, {0, 1, 0});
    if (!isAxisNol(axis)) {
        MinMaxSegitiga = projectSegitiga(X, Y, Z, axis);
        MinMaxKubus = projectKubus(A, G, axis);
        if (!overlap(MinMaxSegitiga, MinMaxKubus)) return false;
    }
    
    // edge ZX, sumbu z
    axis = cross (ZX, {0, 0, 1});
    if (!isAxisNol(axis)) {
        MinMaxSegitiga = projectSegitiga(X, Y, Z, axis);
        MinMaxKubus = projectKubus(A, G, axis);
        if (!overlap(MinMaxSegitiga, MinMaxKubus)) return false;
    }

    return true;

}


vector<string> parse(string s) {
    stringstream ss(s);
    string token;
    vector <string> res;

    while (ss >> token) {
        res.push_back(token);
    }

    return res;
}

pair<bool, int> verifyObjFile (ifstream &fileInput) {
    // string s;
    double pos[3];
    int vertex[3];
    int line = 1;
    while (!fileInput.eof()) {
        string s;
        vector<string> v;
        getline(fileInput, s);
        v = parse(s);

        if (v.size() == 0) {
            line++;
            continue;
        }
        else if (v.size() > 4) {
            cout << "Isi file tidak valid, terdapat baris yang tokennya terlalu banyak" << endl;
            return {false, line};
        }
        else if (v.size() < 4) {
            cout << "Isi file tidak valid, terdapat baris yang tokennya terlalu sedikit" << endl;
            return {false, line};
        }
        else {
            if (v[0] == "v") {
                for (int i = 1; i < 4; i++) {
                    try {
                        pos[i-1] = stod(v[i]);
                    }
                    catch (invalid_argument &e) {
                        cout << "Isi file tidak valid, terdapat baris yang bukan berisi angka" << endl;
                        return {false, line};
                    }
                    catch (out_of_range &e) {
                        cout << "Isi file tidak valid, terdapat angka yang terlalu besar" << endl;
                        return {false, line};
                    }
                }
                vertex_list.push_back({pos[0], pos[1], pos[2]});
            }
            else if (v[0] == "f") {
                for (int i = 1; i < 4; i++) {
                    try {
                        vertex[i-1] = stoi(v[i]);
                    }
                    catch (invalid_argument &e) {
                        cout << "Isi file tidak valid, terdapat baris yang bukan berisi angka" << endl;
                        return {false, line};
                    }
                    catch (out_of_range &e) {
                        cout << "Isi file tidak valid, terdapat angka yang terlalu besar" << endl;
                        return {false, line};
                    }

                    if (vertex[i-1] > vertex_list.size()) {
                        cout << "Isi file tidak valid, Face mengakses vertex yang belum terbentuk" << endl;
                        return {false, line};
                    } 
                }
                face_list.push_back({vertex[0], vertex[1], vertex[2]});
            }
            else {
                cout << "Program tidak menerima token pertama selain v atau f" << endl;
                return {false, line};
            }
        }
        line++;
    }

    return {true, line};
}

void readAndVerifyObjFile () {
    cout << "SELAMAT DATANG DI PROGRAM VOXELISASI POLYGON" << endl;
    while (true) {
        vertex_list.clear();
        face_list.clear();
        cout << "Masukkan nama file .obj: ";
        string s;
        getline(cin, s);

        ifstream stream1 ("test/input/" + s);
        ifstream stream2 ("test/input/" + s + ".obj");

        if (!stream1.is_open() && !stream2.is_open()) {
            cout << "File .obj dengan nama " << s << " tidak ditemukan!\n";
            continue;
        }
        else {
            ifstream fileInput;
            if (stream1.is_open()) {
                fileInput.open("test/input/" + s);
            }
            else {
                fileInput.open("test/input/" + s + ".obj");
            }

            pair<bool, int> valid = verifyObjFile(fileInput);
            if (!valid.first) {
                cout << "Error line: " << valid.second << endl; 
                continue;
            }
            else {
                int depth;
                while (true) {
                    cout << "Masukkan kedalaman octree : ";

                    cin >> depth;

                    if (cin.fail()) {
                        cout << "Masukan harus berupa integer!" << endl;

                        cin.clear();
                        cin.ignore(10000, '\n');
                        continue;
                    }
                    else if (depth > 20) {
                        cout << "Masukan tidak boleh melebihi 20!" << endl;

                        cin.clear();
                        cin.ignore(10000, '\n');
                        continue;
                    }

                    OCTREE_DEPTH = depth;
                    inputFileName = s;
                    cout << "PROCESSING..." << endl;
                    cout << "Filename : " << s << ".obj" << endl;
                    cout << "Max Depth : " << depth << endl;
                    break;
                }
                break;
            }
        }
    }

}

void saveObjFile () {
    outputFilePath = "test/output/" + inputFileName + "-voxelized.obj";
    ofstream fileOutput (outputFilePath);

    for (point v : vertex_hasil) {
        fileOutput << "v " << v.x << ' ' << v.y << ' ' << v.z << endl;
    }

    for (face f : face_hasil) {
        fileOutput << "f " << f.v1 << ' ' << f.v2 << ' ' << f.v3 << endl;
    }

}

void printStatistics () {
    cout << "==================== VOXELIZATION STATISTICS ====================" << endl;
    cout << "Voxel Count                : " << freq[1] << endl;
    cout << "Vertex Count               : " << vertex_hasil.size() << endl;
    cout << "Face Count                 : " << face_hasil.size() << endl;

    for (int i = 1; i <= OCTREE_DEPTH; i++) {
    cout << "Octree Built (depth = " << i << ")   : " << freq[i] << endl;
    }

    for (int i = 1; i <= OCTREE_DEPTH; i++) {
    cout << "Octree Skipped (depth = " << i << ") : " << int(pow(8, OCTREE_DEPTH - i)) - freq[i] << endl;
    }

    cout << "Max Octree Depth           : " << OCTREE_DEPTH << endl;
    cout << "Duration                   : " << ms << " ms" << endl;
    cout << "Successfully saved result at " + outputFilePath << endl;

    cout << "=================================================================" << endl;
}