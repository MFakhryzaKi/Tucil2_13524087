#include "solver.hpp"

vector<point> vertex_hasil;
vector<face> face_hasil;
int freq[25];

void createCube (point A, point B, point C, point D,
                point E, point F, point G, point H) 
{
    int curIdx = vertex_hasil.size();
    vertex_hasil.push_back(A);
    vertex_hasil.push_back(B);
    vertex_hasil.push_back(C);
    vertex_hasil.push_back(D);
    vertex_hasil.push_back(E);
    vertex_hasil.push_back(F);
    vertex_hasil.push_back(G);
    vertex_hasil.push_back(H);

    face_hasil.push_back({curIdx + 1, curIdx + 2, curIdx + 3}); // ABC
    face_hasil.push_back({curIdx + 1, curIdx + 3, curIdx + 4}); // ACD
    face_hasil.push_back({curIdx + 1, curIdx + 5, curIdx + 6}); // AEF
    face_hasil.push_back({curIdx + 1, curIdx + 6, curIdx + 2}); // AFB
    face_hasil.push_back({curIdx + 1, curIdx + 5, curIdx + 8}); // AEH
    face_hasil.push_back({curIdx + 1, curIdx + 8, curIdx + 4}); // AHD
    face_hasil.push_back({curIdx + 5, curIdx + 6, curIdx + 7}); // EFG
    face_hasil.push_back({curIdx + 5, curIdx + 7, curIdx + 8}); // EGH
    face_hasil.push_back({curIdx + 2, curIdx + 6, curIdx + 7}); // BFG
    face_hasil.push_back({curIdx + 2, curIdx + 7, curIdx + 3}); // BGC
    face_hasil.push_back({curIdx + 4, curIdx + 8, curIdx + 7}); // DHG
    face_hasil.push_back({curIdx + 4, curIdx + 7, curIdx + 3}); // DGC

}

bool isIntersect (point A, point G)
{
    for (point p : vertex_list) {
        if (p.x >= A.x && p.y >= A.y && p.z >= A.z &&
            p.x <= G.x && p.y <= G.y && p.z <= G.z) 
        {
            return true;
        }
    }

    for (face f : face_list) {
        if (intersectFaceKubus (vertex_list[f.v1-1], vertex_list[f.v2-1], vertex_list[f.v3-1],
            A, G))
        {
            return true;
        }
    }

    return false;
}

void solve (point A, point B, point C, point D,
            point E, point F, point G, point H,
            int depth)
{
    // itung jumlah octree pada depth
    if (isIntersect (A, G)) {
        freq[depth]++;
    }
    else {
        return;
    }

    if (depth == 1) {
        if (isIntersect(A, G)) {
            createCube (A, B, C, D, E, F, G, H);
        }
        else {
            // gausa ngapa2in
        }
    }
    else {
        // bagi 8 terus solve masing2 dengan depth = depth-1
        
        // tengah2 antar titik sudut tetangga
        point AB = midPoints(A, B);
        point BC = midPoints(B, C);
        point CD = midPoints(C, D);
        point DA = midPoints(D, A);
        point EF = midPoints(E, F);
        point FG = midPoints(F, G);
        point GH = midPoints(G, H);
        point HE = midPoints(H, E);
        point AE = midPoints(A, E);
        point BF = midPoints(B, F);
        point CG = midPoints(C, G);
        point DH = midPoints(D, H);

        // tengah2 di permukaan tiap sisi kubus
        point bawah = midPoints(AB, CD);
        point atas = midPoints(EF, GH);
        point kiri = midPoints(AB, EF);
        point kanan = midPoints(CD, GH);
        point belakang = midPoints(DA, HE);
        point depan = midPoints(BC, FG);

        // tengah2 kubus
        point tengah = midPoints(bawah, atas);

        // solve jadi 8
        solve (A, AB, bawah, DA, AE, kiri, tengah, belakang, depth - 1);
        solve (AB, B, BC, bawah, kiri, BF, depan, tengah, depth - 1);
        solve (bawah, BC, C, CD, tengah, depan, CG, kanan, depth - 1);
        solve (DA, bawah, CD, D, belakang, tengah, kanan, DH, depth - 1);
        solve (AE, kiri, tengah, belakang, E, EF, atas, HE, depth - 1);
        solve (kiri, BF, depan, tengah, EF, F, FG, atas, depth - 1);
        solve (tengah, depan, CG, kanan, atas, FG, G, GH, depth - 1);
        solve (belakang, tengah, kanan, DH, HE, atas, GH, H, depth - 1);
    }
}

void startSolve ()
{
    int max_size = 0;
    for (point p : vertex_list) {
        if (abs(p.x) > max_size) {
            max_size = fabs(p.x);
        }
        if (abs(p.y) > max_size) {
            max_size = fabs(p.y);
        }
        if (abs(p.z) > max_size) {
            max_size = fabs(p.z);
        }
    }

    double use_size = 0;
    while (max_size > 0) {
        use_size++;
        max_size /= 2;
    }

    use_size = pow (2, use_size);

    solve ({-use_size, -use_size, -use_size}, {-use_size, -use_size, use_size}, 
            {use_size, -use_size, use_size}, {use_size, -use_size, -use_size}, 
            {-use_size, use_size, -use_size}, {-use_size, use_size, use_size}, 
            {use_size, use_size, use_size}, {use_size, use_size, -use_size}, OCTREE_DEPTH);
    cout << "Voxel berhasil dibuat!\n";
}