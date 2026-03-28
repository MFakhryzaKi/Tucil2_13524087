# Tucil2_13524087
13524087
Muhammad Fakhry Zaki

## Pendahuluan
Program ini adalah program untuk mem-voxelisasi sebuah file dengan ekstensi .obj. Program ini terbatas pada file .obj yang hanya terdiri dari faces (f) dan vertexes (v).

## Menjalankan Program
1. Pastikan anda berada di direktori utama (root), kemudian jalankan perintah ini pada terminal untuk melakukan kompilasi program
```bash
g++ -o .\bin\main .\src\main.cpp .\src\utils.cpp .\src\solver.cpp
```

2. Setelah itu, jalankan file executable yang sudah terkompilasi dengan perintah berikut
```bash
.\bin\main.exe
```

3. Setelah program dijalankan, program akan meminta user untuk meng-input nama file .obj yang akan di-voxelisasi beserta kedalaman maximun dari Octree yanga akn digunakan. Pastikan file .obj yang ingin digunakan berada pada folder input (dalam folder test) dan ukuran Octree yang diinput tidak melebihi 20

4. Setelah memasukkan file .obj dan kedalaman Octree yang diinginkan, program akan langsung melakukan voxelization pada file tersebut. Kemudian, secara otomatis juga hasil voxelization akan langsung tersimpan pada folder output, dalam folder test, dengan nama file [nama file input]-voxelized.obj