// Offline golden-vector generator against vendored ruckig headers (tag v0.17.3).
//
// Build (from repo root, MSVC or g++):
//   cl /EHsc /I ruckig/include /std:c++17 RuckigJava/tools/generate_golden_cpp.cpp /Fe:RuckigJava/tools/generate_golden.exe
//   g++ -std=c++17 -I ruckig/include RuckigJava/tools/generate_golden_cpp.cpp -o RuckigJava/tools/generate_golden
//
// Note: most Ruckig sources are header-only except brake + step translation units.
// Link those .cpp files when expanding this harness, or use the Python package when available.
//
// This skeleton emits a single rest-to-rest case in the same JSON schema as generate_golden.py.

#include <cstdio>
#include <iostream>

// Full generation requires compiling position_third_step*.cpp, brake.cpp, etc.
// Prefer: pip install ruckig==0.17.3 && python generate_golden.py

int main() {
    std::cerr << "Skeleton only. Build against full ruckig object files or use generate_golden.py.\n";
    std::cout << "{\n"
                 "  \"version\": \"0.17.3\",\n"
                 "  \"suite\": \"cpp_skeleton\",\n"
                 "  \"cases\": []\n"
                 "}\n";
    return 0;
}
