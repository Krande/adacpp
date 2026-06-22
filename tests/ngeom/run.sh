#!/usr/bin/env bash
# Build + run the standalone NGEOM unit tests (no OCC / nanobind needed).
# libtess2 is C: compile its sources with gcc, link the C++ tests with g++.
set -euo pipefail
cd "$(dirname "$0")/../.."

INC="-I src/geom/neutral -I third_party/libtess2/Include"
TESS_SRC=(third_party/libtess2/Source/*.c)

obj=$(mktemp -d)
gcc -c -O2 -I third_party/libtess2/Include "${TESS_SRC[@]}"
mv ./*.o "$obj"/

# header-only suites
for t in test_analytic test_bspline test_decode; do
    g++ -std=c++20 -O2 -Wall $INC "tests/ngeom/$t.cpp" -o "$obj/$t"
    "$obj/$t"
done
# tessellator suite (needs libtess2 objects + the .cpp)
g++ -std=c++20 -O2 -Wall $INC tests/ngeom/test_tessellate.cpp \
    src/geom/neutral/ngeom_tessellate.cpp "$obj"/*.o -o "$obj/test_tessellate"
"$obj/test_tessellate"

echo "ngeom: all suites passed"
