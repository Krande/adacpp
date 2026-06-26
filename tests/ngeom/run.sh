#!/usr/bin/env bash
# Build + run the standalone NGEOM unit tests (no OCC / nanobind needed).
# libtess2 is C: compile its sources with gcc, link the C++ tests with g++.
set -euo pipefail
cd "$(dirname "$0")/../.."

INC="-I src/geom/neutral -I third_party/libtess2/Include"
TESS_SRC=(third_party/libtess2/Source/*.c)

obj=$(mktemp -d)
# -DNDEBUG: drop libtess2's debug asserts; real sweep errors fail soft via setjmp/longjmp
# (tessTesselate returns 0), which the tessellator handles with a shrunk-hole retry.
gcc -c -O2 -DNDEBUG -I third_party/libtess2/Include "${TESS_SRC[@]}"
mv ./*.o "$obj"/

# header-only suites
for t in test_analytic test_bspline test_decode; do
    g++ -std=c++20 -O2 -Wall $INC "tests/ngeom/$t.cpp" -o "$obj/$t"
    "$obj/$t"
done
# STEP Part-21 tokenizer — header-only, no OCC/libtess2
for t in test_step_part21; do
    g++ -std=c++20 -O2 -Wall -I src/cadit/step "tests/step/$t.cpp" -o "$obj/$t"
    "$obj/$t"
done

# tessellation-linked suites: libtess2 objects + the tessellator + the boolean stub
# (ngeom_tessellate.cpp references mesh_boolean; ngeom_boolean.cpp without ADACPP_HAVE_MANIFOLD
# compiles the no-op stub, so these link OCC/Manifold-free).
TESS_LINK="src/geom/neutral/ngeom_tessellate.cpp src/geom/neutral/ngeom_boolean.cpp"
g++ -std=c++20 -O2 -Wall $INC tests/ngeom/test_tessellate.cpp $TESS_LINK "$obj"/*.o -o "$obj/test_tessellate"
"$obj/test_tessellate"
g++ -std=c++20 -O2 -Wall $INC -I src/cadit/step tests/step/test_step_reader.cpp $TESS_LINK "$obj"/*.o \
    -o "$obj/test_step_reader"
"$obj/test_step_reader"

echo "ngeom: all suites passed"
