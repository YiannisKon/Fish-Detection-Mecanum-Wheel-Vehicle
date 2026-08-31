#!/bin/zsh
# Build pyrealsense2 from source for Apple Silicon (arm64) macOS + miniconda Python 3.12.
# No prebuilt wheel exists for osx-arm64, so we compile the SDK's Python bindings.
set -u
PYEXE=/opt/miniconda3/bin/python3
BUILD_ROOT=/tmp/librealsense-build
SRC=$BUILD_ROOT/src

echo "==> Cleaning $BUILD_ROOT"
rm -rf "$BUILD_ROOT"
mkdir -p "$BUILD_ROOT"
cd "$BUILD_ROOT" || exit 1

echo "==> Cloning librealsense (with submodules for pybind11) ..."
CLONED=0
for REPO in "https://github.com/realsenseai/librealsense.git" "https://github.com/IntelRealSense/librealsense.git"; do
  for TAG in "v2.58.3" "2.58.3"; do
    echo "   trying $REPO @ $TAG"
    if git clone --depth 1 --recurse-submodules --shallow-submodules --branch "$TAG" "$REPO" src 2>/dev/null; then
      CLONED=1; echo "   cloned $REPO @ $TAG"; break
    fi
  done
  [ $CLONED -eq 1 ] && break
done
if [ $CLONED -eq 0 ]; then
  echo "   tag clone failed; cloning default branch of IntelRealSense"
  git clone --depth 1 --recurse-submodules --shallow-submodules https://github.com/IntelRealSense/librealsense.git src || { echo "CLONE FAILED"; exit 1; }
fi

cd "$SRC" || exit 1
mkdir -p build && cd build || exit 1

echo "==> Configuring with CMake ..."
/opt/homebrew/bin/cmake .. \
  -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_PYTHON_BINDINGS=ON \
  -DPYTHON_EXECUTABLE="$PYEXE" \
  -DPython_EXECUTABLE="$PYEXE" \
  -DPython3_EXECUTABLE="$PYEXE" \
  -DBUILD_EXAMPLES=OFF \
  -DBUILD_GRAPHICAL_EXAMPLES=OFF \
  -DBUILD_TOOLS=OFF \
  -DBUILD_UNIT_TESTS=OFF \
  -DCHECK_FOR_UPDATES=OFF \
  -DFORCE_RSUSB_BACKEND=ON \
  -DCMAKE_POLICY_VERSION_MINIMUM=3.5 2>&1 | tail -40
CFG=${pipestatus[1]}
if [ "${CFG:-1}" -ne 0 ]; then echo "CMAKE CONFIGURE FAILED (status $CFG)"; exit 1; fi

echo "==> Building (this takes a while) ..."
/opt/homebrew/bin/cmake --build . --config Release -j4 2>&1 | tail -60
BLD=${pipestatus[1]}
if [ "${BLD:-1}" -ne 0 ]; then echo "BUILD FAILED (status $BLD)"; exit 1; fi

echo "==> Build products:"
find "$SRC/build" -name "pyrealsense2*.so" -o -name "librealsense2*.dylib" 2>/dev/null
echo "BUILD DONE OK"
