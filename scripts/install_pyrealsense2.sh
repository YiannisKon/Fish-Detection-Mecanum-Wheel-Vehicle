#!/bin/zsh
# Install the freshly-built pyrealsense2 into miniconda's site-packages.
# The .so uses @rpath for its sibling dylibs, so we co-locate them and add an
# @loader_path rpath, then ad-hoc re-sign (Apple Silicon refuses to load a
# Mach-O whose signature was invalidated by install_name_tool).
set -u
SRC=/tmp/librealsense-build/src/build/Release
DEST=/opt/miniconda3/lib/python3.12/site-packages

FILES=(
  pyrealsense2.cpython-312-darwin.so
  pyrealsense2.2.58.cpython-312-darwin.so
  pyrealsense2.2.58.3.cpython-312-darwin.so
  librealsense2.dylib
  librealsense2.2.58.dylib
  librealsense2.2.58.3.dylib
)

echo "==> Copying binding + native libs to site-packages"
for f in $FILES; do
  if [ -e "$SRC/$f" ]; then
    cp -a "$SRC/$f" "$DEST/$f"
    echo "   $f"
  else
    echo "   MISSING $f"
  fi
done

echo "==> Fixing rpath (@loader_path) and re-signing"
for f in $FILES; do
  [ -e "$DEST/$f" ] || continue
  # add rpath; ignore 'already present' errors
  install_name_tool -add_rpath @loader_path "$DEST/$f" 2>/dev/null
  codesign --remove-signature "$DEST/$f" 2>/dev/null
  codesign -s - -f "$DEST/$f" 2>/dev/null && echo "   signed $f"
done

echo "==> Import test"
/opt/miniconda3/bin/python3 - <<'PY'
import pyrealsense2 as rs
print("pyrealsense2 imported OK")
try:
    print("version:", rs.__version__)
except Exception:
    print("version attr: (n/a)")
# creating a context enumerates USB but does NOT open the camera stream
ctx = rs.context()
print("context created; devices seen:", len(ctx.query_devices()))
PY
