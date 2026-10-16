# Official MuJoCo Simulate source snapshot

- Source: https://github.com/google-deepmind/mujoco
- Version: `3.4.0`
- Commit: `e55fff5dea6f1d5dd7963ca52eecc41d05ad0922`
- Files: the nine C++ sources/headers under `simulate/`, used to build the official viewer library. The standalone sample `main.cc` is not used.
- License: Apache-2.0, preserved in `LICENSE` and each file header.

The PNG dependency is the revision pinned by MuJoCo 3.4.0:

- Source: https://github.com/lvandeve/lodepng
- Commit: `b4ed2cd7ecf61d29076169b49199371456d4f90b`
- Files: `lodepng/lodepng.cpp`, `lodepng/lodepng.h`.
- License: zlib-style notice retained in both sources and `LICENSE.lodepng`.

All upstream source files are unmodified. `SHA256SUMS` records their contents.
The project's CMake builds this subset against the existing MuJoCo 3.4.0 library
and system GLFW/OpenGL; builds do not fetch dependencies from the network.

Project-specific loading, thread lifetime and passive-viewer synchronization
are implemented in `ros2/go2_control_cpp/src/simulation.cpp`.
