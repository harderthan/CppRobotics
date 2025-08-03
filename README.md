# CppRobotics

![Linux-CI](https://github.com/harderthan/CppRobotics/actions/workflows/linux-ci.yml/badge.svg)


# Installation

## Build Instructions

### 1. Clone the repository

```bash
git clone https://github.com/harderthan/CppRobotics.git
cd CppRobotics
```

### 2. Install Python dependencies

```bash
# Install Python dependencies.
pip install --upgrade pip
pip install -r ./requirements.txt

# Install C++ packages with vcpkg
cd third_party/vcpkg
./bootstrap-vcpkg.sh
./vcpkg install implot
```

### 3. Configure and Build

```bash
cmake -B build -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_TOOLCHAIN_FILE=third_party/vcpkg/scripts/buildsystems/vcpkg.cmake
cmake --build build 
```

## Troubleshooting

- If you continue to experience issues, please create an issue on the GitHub repository:
  - Visit: https://github.com/harderthan/CppRobotics/issues
  - Provide detailed information about your environment and the error you're encountering
  - Include any relevant error messages or build logs


# Demos

## Path Planning

### A-Star Planning

![a_star](https://github.com/harderthan/CppRobotics/raw/main/doc/img/path_planning/a_star.gif)


### Path Tracking

![pure_pursuit](https://github.com/harderthan/CppRobotics/raw/main/doc/img/path_tracking/pure_pursuit.gif)
