# CppRobotics

![Linux-CI](https://github.com/harderthan/CppRobotics/actions/workflows/linux-ci.yml/badge.svg)

### Download

```bash
mkdir -p ${YOUR_PATH}/src
git clone https://github.com/harderthan/CppRobotics.git ${YOUR_PATH}/src/

```

### Build with colcon

```bash
cd ${YOUR_PATH}
colcon build
```

### Run

```bash
source ${YOUR_PATH}/install/setup.bash
ros2 run grid_based_search a_star_node 
# or
ros2 launch grid_based_search a_star_node.launch.py
```

## Troubleshooting

# Demos
