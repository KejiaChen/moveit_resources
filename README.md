MoveIt Resources
================

This repository includes various resources (URDFs, meshes, moveit_config packages) needed for MoveIt testing.

GitHub Actions: [![Formatting (pre-commit))](https://github.com/ros-planning/moveit_resources/actions/workflows/format.yml/badge.svg?branch=ros2)](https://github.com/ros-planning/moveit_resources/actions/workflows/format.yml?query=branch%3Aros2) [![Build and Test](https://github.com/ros-planning/moveit_resources/actions/workflows/industrial_ci_action.yml/badge.svg?branch=ros2)](https://github.com/ros-planning/moveit_resources/actions/workflows/industrial_ci_action.yml?query=branch%3Aros2)

## Included Robots

- PR2
- Fanuc M-10iA
- Franka Emika Panda

## Notes

The benchmarking resources have been moved to https://github.com/ros-planning/moveit_benchmark_resources.


## Merge into Default MoveIt2 Installation
Before you start, make sure the default moveit2 installation passes compiling without errors.

Check the current remote repository of ```moveit_resources```
```
cd ~/<your_moveit_workspace>/src/moveit_resources
git remote -v
```
By default, you should see
```
origin	https://github.com/moveit/moveit_resources (fetch)
origin	https://github.com/moveit/moveit_resources (push)
```
Change the repository to our fork:
```
git remote set-url origin https://github.com/KejiaChen/moveit_resources.git
```
Pull from ```ros2```branch
```
# confirm your current branch
git branch 
# if it is ros2, pull from the corresponding branch
git pull origin ros2
```
You should see modifications pulling from the fork branch.

After that, rebuild your workspace.