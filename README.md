# Maze_project

the final project of eep520 2025 winter

## Overview

This project implements a robot that can autonomusly navigate in the simple maze environment and find the exit. The logic of robot is divided into four stages which are turing, moving, aligning and destination.

Moving State: The robot moves forward, monitors obstacles and targets. It transitions to Turning state when encountering obstacles, to Aligning state when detecting the target from the side, and to Destination state when reaching the target.

Turning State: When facing obstacles, the robot compares left and right sensor readings to decide rotation direction, completes a 90° turn, then returns to Moving state.

Aligning State: When side sensors detect the target, the robot stops and rotates until its front sensor faces the target directly, then returns to Moving state.

Destination State: The robot enters this terminal state upon reaching the target, stopping all movement.

These four states automatically transition through sensor-triggered events, creating a complete maze navigation system.RetryClaude can make mistakes. Please double-check responses.

## key chanllenges

1: how to memorize the path that robot has already traversed.
2: how to define different stages and when to transfer between them.
3: how to implement the robot's automatic acceleration and deceleration.

## how to run and use the projects

1: download the docker container with ENVIRO pre-loaded into it (In the root folder of the project)

```
docker run -p80:80 -p8765:8765 -v $PWD:/source -it klavins/enviro:v1.61 bash
```

2: Make the project and start the enviro server as follows.

```
make
enviro
```
