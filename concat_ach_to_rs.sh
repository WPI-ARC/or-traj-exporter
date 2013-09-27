#!/bin/bash

cp /home/jmainpri/workspace/ros_workspace/src/DRC/hubo_valve_turning_planner/valve_planner/trajectories/*.traj tmp/
./build/tconv -a -d tmp
./build/tconv -a2rs tmp/ach_final.traj
cp robot_commands.log /home/jmainpri/workspace/RobotSim
