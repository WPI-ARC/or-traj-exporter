#!/bin/bash

export dir=$1

echo "moving trajectories to "$dir

cp $dir/home2init.txt data/movetraj0.txt
cp $dir/init2start.txt data/movetraj1.txt
cp $dir/start2goal.txt data/movetraj2.txt
cp $dir/goal2start.txt data/movetraj3.txt
cp $dir/start2init.txt data/movetraj4.txt
cp $dir/init2home.txt data/movetraj5.txt
