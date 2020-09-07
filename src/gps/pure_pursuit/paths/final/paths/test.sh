#!/bin/bash

A=10
for var in {1..29}
do
  if [ $var -lt $A ]; then
    ./waypoint_mode.py -m $var -p 0$var.txt
  else
    ./waypoint_mode.py -m $var -p $var.txt
  fi
done
