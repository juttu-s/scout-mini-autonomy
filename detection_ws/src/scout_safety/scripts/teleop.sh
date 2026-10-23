#!/bin/bash
# Keyboard teleop through the safety layer (publishes /teleop/cmd_vel, never /cmd_vel).
# repeat_rate keeps commands coming faster than the safety node's 0.5 s input timeout; key_timeout stops on release.
source ~/ros_utils/setup.bash
rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=/teleop/cmd_vel \
  _speed:=${SPEED:-0.2} _turn:=${TURN:-0.5} _repeat_rate:=10.0 _key_timeout:=0.6
