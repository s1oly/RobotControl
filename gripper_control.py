#!/usr/bin/env python3

import sys
sys.path.append('/home/rocky/jingyixiang/base_ws/src/ur5_lib')

import gripper


if __name__ == '__main__':
    gripper.homing()
    gripper.set_force(force=20)
    gripper.move(width=75, speed=50)
