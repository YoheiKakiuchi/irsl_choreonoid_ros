#!/usr/bin/env python3

from MotionExecuteGraphGenerator import MotionExecuteGraphGenerator as meg_gen

meg = meg_gen('sample.dot')

meg.extract_outcomes()

with open('smach_definition.py', mode='w') as f:
    meg.print_smach_definition(f)

with open('motionlib.py', mode='w') as f:
    meg.print_motion_library_template(f)
