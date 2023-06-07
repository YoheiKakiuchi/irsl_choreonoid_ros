#!/usr/bin/env python3

## for smach
import smach_definition
import motionlib

mlib = motionlib.MotionLib()

sm = smach_definition.create_state_machine(lib=mlib)

print('start state-machine')
result = sm.execute()
print('result = {}'.format(result))
print('userdata = {}'.format(sm.userdata.udata))
