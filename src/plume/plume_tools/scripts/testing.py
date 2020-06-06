#! /usr/bin/env python

def abc(*dir):
    if not dir:
        dir = 1
    else:
        dir = dir[0]
    dir = dir + 1    
    print(dir)

abc()