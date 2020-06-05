#! /usr/bin/env python

class waypoints:
    def __init__(self):
        self.a = 0

    def abccd(self):
        pass
        
class abc(waypoints):
    def __init__(self):
        waypoints.__init__()

A = abc()
print(A.a)
        
