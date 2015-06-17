# -*- coding: UTF-8 -*-
# planner.py

class Point(object):
    """Point (x, y)"""
    def __init__(self, x, y):
        super(Point, self).__init__()
        self.x = x
        self.y = y

def crossProduct(p0, p1, p2):
    """return vector (p0->p1) cross (p0->p2)"""
    return (p1.x - p0.x) * (p2.y - p0.y) - (p2.x - p0.x) * (p1.y - p0.y)

class PointSet(object):
    """Set {P1, P2, ...}"""
    def __init__(self, point_list):
        super(PointSet, self).__init__()
        self.point_list = point_list

    def sortByX(self):
        cmp = lambda p1, p2: p1.x < p2.x
        self.point_list.sort(cmp)

    def convexHull(self):
        """return vertex set of convex hull"""
        