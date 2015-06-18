# -*- coding: UTF-8 -*-
# planner.py

from copy import deepcopy
from itertools import product

class Point(object):
    """Point (x, y)"""
    def __init__(self, x, y):
        super(Point, self).__init__()
        self.x = x
        self.y = y

    def __repr__(self):
        return '(%d, %d)' % (self.x, self.y)

    def __str__(self):
        return '<Point (%0.1f, %0.1f)>' % (self.x, self.y)

    def __lt__(self, other):
        return self.x < other.x or (self.x == other.x and self.y < other.y)

    def __eq__(self, other):
        return (self.x == other.x) and (self.y == other.y)

def crossProduct(p0, p1, p2):
    """return vector (p0->p1) cross (p0->p2)"""
    return (p1.x - p0.x) * (p2.y - p0.y) - (p2.x - p0.x) * (p1.y - p0.y)

class PointSet(object):
    """Set {P1, P2, ...}"""
    def __init__(self, point_list):
        super(PointSet, self).__init__()
        self.point_list = point_list
        self.need_update = True

    def __repr__(self):
        return self.point_list.__repr__()

    def __str__(self):
        return self.point_list.__str__()

    def __iter__(self):
        return self.point_list.__iter__()

    def __getitem__(self, index):
        return self.point_list[index]

    @property
    def norm(self):
        return len(self.point_list)

    def sortPoint(self):
        self.point_list.sort()

    def addPoint(self, new_point):
        self.point_list.append(new_point)
        self.need_update = True

    @property
    def convex_hull(self):
        """return vertex set of convex hull"""
        if self.need_update:
            # sort by coordinate x
            self.sortPoint()

            lower_hull = []
            # lower hull
            for point in self:
                while (len(lower_hull) >= 2
                        and crossProduct(lower_hull[-2], lower_hull[-1], point) <= 0):
                    lower_hull.pop()
                lower_hull.append(point)

            upper_hull = []
            # upper hull
            for point in self:
                while (len(upper_hull) >= 2
                        and crossProduct(upper_hull[-2], upper_hull[-1], point) >= 0):
                    upper_hull.pop()
                upper_hull.append(point)
            upper_hull.pop(0)
            upper_hull.pop()
            upper_hull.reverse()

            self._convex_hull = lower_hull + upper_hull
            self.need_update = False

            self._inner_points = deepcopy(self.point_list)
            for point in self._convex_hull:
                self.inner_points.remove(point)

        return ConvexHull(self._convex_hull)

    @property
    def inner_points(self):
        if self.need_update:
            self.convex_hull
        return self._inner_points

    def cover(self, point):
        """return whether point is inside convex hull's coverage"""
        for i in xrange(1, self.convex_hull.norm):
            if crossProduct(point, self.convex_hull[i-1], self.convex_hull[i]) < 0:
                return False
        return True


class ConvexHull(PointSet):
    def __init__(self, point_list):
        super(ConvexHull, self).__init__(point_list)

    def partition(self, index):
        partitions = []
        middle_part = ConvexHull([self[0], self[index], self[-1]])
        right_part = ConvexHull(self[:index+1])
        left_part = ConvexHull(self[index:])
        if self.norm >= 3:
            for i in range(1, right_part.norm-1):
                for j in range(1, left_part.norm-1):
                    partitions.append([middle_part, right_part.partition(i), left_part.partition(j)])
        return partitions
        
    def triangulation(self):
        """return set of [triangle, triangle, ...]"""
        partitions = []
        partition = []
        if self.norm >= 3:
            for i in range(1, self.norm-1):
                middle_part = ConvexHull([self[0], self[i], self[-1]])
                right_part = ConvexHull(self[:i+1])
                left_part = ConvexHull(self[i:])
                if right_part.norm < 3:
                    if left_part.norm < 3:
                        partition.append(middle_part)
                        partitions.append(partition)
                    else:
                        for a in middle_part.triangulation():
                            for b in left_part.triangulation():
                                partitions.append(partition + a + b)
                elif left_part.norm < 3:
                    for a in middle_part.triangulation():
                        for b in right_part.triangulation():
                            partitions.append(partition + a + b)
                else:
                    for a in middle_part.triangulation():
                        for b in right_part.triangulation():
                            for c in left_part.triangulation():
                                partitions.append(partition + a + b + c)
        return partitions

if __name__ == '__main__':
    s = PointSet([Point(0,0), Point(0,1), Point(1,1), Point(1.2,0.2), Point(2,0), Point(0.5,-0.5), Point(1,-1)])
    print s.convex_hull
    print s.inner_points
    print s.cover(Point(0.1,0.1))

    ch = s.convex_hull
    for partition in ch.triangulation():
        print partition

