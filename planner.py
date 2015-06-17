# -*- coding: UTF-8 -*-
# planner.py

class Point(object):
    """Point (x, y)"""
    def __init__(self, x, y):
        super(Point, self).__init__()
        self.x = x
        self.y = y

    def __repr__(self):
        return '<Point (%0.1f, %0.1f)>' % (self.x, self.y)

    def __str__(self):
        return '<Point (%0.1f, %0.1f)>' % (self.x, self.y)

    def __lt__(self, other):
        return self.x < other.x or (self.x == other.x and self.y < other.y)

def crossProduct(p0, p1, p2):
    """return vector (p0->p1) cross (p0->p2)"""
    return (p1.x - p0.x) * (p2.y - p0.y) - (p2.x - p0.x) * (p1.y - p0.y)

class PointSet(object):
    """Set {P1, P2, ...}"""
    def __init__(self, point_list):
        super(PointSet, self).__init__()
        self.point_list = point_list

    def __repr__(self):
        return self.point_list.__repr__()

    def __str__(self):
        return self.point_list.__str__()

    @property
    def norm(self):
        return len(self.point_list)

    def sortPoint(self):
        self.point_list.sort()

    def addPoint(self, new_point):
        self.point_list.append(new_point)

    def convexHull(self):
        """return vertex set of convex hull"""
        # sort by coordinate x
        self.sortPoint()

        lower_hull = []
        # lower hull
        for point in self.point_list:
            while (len(lower_hull) >= 2
                    and crossProduct(lower_hull[-2], lower_hull[-1], point) <= 0):
                lower_hull.pop()
            lower_hull.append(point)

        upper_hull = []
        # upper hull
        for point in self.point_list:
            while (len(upper_hull) >= 2
                    and crossProduct(upper_hull[-2], upper_hull[-1], point) >= 0):
                upper_hull.pop()
            upper_hull.append(point)
        upper_hull.pop(0)
        upper_hull.pop()
        upper_hull.reverse()

        return PointSet(lower_hull + upper_hull)


if __name__ == '__main__':
    s = PointSet([Point(0,0), Point(0,1), Point(1,1), Point(1.2,0.2), Point(2,0), Point(0.5,-0.5), Point(1,-1)])
    print s.convexHull()
