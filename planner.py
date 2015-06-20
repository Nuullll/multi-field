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
        return '(%0.1f, %0.1f)' % (self.x, self.y)

    def __str__(self):
        return '(%0.1f, %0.1f)' % (self.x, self.y)

    def __lt__(self, other):
        return self.x < other.x or (self.x == other.x and self.y < other.y)

    def __eq__(self, other):
        return (self.x == other.x) and (self.y == other.y)

    def __hash__(self):
        return hash(self.x) + hash(self.y)

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

    def remove(self, target):
        self.point_list.remove(target)

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

            self._convex_hull = ConvexHull(lower_hull + upper_hull)
            self.need_update = False

            self.inner_points = deepcopy(self.point_list)
            for point in self._convex_hull:
                self.inner_points.remove(point)

        return self._convex_hull

    def cover(self, point):
        """return whether point is inside convex hull's coverage"""
        for i in xrange(1, self.convex_hull.norm):
            if crossProduct(point, self.convex_hull[i-1], self.convex_hull[i]) <= 0:
                return False
        return True if crossProduct(point, self.convex_hull[-1], self.convex_hull[0]) > 0 else False

    def divide(self, partition):
        """determine the location of each point under the partition"""
        new_partition = []
        for triangle in partition:
            # IsInstance(triangle, ConvexHull) is True
            if not isinstance(triangle, Triangle):
                triangle = Triangle(triangle.point_list)    # triangle with no inner points
            for point in self.inner_points:
                triangle.addInnerPoints(point)
            new_partition.append(triangle)
        return new_partition

    def findBalanceKeySolution(self):
        pass


class ConvexHull(PointSet):
    def __init__(self, point_list):
        super(ConvexHull, self).__init__(point_list)
        
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

class Triangle(PointSet):
    """a point set whose convex hull is a triangle (ABC), find balance-key solution here"""

    result_map = {}
    
    def __init__(self, point_list):
        super(Triangle, self).__init__(point_list)
        self.A = self.convex_hull[0]
        self.B = self.convex_hull[1]
        self.C = self.convex_hull[2]

    def __repr__(self):
        return ('\n' + 'outer: ' + [self.A, self.B, self.C].__repr__() + '\n'
                + 'inner: ' + self.inner_points.__repr__())

    def __str__(self):
        return ('\n' + 'outer: ' + [self.A, self.B, self.C].__str__() + '\n'
                + 'inner: ' + self.inner_points.__str__())

    def __eq__(self, other):
        return (self.A == other.A and self.B == other.B and self.C == other.C)

    def __hash__(self):
        return self.A.__hash__() + self.B.__hash__() + self.C.__hash__()

    def addInnerPoints(self, new_point):
        if self.cover(new_point):
            self.point_list.append(new_point)
            self.inner_points.append(new_point)

    def divideIntoThreeTriangle(self, divider):
        ABD = Triangle([self.A, self.B, divider])
        BCD = Triangle([self.B, self.C, divider])
        DCA = Triangle([divider, self.C, self.A])
        return self.divide([ABD, BCD, DCA])
        
    def findInsideBalanceKeySolution(self):
        """return a map
        'key': min(max(degree_in) for all vertices) for all partition, 
        'links'
        'out_degree_A', 'out_degree_B', 'out_degree_C'
        'in_degree_A', 'in_degree_B', 'in_degree_C'
        remark: degree_in does NOT count the degree related with edge AB, BC, CA
        """
        result = {}
        if self.norm == 3:
            return {'key':0, 'links':[], 
                    'out_degree_A':0, 'out_degree_B':0, 'out_degree_C':0,
                    'in_degree_A':0, 'in_degree_B':0, 'in_degree_C':0}
        # traverse all possible divide point
        for divider in self.inner_points:    # mark divider as 'D'
            # traverse jet link in [A->D, B->D, C->D]
            for jet_point in [self.A, self.B, self.C]:  # mark jet_point as 'A'
                # traverse direction of BD
                for d_BD in [0, 1]:     # d_BD == 1 means B->D
                    # traverse direction of CD
                    for d_CD in [0, 1]:     # d_CD == 1 means C->D
                        sub_triangles = self.divideIntoThreeTriangle(divider)
                        sub_results = []
                        for triangle in sub_triangles:
                            if triangle.norm == 3:
                                sub_results.append(triangle.findInsideBalanceKeySolution())
                            elif triangle in Triangle.result_map:
                                sub_results.append(Triangle.result_map[triangle])
                            else:
                                Triangle.result_map[triangle] = triangle.findInsideBalanceKeySolution()
                                sub_results.append(Triangle.result_map[triangle])
                        tmp_result = {}

                        tmp_result['links'] = sub_results[0]['links'] + sub_results[1]['links'] + sub_results[2]['links']

                        if jet_point == self.A:
                            tmp_result['out_degree_A'] = 1 + sub_results[0]['out_degree_A'] + sub_results[2]['out_degree_A']
                            tmp_result['out_degree_B'] = d_BD + sub_results[0]['out_degree_B'] + sub_results[1]['out_degree_B']
                            tmp_result['out_degree_C'] = d_CD + sub_results[1]['out_degree_C'] + sub_results[2]['out_degree_C']
                            tmp_result['in_degree_A'] = sub_results[0]['in_degree_A'] + sub_results[2]['in_degree_A']
                            tmp_result['in_degree_B'] = 1-d_BD + sub_results[0]['in_degree_B'] + sub_results[1]['in_degree_B']
                            tmp_result['in_degree_C'] = 1-d_CD + sub_results[1]['in_degree_C'] + sub_results[2]['in_degree_C']
                            tmp_result['links'].append(Link(self.A, divider, jet_link=True, triangle=self))
                            tmp_result['links'].append(Link(self.B, divider, 1-d_BD))
                            tmp_result['links'].append(Link(self.C, divider, 1-d_CD))
                        elif jet_point == self.B:
                            tmp_result['out_degree_B'] = 1 + sub_results[0]['out_degree_B'] + sub_results[2]['out_degree_B']
                            tmp_result['out_degree_C'] = d_BD + sub_results[0]['out_degree_C'] + sub_results[1]['out_degree_C']
                            tmp_result['out_degree_A'] = d_CD + sub_results[1]['out_degree_A'] + sub_results[2]['out_degree_A']
                            tmp_result['in_degree_B'] = sub_results[0]['in_degree_B'] + sub_results[2]['in_degree_B']
                            tmp_result['in_degree_C'] = 1-d_BD + sub_results[0]['in_degree_C'] + sub_results[1]['in_degree_C']
                            tmp_result['in_degree_A'] = 1-d_CD + sub_results[1]['in_degree_A'] + sub_results[2]['in_degree_A']
                            tmp_result['links'].append(Link(self.B, divider, jet_link=True, triangle=self))
                            tmp_result['links'].append(Link(self.C, divider, 1-d_BD))
                            tmp_result['links'].append(Link(self.A, divider, 1-d_CD))
                        else:
                            tmp_result['out_degree_C'] = 1 + sub_results[0]['out_degree_C'] + sub_results[2]['out_degree_C']
                            tmp_result['out_degree_A'] = d_BD + sub_results[0]['out_degree_A'] + sub_results[1]['out_degree_A']
                            tmp_result['out_degree_B'] = d_CD + sub_results[1]['out_degree_B'] + sub_results[2]['out_degree_B']
                            tmp_result['in_degree_C'] = sub_results[0]['in_degree_C'] + sub_results[2]['in_degree_C']
                            tmp_result['in_degree_A'] = 1-d_BD + sub_results[0]['in_degree_A'] + sub_results[1]['in_degree_A']
                            tmp_result['in_degree_B'] = 1-d_CD + sub_results[1]['in_degree_B'] + sub_results[2]['in_degree_B']
                            tmp_result['links'].append(Link(self.C, divider, jet_link=True, triangle=self))
                            tmp_result['links'].append(Link(self.A, divider, 1-d_BD))
                            tmp_result['links'].append(Link(self.B, divider, 1-d_CD))

                        in_degree_D = 1 + d_BD + d_CD + sub_results[0]['in_degree_C'] + sub_results[1]['in_degree_C'] + sub_results[2]['in_degree_A']

                        tmp_result['key'] = max(max([sub_result['key'] for sub_result in sub_results]), 
                                            tmp_result['in_degree_A'], tmp_result['in_degree_B'], tmp_result['in_degree_C'],
                                            in_degree_D)

                        if tmp_result['out_degree_A'] > 8 or tmp_result['out_degree_B'] > 8 or tmp_result['out_degree_C'] > 8:
                            tmp_result['key'] = float('inf')

                        # test feasibility
                        if not testFeasibility(tmp_result['links']):
                            tmp_result['key'] = float('inf')

                        if result == {} or tmp_result['key'] < result['key']:
                            result = tmp_result
        return result

def testFeasibility(links):
    jet_links = [link for link in links if link.jet_link]
    jet_links.reverse();

    for jet_link in jet_links:
        pass

def drawOtherLinks(jet_link, jet_links, links, edges=[]):
    BD = Link(jet_link.triangle.B, jet_link.target)
    CD = Link(jet_link.triangle.C, jet_link.target)
    BC = Link(jet_link.triangle.B, jet_link.triangle.C)
    AB = Link(jet_link.origin, jet_link.triangle.B)
    AC = Link(jet_link.origin, jet_link.triangle.C)
    seq = [BD, CD, BC, AB, AC]
    can_link = []
    for edge in seq:
        if edge in jet_links:
            return edges + drawOtherLinks(edge, jet_links, links, edges)
        else:
            for link in links:
                if link == edge:    # __eq__ (almost equal)
                    edge = link
                    break
            # find triangle
            # get all vertices
            vertices = []
            for ele in edges:
                vertices.append(ele.origin)
                vertices.append(ele.target)
            vertices = {}.fromkeys(vertices).keys()

            for ele in edges:
                for vertex in vertices:
                    if Link(ele.origin, vertex) in edges and Link(ele.target, vertex) in edges:
                        # find a triangle
                        if Triangle([ele.origin, ele.target, vertex].cover(edge.origin)):
                            # origin inside an existing triangle
                            raise NameError('Inside an existing field!')

            # can link
            can_link.append(edge)

    return edges + can_link


class Link(object):
    """link from origin to target"""
    def __init__(self, origin, target, reverse=False, jet_link=False, triangle=None):
        super(Link, self).__init__()
        self.origin = origin if not reverse else target
        self.target = target if not reverse else origin
        self.jet_link = jet_link
        self.triangle = triangle

    def __repr__(self):
        return self.origin.__repr__() + '->' + self.target.__str__()

    def __str__(self):
        return self.origin.__str__() + '->' + self.target.__str__()

    def __eq__(self, other):
        return ((self.origin == other.origin and self.target == other.target)
                or (self.origin == other.target and self.target == other.origin))


if __name__ == '__main__':
    # s = PointSet([Point(0,0), Point(0,1), Point(1,1), Point(1.2,0.2), Point(2,0), Point(0.5,-0.5), Point(1,-1)])
    # print s.convex_hull
    # print s.inner_points
    # print s.cover(Point(0.1,0.1))

    # ch = s.convex_hull
    # partitions = ch.triangulation()
    # for partition in partitions:
    #     # for element in partition:
    #     for triangle in s.divide(partition):
    #         print triangle
    #     print

    t = Triangle([Point(0,0), Point(10,30), Point(30,0), Point(10,10), Point(20,10), 
                  Point(5,10), Point(20,5), Point(20,6), Point(21,4), Point(11,13), Point(13,3)])
    # for triangle in t.divideIntoThreeTriangle(Point(1,1)):
    #     print triangle
    # print 

    # print {}.fromkeys([Point(0,0), Point(0,0)]).keys()

    # print t.findInsideBalanceKeySolution()