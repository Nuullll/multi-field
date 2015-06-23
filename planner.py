# -*- coding: UTF-8 -*-
# planner.py

from copy import deepcopy
from itertools import product
from random import shuffle

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
    
    def __init__(self, point_list, custom=False):
        super(Triangle, self).__init__(point_list)
        if not custom:
            self.A = self.convex_hull[0]
            self.B = self.convex_hull[1]
            self.C = self.convex_hull[2]
        else:
            self.A = point_list[0]
            self.B = point_list[1]
            self.C = point_list[2]

    def __repr__(self):
        return ('\n' + 'outer: ' + [self.A, self.B, self.C].__repr__() + '\n'
                + 'inner: ' + self.inner_points.__repr__())

    def __str__(self):
        return ('\n' + 'outer: ' + [self.A, self.B, self.C].__str__() + '\n'
                + 'inner: ' + self.inner_points.__str__())

    def __eq__(self, other):
        return ((self.A == other.A and self.B == other.B and self.C == other.C)
             or (self.A == other.B and self.B == other.C and self.C == other.A)
             or (self.A == other.C and self.B == other.A and self.C == other.B))

    def __hash__(self):
        return self.A.__hash__() + self.B.__hash__() + self.C.__hash__()

    def addInnerPoints(self, new_point):
        if self.cover(new_point):
            self.point_list.append(new_point)
            self.inner_points.append(new_point)

    def nextVertex(self, vertex):
        if vertex == self.A:
            return self.B
        elif vertex == self.B:
            return self.C
        else:
            return self.A

    def previousVertex(self, vertex):
        if vertex == self.A:
            return self.C
        elif vertex == self.B:
            return self.A
        else:
            return self.B

    def divideIntoThreeTriangle(self, divider):
        ABD = Triangle([self.A, self.B, divider], custom=True)
        BCD = Triangle([self.B, self.C, divider], custom=True)
        DCA = Triangle([divider, self.C, self.A], custom=True)
        return self.divide([ABD, BCD, DCA])
        
    def findInsideBalanceKeySolution(self, outer_links=None, depth=0):
        """return a map
        'key': min(max(degree_in) for all vertices) for all partition, 
        'links'
        'out_degree_A', 'out_degree_B', 'out_degree_C'
        'in_degree_A', 'in_degree_B', 'in_degree_C'
        """
        if outer_links == None:
            outer_links_set = product([Link(self.A, self.B), Link(self.A, self.B, reverse=True)],
                                      [Link(self.B, self.C), Link(self.B, self.C, reverse=True)],
                                      [Link(self.C, self.A), Link(self.C, self.A, reverse=True)])
        else:
            outer_links_set = [outer_links]
        result = {}
        if self.norm == 3:
            return {'key':0, 'links':[],
                    'out_degree_A':0, 'out_degree_B':0, 'out_degree_C':0,
                    'in_degree_A':0, 'in_degree_B':0, 'in_degree_C':0}
        for outer_links in outer_links_set:
            # traverse all possible divide point
            for divider in self.inner_points:    # mark divider as 'D'
                # traverse jet link in [A->D, B->D, C->D]
                for jet_point in [self.A, self.B, self.C]:  # mark jet_point as 'A'
                    # traverse direction of BD
                    for d_BD in [0, 1]:     # d_BD == 1 means B->D
                        # traverse direction of CD
                        for d_CD in [0, 1]:     # d_CD == 1 means C->D
                            if jet_point == self.A:
                                AD = Link(self.A, divider, jet_link=True, triangle=self)
                                BD = Link(self.B, divider, 1-d_BD)
                                CD = Link(self.C, divider, 1-d_CD)
                            elif jet_point == self.B:
                                AD = Link(self.A, divider, 1-d_CD)
                                BD = Link(self.B, divider, jet_link=True, triangle=self)
                                CD = Link(self.C, divider, 1-d_BD)
                            else:
                                AD = Link(self.A, divider, 1-d_BD)
                                BD = Link(self.B, divider, 1-d_CD)
                                CD = Link(self.C, divider, jet_link=True, triangle=self)

                            sub_triangles = self.divideIntoThreeTriangle(divider)
                            sub_results = []
                            for triangle in sub_triangles:
                                if triangle.norm == 3:
                                    sub_results.append(triangle.findInsideBalanceKeySolution(depth=depth+1))
                                elif triangle in Triangle.result_map:
                                    sub_results.append(Triangle.result_map[triangle])
                                else:
                                    index = sub_triangles.index(triangle)
                                    if index == 0:
                                        tmp = triangle.findInsideBalanceKeySolution([outer_links[0], BD, AD], depth=depth+1)
                                    elif index == 1:
                                        tmp = triangle.findInsideBalanceKeySolution([outer_links[1], CD, BD], depth=depth+1)
                                    else:
                                        tmp = triangle.findInsideBalanceKeySolution([CD, outer_links[2], AD], depth=depth+1)
                                    Triangle.result_map[triangle] = tmp
                                    sub_results.append(tmp)
                            tmp_result = {}

                            tmp_result['links'] = sub_results[0]['links'] + sub_results[1]['links'] + sub_results[2]['links']

                            tmp_result['links'] += [AD, BD, CD]
                            tmp_result['out_degree_A'] = sub_results[0]['out_degree_A'] + sub_results[2]['out_degree_C']
                            tmp_result['out_degree_B'] = sub_results[0]['out_degree_B'] + sub_results[1]['out_degree_A']
                            tmp_result['out_degree_C'] = sub_results[1]['out_degree_B'] + sub_results[2]['out_degree_B']
                            tmp_result['in_degree_A'] = sub_results[0]['in_degree_A'] + sub_results[2]['in_degree_C']
                            tmp_result['in_degree_B'] = sub_results[0]['in_degree_B'] + sub_results[1]['in_degree_A']
                            tmp_result['in_degree_C'] = sub_results[1]['in_degree_B'] + sub_results[2]['in_degree_B']

                            if jet_point == self.A:
                                tmp_result['out_degree_A'] += 1
                                tmp_result['out_degree_B'] += d_BD
                                tmp_result['out_degree_C'] += d_CD
                                tmp_result['in_degree_B'] += 1-d_BD
                                tmp_result['in_degree_C'] += 1-d_CD
                            elif jet_point == self.B:
                                tmp_result['out_degree_B'] += 1
                                tmp_result['out_degree_C'] += d_BD
                                tmp_result['out_degree_A'] += d_CD
                                tmp_result['in_degree_C'] += (1-d_BD)
                                tmp_result['in_degree_A'] += (1-d_CD)
                            else:
                                tmp_result['out_degree_C'] += 1
                                tmp_result['out_degree_A'] += d_BD
                                tmp_result['out_degree_B'] += d_CD
                                tmp_result['in_degree_A'] += (1-d_BD)
                                tmp_result['in_degree_B'] += (1-d_CD)

                            in_degree_D = 1 + d_BD + d_CD + sub_results[0]['in_degree_C'] + sub_results[1]['in_degree_C'] + sub_results[2]['in_degree_A']
                            if depth == 0:
                                tmp_result['in_degree_D'] = in_degree_D
                                # print divider, jet_point, d_BD, d_CD
                                # print sub_results
                                # raw_input()
                            # if self == Triangle([Point(0,0), Point(30,0), Point(10,30)]):
                            #     print tmp_result
                            str_dict = {self.A:'A', self.B:'B', self.C:'C'}
                            actual_out_degrees = {'A':tmp_result['out_degree_A'], 'B':tmp_result['out_degree_B'], 'C':tmp_result['out_degree_C']}
                            actual_in_degrees = {'A':tmp_result['in_degree_A'], 'B':tmp_result['in_degree_B'], 'C':tmp_result['in_degree_C']}
                            for link in outer_links:
                                actual_out_degrees[str_dict[link.origin]] += 1
                                actual_in_degrees[str_dict[link.target]] += 1
                            if depth == 0:
                                # print tmp_result['links']
                                for p in str_dict.values():
                                    tmp_result['out_degree_'+p] = actual_out_degrees[p]
                                    tmp_result['in_degree_'+p] = actual_in_degrees[p]

                            tmp_result['key'] = max(max([sub_result['key'] for sub_result in sub_results]), 
                                                max(actual_in_degrees.values()), in_degree_D)

                            if max(actual_out_degrees.values()) > 8:
                                tmp_result['key'] = float('inf')


                            # print 'wtf'
                            # print tmp_result['links']
                            # print outer_links

                            # test feasibility
                            test_result = testFeasibility(tmp_result['links'] + list(outer_links))
                            if not test_result[0]:
                                tmp_result['key'] = float('inf')
                            elif depth == 0:
                                tmp_result['links'] = test_result[1]

                            if result == {} or tmp_result['key'] <= result['key']:
                                result = tmp_result

                            # print 'depth:', depth
                            # print self.A, self.B, self.C
                            # print outer_links
                            # print tmp_result
                            # print in_degree_D
                            # print actual_out_degrees
                            # print actual_in_degrees
                            # raw_input()
        return result

def testFeasibility(links):
    jet_links = [link for link in links if link.jet_link]
    jet_links.sort()

    try:
        link_seq = drawOtherLinks(jet_links[0], jet_links, links)
    except NameError:
        return (False, [])
    # except ValueError:
    #     return (True, links)
    return (True, link_seq)

def drawOtherLinks(jet_link, jet_links, links, edges=[]):
    A = jet_link.origin
    D = jet_link.target
    B = jet_link.triangle.nextVertex(A)
    C = jet_link.triangle.previousVertex(A)
    BD = Link(B, D)
    CD = Link(C, D)
    BC = Link(B, C)
    AB = Link(A, B)
    AC = Link(A, C)
    seq = [edge for edge in [BD, CD, BC, AB, AC] if edge not in edges]
    can_link = []
    # print 'jet_links: ', jet_links
    # print 'links: ', links
    # print 'edges: ', edges
    # print 'seq: ', seq
    for edge in seq:
        # print jet_links
        # print edge
        # print
        for link in links:
            if edge == link:
                edge = link
        # find triangle
        # get all vertices
        vertices = []
        for ele in edges + can_link:
            vertices.append(ele.origin)
            vertices.append(ele.target)
        vertices = {}.fromkeys(vertices).keys()

        for ele in edges + can_link:
            for vertex in vertices:
                if Link(ele.origin, vertex) in edges + can_link and Link(ele.target, vertex) in edges + can_link:
                    # find a triangle
                    if Triangle([ele.origin, ele.target, vertex]).cover(edge.origin):
                        # origin inside an existing triangle
                        raise NameError('Inside an existing field!')

        if edge in jet_links:
            # print 'wtf'
            return (drawOtherLinks(edge, jet_links, links, edges + can_link))
        else:
            # print links
            # for jet_link in jet_links:
            #     print jet_link.triangle
            # print edge
            # print 
            # can link
            can_link.append(edge)
    jet_links.remove(jet_link)
    if jet_links != []:
        return drawOtherLinks(jet_links[0], jet_links, links, edges + can_link + [jet_link])
    else:
        return edges + can_link + [jet_link]


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

    def __lt__(self, other):
        if self.jet_link and other.jet_link:
            if other.triangle.cover(self.origin):
                return -1
            elif self.triangle.cover(other.origin):
                return 1
            else:
                return 0
        return 0


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

    t = Triangle([Point(0,0), Point(10,30), Point(30,0), Point(20,10), Point(8,7), Point(15,9), Point(10,1),
                  Point(5,10), Point(20,5), Point(21,4), Point(11,13), Point(13,3)])
    # t = Triangle([Point(0,0), Point(10,30), Point(30,0), Point(10,10), Point(5,3), Point(20,4)])
    # t = Triangle([Point(0,0), Point(10,30), Point(30,0), Point(10,10), Point(5,10)])
    # for triangle in t.divideIntoThreeTriangle(Point(1,1)):
    #     print triangle
    # print t.cover(Point(1,1))

    # l = [Link(Point(0,0), Point(0,1))]
    # print l.index(Link(Point(0,1), Point(0,0)))

    result = t.findInsideBalanceKeySolution()
    print result
    print 
    i = 0
    for link in result['links']:
        print i, ': ', link
        i += 1