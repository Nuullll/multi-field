# -*- coding: UTF-8 -*-
# planner.py

from copy import deepcopy
from itertools import product
from random import shuffle
from operator import itemgetter

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
        if isinstance(other, Point):
            return (self.x == other.x) and (self.y == other.y)
        else:
            return False

    def __ne__(self, other):
        return (not self.__eq__(other))

    def __hash__(self):
        return hash(self.__repr__())

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
        partitions = [self.divide(partition) for partition in self.convex_hull.triangulation()]
        solution = {}
        for partition in partitions:
            # print partition
            # raw_input()
            triangle_result_map = {}
            for triangle in partition:
                # print triangle
                # raw_input()
                if triangle not in triangle_result_map:
                    outer_links_set = None
                    init_out = {}
                    init_in = {}
                    for tri in triangle_result_map:
                        result = triangle_result_map[tri]
                        if tri.isNeighbour(triangle):
                            str_dict = {tri.A:'A', tri.B:'B', tri.C:'C'}
                            common_edge = tri.commonEdge(triangle)
                            # print common_edge
                            # print result['links']
                            # raw_input()
                            directed_edge = result['links'][result['links'].index(common_edge)]
                            # print directed_edge
                            # print common_edge
                            outer_links_set = genOuterLinksSet(triangle.getOuterIndex(directed_edge), directed_edge, triangle)
                            init_out = {directed_edge.origin:result['out_degree_'+str_dict[directed_edge.origin]]-1}
                            init_in = {directed_edge.target:result['in_degree_'+str_dict[directed_edge.target]]-1} 
                            break
                    triangle_result_map[triangle] = triangle.findBalanceKeySolution(outer_links_set=outer_links_set,
                                                                                    init_out=init_out, init_in=init_in)
                # print triangle_result_map

            inner_max_key = max([value['key'] for value in triangle_result_map.values()])
            outer_max_key_dict = dict([(vertex, 0) for vertex in self.convex_hull])
            outer_out_degree_dict = dict([(vertex, 0) for vertex in self.convex_hull])
            tmp_links = []
            for triangle, result in triangle_result_map.items():
                tmp_links += result['links']
                # str_dict = {triangle.A:'A', triangle.B:'B', triangle.C:'C'}
                outer_max_key_dict[triangle.A] += result['in_degree_A']
                outer_max_key_dict[triangle.B] += result['in_degree_B']
                outer_max_key_dict[triangle.C] += result['in_degree_C']
                outer_out_degree_dict[triangle.A] += result['out_degree_A']
                outer_out_degree_dict[triangle.B] += result['out_degree_B']
                outer_out_degree_dict[triangle.C] += result['out_degree_C']
                for link in result['links']:
                    if (link.origin in [triangle.A, triangle.B, triangle.C] and
                        link.target in [triangle.A, triangle.B, triangle.C]):
                        # whether link is bound of convex hull
                        i1 = self.convex_hull.point_list.index(link.origin)
                        i2 = self.convex_hull.point_list.index(link.target)
                        if not((abs(i1 - i2) == 1) or (i1 + i2 == self.convex_hull.norm - 1)):
                            outer_max_key_dict[link.target] -= 0.5
                            outer_out_degree_dict[link.origin] -= 0.5
            max_key = max(inner_max_key, max(outer_max_key_dict.values()))
            max_out_degree = max(outer_out_degree_dict.values())
            links = []
            for link in tmp_links:
                if link not in links:
                    links.append(link)
            if max_out_degree > 8:
                max_key = float('inf')
            if solution == {} or max_key <= solution['max_key']:
                solution = {'max_key':max_key, 'max_out_degree':max_out_degree, 'links':links}
        return solution


def genOuterLinksSet(index, link, triangle):
    sets = [[Link(triangle.A, triangle.B), Link(triangle.A, triangle.B, reverse=True)],
            [Link(triangle.B, triangle.C), Link(triangle.B, triangle.C, reverse=True)],
            [Link(triangle.C, triangle.A), Link(triangle.C, triangle.A, reverse=True)]]
    sets[index] = [link]
    # print link
    return product(sets[0], sets[1], sets[2])


class ConvexHull(PointSet):
    def __init__(self, point_list):
        super(ConvexHull, self).__init__(point_list)
        
    def isNeighbour(self, other):
        return len(set(self.point_list).intersection(set(other.point_list))) == 2

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
                        for a in left_part.triangulation():
                            for b in middle_part.triangulation():
                                partitions.append(a + b + partition)
                elif left_part.norm < 3:
                    for a in middle_part.triangulation():
                        for b in right_part.triangulation():
                            partitions.append(a + partition + b)
                else:
                    for a in left_part.triangulation():
                        for b in middle_part.triangulation():
                            for c in right_part.triangulation():
                                partitions.append(a + b + partition + c)
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

    def isNeighbour(self, other):
        return len(set([self.A, self.B, self.C]).intersection(set([other.A, other.B, other.C]))) == 2

    def commonEdge(self, other):
        intersection = list(set([self.A, self.B, self.C]).intersection(set([other.A, other.B, other.C])))
        return Link(intersection[0], intersection[1])

    def getOuterIndex(self, link):
        """return index of link in [AB, BC, CA]"""
        if link == Link(self.A, self.B):
            return 0
        elif link == Link(self.B, self.C):
            return 1
        elif link == Link(self.C, self.A):
            return 2
        else:
            return None

    def divideIntoThreeTriangle(self, divider):
        ABD = Triangle([self.A, self.B, divider], custom=True)
        BCD = Triangle([self.B, self.C, divider], custom=True)
        DCA = Triangle([divider, self.C, self.A], custom=True)
        return self.divide([ABD, BCD, DCA])
        
    def findBalanceKeySolution(self, outer_links_set=None, depth=0, init_out={}, init_in={}):
        """return a map
        'key': min(max(degree_in) for all vertices) for all partition, 
        'links'
        'out_degree_A', 'out_degree_B', 'out_degree_C'
        'in_degree_A', 'in_degree_B', 'in_degree_C'
        """
        if outer_links_set == None:
            outer_links_set = product([Link(self.A, self.B), Link(self.A, self.B, reverse=True)],
                                      [Link(self.B, self.C), Link(self.B, self.C, reverse=True)],
                                      [Link(self.C, self.A), Link(self.C, self.A, reverse=True)])
        result = {}
        if self.norm == 3:
            result = {'key':0, 'links':[],
                    'out_degree_A':0, 'out_degree_B':0, 'out_degree_C':0,
                    'in_degree_A':0, 'in_degree_B':0, 'in_degree_C':0}
            # if depth == 0:
            result['links'] = testFeasibility(list(iter(outer_links_set).next()))[1]
            return result
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
                                    sub_results.append(triangle.findBalanceKeySolution(depth=depth+1, init_out=init_out, init_in=init_in))
                                elif triangle in Triangle.result_map:
                                    sub_results.append(Triangle.result_map[triangle])
                                else:
                                    index = sub_triangles.index(triangle)
                                    if index == 0:
                                        # print [outer_links[0], BD, AD]
                                        # print triangle
                                        # raw_input()
                                        tmp = triangle.findBalanceKeySolution([[outer_links[0], BD, AD]], depth=depth+1, init_out=init_out, init_in=init_in)
                                    elif index == 1:
                                        tmp = triangle.findBalanceKeySolution([[outer_links[1], CD, BD]], depth=depth+1, init_out=init_out, init_in=init_in)
                                    else:
                                        tmp = triangle.findBalanceKeySolution([[CD, outer_links[2], AD]], depth=depth+1, init_out=init_out, init_in=init_in)
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
                            str_dict = {self.A:'A', self.B:'B', self.C:'C'}
                            actual_out_degrees = {'A':tmp_result['out_degree_A'], 'B':tmp_result['out_degree_B'], 'C':tmp_result['out_degree_C']}
                            actual_in_degrees = {'A':tmp_result['in_degree_A'], 'B':tmp_result['in_degree_B'], 'C':tmp_result['in_degree_C']}
                            for link in outer_links:
                                # print self.A, self.B, self.C
                                # print outer_links
                                # print link
                                # raw_input()
                                actual_out_degrees[str_dict[link.origin]] += 1
                                actual_in_degrees[str_dict[link.target]] += 1

                            for point in [self.A, self.B, self.C]:
                                if point in init_out:
                                    actual_out_degrees[str_dict[point]] += init_out[point]
                                if point in init_in:
                                    actual_in_degrees[str_dict[point]] += init_in[point]

                            if depth == 0:
                                for p in str_dict.values():
                                    tmp_result['out_degree_'+p] = actual_out_degrees[p]
                                    tmp_result['in_degree_'+p] = actual_in_degrees[p]

                            tmp_result['key'] = max(max([sub_result['key'] for sub_result in sub_results]), 
                                                max(actual_in_degrees.values()), in_degree_D)

                            if max(actual_out_degrees.values()) > 8:
                                tmp_result['key'] = float('inf')

                            # test feasibility
                            test_result = testFeasibility(tmp_result['links'] + list(outer_links))
                            if not test_result[0]:
                                tmp_result['key'] = float('inf')
                            elif depth == 0:
                                tmp_result['links'] = test_result[1]

                            if result == {} or tmp_result['key'] <= result['key']:
                                result = tmp_result
        return result

def testFeasibility(links):
    jet_links = [link for link in links if link.jet_link]
    jet_links.sort()

    try:
        link_seq = drawOtherLinks(jet_links[0], jet_links, links)
    except NameError:
        return (False, [])
    except IndexError:
        return (True, links)
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
    for edge in seq:
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
            return (drawOtherLinks(edge, jet_links, links, edges + can_link))
        else:
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

    def __hash__(self):
        return self.origin.__hash__() + self.target.__hash__()


if __name__ == '__main__':
    # s = PointSet([Point(0,0), Point(30,0), Point(40,30), Point(20,50), Point(10,20), Point(20,10),
    #               Point(8,7), Point(15,9), Point(10,30), Point(10,1), Point(28,28)])
    s = PointSet([Point(40009746,116325990), Point(40009195,116326570), Point(40009154,116327023), 
                  Point(40008720,116325757), Point(40008616,116325533), Point(40008519,116326469), 
                  Point(40008455,116325360), Point(40008396,116325855), Point(40008393,116326263), 
                  Point(40008239,116325485), Point(40008224,116326805), Point(40008151,116327164), 
                  Point(40008131,116326011), Point(40008129,116326349), Point(40008102,116326605), 
                  Point(40008082,116325288), Point(40008077,116324930), Point(40008034,116325578), 
                  Point(40008006,116326991), Point(40008008,116327477), Point(40007854,116324331), 
                  Point(40007761,116324901)])
    # t = Triangle([Point(0,0), Point(10,30), Point(30,0), Point(20,10), Point(8,7), Point(15,9), Point(10,1),
    #               Point(5,10), Point(20,5), Point(21,4), Point(11,13), Point(13,3)])
    solution = s.findBalanceKeySolution()
    print solution
    print 
    i = 0
    for link in solution['links']:
        print i, ': ', link
        i += 1
    # t = Triangle([Point(0,0), Point(10,30), Point(30,0), Point(10,10), Point(5,3), Point(20,4)])
    # t = Triangle([Point(0,0), Point(10,30), Point(30,0), Point(10,10), Point(5,10)])
    # for triangle in t.divideIntoThreeTriangle(Point(1,1)):
    #     print triangle
    # print t.cover(Point(1,1))

    # l = [Link(Point(0,0), Point(0,1))]
    # print l.index(Link(Point(0,1), Point(0,0)))

    # result = t.findBalanceKeySolution(init_out={Point(0,0):4})
    # print result
    # print 
    # i = 0
    # for link in result['links']:
    #     print i, ': ', link
    #     i += 1