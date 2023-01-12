from typing import List
from collections import namedtuple
import time


class Point(namedtuple("Point", "x y")):
    # get a tuple named "Point"
    def __repr__(self) -> str:
        return f'Point{tuple(self)!r}'


class Rectangle(namedtuple("Rectangle", "lower upper")):
    # Let the two points in the input parameter be the diagonal of the rectangle to get the query rectangle
    def __repr__(self) -> str:
        return f'Rectangle{tuple(self)!r}'

    def is_contains(self, p: Point) -> bool:
        return self.lower.x <= p.x <= self.upper.x and self.lower.y <= p.y <= self.upper.y


class Node(namedtuple("Node", "location left right")):
    # get a tuple named "Node"
    """
    location: Point
    left: Node
    right: Node
    """

    def __repr__(self):
        return f'{tuple(self)!r}'


class KDTree:
    """k-d tree"""

    def __init__(self):
        self._root = None
        self._n = 0

    def insert(self, p: List[Point]):
        """insert a list of points"""
        dim=2
        # print("self._n:",self._n)
        depth=0
        def add(pts,depth):
            if len(pts) > 1:
                axis = depth% dim#Split the dimensions in turn
                pts.sort(key=lambda x: x[axis])
                m = int(len(pts) /2)#Split the left and right subtrees
                # print("m:",m)
                leftc=add(pts[:m],depth+1)
                rightc=add(pts[m + 1:],depth+1)
                self._n=self._n+1#increase the number of nodes in the tree
                
                return Node(pts[m], leftc, rightc)
            if len(pts) == 1:
                return Node(pts[0], None, None)

        self._root=add(p, 0)




    def range(self, rectangle: Rectangle) -> List[Point]:#Range search of KDTree
        """range query"""
        # Return a list of all points in the tree 'self' that lie within or
        # on the boundary of the given query rectangle, which is defined by
        # a pair of points (bottom_left, top_right), both of which are Vecs.

        # Given axis-aligned rectangle
        # Return (or count) all the points inside it

        # traverse the whole tree, BUT
        # • prune if bounding box doesn’t intersect with Query
        # • stop recursing or print all points in subtree if bounding box is entirely inside Query
        # print("rectangle:\n",rectangle)

        matches = []

        def travel(node, depth):
            # print("node:\n",node)
            x = node[0][0]
            y = node[0][1]
            xmin=rectangle[0][0]
            xmax=rectangle[1][0]
            ymin=rectangle[0][1]
            ymax=rectangle[1][1]
            # the node is in the range
            if xmin <= x and x <= xmax and ymin <= y and y <= ymax:
                # print("node:\n",node)
                matches.append(node[0])
            if depth % 2 == 0:#search as the x value
                if node[1] != None:  # left exist
                    if x >= xmin:
                        travel(node[1], depth + 1)
                if node[2] != None:  # right exist
                    if x <= xmax:
                        travel(node[2], depth + 1)
            else:#search as the y value
                if node[1] != None:  # left exist
                    if y >= ymin:
                        travel(node[1], depth + 1)
                if node[2] != None:  # right exist
                    if y <= ymax:
                        travel(node[2], depth + 1)

        travel(self._root, 0)

        return matches
    
    def distance(self, point1, point2):
        x1, y1 = point1
        x2, y2 = point2

        dx = x1 - x2
        dy = y1 - y2

        return dx * dx + dy * dy

    def _nearest_neighbor(self, point, node, closest, depth):
        if node is None:
            return closest
        if closest is None or self.distance(point, node.location) < self.distance(point, closest):
            closest = node.location
        axis = depth % 2
        if point[axis] < node.location[axis]:
            closest = self._nearest_neighbor(point, node.left, closest, depth+1)
        else:
            closest = self._nearest_neighbor(point, node.right, closest, depth+1)
        return closest

    def nearest_neighbor(self, point):
        closest = None
        depth = 0
        return self._nearest_neighbor(point, self._root, closest, depth)



def range_test():
    points = [Point(7, 2), Point(5, 4), Point(9, 6), Point(4, 7), Point(8, 1), Point(2, 3)]
    kd = KDTree()
    kd.insert(points)
    result = kd.range(Rectangle(Point(0, 0), Point(6, 6)))
    print("result:\n",result)
    result_nn =kd.nearest_neighbor(Point(0, 0))
    print("result_nn:\n",result_nn)
    assert sorted(result) == sorted([Point(2, 3), Point(5, 4)])


def performance_test():
    points = [Point(x, y) for x in range(1000) for y in range(1000)]

    lower = Point(500, 500)
    upper = Point(504, 504)
    rectangle = Rectangle(lower, upper)
    #  naive method
    start = int(round(time.time() * 1000))
    result1 = [p for p in points if rectangle.is_contains(p)]
    end = int(round(time.time() * 1000))
    print(f'Naive method: {end - start}ms')

    kd = KDTree()
    kd.insert(points)
    # k-d tree
    start = time.time() * 1000
    # print("time.time():\n",time.time())
    result2 = kd.range(rectangle)
    end = time.time() * 1000
    # print("time.time():\n",time.time())
    kdtime=end - start
    print('K-D tree: %.3fms' % (end - start))

    # print("result1:\n",sorted(result1))
    # print("result2:\n",sorted(result2))

    assert sorted(result1) == sorted(result2)


if __name__ == '__main__':
    range_test()
    performance_test()