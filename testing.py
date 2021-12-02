import unittest
import sys
sys.path.append("/home/ric/Documents/Escuela/7mo_semestre/CompGeo/Tarea3/Modules/PyS4DCEL/S4DCELpp/cmake-build-debug")
import PyS4DCEL as dcel
import numpy as np

class TestDCEL(unittest.TestCase):
    def test_dcel(self):
        V = [(-7,2),(-4,6),(2,4),(-3,2),(-1,0),(5,-1)]
        E = [(0,1),(1,2),(2,3),(4,3),(5,2),(4,5),(0,3)]
        testdcel = dcel.dcel(V,E)
        print(testdcel)
        self.assertIsNotNone(testdcel)

        fid = testdcel.get_face(0)
        print(fid)
        self.assertIsNotNone(fid)

        bound = testdcel.get_boundry(fid)
        print(bound)
        self.assertIsNotNone(bound)

        points = [(-3.26,3.88),(1.58,1.19),(3.01,6.75)]
        #points = [(1.58,1.19)]
        for p in points:
            lfid = testdcel.landing_face(p)

            lbound = testdcel.get_boundry(lfid)
            print(lbound)

    def test_line(self):
        V = [(-7,2),(-4,6),(2,4),(-3,2),(-1,0),(5,-1)]
        E = [(0,1),(1,2),(2,3),(4,3),(5,2),(4,5),(0,3)]
        testdcel = dcel.dcel(V,E)

        points = [(-3.26,3.88),(1.58,1.19),(3.01,6.75)]
        point = points[1]

        testGraph = testdcel.G

        bisector = dcel.get_bisector(point,testGraph[0][2])

        fid = testdcel.landing_face(point)
        bound = testdcel.get_boundry(fid)
        intersections = []
        to_split = []
        for line in bound:
            auxInter = dcel.get_intersection(bisector, line)
            print(type(auxInter))
            if auxInter is not None:
                print("AAAAA")
                intersections.append(auxInter)
                to_split.append(line.on_bound_id)
                print(intersections)

        intersections = np.array(intersections)
        print(intersections)

        print("EEEEE")

        split_point = testdcel.split_edge(tuple(intersections[0]), to_split[0])
        print(split_point)

        store = testdcel.G
        print(store)





if __name__ == '__main__':
    unittest.main()
