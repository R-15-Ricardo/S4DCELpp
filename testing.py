import unittest
import sys
sys.path.append("/home/ric/Documents/Escuela/7mo_semestre/CompGeo/Tarea3/Modules/PyS4DCEL/S4DCELpp/cmake-build-debug")
import PyS4DCEL as dcel
import numpy as np
import matplotlib.pyplot as plt

class TestDCEL(unittest.TestCase):
    # def test_dcel(self):
    #     V = [(-7,2),(-4,6),(2,4),(-3,2),(-1,0),(5,-1)]
    #     E = [(0,1),(1,2),(2,3),(4,3),(5,2),(4,5),(0,3)]
    #     testdcel = dcel.dcel(V,E)
    #     print(testdcel)
    #     self.assertIsNotNone(testdcel)
    #
    #     fid = testdcel.get_face(0)
    #     print(fid)
    #     self.assertIsNotNone(fid)
    #
    #     bound = testdcel.get_boundry(fid)
    #     print(bound)
    #     self.assertIsNotNone(bound)

        # points = [(-3.26,3.88),(1.58,1.19),(3.01,6.75)]
        # #points = [(1.58,1.19)]
        # for p in points:
        #     lfid = testdcel.landing_face(p)
        #
        #     lbound = testdcel.get_boundry(lfid)
        #     print(lbound)

    # def test_line(self):
    #     V = [(-7,2),(-4,6),(2,4),(-3,2),(-1,0),(5,-1)]
    #     E = [(0,1),(1,2),(2,3),(4,3),(5,2),(4,5),(0,3)]
    #     testdcel = dcel.dcel(V,E)
    #
    #     points = [(-3.26,3.88),(1.58,1.19),(3.01,6.75)]
    #     point = points[1]
    #
    #     testGraph = testdcel.G
    #
    #     bisector = dcel.get_bisector(point,testGraph[0][2])
    #
    #     fid = testdcel.landing_face(point)
    #     bound = testdcel.get_boundry(fid)
    #     intersections = []
    #     to_split = []
    #     for line in bound:
    #         auxInter = dcel.get_intersection(bisector, line)
    #         print(type(auxInter))
    #         if auxInter is not None:
    #             print("AAAAA")
    #             intersections.append(auxInter)
    #             to_split.append(line.on_bound_id)
    #             print(intersections)
    #
    #     intersections = np.array(intersections)
    #     print(intersections)
    #
    #     print("EEEEE")
    #
    #     to_join = []
    #     i = 0
    #     for p in intersections:
    #         to_join.append(testdcel.split_edge(tuple(p), to_split[i]))
    #         i+=1
    #
    #     testdcel.split_face(fid,to_join[0],to_join[1])
    #     store = testdcel.G
    #
    #     fid = testdcel.landing_face(point)
    #     print(fid)

    def test_newTest(self):
        V = [(-4,4),(4,4),(4,-4),(-4,-4),(0,0)]
        E = [(0,1),(1,2),(2,3),(3,0),(4,0),(4,1),(4,2),(4,3)]

        o1 = (-1,2)
        o2 = (2,1)
        o3 = (1,-2)
        o4 = (-2,-1)
        d1 = (1,0)
        d2 = (0,1)

        l1 = dcel.line(o1,d1,"full")
        l2 = dcel.line(o2,d2,"full")
        l3 = dcel.line(o3,d1,"full")
        l4 = dcel.line(o4,d2,"full")

        test2 = dcel.dcel(V,E)

        landings = [o1,o2,o3,o4]
        lines = [l1,l2,l3,l4]

        new_face = []

        for i in range(4):
            to_join = []
            interseccions = []

            fid = test2.landing_face(landings[i])
            bound = fid.boundry
            for edge in bound:
                inter = dcel.get_intersection(lines[i],edge.line)
                if inter is not None:
                    if len(to_join) == 0:
                        interseccions.append(tuple(inter))
                        to_join.append(test2.split_edge(tuple(inter),edge))
                    elif interseccions[-1] != tuple(inter):
                        interseccions.append(tuple(inter))
                        to_join.append(test2.split_edge(tuple(inter),edge))
            new_face.append(test2.split_face(fid,to_join[0],to_join[1]))

        test2.delete_interior(new_face,(-1,1.5))

        store = test2.G
        for arrow in store[1]:
            plt.arrow(arrow[0],arrow[1],arrow[2],arrow[3],head_width=0.35,length_includes_head=True,shape='left')
        plt.scatter(store[0].T[0],store[0].T[1])
        plt.show()





if __name__ == '__main__':
    unittest.main()
