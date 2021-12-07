import unittest
import sys
import PyS4DCEL as dcel

def draw_voronoi(G: tuple()):
    import matplotlib.pyplot as plt
    for arrow in G[1]:
        plt.arrow(arrow[0],arrow[1],arrow[2],arrow[3],head_width=0.07,length_includes_head=True,shape='left')
    plt.scatter(G[0].T[0],G[0].T[1])
    plt.show()

class TestDCEL(unittest.TestCase):
    def test_fullAction(self):
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

        hint = test2.get_face(1)
        test2.delete_interior(new_face,hint)

        store = test2.G
        draw_voronoi(store)


if __name__ == '__main__':
    unittest.main()
