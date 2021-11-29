import unittest
import sys
sys.path.append("/home/ric/Documents/Escuela/7mo_semestre/CompGeo/Tarea3/Modules/PyS4DCEL/S4DCELpp/cmake-build-debug")
import PyS4DCEL as dcel

class TestDCEL(unittest.TestCase):
    def test_something(self):
        V = [(-7,2),(-4,6),(2,4),(-3,2),(-1,0),(5,-1)]
        E = [(0,1),(1,2),(2,3),(4,3),(5,2),(4,5),(0,3)]
        test = dcel.dcel(V,E)
        print(test)
        self.assertIsNotNone(test)


if __name__ == '__main__':
    unittest.main()
